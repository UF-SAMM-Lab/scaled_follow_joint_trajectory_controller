#pragma once // workaround qtcreator clang-tidy


#include <memory>
#include <thread>
#include <name_sorting/sort_trajectories.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <scaled_fjt_controller/scaled_fjt_controller.h>

namespace eu = eigen_utils;

namespace cnr
{
namespace control
{

template<class H, class T>
ScaledFJTController<H,T>::~ScaledFJTController()
{
  CNR_DEBUG(this->logger(), "Destroying Thor Prefilter Controller");
  joinActionServerThread();
  m_as.reset();
  CNR_DEBUG(this->logger(), "Destroyed Thor Prefilter Controller");
}

template<class H, class T>
ScaledFJTController<H,T>::ScaledFJTController()
{
  m_is_in_tolerance = false;
  m_preempted = false;
}

template<class H, class T>
bool ScaledFJTController<H,T>::doInit()
{
  CNR_TRACE_START(this->logger());

  std::string what;
  // ^^^^^^
  m_goal_tolerance.resize(this->getPosition().size());
  eu::setConstant(m_goal_tolerance, 0.001);
  if(!rosparam_utilities::getParam(this->getControllerNh(), "goal_tolerance", m_goal_tolerance, what, &m_goal_tolerance))
  {
    CNR_ERROR(this->logger(), "Error in getting the goal tolerance: " << what);
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_WARN_COND(this->logger(), what.size()>0, what);

  rosdyn::VectorXd path_tolerance;  //it may be a double or a eigen::vector
  eu::setConstant(path_tolerance, 0.001);
  if(!rosparam_utilities::getParam(this->getControllerNh(), "path_tolerance", path_tolerance, what, &path_tolerance))
  {
    CNR_ERROR(this->logger(), "Error in getting the path tolerance: " << what);
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_WARN_COND(this->logger(), what.size()>0, what);

  std::vector<std::string> overrides;
  if (!this->getControllerNh().getParam("overrides",overrides))
  {
    CNR_DEBUG(this->logger(),"overrides are not speficied for controllers. Using default");
    overrides.push_back("/speed_ovr");
    overrides.push_back("/safe_ovr_1");
    overrides.push_back("/safe_ovr_2");
  }

  if (!this->getControllerNh().getParam("check_tolerance",m_check_tolerance))
  {
    CNR_DEBUG(this->logger(),"check_tolerance are not speficied for controllers. Using default (true)");
    m_check_tolerance=true;
  }

  CNR_TRACE(this->logger(),"subscribe override topics");

  ros::NodeHandle controller_nh("foo");
  for (const std::string& override_name: overrides)
  {
    auto cb=boost::bind(&cnr::control::ScaledFJTController<H,T>::overrideCallback,this,_1,override_name);
    this->template add_subscriber<std_msgs::Int64>(override_name,1,cb,false);
    m_overrides.insert(std::pair<std::string,double>(override_name,1.0));
    CNR_DEBUG(this->logger(),"subscribe override = " << override_name);
  }
  m_global_override=1.0;
  m_is_in_tolerance=true;

  CNR_TRACE(this->logger(),"create publisher");
  m_scaled_pub_id   = this->template add_publisher<std_msgs::Float64>("scaled_time",1);
  m_ratio_pub_id    = this->template add_publisher<std_msgs::Float64>("execution_ratio",1);
  m_unscaled_pub_id = this->template add_publisher<sensor_msgs::JointState>("unscaled_joint_target",1);

  CNR_TRACE(this->logger(),"create action server");
  try
  {
    m_as.reset(new actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>(
                 this->getControllerNh(),
                 "follow_joint_trajectory",
                 boost::bind(&cnr::control::ScaledFJTController<H,T>::actionGoalCallback, this, _1),
                 boost::bind(&cnr::control::ScaledFJTController<H,T>::actionCancelCallback, this, _1),
                 false));

    CNR_TRACE(this->logger(),"create action server DONE");
  }
  catch (std::exception& e)
  {
    CNR_ERROR(this->logger(),"unable to create action server. exception: " << e.what());
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_RETURN_TRUE(this->logger());
}

template<class H, class T>
bool ScaledFJTController<H,T>::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger());

  m_currenct_point.time_from_start = ros::Duration(0.0);
  m_currenct_point.positions   = std::vector<double>(this->getPosition().data(), this->getPosition().data() + this->nAx());
  m_currenct_point.velocities  = std::vector<double>(this->getVelocity().data(), this->getVelocity().data() + this->nAx());
  m_currenct_point.accelerations.resize(this->nAx(), 0);
  m_currenct_point.effort.resize(this->nAx(), 0);

  CNR_TRACE(this->logger(),"starting point = \n"<<m_currenct_point);

  trajectory_msgs::JointTrajectoryPtr trj(new trajectory_msgs::JointTrajectory());
  trj->points.push_back(m_currenct_point);

  m_scaled_time=ros::Duration(0);;
  m_time=ros::Duration(0);

  m_microinterpolator.reset(new cnr::control::Microinterpolator());
  m_microinterpolator->setTrajectory(trj);

  int spline_order=1;
  if (!this->getControllerNh().getParam("continuity_order",spline_order))
  {
    CNR_DEBUG(this->logger(),"continuity_order is not set, set equal to 1");
    spline_order=1;
  }
  if (spline_order<0)
    spline_order=0;

  m_microinterpolator->setSplineOrder(spline_order);
  CNR_TRACE(this->logger(),"Starting interpolator with continuity order = \n" << spline_order);

  m_as->start();
  CNR_RETURN_TRUE(this->logger());
}

template<class H, class T>
bool ScaledFJTController<H,T>::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  try
  {
    std::lock_guard<std::mutex> lock(m_mtx);
    if( !m_microinterpolator->interpolate(m_scaled_time,m_currenct_point,m_global_override) )
    {
      CNR_ERROR_THROTTLE(this->logger(),0.5,"something wrong in interpolation.");
      CNR_ERROR_THROTTLE(this->logger(),0.5, "scaled time     = "  << m_scaled_time);
      CNR_ERROR_THROTTLE(this->logger(),0.5, "global override = "  << m_global_override);
      CNR_ERROR_THROTTLE(this->logger(),0.5, "current point   = "  << m_currenct_point);
      CNR_ERROR_THROTTLE(this->logger(),0.5, "current point   = "  << m_currenct_point);
      CNR_ERROR_THROTTLE(this->logger(),0.5, "trajectory      = "  << *m_microinterpolator->getTrajectory());
      CNR_RETURN_FALSE(this->logger());
    }
  }
  catch (std::exception& e)
  {
    CNR_ERROR(this->logger(), "Got and exception: '" << e.what()
              << "'(function input: Time: " << time.toSec() << " Duration: " << period.toSec() << ")");
    CNR_RETURN_FALSE(this->logger());
  }

  try
  {
    m_scaled_time += period * m_global_override;
    m_time        += period;
    std_msgs::Float64Ptr scaled_msg(new std_msgs::Float64());
    scaled_msg->data=m_scaled_time.toSec();
    this->publish(m_scaled_pub_id,scaled_msg);

    std_msgs::Float64Ptr ratio_msg(new std_msgs::Float64());
    if (m_microinterpolator->trjTime().toSec()>0)
      ratio_msg->data=std::min(1.0,m_scaled_time.toSec()/m_microinterpolator->trjTime().toSec());
    else
      ratio_msg->data=1;
    this->publish(m_ratio_pub_id,ratio_msg);


    sensor_msgs::JointStatePtr unscaled_js_msg(new sensor_msgs::JointState());
    trajectory_msgs::JointTrajectoryPoint unscaled_pnt;
    if( !m_microinterpolator->interpolate(m_scaled_time,unscaled_pnt,1) )
    {
      CNR_ERROR_THROTTLE(this->logger(),0.5,"something wrong in interpolation.");
      CNR_RETURN_FALSE(this->logger());
    }

    unscaled_js_msg->name = this->jointNames();
    unscaled_js_msg->position.resize(this->nAx());
    unscaled_js_msg->velocity.resize(this->nAx());
    unscaled_js_msg->effort.resize(this->nAx(),0);
    unscaled_js_msg->position      = unscaled_pnt.positions;
    unscaled_js_msg->velocity      = unscaled_pnt.velocities;
    unscaled_js_msg->header.stamp  = ros::Time::now();
    this->publish(m_unscaled_pub_id,unscaled_js_msg);

    m_is_in_tolerance=true;
    for (size_t iAx=0;iAx<this->nAx();iAx++)
    {
      this->setCommandPosition     (m_currenct_point.positions.at(iAx),iAx);
      this->setCommandVelocity     (m_currenct_point.velocities.at(iAx),iAx);
      this->setCommandAcceleration (m_currenct_point.accelerations.at(iAx),iAx);
      this->setCommandEffort       (m_currenct_point.effort.at(iAx),iAx);
    }
    rosdyn::VectorXd actual_position = this->getPosition( );
    rosdyn::VectorXd actual_target_position = this->getCommandPosition( );

    if (m_check_tolerance)
    {
      if (m_goal_tolerance.size()==1)
      {
        m_is_in_tolerance = (actual_target_position-actual_position).cwiseAbs().maxCoeff()<m_goal_tolerance(0);
      }
      else
      {
        m_is_in_tolerance = (((actual_target_position-actual_position).cwiseAbs()-m_goal_tolerance).array()<0.0).all();
      }
    }
    if (m_is_in_tolerance)
      ROS_INFO_STREAM_THROTTLE(1,
                               "\n error = " << (actual_target_position-actual_position).transpose() <<
                               "\n tolerance = " << m_goal_tolerance.transpose());
    else
      ROS_WARN_STREAM_THROTTLE(1,
                               "\n actual_target_position = " << (actual_target_position).transpose() <<
                               "\n actual_position = " << (actual_position).transpose() <<
                               "\n error = " << (actual_target_position-actual_position).transpose() <<
                               "\n tolerance = " << m_goal_tolerance.transpose());


  }
  catch (std::exception& e)
  {
    CNR_ERROR(this->logger(), "Got and exception: '" << e.what()
              << "'(function input: Time: " << time.toSec() << " Duration: " << period.toSec() << ")");
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}

template<class H, class T>
bool ScaledFJTController<H,T>::doStopping(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger());
  if (m_gh)
  {
    m_gh->setCanceled();
  }
  joinActionServerThread();
  m_gh.reset();
  CNR_RETURN_TRUE(this->logger());
}

template<class H, class T>
void ScaledFJTController<H,T>::overrideCallback(const std_msgs::Int64ConstPtr& msg, const std::string& override_name)
{
  double ovr;
  if (msg->data>100)
    ovr=1.0;
  else if (msg->data<0)
    ovr=0.0;
  else
    ovr=msg->data*0.01;
  m_overrides.at(override_name)=ovr;
  double global_override=1;
  for (const std::pair<std::string,double>& p: m_overrides)
    global_override*=p.second;
  m_global_override=global_override;
}


template<class H, class T>
bool ScaledFJTController<H,T>::joinActionServerThread()
{
  m_preempted = true;
  if (m_as_thread.joinable())
  {
    m_as_thread.join();
  }
  m_preempted = false;
  return true;
}

template<class H, class T>
void ScaledFJTController<H,T>::actionServerThread()
{
  std::stringstream report;
  CNR_DEBUG(this->logger(), "START ACTION GOAL LOOPING");
  ros::WallRate lp(100);
  while (ros::ok())
  {
    lp.sleep();
    if (!m_gh)
    {
      CNR_ERROR(this->logger(), "Goal handle is not initialized");
      break;
    }

    if ((m_preempted)
        || (m_gh->getGoalStatus().status == actionlib_msgs::GoalStatus::RECALLED)
        || (m_gh->getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTED))
    {
      CNR_WARN(this->logger(), "Action Server Thread Preempted");
      CNR_DEBUG(this->logger(), "(m_gh->getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTED)  = "
                << (int)(m_gh->getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTED));
      CNR_DEBUG(this->logger(), "m_preempted  = %d" << (int)m_preempted);
      CNR_DEBUG(this->logger(), "(m_gh->getGoalStatus().status == actionlib_msgs::GoalStatus::RECALLED)  = "
                << (int)(m_gh->getGoalStatus().status == actionlib_msgs::GoalStatus::RECALLED));

      this->addDiagnosticsMessage("OK", (m_preempted ? "preempted" : "cancelled"), {{"Interpolator", "Cancelled"}}, &report);
      CNR_INFO(this->logger(), report.str());

      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = 0;
      result.error_string = "preempted";
      m_gh->setRejected(result);
      CNR_DEBUG(this->logger(), "Preempted old goal DONE");

      break;
    }

    if ((m_is_finished==1) || (((m_scaled_time-m_microinterpolator->trjTime()).toSec()>0) && m_is_in_tolerance))
    {
      this->addDiagnosticsMessage("OK", "£", {{"INTERPOLATOR", "Goal tolerance achieved!"}} , &report);
      CNR_INFO(this->logger(), report.str());

      control_msgs::FollowJointTrajectoryResult result;
      m_gh->setSucceeded(result);

      break;
    }
    else if (m_is_finished == -2)
    {
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = -4;
      result.error_string = "Some problem occurs";

      this->addDiagnosticsMessage("ERROR", "£", {{"INTERPOLATOR", "Some problem occurs"}}, &report);
      CNR_ERROR(this->logger(), report.str());

      m_gh->setAborted(result);
      break;
    }
  }
  m_gh.reset();
  CNR_DEBUG(this->logger(), "START ACTION GOAL END");
}

template<class H, class T>
void ScaledFJTController<H,T>::actionGoalCallback(
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle gh)
{
  CNR_TRACE_START(this->logger());
  CNR_INFO(this->logger(), "Received a goal");
  auto goal = gh.getGoal();

  boost::shared_ptr<actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle> current_gh;

  if (m_gh)
  {
    // PREEMPTED OLD GOAL
    CNR_INFO(this->logger(), "preempting old goal");
    m_gh->setAborted();
    joinActionServerThread();

    CNR_INFO(this->logger(), "Goal Stopped");
  }
  else
  {
    CNR_DEBUG(this->logger(), "No goals running yet");
  }

  current_gh.reset(new actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle(gh));
  m_gh = current_gh;
  unsigned  int nPnt = goal->trajectory.points.size();

  if (nPnt == 0)
  {
    CNR_DEBUG(this->logger(),"TRAJECTORY WITH NO POINT");
    control_msgs::FollowJointTrajectoryResult result;
    m_gh->setAccepted();
    current_gh->setSucceeded(result);
    m_gh.reset();
    return;
  }

  trajectory_msgs::JointTrajectoryPtr trj(new trajectory_msgs::JointTrajectory());

  if (!trajectory_processing::sort_trajectory(this->jointNames(), goal->trajectory, *trj))
  {
    CNR_ERROR(this->logger(), "Names are different");
    m_gh->setAborted();
    joinActionServerThread();
    return;
  }

  try
  {
    std::lock_guard<std::mutex> lock(m_mtx);
    CNR_DEBUG(this->logger(), "Starting managing new goal, trajectory has " << trj->points.size() << " points");
    m_microinterpolator->setTrajectory(trj);
    m_scaled_time=ros::Duration(0);
    m_time=ros::Duration(0);
    m_is_finished=0;

    std::stringstream ss1;
    ss1 << "[ ";
    for(const auto & q : trj->points.front().positions)
    {
      ss1 << std::to_string(q) << " "; ss1 << "]";
    }
    std::stringstream ss2;
    ss2 << "[ ";
    for(const auto & qd : trj->points.front().velocities)
    {
      ss2 << std::to_string(qd) << " ";  ss2 << "]";
    }
    CNR_DEBUG(this->logger(), "First Point of the trajectory:\n q : " + ss1.str() + "\n" + " qd:" + ss2.str());

  }
  catch(std::exception& e)
  {
    CNR_ERROR(this->logger(), "Set Trajectory Failed");
  }

  m_gh->setAccepted();

  joinActionServerThread();

  m_as_thread = std::thread(&ScaledFJTController<H,T>::actionServerThread, this);

  CNR_RETURN_OK(this->logger(), void());
}

template<class H, class T>
void ScaledFJTController<H,T>::actionCancelCallback(
    actionlib::ActionServer< control_msgs::FollowJointTrajectoryAction >::GoalHandle gh)
{
  CNR_TRACE_START(this->logger());
  CNR_DEBUG(this->logger(), "Cancel active goal Callback");
  if (m_gh)
  {
    m_gh->setCanceled();
    joinActionServerThread();
    m_gh.reset();

    try
    {
      std::lock_guard<std::mutex> lock(m_mtx);

      trajectory_msgs::JointTrajectoryPoint pnt=m_currenct_point;
      std::fill(pnt.velocities.begin(),pnt.velocities.end(),0.0);
      std::fill(pnt.accelerations.begin(),pnt.accelerations.end(),0.0);
      std::fill(pnt.effort.begin(),pnt.effort.end(),0.0);
      pnt.time_from_start=ros::Duration(0);

      trajectory_msgs::JointTrajectoryPtr trj(new trajectory_msgs::JointTrajectory());
      trj->points.push_back(pnt);

      m_microinterpolator->setTrajectory(trj);
      CNR_TRACE(this->logger(),"cancel trajectory. Stay in \n" << trj);
      m_scaled_time=ros::Duration(0);
      m_time=ros::Duration(0);
    }
    catch(std::exception& e)
    {
      CNR_ERROR(this->logger(), "Set Trajectory Failed. Exception:" << std::string(e.what()));
    }
  }
  else
  {
    CNR_WARN(this->logger(), "No goal to cancel");
  }
  CNR_RETURN_OK(this->logger(), void());
}

}  // namespace control
}  // namespace cnr

