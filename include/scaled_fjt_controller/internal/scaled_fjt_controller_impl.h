#pragma once // workaround qtcreator clang-tidy


#include <memory>
#include <thread>
#include <name_sorting/sort_trajectories.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <scaled_fjt_controller/scaled_fjt_controller.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
// #include <moveit/trajectory_processing/iterative_time_parameterization.h>

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

  CNR_INFO(this->logger(),"scaledfjt doinit");
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

  if (!this->getControllerNh().getParam("clik_gain",k_clik_))
  {
    k_clik_=0.0;
  }
  if (!this->getControllerNh().getParam("saturation_override",use_saturation_override_))
  {
    use_saturation_override_=false;
  }
  if (!this->getControllerNh().getParam("time_compensation",use_time_compensation_))
  {
    use_time_compensation_=false;
  }


  if (!this->getControllerNh().getParam("check_tolerance",m_check_tolerance))
  {
    CNR_DEBUG(this->logger(),"check_tolerance are not speficied for controllers. Using default (true)");
    m_check_tolerance=true;
  }

  CNR_TRACE(this->logger(),"subscribe override topics");

  for (const std::string& override_name: overrides)
  {
    auto cb=boost::bind(&cnr::control::ScaledFJTController<H,T>::overrideCallback,this,_1,override_name);
    this->template add_subscriber<std_msgs::Int64>(override_name,1,cb,false);
    m_overrides.insert(std::pair<std::string,double>(override_name,1.0));
    CNR_DEBUG(this->logger(),"subscribe override = " << override_name);
  }
  m_global_override=1.0;
  m_is_in_tolerance=true;


  CNR_TRACE(this->logger(),"subscribe blend topic");

  auto blend_cb=boost::bind(&cnr::control::ScaledFJTController<H,T>::blendTrajCallback,this,_1);
  this->template add_subscriber<trajectory_msgs::JointTrajectory>("blend_trajectory",1,blend_cb,false);

  CNR_TRACE(this->logger(),"create publisher");
  m_scaled_pub_id   = this->template add_publisher<std_msgs::Float64>("scaled_time",1);
  m_ratio_pub_id    = this->template add_publisher<std_msgs::Float64>("execution_ratio",1);
  m_unscaled_pub_id = this->template add_publisher<sensor_msgs::JointState>("unscaled_joint_target",1);
  m_traj_pub_id = this->template add_publisher<trajectory_msgs::JointTrajectory>("act_trajectory",1);

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
  this->publish(m_traj_pub_id,*trj);

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

  last_target_velocity_.resize(this->nAx());
  last_target_velocity_.setZero();
  last_target_position_.resize(this->nAx());
  last_target_position_.setZero();
  last_trajectory_target_velocity_.resize(this->nAx());
  last_trajectory_target_velocity_.setZero();

  m_as->start();
  CNR_RETURN_TRUE(this->logger());
}

template<class H, class T>
bool ScaledFJTController<H,T>::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  rosdyn::VectorXd last_saturated_target_velocity=this->getCommandVelocity();
  rosdyn::VectorXd last_saturated_target_position=this->getCommandPosition();

  rosdyn::VectorXd target_position(this->nAx());
  rosdyn::VectorXd target_velocity(this->nAx());
  rosdyn::VectorXd target_acceleration(this->nAx());
  rosdyn::VectorXd target_effort(this->nAx());

  double saturation_override=1.0;
  try
  {

    /*******************************************************************
     *                          TIME COMPENSATION                      *
     * if there was a saturation, bring back the scaled time           *
     * proportionally to the ratio between the error in the trajectory *
     * direction and the trajectory velocity                           *
     * v=dx/dt -> t = t - alpha*dx/v. alpha to avoid overcompensation  *
     *******************************************************************/
    if (use_time_compensation_)
    {
      double trj_error=(last_target_position_-last_saturated_target_position).dot(last_trajectory_target_velocity_.normalized());
      if (last_trajectory_target_velocity_.norm()>0)
      {
        std::lock_guard<std::mutex> lock(m_mtx);
        m_scaled_time-=ros::Duration(std::max(0.0,std::min(period.toSec(),0.5*trj_error/last_trajectory_target_velocity_.norm())));
        if (m_scaled_time.toSec()<0.0)
          m_scaled_time=ros::Duration(0.0);
      }
    }

    trajectory_msgs::JointTrajectoryPoint actual_point;
    if( !m_microinterpolator->interpolate(m_scaled_time,actual_point,1) )
    {
      CNR_ERROR_THROTTLE(this->logger(),0.5,"something wrong in interpolation.");
      CNR_RETURN_FALSE(this->logger());
    }
    for (size_t iAx=0;iAx<this->nAx();iAx++)
    {
      target_position(iAx)     = actual_point.positions.at(iAx);
    }
    /*******************************************************************
     *                          SATURATION OVERRIDE                    *
     * if the error is too high, slow down the trajectory to recover   *
     * use a log scale                                                 *
     * error = 1.0e-1 -> override = 0.0                                *
     * error = 1.0e-2 -> override = 0.33                               *
     * error = 1.0e-3 -> override = 0.66                               *
     * error = 1.0e-4 -> override = 1.00                               *
     *******************************************************************/
    if (use_saturation_override_)
    {
      double error=(target_position-last_saturated_target_position).norm();
      if (error>1.0e-4)
      {
        double log10=std::log10(error);  //>=-4
        saturation_override=std::max(0.0,1.0-(log10-(-4.0))/3.0);
      }
    }



    std::lock_guard<std::mutex> lock(m_mtx);
    if( !m_microinterpolator->interpolate(m_scaled_time,m_currenct_point,m_global_override*saturation_override) )
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


    /*******************************************************************
     *                              CLIK                               *
     * add  a velocity component proportional to the error             *
     * between the actual target position (using the compensated time) *
     * and the last saturated target position                          *
     ******************************************************************/
    rosdyn::VectorXd clik_velocity=k_clik_*(target_position-last_saturated_target_position);

    m_mtx.lock();
    m_scaled_time += period * m_global_override*saturation_override;
    m_time        += period;
    m_mtx.unlock();
    last_period = period;
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

    std::string p = "";
    std::string v = "";

    for (size_t iAx=0;iAx<this->nAx();iAx++)
    {
      target_position(iAx)     = m_currenct_point.positions.at(iAx);
      p+= std::to_string(target_position(iAx)) + ",";
      target_velocity(iAx)     = m_currenct_point.velocities.at(iAx);
      v+= std::to_string(target_velocity(iAx)) + ",";
      target_acceleration(iAx) = m_currenct_point.accelerations.at(iAx);
      target_effort(iAx)       = m_currenct_point.effort.at(iAx);
    }

    // ROS_INFO_STREAM("p:"<<p<<"-v:"<<v);
    last_trajectory_target_velocity_=target_velocity;
    target_velocity+=clik_velocity;
    this->setCommandPosition(target_position);
    this->setCommandVelocity(target_velocity);
    this->setCommandAcceleration(target_acceleration);
    this->setCommandEffort(target_effort);
    last_target_velocity_=target_velocity;
    last_target_position_=target_position;

    rosdyn::VectorXd actual_position = this->getPosition( );

    if (m_check_tolerance)
    {
      if (m_goal_tolerance.size()==1)
      {
        m_is_in_tolerance = (target_position-actual_position).cwiseAbs().maxCoeff()<m_goal_tolerance(0);
      }
      else
      {
        m_is_in_tolerance = (((target_position-actual_position).cwiseAbs()-m_goal_tolerance).array()<0.0).all();
      }
    }
    if (not m_is_in_tolerance)
      ROS_DEBUG_STREAM_THROTTLE(1,
                               "\n target_position  = " << (target_position).transpose() <<
                               "\n actual_position  = " << (actual_position).transpose() <<
                               "\n error            = " << (target_position-actual_position).transpose() <<
                               "\n tolerance        = " << m_goal_tolerance.transpose());


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

    // if (m_start_new_blend_traj) {
    //   CNR_INFO(this->logger(), "ending interpolation of current trajectory to start new blend trajectory");
    //   m_start_new_blend_traj = false;
    //   break;
    // }
    // std::string in_tol = "in tol";
    // if (!m_is_in_tolerance) in_tol = "not in tol";
    // CNR_INFO(this->logger(), "done?:"<<(m_scaled_time-m_microinterpolator->trjTime()).toSec()<<","<<in_tol);

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
void ScaledFJTController<H,T>::blendTrajCallback(const trajectory_msgs::JointTrajectoryConstPtr& trajectory)
{
  CNR_DEBUG(this->logger(), "Received a new blend trajectory");
  unsigned  int nPnt = trajectory->points.size();

  if (nPnt == 0)
  {
    CNR_ERROR(this->logger(),"BLEND TRAJECTORY WITH NO POINT");
    return;
  }
  std::lock_guard<std::mutex> lock(m_mtx);

  trajectory_msgs::JointTrajectoryPoint act_pt; 
  act_pt.time_from_start = ros::Duration(0.0);
  act_pt.positions   = std::vector<double>(this->getPosition().data(), this->getPosition().data() + this->nAx());
  act_pt.velocities  = std::vector<double>(this->getVelocity().data(), this->getVelocity().data() + this->nAx());
  act_pt.accelerations.resize(this->nAx(), 0);
  act_pt.effort.resize(this->nAx(), 0);

  try
  {
    trajectory_msgs::JointTrajectory tmp_traj;
    tmp_traj.joint_names = trajectory->joint_names;
    if(!m_microinterpolator->interpolate(m_scaled_time,m_currenct_point,m_global_override*1.0) )
    {
      CNR_ERROR(this->logger(), "Couldn't interpolate next point");
    }
    double tmp_ovr = std::max(m_global_override,0.01);
    for (int i=0;i<m_currenct_point.velocities.size();i++) {
      m_currenct_point.velocities[i] *= (1/tmp_ovr);
      m_currenct_point.accelerations[i] *= (1/(tmp_ovr*tmp_ovr));
    }
    // CNR_INFO(this->logger(),"first pt");
    // CNR_INFO(this->logger(),m_currenct_point);
    tmp_traj.points.push_back(m_currenct_point);
    ros::Duration original_time_from_start = trajectory->points.front().time_from_start;
    tmp_traj.points.push_back(trajectory->points.front());
    tmp_traj.points.front().time_from_start = ros::Duration(0.0);
    
    // blend_time_parameterize(tmp_traj);


    int dof = tmp_traj.points[0].positions.size();
    Eigen::ArrayXd q_start(dof);
    for (int q=0;q<dof;q++) q_start[q] = tmp_traj.points[0].positions[q];
    Eigen::ArrayXd q_end(dof);
    for (int q=0;q<dof;q++) q_end[q] = tmp_traj.points[1].positions[q];
    Eigen::ArrayXd q_dot_start(dof);
    for (int q=0;q<dof;q++) q_dot_start[q] = tmp_traj.points[0].velocities[q];
    Eigen::ArrayXd q_dot_end(dof);
    for (int q=0;q<dof;q++) q_dot_end[q] = tmp_traj.points[1].velocities[q];
    Eigen::ArrayXd diff = q_end-q_start;
    Eigen::ArrayXd vel_diff = q_dot_end-q_dot_start;
    Eigen::ArrayXd avg_vel = 0.5*(q_dot_end+q_dot_start);
    Eigen::ArrayXd avg_time = diff.abs()/avg_vel.abs();
    Eigen::ArrayXd max_accels_ = this->chain().getDDQMax().array();
    Eigen::ArrayXd vel_time = vel_diff.abs()/max_accels_;
    double max_time = std::min(std::max(0.7*tmp_traj.points[1].time_from_start.toSec(),vel_time.maxCoeff()),6.0);
    tmp_traj.points[1].time_from_start = ros::Duration(max_time);

    CNR_DEBUG(this->logger(),"pt 1 time from start:"<<tmp_traj.points[1].time_from_start.toSec());
    //append the rest of trajectory to tmp_traj


    //now loop and fix start times
    for (int i=1;i<trajectory->points.size();i++) {
      tmp_traj.points.push_back(trajectory->points[i]);
      tmp_traj.points.back().time_from_start = tmp_traj.points.back().time_from_start-original_time_from_start+tmp_traj.points[1].time_from_start;
    }
    

    trajectory_msgs::JointTrajectoryPtr trj(new trajectory_msgs::JointTrajectory());

    if (!trajectory_processing::sort_trajectory(this->jointNames(), tmp_traj, *trj))
    {
      CNR_ERROR(this->logger(), "Blend - Names are different");
      return;
    }

  
    CNR_DEBUG(this->logger(), "Starting managing new blend trajectory, trajectory has " << trj->points.size() << " points");
    m_microinterpolator->setTrajectory(trj);
    this->publish(m_traj_pub_id,*trj);
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
    // m_start_new_blend_traj = 1;
  }
  catch(std::exception& e)
  {
    CNR_ERROR(this->logger(), "Set Trajectory Failed");
  }

  // joinActionServerThread();

  // m_as_thread = std::thread(&ScaledFJTController<H,T>::actionServerThread, this);

  // m_mtx.lock();
  // m_scaled_time += last_period * m_global_override*1.0;
  // m_time        += last_period;
  trajectory_msgs::JointTrajectoryPoint test_pt = act_pt;
  // CNR_INFO(this->logger(),*m_microinterpolator->getTrajectory());
  // CNR_INFO(this->logger(),m_global_override);
  m_microinterpolator->interpolate(m_scaled_time,test_pt,m_global_override*1.0);

  // CNR_INFO(this->logger(),"act pt:");
  // CNR_INFO(this->logger(),act_pt);

  // CNR_INFO(this->logger(),"cur pt:");
  // CNR_INFO(this->logger(),m_currenct_point);
  // CNR_INFO(this->logger(),"nxt pt:");
  // CNR_INFO(this->logger(),test_pt);
  // m_mtx.unlock();

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
    this->publish(m_traj_pub_id,*trj);
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
      this->publish(m_traj_pub_id,*trj);
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


template<class H, class T>
void ScaledFJTController<H,T>::blend_time_parameterize(trajectory_msgs::JointTrajectory &plan) {
  // vel_profile.clear();
  Eigen::ArrayXd max_vels_ = this->chain().getDQMax().array();
  Eigen::ArrayXd max_accels_ = this->chain().getDDQMax().array();
  std::cout<<"max vels:"<<max_vels_<<std::endl;
  std::cout<<"max max_accels_:"<<max_accels_<<std::endl;
  double last_end_time = 0.0;
  int dof = plan.points[0].positions.size();
  for (int i=1;i<plan.points.size();i++) {
    // std::cout<<"starting: "<<i<<","<<plan.points.size()<<","<<plan.points[i-1].positions.size()<<","<<plan.points[i].positions.size()<<std::endl;
    // std::cout<<plan.points[i-1]<<std::endl;
    // std::cout<<plan.points[i]<<std::endl;
    Eigen::ArrayXd q_start(dof);
    for (int q=0;q<dof;q++) q_start[q] = plan.points[i-1].positions[q];
    Eigen::ArrayXd q_end(dof);
    for (int q=0;q<dof;q++) q_end[q] = plan.points[i].positions[q];
    Eigen::ArrayXd q_dot_start(dof);
    for (int q=0;q<dof;q++) q_dot_start[q] = plan.points[i-1].velocities[q];
    Eigen::ArrayXd q_dot_end(dof);
    for (int q=0;q<dof;q++) q_dot_end[q] = plan.points[i].velocities[q];
    Eigen::ArrayXd diff = q_end-q_start;
    Eigen::ArrayXd abs_diff = diff.abs();
    Eigen::ArrayXd sgn1 = Eigen::ArrayXd::Zero(abs_diff.size());
    for (int q=0;q<dof;q++) sgn1[q] = 1.0*(diff[q]>0.0)-1.0*(diff[q]<0.0)+0.0;
    // std::cout<<"sgn1"<<sgn1<<std::endl;
    Eigen::ArrayXd times_acc_to_full_speed = ((sgn1*max_vels_-q_dot_start)/max_accels_).abs();
    double time_acc_to_full_speed = times_acc_to_full_speed.maxCoeff();
    Eigen::ArrayXd times_to_decc_from_full_speed = ((sgn1*max_vels_-q_dot_end)/max_accels_).abs();
    double time_to_decc_from_full_speed = times_to_decc_from_full_speed.maxCoeff();
    Eigen::ArrayXd dq_to_full_speed = 0.5*sgn1*max_accels_*times_acc_to_full_speed*times_acc_to_full_speed+q_dot_start*times_acc_to_full_speed;
    Eigen::ArrayXd dq_full_to_next_speed = -0.5*sgn1*max_accels_*times_to_decc_from_full_speed*times_to_decc_from_full_speed+sgn1*max_vels_*times_to_decc_from_full_speed;
    Eigen::ArrayXd dq_tot = dq_to_full_speed+dq_full_to_next_speed;
    Eigen::ArrayXd full_spd_q = diff.abs()-dq_tot.abs();
    Eigen::ArrayXd times_at_full_spd = full_spd_q/max_vels_;
    Eigen::ArrayXd full_spd_tot_times = times_at_full_spd+times_acc_to_full_speed+times_to_decc_from_full_speed;
    double full_spd_tot_time = 0.0;
    int full_spd_jnt = 0;
    for (int q=0;q<dof;q++) {
      if (full_spd_tot_times[q]>full_spd_tot_time) {
        full_spd_tot_time = full_spd_tot_times[q];
        full_spd_jnt = q;
      }
    }
    std::cout<<"full spd jnt:"<<full_spd_jnt<<", full spd total time:"<<full_spd_tot_time<<","<<times_at_full_spd[full_spd_jnt]<<std::endl;
    ArrayXb may_go_too_fast = (full_spd_tot_time*q_dot_start).array().abs()>abs_diff.array();
    // for (int q=0;q<may_go_too_fast.size();q++) {
    //   if (may_go_too_fast[q]) sgn1[q]=-sgn1[q];
    // }
    Eigen::ArrayXd accel_stop_time = Eigen::ArrayXd::Zero(6);
    Eigen::ArrayXd deccel_start_time = Eigen::ArrayXd::Zero(6);
    Eigen::ArrayXd accelerations = Eigen::ArrayXd::Zero(6);
    Eigen::ArrayXd deccelerations = Eigen::ArrayXd::Zero(6);
    Eigen::ArrayXd t1_ = Eigen::ArrayXd::Zero(6);
    Eigen::ArrayXd t2_ = Eigen::ArrayXd::Zero(6);
    Eigen::ArrayXd alt_a = Eigen::ArrayXd::Zero(6);

    ArrayXb spd_limit_jnts = ArrayXb::Constant(6,true);
    if (times_at_full_spd[full_spd_jnt]>0) {
      spd_limit_jnts[full_spd_jnt] = false;
      accelerations[full_spd_jnt] = max_accels_[full_spd_jnt];
      deccelerations[full_spd_jnt] = max_accels_[full_spd_jnt];
      accel_stop_time[full_spd_jnt] = times_acc_to_full_speed[full_spd_jnt];
      deccel_start_time[full_spd_jnt] = times_acc_to_full_speed[full_spd_jnt]+times_at_full_spd[full_spd_jnt];
      double mid_t = times_acc_to_full_speed[full_spd_jnt]+0.5*times_at_full_spd[full_spd_jnt];
      Eigen::ArrayXd c = (sgn1*max_vels_-q_dot_start)/(sgn1*max_vels_-q_dot_end);
      Eigen::ArrayXd c_round = 0.01*(100*c).round();
      t2_ = (diff-full_spd_tot_time*sgn1*max_vels_)/(0.5*(c*c-1)*(q_dot_end-q_dot_start)/(c-1)+c*(q_dot_start-sgn1*max_vels_));
      Eigen::ArrayXd alt_t2 = (diff-full_spd_tot_time*sgn1*max_vels_)/(q_dot_start-sgn1*max_vels_);
      for (int q=0;q<dof;q++) {
        if (c_round[q]==1.0) {
          t2_[q] = alt_t2[q];
        }
      }
      t1_ = c*t2_;
      alt_a = ((q_dot_end-q_dot_start)/(c-1)/t2_).abs();
      Eigen::ArrayXd alt_alt_a = ((sgn1*max_vels_-q_dot_start)/c/t2_).abs();
      for (int q=0;q<dof;q++) {
        if (c_round[q]==1.0) {
          alt_a[q] = alt_alt_a[q];
        }
      }
    }
    // std::cout<<"full spd time:"<<full_spd_tot_time<<std::endl;
    Eigen::ArrayXd qm_dot = max_vels_;
    Eigen::ArrayXd tm;
    Eigen::ArrayXd t_fm;
    Eigen::ArrayXd sgn = sgn1;
    Eigen::ArrayXd avg_spd_times = abs_diff/(0.5*(q_dot_end+q_dot_start).abs());
    bool dq_neg = false;
    while (true) {
      sgn = sgn1;
      tm = (sgn*qm_dot-q_dot_start).abs()/max_accels_;
      t_fm = (sgn*qm_dot-q_dot_end).abs()/max_accels_;
      for (int q=0;q<dof;q++) {
        if ((spd_limit_jnts[q]) && ((tm[q]+t_fm[q])>avg_spd_times[q])) {
          sgn[q] = -sgn1[q];
        }
      }
      tm = (sgn*qm_dot-q_dot_start).abs()/max_accels_;
      t_fm = (sgn*qm_dot-q_dot_end).abs()/max_accels_;
      Eigen::ArrayXd diff2 = 0.5*sgn*max_accels_*(tm*tm-t_fm*t_fm)+sgn*qm_dot*t_fm+q_dot_start*tm;
      std::cout<<"diff 2:"<<diff2.transpose()<<std::endl;
      ArrayXb diff_bool = ((abs_diff-diff2.abs())>=0);
      std::cout<<"diff_bool:"<<diff_bool.transpose()<<std::endl;
      std::cout<<"qm_dot:"<<qm_dot.transpose()<<std::endl;
      if (diff_bool.all()) break;
      for (int q=0;q<qm_dot.size();q++) {
        if (!diff_bool[q]) qm_dot[q]-=0.01;
        if (qm_dot[q]<0) dq_neg = true;
      }
      if (dq_neg) break;
    }
    std::cout<<"final qm dot:"<<qm_dot.transpose()<<std::endl;
    Eigen::ArrayXd tot_time = tm + t_fm;
    double max_tot_time = 0.0;
    int limit_joint = 0;
    for (int q=0;q<dof;q++) {
      if (tot_time[q]>max_tot_time) {
        max_tot_time = tot_time[q];
        limit_joint = q;
      }
    }
    double mid_time = tm[limit_joint];
    double end_time = tot_time[limit_joint];
    if (times_at_full_spd[full_spd_jnt]>0) {
      end_time = full_spd_tot_time;
      mid_time = 0.5*end_time;
    }
    std::cout<<"mid_time"<<mid_time<<std::endl;
    std::cout<<"end_time"<<end_time<<std::endl;
    Eigen::MatrixXd knowns = Eigen::MatrixXd::Zero(2,diff.size());
    knowns.row(0) = (diff-end_time*q_dot_start).matrix();
    knowns.row(1) = (q_dot_end-q_dot_start).matrix();
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2,2);
    A(0,0) = 0.5*mid_time*mid_time+mid_time*(end_time-mid_time);
    A(0,1) = -0.5*(end_time-mid_time)*(end_time-mid_time);
    A(1,0) = mid_time;
    A(1,1) = -(end_time-mid_time);
    Eigen::MatrixXd a_d = A.inverse()*knowns;
    std::cout<<"knowns"<<knowns<<std::endl;
    std::cout<<"A"<<A<<std::endl;
    std::cout<<"a_d"<<a_d<<std::endl;
    for (int q=0;q<dof;q++) {
      if (spd_limit_jnts[q]) {
        accelerations[q] = abs(a_d(0,q));
        deccelerations[q] = abs(a_d(1,q));
        accel_stop_time[q] = mid_time;
        deccel_start_time[q] = mid_time;
      }
    }
    Eigen::ArrayXd mid_spds = q_dot_start+mid_time*a_d.row(0).transpose().array();
    ArrayXb tmp_comparitor = (mid_spds.abs()>max_vels_);
    for (int q=0;q<dof;q++) {
      if (spd_limit_jnts[q] && tmp_comparitor[q]) {
          accel_stop_time[q] = t1_[q];
          deccel_start_time[q] = full_spd_tot_time-t2_[q];
          accelerations[q] = alt_a[q];
          deccelerations[q] = alt_a[q];
          // std::cout<<q<<",a:"<<accelerations[q]<<std::endl;
          // std::cout<<q<<",d:"<<deccelerations[q]<<std::endl;
      }
    }
    if (times_at_full_spd[full_spd_jnt]>0) {
      mid_spds[full_spd_jnt] = sgn[full_spd_jnt]*max_vels_[full_spd_jnt];
    }
    std::cout<<"end time:"<<end_time<<std::endl;
    std::cout<<sgn<<std::endl;
    std::cout<<accelerations<<std::endl;
    std::cout<<accel_stop_time<<std::endl;
    std::cout<<deccelerations<<std::endl;
    std::cout<<deccel_start_time<<std::endl;
    plan.points[i].time_from_start = ros::Duration(end_time + last_end_time);
    // vel_profile.emplace_back(sgn*accelerations,-1.0*sgn*deccelerations,accel_stop_time+last_end_time,deccel_start_time+last_end_time);
    last_end_time += end_time;
  }
}

}  // namespace control
}  // namespace cnr
