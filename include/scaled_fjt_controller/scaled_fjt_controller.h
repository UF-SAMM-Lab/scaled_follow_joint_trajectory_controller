/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy (CNR)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef SCALED_FJT_CONTROLLER__H
#define SCALED_FJT_CONTROLLER__H

#include <mutex>
#include <memory>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <pluginlib/class_loader.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>
#include <scaled_fjt_controller/microinterpolator.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <trajectory_msgs/JointTrajectory.h>


typedef Eigen::Array<bool,Eigen::Dynamic,1> ArrayXb;
namespace cnr
{
namespace control
{

template<class H, class T>
class ScaledFJTController : public cnr::control::JointCommandController<H,T>
{
public:
  ScaledFJTController();
  ~ScaledFJTController();

  virtual bool doInit( );
  virtual bool doUpdate(const ros::Time& time, const ros::Duration& period);
  virtual bool doStarting(const ros::Time& time);
  virtual bool doStopping(const ros::Time& time);

protected:
  bool joinActionServerThread();
  void actionServerThread();
  void actionGoalCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle gh);
  void actionCancelCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle gh);
  void overrideCallback(const std_msgs::Int64ConstPtr& msg, const std::string& override_name);
  void blendTrajCallback(const trajectory_msgs::JointTrajectoryConstPtr& trajectory);
  void blend_time_parameterize(trajectory_msgs::JointTrajectory &plan);

  size_t m_scaled_pub_id;
  size_t m_ratio_pub_id;
  size_t m_unscaled_pub_id;

  std::map<std::string,double> m_overrides;
  std::vector<ros::Subscriber> m_override_topic;
  double m_global_override;
  rosdyn::VectorXd last_target_velocity_;
  rosdyn::VectorXd last_trajectory_target_velocity_; // without clik contribution
  rosdyn::VectorXd last_target_position_;
  double k_clik_;
  bool use_time_compensation_;
  bool use_saturation_override_;

  int m_is_finished;
  std::mutex m_mtx;

  boost::shared_ptr<actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>> m_as;
  boost::shared_ptr<actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle> m_gh;
  std::thread m_as_thread;

  bool m_preempted;
  bool m_is_in_tolerance;
  bool m_check_tolerance=true;
  ros::Duration m_scaled_time;
  ros::Duration m_time;
  ros::Duration last_period;
  trajectory_msgs::JointTrajectoryPoint m_currenct_point;
  std::shared_ptr<cnr::control::Microinterpolator> m_microinterpolator;
  rosdyn::VectorXd m_goal_tolerance;  //it may be a double or a eigen::vector


};
}  // namespace control

} // namespace cnr


#include <scaled_fjt_controller/internal/scaled_fjt_controller_impl.h>


#endif  // SCALED_FJT_CONTROLLER__H
