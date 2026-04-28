/*! \file
 *
 * \author Emmanuel Dean
 *
 * \version 0.1
 * \date 15.03.2021
 *
 * \copyright Copyright 2021 Chalmers
 *
 * #### License
 * All rights reserved.
 *
 * Software License Agreement (BSD License 2.0)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef ROBOT_2W_CTRL_H
#define ROBOT_2W_CTRL_H

/*! \file Robot2WCtrl.h
 *   \brief Controls the robot_2w to move from the current state to the desired state
 *
 *   Provides the following functionalities:
 *     - Subscriber to a Robot2WStateStamped topic (desired state)
 *     - Publisher of a Robot2WStateStamped topic (for the robot_2w visualizer)
 *     - TF frames
 */

// Eigen Library
#include <Eigen/Dense>
#include <Eigen/StdVector>

// ROS2 messages
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"


// ros library
#include "rclcpp/rclcpp.hpp"

// Parameter Callback
#include "rcl_interfaces/msg/set_parameters_result.hpp"



// tf publisher
#include <tf2_ros/transform_broadcaster.h>

namespace robot_2w_examples
{
class Robot2WCtrl : public rclcpp::Node
{
public:

  using SubRobot2WOdometry = rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr;  ///< Definition for Robot2W odometry subscriber
  using Robot2WOdomShPt = nav_msgs::msg::Odometry::SharedPtr; ///< Definition for Odom shared ptr

  using SubTrajectoryPoint = rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr;  ///< Definition for Robot2W odometry subscriber
  using TrajectoryPointShPt = geometry_msgs::msg::Point::SharedPtr; ///< Definition for Odom shared ptr

  using PubRobot2WTwist = rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr;     ///< Definition for Robot2W twist publisher

  using PubPosePoint = rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr;     ///< Definition for Robot2W pose error publisher

private:

  Eigen::Matrix3d K_;                ///< Control gain matrix
  bool first_message_;              ///< Flag to control when we have the robot_2w state
  bool init_ctrl_;                  ///< Flag to set the robot_2w initial position using the Robot2WState
  
  std::string odom_topic_name_;      ///< topic name for the odom pose subscriber 
  std::string trajectory_topic_name_;      ///< topic name for the trajectory subscriber
  std::string commanded_twist_topic_name_; ///< topic name for the commanded twist (to command the diff drive)
  int64_t ctrl_period_;                  ///< Control period in ms
  
  OnSetParametersCallbackHandle::SharedPtr callback_handle_; // Param callback handle
  
  SubRobot2WOdometry sub_odometry_; ///< Subscriber to receive the robot odometry
  Robot2WOdomShPt robot_2w_odom_msg_; ///< Shared object to share the robot_2w odometry msg between the control loop and the subscriber
  std::mutex odom_data_mutex_;           ///< Mutex to protect the reading/writing process
  bool got_odom_;

  SubTrajectoryPoint sub_trajectory_; ///< Subscriber to receive the trajectory point
  TrajectoryPointShPt trajectory_point_msg_; ///< Shared object to share the trajectory msg between the control loop and the subscriber
  std::mutex trajectory_data_mutex_;     ///< Mutex to protect the reading/writing process
  bool got_traj_point_;

  PubRobot2WTwist robot_twist_pub_; ///< Publisher for the commanded twist
  
  PubPosePoint pose_error_pub_; ///< Publisher for the pose error (Point)
  bool publish_pose_error_; ///< Flag to control when the errors will be published
  std::string pose_error_topic_name_; ///< topic name where the pose error will be published 

  PubPosePoint pose_pub_; ///< Publisher for the robot pose (x,y,theta)
  std::string pose_topic_name_; ///< topic name where the robot's pose will be published 

  rclcpp::TimerBase::SharedPtr timer_;  ///< Timer to run a parallel process

public:
  /**
   * @brief Default constructor
   *
   */
  Robot2WCtrl(/* args */);

  /**
   * @brief Destroy the Robot2W Ctrl object
   *
   */
  ~Robot2WCtrl();

  /**
   * @brief Control initialization, sets the control and node parameters
   *
   */
  void init();


  rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);

  bool load_parameters();

  void odom_topic_callback(const Robot2WOdomShPt msg);

  void trajectory_topic_callback(const TrajectoryPointShPt msg); 

  /**
   * @brief Callback function for the control loop. Internal thread for the controller.
   *
   */
  void timer_callback();
};

}  // namespace robot_2w_examples

#endif