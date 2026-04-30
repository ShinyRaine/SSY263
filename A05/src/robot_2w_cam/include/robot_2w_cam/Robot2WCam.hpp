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

#ifndef ROBOT_2W_CAM_H
#define ROBOT_2W_CAM_H

/*! \file Robot2WCam.h
 *   \brief Provides a simulated Visual Perception module
 *
 *   Provides the following functionalities:
 *      - Get the list of target sequences (Path) from yaml file 
 *      - Get the target poses from a yaml file
 *      - A path Publisher to visualize the path in Rviz. This path is relative 
 *          the camera frame
 *      - A Service to request the list of target poses. This list of poses is 
 *         randomly selected
 *      - A service to request the camera extrinsic parameters (Homogeneous 
 *          transformation of the camera frame wrt Odom 
 *      - A TF broadcaster to visualize the different target positions wrt camera frame
 */


// Eigen Library
#include <Eigen/Dense>
#include <Eigen/StdVector>

// ros library
#include "rclcpp/rclcpp.hpp"

// tf publisher
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <robot_2w_interfaces/srv/get_target_poses_from_cam.hpp>
#include <robot_2w_interfaces/srv/get_tf_camera_odom.hpp>



namespace robot_2w_examples
{

class Robot2WCam: public rclcpp::Node
{

public:
  using VPoint = std::vector<geometry_msgs::msg::Point>; ///< Definition for vector of points
  using VVec3 = std::vector<Eigen::Vector3d>; ///< Definition for vector of vectors 3D
  using PubPath = rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr;     ///< Definition for Path Publisher
  using VVString = std::vector<std::vector<std::string>>; ///< define a vector of vectors to store the target sequences
  
  using SrvGetPoseFromCamShPt = rclcpp::Service<robot_2w_interfaces::srv::GetTargetPosesFromCam>::SharedPtr; ///< Service to send Poses from Cam
  using GetTargetPosesFromCamReqShPt = std::shared_ptr<robot_2w_interfaces::srv::GetTargetPosesFromCam::Request>; ///< Service Request Class
  using GetTargetPosesFromCamResShPt = std::shared_ptr<robot_2w_interfaces::srv::GetTargetPosesFromCam::Response>; ///< Service Response Class

  using SrvGetCamTransformShPt = rclcpp::Service<robot_2w_interfaces::srv::GetTFCameraOdom>::SharedPtr; ///< Service to send Poses from Cam
  using GetCamTransformReqShPt = std::shared_ptr<robot_2w_interfaces::srv::GetTFCameraOdom::Request>; ///< Service Request Class
  using GetCamTransformResShPt = std::shared_ptr<robot_2w_interfaces::srv::GetTFCameraOdom::Response>; ///< Service Response Class


private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; ///< TF broadcaster
  rclcpp::TimerBase::SharedPtr timer_;  ///< Timer to run the  main parallel process
  std::vector<std::string> vlabels_; ///< list of target labels
  std::vector<double> vpx_cam_;   ///< list of target x positions
  std::vector<double> vpy_cam_;   ///< list of target y positions
  std::vector<double> vpz_cam_;   ///< list of target z positions
  VPoint v_points_msg_cam_;   ///< Vector of point msgs
  VVec3 v_points_cam_;        ///< Vector of 3D points wrt camera frame
  VVString target_sequences_;  ///< Vector of target sequences
  std::vector<std::string> v_sequence_names_; ///< List of target sequence parameter names
  int n_seq_;   ///< number of provided target sequences

  bool publish_path_cam_; ///< flag to trigger the path publisher
  std::string frame_id_; ///< name of reference frame, in this case "camera"
  std::string path_topic_name_; ///< topic name to publish the path wrt camera frame
  std::string srv_name_get_poses_; ///< topic name to advertise the service get poses
  std::string srv_name_get_cam_tf_; ///< topic name to advertise the service get cam tf
  std::vector<std::string> random_path_; ///< Randomly selected path as target path

  tf2::Transform tf_cam_odom_; ///< TF camera frame wrt odom 
  std::vector<double> p_cam_odom_; ///< position Camera wrt Odom
  std::vector<double> o_cam_odom_; ///< orientation Camera wrt Odom (RPY)
  geometry_msgs::msg::TransformStamped tf_msg_cam_odom_; ///< TF camera frame wrt odom as msg
  bool cam_transform_ready_;

  PubPath pub_path_; ///< Path publisher
  SrvGetPoseFromCamShPt srv_get_poses_from_cam_; ///< Service to send the target path

  SrvGetCamTransformShPt srv_get_cam_odom_; ///< Service to send the camera frame wrt odom 
  

  std::map<std::string,Eigen::Vector3d> map_label_points_; ///< Map to connect the target labels with their corresponding 
                                                           ///< 3D points wrt camera frame

  std::map<std::string,geometry_msgs::msg::Point> map_label_points_msg_; ///< Map to connect the target labels with their 
                                                               ///< corresponding msg points wrt camera frame 

public:
  Robot2WCam(/* args */);
  ~Robot2WCam();
private:
  /**
   * @brief Declares and load the ros parameters. This function reads the yaml file
   *          and generates the list of target positions and the list of target 
   *          sequences. It also creates the maps between target labels and positions
   * 
   */
  void init();

  /**
   * @brief Loads the ros parameters. Calculates the Homogeneous transformation 
   *          of the camera wrt odom. Read the target sequences from yaml file. 
   * 
   * @return true all the parameters are loaded
   * @return false the number of target positions and labels is not the same (in the yaml)
   */
  bool load_parameters();

  /**
   * @brief Gets the 3D coordinates of the targets wrt camera frame and creates the map between labels and 3D points
   * 
   */
  void extract_points();

   /**
   * @brief Callback function for the main loop. Publishes the tfs for the target positions.
   *          Once there's a request for a target path (poses), the selected path is
   *          published. 
   *
   */
  void timer_callback();

  /**
   * @brief Callback function for the service to send the target poses wrt camera frame
   * 
   * @param request emtpy
   * @param response confirmation=true when succeeded to send the list of target poses
   */
  void getPosesFromCameraCB(const GetTargetPosesFromCamReqShPt request,
                                    GetTargetPosesFromCamResShPt response);

  /**
   * @brief Callback function to request the TF of the camera wrt odom
   * 
   * @param request empty
   * @param response confirmation=true when succeeded to send the transformation
   */
  void getCameraTransformCB(const GetCamTransformReqShPt request,
                                    GetCamTransformResShPt response);

};


}

#endif