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


#include <robot_2w_cam/Robot2WCam.hpp>

#include <random>

// Place holders for the service definitions
using std::placeholders::_1;
using std::placeholders::_2;

namespace robot_2w_examples
{

Robot2WCam::Robot2WCam(/* args */) : Node("robot_2w_cam"), publish_path_cam_(false), cam_transform_ready_(false)
{
  // Init Parameters and load node parameters.
  init();

  // Initialize the transform broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Path publisher wrt camera frame
  pub_path_ = create_publisher<nav_msgs::msg::Path>(path_topic_name_, 10);

  // Service to send Target poses with respect to Camera frame
  srv_get_poses_from_cam_ = create_service<robot_2w_interfaces::srv::GetTargetPosesFromCam>(
      srv_name_get_poses_, std::bind(&Robot2WCam::getPosesFromCameraCB, this, _1, _2));

  // Service to send the camera tf wrt odom
  srv_get_cam_odom_ = create_service<robot_2w_interfaces::srv::GetTFCameraOdom>(
      srv_name_get_cam_tf_, std::bind(&Robot2WCam::getCameraTransformCB, this, _1, _2));

  // main thread.
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Robot2WCam::timer_callback, this));
}

Robot2WCam::~Robot2WCam()
{
}

void Robot2WCam::init()
{
  // ROS2 parameter declaration
  declare_parameter("frame_id", "/cam");
  declare_parameter("path_topic_name", "/p_cam");
  declare_parameter("service_get_path_cam", "/srv_cam");
  declare_parameter("service_get_cam_tf", "/srv_cam_odom");
  declare_parameter("positions.x", std::vector<double>{ 0.0, 0.0, 0.0 });
  declare_parameter("positions.y", std::vector<double>{ 0.0, 0.0, 0.0 });
  declare_parameter("positions.z", std::vector<double>{ 0.0, 0.0, 0.0 });
  declare_parameter("labels", std::vector<std::string>{ "a", "b", "c" });

  declare_parameter("h_cam_odom.position", std::vector<double>{ 0.0, 0.0, 0.0 });
  declare_parameter("h_cam_odom.orientation", std::vector<double>{ 0.0, 0.0, 0.0 });

  // Number of predefined sequences
  declare_parameter("sequences.number", 0);

  // The target sequences (path) are defined as seq_X in the yaml file.
  // We need to dynamically create the labels "seq_x" to get each of them.
  n_seq_ = get_parameter("sequences.number").as_int();

  for (int i = 0; i < n_seq_; i++)
  {
    std::stringstream s;
    s << "sequences.seq_" << i + 1;
    v_sequence_names_.push_back(s.str());
    s.str("");
  }

  // Declare all the sequence parameters
  for (auto&& names : v_sequence_names_)
  {
    declare_parameter(names, std::vector<std::string>{ "a", "b" });
  }

  // Get the ros parameters (for example, from a yaml file or via the console )
  if (!load_parameters())
    exit(-1);

  // Extract the 3D points of each target and generate 3D vectors. It also populates the
  // map between target labels and 3D points
  extract_points();
}

bool Robot2WCam::load_parameters()
{
  // copy the ros parameters to the class member variables
  frame_id_ = get_parameter("frame_id").as_string();
  path_topic_name_ = get_parameter("path_topic_name").as_string();
  srv_name_get_poses_ = get_parameter("service_get_path_cam").as_string();
  srv_name_get_cam_tf_ = get_parameter("service_get_cam_tf").as_string();
  vlabels_ = get_parameter("labels").as_string_array();
  vpx_cam_ = get_parameter("positions.x").as_double_array();
  vpy_cam_ = get_parameter("positions.y").as_double_array();
  vpz_cam_ = get_parameter("positions.z").as_double_array();

  // These auxilary position and orientation are needed to compute the camera frame
  // transformation
  std::vector<double> p_cam_odom_ = get_parameter("h_cam_odom.position").as_double_array();
  std::vector<double> o_cam_odom_ = get_parameter("h_cam_odom.orientation").as_double_array();

  // TODO_1: Define the transformation of the camera frame  wrt odom
  tf_cam_odom_.setOrigin(tf2::Vector3(0,0,0)); //You need to replace (0,0,0) with the correct values
  tf2::Quaternion qtf;
  //TODO_2: Convert the roll, pitch, and yaw orientation into a quaternion representation
  qtf.setRPY(0,0,0); //You need to replace (0,0,0) with the correct values
  tf_cam_odom_.setRotation(qtf);

  // Create a msg version of the camera transformation
  tf_msg_cam_odom_.transform = tf2::toMsg(tf_cam_odom_);

  // Set the reference frame for the TF (parent and child link)
  tf_msg_cam_odom_.header.frame_id = "/odom";
  // The /camera_validation should be located exactly at the same pose as the 
  // /camera frame (This is just for validation)
  tf_msg_cam_odom_.child_frame_id = "/camera_validation";

  // The tf camera wrt odom has been populated, we can use the service to send
  // this transformation
  cam_transform_ready_ = true;

  // Get the list of target sequences (paths)
  for (auto&& names : v_sequence_names_)
  {
    std::vector<std::string> v_aux;

    v_aux = get_parameter(names).as_string_array();

    target_sequences_.push_back(v_aux);
  }

  RCLCPP_WARN_STREAM(get_logger(), "Target Sequences");
  for (auto&& seq : target_sequences_)
  {
    std::stringstream s;

    for (auto&& label : seq)
    {
      s << label << ",";
    }

    RCLCPP_WARN_STREAM(get_logger(), s.str());
    s.str("");
  }

  // std::stringstream s;

  // for (auto&& i : vlabels_)
  // {
  //   s << i << ", ";
  // }

  // RCLCPP_INFO_STREAM(get_logger(), "labels[" << vlabels_.size() << "]: " << s.str());

  // s.str("");

  // for (auto&& i : vpx_cam_)
  // {
  //   s << i << ", ";
  // }

  // RCLCPP_INFO_STREAM(get_logger(), "x [" << vpx_cam_.size() << "]: " << s.str());
  // s.str("");

  // for (auto&& i : vpy_cam_)
  // {
  //   s << i << ", ";
  // }

  // RCLCPP_INFO_STREAM(get_logger(), "y [" << vpy_cam_.size() << "]: " << s.str());
  // s.str("");

  // for (auto&& i : vpz_cam_)
  // {
  //   s << i << ", ";
  // }

  // RCLCPP_INFO_STREAM(get_logger(), "z [" << vpz_cam_.size() << "]: " << s.str());
  // s.str("");

  // We verify that all the coordinates and the labels have the same number of items
  if ((vpx_cam_.size() != vpy_cam_.size()) || (vpx_cam_.size() != vpz_cam_.size()) ||
      (vpz_cam_.size() != vpy_cam_.size()) || (vpx_cam_.size() != vlabels_.size()))
  {
    RCLCPP_ERROR_STREAM(get_logger(), "The number of positions and labels are not the same!");

    return false;
  }

  return true;
}

void Robot2WCam::extract_points()
{
  // Get the number of poses. Each pose has its unique label defined in the yaml file
  std::size_t n = vlabels_.size();

  // The target poses are defined as a list of x, y, and z coordinates.
  // We extract the axes triplet for each pose. Then, we represent each of these
  // poses as Vector3d, Point (geometry_msgs). Finally, we create maps between the
  // target labels and its corresponding poses (Vector3D and msg)
  for (size_t i = 0; i < n; i++)
  {
    Eigen::Vector3d aux(vpx_cam_.at(i), vpy_cam_.at(i), vpz_cam_.at(i));

    geometry_msgs::msg::Point aux_point;
    aux_point.x = vpx_cam_.at(i);
    aux_point.y = vpy_cam_.at(i);
    aux_point.z = vpz_cam_.at(i);

    v_points_cam_.push_back(aux);
    v_points_msg_cam_.push_back(aux_point);

    map_label_points_[vlabels_.at(i)] = v_points_cam_.at(i);
    map_label_points_msg_[vlabels_.at(i)] = v_points_msg_cam_.at(i);
  }

  RCLCPP_INFO_STREAM(get_logger(), "Target coordinates wrt camera frame");
  for (auto&& label : vlabels_)
  {
    RCLCPP_INFO_STREAM(get_logger(), label << ": " << map_label_points_[label].transpose());
  }
}

void Robot2WCam::timer_callback()
{
  // We will create a vector of transformations to publish them all at once
  // This makes the visualization more efficient 
  std::vector<geometry_msgs::msg::TransformStamped> v_ts;
  
  // Auxiliary transformation to populate v_ts
  geometry_msgs::msg::TransformStamped ts;
  
  // Each tf needs a name, we created this name dynamically using the target label
  std::stringstream s;

  // Get the current time to stamp the messages
  rclcpp::Time aux_time = now();

  // TF object to populate our TF message
  tf2::Transform tf;

  // Auxiliary quaternion object to define the orientation of the TF message
  tf2::Quaternion qtf;

  for (auto&& label : vlabels_)
  {
    // RCLCPP_INFO_STREAM(get_logger(), label << ": " << map_label_points_msg_[label].x << ", "
    //                                        << map_label_points_msg_[label].y << ", " <<
    //                                        map_label_points_msg_[label].z);

    // Get pose from msg
    // We define a quaternion using a basic rotation in z (Yaw)
    qtf.setRPY(0, 0, 0);

    // TODO_3: Set the position of the TF.
    // We use the target labels to retrieve the coordinated of the target positions
    // For example, this function says: "give me the position in x, y, and z of 
    // the target with label 'a'!""
    tf.setOrigin(
        tf2::Vector3(0,0,0)); //Modify (0,0,0) with the correct values

    // TODO_ 4 :Set the orientation of the TF
    //tf.setRotation(qtf);

    // Transform the TF object to TF message to publish it 
    ts.transform = tf2::toMsg(tf);

    // Set the reference frame for the TF (parent link). In this case is "camera"
    ts.header.frame_id = frame_id_;
    // Set the time stamp for the message
    ts.header.stamp = aux_time;
    // Set the name for the TF using the target label
    s << "target_" << label;
    ts.child_frame_id = s.str();
    s.str("");

    // Populate the vector of tfs
    v_ts.push_back(ts);
  }

  // We also publish the camera_validation tf to check if we have the correct 
  // transformations (Homogeneous Transformation)
  // TF Camera frame wrt Odom
  // Set the time stamp for the message
  tf_msg_cam_odom_.header.stamp = aux_time;
  // add the transformation to the vector of transformations
  v_ts.push_back(tf_msg_cam_odom_);

  // Publish the selected Path
  // When the service "get_pose_from_camera" is requested by a client, we 
  // randomly select one sequence of poses (path) and send it to the client.
  // To verify the path, we also publish it to rviz. NOTE: this path is wrt camera
  if ((publish_path_cam_) && (!random_path_.empty()))
  {
    // Aux variables to populate the Path msg
    nav_msgs::msg::Path path;
    geometry_msgs::msg::PoseStamped pose;

    path.header.stamp = aux_time;
    // In this case, frame_id is the camera frame
    path.header.frame_id = frame_id_;

    // The random_path is selected by the service. We use its positions to populate
    // the Path msg
    for (auto&& label : random_path_)
    {
      pose.header.frame_id = frame_id_;
      pose.header.stamp = aux_time;
      pose.pose.position.x = map_label_points_msg_[label].x;
      pose.pose.position.y = map_label_points_msg_[label].y;
      pose.pose.position.z = map_label_points_msg_[label].z;

      path.poses.push_back(pose);
    }

    // Then, we publish the Path msg. This path can be visualized in Rviz
    pub_path_->publish(path);
  }

  // Send the vector of transformations
  tf_broadcaster_->sendTransform(v_ts);
}

void Robot2WCam::getPosesFromCameraCB(const GetTargetPosesFromCamReqShPt request, GetTargetPosesFromCamResShPt response)
{
  // Verify that the list of target sequences (paths) is already created
  // If the list is not yet available, we trigger an error and report it to the 
  // client
  if (target_sequences_.empty())
  {
    std::stringstream s;

    s << "The list of available target paths is empty";

    RCLCPP_ERROR_STREAM(get_logger(), s.str());
    response->confirmation = false;
    response->message = s.str();

    publish_path_cam_ = false;

    // dummy if to prevent warning "unused variable"
    if (request)
    {
    }  
  }
  else
  {
    // Initialize random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, target_sequences_.size() - 1);

    // Generate a random index to select a target sequence (Path)
    int random_index = dis(gen);

    // Select a path from the list using the random index
    random_path_ = target_sequences_.at(random_index);

    std::stringstream s;

    // We create the poses msg with the randomly selected path
    s << "Path(" << random_index + 1 << ")[";

    // response->poses is a list of poses, defined by the selected sequence (path)
    for (auto&& target_label : random_path_)
    {
      s << target_label << ",";

      geometry_msgs::msg::Pose aux_pose;

      aux_pose.position.x = map_label_points_msg_[target_label].x;
      aux_pose.position.y = map_label_points_msg_[target_label].y;
      aux_pose.position.z = map_label_points_msg_[target_label].z;

      response->poses.push_back(aux_pose);
    }

    s << "]";

    RCLCPP_INFO_STREAM(get_logger(), s.str());

    // Confirm request was successful
    response->confirmation = true;
    // We also send the sequence of target labels that define the path as a msg
    response->message = s.str();

    // Once a path has been confirmed, we publish it to visualize it in Rviz
    publish_path_cam_ = true;
  }
}

void Robot2WCam::getCameraTransformCB(const GetCamTransformReqShPt request, GetCamTransformResShPt response)
{
  // If the tf of the camera wrt odom has been populated, we serve the request
  if(!cam_transform_ready_)
  {
    std::stringstream s;
    s<<"The camera tf is not ready yet";
    response->confirmation=false;
    response->message=s.str();

    if (request){}  // dummy if to prevent warning "unused variable"
  }
  else
  {
    response->confirmation=true;
    // Send the tf camera wrt odom
    // This transformation is useful to map coordinates defined in the camera frame
    // to coordinates in the odom frame (in our case, the odom frame is the world frame)
    response->tf_cam_odom = tf_msg_cam_odom_;
  }
}

}  // namespace robot_2w_examples