# \file
#
#  \author Emmanuel Dean
#
#  \version 0.1
#  \date 15.03.2021
#
#  \copyright Copyright 2021 Chalmers
#
#  #### License
# All rights reserved.

# Software License Agreement (BSD License 2.0)

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:

#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# /


# ROS module
from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import Parameter

# ROS geometry message module
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import Header  # Import Header message
from std_msgs.msg import String

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import SetParametersResult

from tf2_ros import TransformBroadcaster

# ROS custom messages and services
from robot_2w_interfaces.srv import SetDesiredPoses

# This class provides methods to generate a smooth trajectory from the current turtle pose to a desired turtle pose
# This smooth trajectory will generate the commanded turtle state for the controller
class PathGenerator(Node):
    def __init__(self):
        super().__init__("path_generator")

        params = ["srv_name", "pub_topic_name", "period", "odom_topic_name","odom_frame_name","odom_forward_frame_name", "target_frame_name", "trajectory_topic_name"]
        
        # Declare the ros2 parameters 
        self.declare_parameters(
            namespace="",
            parameters=[
                ("srv_name", "set_desired_poses_aux"),
                ("pub_topic_name", "/path_1"),
                ("period", 0.1),
                ("odom_topic_name", "/odom"),
                ("odom_frame_name", "/odom"),
                ("odom_forward_frame_name", "/odom_forward"),
                ("target_frame_name", "/target_1"),
                ("trajectory_topic_name", "/trajectory_1"),
                
            ],
        )

        # Load Parameters and save them into member variables
        self.load_parameters()
        
        # Print all the loaded parameters. 
        for param in params:
            param_type=self.get_parameter_type(param)
            if param_type == ParameterType.PARAMETER_DOUBLE:
                val= self.get_parameter(param).get_parameter_value().double_value
                
            elif param_type == ParameterType.PARAMETER_STRING:
                val= self.get_parameter(param).get_parameter_value().string_value
            
            self.get_logger().info(f"Default {param}: {val}")    
        # end for
        
        # Register a callback function that will be called whenever there is an attempt to
        # change one or more parameters of the node.
        # For example, using in the terminal: ros2 param set ....
        self.add_on_set_parameters_callback(self.parameter_change_callback)

        # Service to set the desired robot poses wrt the odom frame
        self.srv = self.create_service(
            SetDesiredPoses, self.srv_name, self.set_desired_poses_callback
        )

        # Subscriber to receive the robot pose (Odometry)
        # We need the odometry to get the initial robot pose.
        # This initial pose will be added at the beginning and end of the list
        # of target poses to generate a cyclic path

        self.odom_subs = self.create_subscription(
            Odometry, self.subs_topic_name, self.listener_callback, 10
        )
        self.odom_subs  # prevent unused variable warning

        # Publisher to publish the Path relative to the odom frame
        self.publisher_ = self.create_publisher(Path, self.pub_topic_name, 10)
        
        # Trajectory Publisher. We will publish the desired robot pose wrt odom frame
        # The desired poses cover all the target poses defined in the path
        # This desired robot pose is the reference for the controller to calculate
        # the commanded robot velocity (cmd_vel)
        self.traj_pub = self.create_publisher(Point, self.traj_pub_name, 10)


        # Flag to control when to start publishing data. This should be done once we have received the first request  and the first odometry message
        self.got_request = False
        self.got_odometry = False

        # Main thread. Computes the smooth trajectory and publish the result as a path
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Initialize the transform broadcaster to visualize the target pose
        self.tf_broadcaster = TransformBroadcaster(self)

        # Flag to reset the internal clock for the trajectory generator
        self.reset_vars = True
        # elapsed_time is the time used in the calculation of the desired robot pose
        # Each time a request is received, we reset the elapsed_time clock 
        self.elapsed_time = 0.0

        # Robot Initial Pose. We will read the odom message and get the robot pose
        self.robot_pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(w=1.0)
        )

        # Path msg
        self.path_msg = Path()
        self.poses = []
        # We initialize the list of times with zero since all the trajectories 
        # will start in t=0.0
        self.times = [0.0]
        self.number_poses = 0
        self.current_pose_id = 0
        
        # Aux variables to compute the elapsed_time
        self.ti = Time()
        self.tc = Time()
        
        # Node Clock
        self.clock = self.get_clock()
    
    def load_parameters(self):
        """ Loads the node parameters and saves them into member variables
        """
        # subscriber topic name
        self.srv_name = (
            self.get_parameter("srv_name").get_parameter_value().string_value
        )
        # Publisher topic name
        self.pub_topic_name = (
            self.get_parameter("pub_topic_name").get_parameter_value().string_value
        )
        # Sample period for the main thread
        self.timer_period = (
            self.get_parameter("period").get_parameter_value().double_value
        )
        # Subscriber topic name
        self.subs_topic_name = (
            self.get_parameter("odom_topic_name").get_parameter_value().string_value
        )
        
        # Odom TF frame name
        self.odom_frame_name = (
            self.get_parameter("odom_frame_name").get_parameter_value().string_value
        )
        
        # Odom Forward TF frame name
        self.odom_forward_frame_name = (
            self.get_parameter("odom_forward_frame_name").get_parameter_value().string_value
        )
        
        # Target TF frame name
        self.target_frame_name = (
            self.get_parameter("target_frame_name").get_parameter_value().string_value
        )
        
        # Trajectory Publisher topic name
        self.traj_pub_name = (
            self.get_parameter("trajectory_topic_name").get_parameter_value().string_value
        ) 
        self.get_logger().warn("New parameters loaded")       
    
    def parameter_change_callback(self, params):
        """Gets called whenever there is an attempt to change one or more parameters.
 
        Args:
            params (List[Parameter]): A list of Parameter objects representing the parameters that are being attempted to change.
         
        Returns:
            SetParametersResult: Object indicating whether the change was successful.
        """
        result = SetParametersResult()

        result.successful = True
        
        for param in params:
            if param.type_ == Parameter.Type.DOUBLE:
                val= self.get_parameter(param.name).get_parameter_value().double_value
                
            elif param.type_ == Parameter.Type.STRING:
                val= self.get_parameter(param.name).get_parameter_value().string_value
            
            else:
                result.successful = False
                
            if result.successful:
                self.get_logger().warn(f"New parameter[{param.name}]: {val}")
            else:
                self.get_logger().error(f"Parameter {param.name} type {param.type_} not listed")
        
        if result.successful:
            self.load_parameters()
 
        return result

    def listener_callback(self, msg):
        """ Callback function to receive the robot pose wrt odom

        Args:
            msg (nav_msgs/msg/Odometry): odom info: the robot's differential drive estimates
                          the robot's position using odometry. This pose is published 
                          in the topic /odom   
        """
        # self.get_logger().info(" Got Odom msg")

        # To verify the odom msg, we echo it as a TF to visualize it in rviz
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.clock.now().to_msg()
        t.header.frame_id = self.odom_frame_name
        t.child_frame_id = self.odom_forward_frame_name

        # Save the current pose in the shared variable
        self.robot_pose = msg.pose.pose

        t.transform.translation.x = self.robot_pose.position.x
        t.transform.translation.y = self.robot_pose.position.y
        t.transform.translation.z = self.robot_pose.position.z
        t.transform.rotation.x = self.robot_pose.orientation.x
        t.transform.rotation.y = self.robot_pose.orientation.y
        t.transform.rotation.z = self.robot_pose.orientation.z
        t.transform.rotation.w = self.robot_pose.orientation.w

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
        
        # We set the flag when we have the odometry to start receiving requests
        self.got_odometry = True
        

    
    def set_desired_poses_callback(self, request, response):
        """ Callback function for the service. The client specifies the desired robot poses and the times when these poses must be reached

        Args:
            request: list of poses and reaching times 
            response: true if succeed and a human readable message
        Returns:
            The response flag and messages 
            """
        
        # The number of poses must be equal to the number of reaching times
        if  len(request.poses) != len(request.times):
            # Response for the client
            response.confirmation = False
            msg = String()
            msg.data=f"The number of poses ({len(request.poses)}) is different from the number of times ({len(request.times)})"
            
            self.get_logger().error(msg.data)
            response.msg = msg
            
            return response  
        
        # We process the request if we have already received the odom info (start position)
        # and if there's no other request
        if self.got_odometry and not self.got_request :
            
            # We append the current robot pose as the first and last poses in the target poses
            # list. This is to start the trajectory generator of the robot in that position and
            # end the path in the initial robot's position. This creates cyclic paths
            self.get_logger().info(
                    "Begin robot_ini: %f, %f, %f"
                    % (
                        self.robot_pose.position.x,
                        self.robot_pose.position.y,
                        self.robot_pose.position.z,
                    )
                )
            # Target poses start in the initial robot's pose
            self.poses = [self.robot_pose]
            
            # We also append the initial time t0=0.0
            self.times = [0.0]

            # TODO_5: Append the target poses into the shared variables
            self.poses += [0.0,0.0,0.0] #replace [0.0,0.0,0.0] with the correct value
            
            # TODO_6: Append the time variable into the shared variables
            self.times += [0.0] #replace [0.0] with the correct value
            
            # TODO_7: Currently, the list of poses include the initial robot position as
            # the first pose, the target poses sent by the path client. To generate a cyclic
            # path, you need to include the initial robot position as the final pose in the list
                # Hint: you need to modify the variable self.poses 
            
            # Since we added a new pose to the list, we also need to add a new reaching time.
            # TODO_8: add a final time to the list of reaching times. The final time should be 5 seconds
            # after the last requested time
            # We define the time for the final position as the last commanded time + 5 seconds
                # Hint: you need to modify the variable self.times 
            
            print(self.times)
            
            # TODO_9: Define the number of poses. Since the trajectory generator uses a 
            # pair of poses to calculate the spline, we use (number_poses -1) iterations 
            self.number_poses = 0 #Replace "0" with the correct value

            self.get_logger().info(f"poses received:{self.number_poses}")
            # print out the requested data
            for pose in self.poses:
                self.get_logger().info(
                    f"Target pose: {pose.position.x}, {pose.position.y}, {pose.position.z}"
                )
            # self.get_logger().info(f"poses :{self.poses}")
            self.get_logger().info(f"times received:{self.times}")
            
            # We change the flag to indicate that we are processing a requested
            # path. Therefore, if there's another request, it will be rejected until
            # we finish the current request.
            self.got_request = True
            
            # Response for the client
            response.confirmation = True
            response.msg = String()
        else:
            # We haven't received the odometry or we have already a request. 
            # We cannot receive requests
            msg = String()
            
            if self.got_request:
                msg.data = "We are processing a previous request"
            else:
                msg.data = "We are waiting for the odometry topic before receiving requests"
            
            self.get_logger().warn(msg.data)
            
            response.confirmation = False
            response.msg = msg

        return response

    

    def timer_callback(self):
        """  Main thread callback function. This function will publish: 
             a) The Path wrt to odom frame, 
             b) The target position (calculated with a Spline) wrt odom frame, 
             c) a TF to visualize the target position wrt odom frame
        """

        # Only when the service has been triggered, we start to generate a new trajectory
        if self.got_request:
            
            # We initialize the trajectory generator parameters, such as 
            # initial time, elapsed time (0.0 by default)
            if self.reset_vars:
                # We stop updating the poses[0] with the current robot_pose (odom msg)
                self.reset_vars = False
                # Set the initial clock 
                self.ti = self.clock.now()
                
                # self.get_logger().info(f'ti: {self.ti.nanoseconds/1e9}')
                
                self.elapsed_time = 0.0
                
                # Only the first time we set the start pose as the first element
                # in the target poses list
                self.current_pose_id = 0
            
            tc = self.clock.now()
            # self.get_logger().info(f"tc: {tc.nanoseconds/1e9}")
            

            ## Publishing the Path wrt odom
            waypoints = []

            for pose in self.poses:
                waypoints.append(
                    PoseStamped(header=Header(stamp=tc.to_msg()), pose=pose)
                )

            #TODO_10: Compute the path wrt to the odom frame
            """
            HINT: You need to get the correct values to the following variables

            self.path_msg.header.frame_id =   
            self.path_msg.header.stamp = 
            self.path_msg.poses = 
            
            """
            
            self.publisher_.publish(self.path_msg)
            
            # self.get_logger().info("StampedPath published!")

            # Trajectory generator loops through all the elements in the target pose 
            # and times lists. The poses always start in the initial robot pose 
            # at the time t=0.0, and ends in the same initial robot pose.
            position_msg=Point()
            
            # We will loop trough all the poses in the list. When we call the last pose
            # we reset all the flags to receive more poses (clients)
            
            # self.get_logger().info(f"{self.current_pose_id}<{self.number_poses}")
            
            # Each time we finished one segment (a pair of target poses), we increase 
            # the target pose counter
            if (self.current_pose_id<self.number_poses):
                # Get the time range for the current path segment (pair of target poses)
                # e.g. [t0 t1], [t1 t2], etc.
                time_range=self.times[self.current_pose_id:self.current_pose_id+2]
                # print(time_range)
                
                # Calculate the elapsed time
                self.elapsed_time = (tc - self.ti).nanoseconds/(1e9)
                
                # self.get_logger().warn(f"elapsed: {self.elapsed_time} pose[{self.current_pose_id}] r:{time_range}")
                # self.get_logger().error(f"poses[{len(self.poses)}]: {self.poses}")
                # self.get_logger().error(f"times: {self.times}")
                
                # The path segment is generated with a spline that joins two consecutive 
                # target poses, i.e., the start pose with the goal pose. 
                # Each segment changes this pair, e.g.,
                # first iteration: start = robot_init, goal=p1; 
                # second iteration: start=p1, goal=p2;
                # second last iteration: start=pn-1, goal=pn;
                # last iteration: start=pn, goal=robot_init;
                
                # The start pose is based on the poses list
                start = self.poses[self.current_pose_id].position
                
                # self.get_logger().warn(f"Start({self.current_pose_id}):[{start.x}, {start.y}, {start.z}]")
                
                # The goal pose is the next element in the poses list
                goal = self.poses[(self.current_pose_id) + 1].position
                
                # self.get_logger().warn(f"Goal({self.current_pose_id +1}):[{goal.x}, {goal.y}, {goal.z}]")
                
                # TODO_11: We calculate the target position using Spline functions for each x, y 
                # axis
                #Hint: you need to call the get_spline_5 function that you need to define
                position_msg.x = 0.0 #replace 0.0 with the value from the spline function
                position_msg.y = 0.0 #replace 0.0 with the value from the spline function
                
                # Since it is a 2D robot, we always set z=0.0
                # The target orientation will be defined with the current robot pose and
                # the target position calculated with the Spline function
                position_msg.z = 0.0
                
                # Check if current time (tc) is within the range (min_time, max_time)
                # If tc is not within the time range, it means the trajectory generator 
                # has finished with the current segment and it is time to change to the next
                # pair of target poses
                if not (self.elapsed_time >= time_range[0] and self.elapsed_time <= time_range[1]):
                    # if the elapsed time is above the range, then we need to change
                    # to the next time range and next pair of poses. 
                    # This is controlled by the current_pose_id counter
                    self.current_pose_id+=1
            else: # All the poses in the poses list have been processed. We can receive new request (target poses)
                
                self.get_logger().info("Final Position")
                # We use the last pose as the target position
                position_msg.x = self.poses[-1].position.x
                position_msg.y = self.poses[-1].position.y
                
                # We use the position is z to control when the robot ctrl should stop/start
                # When the robot control node receives a z=-1000, it should stop the robot, 
                # i.e., it should send cmd_vel=Twist(0)
                position_msg.z = -1000.0
                
                # All the poses have been processed and we need to clear the variable to receive new poses
                self.poses = []
                self.times = []
                
                # Set the flags to reset the state of the trajectory generator
                # and the clocks/timers
                self.got_request = False
                self.reset_vars = True
            
            # Publish the Transformation Frame of the target pose
            t = TransformStamped()

            # Define the TF header
            t.header.stamp = tc.to_msg()
            t.header.frame_id = self.odom_frame_name
            t.child_frame_id = self.target_frame_name

            # TODO:_12: Set the TF position as the target position (Calculated with the Spline functions)
            t.transform.translation.x = 0.0 #replace 0.0 with the correct value
            t.transform.translation.y = 0.0 #replace 0.0 with the correct value
            t.transform.translation.z = 0.0 #replace 0.0 with the correct value
            # In this example, we don't change the orientation
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            
            # Send the target transformation
            self.tf_broadcaster.sendTransform(t)
            
            # Publish Target Position
            self.traj_pub.publish(position_msg)

    # TODO_13 Simple 5th grade polynomial function for point to point motion
    def get_spline_5(self, start, goal, elapsed_time, time_range):
        """ TODO_13: Write a simple 5th grade polynomial function for point to point motion

        Args:
            start (Pose): initial pose for the Spline
            goal (Pose): final pose for the Spline
            elapsed_time (double): the current time since the request of the service
            time_range ([times]]): pair of times that define period for the spline function

        Returns:
            double: the target position (single axis) for the current time (tc)
        """

        
        return 0
