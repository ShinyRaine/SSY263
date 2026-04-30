import sys

from robot_2w_interfaces.srv import SetDesiredPoses

from robot_2w_interfaces.srv import GetTFCameraOdom
from robot_2w_interfaces.srv import GetTargetPosesFromCam

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

import numpy as np

# Debugging
import inspect


def quat2rot(q):
    R = np.array(
        [
            [
                1 - 2 * q.y * q.y - 2 * q.z * q.z,
                2 * q.x * q.y - 2 * q.z * q.w,
                2 * q.x * q.z + 2 * q.y * q.w,
            ],
            [
                2 * q.x * q.y + 2 * q.z * q.w,
                1 - 2 * q.x * q.x - 2 * q.z * q.z,
                2 * q.y * q.z - 2 * q.x * q.w,
            ],
            [
                2 * q.x * q.z - 2 * q.y * q.w,
                2 * q.y * q.z + 2 * q.x * q.w,
                1 - 2 * q.x * q.x - 2 * q.y * q.y,
            ],
        ]
    )

    return R


class PathClientWithCamInfo(Node):
    def __init__(self):
        super().__init__("path_client_with_cam_info")

        self.H_cam_odom = np.eye(4)
        self.H_odom_cam = np.eye(4)

        # Target poses wrt camera and odom frame
        self.poses_cam =[]
        self.poses_odom =[]
        
        self.get_logger().warn(f"{__file__}:{inspect.currentframe().f_lineno}")

        # Client to request the TF camera wrt odom
        self.cli_get_cam_odom = self.create_client(
            GetTFCameraOdom, "/robot_2w_cam/get_tf_camera_odom"
        )

        while not self.cli_get_cam_odom.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("service not available, waiting again...")

        self.req_cam_odom = GetTFCameraOdom.Request()

        # Client to request target poses wrt cam
        self.cli_get_poses_cam = self.create_client(
            GetTargetPosesFromCam, "/robot_2w_cam/get_poses_from_camera"
        )

        while not self.cli_get_poses_cam.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("service not available, waiting again...")

        self.req_poses_cam = GetTargetPosesFromCam.Request()

        # Client to send the target poses wrt odom
        self.cli = self.create_client(SetDesiredPoses, '/path_generator/set_desired_poses')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service not available, waiting again...')
        self.req = SetDesiredPoses.Request()

    def send_get_cam_odom_request(self):
        self.future = self.cli_get_cam_odom.call_async(self.req_cam_odom)

        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()

    def send_get_poses_cam_request(self):
        self.future = self.cli_get_poses_cam.call_async(self.req_poses_cam)

        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()

    def send_poses_odom_request(self):
        # Define your desired waypoints here
        self.req.poses = self.poses_odom

        # Define constant time for each pose 
        dt=7.0
        
        self.req.times = [i * dt for i in range(1, len(self.req.poses) + 1)]
        
        if not len(self.req.times) == len(self.req.poses):
            self.get_logger().error(f"Number of poses ({len(self.req.poses)}) different from times ({self.req.times}) ")
            sys.exit(-1)

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = PathClientWithCamInfo()

    # Get the Camera TF wrt to Odom
    response = minimal_client.send_get_cam_odom_request()

    minimal_client.get_logger().info(
        "Result: (%d) %s [%f, %f, %f,]"
        % (
            response.confirmation,
            response.message,
            response.tf_cam_odom.transform.translation.x,
            response.tf_cam_odom.transform.translation.y,
            response.tf_cam_odom.transform.translation.z,
        )
    )

    if not response.confirmation:
        minimal_client.get_logger().error(
            f"Service TF cam failed with msg: {response.message}"
        )
        sys.exit(-1)

    # Homogeneous Transformation Camera wrt Odom
    minimal_client.H_cam_odom = np.eye(4)

    minimal_client.H_cam_odom[:3, 3] = np.array(
        [
            response.tf_cam_odom.transform.translation.x,
            response.tf_cam_odom.transform.translation.y,
            response.tf_cam_odom.transform.translation.z,
        ]
    )

    q = Quaternion()

    q.x = response.tf_cam_odom.transform.rotation.x
    q.y = response.tf_cam_odom.transform.rotation.y
    q.z = response.tf_cam_odom.transform.rotation.z
    q.w = response.tf_cam_odom.transform.rotation.w

    minimal_client.H_cam_odom[:3, :3] = quat2rot(q)

    # Homogeneous transformation Odom wrt Cam
    minimal_client.H_odom_cam = np.linalg.inv(minimal_client.H_cam_odom)

    # print(minimal_client.H_cam_odom)
    # print(minimal_client.H_odom_cam)

    # Get target poses wrt camera frame
    response = minimal_client.send_get_poses_cam_request()

    minimal_client.get_logger().info(
        "Result: (%d) %s "
        % (
            response.confirmation,
            response.message,
        )
    )
    
    minimal_client.poses_cam = response.poses
    
    
    # Transform the target poses in the camera to the odom frame
    
    for pose in minimal_client.poses_cam:
        # minimal_client.get_logger().info(f"[{pose.position.x}, {pose.position.y}, {pose.position.z}]")
        p_cam=np.array([pose.position.x, pose.position.y, pose.position.z, 1.0])
        # minimal_client.get_logger().info(f"[{p_cam[0]}, {p_cam[1]}, {p_cam[2]}, {p_cam[3]}]")
        p_cam=p_cam.T
        
        # print(p_cam)
        p_odom = np.dot(minimal_client.H_cam_odom,p_cam)
        
        # print("---------------------------------")
        # print(p_odom)
        # print("+++++++++++++++++++++++++++++++++") 
        
        aux_p_odom = Pose()
        aux_p_odom.position.x = p_odom[0]
        aux_p_odom.position.y = p_odom[1]
        aux_p_odom.position.z = p_odom[2]
        
        minimal_client.poses_odom.append(aux_p_odom)
        
         
    # end for
    
    
    for pose in minimal_client.poses_odom:
        minimal_client.get_logger().info(f"[{pose.position.x}, {pose.position.y}, {pose.position.z}]")
    # end for
    
    
    # Send the target poses wrt Odom to the Trajectory generator
    response = minimal_client.send_poses_odom_request()
    minimal_client.get_logger().info(
        'Result: [%d] %s' %
        (response.confirmation, response.msg.data))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
