import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import threading
import time

class RotateWheelNode(Node):
  def __init__(self,name):
    super().__init__(name)
    self.get_logger().info(f"node {name} init..")
    self._init_joint_states()

    self.joint_states_publisher_=self.create_publisher(
      JointState,"joint_states", 10
    )
    #  the publish rate
    self.pub_rate = self.create_rate(30) #30Hz
    self.thread_ = threading.Thread(target=self._thread_pub)
    self.thread_.start()

  def _thread_pub(self):
    last_update_time = time.time()
    while rclpy.ok():
      delta_time = time.time()-last_update_time
      last_update_time = time.time()
      self.joint_states.position[0] += delta_time*self.joint_states.velocity[0]
      self.joint_states.position[1] += delta_time*self.joint_states.velocity[1]
      self.joint_states.velocity = self.joint_speeds
      self.joint_states.header.stamp = self.get_clock().now().to_msg()
      self.joint_states_publisher_.publish(self.joint_states)
      self.pub_rate.sleep()

  def _init_joint_states(self):
    self.joint_speeds =[0.0, 0.0]
    self.joint_states = JointState()
    self.joint_states.header.stamp=self.get_clock().now().to_msg()
    self.joint_states.header.frame_id = ""
    #Use the same joint names as in the URDF file
    self.joint_states.name = ['left_wheel_joint', 'right_wheel_joint']
    self.joint_states.position = [0.0, 0.0]
    self.joint_states.velocity = self.joint_speeds
    self.joint_states.effort =[]

  def update_speed(self, speeds):
    self.joint_speeds =speeds


def main(args=None):
  rclpy.init(args=args)
  node = RotateWheelNode("rotate_2wRobot_wheel")
  node.update_speed([5.0, -5.0])
  rclpy.spin(node)
  rclpy.shutdown()
