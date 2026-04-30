from robot_2w_interfaces.srv import SetDesiredPoses
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('path_client')
        # Create a client to request the service 'set_desired_poses'
        self.cli = self.create_client(SetDesiredPoses, '/path_generator/set_desired_poses')
        
        # Connect to the service
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        # Define the request variable based on the service type
        self.req = SetDesiredPoses.Request()

    def send_request(self):
        """Send request to the service

        Returns:
            service response: the response from the service
        """
        # Hardcoded list of target poses
        self.req.poses = [ Pose( 
                                position=Point(x=2.0, y=0.0, z=0.0),  
                                orientation=Quaternion(w=1.0)  # Assuming no rotation
                                ),
                            Pose( 
                                position=Point(x=3.0, y=1.0, z=0.0),  
                                orientation=Quaternion(w=1.0)  # Assuming no rotation
                                ),
                            Pose( 
                                position=Point(x=3.0, y=3.0, z=0.0),  
                                orientation=Quaternion(w=1.0)  # Assuming no rotation
                                ),
                            Pose( 
                                position=Point(x=2.0, y=4.0, z=0.0),  
                                orientation=Quaternion(w=1.0)  # Assuming no rotation
                                ),
                            Pose( 
                                position=Point(x=0.0, y=4.0, z=0.0),  
                                orientation=Quaternion(w=1.0)  # Assuming no rotation
                                ),
            # Add more waypoints as needed
        ]

        # Requested reaching times. The trajectory generator will split the path into
        # segments formed by a pair of consecutive target poses.
        # In this example:
        # First segment: pose_1=[2,0,0], pose_2=[3,1,0], and the time to move from
        # pose_1 to pose_2 is 7 seconds, i.e. the time range for the spline is [0, 7]s
        # Second segment: pose_2=[3,1,0], pose_3=[3,3,0] with a time range [7, 14]s
        # etc! 
        self.req.times = [7.0, 14.0, 21.0, 28.0, 35.0]
        
        # Call the service and make the request
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    print(args)
    
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request()
    minimal_client.get_logger().info(
        'Result: [%d] %s' %
        (response.confirmation, response.msg.data))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()