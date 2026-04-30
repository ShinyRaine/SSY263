## Description

This ros2 package provides a trajectory and path generator. The trajectory and path
are defined relative to the  "/odom" frame.  The generated target trajectory is published as 
a topic. This target trajectory should be used as the desired trajectory in the 
control node, see Figure 5 from the assignment document.

The node provides the following publishers and subscribers:

*Subscribers*:
  **/odom**: nav_msgs/msg/Odometry. The trajectory generator needs the odom information
                                to get the current robot position. This current position
                                is used to calculate a trajectory starting from and ending at
                                the robot's  initial position.
*Publishers*:
  **/path_generator/robot_2w_path**: nav_msgs/msg/Path. The Path wrt the /odom frame, 
                                                     to visualize it in rviz
  **/path_generator/trajectory**: geometry_msgs/msg/Point. 
  The target trajectory is a time-based function that provides the target position
  as a Point (x,y, 0) at a time t. This target point moves along the path. When the 
  last point has been sent, i.e., the robot has reached its destination, the final point
  is (x_final, y_final, -10000). We use that value to control when the robot control 
  should stop.  
*Service Servers*:
  **/path_generator/set_desired_poses**: robot_2w_interfaces/srv/SetDesiredPoses. 
  This service receives the list of target positions that the robot should cover,
  defined in the /odom reference frame, and the list of times when these points 
  should be reached. Both lists should have the same number of items. 
  The trajectory generator will use these target positions to calculate the 
  trajectory, using a Spline function, that the robot should follow, i.e., 
  the desired position for the robot. 

## Tasks

**robot_2w_path_generator/robot_2w_path_generator/trajectory_generator_module.py**:
Currently, the PathGenerator class receives the list of target positions and 
adds the initial robot position at the beginning of the list. In this way, the 
trajectory generator will compute a trajectory that starts at the initial 
robot position and ends at the last target position requested by a client. 
Your task is to modify the code to append the robot initial position also at the
end of the list of target positions, and append a new reaching time in the list 
reaching times. Therefore, the trajectory will start at the initial robot position
and end at the same initial robot position, i.e., it will generate a cyclic path
(round trip), see the and solve the **TODO:** inside this file. 

## How 2 run

### Terminal 1 main launch file 

This launch file will run: Gazebo, Robot State Publisher, Camera node, Rviz, and 
Path Generator node.

```
cd path_to_ws/
source install/setup.bash
ros2 launch robot_2w_path_generator robot_2w_path_generator_launch.py
```

### Terminal 2 Path client

```
cd path_to_ws/
source install/setup.bash
ros2 run robot_2w_path_generator path_client
``` 
