## Description

This ros2 package provides the urdf/xacro files with the robot2w description. 
This robot xacro file is used to define the joints and transformations in the robot.
Furthermore, this xacro file includes the robot's sensors (imu, lidar, etc) and actuators
(differential drive) in the form of gazebo plugins. 

These sensors and actuators definitions can be found in the files:

**robot_2w_description/urdf/robot_2w_sensors.urdf.xacro**: sensor definitions and gazebo plugins

**robot_2w_description/urdf/robot_2w_actuators.urdf.xacro**: actuator definition and gazebo plugins

The main xacro file is **robot_2w_description/urdf/robot_2w.urdf.xacro**, this file load the link definitions (*robot_2w_description/urdf/robot_2w_links/*) and the above sensor and actuator definitions. 

This ros package also includes other models needed to build the SSY263 Arena. 


## Modify this file with your solutions from A03

Include your solutions for the tasks 01 of Assignment 03!!!! to see the robot in the correct form.


## How to run

### Terminal 1 Gazebo and Rviz

Once you include your solutions from Assignment 03, you should see the robot in gazebo using the following command:

```
cd path_to_ws/
source install/setup.bash
ros2 launch robot_2w_description gazebo_robot.launch.py
```

### Terminal 2 Teleop-twist-Keyboar

The urdf/xacro file includes the robot's actuator (differential drive). This actuator 
subscribes to the Twist topic /cmd_vel. Using the linear velocity in the x axis and the 
angular velocity in the z axis (relative to the robot's frame), the differential drive 
calculates the wheel angular velocities (left and right). Then, the robot can move.

```
cd path_to_ws/
source install/setup.bash
ros2 run  teleop_twist_keyboard teleop_twist_keyboard
```


