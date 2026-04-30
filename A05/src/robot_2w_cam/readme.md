## Description

This ros package generates a node "robot_2w_cam_sim_node" that provides two services, 
see Figure 5 from the assignment document.

**GetTFCameraOdom** (robot_2w_interfaces/srv/GetTFCameraOdom.srv): This service provides the Transformation between the camera frame and the odom frame, i.e., the information for the homogeneous transformation  
H_cam_odom.

**GetTargetPosesFromCam** (robot_2w_interfaces/srv/GetTargetPosesFromCam.srv): This service provides a randomly selected path from the sequences defined in the config file "robot_2w_cam/configs/sim_cam.yaml".

These two services should be called from a path_client node. 

## Tasks

Modify the content of the C++ file “Robot2WCam.cpp” indicated by the keyword TODO, throughout the file, mainly inside the functions **bool Robot2WCam::load_parameters()** and **void Robot2WCam::timer_callback()**

**Recommended**: When you are developing your solutions, you can try a single path instead of random paths.
To do that, change the file **robot_2w_cam/configs/sim_cam.yaml**

Modify the value of the variable sequences: number: to 1. 

In this way, only the first sequence "seq_1" will be always selected.



## How 2 run 

### Terminal 1  Camera Node

```
cd path_to_ws/
source install/setup.bash
ros2 launch robot_2w_cam robot_2w_cam_launch.py
```

### Terminal 2 Call service Cam Transformation 

```
cd path_to_ws/
source install/setup.bash
ros2 service call /robot_2w_cam/get_tf_camera_odom robot_2w_interfaces/srv/GetTFCameraOdom {}
```

### Terminal 3 Call service Get Target Poses

```
cd path_to_ws/
source install/setup.bash
ros2 service call /robot_2w_cam/get_poses_from_camera robot_2w_interfaces/srv/GetTargetPosesFromCam {}
```
