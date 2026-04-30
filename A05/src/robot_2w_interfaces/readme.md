## Description

This ros2 package provides the interfaces to communicate the different nodes in this workspace. 

The services provided by this interface package are:

**GetTargetPosesFromCam**: Service to request a Path with a list of target poses from the camera node. The provided path is relative to the camera frame.

**GetTFCameraOdom**: Service to request the transformation between the camera frame and the odom frame. This transformation is defined by a position (x,y,z), and orientation (quaternion). This transformation must be used to calculate the Homogeneous Transformation of the camera relative to the odom frame, i.e. H_cam_odom. Therefore, the orientation defined in quaternion format must be transformed into a Rotation matrix.

**SetDesiredPoses**: Service to send the list of target poses to the trajectory generator node. The target poses must be relative to the /odom frame. This service requires also a list
of reaching times. For example, let us assume the desired path as the sequence of target poses relative to odom path=[pose_1,pose_2,pose_3] and reaching times times=[10,15,25]. 
Then, the generated trajectory will be a Spline function that starts at the initial robot position, ends at pose_1, within the time range [0-10], i.e. it takes 10 seconds to reach from the initial robot position (obtained from the /odom topic) to pose_1. When the trajectory reaches pose_1, a second Spline function will be calculated. This second Spline function starts at pose_1, ends at pose_2, and takes 5 seconds, t=[10,15]. Finally, the last Spline function will start at pose_2, ends at pose_3, in the time range  t=[15,25], i.e., duration of 10 seconds.

## Task

Get familiar with the three services and understand the request and response messages.

## How to run

### Terminal 1 Interface information

```
cd path_to_ws/
source install/setup.bash
ros2 interface show robot_2w_interfaces/srv/GetTFCameraOdom
```

You should see something like this:

``` 
# Empty request
---
# Call succeeded = true
bool confirmation
# Human readable message
string message

# TF Camera frame wrt Odom (H_cam_odom)
geometry_msgs/TransformStamped tf_cam_odom
	#
	#
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	string child_frame_id
	Transform transform
		Vector3 translation
			float64 x
			float64 y
			float64 z
		Quaternion rotation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
```

Repeat the step for the other two services, so you get familiar with them.
