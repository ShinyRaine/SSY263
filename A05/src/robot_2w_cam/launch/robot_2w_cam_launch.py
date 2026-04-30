from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
from ament_index_python import get_package_prefix


def generate_launch_description():
    # Declare arguments: We need to declare these arguments to expose them to the
    # command line
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="robot_2w_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="robot_2w.urdf.xacro",
            description="URDF/XACRO description file with the 2-wheeled robot.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Rviz2 and Joint State Publisher gui automatically \
        with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup.",
        )
    )



    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    gui = LaunchConfiguration("gui")
    prefix = LaunchConfiguration("prefix")
    # gazebo_world = LaunchConfiguration("gazebo_world")
    # robot_name_in_model = LaunchConfiguration("robot_name_in_model")
    # gazebo_world_config_file = PathJoinSubstitution(
    #     [FindPackageShare(description_package), "world", gazebo_world]
    # )

    # Add the package path to include the robot2w meshes in gazebo
    # pkg_share_path = os.pathsep + os.path.join(get_package_prefix("robot_2w_description"), 'share')
    # if 'GAZEBO_MODEL_PATH' in os.environ:
    #     os.environ['GAZEBO_MODEL_PATH'] += pkg_share_path
    # else:
    #     os.environ['GAZEBO_MODEL_PATH'] =  pkg_share_path


    # Start Gazebo server
    # start_gazebo_cmd = ExecuteProcess(
    #     cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_config_file],
    #     output='screen')

    # Get robot description from the URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # The robot state publisher advertises the joint_state and the robot_description topics
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    # The joint state publisher finds all the "continuous" joints defined in 
    # the urdf file and publish them as the joint_state topic. It also creates
    # a GUI to move these joints with sliders
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )
    
   
    # To unify the different models, we create static transformations between their
    # reference frames
    
    # Build the arguments list for the static_transform_publisher
    args = [
        '--frame-id', 'odom',
        '--child-frame-id', 'camera',
        '--x', str('3.0'),
        '--y', str('3.0'),
        '--z', str('2.0'),
        '--roll', str('0.0'),  
        '--pitch', str('3.14159'),
        '--yaw', str('0.0'),  
        ]
    
    # Publish the TF between the camera and the odom frames
    # NOTE: If you change the parameters in the variable "args", you need to also modify  
    # the camera parameters defined in the yam file robot_2w_cam/configs/sim_cam.yaml
    camera_frame_st=Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_h_cam_odom",
        output="both",
        arguments=args
    )
    
    # Build the arguments list for the static_transform_publisher 
    args = [
        '--frame-id', 'odom',
        '--child-frame-id', 'base_footprint',
        # '--x', str('0.0'),
        # '--y', str('0.0'),
        # '--z', str('0.0'),
        # '--roll', str('0.0'),  # Include roll argument
        # '--pitch', str('0.0'),  # Include pitch argument
        # '--yaw', str('0.0'),  # Include yaw argument
        ]
    
    # Publish the TF between the base_footprint and the odom frames
    base_footprint_frame_st=Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_h_footprint_odom",
        output="both",
        arguments=args
    )
    
    # Simulated Camera Node. This node simulates a camera that provides the target
    # poses wrt the camera frame
    cam_sim_yaml = PathJoinSubstitution(
        [FindPackageShare("robot_2w_cam"), "configs", "sim_cam.yaml"]
    )
    cam_sim_node = Node(
        package="robot_2w_cam",
        # the node's name should match the header in the yaml file 
        name="robot_2w_cam",
        executable="robot_2w_cam_sim_node",
        parameters=[cam_sim_yaml],
        output="screen",
        emulate_tty=True,
    )
    
    # Launch rviz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "robot_2w_cam.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )
    
    # Path and Trajectory generator. This node receives the target path from a 
    # client. This target path is relative to the odom frame
    # path_generator_yaml = PathJoinSubstitution(
    #     [FindPackageShare("robot_2w_path_generator"), "configs", "robot_2w_path_gen.yaml"]
    # )
    # path_generator_node = Node(
    #     package="robot_2w_path_generator",
    #     # the node's name should match the header in the yaml file 
    #     name="path_generator",
    #     executable="path_generator",
    #     parameters=[path_generator_yaml],
    #     output="screen",
    #     emulate_tty=True,
    # )
    
    # Kinematic Controller. This node reads the target positions and the current
    # robot's pose. Calculates an error defined in the robot's frame and computes
    # a commanded velocity /cmd_vel relative to the robot's frame. This cmd_vel
    # is sent to the robot's differential drive to change the wheel velocities and
    # move the robot.
    # robot_ctrl_yaml = PathJoinSubstitution(
    #     [FindPackageShare("robot_2w_ctrl"), "configs", "robot_2w_ctrl.yaml"]
    # )
    # robot_control_node = Node(
    #     package="robot_2w_ctrl",
    #     # the node's name should match the header in the yaml file 
    #     name="robot_2w_control",
    #     executable="robot_2w_control_node",
    #     parameters=[robot_ctrl_yaml],
    #     output="screen",
    #     emulate_tty=True,
    # )

    # We create a list of nodes
    nodes = [
        base_footprint_frame_st,
        robot_state_publisher_node,
        joint_state_publisher_node,
        camera_frame_st,
        cam_sim_node,
        rviz_node,
    ]
    
    # We execute each of the nodes in the list, including the declared arguments
    return LaunchDescription(declared_arguments + nodes)
