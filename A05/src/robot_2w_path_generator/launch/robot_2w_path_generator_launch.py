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
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name_in_model",
            default_value="robot_2w",
            description="Robot name in the gazebo world",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    gui = LaunchConfiguration("gui")
    prefix = LaunchConfiguration("prefix")
    robot_name_in_model = LaunchConfiguration("robot_name_in_model")

    # Add the package path to include the robot2w meshes in gazebo
    pkg_share_path = os.pathsep + os.path.join(get_package_prefix("robot_2w_description"), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  pkg_share_path


    # Start Gazebo server
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], 
        output='screen')

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

    # Launch the robot2w model
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", robot_name_in_model],
        output="screen",
        emulate_tty=True
    )

    # Launch rviz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "robot_2w_path_gen_gazebo.rviz"]
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
    path_generator_yaml = PathJoinSubstitution(
        [FindPackageShare("robot_2w_path_generator"), "configs", "robot_2w_path_gen.yaml"]
    )
    path_generator_node = Node(
        package="robot_2w_path_generator",
        # the node's name should match the header in the yaml file 
        name="path_generator",
        executable="path_generator",
        parameters=[path_generator_yaml],
        output="screen",
        emulate_tty=True,
    )

    # We create a list of nodes
    nodes = [
        start_gazebo_cmd,
        robot_state_publisher_node,
        spawn_entity,
        rviz_node,
        path_generator_node,
    ]

    # We execute each of the nodes in the list, including the declared arguments
    return LaunchDescription(declared_arguments + nodes)
