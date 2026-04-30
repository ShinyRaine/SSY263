from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare




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

    # The joint state publisher finds all the "continuous" joints defined in 
    # the urdf file and publish them as the joint_state topic. It also creates
    # a GUI to move these joints with sliders
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )
    
    # The robot state publisher advertises the joint_state and the robot_description topics
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    # Launch rviz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "robot_2w_view.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # We create a list of nodes
    nodes = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    # We execute each of the nodes in the list, including the declared arguments
    return LaunchDescription(declared_arguments + nodes)
