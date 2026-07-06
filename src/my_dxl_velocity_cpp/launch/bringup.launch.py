from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("my_dxl_velocity_cpp")

    robot_description = Command([
        "xacro ",
        PathJoinSubstitution([pkg, "urdf", "single_xh540_ros2_control.urdf.xacro"])
    ])

    controllers_file = PathJoinSubstitution([pkg, "config", "controllers.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false"
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),

        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[
                {"robot_description": robot_description},
                controllers_file,
            ],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["dxl_velocity_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),

        # Node(
        #     package="controller_manager",
        #     executable="spawner",
        #     arguments=["dxl_current_controller", "--controller-manager", "/controller_manager"],
        #     output="screen",
        # ),
    ])
