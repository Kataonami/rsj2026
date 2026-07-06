from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("my_dxl_velocity_cpp")
    urdf_path = LaunchConfiguration("urdf_path")
    start_hardware = LaunchConfiguration("start_hardware")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            urdf_path,
        ]),
        value_type=str,
    )

    controllers_file = PathJoinSubstitution([pkg, "config", "controllers.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false"
        ),
        DeclareLaunchArgument(
            "urdf_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("PFR_Arm2"),
                "PFR_Arm2.urdf",
            ]),
            description="Common PFR URDF used by ros2_control and robot_state_publisher.",
        ),
        DeclareLaunchArgument(
            "start_hardware",
            default_value="true",
            description="Start Dynamixel ros2_control and controller spawners.",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "ignore_timestamp": True,
            }],
        ),

        Node(
            package="controller_manager",
            executable="ros2_control_node",
            condition=IfCondition(start_hardware),
            output="screen",
            parameters=[
                {"robot_description": robot_description},
                controllers_file,
            ],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            condition=IfCondition(start_hardware),
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            condition=IfCondition(start_hardware),
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
