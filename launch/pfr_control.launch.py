from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    urdf_path = LaunchConfiguration("urdf_path")
    start_hardware = LaunchConfiguration("start_hardware")
    start_rviz = LaunchConfiguration("start_rviz")
    low_level_control_mode = LaunchConfiguration("low_level_control_mode")
    launch_dir = Path(__file__).resolve().parent

    joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "joy.launch.py")),
    )

    pfr_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "pfr_bringup.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "urdf_path": urdf_path,
            "start_hardware": start_hardware,
        }.items(),
    )

    high_level_controller = Node(
        package="pfr_high_level_control",
        executable="pfr_high_level_control",
        name="pfr_high_level_control",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "trajectory_publish_rate_hz": 20.0,
        }],
    )

    low_level_controller = Node(
        package="pfr_low_level_control",
        executable="pfr_low_level_control",
        name="pfr_low_level_control",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "urdf_path": urdf_path,
            "control_rate_hz": 50.0,
            "control_mode": low_level_control_mode,
        }],
    )

    path_visualizer = Node(
        package="pfr_rviz_visualization",
        executable="ee_path_visualizer",
        name="pfr_ee_path_visualizer",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(start_rviz),
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution([
                FindPackageShare("pfr_rviz_visualization"),
                "config",
                "pfr_control.rviz",
            ]),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time for the controller nodes.",
        ),
        DeclareLaunchArgument(
            "urdf_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("PFR_Arm2"),
                "PFR_Arm2.urdf",
            ]),
            description="Common PFR URDF used by all model consumers.",
        ),
        DeclareLaunchArgument(
            "start_hardware",
            default_value="true",
            description="Start the Dynamixel hardware and ros2_control controllers.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="false",
            description="Start RViz with desired and actual end-effector paths.",
        ),
        DeclareLaunchArgument(
            "low_level_control_mode",
            default_value="generalized_jacobian",
            description=(
                "Low-level mode: 'generalized_jacobian' or 'analytic_ik'."
            ),
        ),
        pfr_bringup,
        joy,
        high_level_controller,
        low_level_controller,
        path_visualizer,
        rviz,
    ])
