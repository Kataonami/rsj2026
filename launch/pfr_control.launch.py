from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_dir = Path(__file__).resolve().parent

    joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "joy.launch.py")),
    )

    pfr_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "pfr_bringup.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    high_level_controller = Node(
        package="pfr_high_level_control",
        executable="pfr_high_level_control",
        name="pfr_high_level_control",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    low_level_controller = Node(
        package="pfr_low_level_control",
        executable="pfr_low_level_control",
        name="pfr_low_level_control",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time for the controller nodes.",
        ),
        pfr_bringup,
        joy,
        high_level_controller,
        low_level_controller,
    ])
