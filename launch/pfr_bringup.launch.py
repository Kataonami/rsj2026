from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    dynamixel_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("my_dxl_velocity_cpp"),
                "launch",
                "bringup.launch.py",
            ])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    state_estimator = Node(
        package="pfr_state_estimator",
        executable="pfr_state_estimator",
        name="pfr_state_estimator",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        ),
        dynamixel_bringup,
        state_estimator,
    ])
