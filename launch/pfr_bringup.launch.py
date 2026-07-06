from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    urdf_path = LaunchConfiguration("urdf_path")
    start_hardware = LaunchConfiguration("start_hardware")

    dynamixel_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("my_dxl_velocity_cpp"),
                "launch",
                "bringup.launch.py",
            ])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "urdf_path": urdf_path,
            "start_hardware": start_hardware,
        }.items(),
    )

    state_estimator = Node(
        package="pfr_state_estimator",
        executable="pfr_state_estimator",
        name="pfr_state_estimator",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "urdf_path": urdf_path,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "urdf_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("PFR_Arm2"),
                "PFR_Arm2.urdf",
            ]),
            description="Common PFR URDF used by Dynamixel and State Estimator.",
        ),
        DeclareLaunchArgument(
            "start_hardware",
            default_value="true",
            description="Start the Dynamixel hardware and ros2_control controllers.",
        ),
        dynamixel_bringup,
        state_estimator,
    ])
