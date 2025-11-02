from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))

    config_path = PathJoinSubstitution([FindPackageShare('41068_ignition_bringup'), 'config'])

    slam = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': PathJoinSubstitution([config_path, 'slam_params.yaml'])
        }.items()
    )

    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([config_path, 'nav2_params.yaml'])
        }.items()
    )

    ld.add_action(slam)
    ld.add_action(nav2)
    return ld
