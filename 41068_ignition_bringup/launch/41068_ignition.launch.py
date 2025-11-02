from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # --- Args ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    nav2 = LaunchConfiguration('nav2')
    world = LaunchConfiguration('world')

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True',  description='use /clock'))
    ld.add_action(DeclareLaunchArgument('rviz',        default_value='False', description='launch RViz'))
    ld.add_action(DeclareLaunchArgument('nav2',        default_value='True',  description='launch Nav2'))
    ld.add_action(DeclareLaunchArgument(
        'world', default_value='simple_trees.sdf',
        description='World SDF in package worlds/ (e.g. simple_trees.sdf, large_demo.sdf)'
    ))

    # --- Paths ---
    pkg = FindPackageShare('41068_ignition_bringup')
    config_dir = PathJoinSubstitution([pkg, 'config'])
    worlds_dir = PathJoinSubstitution([pkg, 'worlds'])
    husky_xacro = PathJoinSubstitution([pkg, 'urdf', 'husky.urdf.xacro'])
    rviz_cfg   = PathJoinSubstitution([config_dir, '41068.rviz'])
    bridge_yaml= PathJoinSubstitution([config_dir, 'gazebo_bridge.yaml'])
    rl_yaml    = PathJoinSubstitution([config_dir, 'robot_localization.yaml'])

    # --- Start Gazebo (ros_gz_sim) ---
    gz = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            # pass full path to the world + "-r"
            'gz_args': [PathJoinSubstitution([worlds_dir, world]), TextSubstitution(text=' -r')]
        }.items()
    )
    ld.add_action(gz)

    # --- robot_state_publisher ---
    robot_description = ParameterValue(Command(['xacro ', husky_xacro]), value_type=str)
    ld.add_action(Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    ))

    # --- Prefer robust spawn from rendered file (not /robot_description topic) ---
    urdf_out = '/tmp/41068_husky.urdf'
    render = ExecuteProcess(cmd=['xacro', husky_xacro, '-o', urdf_out], output='both')
    ld.add_action(render)

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'husky', '-file', urdf_out, '-z', '0.40'],
    )
    ld.add_action(TimerAction(period=1.0, actions=[spawn]))

    # --- Bridge topics (ros_gz_bridge) ---
    ld.add_action(Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen', name='gz_bridge_husky',
        parameters=[{'config_file': bridge_yaml, 'use_sim_time': use_sim_time}],
    ))

    # --- Robot Localization (node name MUST match YAML block "husky_ekf") ---
    ld.add_action(TimerAction(period=1.2, actions=[Node(
        package='robot_localization', executable='ekf_node',
        name='husky_ekf',  # <— matches the top-level key in robot_localization.yaml
        output='screen',
        parameters=[rl_yaml, {'use_sim_time': use_sim_time}],
    )]))

    # --- RViz (optional) ---
    ld.add_action(Node(
        package='rviz2', executable='rviz2', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz),
    ))

    # --- Nav2 (optional) ---
    nav2_inc = IncludeLaunchDescription(
        PathJoinSubstitution([pkg, 'launch', '41068_navigation.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(nav2),
    )
    # give EKF a head start to avoid TF timeouts
    ld.add_action(TimerAction(period=2.0, actions=[nav2_inc]))

    return ld
