# launch/41068_ignition_husky_and_drone.launch.py

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess,
    RegisterEventHandler, TimerAction
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # --- Args ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    nav2 = LaunchConfiguration('nav2')
    world_file = LaunchConfiguration('world_file')
    husky_z = LaunchConfiguration('husky_z')
    drone_z = LaunchConfiguration('drone_z')

    # Namespace for Nav2 + Collision Monitor (default matches your working setup)
    nav2_ns = LaunchConfiguration('nav2_ns')

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument('rviz', default_value='False'))
    ld.add_action(DeclareLaunchArgument('nav2', default_value='False'))
    ld.add_action(DeclareLaunchArgument(
        'world_file', default_value='large_demo.sdf',
        description='World SDF in package worlds/ (e.g. simple_trees.sdf, large_demo.sdf)'
    ))
    ld.add_action(DeclareLaunchArgument('husky_z', default_value='0.15'))
    ld.add_action(DeclareLaunchArgument('drone_z', default_value='2.0'))
    ld.add_action(DeclareLaunchArgument('nav2_ns', default_value=''))

    # --- Paths ---
    pkg = FindPackageShare('41068_ignition_bringup')
    worlds_dir = PathJoinSubstitution([pkg, 'worlds'])
    config_dir = PathJoinSubstitution([pkg, 'config'])
    husky_xacro = PathJoinSubstitution([pkg, 'urdf', 'husky.urdf.xacro'])
    drone_xacro = PathJoinSubstitution([pkg, 'urdf_drone', 'parrot.urdf.xacro'])
    rl_yaml = PathJoinSubstitution([config_dir, 'robot_localization.yaml'])  # single file with both ekf blocks
    gz_bridge_husky_yaml = PathJoinSubstitution([config_dir, 'gazebo_bridge.yaml'])
    gz_bridge_parrot_yaml = PathJoinSubstitution([config_dir, 'gazebo_bridge_parrot.yaml'])
    rviz_cfg = PathJoinSubstitution([config_dir, '41068.rviz'])

    # Collision Monitor params (keep separate YAML)
    collision_params = PathJoinSubstitution([config_dir, 'collision_monitor_params.yaml'])

    # --- Gazebo (GZ) ---
    gz_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gz_args': [PathJoinSubstitution([worlds_dir, world_file]), TextSubstitution(text=' -r')]
        }.items(),
    )
    ld.add_action(gz_launch)

    # --- Robot State Publishers ---
    husky_description = ParameterValue(Command(['xacro ', husky_xacro]), value_type=str)
    drone_description = ParameterValue(Command(['xacro ', drone_xacro]), value_type=str)

    ld.add_action(Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': husky_description, 'use_sim_time': use_sim_time}],
    ))

    ld.add_action(Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='parrot_state_publisher', output='screen',
        parameters=[{'robot_description': drone_description, 'use_sim_time': use_sim_time, 'frame_prefix': 'parrot/'}],
    ))

    # --- Render xacros to /tmp (optional, for GZ create) ---
    husky_out = '/tmp/41068_husky.urdf'
    drone_out = '/tmp/41068_parrot.urdf'
    render_husky = ExecuteProcess(cmd=['xacro', husky_xacro, '-o', husky_out], output='both')
    render_drone = ExecuteProcess(cmd=['xacro', drone_xacro, '-o', drone_out], output='both')
    ld.add_action(render_husky)
    ld.add_action(render_drone)

    # --- Spawn order: Husky then Drone ---
    spawn_husky = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'husky', '-file', husky_out, '-z', husky_z],
    )
    spawn_drone = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'parrot', '-file', drone_out, '-z', drone_z],
    )
    ld.add_action(RegisterEventHandler(
        OnProcessExit(target_action=render_husky, on_exit=[TimerAction(period=2.0, actions=[spawn_husky])])
    ))
    ld.add_action(RegisterEventHandler(
        OnProcessExit(target_action=spawn_husky, on_exit=[TimerAction(period=2.0, actions=[spawn_drone])])
    ))

    # --- GZ ↔ ROS bridges (unique names) ---
    ld.add_action(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge_husky', output='screen',
        parameters=[{'config_file': gz_bridge_husky_yaml, 'use_sim_time': use_sim_time}],
    ))
    ld.add_action(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge_parrot', output='screen',
        parameters=[{'config_file': gz_bridge_parrot_yaml, 'use_sim_time': use_sim_time}],
    ))

    # --- Robot Localization (two EKFs from one YAML) ---
    # YAML must have two top-level blocks: "husky_ekf:" and "ekf_parrot:" (or rename to match).
    ld.add_action(Node(
        package='robot_localization', executable='ekf_node',
        name='husky_ekf', output='screen',
        parameters=[rl_yaml, {'use_sim_time': use_sim_time}],
    ))
    ld.add_action(Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_parrot', output='screen',   # <-- name matches YAML block
        parameters=[rl_yaml, {'use_sim_time': use_sim_time}],
    ))

    # --- NO static TF between odom and parrot/odom! ---

    # --- RViz & Nav2 (Husky only) ---
    ld.add_action(Node(
        package='rviz2', executable='rviz2', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz),
    ))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([pkg, 'launch', '41068_navigation.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time
            # If 41068_navigation.launch.py accepts a namespace arg, pass it here too:
            # 'namespace': nav2_ns
        }.items(),
        condition=IfCondition(nav2),
    ))

    # --- Collision Monitor (same namespace as Nav2) ---
    # Requires config/collision_monitor_params.yaml; not lifecycle-managed.
    ld.add_action(Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        namespace=nav2_ns,
        output='screen',
        parameters=[collision_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(nav2),
    ))

    return ld
