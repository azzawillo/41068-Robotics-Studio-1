# 41068_ignition_bringup/launch/parrot_bringup.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    # Args
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    world_file = LaunchConfiguration('world_file')
    z0 = LaunchConfiguration('drone_z')

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument('rviz', default_value='False'))
    ld.add_action(DeclareLaunchArgument('world_file', default_value='simple_trees.sdf'))
    ld.add_action(DeclareLaunchArgument('drone_z', default_value='2.0'))

    # Paths
    pkg = FindPackageShare('41068_ignition_bringup')
    worlds_dir = PathJoinSubstitution([pkg, 'worlds'])
    config_dir = PathJoinSubstitution([pkg, 'config'])
    drone_xacro = PathJoinSubstitution([pkg, 'urdf_drone', 'parrot.urdf.xacro'])

    # Start Gazebo (ros_gz_sim)
    gz = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gz_args': [PathJoinSubstitution([worlds_dir, world_file]), TextSubstitution(text=' -r')]
        }.items()
    )
    ld.add_action(gz)

    # Robot description + state publisher (with frame_prefix)
    drone_description = ParameterValue(Command(['xacro ', drone_xacro]), value_type=str)
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='parrot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': drone_description,
            'use_sim_time': use_sim_time,
            'frame_prefix': 'parrot/',
            'publish_frequency': 0.0,    # publish fixed joints as /tf_static
            'use_tf_static': True        # make them latched        # <<< keep parrot TF separate
        }],
    ))

    # Render xacro -> spawn from file (more robust than -topic)
    urdf_out = '/tmp/41068_parrot.urdf'
    render = ExecuteProcess(cmd=['xacro', drone_xacro, '-o', urdf_out], output='both')
    ld.add_action(render)

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-name', 'parrot', '-file', urdf_out, '-z', z0],
    )
    # tiny delay so /clock + world exist
    ld.add_action(TimerAction(period=1.5, actions=[spawn]))

    # Bridge GZ<->ROS for the drone ONLY (uses your parrot bridge yaml)
    ld.add_action(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_parrot',
        output='screen',
        parameters=[{
            'config_file': PathJoinSubstitution([config_dir, 'gazebo_bridge_parrot.yaml']),
            'use_sim_time': use_sim_time
        }],
    ))

    # Static TF to hang the drone odom under global odom (optional but nice)
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_parrot_odom',
        arguments=['0','0','0','0','0','0','1', 'odom', 'parrot/odom']
    ))

    # EKF just for the drone (separate frames & topics)
    ld.add_action(TimerAction(period=2.0, actions=[Node(
        package='robot_localization',
        executable='ekf_node',
        name='parrot_ekf',
        output='screen',
        parameters=[PathJoinSubstitution([config_dir, 'robot_localization_parrot.yaml']),
                    {'use_sim_time': use_sim_time}],
    )]))
    

    # Optional RViz
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_dir, '41068.rviz'])],
        condition=IfCondition(rviz),
    ))

    return ld