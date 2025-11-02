# 41068_ignition_husky_and_drone.launch.py
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            ExecuteProcess, RegisterEventHandler, TimerAction,
                            GroupAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution, TextSubstitution)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # ---- Args ----
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz   = LaunchConfiguration("rviz")
    nav2   = LaunchConfiguration("nav2")
    slam   = LaunchConfiguration("slam")
    map_yaml = LaunchConfiguration("map")
    world_file = LaunchConfiguration("world_file")
    husky_z = LaunchConfiguration("husky_z")
    drone_x = LaunchConfiguration("drone_x")
    drone_y = LaunchConfiguration("drone_y")
    drone_z = LaunchConfiguration("drone_z")
    nav2_ns = LaunchConfiguration("nav2_ns")

    for name, default in [
        ("use_sim_time", "True"),
        ("rviz", "True"),
        ("nav2", "True"),
        ("slam", "True"),
        ("map", ""),
        ("world_file", "large_demo.sdf"),
        ("husky_z", "0.15"),
        ("drone_x", "0.0"),
        ("drone_y", "0.0"),
        ("drone_z", "2.0"),
        ("nav2_ns", ""),
    ]:
        ld.add_action(DeclareLaunchArgument(name, default_value=default))

    # ---- Paths ----
    pkg        = FindPackageShare("41068_ignition_bringup")
    worlds_dir = PathJoinSubstitution([pkg, "worlds"])
    config_dir = PathJoinSubstitution([pkg, "config"])
    husky_xacro = PathJoinSubstitution([pkg, "urdf", "husky.urdf.xacro"])
    drone_xacro = PathJoinSubstitution([pkg, "urdf_drone", "parrot.urdf.xacro"])

    rl_yaml               = PathJoinSubstitution([config_dir, "robot_localization.yaml"])
    gz_bridge_husky_yaml  = PathJoinSubstitution([config_dir, "gazebo_bridge.yaml"])
    gz_bridge_parrot_yaml = PathJoinSubstitution([config_dir, "gazebo_bridge_parrot.yaml"])
    rviz_cfg              = PathJoinSubstitution([config_dir, "41068.rviz"])
    collision_params      = PathJoinSubstitution([config_dir, "collision_monitor_params.yaml"])
    nav2_parrot_params    = PathJoinSubstitution([config_dir, "nav2_params_parrot.yaml"])

    # *** Correct filename here ***
    slam_params           = PathJoinSubstitution([config_dir, "slam_params.yaml"])

    # ---- Gazebo ----
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
            ),
            launch_arguments={
                "gz_args": [PathJoinSubstitution([worlds_dir, world_file]), TextSubstitution(text=" -r")]
            }.items(),
        )
    )

    # ---- State publishers ----
    husky_description = ParameterValue(Command(["xacro ", husky_xacro]), value_type=str)
    drone_description = ParameterValue(Command(["xacro ", drone_xacro]), value_type=str)

    ld.add_action(Node(
        package="robot_state_publisher", executable="robot_state_publisher", output="screen",
        parameters=[{"robot_description": husky_description, "use_sim_time": use_sim_time}],
    ))

    ld.add_action(Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="parrot_state_publisher", namespace="parrot", output="screen",
        parameters=[{
            "robot_description": drone_description,
            "use_sim_time": use_sim_time,
            "frame_prefix": "parrot/",
            "publish_frequency": 0.0,
            "use_tf_static": True,
        }],
    ))

    # ---- Render & spawn ----
    husky_out = "/tmp/41068_husky.urdf"
    drone_out = "/tmp/41068_parrot.urdf"
    render_husky = ExecuteProcess(cmd=["xacro", husky_xacro, "-o", husky_out], output="both")
    render_drone = ExecuteProcess(cmd=["xacro", drone_xacro, "-o", drone_out], output="both")
    ld.add_action(render_husky); ld.add_action(render_drone)

    spawn_husky = Node(package="ros_gz_sim", executable="create", output="screen",
                       arguments=["-name", "husky", "-file", husky_out, "-z", husky_z])
    spawn_drone = Node(package="ros_gz_sim", executable="create", output="screen",
                       arguments=["-name", "parrot", "-file", drone_out, "-x", drone_x, "-y", drone_y, "-z", drone_z])

    ld.add_action(RegisterEventHandler(OnProcessExit(target_action=render_husky, on_exit=[TimerAction(period=1.5, actions=[spawn_husky])])))
    ld.add_action(RegisterEventHandler(OnProcessExit(target_action=spawn_husky, on_exit=[TimerAction(period=1.5, actions=[spawn_drone])])))

    # ---- Bridges ----
    ld.add_action(Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge_husky",
                       output="screen", parameters=[{"config_file": gz_bridge_husky_yaml, "use_sim_time": use_sim_time}]))
    ld.add_action(Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge_parrot",
                       output="screen", parameters=[{"config_file": gz_bridge_parrot_yaml, "use_sim_time": use_sim_time}]))

    # ---- Robot Localization (Husky + Parrot) ----
    ld.add_action(TimerAction(period=1.0, actions=[Node(
        package="robot_localization", executable="ekf_node", name="husky_ekf",
        output="screen", parameters=[rl_yaml, {"use_sim_time": use_sim_time}],
    )]))
    ld.add_action(TimerAction(period=1.2, actions=[Node(
        package="robot_localization", executable="ekf_node", name="ekf_parrot",
        output="screen", parameters=[rl_yaml, {"use_sim_time": use_sim_time}],
    )]))

    # ---- AMCL for Parrot (localize in Husky's /map) ----
    ld.add_action(TimerAction(period=2.1, actions=[GroupAction([
        PushRosNamespace("parrot"),
        Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen",
             parameters=[nav2_parrot_params, {"use_sim_time": use_sim_time}]),
    ])]))

    # ---- RViz ----
    ld.add_action(TimerAction(period=1.5, actions=[Node(
        package="rviz2", executable="rviz2", output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-d", rviz_cfg],
        additional_env={
            "LIBGL_ALWAYS_SOFTWARE": "1",
            "LIBGL_DRI3_DISABLE": "1",
            "QT_XCB_GL_INTEGRATION": "none",
            "MESA_GL_VERSION_OVERRIDE": "3.3",
            "MESA_GLSL_VERSION_OVERRIDE": "330",
        },
        condition=IfCondition(rviz),
    )]))

    # ---- SLAM Toolbox (Husky) ----
    ld.add_action(TimerAction(period=1.6, actions=[Node(
        package="slam_toolbox", executable="sync_slam_toolbox_node", name="slam_toolbox",
        output="screen", parameters=[slam_params, {"use_sim_time": use_sim_time}],
        condition=IfCondition(slam),
    )]))

    # Map saver
    ld.add_action(TimerAction(period=1.7, actions=[Node(
        package="nav2_map_server", executable="map_saver_server", name="map_saver",
        output="screen", parameters=[{"use_sim_time": use_sim_time, "save_map_timeout": 5.0}],
        condition=IfCondition(slam),
    )]))

    # ---- Nav2 (Husky) — do NOT autostart SLAM again ----
    ld.add_action(TimerAction(period=2.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, "launch", "41068_navigation.launch.py"])),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam": "False",        # <<< prevent duplicate SLAM
            "map": map_yaml,
        }.items(),
        condition=IfCondition(nav2),
    )]))

    # ---- Nav2 (Parrot, namespaced) — no SLAM here either ----
    ld.add_action(TimerAction(period=2.2, actions=[GroupAction([
        PushRosNamespace("parrot"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": nav2_parrot_params,
                "autostart": "True",
                "slam": "False",     # <<< Parrot uses AMCL + Husky’s /map
                "map": map_yaml,
            }.items(),
            condition=IfCondition(nav2),
        ),
    ])]))

    # ---- Collision Monitor (optional) ----
    ld.add_action(TimerAction(period=3.0, actions=[Node(
        package="nav2_collision_monitor", executable="collision_monitor",
        name="collision_monitor", namespace=nav2_ns, output="screen",
        parameters=[collision_params, {"use_sim_time": use_sim_time}],
        condition=IfCondition(nav2),
    )]))

    return ld
