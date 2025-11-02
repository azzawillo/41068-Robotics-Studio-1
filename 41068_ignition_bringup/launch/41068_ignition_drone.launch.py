# 41068_ignition_drone.launch.py

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, TextSubstitution, Command
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    # -------- Args --------
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz         = LaunchConfiguration("rviz")
    rviz_config  = LaunchConfiguration("rviz_config")
    nav2         = LaunchConfiguration("nav2")
    nav2_ns      = LaunchConfiguration("nav2_ns")
    world_file   = LaunchConfiguration("world_file")
    z0           = LaunchConfiguration("drone_z")
    slam         = LaunchConfiguration("slam")              # NEW
    map_yaml     = LaunchConfiguration("map")               # NEW

    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="True"))
    ld.add_action(DeclareLaunchArgument("rviz", default_value="False"))
    ld.add_action(DeclareLaunchArgument("world_file", default_value="simple_trees.sdf"))
    ld.add_action(DeclareLaunchArgument("drone_z", default_value="2.0"))
    ld.add_action(DeclareLaunchArgument("nav2", default_value="False"))
    ld.add_action(DeclareLaunchArgument("nav2_ns", default_value="parrot"))
    ld.add_action(DeclareLaunchArgument("slam", default_value="True"))      # NEW
    # If your Nav2 bringup insists on a map arg even when slam:=True, give it a harmless default:
    ld.add_action(DeclareLaunchArgument("map", default_value=""))           # NEW

    # -------- Paths --------
    pkg        = FindPackageShare("41068_ignition_bringup")
    worlds_dir = PathJoinSubstitution([pkg, "worlds"])
    config_dir = PathJoinSubstitution([pkg, "config"])
    drone_xacro= PathJoinSubstitution([pkg, "urdf_drone", "parrot.urdf.xacro"])
    rl_yaml    = PathJoinSubstitution([config_dir, "robot_localization.yaml"])     # ekf_parrot block
    nav2_params= PathJoinSubstitution([config_dir, "nav2_params_parrot.yaml"])     # Parrot-specific Nav2 YAML

    # RViz config (allow override)
    default_rviz = PathJoinSubstitution([config_dir, "41068.rviz"])
    ld.add_action(DeclareLaunchArgument("rviz_config", default_value=default_rviz))

    # -------- Gazebo (ros_gz_sim) --------
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

    # -------- Robot description + state publisher --------
    drone_description = ParameterValue(Command(["xacro ", drone_xacro]), value_type=str)
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="parrot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": drone_description,
                "use_sim_time": use_sim_time,
                "frame_prefix": "parrot/",
                "publish_frequency": 0.0,   # /tf_static only
                "use_tf_static": True,
            }],
        )
    )

    # -------- Render xacro → spawn from file --------
    urdf_out = "/tmp/41068_parrot.urdf"
    render = ExecuteProcess(cmd=["xacro", drone_xacro, "-o", urdf_out], output="both")
    ld.add_action(render)

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-name", "parrot", "-file", urdf_out, "-z", z0],
    )
    ld.add_action(TimerAction(period=1.5, actions=[spawn]))

    # -------- GZ ↔ ROS bridges (drone only) --------
    ld.add_action(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_bridge_parrot",
            output="screen",
            parameters=[{
                "config_file": PathJoinSubstitution([config_dir, "gazebo_bridge_parrot.yaml"]),
                "use_sim_time": use_sim_time,
            }],
        )
    )

    # -------- Robot Localization (EKF) for the drone --------
    ld.add_action(
        TimerAction(
            period=1.8,
            actions=[Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_parrot",                     # must match "ekf_parrot" block in YAML
                output="screen",
                parameters=[rl_yaml, {"use_sim_time": use_sim_time}],
            )],
        )
    )

    # -------- SLAM (slam_toolbox) when slam:=True --------
    # Uses your config/slam_params.yaml (put it there) and runs in the same namespace as Nav2
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"])
            ),
            launch_arguments={
                "namespace": nav2_ns,
                "use_sim_time": use_sim_time,
                "slam_params_file": PathJoinSubstitution([config_dir, "slam_params.yaml"]),
            }.items(),
            condition=IfCondition(slam),
        )
    )

    # -------- Nav2 (optional) --------
    # Pass BOTH slam and map arguments; when slam:=True, Nav2 will bring up without AMCL and ignore map.
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"])
            ),
            launch_arguments={
                "namespace":   nav2_ns,
                "use_sim_time":use_sim_time,
                "params_file": nav2_params,
                "autostart":  "True",
                "slam":        slam,         # NEW: tell Nav2 we’re using SLAM
                "map":         map_yaml,     # NEW: satisfy bringup’s required arg
            }.items(),
            condition=IfCondition(nav2),
        )
    )

    # -------- RViz (optional) --------
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=["-d", rviz_config],
            condition=IfCondition(rviz),
        )
    )

    return ld
