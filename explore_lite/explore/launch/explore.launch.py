#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock"
    )
    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace for explore node"
    )

    config = os.path.join(
        get_package_share_directory("explore_lite"), "config", "params.yaml"
    )

    # Keep only TF remaps; explore_lite does not use /goal_pose
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    node = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        namespace=namespace,
        output="screen",
        parameters=[config, {"use_sim_time": use_sim_time}],
        remappings=remappings,
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_namespace)
    ld.add_action(node)
    return ld
