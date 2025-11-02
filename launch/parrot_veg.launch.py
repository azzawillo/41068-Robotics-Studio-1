from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='41068_ignition_bringup',
            executable='parrot_veg_node.py',
            name='parrot_veg',
            output='screen',
            parameters=[
                {'image_topic': '/parrot/camera/image'},
                {'publish_debug': True},
                {'hsv_lower': [25, 40, 20]},
                {'hsv_upper': [90, 255, 255]},
                {'min_area_px': 200},
                {'downscale': 1.0},
            ],
        ),
        Node(
            package='41068_ignition_bringup',
            executable='parrot_lawnmower.py',
            name='parrot_lawnmower',
            output='screen',
            parameters=[
                {'area_min_x': -10.0},
                {'area_max_x':  10.0},
                {'area_min_y': -10.0},
                {'area_max_y':  10.0},
                {'lane_spacing': 2.0},
                {'speed': 0.7},
                {'yaw_rate': 0.8},
                {'goal_tol': 0.4},
            ],
        ),
    ])
