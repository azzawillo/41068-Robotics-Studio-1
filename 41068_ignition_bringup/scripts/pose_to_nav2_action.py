#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class PoseToNav2(Node):
    def __init__(self):
        super().__init__('pose_to_nav2')

        # params
        self.declare_parameter('in_topic', '/explore/goal')      # PoseStamped from explore_lite
        self.declare_parameter('action_name', '/navigate_to_pose')  # Nav2 action
        self.declare_parameter('global_frame', 'map')

        self.in_topic = self.get_parameter('in_topic').get_parameter_value().string_value
        self.action_name = self.get_parameter('action_name').get_parameter_value().string_value
        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value

        # nav2 action client
        self.ac = ActionClient(self, NavigateToPose, self.action_name)

        # subscribe to explore_lite goal
        self.sub = self.create_subscription(PoseStamped, self.in_topic, self.cb, 10)

        self.get_logger().info(f'PoseToNav2 bridging {self.in_topic} -> {self.action_name}')

    def cb(self, msg: PoseStamped):
        # stamp + frame sanity
        msg.header.frame_id = msg.header.frame_id or self.global_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        if not self.ac.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('Nav2 action server not ready')
            return

        goal = NavigateToPose.Goal()
        goal.pose = msg

        self.get_logger().info(
            f'Sending NavigateToPose: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) in {msg.header.frame_id}'
        )
        self.ac.send_goal_async(goal).add_done_callback(self._on_goal_sent)

    def _on_goal_sent(self, fut):
        gh = fut.result()
        if not gh or not gh.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            return
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, fut):
        try:
            result = fut.result()
            self.get_logger().info(f'Nav2 finished with status: {result.status}')
        except Exception as e:
            self.get_logger().warn(f'Failed to get result: {e}')

def main():
    rclpy.init()
    node = PoseToNav2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
