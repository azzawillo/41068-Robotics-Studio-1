#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def wrap(a):
    return (a + math.pi) % (2*math.pi) - math.pi


class LawnMower(Node):
    def __init__(self):
        super().__init__('parrot_lawnmower')
        self.declare_parameter('area_min_x', -10.0)
        self.declare_parameter('area_max_x',  10.0)
        self.declare_parameter('area_min_y', -10.0)
        self.declare_parameter('area_max_y',  10.0)
        self.declare_parameter('lane_spacing', 2.0)
        self.declare_parameter('speed', 0.7)
        self.declare_parameter('yaw_rate', 0.8)
        self.declare_parameter('goal_tol', 0.4)

        aminx = float(self.get_parameter('area_min_x').value)
        amaxx = float(self.get_parameter('area_max_x').value)
        aminy = float(self.get_parameter('area_min_y').value)
        amaxy = float(self.get_parameter('area_max_y').value)
        lane  = float(self.get_parameter('lane_spacing').value)

        ys = np.arange(aminy, amaxy + 1e-6, lane)
        self.waypoints = []
        flip = False
        for y in ys:
            if flip:
                self.waypoints.append((amaxx, y, 0.0))
                self.waypoints.append((aminx, y, math.pi))
            else:
                self.waypoints.append((aminx, y, math.pi))
                self.waypoints.append((amaxx, y, 0.0))
            flip = not flip

        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        self.sub = self.create_subscription(Odometry, '/parrot/odometry', self.cb_odom, qos)
        self.pub = self.create_publisher(Twist, '/parrot/cmd_vel', 10)

        self.idx = 0
        self.pose = None
        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz

        self.speed = float(self.get_parameter('speed').value)
        self.yaw_rate = float(self.get_parameter('yaw_rate').value)
        self.goal_tol = float(self.get_parameter('goal_tol').value)

        self.get_logger().info(f'Lawnmower: {len(self.waypoints)} waypoints ready')

    def cb_odom(self, msg: Odometry):
        self.pose = msg

    def loop(self):
        if self.pose is None or self.idx >= len(self.waypoints):
            return

        x = self.pose.pose.pose.position.x
        y = self.pose.pose.pose.position.y
        q = self.pose.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

        gx, gy, gyaw = self.waypoints[self.idx]
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        gdir = math.atan2(dy, dx)
        yaw_err = wrap(gdir - yaw)

        cmd = Twist()
        cmd.angular.z = max(min(yaw_err, self.yaw_rate), -self.yaw_rate)
        cmd.linear.x = self.speed * max(0.0, 1.0 - abs(yaw_err))
        self.pub.publish(cmd)

        if dist < self.goal_tol:
            self.idx += 1
            self.get_logger().info(f'Waypoint {self.idx}/{len(self.waypoints)} reached')


def main():
    rclpy.init()
    n = LawnMower()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
