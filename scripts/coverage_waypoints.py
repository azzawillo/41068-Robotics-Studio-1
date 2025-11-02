#!/usr/bin/env python3
import math, time, csv, yaml, os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.srv import SaveMap

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

DEFAULT_FREE_MAX = 25        # <= this considered free
DEFAULT_STEP_M   = 1.0       # grid step for waypoint generation
DEFAULT_YAW      = 0.0
SAVE_YAML_DEFAULT = "/tmp/coverage_waypoints.yaml"
CSV_LOG_DEFAULT   = "/tmp/slam_coverage_log.csv"

def q_from_yaw(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw/2.0), w=math.cos(yaw/2.0))

def yaw_from_quat(q: Quaternion) -> float:
    return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

class Coverage(Node):
    def __init__(self):
        super().__init__('coverage_planner')

        # -------- Parameters (so this works for Husky or Parrot) --------
        self.declare_parameter('free_max', DEFAULT_FREE_MAX)
        self.declare_parameter('step_m', DEFAULT_STEP_M)
        self.declare_parameter('yaw', DEFAULT_YAW)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')   # Husky=base_link, Parrot=parrot/base_link
        self.declare_parameter('save_yaml', SAVE_YAML_DEFAULT)
        self.declare_parameter('csv_log', CSV_LOG_DEFAULT)
        self.declare_parameter('autosave_period_sec', 0.0)  # set >0 (e.g., 60.0) to enable
        self.declare_parameter('save_map_prefix', '/tmp/mymap')
        self.declare_parameter('save_map_service', '/slam_toolbox/save_map')  # or '/save_map'

        self.free_max  = int(self.get_parameter('free_max').value)
        self.step_m    = float(self.get_parameter('step_m').value)
        self.yaw_fixed = float(self.get_parameter('yaw').value)
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame= self.get_parameter('base_frame').get_parameter_value().string_value
        self.save_yaml_path = self.get_parameter('save_yaml').get_parameter_value().string_value
        self.csv_path  = self.get_parameter('csv_log').get_parameter_value().string_value
        self.autosave_period = float(self.get_parameter('autosave_period_sec').value)
        self.save_map_prefix = self.get_parameter('save_map_prefix').get_parameter_value().string_value
        self.save_map_srv_name = self.get_parameter('save_map_service').get_parameter_value().string_value

        # -------- Map + TF --------
        self.map = None
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -------- Nav2 action --------
        self.client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        # -------- Logging --------
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        self.csv_fp = open(self.csv_path, 'a', newline='')
        self.csv = csv.writer(self.csv_fp)
        if os.path.getsize(self.csv_path) == 0:
            self.csv.writerow([
                'stamp_sec','stamp_nsec',
                'known_pct','free_pct','occ_pct',
                'map_res','map_w','map_h','map_origin_x','map_origin_y',
                'robot_x','robot_y','robot_yaw'
            ])
        self.last_log_flush = self.get_clock().now()

        # -------- Optional periodic map save --------
        self.save_cli = self.create_client(SaveMap, self.save_map_srv_name)
        self.last_save = self.get_clock().now()

        # Periodic SLAM/pose logging
        self.create_timer(1.0, self.log_tick)

    # ---------- Map handling ----------
    def map_cb(self, msg: OccupancyGrid):
        first = (self.map is None)
        self.map = msg
        if first:
            mi = msg.info
            self.get_logger().info(
                f"Map received ({mi.width}x{mi.height} @ {mi.resolution:.3f} m), "
                f"origin=({mi.origin.position.x:.2f},{mi.origin.position.y:.2f})"
            )

    def is_free_cell(self, cx: int, cy: int) -> bool:
        w = self.map.info.width
        h = self.map.info.height
        if cx < 0 or cx >= w or cy < 0 or cy >= h:
            return False
        v = self.map.data[cy * w + cx]
        return (v >= 0) and (v <= self.free_max)

    def cell_to_world(self, cx: int, cy: int):
        res = self.map.info.resolution
        ox  = self.map.info.origin.position.x
        oy  = self.map.info.origin.position.y
        return (ox + (cx + 0.5)*res, oy + (cy + 0.5)*res)

    # ---------- Coverage waypoint generation ----------
    def build_waypoints(self, step_m: float):
        mi = self.map.info
        res = mi.resolution
        w, h = mi.width, mi.height
        step_cells = max(1, int(step_m / res))

        # Build bbox of known cells (sampled on step grid)
        xs, ys = [], []
        data = self.map.data
        for cy in range(0, h, step_cells):
            base = cy * w
            for cx in range(0, w, step_cells):
                if data[base + cx] >= 0:
                    xs.append(cx); ys.append(cy)
        if not xs:
            self.get_logger().warn("Map has no known cells yet.")
            return []

        minx, maxx = min(xs), max(xs)
        miny, maxy = min(ys), max(ys)

        # Keep a 1-cell safety margin from edges to avoid worldToMap failures
        pad = 1
        minx = max(minx + pad, 0)
        miny = max(miny + pad, 0)
        maxx = min(maxx - pad, w - 1)
        maxy = min(maxy - pad, h - 1)
        if minx >= maxx or miny >= maxy:
            self.get_logger().warn("Bbox too small after padding; aborting coverage gen.")
            return []

        poses = []
        reverse = False
        for cy in range(miny, maxy, step_cells):          # note: exclusive upper bound
            xs_iter = range(minx, maxx, step_cells)       # note: exclusive upper bound
            if reverse:
                xs_iter = reversed(list(xs_iter))
            for cx in xs_iter:
                if not self.is_free_cell(cx, cy):
                    continue
                wx, wy = self.cell_to_world(cx, cy)

                # Clip world pose inside map bounds by small epsilon
                eps = 0.5 * res
                min_wx = mi.origin.position.x + eps
                min_wy = mi.origin.position.y + eps
                max_wx = mi.origin.position.x + w * res - eps
                max_wy = mi.origin.position.y + h * res - eps
                wx = min(max(wx, min_wx), max_wx)
                wy = min(max(wy, min_wy), max_wy)

                p = PoseStamped()
                p.header.frame_id = self.map_frame
                p.pose.position.x = wx
                p.pose.position.y = wy
                p.pose.orientation = q_from_yaw(self.yaw_fixed)
                poses.append(p)
            reverse = not reverse

        # Downsample to keep Nav2 responsive
        keep_every = 1
        if len(poses) > 500:
            keep_every = max(1, len(poses)//500)
            poses = poses[::keep_every]
        self.get_logger().info(f"Generated {len(poses)} coverage waypoints (keep_every={keep_every}).")
        return poses

    def save_waypoints_yaml(self, poses):
        data = [dict(x=float(p.pose.position.x), y=float(p.pose.position.y), yaw=float(self.yaw_fixed)) for p in poses]
        with open(self.save_yaml_path, 'w') as f:
            yaml.safe_dump(dict(waypoints=data), f)
        self.get_logger().info(f"Saved waypoint list: {self.save_yaml_path}")

    def send_waypoints(self, poses):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_through_poses action server not available")
            return
        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        goal.behavior_tree = ""  # use default BT

        self.get_logger().info(f"Sending {len(poses)} poses...")
        send_fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        goal_handle = send_fut.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by bt_navigator.")
            return

        result_fut = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_fut)
        result = result_fut.result()
        if result is None:
            self.get_logger().error("No result from bt_navigator.")
        else:
            self.get_logger().info(f"NavigateThroughPoses result: {result.result}")

    # ---------- SLAM logging ----------
    def compute_map_stats(self):
        if self.map is None:
            return None
        data = self.map.data
        total = len(data)
        if total == 0:
            return (0.0, 0.0, 0.0)
        known = sum(1 for v in data if v >= 0)
        free  = sum(1 for v in data if (v >= 0 and v <= self.free_max))
        occ   = sum(1 for v in data if v >= 65)
        return (known/total, free/total, occ/total)

    def get_robot_pose_in_map(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            return (t.x, t.y, yaw_from_quat(q))
        except TransformException:
            return (float('nan'), float('nan'), float('nan'))

    def try_save_map(self):
        if self.autosave_period <= 0.0:
            return False
        # Wait briefly for service if needed
        if not self.save_cli.service_is_ready():
            self.save_cli.wait_for_service(timeout_sec=0.5)
        if not self.save_cli.service_is_ready():
            return False
        req = SaveMap.Request()
        req.map_url = self.save_map_prefix
        req.image_format = "pgm"
        req.free_thresh = 0.25
        req.occupied_thresh = 0.65
        fut = self.save_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        return fut.done() and (fut.result() is not None)

    def log_tick(self):
        if self.map is None:
            return
        now = self.get_clock().now()
        kfo = self.compute_map_stats()
        rx, ry, ryaw = self.get_robot_pose_in_map()
        if kfo is not None:
            known_pct, free_pct, occ_pct = kfo
            mi = self.map.info
            self.csv.writerow([
                int(now.nanoseconds // 1_000_000_000),
                int(now.nanoseconds % 1_000_000_000),
                round(known_pct, 6), round(free_pct, 6), round(occ_pct, 6),
                mi.resolution, mi.width, mi.height, mi.origin.position.x, mi.origin.position.y,
                rx, ry, ryaw
            ])
            if (now - self.last_log_flush) > Duration(seconds=5.0):
                try:
                    self.csv_fp.flush()
                except Exception:
                    pass
                self.last_log_flush = now

        if self.autosave_period > 0.0 and (now - self.last_save) > Duration(seconds=self.autosave_period):
            if self.try_save_map():
                self.get_logger().info(f"Saved map snapshot to {self.save_map_prefix}.* via {self.save_map_srv_name}")
            else:
                self.get_logger().warn(f"Map save service {self.save_map_srv_name} not ready.")
            self.last_save = now

    # ---------- Orchestrate ----------
    def run(self):
        self.get_logger().info("Waiting for /map ...")
        while rclpy.ok() and self.map is None:
            rclpy.spin_once(self, timeout_sec=0.2)

        poses = self.build_waypoints(self.step_m)
        if poses:
            self.save_waypoints_yaml(poses)
            self.send_waypoints(poses)

    def destroy_node(self):
        try:
            self.csv_fp.flush()
            self.csv_fp.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = Coverage()
    try:
        node.run()
        rclpy.spin(node)   # keep logging / autosaving
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
