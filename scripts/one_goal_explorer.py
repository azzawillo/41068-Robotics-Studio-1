#!/usr/bin/env python3
# one_goal_explorer.py
# Picks a single safe goal on the nearest frontier and sends it to Nav2.

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

import tf_transformations
from tf2_ros import Buffer, TransformListener

UNK = -1  # unknown in OccupancyGrid

def yaw_to_quat(yaw: float) -> Quaternion:
    q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

class OneGoalExplorer(Node):
    def __init__(self):
        super().__init__('one_goal_explorer')

        # ---- Params ----
        self.declare_parameter('edge_margin_m', 0.35)
        self.declare_parameter('safety_radius_m', 0.40)
        self.declare_parameter('min_frontier_size', 12)
        self.declare_parameter('plan_timeout_s', 180.0)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        # Threshold tuning
        self.declare_parameter('free_thresh', 25)   # <= free
        self.declare_parameter('occ_thresh', 65)    # >= occupied
        self.declare_parameter('core_cells', 2)     # inner core must be known-free

        self.edge_margin_m   = float(self.get_parameter('edge_margin_m').value)
        self.safety_radius_m = float(self.get_parameter('safety_radius_m').value)
        self.min_frontier    = int(self.get_parameter('min_frontier_size').value)
        self.plan_timeout    = float(self.get_parameter('plan_timeout_s').value)
        self.map_topic       = self.get_parameter('map_topic').value
        self.global_frame    = self.get_parameter('global_frame').value
        self.base_frame      = self.get_parameter('base_frame').value

        self.FREE_THRESH = int(self.get_parameter('free_thresh').value)
        self.OCC_THRESH  = int(self.get_parameter('occ_thresh').value)
        self.CORE_CELLS  = max(1, int(self.get_parameter('core_cells').value))

        # ---- Subscriptions / QoS ----
        # Map is commonly latched (TRANSIENT_LOCAL). Mirror it to receive data reliably.
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, map_qos)

        # TF + Nav2 action
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Tick @1 Hz: choose and send a goal if idle
        self.timer = self.create_timer(1.0, self.tick)

        self.map = None
        self.navigating = False

        self.get_logger().info('OneGoalExplorer started.')

    # ---------- Utilities ----------
    def get_yaw(self, info) -> float:
        q = info.origin.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def map_to_world(self, ix: int, iy: int, info):
        yaw = self.get_yaw(info)
        c = math.cos(yaw); s = math.sin(yaw)
        gx = (ix + 0.5) * info.resolution
        gy = (iy + 0.5) * info.resolution
        rx =  c*gx - s*gy
        ry =  s*gx + c*gy
        return (rx + info.origin.position.x, ry + info.origin.position.y)

    def cell(self, data, w, h, ix, iy):
        if ix < 0 or iy < 0 or ix >= w or iy >= h:
            return None
        return data[iy*w + ix]

    def is_free(self, data, w, h, ix, iy) -> bool:
        v = self.cell(data, w, h, ix, iy)
        return v is not None and v != UNK and v <= self.FREE_THRESH

    def is_unknown(self, data, w, h, ix, iy) -> bool:
        v = self.cell(data, w, h, ix, iy)
        return v == UNK

    def is_occ(self, data, w, h, ix, iy) -> bool:
        v = self.cell(data, w, h, ix, iy)
        return v is not None and v >= self.OCC_THRESH

    def ring_free(self, data, w, h, ix, iy, cells_rad) -> bool:
        """
        Safety: inner CORE_CELLS square must be known-free; outer ring up to cells_rad
        must have no occupied (unknown is tolerated). This enables stepping toward frontiers.
        """
        # Inner core strict
        for dx in range(-self.CORE_CELLS, self.CORE_CELLS + 1):
            for dy in range(-self.CORE_CELLS, self.CORE_CELLS + 1):
                v = self.cell(data, w, h, ix + dx, iy + dy)
                if v is None or v == UNK or v >= self.OCC_THRESH:
                    return False

        if cells_rad <= self.CORE_CELLS:
            return True

        # Outer ring: no occupied; unknown is OK
        for dx in range(-cells_rad, cells_rad + 1):
            for dy in (-cells_rad, cells_rad):
                v = self.cell(data, w, h, ix + dx, iy + dy)
                if v is None or (v is not None and v >= self.OCC_THRESH):
                    return False
        for dy in range(-cells_rad + 1, cells_rad):
            for dx in (-cells_rad, cells_rad):
                v = self.cell(data, w, h, ix + dx, iy + dy)
                if v is None or (v is not None and v >= self.OCC_THRESH):
                    return False
        return True

    # ---------- Frontier detection ----------
    def find_frontiers(self, grid: OccupancyGrid):
        data = grid.data
        w = grid.info.width
        h = grid.info.height
        res = grid.info.resolution

        edge_cells = max(0, int(self.edge_margin_m / res))
        sx = edge_cells; sy = edge_cells
        ex = w - edge_cells; ey = h - edge_cells

        frontier_mask = [[False]*h for _ in range(w)]
        for iy in range(sy, ey):
            row = iy * w
            for ix in range(sx, ex):
                v = data[row + ix]
                if v != UNK:
                    continue
                # 4-neighborhood has free?
                if (self.is_free(data, w, h, ix+1, iy) or
                    self.is_free(data, w, h, ix-1, iy) or
                    self.is_free(data, w, h, ix, iy+1) or
                    self.is_free(data, w, h, ix, iy-1)):
                    frontier_mask[ix][iy] = True

        # BFS clustering
        visited = [[False]*h for _ in range(w)]
        clusters = []
        for iy in range(sy, ey):
            for ix in range(sx, ex):
                if frontier_mask[ix][iy] and not visited[ix][iy]:
                    q = [(ix, iy)]
                    visited[ix][iy] = True
                    pts = []
                    while q:
                        cx, cy = q.pop()
                        pts.append((cx, cy))
                        for nx, ny in ((cx+1,cy),(cx-1,cy),(cx,cy+1),(cx,cy-1)):
                            if sx <= nx < ex and sy <= ny < ey and frontier_mask[nx][ny] and not visited[nx][ny]:
                                visited[nx][ny] = True
                                q.append((nx, ny))
                    if len(pts) >= self.min_frontier:
                        clusters.append(pts)
        return clusters

    def nearest_cluster_center(self, clusters, grid: OccupancyGrid, robot_xy):
        if not clusters:
            return None
        best = None
        best_d2 = float('inf')
        for pts in clusters:
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            wx, wy = self.map_to_world(int(round(cx)), int(round(cy)), grid.info)
            d2 = (wx - robot_xy[0])**2 + (wy - robot_xy[1])**2
            if d2 < best_d2:
                best_d2 = d2
                best = (int(round(cx)), int(round(cy)))
        return best

    # ---------- ROS Callbacks / Loop ----------
    def on_map(self, msg: OccupancyGrid):
        self.map = msg

    def get_robot_xy(self):
        if self.map is None:
            return None
        try:
            tr = self.tf_buffer.lookup_transform(
                self.map.header.frame_id, self.base_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.2)
            )
            return (tr.transform.translation.x, tr.transform.translation.y)
        except Exception as e:
            self.get_logger().warning(f"TF lookup failed: {e}")
            return None

    def tick(self):
        if self.navigating or self.map is None:
            return

        robot_xy = self.get_robot_xy()
        if robot_xy is None:
            return

        clusters = self.find_frontiers(self.map)
        if not clusters:
            self.get_logger().info("No frontiers found. Exploration complete?")
            return

        goal_ij = self.nearest_cluster_center(clusters, self.map, robot_xy)
        if goal_ij is None:
            return

        # Search for a safe landing cell near the frontier center
        res = self.map.info.resolution
        base_rad = max(1, int(self.safety_radius_m / res))
        gx, gy = goal_ij
        data = self.map.data
        w = self.map.info.width
        h = self.map.info.height

        best = None
        best_cost = float('inf')

        # Try full radius, relax by 2 cells each pass; expand window a bit as we relax
        for ring_cells in range(base_rad, 0, -2):
            found = False
            search_win = max(5, min(12, ring_cells + 4))
            for dx in range(-search_win, search_win + 1):
                for dy in range(-search_win, search_win + 1):
                    ix = gx + dx; iy = gy + dy
                    if not self.is_free(data, w, h, ix, iy):
                        continue
                    if not self.ring_free(data, w, h, ix, iy, ring_cells):
                        continue
                    cost = abs(dx) + abs(dy)  # prefer closer to frontier center
                    if cost < best_cost:
                        best_cost = cost
                        best = (ix, iy)
                        found = True
            if found:
                break

        if best is None:
            self.get_logger().info("Nearest frontier has no safe landing cell. Skipping.")
            return

        wx, wy = self.map_to_world(best[0], best[1], self.map.info)
        self.send_nav_goal(wx, wy)

    def send_nav_goal(self, x: float, y: float):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning("Nav2 action server not ready")
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation = yaw_to_quat(0.0)

        self.navigating = True
        self.get_logger().info(f"Sending goal: ({x:.2f}, {y:.2f})")
        self.nav_client.send_goal_async(goal).add_done_callback(self.on_goal_sent)

    def on_goal_sent(self, fut):
        goal_handle = fut.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().info("Goal rejected; will try another frontier.")
            self.navigating = False
            return
        self.get_logger().info("Goal accepted.")
        goal_handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, fut):
        self.navigating = False
        try:
            result = fut.result()
            self.get_logger().info(f"NavigateToPose finished with status: {result.status}")
        except Exception as e:
            self.get_logger().warning(f"Result retrieval failed: {e}")

def main():
    rclpy.init()
    node = OneGoalExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
