#!/usr/bin/env python3
"""
Parrot vegetation detector (ROS 2 / rclpy)

Subscribes:
  - image (sensor_msgs/Image) [param: image_topic, default '/parrot/camera/image']

Publishes:
  - /parrot/veg_coverage   (std_msgs/Float32)                -> % of vegetation pixels [0..1]
  - /parrot/veg_detections (vision_msgs/Detection2DArray)    -> bounding boxes of green blobs
  - /parrot/camera/veg_mask (sensor_msgs/Image, mono8)       -> optional, if publish_debug
  - /parrot/camera/veg_annotated (sensor_msgs/Image, bgr8)   -> optional, if publish_debug

Params:
  image_topic:     string  (default '/parrot/camera/image')
  publish_debug:   bool    (default True)
  hsv_lower:       int[3]  (default [25, 40, 20])
  hsv_upper:       int[3]  (default [90, 255, 255])
  min_area_px:     int     (default 200)
  downscale:       double  (default 1.0)  # e.g., 0.5 for speed
  log_throttle_sec: double (default 1.0)  # throttle console detect logs
  log_to_file:     bool    (default False)
  log_file:        string  (default '/tmp/parrot_veg_detections.csv')
"""

import os
import csv
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge


class ParrotVeg(Node):
    def __init__(self):
        super().__init__('parrot_veg')

        # ---------------- Params ----------------
        self.declare_parameter('image_topic', '/parrot/camera/image')
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('hsv_lower', [25, 40, 20])
        self.declare_parameter('hsv_upper', [90, 255, 255])
        self.declare_parameter('min_area_px', 200)
        self.declare_parameter('downscale', 1.0)

        self.declare_parameter('log_throttle_sec', 1.0)
        self.declare_parameter('log_to_file', False)
        self.declare_parameter('log_file', '/tmp/parrot_veg_detections.csv')

        # Fetch params
        self.image_topic   = self.get_parameter('image_topic').get_parameter_value().string_value
        self.publish_debug = self.get_parameter('publish_debug').get_parameter_value().bool_value
        self.hsv_lower     = np.array(self.get_parameter('hsv_lower').get_parameter_value().integer_array_value, dtype=np.uint8)
        self.hsv_upper     = np.array(self.get_parameter('hsv_upper').get_parameter_value().integer_array_value, dtype=np.uint8)
        self.min_area_px   = int(self.get_parameter('min_area_px').get_parameter_value().integer_value)
        self.downscale     = float(self.get_parameter('downscale').get_parameter_value().double_value)

        self.log_throttle  = float(self.get_parameter('log_throttle_sec').get_parameter_value().double_value)
        self.log_to_file   = self.get_parameter('log_to_file').get_parameter_value().bool_value
        self.log_file      = self.get_parameter('log_file').get_parameter_value().string_value

        # ---------------- Pub/Sub ----------------
        qos_cam = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.image_topic, self.cb_img, qos_cam)

        self.cover_pub = self.create_publisher(Float32, '/parrot/veg_coverage', 10)
        self.det_pub   = self.create_publisher(Detection2DArray, '/parrot/veg_detections', 10)

        self.mask_pub  = None
        self.ann_pub   = None
        if self.publish_debug:
            self.mask_pub = self.create_publisher(Image, '/parrot/camera/veg_mask', 10)
            self.ann_pub  = self.create_publisher(Image, '/parrot/camera/veg_annotated', 10)

        # ---------------- Logging helpers ----------------
        self.prev_count = 0
        self.last_log_time = self.get_clock().now()

        # Optional CSV logging
        self.csv_fp = None
        self.csv_writer = None
        if self.log_to_file:
            new_file = not os.path.exists(self.log_file)
            try:
                os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
                self.csv_fp = open(self.log_file, 'a', newline='')
                self.csv_writer = csv.writer(self.csv_fp)
                if new_file:
                    self.csv_writer.writerow(
                        ['stamp_sec', 'stamp_nsec', 'frame_id', 'cx', 'cy', 'w', 'h', 'score']
                    )
                self.get_logger().info(f'Logging detections to {self.log_file}')
            except Exception as e:
                self.get_logger().error(f'Failed to open log_file "{self.log_file}": {e}')
                self.csv_fp = None
                self.csv_writer = None

        # Morphology kernel
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        self.get_logger().info(
            f'Veg detector subscribed: {self.image_topic}\n'
            f'HSV lower={self.hsv_lower.tolist()} upper={self.hsv_upper.tolist()} '
            f'min_area_px={self.min_area_px} downscale={self.downscale}'
        )

    # --------------- Core callback ----------------
    def cb_img(self, msg: Image):
        # Convert to OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        H0, W0 = frame.shape[:2]
        s = max(self.downscale, 1e-3)
        if abs(s - 1.0) > 1e-3:
            small = cv2.resize(frame, (int(W0 * s), int(H0 * s)), interpolation=cv2.INTER_AREA)
        else:
            small = frame

        # HSV threshold for vegetation
        hsv  = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # Basic cleanup
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self.kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=1)

        # Coverage metric
        total = float(mask.size) if mask.size else 1.0
        coverage = float(cv2.countNonZero(mask)) / total
        self.cover_pub.publish(Float32(data=coverage))

        # Find contours and build detections
        dets_msg = Detection2DArray()
        dets_msg.header = msg.header

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        annotated = small.copy()

        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area_px:
                continue

            x, y, w, h = cv2.boundingRect(c)

            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = float(x + w / 2.0)
            det.bbox.center.position.y = float(y + h / 2.0)
            det.bbox.size_x = float(w)
            det.bbox.size_y = float(h)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = 'vegetation'
            roi = mask[y:y + h, x:x + w]
            hyp.hypothesis.score = float(cv2.countNonZero(roi)) / float(max(1, roi.size))
            det.results.append(hyp)

            dets_msg.detections.append(det)

            if self.publish_debug:
                cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(annotated, f"veg {hyp.hypothesis.score:.2f}",
                            (x, max(0, y - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

            # CSV log (per detection)
            if self.csv_writer is not None:
                try:
                    self.csv_writer.writerow([
                        msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id,
                        int(det.bbox.center.position.x), int(det.bbox.center.position.y),
                        int(det.bbox.size_x), int(det.bbox.size_y),
                        float(hyp.hypothesis.score),
                    ])
                except Exception as e:
                    self.get_logger().warn(f'CSV write failed: {e}')

        # Publish detections
        self.det_pub.publish(dets_msg)

        # Debug image pubs
        if self.publish_debug:
            if abs(s - 1.0) > 1e-3:
                mask_pub = cv2.resize(mask, (W0, H0), interpolation=cv2.INTER_NEAREST)
                ann_pub  = cv2.resize(annotated, (W0, H0), interpolation=cv2.INTER_LINEAR)
            else:
                mask_pub, ann_pub = mask, annotated

            try:
                self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask_pub, encoding='mono8'))
                self.ann_pub.publish(self.bridge.cv2_to_imgmsg(ann_pub,  encoding='bgr8'))
            except Exception as e:
                self.get_logger().warn(f'Failed to publish debug images: {e}')

        # Throttled console logging
        self._maybe_log_detections(dets_msg)

    # --------------- Helpers ----------------
    def _maybe_log_detections(self, dets_msg: Detection2DArray):
        count = len(dets_msg.detections)
        now = self.get_clock().now()

        edge = (self.prev_count == 0 and count > 0) or (count != self.prev_count)
        periodic = (now - self.last_log_time) > Duration(seconds=self.log_throttle)

        if count > 0 and (edge or periodic):
            sample = dets_msg.detections[:3]
            brief = []
            for d in sample:
                if not d.results:
                    continue
                cx = int(d.bbox.center.position.x)
                cy = int(d.bbox.center.position.y)
                w  = int(d.bbox.size_x)
                h  = int(d.bbox.size_y)
                s  = float(d.results[0].hypothesis.score)
                brief.append(f"({cx},{cy}) {w}x{h} s={s:.2f}")
            more = "" if count <= 3 else f" (+{count-3} more)"
            self.get_logger().info(f"[DETECT] {count} veg boxes | {', '.join(brief)}{more}")
            self.last_log_time = now

        if self.prev_count > 0 and count == 0:
            self.get_logger().info("[DETECT] no vegetation boxes")

        self.prev_count = count

    # Ensure CSV file is flushed/closed
    def destroy_node(self):
        try:
            if self.csv_fp:
                self.csv_fp.flush()
                self.csv_fp.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = ParrotVeg()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
