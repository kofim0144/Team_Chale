#!/usr/bin/env python3
"""
Fruit Detector Node
-------------------
Subscribes to RGB-D camera topics, performs HSV color segmentation,
estimates 3D position via the depth image, transforms the point from
the camera optical frame into the manipulator base frame, and publishes:
    /fruit/pose        geometry_msgs/PoseStamped   (in base frame)
    /fruit/diameter    std_msgs/Float32            (estimated diameter in m)
    /fruit/markers     visualization_msgs/MarkerArray   (for RViz)
    /fruit/debug_image sensor_msgs/Image           (detector overlay)

Assumes RGB and depth images are spatially aligned (typical for the
Gazebo rgbd_camera plugin). If you split them into two sensors, you must
use the depth camera's intrinsics + frame instead.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

from cv_bridge import CvBridge
import cv2
import numpy as np

import message_filters
import tf2_ros
from tf2_geometry_msgs import do_transform_point


class FruitDetector(Node):
    def __init__(self):
        super().__init__('fruit_detector')

        # -------- Parameters --------
        self.declare_parameter('rgb_topic',         '/camera/color/image_raw')
        self.declare_parameter('depth_topic',       '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('target_frame',      'base_link')   # manipulator base
        self.declare_parameter('min_area_px',       300)
        self.declare_parameter('max_depth_m',       2.0)
        # Two HSV ranges because red wraps around H=0/180. Tune for your fruit.
        self.declare_parameter('hsv_lower1', [0,   120, 70])
        self.declare_parameter('hsv_upper1', [10,  255, 255])
        self.declare_parameter('hsv_lower2', [170, 120, 70])
        self.declare_parameter('hsv_upper2', [180, 255, 255])

        self.target_frame = self.get_parameter('target_frame').value
        self.min_area     = self.get_parameter('min_area_px').value
        self.max_depth    = self.get_parameter('max_depth_m').value

        self.bridge = CvBridge()
        self.K = None            # camera intrinsic matrix
        self.cam_frame = None    # camera optical frame (from CameraInfo header)

        # -------- TF --------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -------- Subscriptions --------
        # Gazebo sensor plugins usually publish with BEST_EFFORT reliability.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)

        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.info_cb, 10)

        rgb_sub = message_filters.Subscriber(
            self, Image, self.get_parameter('rgb_topic').value, qos_profile=qos)
        depth_sub = message_filters.Subscriber(
            self, Image, self.get_parameter('depth_topic').value, qos_profile=qos)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.rgbd_cb)

        # -------- Publishers --------
        self.pose_pub   = self.create_publisher(PoseStamped, '/fruit/pose', 10)
        self.size_pub   = self.create_publisher(Float32,     '/fruit/diameter', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/fruit/markers', 10)
        self.debug_pub  = self.create_publisher(Image,       '/fruit/debug_image', 1)

        self.get_logger().info('Fruit detector ready. Waiting for CameraInfo...')

    # -------------------------------------------------------------------
    def info_cb(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.cam_frame = msg.header.frame_id
            self.get_logger().info(
                f'Got intrinsics. fx={self.K[0,0]:.1f} fy={self.K[1,1]:.1f} '
                f'cx={self.K[0,2]:.1f} cy={self.K[1,2]:.1f} frame={self.cam_frame}')

    # -------------------------------------------------------------------
    def rgbd_cb(self, rgb_msg: Image, depth_msg: Image):
        if self.K is None:
            return

        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'cv_bridge: {e}')
            return

        # Normalize depth to meters. 16UC1 => mm, 32FC1 => m (Gazebo default).
        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) / 1000.0
        else:
            depth_m = depth.astype(np.float32)

        # -------- HSV color mask --------
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        l1 = np.array(self.get_parameter('hsv_lower1').value, dtype=np.uint8)
        u1 = np.array(self.get_parameter('hsv_upper1').value, dtype=np.uint8)
        l2 = np.array(self.get_parameter('hsv_lower2').value, dtype=np.uint8)
        u2 = np.array(self.get_parameter('hsv_upper2').value, dtype=np.uint8)
        mask = cv2.inRange(hsv, l1, u1) | cv2.inRange(hsv, l2, u2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) >= self.min_area]
        if not contours:
            self._publish_debug(rgb, None, None)
            return

        # Pick the biggest blob (swap for nearest-in-depth if you prefer).
        c = max(contours, key=cv2.contourArea)
        (u, v), radius_px = cv2.minEnclosingCircle(c)
        u_i, v_i = int(u), int(v)

        # Sample depth in a small window; reject zeros and out-of-range.
        h, w = depth_m.shape
        window = depth_m[max(0, v_i-3):min(h, v_i+4),
                         max(0, u_i-3):min(w, u_i+4)]
        valid = window[(window > 0.05) & (window < self.max_depth)]
        if valid.size == 0:
            return
        z = float(np.median(valid))

        # -------- Back-project pixel to 3D (camera optical frame) --------
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        X = (u - cx) * z / fx
        Y = (v - cy) * z / fy
        Z = z

        # Physical diameter (m) from pinhole geometry.
        diameter_m = 2.0 * radius_px * z / fx

        # -------- Transform into manipulator base frame --------
        pt_cam = PointStamped()
        pt_cam.header = rgb_msg.header
        pt_cam.header.frame_id = self.cam_frame
        pt_cam.point.x, pt_cam.point.y, pt_cam.point.z = X, Y, Z

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, self.cam_frame,
                rclpy.time.Time(),  # latest available
                timeout=rclpy.duration.Duration(seconds=0.2))
            pt_base = do_transform_point(pt_cam, tf)
        except Exception as e:
            self.get_logger().warn(
                f'TF {self.cam_frame} -> {self.target_frame} failed: {e}')
            return

        # -------- Publish results --------
        pose = PoseStamped()
        pose.header.stamp    = rgb_msg.header.stamp
        pose.header.frame_id = self.target_frame
        pose.pose.position   = pt_base.point
        pose.pose.orientation.w = 1.0   # let grasp planner set orientation
        self.pose_pub.publish(pose)

        self.size_pub.publish(Float32(data=float(diameter_m)))
        self._publish_marker(pose, diameter_m)
        self._publish_debug(rgb, c, (u_i, v_i))

        self.get_logger().info(
            f'Fruit @ {self.target_frame}: '
            f'x={pt_base.point.x:+.2f} y={pt_base.point.y:+.2f} '
            f'z={pt_base.point.z:+.2f}  d={diameter_m*100:.1f} cm')

    # -------------------------------------------------------------------
    def _publish_marker(self, pose: PoseStamped, diameter: float):
        arr = MarkerArray()
        m = Marker()
        m.header = pose.header
        m.ns, m.id = 'fruit', 0
        m.type, m.action = Marker.SPHERE, Marker.ADD
        m.pose = pose.pose
        m.scale.x = m.scale.y = m.scale.z = max(0.02, float(diameter))
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.8
        m.lifetime.sec = 1
        arr.markers.append(m)
        self.marker_pub.publish(arr)

    def _publish_debug(self, rgb, contour, center):
        img = rgb.copy()
        if contour is not None:
            cv2.drawContours(img, [contour], -1, (0, 255, 0), 2)
            if center is not None:
                cv2.circle(img, center, 5, (0, 0, 255), -1)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding='bgr8'))


def main():
    rclpy.init()
    node = FruitDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
