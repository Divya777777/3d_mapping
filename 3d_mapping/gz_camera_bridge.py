#!/usr/bin/env python3
"""
Camera bridge: Gazebo Harmonic → ROS 2 using native gz-transport13 Python bindings.
FIX 1: Stamps messages from Gazebo message header (no ROS clock lag).
FIX 2: Waits for odom→base_footprint TF before publishing.
FIX 3: Discards stale buffered frames — only publishes frames within 1s of sim time.
"""

import os
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

import rclpy
import rclpy.time
import rclpy.duration
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from builtin_interfaces.msg import Time
from tf2_ros import Buffer, TransformListener

from gz.transport13 import Node as GzNode, SubscribeOptions
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.camera_info_pb2 import CameraInfo as GzCameraInfo


GZ_PIXEL_FORMAT_TO_ROS = {
    1: 'l8',
    2: 'l16',
    3: 'rgb8',
    4: 'rgba8',
    5: 'bgra8',
    6: 'rgb16',
    7: 'rgb32f',
    8: '32FC1',
    9: '16UC1',
}


class GzCameraBridge(Node):
    def __init__(self):
        super().__init__('gz_camera_bridge',
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 'use_sim_time',
                                 rclpy.Parameter.Type.BOOL,
                                 True
                             )
                         ])

        self.declare_parameter('gz_topic', 'camera')
        gz_topic  = self.get_parameter('gz_topic').value
        gz_prefix = f'/{gz_topic}'

        self._cached_camera_info_msg = None
        self._warmup_done = False

        # ── TF-based warmup ───────────────────────────────────────────
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_check_timer = self.create_timer(1.0, self._check_tf_ready)
        self.get_logger().info(
            'Camera bridge waiting for odom→base_footprint TF...')

        # ── ROS 2 Publishers ──────────────────────────────────────────
        self.color_pub = self.create_publisher(Image,      '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image,      '/camera/depth/image_raw', 10)
        self.info_pub  = self.create_publisher(CameraInfo, '/camera/camera_info',     10)

        # ── Frame Buffers ───────────────────────────────────────────
        self._color_bytes = None
        self._depth_bytes = None
        self._info_bytes  = None

        # ── Processing Timer (10 Hz) ────────────────────────────────
        self.create_timer(0.1, self._process_camera_data)

        # ── Gazebo Transport Subscribers ──────────────────────────────
        self.gz_node = GzNode()
        opts = SubscribeOptions()

        ok = self.gz_node.subscribe_raw(
            f'{gz_prefix}/image',
            self._on_color_image, 'gz.msgs.Image', opts)
        self.get_logger().info(f"{'✓' if ok else '✗'} Color:  {gz_prefix}/image")

        ok = self.gz_node.subscribe_raw(
            f'{gz_prefix}/depth_image',
            self._on_depth_image, 'gz.msgs.Image', opts)
        self.get_logger().info(f"{'✓' if ok else '✗'} Depth:  {gz_prefix}/depth_image")

        ok = self.gz_node.subscribe_raw(
            f'{gz_prefix}/camera_info',
            self._on_camera_info, 'gz.msgs.CameraInfo', opts)
        self.get_logger().info(f"{'✓' if ok else '✗'} Info:   {gz_prefix}/camera_info")

    # ── TF warmup ─────────────────────────────────────────────────────

    def _check_tf_ready(self):
        if self._warmup_done:
            self._tf_check_timer.cancel()
            return
        try:
            self._tf_buffer.lookup_transform(
                'odom', 'base_footprint',
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1)
            )
            self.get_logger().info(
                'TF found — waiting 2s for history depth before publishing...')
            self.create_timer(2.0, self._enable_publishing)
            self._tf_check_timer.cancel()
        except Exception:
            self.get_logger().info(
                'Waiting for odom→base_footprint TF...',
                throttle_duration_sec=3.0)

    def _enable_publishing(self):
        self._warmup_done = True
        self.get_logger().info('Camera bridge ACTIVE — publishing to ROS now')

    # ── Freshness check ───────────────────────────────────────────────

    def _is_fresh(self, stamp: Time) -> bool:
        """
        Require TF to be ready AND frame must be within 1s of current sim time.
        This discards the backlog of buffered Gazebo frames that gz-transport
        delivers all at once when the subscriber first connects.
        """
        if not self._warmup_done:
            return False

        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return False  # sim clock not yet ticking

        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        diff_sec = abs(now.nanoseconds - stamp_ns) / 1_000_000_000

        if diff_sec > 1.0:
            self.get_logger().debug(
                f'Stale frame dropped: stamp={stamp.sec}s '
                f'now={now.nanoseconds // 1_000_000_000}s '
                f'diff={diff_sec:.2f}s',
                throttle_duration_sec=2.0)
            return False
        return True

    # ── Timestamp helper ──────────────────────────────────────────────

    def _gz_stamp_to_ros(self, gz_header) -> Time:
        t = Time()
        try:
            t.sec     = gz_header.stamp.sec
            t.nanosec = gz_header.stamp.nsec
        except Exception:
            t = self.get_clock().now().to_msg()
        return t

    # ── Callbacks ─────────────────────────────────────────────────────

    def _on_color_image(self, raw_bytes, msg_info):
        self._color_bytes = raw_bytes

    def _on_depth_image(self, raw_bytes, msg_info):
        self._depth_bytes = raw_bytes

    def _on_camera_info(self, raw_bytes, msg_info):
        self._info_bytes = raw_bytes

    # ── Processor ─────────────────────────────────────────────────────

    def _process_camera_data(self):
        if not self._warmup_done:
            return

        color_buf = self._color_bytes
        self._color_bytes = None
        if color_buf:
            try:
                gz_img = GzImage()
                gz_img.ParseFromString(color_buf)
                stamp = self._gz_stamp_to_ros(gz_img.header)

                if self._is_fresh(stamp):
                    msg = Image()
                    msg.header.stamp    = stamp
                    msg.header.frame_id = 'camera_optical_frame'
                    msg.width           = gz_img.width
                    msg.height          = gz_img.height
                    msg.encoding        = GZ_PIXEL_FORMAT_TO_ROS.get(gz_img.pixel_format_type, 'rgb8')
                    msg.step            = gz_img.step
                    msg.is_bigendian    = False
                    msg.data            = gz_img.data
                    self.color_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Color parsing error: {e}', throttle_duration_sec=5.0)

        depth_buf = self._depth_bytes
        self._depth_bytes = None
        if depth_buf:
            try:
                gz_img = GzImage()
                gz_img.ParseFromString(depth_buf)
                stamp = self._gz_stamp_to_ros(gz_img.header)

                if self._is_fresh(stamp):
                    msg = Image()
                    msg.header.stamp    = stamp
                    msg.header.frame_id = 'camera_optical_frame'
                    msg.width           = gz_img.width
                    msg.height          = gz_img.height
                    msg.encoding        = GZ_PIXEL_FORMAT_TO_ROS.get(gz_img.pixel_format_type, '32FC1')
                    msg.step            = gz_img.step
                    msg.is_bigendian    = False
                    msg.data            = gz_img.data
                    self.depth_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Depth parsing error: {e}', throttle_duration_sec=5.0)

        info_buf = self._info_bytes
        self._info_bytes = None
        if info_buf:
            try:
                gz_info = GzCameraInfo()
                gz_info.ParseFromString(info_buf)
                stamp = self._gz_stamp_to_ros(gz_info.header)

                if self._is_fresh(stamp):
                    if self._cached_camera_info_msg is None:
                        msg = CameraInfo()
                        msg.header.frame_id = 'camera_optical_frame'
                        msg.width           = gz_info.width
                        msg.height          = gz_info.height

                        if gz_info.HasField('distortion'):
                            msg.distortion_model = 'plumb_bob'
                            d = gz_info.distortion
                            msg.d = [d.k[i] if i < len(d.k) else 0.0 for i in range(5)]

                        if gz_info.HasField('intrinsics'):
                            k = gz_info.intrinsics
                            if len(k.k) >= 9:
                                msg.k = list(k.k[:9])

                        if gz_info.HasField('projection'):
                            p = gz_info.projection
                            if len(p.p) >= 12:
                                msg.p = list(p.p[:12])

                        if len(gz_info.rectification_matrix) >= 9:
                            msg.r = list(gz_info.rectification_matrix[:9])
                        else:
                            msg.r = [1.0, 0.0, 0.0,
                                     0.0, 1.0, 0.0,
                                     0.0, 0.0, 1.0]

                        self._cached_camera_info_msg = msg

                    self._cached_camera_info_msg.header.stamp = stamp
                    self.info_pub.publish(self._cached_camera_info_msg)
            except Exception as e:
                self.get_logger().error(f'Info parsing error: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = GzCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()