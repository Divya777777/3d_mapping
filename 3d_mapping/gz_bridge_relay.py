#!/usr/bin/env python3
"""
Bridge relay between ROS 2 and Gazebo Harmonic.
Bridges: cmd_vel (ROS→GZ), clock + odom + tf (GZ→ROS)
"""

import subprocess
import threading
import re
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, TransformStamped
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time


class GzBridgeRelay(Node):
    def __init__(self):
        super().__init__('gz_bridge_relay')

        self._sim_time = Time()

        # ── cmd_vel: ROS → Gazebo ─────────────────────────────────────
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # ── Publishers ────────────────────────────────────────────────
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.odom_pub  = self.create_publisher(Odometry, '/odom', 10)

        tf_qos = QoSProfile(depth=100)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', tf_qos)

        # ── Gazebo readers ────────────────────────────────────────────
        self._start_reader('/world/warehouse/clock', self.parse_clock)
        self._start_reader('/odom',  self.parse_odom_msg)   # gz odom → /odom + TF
        self._start_reader('/tf',    self.parse_gz_tf)      # gz tf  → /tf (backup)

        self.get_logger().info('GZ Bridge Relay started')

    # ── Thread helpers ────────────────────────────────────────────────
    def _start_reader(self, gz_topic, parser):
        t = threading.Thread(
            target=self._gz_reader, args=(gz_topic, parser), daemon=True)
        t.start()

    def _gz_reader(self, gz_topic, parser):
        import time
        while rclpy.ok():
            try:
                proc = subprocess.Popen(
                    ['gz', 'topic', '-e', '-t', gz_topic],
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True
                )
                buffer = ''
                for line in proc.stdout:
                    buffer += line
                    # Each message ends with a blank line
                    if line.strip() == '' and buffer.strip():
                        try:
                            parser(buffer)
                        except Exception as e:
                            self.get_logger().debug(
                                f'Parser error on {gz_topic}: {e}')
                        buffer = ''
                proc.wait()
            except Exception as e:
                self.get_logger().error(f'Reader {gz_topic} crashed: {e}')
                time.sleep(1.0)

    # ── cmd_vel: ROS → Gazebo ─────────────────────────────────────────
    def cmd_vel_callback(self, msg):
        cmd = [
            'gz', 'topic', '-t', '/cmd_vel',
            '-m', 'gz.msgs.Twist',
            '-p',
            f'linear: {{x: {msg.linear.x}, y: {msg.linear.y}, z: {msg.linear.z}}}, '
            f'angular: {{x: {msg.angular.x}, y: {msg.angular.y}, z: {msg.angular.z}}}'
        ]
        try:
            subprocess.Popen(cmd,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

    # ── Clock ─────────────────────────────────────────────────────────
    def parse_clock(self, text):
        m = re.search(
            r'sim\s*\{\s*sec:\s*(\d+)\s*nsec:\s*(\d+)\s*\}', text)
        if m:
            msg = Clock()
            msg.clock.sec      = int(m.group(1))
            msg.clock.nanosec  = int(m.group(2))
            self.clock_pub.publish(msg)
            self._sim_time = msg.clock

    # ── Odometry (gz /odom → ROS /odom + odom→base_footprint TF) ─────
    def parse_odom_msg(self, text):
        """
        Gazebo odom format (note: header comes AFTER pose block):

            pose {
              position { x: ...  y: ... }
              orientation { z: ...  w: ... }
            }
            twist { ... }
            header {
              stamp { sec: N  nsec: N }
              data { key: "frame_id"       value: "odom" }
              data { key: "child_frame_id" value: "base_footprint" }
            }
        """
        pos = self._extract_xyz(text, 'position')
        rot = self._extract_xyzw_partial(text, 'orientation')
        stamp = self._extract_stamp(text)   # uses nsec

        if not pos or not rot:
            return

        t = stamp if stamp else self._sim_time

        # Publish /odom
        odom = Odometry()
        odom.header.stamp        = t
        odom.header.frame_id     = 'odom'
        odom.child_frame_id      = 'base_footprint'
        odom.pose.pose.position.x    = pos[0]
        odom.pose.pose.position.y    = pos[1]
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = rot[0]
        odom.pose.pose.orientation.y = rot[1]
        odom.pose.pose.orientation.z = rot[2]
        odom.pose.pose.orientation.w = rot[3]
        self.odom_pub.publish(odom)

        # Publish odom → base_footprint TF  ← THE MISSING LINK
        ts = TransformStamped()
        ts.header.stamp       = t
        ts.header.frame_id    = 'odom'
        ts.child_frame_id     = 'base_footprint'
        ts.transform.translation.x = pos[0]
        ts.transform.translation.y = pos[1]
        ts.transform.translation.z = 0.0
        ts.transform.rotation.x    = rot[0]
        ts.transform.rotation.y    = rot[1]
        ts.transform.rotation.z    = rot[2]
        ts.transform.rotation.w    = rot[3]

        tf_msg = TFMessage()
        tf_msg.transforms.append(ts)
        self.tf_pub.publish(tf_msg)

    # ── TF from gz /tf topic (backup / cross-check) ───────────────────
    def parse_gz_tf(self, text):
        """
        Gazebo /tf format (Pose_V, one pose per odom→base_footprint update):

            pose {
              header {
                stamp { sec: N  nsec: N }
                data { key: "frame_id"       value: "odom" }
                data { key: "child_frame_id" value: "base_footprint" }
              }
              position    { x: ...  y: ... }
              orientation { z: ...  w: ... }
            }
        """
        # Split into individual pose blocks
        blocks = re.split(r'\npose \{', text)
        for block in blocks:
            # Only process odom→base_footprint
            if 'base_footprint' not in block:
                continue

            pos   = self._extract_xyz(block, 'position')
            rot   = self._extract_xyzw_partial(block, 'orientation')
            stamp = self._extract_stamp(block)

            if not pos or not rot:
                continue

            t = stamp if stamp else self._sim_time

            ts = TransformStamped()
            ts.header.stamp       = t
            ts.header.frame_id    = 'odom'
            ts.child_frame_id     = 'base_footprint'
            ts.transform.translation.x = pos[0]
            ts.transform.translation.y = pos[1]
            ts.transform.translation.z = 0.0
            ts.transform.rotation.x    = rot[0]
            ts.transform.rotation.y    = rot[1]
            ts.transform.rotation.z    = rot[2]
            ts.transform.rotation.w    = rot[3]

            tf_msg = TFMessage()
            tf_msg.transforms.append(ts)
            self.tf_pub.publish(tf_msg)
            return  # one transform per message is enough

    # ── Regex helpers ─────────────────────────────────────────────────
    @staticmethod
    def _extract_stamp(text):
        """Extract stamp using nsec (Gazebo Harmonic uses nsec, not nanosec)."""
        m = re.search(
            r'stamp\s*\{\s*sec:\s*(\d+)\s*(?:nsec|nanosec):\s*(\d+)\s*\}',
            text)
        if m:
            t = Time()
            t.sec     = int(m.group(1))
            t.nanosec = int(m.group(2))
            return t
        # sec only (nsec=0)
        m2 = re.search(r'stamp\s*\{\s*sec:\s*(\d+)\s*\}', text)
        if m2:
            t = Time()
            t.sec     = int(m2.group(1))
            t.nanosec = 0
            return t
        return None

    @staticmethod
    def _extract_xyz(text, block_name):
        """Extract x,y,z — z is optional (2D odometry omits it)."""
        m = re.search(
            block_name +
            r'\s*\{\s*x:\s*([-\d.eE+]+)\s*y:\s*([-\d.eE+]+)'
            r'(?:\s*z:\s*([-\d.eE+]+))?\s*\}',
            text)
        if m:
            return [float(m.group(1)), float(m.group(2)),
                    float(m.group(3)) if m.group(3) else 0.0]
        return None

    @staticmethod
    def _extract_xyzw_partial(text, block_name):
        """
        Extract orientation — handles partial quaternions.
        Gazebo often omits x,y when zero and only emits z,w.
        """
        # Full xyzw
        m = re.search(
            block_name +
            r'\s*\{[^}]*x:\s*([-\d.eE+]+)[^}]*y:\s*([-\d.eE+]+)'
            r'[^}]*z:\s*([-\d.eE+]+)[^}]*w:\s*([-\d.eE+]+)[^}]*\}',
            text, re.DOTALL)
        if m:
            return [float(m.group(1)), float(m.group(2)),
                    float(m.group(3)), float(m.group(4))]

        # z and w only (x=y=0)
        m2 = re.search(
            block_name +
            r'\s*\{[^}]*z:\s*([-\d.eE+]+)[^}]*w:\s*([-\d.eE+]+)[^}]*\}',
            text, re.DOTALL)
        if m2:
            return [0.0, 0.0, float(m2.group(1)), float(m2.group(2))]

        # w only (identity-ish, z=0)
        m3 = re.search(
            block_name + r'\s*\{[^}]*w:\s*([-\d.eE+]+)[^}]*\}',
            text, re.DOTALL)
        if m3:
            return [0.0, 0.0, 0.0, float(m3.group(1))]

        # Empty block = identity
        if re.search(block_name + r'\s*\{\s*\}', text):
            return [0.0, 0.0, 0.0, 1.0]

        return None


def main(args=None):
    rclpy.init(args=args)
    node = GzBridgeRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()