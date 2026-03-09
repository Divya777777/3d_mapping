import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import time

class SyncTest(Node):
    def __init__(self):
        super().__init__('sync_test')
        self.create_subscription(Image, '/camera/color/image_raw', self.img_cb, 1)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 1)
        self.last_img = None
        self.last_odom = None
        
    def img_cb(self, msg):
        self.last_img = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        self.check()
        
    def odom_cb(self, msg):
        self.last_odom = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        self.check()
        
    def check(self):
        if self.last_img and self.last_odom:
            diff = self.last_img - self.last_odom
            print(f"Img - Odom delay: {diff:.3f} seconds (positive means img is newer)")

rclpy.init()
node = SyncTest()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
