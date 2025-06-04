#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import socket

class ImageStreamer(Node):
    def __init__(self):
        super().__init__('image_streamer')
        self.bridge = CvBridge()
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('10.9.163.23', 5555))  # แทนที่ด้วย IP ของเครื่อง VR
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        _, jpeg = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        
        data = jpeg.tobytes()
        size = len(data).to_bytes(4, byteorder='big')  # ส่งขนาดก่อน
        try:
            self.sock.sendall(size + data)
        except:
            self.get_logger().warn('Lost connection to VR client')

def main(args=None):
    rclpy.init(args=args)
    node = ImageStreamer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
