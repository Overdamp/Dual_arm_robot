#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socket
import cv2
import struct

class ImageStreamServer(Node):
    def __init__(self):
        super().__init__('image_stream_server')

        # Parameters
        self.declare_parameter('host', '192.168.')  # Listen on all interfaces
        self.declare_parameter('port', 8888)
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value

        self.bridge = CvBridge()

        # TCP Server setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((self.host, self.port))
        self.sock.listen(1)
        self.get_logger().info(f"Waiting for Unity client on {self.host}:{self.port}...")

        self.conn, self.addr = self.sock.accept()
        self.get_logger().info(f"Unity client connected from {self.addr}")

        # Subscribe to Realsense color topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Compress image as JPEG
            success, jpeg = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not success:
                self.get_logger().warn("Failed to encode image")
                return

            data = jpeg.tobytes()
            length = struct.pack('>I', len(data))  # 4-byte big-endian size prefix

            # Send image length + data
            self.conn.sendall(length + data)

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")
            self.conn.close()
            self.get_logger().info("Client disconnected. Waiting for reconnect...")
            self.conn, self.addr = self.sock.accept()
            self.get_logger().info(f"New client connected from {self.addr}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageStreamServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
