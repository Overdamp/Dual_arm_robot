#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import yaml
import time
import os
import math
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import select
from itertools import chain  # เพิ่มเพื่อรวมช่วงดัชนี

class VRClient(Node):
    """โหนด ROS 2 สำหรับเชื่อมต่อกับเซิร์ฟเวอร์ VR และเผยแพร่ข้อมูลคอนโทรลเลอร์ขวาและซ้ายแบบสัมพัทธ์"""
    
    def __init__(self):
        super().__init__('vr_client')

        # โหลดการตั้งค่าจาก vr_config.yaml
        try:
            config_path = os.path.join(
                get_package_share_directory('ur_vr_control'),
                'config',
                'vr_config.yaml'
            )
            self.get_logger().info(f'กำลังโหลดการตั้งค่าจาก: {config_path}')
            
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
            
            if self.config is None:
                raise ValueError("ไม่สามารถโหลด vr_config.yaml: ไฟล์ว่างเปล่าหรือรูปแบบไม่ถูกต้อง")
        
        except FileNotFoundError:
            self.get_logger().error(f"❌ ไม่พบไฟล์ vr_config.yaml ที่ {config_path}")
            raise
        except yaml.YAMLError as e:
            self.get_logger().error(f"❌ ข้อผิดพลาดในการแยกวิเคราะห์ YAML ใน vr_config.yaml: {e}")
            raise
        except Exception as e:
            self.get_logger().error(f"❌ ไม่สามารถโหลด vr_config.yaml: {e}")
            raise
        
        # พารามิเตอร์การเชื่อมต่อเซิร์ฟเวอร์ VR
        try:
            self.vr_server_ip = self.config["vr_client"]["vr_server_ip"]
            self.vr_server_port = self.config["vr_client"]["vr_server_port"]
            self.reconnect_interval = self.config["vr_client"]["reconnect_interval"]
            self.socket_timeout = self.config["vr_client"]["socket_timeout"]
            self.get_logger().info(f'การตั้งค่าเซิร์ฟเวอร์ VR: ip={self.vr_server_ip}, port={self.vr_server_port}, timeout={self.socket_timeout}')
        except KeyError as e:
            self.get_logger().error(f"❌ ไม่พบคีย์ใน vr_config.yaml: {e}")
            raise

        # ผู้เผยแพร่สำหรับข้อมูลคอนโทรลเลอร์ VR (ขวาและซ้าย)
        self.right_pose_pub = self.create_publisher(Pose, 'vr_controller/right/pose', 10)
        self.right_gripper_pub = self.create_publisher(Bool, 'vr_controller/right/gripper', 10)
        self.right_move_enable_pub = self.create_publisher(Bool, 'vr_controller/right/move_enable', 10)
        
        self.left_pose_pub = self.create_publisher(Pose, 'vr_controller/left/pose', 10)
        self.left_gripper_pub = self.create_publisher(Bool, 'vr_controller/left/gripper', 10)
        self.left_move_enable_pub = self.create_publisher(Bool, 'vr_controller/left/move_enable', 10)
        
        # เริ่มต้นซ็อกเก็ต
        self.socket = None
        self.connect_socket()
        
        # ตัวจับเวลาสำหรับอ่านข้อมูล VR
        self.timer = self.create_timer(0.01, self.read_vr_data)

        # สถานะสำหรับตำแหน่งสัมพัทธ์
        self.initial_right_vr_pose = None
        self.initial_left_vr_pose = None
        self.right_move_enabled = False
        self.left_move_enabled = False
        self.last_right_move_enable_time = 0.0
        self.last_left_move_enable_time = 0.0

        # การสมัครสมาชิกสำหรับ move_enable
        self.create_subscription(
            Bool,
            'vr_controller/right/move_enable',
            self.right_move_enable_callback,
            10
        )
        self.create_subscription(
            Bool,
            'vr_controller/left/move_enable',
            self.left_move_enable_callback,
            10
        )

    def connect_socket(self):
        """สร้างการเชื่อมต่อ TCP กับเซิร์ฟเวอร์ VR"""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setblocking(False)
        try:
            self.socket.connect((self.vr_server_ip, self.vr_server_port))
            self.socket.settimeout(0.5)
            self.get_logger().info(f'✅ เชื่อมต่อกับเซิร์ฟเวอร์ VR ที่ {self.vr_server_ip}:{self.vr_server_port}')
            return True
        except socket.error as e:
            if e.errno == 115 or e.errno == 36:
                r, w, x = select.select([], [self.socket], [], self.socket_timeout)
                if w:
                    error = self.socket.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
                    if error == 0:
                        self.socket.settimeout(0.5)
                        self.get_logger().info(f'✅ เชื่อมต่อกับเซิร์ฟเวอร์ VR ที่ {self.vr_server_ip}:{self.vr_server_port}')
                        return True
                    else:
                        self.get_logger().error(f'❌ การเชื่อมต่อล้มเหลว: {error}')
                        return False
                else:
                    self.get_logger().error(f'❌ การเชื่อมต่อหมดเวลา')
                    return False
            else:
                self.get_logger().error(f'❌ ไม่สามารถเชื่อมต่อกับเซิร์ฟเวอร์ VR: {e}')
                return False

    def reconnect_socket(self):
        """พยายามเชื่อมต่อใหม่กับเซิร์ฟเวอร์ VR หากการเชื่อมต่อขาด"""
        self.get_logger().warn('⚠️ กำลังพยายามเชื่อมต่อใหม่กับเซิร์ฟเวอร์ VR...')
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        self.socket = None
        while rclpy.ok() and not self.connect_socket():
            time.sleep(self.reconnect_interval)

    def right_move_enable_callback(self, msg):
        """จัดการการเปลี่ยนแปลงสถานะ move_enable สำหรับคอนโทรลเลอร์ขวาด้วยการหน่วงเวลา"""
        current_time = time.time()
        if current_time - self.last_right_move_enable_time < 0.5:  # หน่วงเวลา 0.5 วินาที
            return
        
        if self.right_move_enabled != msg.data:
            self.right_move_enabled = msg.data
            self.last_right_move_enable_time = current_time
            if self.right_move_enabled:
                self.get_logger().info('🔄 เปิดใช้งานการเคลื่อนไหวขวา รอตำแหน่ง VR แรกเพื่อตั้งค่าตำแหน่งเริ่มต้น')
                self.initial_right_vr_pose = None
            else:
                self.get_logger().info('🔄 ปิดใช้งานการเคลื่อนไหวขวา')
                self.initial_right_vr_pose = None
                # เผยแพร่ตำแหน่งศูนย์เมื่อปิดใช้งาน
                pose = Pose()
                pose.position.x = 0.0
                pose.position.y = 0.0
                pose.position.z = 0.0
                pose.orientation.w = 1.0
                self.right_pose_pub.publish(pose)

    def left_move_enable_callback(self, msg):
        """จัดการการเปลี่ยนแปลงสถานะ move_enable สำหรับคอนโทรลเลอร์ซ้ายด้วยการหน่วงเวลา"""
        current_time = time.time()
        if current_time - self.last_left_move_enable_time < 0.5:  # หน่วงเวลา 0.5 วินาที
            return
        
        if self.left_move_enabled != msg.data:
            self.left_move_enabled = msg.data
            self.last_left_move_enable_time = current_time
            if self.left_move_enabled:
                self.get_logger().info('🔄 เปิดใช้งานการเคลื่อนไหวซ้าย รอตำแหน่ง VR แรกเพื่อตั้งค่าตำแหน่งเริ่มต้น')
                self.initial_left_vr_pose = None
            else:
                self.get_logger().info('🔄 ปิดใช้งานการเคลื่อนไหวซ้าย')
                self.initial_left_vr_pose = None
                # เผยแพร่ตำแหน่งศูนย์เมื่อปิดใช้งาน
                pose = Pose()
                pose.position.x = 0.0
                pose.position.y = 0.0
                pose.position.z = 0.0
                pose.orientation.w = 1.0
                self.left_pose_pub.publish(pose)

    def read_vr_data(self):
        """อ่านข้อมูลจากเซิร์ฟเวอร์ VR และเผยแพร่ตำแหน่งสัมพัทธ์ไปยังหัวข้อ ROS 2 สำหรับทั้งสองคอนโทรลเลอร์"""
        if not self.socket:
            self.reconnect_socket()
            return
        
        try:
            r, _, _ = select.select([self.socket], [], [], 0.1)
            if not r:
                return
            
            data = self.socket.recv(4096).decode('utf-8', errors='ignore')
            if not data:
                self.get_logger().warn('⚠️ การเชื่อมต่อถูกปิดโดยเซิร์ฟเวอร์ VR')
                self.reconnect_socket()
                return
            
            self.get_logger().debug(f'ข้อมูลดิบ: {repr(data)}')
            
            lines = data.splitlines()
            for line in lines:
                line = line.strip().replace('\r', '')
                if not line:
                    continue
                
                try:
                    values = [v.strip() for v in line.split(',')]
                    self.get_logger().debug(f'ค่าที่แยกวิเคราะห์: {values}, จำนวน={len(values)}')
                    
                    if len(values) != 18:
                        self.get_logger().warn(f'⚠️ ข้อมูล VR ไม่ถูกต้อง: {line}, คาดหวัง 18 ค่า, ได้ {len(values)}')
                        continue
                    
                    # ตรวจสอบค่าลอยตัวสำหรับทั้งสองคอนโทรลเลอร์
                    float_indices = list(chain(range(7), range(9, 16)))  # ดัชนีสำหรับ x, y, z, qx, qy, qz, qw สำหรับทั้งสอง
                    for i in float_indices:
                        try:
                            float(values[i])
                        except ValueError:
                            self.get_logger().warn(f'⚠️ ค่าลอยตัวไม่ถูกต้องในข้อมูล VR: {values[i]} ที่ดัชนี {i}')
                            continue
                    
                    # แยกวิเคราะห์ข้อมูลคอนโทรลเลอร์ขวา (9 ค่าแรก)
                    right_raw_pose = Pose()
                    right_raw_pose.position.x = float(values[0])
                    right_raw_pose.position.y = float(values[1])
                    right_raw_pose.position.z = float(values[2])
                    right_raw_pose.orientation.x = float(values[3])
                    right_raw_pose.orientation.y = float(values[4])
                    right_raw_pose.orientation.z = float(values[5])
                    right_raw_pose.orientation.w = float(values[6])
                    
                    right_gripper = Bool()
                    try:
                        right_gripper.data = bool(int(values[7]))
                    except ValueError:
                        self.get_logger().warn(f'⚠️ ค่า gripper ขวาไม่ถูกต้อง: {values[7]}')
                        continue
                    
                    right_move_enable = Bool()
                    try:
                        right_move_enable.data = bool(int(values[8]))
                    except ValueError:
                        self.get_logger().warn(f'⚠️ ค่า move_enable ขวาไม่ถูกต้อง: {values[8]}')
                        continue
                    
                    # แยกวิเคราะห์ข้อมูลคอนโทรลเลอร์ซ้าย (9 ค่าถัดไป)
                    left_raw_pose = Pose()
                    left_raw_pose.position.x = float(values[9])
                    left_raw_pose.position.y = float(values[10])
                    left_raw_pose.position.z = float(values[11])
                    left_raw_pose.orientation.x = float(values[12])
                    left_raw_pose.orientation.y = float(values[13])
                    left_raw_pose.orientation.z = float(values[14])
                    left_raw_pose.orientation.w = float(values[15])
                    
                    left_gripper = Bool()
                    try:
                        left_gripper.data = bool(int(values[16]))
                    except ValueError:
                        self.get_logger().warn(f'⚠️ ค่า gripper ซ้ายไม่ถูกต้อง: {values[16]}')
                        continue
                    
                    left_move_enable = Bool()
                    try:
                        left_move_enable.data = bool(int(values[17]))
                    except ValueError:
                        self.get_logger().warn(f'⚠️ ค่า move_enable ซ้ายไม่ถูกต้อง: {values[17]}')
                        continue
                    
                    # จัดการตำแหน่งสัมพัทธ์สำหรับคอนโทรลเลอร์ขวา
                    right_pose = Pose()
                    if right_move_enable.data:
                        if self.initial_right_vr_pose is None:
                            self.initial_right_vr_pose = right_raw_pose
                            self.get_logger().info(
                                f'ตั้งค่าตำแหน่ง VR เริ่มต้นขวา: x={right_raw_pose.position.x:.5f}, y={right_raw_pose.position.y:.5f}, z={right_raw_pose.position.z:.5f}, '
                                f'qx={right_raw_pose.orientation.x:.5f}, qy={right_raw_pose.orientation.y:.5f}, qz={right_raw_pose.orientation.z:.5f}, qw={right_raw_pose.orientation.w:.5f}'
                            )
                            right_pose.position.x = 0.0
                            right_pose.position.y = 0.0
                            right_pose.position.z = 0.0
                            right_pose.orientation.w = 1.0
                        else:
                            right_pose.position.x = right_raw_pose.position.x - self.initial_right_vr_pose.position.x
                            right_pose.position.y = right_raw_pose.position.y - self.initial_right_vr_pose.position.y
                            right_pose.position.z = right_raw_pose.position.z - self.initial_right_vr_pose.position.z
                            # คำนวณควอเทอร์เนียนสัมพัทธ์
                            q_current = [right_raw_pose.orientation.x, right_raw_pose.orientation.y, right_raw_pose.orientation.z, right_raw_pose.orientation.w]
                            q_initial = [self.initial_right_vr_pose.orientation.x, self.initial_right_vr_pose.orientation.y,
                                        self.initial_right_vr_pose.orientation.z, self.initial_right_vr_pose.orientation.w]
                            if any(not math.isfinite(v) for v in q_current + q_initial):
                                self.get_logger().warn(f'⚠️ ควอเทอร์เนียนขวาไม่ถูกต้อง: q_current={q_current}, q_initial={q_initial}')
                                right_pose.orientation.w = 1.0
                            else:
                                q_delta = quaternion_multiply(q_current, quaternion_inverse(q_initial))
                                right_pose.orientation.x = q_delta[0]
                                right_pose.orientation.y = q_delta[1]
                                right_pose.orientation.z = q_delta[2]
                                right_pose.orientation.w = q_delta[3]
                    else:
                        right_pose.position.x = 0.0
                        right_pose.position.y = 0.0
                        right_pose.position.z = 0.0
                        right_pose.orientation.w = 1.0
                    
                    # จัดการตำแหน่งสัมพัทธ์สำหรับคอนโทรลเลอร์ซ้าย
                    left_pose = Pose()
                    if left_move_enable.data:
                        if self.initial_left_vr_pose is None:
                            self.initial_left_vr_pose = left_raw_pose
                            self.get_logger().info(
                                f'ตั้งค่าตำแหน่ง VR เริ่มต้นซ้าย: x={left_raw_pose.position.x:.5f}, y={left_raw_pose.position.y:.5f}, z={left_raw_pose.position.z:.5f}, '
                                f'qx={left_raw_pose.orientation.x:.5f}, qy={left_raw_pose.orientation.y:.5f}, qz={left_raw_pose.orientation.z:.5f}, qw={left_raw_pose.orientation.w:.5f}'
                            )
                            left_pose.position.x = 0.0
                            left_pose.position.y = 0.0
                            left_pose.position.z = 0.0
                            left_pose.orientation.w = 1.0
                        else:
                            left_pose.position.x = left_raw_pose.position.x - self.initial_left_vr_pose.position.x
                            left_pose.position.y = left_raw_pose.position.y - self.initial_left_vr_pose.position.y
                            left_pose.position.z = left_raw_pose.position.z - self.initial_left_vr_pose.position.z
                            # คำนวณควอเทอร์เนียนสัมพัทธ์
                            q_current = [left_raw_pose.orientation.x, left_raw_pose.orientation.y, left_raw_pose.orientation.z, left_raw_pose.orientation.w]
                            q_initial = [self.initial_left_vr_pose.orientation.x, self.initial_left_vr_pose.orientation.y,
                                        self.initial_left_vr_pose.orientation.z, self.initial_left_vr_pose.orientation.w]
                            if any(not math.isfinite(v) for v in q_current + q_initial):
                                self.get_logger().warn(f'⚠️ ควอเทอร์เนียนซ้ายไม่ถูกต้อง: q_current={q_current}, q_initial={q_initial}')
                                left_pose.orientation.w = 1.0
                            else:
                                q_delta = quaternion_multiply(q_current, quaternion_inverse(q_initial))
                                left_pose.orientation.x = q_delta[0]
                                left_pose.orientation.y = q_delta[1]
                                left_pose.orientation.z = q_delta[2]
                                left_pose.orientation.w = q_delta[3]
                    else:
                        left_pose.position.x = 0.0
                        left_pose.position.y = 0.0
                        left_pose.position.z = 0.0
                        left_pose.orientation.w = 1.0
                    
                    # เผยแพร่ข้อมูลสำหรับคอนโทรลเลอร์ขวา
                    self.right_pose_pub.publish(right_pose)
                    self.right_gripper_pub.publish(right_gripper)
                    self.right_move_enable_pub.publish(right_move_enable)
                    
                    self.get_logger().info(
                        f'เผยแพร่ข้อมูล VR ขวา: ตำแหน่ง=[{right_pose.position.x:.5f}, {right_pose.position.y:.5f}, {right_pose.position.z:.5f}], '
                        f'qx={right_pose.orientation.x:.5f}, qy={right_pose.orientation.y:.5f}, qz={right_pose.orientation.z:.5f}, qw={right_pose.orientation.w:.5f}, '
                        f'gripper={right_gripper.data}, move_enable={right_move_enable.data}'
                    )
                    
                    # เผยแพร่ข้อมูลสำหรับคอนโทรลเลอร์ซ้าย
                    self.left_pose_pub.publish(left_pose)
                    self.left_gripper_pub.publish(left_gripper)
                    self.left_move_enable_pub.publish(left_move_enable)
                    
                    self.get_logger().info(
                        f'เผยแพร่ข้อมูล VR ซ้าย: ตำแหน่ง=[{left_pose.position.x:.5f}, {left_pose.position.y:.5f}, {left_pose.position.z:.5f}], '
                        f'qx={left_pose.orientation.x:.5f}, qy={left_pose.orientation.y:.5f}, qz={left_pose.orientation.z:.5f}, qw={left_pose.orientation.w:.5f}, '
                        f'gripper={left_gripper.data}, move_enable={left_move_enable.data}'
                    )
                
                except Exception as e:
                    self.get_logger().error(f'❌ ไม่สามารถแยกวิเคราะห์บรรทัด: {line}, ข้อผิดพลาด: {e}')
                    continue
        
        except Exception as e:
            self.get_logger().error(f'❌ ไม่สามารถอ่านข้อมูล VR: {e}')
            self.reconnect_socket()

    def destroy_node(self):
        """ทำความสะอาดทรัพยากรเมื่อปิดตัว"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        self.get_logger().info('❎ ตัดการเชื่อมต่อจากเซิร์ฟเวอร์ VR')
        super().destroy_node()

# ฟังก์ชันยูทิลิตี้ควอเทอร์เนียน
def quaternion_multiply(q1, q2):
    """คูณสองควอเทอร์เนียน"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ]

def quaternion_inverse(q):
    """คำนวณผกผันของควอเทอร์เนียน"""
    x, y, z, w = q
    norm_sq = x*x + y*y + z*z + w*w
    if norm_sq == 0:
        return [0.0, 0.0, 0.0, 1.0]
    return [-x/norm_sq, -y/norm_sq, -z/norm_sq, w/norm_sq]

def main(args=None):
    rclpy.init(args=args)
    node = VRClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('โหนดถูกขัดจังหวะโดยผู้ใช้')
    except Exception as e:
        node.get_logger().error(f'ข้อผิดพลาดที่ไม่คาดคิด: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()