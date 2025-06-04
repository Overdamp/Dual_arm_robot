import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    # ประกาศตัวแปร launch arguments
    left_prefix = LaunchConfiguration('left_prefix')
    right_prefix = LaunchConfiguration('right_prefix')
    left_robot_ip = LaunchConfiguration('left_robot_ip')
    right_robot_ip = LaunchConfiguration('right_robot_ip')
    left_controllers_file = LaunchConfiguration('left_controllers_file')
    right_controllers_file = LaunchConfiguration('right_controllers_file')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    ur_type = LaunchConfiguration('ur_type')
    robot_name = LaunchConfiguration('robot_name')

    # กำหนดค่าเริ่มต้น
    left_prefix_arg = DeclareLaunchArgument(
        'left_prefix',
        default_value='left_',
        description='คำนำหน้าชื่อข้อต่อสำหรับหุ่นยนต์ด้านซ้าย'
    )
    right_prefix_arg = DeclareLaunchArgument(
        'right_prefix',
        default_value='right_',
        description='คำนำหน้าชื่อข้อต่อสำหรับหุ่นยนต์ด้านขวา'
    )
    left_robot_ip_arg = DeclareLaunchArgument(
        'left_robot_ip',
        default_value='192.168.56.101',
        description='IP address ของหุ่นยนต์ด้านซ้าย'
    )
    right_robot_ip_arg = DeclareLaunchArgument(
        'right_robot_ip',
        default_value='192.168.56.102',
        description='IP address ของหุ่นยนต์ด้านขวา'
    )
    left_controllers_file_arg = DeclareLaunchArgument(
        'left_controllers_file',
        default_value='controllers.yaml',
        description='ไฟล์ตั้งค่าคอนโทรลเลอร์สำหรับหุ่นยนต์ด้านซ้าย'
    )
    right_controllers_file_arg = DeclareLaunchArgument(
        'right_controllers_file',
        default_value='controllers.yaml',
        description='ไฟล์ตั้งค่าคอนโทรลเลอร์สำหรับหุ่นยนต์ด้านขวา'
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='ใช้ fake hardware สำหรับการทดสอบ'
    )
    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur3e',
        description='ประเภทของหุ่นยนต์ UR (เช่น ur3e)'
    )
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='ur',
        description='ชื่อหุ่นยนต์ที่ใช้ใน SRDF'
    )

    ur_driver_pkg = get_package_share_directory('ur_robot_driver')
    ur_moveit_pkg = get_package_share_directory('ur_moveit_config')

    # หุ่นยนต์ด้านซ้าย
    left_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_driver_pkg, 'launch', 'ur_control.launch.py')),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': left_robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'launch_rviz': 'false',
            'controller_config_file': left_controllers_file,
            'prefix': left_prefix,
            'robot_name': ' '
        }.items()
    )

    left_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_moveit_pkg, 'launch', 'ur_moveit.launch.py')),
        launch_arguments={
            'ur_type': ur_type,
            'prefix': left_prefix,
            'launch_rviz': 'true',
            'use_fake_hardware': use_fake_hardware,
            'moveit_config_file': 'ur.srdf.xacro',
            'name': ' '
        }.items()
    )

    left_group = GroupAction(
        actions=[
            PushRosNamespace('left'),
            left_driver,
            left_moveit
        ]
    )

    # หุ่นยนต์ด้านขวา
    right_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_driver_pkg, 'launch', 'ur_control.launch.py')),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': right_robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'launch_rviz': 'false',
            'controller_config_file': right_controllers_file,
            'prefix': right_prefix,
            'robot_name': robot_name
        }.items()
    )

    right_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_moveit_pkg, 'launch', 'ur_moveit.launch.py')),
        launch_arguments={
            'ur_type': ur_type,
            'prefix': right_prefix,
            'launch_rviz': 'false',
            'use_fake_hardware': use_fake_hardware,
            'moveit_config_file': 'ur.srdf.xacro',
            'name': robot_name
        }.items()
    )

    right_group = GroupAction(
        actions=[
            PushRosNamespace('right'),
            right_driver,
            right_moveit
        ]
    )

    return LaunchDescription([
        left_prefix_arg,
        right_prefix_arg,
        left_robot_ip_arg,
        right_robot_ip_arg,
        left_controllers_file_arg,
        right_controllers_file_arg,
        use_fake_hardware_arg,
        ur_type_arg,
        robot_name_arg,
        left_group,
        right_group
    ])