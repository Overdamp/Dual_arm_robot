from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace, Node

def generate_launch_description():
    # Declare robot IP arguments
    left_robot_ip_arg = DeclareLaunchArgument(
        "left_robot_ip", default_value="192.168.3.212",
        description="IP of the left UR3e robot"
    )
    right_robot_ip_arg = DeclareLaunchArgument(
        "right_robot_ip", default_value="192.168.3.213",
        description="IP of the right UR3e robot"
    )

    # Define UR control launch path
    ur_control_launch = PathJoinSubstitution([
        FindPackageShare("ur_robot_driver"),
        "launch",
        "ur_control.launch.py"
    ])

    # Controller configuration file
    controller_config = PathJoinSubstitution([
        FindPackageShare("dual_arm_robot_moveit_config"),
        "config",
        "controllers.yaml"
    ])

    # Left robot group
    left_group = GroupAction([
        PushRosNamespace("left_ur"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_control_launch),
            launch_arguments={
                "ur_type": "ur3e",
                "robot_ip": LaunchConfiguration("left_robot_ip"),
                "tf_prefix": "left_",
                "reverse_port": "50001",
                "script_sender_port": "50002",
                "script_command_port": "50003",
                "trajectory_port": "50004",
                "controller_manager_name": "/left_ur/controller_manager",
                "controllers_file": controller_config
            }.items()
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", "/left_ur/controller_manager"
            ],
            namespace="left_ur",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_trajectory_controller",
                "--controller-manager", "/left_ur/controller_manager",
                "--inactive"
            ],
            namespace="left_ur",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "scaled_joint_trajectory_controller",
                "--controller-manager", "/left_ur/controller_manager"
            ],
            namespace="left_ur",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "io_and_status_controller",
                "--controller-manager", "/left_ur/controller_manager"
            ],
            namespace="left_ur",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "speed_scaling_state_broadcaster",
                "--controller-manager", "/left_ur/controller_manager"
            ],
            namespace="left_ur",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "force_torque_sensor_broadcaster",
                "--controller-manager", "/left_ur/controller_manager"
            ],
            namespace="left_ur",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "tcp_pose_broadcaster",
                "--controller-manager", "/left_ur/controller_manager"
            ],
            namespace="left_ur",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "ur_configuration_controller",
                "--controller-manager", "/left_ur/controller_manager"
            ],
            namespace="left_ur",
        ),
    ])

    # Right robot group
    right_group = GroupAction([
        PushRosNamespace("right_ur"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_control_launch),
            launch_arguments={
                "ur_type": "ur3e",
                "robot_ip": LaunchConfiguration("right_robot_ip"),
                "tf_prefix": "right_",
                "reverse_port": "51001",
                "script_sender_port": "51002",
                "script_command_port": "51003",
                "trajectory_port": "51004",
                "controller_manager_name": "/right_ur/controller_manager",
                "controllers_file": controller_config
            }.items()
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "right_ur_joint_state_broadcaster",
                "--controller-manager", "/right_ur/controller_manager"
            ],
            namespace="right_ur",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "right_ur_joint_trajectory_controller",
                "--controller-manager", "/right_ur/controller_manager",
                "--inactive"
            ],
            namespace="right_ur",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "right_ur_scaled_joint_trajectory_controller",
                "--controller-manager", "/right_ur/controller_manager"
            ],
            namespace="right_ur",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "right_ur_io_and_status_controller",
                "--controller-manager", "/right_ur/controller_manager"
            ],
            namespace="right_ur",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "right_ur_speed_scaling_state_broadcaster",
                "--controller-manager", "/right_ur/controller_manager"
            ],
            namespace="right_ur",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "right_ur_force_torque_sensor_broadcaster",
                "--controller-manager", "/right_ur/controller_manager"
            ],
            namespace="right_ur",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "right_ur_tcp_pose_broadcaster",
                "--controller-manager", "/right_ur/controller_manager"
            ],
            namespace="right_ur",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "right_ur_ur_configuration_controller",
                "--controller-manager", "/right_ur/controller_manager"
            ],
            namespace="right_ur",
        ),
    ])

    return LaunchDescription([
        left_robot_ip_arg,
        right_robot_ip_arg,
        left_group,
        right_group,
    ])