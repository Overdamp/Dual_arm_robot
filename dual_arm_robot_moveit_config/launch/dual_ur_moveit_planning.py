import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    left_prefix = LaunchConfiguration('left_prefix') 
    right_prefix = LaunchConfiguration('right_prefix') 
    left_controllers_file = LaunchConfiguration('left_controllers_file') 
    right_controllers_file = LaunchConfiguration('right_controllers_file') 
    left_planning_group = LaunchConfiguration('left_planning_group') 
    right_planning_group = LaunchConfiguration('right_planning_group')
    if_use_rtde = LaunchConfiguration('if_use_rtde') 


    left_prefix_arg = DeclareLaunchArgument(
            "left_prefix",
            default_value='"left"',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
    )
    right_prefix_arg = DeclareLaunchArgument(
            "right_prefix",
            default_value='"right"',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
    )
    left_controllers_file_arg = DeclareLaunchArgument(
            "left_controllers_file",
            default_value="left_controllers.yaml",
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
    )
    right_controllers_file_arg = DeclareLaunchArgument(
            "right_controllers_file",
            default_value="right_controllers.yaml",
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
    )
    left_planning_group_arg = DeclareLaunchArgument(
            "left_planning_group",
            default_value="left",
            description="planning group",
    )
    right_planning_group_arg = DeclareLaunchArgument(
            "right_planning_group",
            default_value="right",
            description="planning group",
    )
    if_use_rtde_arg = DeclareLaunchArgument(
            "if_use_rtde",
            default_value="false",
            description="",
    )


    ur_moveit_launch_dir = get_package_share_directory('ur_moveit_config')

    left_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_moveit_launch_dir, 'launch', '2ur_moveit.launch.py')),
        launch_arguments={'prefix': left_prefix,
                          'controllers_file': left_controllers_file,
                          'planning_group' : left_planning_group,
                          'if_use_rtde' : if_use_rtde
                          }.items())
    
    left_moveit_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('left'),
         left_moveit
      ]
    )

    right_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_moveit_launch_dir, 'launch', '2ur_moveit.launch.py')),
        launch_arguments={'prefix': right_prefix,
                          'controllers_file': right_controllers_file,
                          'planning_group' : right_planning_group,
                          'if_use_rtde' : if_use_rtde
                          }.items())
    
    right_moveit_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('right'),
         right_moveit
      ]
    )

    
    return LaunchDescription([
        left_prefix_arg,
        left_controllers_file_arg,
        left_planning_group_arg,
        right_prefix_arg,
        right_controllers_file_arg,
        right_planning_group_arg,
        if_use_rtde_arg,

        # arm control node
        left_moveit_with_namespace,
        right_moveit_with_namespace
    ])