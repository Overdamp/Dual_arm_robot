from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
import os

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("dual_ur_robotiq", package_name="dual_arm_robot_moveit_config")
        .trajectory_execution(file_path=os.path.join("config", "moveit_controllers.yaml"))
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)
