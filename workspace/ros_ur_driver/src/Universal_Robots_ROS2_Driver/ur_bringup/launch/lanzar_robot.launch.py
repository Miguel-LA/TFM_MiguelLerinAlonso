import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
use_sim_time = False
 
def generate_launch_description():
 
    pkg = get_package_share_directory('ur_robot_driver')
    pkg2 = get_package_share_directory('ur_moveit_config')
    pkg3 = get_package_share_directory('ur_bringup')
 
    control = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg,'launch','ur_control.launch.py'
                )]), launch_arguments={'ur_type': 'ur10','use_fake_hardware': 'false','launch_rviz': 'false','robot_ip': '192.168.0.9', 'initial_joint_controller': 'joint_trajectory_controller'}.items()
    )
 
    # Nav2
 
    moveit = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg2,'launch','ur_moveit.launch.py'
                )]), launch_arguments={'ur_type': 'ur10','use_fake_hardware': 'false','launch_rviz': 'true','robot_ip': '192.168.0.9'}.items()
    )
 
    trayectoria = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg3,'launch','test_joint_trajectory_controller.launch.py'
                )])
    )

    return LaunchDescription([
        control,
        moveit,
        #trayectoria,
    ])