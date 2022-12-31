import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription(
    [
      ExecuteProcess(
        cmd=["ros2", "run", "autorccar_ubloxf9r","ubloxf9r"], name="ublox", output='screen'
        ),
      ExecuteProcess(
        cmd=["ros2", "run", "autorccar_navigation","gnss_ins"], name="navi", output='screen'
        ),
      ExecuteProcess(
        cmd=["ros2", "run", "autorccar_path_planning","bezier_curve"], name="path", output='screen'
        ),
      ExecuteProcess(
        cmd=["ros2", "run", "autorccar_control","pure_pursuit"], name="control", output='screen'
        ),
    ]
  )
