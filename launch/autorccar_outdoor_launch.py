from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='autorccar_ubloxf9r',
      executable='ubloxf9r',
      name='sensor'),
    Node(
      package='autorccar_navigation',
      executable='gnss_ins',
      name='navigation'),
    Node(
      package='autorccar_path_planning',
      executable='bezier_curve',
      name='path_planning'),
    Node(
      package='autorccar_control',
      executable='pure_pursuit',
      name='control')
  ])
