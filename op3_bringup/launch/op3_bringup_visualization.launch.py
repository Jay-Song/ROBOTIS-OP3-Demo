import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
  xacro = os.path.join(
    get_package_share_directory('op3_description'),
    'urdf',
    'robotis_op3.urdf.xacro')
  p = subprocess.Popen(['xacro', xacro], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
  robot_desc, stderr = p.communicate()
  urdf = {'robot_description' : robot_desc.decode('utf-8')}

  # # 패키지 경로 설정
  op3_bringup_pkg = get_package_share_directory('op3_bringup')
  
  # Launch description 구성
  return LaunchDescription([
    # joint_state_publisher 노드 설정
    Node(
      package='joint_state_publisher_gui',
      executable='joint_state_publisher_gui',
      name='joint_state_publisher',
      parameters=[{'use_gui': True}],
      remappings=[('/joint_states', ['/robotis/present_joint_states'])]
    ),
    
    # robot_state_publisher 노드 설정
    Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      remappings=[('/joint_states', '/robotis/present_joint_states')],
      parameters=[urdf],
    ),

    # Rviz 노드 설정
    Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      arguments=['-d', os.path.join(op3_bringup_pkg, 'rviz', 'op3_bringup.rviz')],
      output='screen',
      parameters=[urdf],
    ),
  ])