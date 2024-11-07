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
  # op3_description_pkg = get_package_share_directory('op3_description')
  op3_bringup_pkg = get_package_share_directory('op3_bringup')
  
  # # URDF 파일 경로
  # robot_description_content = Command(
  #   ['xacro ', os.path.join(op3_description_pkg, 'urdf', 'robotis_op3.urdf.xacro')]
  # )
  
  # print('robot_description_content:' + robot_description_content)

  # Launch description 구성
  return LaunchDescription([
    # # 로봇 설명 파라미터 설정
    # DeclareLaunchArgument(
    #   'robot_description',
    #   default_value=robot_description_content,
    #   description='Robot description in URDF format'
    # ),

    # joint_state_publisher 노드 설정
    Node(
      package='joint_state_publisher',
      executable='joint_state_publisher',
      name='joint_state_publisher',
      parameters=[{'use_gui': True}, urdf],
      remappings=[('/source_list', '/robotis/present_joint_states')],
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
      #arguments=['-d', os.path.join(op3_bringup_pkg, 'rviz', 'op3_bringup.rviz')],
      output='screen',
      parameters=[urdf],
    ),
  ])