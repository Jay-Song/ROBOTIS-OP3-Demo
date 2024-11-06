from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare parameters
    gazebo = LaunchConfiguration('gazebo')
    gazebo_robot_name = LaunchConfiguration('gazebo_robot_name')
    offset_file_path = LaunchConfiguration('offset_file_path')
    robot_file_path = LaunchConfiguration('robot_file_path')
    init_file_path = LaunchConfiguration('init_file_path')
    device_name = LaunchConfiguration('device_name')
    default_moving_time = LaunchConfiguration('default_moving_time')
    default_moving_angle = LaunchConfiguration('default_moving_angle')

    return LaunchDescription([
        # Declare arguments with default values
        DeclareLaunchArgument('gazebo', default_value='false'),
        DeclareLaunchArgument('gazebo_robot_name', default_value='robotis_op3'),
        DeclareLaunchArgument('offset_file_path', default_value='$(find op3_manager)/config/offset.yaml'),
        DeclareLaunchArgument('robot_file_path', default_value='$(find op3_manager)/config/OP3.robot'),
        DeclareLaunchArgument('init_file_path', default_value='$(find op3_manager)/config/dxl_init_OP3.yaml'),
        DeclareLaunchArgument('device_name', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('default_moving_time', default_value='0.04'),
        DeclareLaunchArgument('default_moving_angle', default_value='90'),

        # OP3 Manager Node
        Node(
            package='op3_manager',
            executable='op3_manager',
            # name='op3_manager',
            output='screen',
            parameters=[{
                'angle_unit': 30,
                'offset_file_path': offset_file_path,
                'robot_file_path': robot_file_path,
                'init_file_path': init_file_path,
                'device_name': device_name,
                '/robotis/direct_control/default_moving_time': default_moving_time,
                '/robotis/direct_control/default_moving_angle': default_moving_angle
            }]
        ),

        # OP3 Localization Node
        Node(
            package='op3_localization',
            executable='op3_localization',
            name='op3_localization',
            output='screen'
        ),

        # OP3 Read-Write Demo Node
        Node(
            package='op3_read_write_demo',
            executable='read_write',
            # name='op3_read_write',
            output='screen'
        )
    ])
