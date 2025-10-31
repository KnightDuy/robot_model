from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Khai báo biến cấu hình use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Cho phép người dùng truyền tham số này qua CLI hoặc file launch khác
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Đường dẫn tới file xacro
    xacro_file = f"{get_package_share_directory('robot_model')}/description/robot.urdf.xacro"
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # Node robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    # Trả về LaunchDescription gồm cả phần khai báo biến và node
    return LaunchDescription([
        declare_use_sim_time,
        rsp,
    ])
