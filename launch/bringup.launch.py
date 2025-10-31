from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():
    # ==== 🔹 Tham số mô phỏng (sim time) ====
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # ==== 🔹 Đường dẫn tới gói robot_model ====
    pkg_path = get_package_share_directory('robot_model')

    # ==== 🔹 Load URDF (xacro) ====
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # Node robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    # Node joint_state_publisher_gui
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # ==== 🔹 Load world file ====
    world_path = os.path.join(pkg_path, 'worlds', 'my_world.sdf')

    # ==== 🔹 Khởi động Ignition Gazebo ====
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_path],
        output='screen'
    )

    # ==== 🔹 Spawn robot sau khi world load (delay 7s) ====
    spawn_robot = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-name', 'my_robot',
                    '-topic', 'robot_description',
                    '-x', '0', '-y', '0', '-z', '0.03'
                ],
                output='screen'
            )
        ]
    )

    # ==== 🔹 Trả về LaunchDescription ====
    return LaunchDescription([
        declare_use_sim_time,
        gazebo,          # chạy world trước
        rsp,             # xuất URDF
        jsp_gui,         # GUI joints
        spawn_robot,     # spawn robot vào world
    ])
