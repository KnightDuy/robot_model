#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    pkg_path = get_package_share_directory('robot_model')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    world_path = os.path.join(pkg_path, 'worlds', 'my_world.sdf')
    rviz_config = os.path.join(pkg_path, 'rviz', 'config.rviz')
    ekf_config = os.path.join(pkg_path, 'config', 'ekf.yaml')

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_path, '-r'],
        output='screen'
    )

    # Spawn robot
    spawn_robot = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_ign_gazebo', 'create',
                    '-name', 'robot',
                    '-topic', 'robot_description',
                    '-x', '-2.5',
                    '-y', '0.0',
                    '-z', '0.05'
                ],
                output='screen'
            )
        ]
    )

    # ========== BRIDGES ==========
    
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_cmd_vel',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )

    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_odom',
        arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        output='screen'
    )

    bridge_scan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_scan',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )

    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_imu',
        arguments=['/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'],
        output='screen'
    )

    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_camera',
        arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen'
    )

    bridge_camera_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_camera_info',
        arguments=['/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        output='screen'
    )

    bridge_depth_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_depth_camera',
        arguments=['/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen'
    )

    bridge_depth_points = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_depth_points',
        arguments=['/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
        output='screen'
    )
    

    bridge_clock = TimerAction(
    period=5.0,  # delay 5s
    actions=[
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_clock',
            arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
            output='screen'
        )
    ]
)

    # ========== ODOM TO TF ==========
    odom_to_tf = Node(
        package='robot_model',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    # Wheel joint state publisher from odometry
    wheel_odom_pub = TimerAction(
        period=7.0,
        actions=[
            Node(
            package='robot_model',
            executable='wheel_odom_publisher.py',
            name='wheel_odom_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    # Fix covariance
    fix_covariance = Node(
        package='robot_model',
        executable='fix_covariance.py',
        name='fix_covariance',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
        # ========== ROBOT LOCALIZATION EKF ==========
    ekf_node = Node(
        package='robot_localization',  # THÊM DÒNG NÀY
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',  # THÊM DÒNG NÀY
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
    )

    # RViz
    rviz = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        rsp,
        gazebo,
        spawn_robot,
        bridge_cmd_vel,
        bridge_odom,
        bridge_scan,
        bridge_imu,
        bridge_camera,
        bridge_camera_info,
        bridge_depth_camera,
        bridge_depth_points,
        bridge_clock,
        wheel_odom_pub,
        odom_to_tf,
        #fix_covariance,
        ekf_node,
        rviz,
    ])

