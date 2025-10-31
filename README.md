Launch : ros2 launch robot_model rsp.launch.py // lenh de chay model robot
lenh mo them joint publisher : ros2 run joint_state_publisher_gui joint_state_publisher_gui // ko can neu chay lenh launch duoi


ros2 launch robot_model bringup.launch.py // lenh de cahy cung gazebo va world ign gazebo 6

de chay teleop : ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
ros2 run teleop_twist_keyboard teleop_twist_keyboard
