Launch : ros2 launch robot_model rsp.launch.py // lenh de chay model robot


ros2 launch robot_model bringup.launch.py // lenh de cahy cung gazebo va world ign gazebo 6

de chay teleop : ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
ros2 run teleop_twist_keyboard teleop_twist_keyboard
