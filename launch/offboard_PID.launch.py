from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
       Node(
           package='px4_ros_com',
           executable='offboard_PID',
           name='offboard_pid_control',
           output='screen',
           prefix='/usr/bin/xterm -T offboard_PID -e',
           emulate_tty=True,
           arguments=[
                '--ros-args', '--log-level', 'offboard_pid_control:=info'
            ],
           parameters=['/home/ros2/ws_sensor_combined/src/px4_ros_com/config/offboard_PID.yaml']
       )
   ])
