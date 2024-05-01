from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stackbot_core',  # Replace 'your_package_name' with the actual name of your package
            executable='model_publish',  # Replace 'cmd_vel_subscriber_node' with the name of your executable
            output='screen',
        ),
        Node(
            package='stackbot_core',
            executable="speed_control_without_pid",
            output="screen"
        )
    ])
