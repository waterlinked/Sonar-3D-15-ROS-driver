from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sonar3d',
            executable='sonar_publisher',
            name='sonar_node',
            output='screen',
            parameters=[
                {'IP': '192.168.194.96'},  # Change to your sonar IP, '192.168.194.96' is the fallback ip.
                {'speed_of_sound': 1491}
            ]
        )
    ])
