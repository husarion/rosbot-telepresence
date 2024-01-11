from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch RViz2 with a default configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/default.rviz'],
            output='screen'
        ),
        # Launch image_transport to republish images
        Node(
            package='image_transport',
            executable='republish',
            name='republish',
            arguments=['ffmpeg', 'in/ffmpeg:=/rosbot2r/camera/color/image_raw/ffmpeg', 'raw', 'out:=/image_raw/uncompressed'],
            output='screen'
        ),
    ])
