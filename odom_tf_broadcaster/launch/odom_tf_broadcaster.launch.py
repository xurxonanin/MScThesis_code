from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odom_tf_broadcaster',
            executable='odom_tf_broadcaster',
            namespace='tb1',
            parameters=[{
                'odom_topic': '/tb1/odom',
                'odom_frame': 'tb1/odom',
                'base_frame': 'tb1/base_footprint',  # matches your AMCL base_frame_id
            }],
            output='screen'
        ),
        Node(
            package='odom_tf_broadcaster',
            executable='odom_tf_broadcaster',
            namespace='tb2',
            parameters=[{
                'odom_topic': '/tb2/odom',
                'odom_frame': 'tb2/odom',
                'base_frame': 'tb2/base_footprint',
            }],
            output='screen'
        )
    ])
