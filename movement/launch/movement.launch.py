from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [   
            # TB1
            Node(
                package='movement',
                executable='movement',
                name='movement',
                namespace = 'tb1',
                output='screen',
                parameters = [
                    {
                        'use_sim_time': True
                    }
                ]
            ),
            # TB2
            Node(
                package='movement',
                executable='movement',
                name='movement',
                namespace = 'tb2',
                output='screen',
                parameters = [
                    {
                        'use_sim_time': True
                    }
                ]
            )
        ]
    )