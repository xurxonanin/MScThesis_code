from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # List of TurtleBot namespaces
    turtlebot_namespaces = ['tb1', 'tb2']  # Add more namespaces as needed

    # Create a list of Node actions for each TurtleBot
    nodes = []
    for namespace in turtlebot_namespaces:
        nodes.append(
            Node(
                package='mape_k_loop',
                executable='mape_k_loop',
                name=f'mape_k_loop_node_{namespace}',
                namespace=namespace,
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        )

        nodes.append(
            Node(
                package='mape_k_loop',
                executable='monitor',
                name=f'monitor_{namespace}',
                namespace=namespace,
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        )
        nodes.append(
            Node(
                package='mape_k_loop',
                executable='planning',
                name=f'planning_{namespace}',
                namespace=namespace,
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        )

    return LaunchDescription(nodes)