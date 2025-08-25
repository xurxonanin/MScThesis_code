import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    turtlebot3_multi_robot_dir = get_package_share_directory('turtlebot3_multi_robot')
    mape_k_loop_dir = get_package_share_directory('mape_k_loop')
    movement_dir = get_package_share_directory('movement')
    odom_tf_broadcaster_dir = get_package_share_directory('odom_tf_broadcaster')


    # Launch configuration variables specific to simulation
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_drive = LaunchConfiguration('enable_drive', default='false')
    enable_rviz = LaunchConfiguration('enable_rviz', default='false')

    # Include the Gazebo simulation launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_multi_robot_dir, 'launch', 'gazebo_multi_nav2_world.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'enable_drive': enable_drive,
            'enable_rviz': enable_rviz
        }.items()
    )

    # Include the monitor position launch file
    loop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mape_k_loop_dir, 'launch', 'mape_k.launch.py')
        )
    )

    movement_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(movement_dir, 'launch', 'movement.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    odom_tf_broadcaster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(odom_tf_broadcaster_dir, 'launch', 'odom_tf_broadcaster.launch.py')
        )
    )


    # IncludeLaunchDescription(
    #                 PythonLaunchDescriptionSource(
    #                     os.path.join(bringup_dir, "launch", "tb3_simulation_launch.py")
    #                 ),
    #                 launch_arguments={
    #                     "namespace": robot["name"],
    #                     "use_namespace": "True",
    #                     "map": map_yaml_file,
    #                     "use_sim_time": "True",
    #                     "params_file": params_file,
    #                     "autostart": autostart,
    #                     "use_rviz": "False",
    #                     "use_simulator": "False",
    #                     "headless": "False",
    #                     "use_robot_state_pub": use_robot_state_pub,
    #                     "robot_sdf": robot_sdf,
    #                     "x_pose": TextSubstitution(text=str(robot["x_pose"])),
    #                     "y_pose": TextSubstitution(text=str(robot["y_pose"])),
    #                     "z_pose": TextSubstitution(text=str(robot["z_pose"])),
    #                     "roll": TextSubstitution(text=str(robot["roll"])),
    #                     "pitch": TextSubstitution(text=str(robot["pitch"])),
    #                     "yaw": TextSubstitution(text=str(robot["yaw"])),
    #                     "robot_name": TextSubstitution(text=robot["name"]),
    #                 }.items(),
    #             ),


    return LaunchDescription([
        gazebo_launch,
        loop_launch,
        movement_launch,
        odom_tf_broadcaster_launch
    ])