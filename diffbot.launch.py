import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('bot_gazebo')
    diffbot_urdf_path = os.path.join(pkg_share, 'urdf', 'diffbot.urdf')
    obstacle_urdf_path = os.path.join(pkg_share, 'urdf', 'obstacle.urdf')

    with open(diffbot_urdf_path, 'r') as file:
        robot_description_content = file.read()
        
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'),'launch', 'gazebo.launch.py' )
            ),
        ),

        # Robot State Publisher for newbot
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_newbot',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),

        # Spawn diffbot from topic
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_diffbot',
            output='screen',
            arguments=['-topic', 'robot_description', '-entity', 'diffbot', '-x', '0', '-y', '0', '-z', '0.1'],
        ),

        # Spawn box directly from file (no robot_state_publisher needed)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_box',
            output='screen',
            arguments=['-file', obstacle_urdf_path, '-entity', 'obstacle', '-x', '2', '-y', '0', '-z', '0.0'],
                ),
        
        Node(
            package='bot_gazebo',
            executable='obstacle_avoid',
            name='obstacle_avoid',
            output='screen',
        ),  

         Node(
            package='bot_gazebo',
            executable='print',
            name='print',
            output='screen',
        ),  
    ])

 