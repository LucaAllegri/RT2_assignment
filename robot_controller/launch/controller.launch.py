from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bme_gazebo_sensors'),
                'launch',
                'spawn_robot_ex.launch.py'
            )
        )
    )
    
    #UI NODE
    user_node = Node(
        package='robot_controller',
        executable='action_client',
        name='client',
        output='screen',
        prefix='xterm -title "ACTION CLIENT" -e'
    )
    
    #DISTANCE NODE
    server_node = Node(
        package='robot_controller',
        executable='action_server',
        name='server',
        output='screen', 
        prefix='xterm -title "ACTION SERVER" -e',
    )

    return LaunchDescription([
        gazebo_launch,
        user_node,
        server_node,
    ])
