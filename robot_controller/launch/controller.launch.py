from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
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
    
    #ACTION CLIENT NODE
    client_node = Node(
        package='robot_controller',
        executable='action_client',
        name='client',
        output='screen',
        prefix='xterm -title "ACTION CLIENT" -e'
    )
    
    #ACTION SERVER NODE
    server_node = Node(
        package='robot_controller',
        executable='action_server',
        name='server',
        output='screen', 
        prefix='xterm -title "ACTION SERVER" -e',
    )

    #BROADCASTER NODE
    broadcaster_node = Node(
        package='robot_controller',
        executable='broadcaster',
        name='broadcaster',
        parameters=[
                {'frame_name': 'goal_frame'}
            ]
    )

    container = ComposableNodeContainer(
            name='robot_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                #ComposableNode(
                #    package='robot_controller',
                #    plugin='robot_controller::RobotActionClient',
                #    name='client',
                #    ),
                    
                ComposableNode(
                    package='robot_controller',
                    plugin='robot_controller::RobotActionServer',
                    name='server',
                    )
            ],
            output='screen',
    )

    return LaunchDescription([
        gazebo_launch,
        client_node,
        server_node,
        broadcaster_node,
    ])
