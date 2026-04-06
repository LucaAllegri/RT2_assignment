from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

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
        prefix='xterm -title "ACTION CLIENT" -e',
    )
    
    #ACTION SERVER NODE
    server_node = Node(
        package='robot_controller',
        executable='action_server',
        name='server',
        output='screen', 
        prefix='xterm -title "ACTION SERVER" -e',
        parameters=[{
                'target_frame_name': LaunchConfiguration('target_frame_name'),
                'world_frame_name': LaunchConfiguration('world_frame_name')
            }]
    )

    #BROADCASTER NODE
    broadcaster_node = Node(
        package='robot_controller',
        executable='broadcaster',
        name='broadcaster',
        parameters=[{
                'target_frame_name': LaunchConfiguration('target_frame_name'),
                'moved_frame_name': LaunchConfiguration('moved_frame_name')
            }]
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
        DeclareLaunchArgument(
            'world_frame_name', 
            default_value='odom',
            description='World frame name'
        ),

        DeclareLaunchArgument(
            'moved_frame_name', 
            default_value='base_link',
            description='Moved frame name'
        ),

        DeclareLaunchArgument(
            'target_frame_name', 
            default_value='goal_frame',
            description='Target frame name'
        ),


        gazebo_launch,
        client_node,
        server_node,
        broadcaster_node,
    ])
