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
    
    #USER INTERFACE NODE
    user_interface_node = Node(
        package='robot_controller',
        executable='manual_composition',
        name='user_interface',
        output='screen',
        prefix='xterm -title "UI" -e',
    )

    container = ComposableNodeContainer(
            name='robot_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='robot_controller',
                    plugin='robot_controller::RobotActionClient',
                    name='client',
                    parameters=[{
                        'target_frame_name': LaunchConfiguration('target_frame_name'),
                        'world_frame_name': LaunchConfiguration('world_frame_name'),
                    }],
                    ),
                    
                ComposableNode(
                    package='robot_controller',
                    plugin='robot_controller::RobotActionServer',
                    name='server',
                    parameters=[{
                        'target_frame_name': LaunchConfiguration('target_frame_name'),
                        'world_frame_name': LaunchConfiguration('world_frame_name'),
                        'moved_frame_name': LaunchConfiguration('moved_frame_name'),
                    }],
                    ),
            ],
            output='screen',
            prefix='xterm -title "Feedback" -e',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_frame_name', 
            default_value='odom',
        ),

        DeclareLaunchArgument(
            'moved_frame_name', 
            default_value='base_link',
        ),

        DeclareLaunchArgument(
            'target_frame_name', 
            default_value='goal_frame',
        ),

        gazebo_launch,
        container,
        user_interface_node,
        
    ])
