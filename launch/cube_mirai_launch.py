from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'source',
        #     default_value='wheat_ver2.json',
        #     description='話題として使うjsonファイルの名前'
        # ),
        DeclareLaunchArgument(
            'assign_cube',
            default_value='def/white',
            description='使用するPoKeBo Cubeのtypeとcolor'
        ),
        Node(
            package='pokebo_cube',
            executable='cube_core',
            name='poke_blue',
            parameters=[{
                        'assign_cube': 'mirai/blue',
                        }],
            output='screen',
            # arguments=['--ros-args', '--log-level', 'debug'],
        ),
        Node(
            package='pokebo_cube',
            executable='cube_core',
            name='poke_yellow',
            parameters=[{
                        'assign_cube': 'mirai/yellow',
                        }],
            output='screen',
            # arguments=['--ros-args', '--log-level', 'debug'],
        ),
        Node(
            package='pokebo_cube',
            executable='cube_core',
            name='poke_green',
            parameters=[{
                        'assign_cube': 'mirai/green',
                        }],
            output='screen',
            # arguments=['--ros-args', '--log-level', 'debug'],
        ),
        Node(
            package='pokebo_cube',
            executable='utterance_core',
            name='utterance_core',
            output='screen',
        ),
        Node(
            package='pokebo_cube',
            executable='human_input',
            name='human_input',
            output='screen',
        ),
    ])
