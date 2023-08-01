import os
from launch import LaunchDescription
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    launch_robot = PathJoinSubstitution(
        [FindPackageShare('articubot_one'), 'launch', 'launch_robot.launch.py']
    )
    ydlidar = PathJoinSubstitution(
        [FindPackageShare('ydlidar_ros2_driver'), 'launch', 'ydlidar_2launch.py']
    )
    laser_filter = PathJoinSubstitution(
        [FindPackageShare('articubot_one'), 'launch', 'scan_filter_nav.launch.py']
    )
    ira_merger = PathJoinSubstitution(
        [FindPackageShare('ira_laser_tools'), 'launch', 'merge_multi_filtered.launch.py']
    )
         
    return LaunchDescription([
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_robot),

        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ydlidar),

        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_filter),
            
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ira_merger),
            
        ),


        # Node(
        #     package='robolaunch_cloudy_navigation',
        #     executable='way_points.py',
        #     name='way_points',
        #     output='screen',
        # ),

        # Node(
        #      package='rviz2',
        #      executable='rviz2',
        #      name='rviz2',
        #      output='screen',
        #      arguments=['-d', rviz_config_path],
        # )

        
    ])