import os
from launch import LaunchDescription
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    articubot_one_launch_path = PathJoinSubstitution(
        [FindPackageShare('articubot_one'), 'launch', 'launch_sim.launch.py']
    )

    ira_laser_tools_launch_path = PathJoinSubstitution(
        [FindPackageShare('ira_laser_tools'), 'launch', 'merge_multi.launch.py']
    )

    filter_launch_path = PathJoinSubstitution(
        [FindPackageShare('articubot_one'), 'launch', 'scan_filter.launch.py']
    )
    
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('articubot_one'), 'config', 'main.rviz']
    )

    localization_launch = "ros2 launch articubot_one nav2_localization.launch.py map:=//home/mert/narrow_ws/src/articubot_one/map/40cm_w_box.yaml"

    nav_launch  = "ros2 launch articubot_one navigation_launch.py"


    # waypoint_path = PathJoinSubstitution(
    #     [FindPackageShare('articubot_one'), 'robolaunch_cloudy_navigation', 'scripts', 'way_points.py']
    # )
    
         
    return LaunchDescription([

        # Node(
        #      package='rviz2',
        #      executable='rviz2',
        #      name='rviz2',
        #      output='screen',
        #      arguments=['-d', rviz_config_path],
        # ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(articubot_one_launch_path),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ira_laser_tools_launch_path),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(filter_launch_path),
        ),

        ExecuteProcess(
            cmd=['bash', '-c', localization_launch],
            name='launch slam_toolbox'
        ),

        ExecuteProcess(
            cmd=['bash', '-c', nav_launch],
            name='launch slam_toolbox'
        ),

        # # Node(
        # #     package='robolaunch_cloudy_navigation',
        # #     executable='way_points.py',
        # #     name='way_points',
        # #     output='screen',
        # # ),

        

        
    ])