from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

playground_world_path = PathJoinSubstitution(
        [FindPackageShare("articubot_one"), "worlds", "playground.world"]
    )

empty_world_path = PathJoinSubstitution(
        [FindPackageShare("articubot_one"), "worlds", "empty.world"]
    )

warehouse_world_path = PathJoinSubstitution(
        [FindPackageShare("articubot_one"), "worlds", "industrial-warehouse.sdf"]
    )

arcelik_world_path = PathJoinSubstitution(
        [FindPackageShare("articubot_one"), "worlds", "arcelik"]
    )

gazebo_path = PathJoinSubstitution(
        [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
    )

description_launch_path = PathJoinSubstitution(
        [FindPackageShare("articubot_one"), "launch", "rsp.launch.py"]
    )

joystick_path = PathJoinSubstitution(
        [FindPackageShare("articubot_one"), "launch", "joystick.launch.py"]
    )

twist_mux_params_path = PathJoinSubstitution(
        [FindPackageShare("articubot_one"), "config", "twist_mux.yaml"]
    )

def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            name="vehicle",
            default_value="'cloudy_v2'",
            description="vehicle type"
        ),

        DeclareLaunchArgument(
            name="world",
            default_value="arcelik",
            description="gazebo world name"
        ),

        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(description_launch_path), 
                launch_arguments={
                    'use_sim_time': 'true', 
                    'use_ros2_control': 'true'
                }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joystick_path), 
            launch_arguments={'use_sim_time': 'true'}.items()
        ),

        Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params_path, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_path),
            launch_arguments={
                'use_sim_time': str("true"),
                'world': playground_world_path,
            }.items(),
            condition=IfCondition(
                PythonExpression(
                    ["'", LaunchConfiguration("world"), "' == 'playground'"]
                )
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_path),
            launch_arguments={
                'use_sim_time': str("true"),
                'world': warehouse_world_path,
            }.items(),
            condition=IfCondition(
                PythonExpression(
                    ["'", LaunchConfiguration("world"), "' == 'warehouse'"]
                )
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_path),
            launch_arguments={
                'use_sim_time': str("true"),
                'world': arcelik_world_path,
            }.items(),
            condition=IfCondition(
                PythonExpression(
                    ["'", LaunchConfiguration("world"), "' == 'arcelik'"]
                )
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_path),
            launch_arguments={
                'use_sim_time': str("true"),
                'world': empty_world_path,
            }.items(),
            condition=IfCondition(
                PythonExpression(
                    ["'", LaunchConfiguration("world"), "' == 'empty'"]
                )
            ),
        ),
         
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
            output='screen'
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_cont"],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_broad"],
        )
    ])