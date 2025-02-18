import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, TextSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_sim = FindPackageShare('sim')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_ros_gz_bridge = FindPackageShare('ros_gz_bridge')

    world_path = [TextSubstitution(text='-r -z 100000 '), PathJoinSubstitution([pkg_sim, 'worlds', 'test.sdf'])]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': world_path
        }.items()
    )


    robot_desc = PathJoinSubstitution([pkg_sim, 'models', 'vehicle', 'model.sdf'])
    robot_desc_content = Command(['cat ', robot_desc])
    world_desc_content = Command(['cat ', PathJoinSubstitution([pkg_sim, 'worlds', 'test.sdf'])])

    gz_spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_spawn_model.launch.py'])
        ),
        launch_arguments={
            'world': 'empty',
            'file': robot_desc,
            'entity_name': 'robot',
            'x': '5.0',
            'y': '5.0',
            'z': '5.0',
        }.items()
    )

    # joint_state_publisher_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     arguments=[robot_desc],
    #     output=['screen']
    # )

    # world_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='world_state_publisher',
    #     output='both',
    #     parameters=[
    #         {'use_sim_time': True},
    #         {'robot_description': world_desc_content},
    #     ]
    # )

    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='both',
    #     parameters=[
    #         {'use_sim_time': True},
    #         {'robot_description': robot_desc_content},
    #     ]
    # )


    gz_ros_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_bridge, 'launch', 'ros_gz_bridge.launch.py'])
        ),
        launch_arguments={
            'bridge_name': 'ros_gz_bridge',
            'config_file': PathJoinSubstitution([pkg_sim, 'config', 'vehicle.config'])
        }.items()
    )

    return LaunchDescription([
        gz_sim,
        gz_spawn_robot,
        # world_state_publisher,
        # robot_state_publisher,
        gz_ros_bridge
    ])