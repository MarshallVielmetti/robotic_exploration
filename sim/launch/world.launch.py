import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_sim = FindPackageShare('sim')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_ros_gz_bridge = FindPackageShare('ros_gz_bridge')

    world_path = [TextSubstitution(text='-r -z 100000 '), PathJoinSubstitution([pkg_sim, 'worlds', 'empty.sdf'])]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': world_path
        }.items()
    )

    gz_spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_spawn_model.launch.py'])
        ),
        launch_arguments={
            'world': 'empty',
            'file': PathJoinSubstitution([pkg_sim, 'models', 'vehicle', 'model.sdf']),
            'entity_name': 'robot',
            'x': '5.0',
            'y': '5.0',
            'z': '5.0',
        }.items()
    )

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
        gz_ros_bridge
    ])
