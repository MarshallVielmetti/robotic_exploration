from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

  params_file = PathJoinSubstitution([get_package_share_directory('exploration_sim_bringup'), 'config', 'mapper_params_online_async.yaml'])

  slam_toolbox_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(PathJoinSubstitution([get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'])),
    launch_arguments={
      'use_sim_time': 'true',
      'slam_params_file': params_file,
    }.items(),
  )

  ld = LaunchDescription()

  ld.add_action(slam_toolbox_launch)

  return ld