from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

  params_file = PathJoinSubstitution([get_package_share_directory('exploration_sim_bringup'), 'config', 'navigation_params.yaml'])

  slam_toolbox_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(PathJoinSubstitution([get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py'])),
    launch_arguments={
      'use_sim_time': 'true',
      'parms_file': params_file,
      'odom_topic': 'diff_drive/odom',
    }.items(),
  )

  ld = LaunchDescription()

  ld.add_action(slam_toolbox_launch)

  return ld