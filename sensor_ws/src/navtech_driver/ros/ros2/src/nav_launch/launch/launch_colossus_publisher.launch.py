from os import path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  share_dir : str = get_package_share_directory('nav_radar')
  return LaunchDescription([

    Node(
        package="nav_radar",
        parameters=[path.join(share_dir, "../config/colossus_publisher.yaml")],
        executable="colossus_publisher"
    )
  ])