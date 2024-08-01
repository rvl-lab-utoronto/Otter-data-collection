from os import path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  share_dir = get_package_share_directory('nav_radar')
  return LaunchDescription([

    Node(
        package="nav_radar",
        parameters=[path.normpath(path.join(share_dir, "../config/b_scan_publisher.yaml"))],
        executable="b_scan_publisher"
    ),

    Node(
        package="tf2_ros",
        arguments = ["0", "0", "0", "0", "0", "0", "map", "b_scan_image"],
        executable="static_transform_publisher"
    ),

    Node(
        package="rviz2",
        arguments=["-d" + path.normpath(path.join(share_dir, "../rviz/b_scan_view.rviz"))],
        executable="rviz2"
    )
  ])