from os import path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  share_dir : str = get_package_share_directory('nav_radar')
  return LaunchDescription([

    Node(
        package="nav_radar",
        parameters=[path.join(share_dir, "../config/colossus_and_camera_publisher.yaml")],
        executable="colossus_and_camera_publisher"
    ),

    Node(
        package="tf2_ros",
        arguments = ["0", "0", "0", "0", "0", "0", "map", "camera_image"],
        executable="static_transform_publisher"
    ),

    Node(
        package="rviz2",
        arguments=["-d" + path.normpath(path.join(share_dir, "../rviz/colossus_and_camera_view.rviz"))],
        executable="rviz2"
    )
  ])