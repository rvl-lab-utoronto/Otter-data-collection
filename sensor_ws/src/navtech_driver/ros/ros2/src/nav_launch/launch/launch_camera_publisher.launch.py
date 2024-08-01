from os import path
from os import path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  share_dir : str = get_package_share_directory('nav_camera')

  return LaunchDescription([

    Node(
        package="nav_camera",
        parameters=[path.join(share_dir, "config/camera_publisher.yaml")],
        executable="camera_publisher"
    ),

    Node(
        package="tf2_ros",
        arguments = ["0", "0", "0", "0", "0", "0", "map", "camera_image"],
        executable="static_transform_publisher"
    ),

    Node(
        package="rviz2",
        arguments=["-d" + path.normpath(path.join(share_dir, "../rviz/camera_view.rviz"))],
        executable="rviz2"
    )
  ])