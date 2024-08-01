# Navtech ROS2 Driver Version History

This document is intended to provide the change history of each ROS2 project, within the Navtech ROS2 driver repository. Details will include project name, version number, release date and a list of changes.

Please see lower level README.md files, for further information regarding usage of changes.

## nav_camera

| Version Number           | Release Date          | Change(s)                                                                         |
| :----------------------- | :-------------------: | :-------------------------------------------------------------------------------: |
| 1.0.0                    | 26/10/2021            | Initial release                                                                   |
| 1.0.1                    | 28/10/2022            | Minor update to create output folder for saving videos                            |
| 1.1.0                    | 03/07/2023            | Update to build .deb packages and camera_ros renamed to nav_camera                |

## nav_launch

| Version Number           | Release Date          | Change(s)                                                                         |
| :----------------------- | :-------------------: | :-------------------------------------------------------------------------------: |
| 1.0.0                    | 26/10/2021            | Initial release                                                                   |
| 1.1.0                    | 03/07/2023            | Update to build .deb packages and launch_ros renamed to nav_launch                |
| 1.2.0                    | 27/09/2023            | Added new point cloud example from pointcloud messages sent directly from radar   |

## navtech_msgs

| Version Number           | Release Date          | Change(s)                                                                         |
| :----------------------- | :-------------------: | :-------------------------------------------------------------------------------: |
| 1.0.0                    | 26/10/2021            | Initial release                                                                   |
| 1.1.0                    | 03/07/2023            | Update to build .deb packages and messages renamed to navtech_msgs                |

## nav_radar

| Version Number           | Release Date          | Change(s)                                                                         |
| :----------------------- | :-------------------: | :-------------------------------------------------------------------------------: |
| 1.0.0                    | 26/10/2021            | Initial release                                                                   |
| 1.1.0                    | 27/10/2021            | Added ability to run *navigation mode point cloud publisher* with onboard or local processing|
| 1.1.1                    | 11/01/2022            | Fixed issue where laser_scan_publisher and point_cloud_publisher would only use a maximumum end bin of 400|
| 1.1.2                    | 12/01/2022            | Set default for end_bin to max bins from radar, improved laser_scan_publisher efficiency|
| 1.1.3                    | 13/01/2022            | Changed laser_scan_publisher and point_cloud_publisher to ignore bins before start_bin when thresholding|
| 1.2.0                    | 28/10/2022            | Updated all radar code to use the new Navtech Radar SDK                           |
| 1.3.0                    | 29/11/2022            | Fixed issue with navigaation mode point cloud not displaying correctly after SDK update|
| 1.4.0                    | 03/07/2023            | Update to build .deb packages and nav_ros renamed to nav_radar                    |
| 1.5.0                    | 27/09/2023            | Added new point cloud example from pointcloud messages sent directly from radar   |
| 1.6.0                    | 10/10/2023            | Updated some examples to include functions from the existing iasdk                |

## nav_rviz

| Version Number           | Release Date          | Change(s)                                                                         |
| :----------------------- | :-------------------: | :-------------------------------------------------------------------------------: |
| 1.0.0                    | 26/10/2021            | Initial release                                                                   |
| 1.1.0                    | 03/07/2023            | rviz_views renamed to nav_rviz                                                    |
| 1.2.0                    | 27/09/2023            | Added new point cloud view for pointcloud messages sent directly from radar       |