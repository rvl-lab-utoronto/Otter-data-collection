from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',                
                # Lidar Topics
                '/ouster/imu_packets',
                '/ouster/lidar_packets',
                '/ouster/metadata',
                
                # Radar topics
                '/radar_data/b_scan_msg',
                '/radar_data/configuration_data',
                
                # Novatel GPS and IMU topics
                '/novatel/oem7/gps/fix',
                '/novatel/oem7/gps/gps',
                '/novatel/oem7/gps/imu',
                '/novatel/oem7/imu/data_raw',
                '/novatel/oem7/bestgnsspos',
                '/novatel/oem7/bestpos',
                '/novatel/oem7/bestutm',
                '/novatel/oem7/bestvel',
                '/novatel/oem7/corrimu',
                '/novatel/oem7/driver/bond',
                '/novatel/oem7/heading2',
                '/novatel/oem7/insconfig',
                '/novatel/oem7/inspva',
                '/novatel/oem7/inspvax',
                '/novatel/oem7/insstdev',
                '/novatel/oem7/odom',
                '/novatel/oem7/oem7raw',
                '/novatel/oem7/ppppos',
                '/novatel/oem7/rxstatus',
                '/novatel/oem7/terrastarinfo',
                '/novatel/oem7/terrastarstatus',
                '/novatel/oem7/time',
            ],
            output='screen'
        ),
    ])
