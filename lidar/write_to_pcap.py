import os
import struct
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

from ouster.sdk import client

output_dir = "/mnt/goose"
# bag_path = "/mnt/goose/rosbag2_2024_08_22-17_32_04"
bag_path = "/mnt/diskstation/tony/aug_16_test_utm_pond/all_sensors_utm_pond/rosbag2_2024_08_16-19_13_07"
topic_name = "/ouster/lidar_packets"
metadata_topic_name = "/ouster/metadata"

os.makedirs(output_dir, exist_ok=True)

rclpy.init()

reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions('', '')
reader.open(storage_options, converter_options)

lidar_packet_msg_type = get_message('ouster_sensor_msgs/msg/PacketMsg')
metadata_msg_type = get_message('std_msgs/msg/String')

# get sensor info
def retrieve_sensor_metadata():
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == metadata_topic_name:
            # deserialization
            metadata_msg = deserialize_message(data, metadata_msg_type)
            return metadata_msg.data
    return None

# ouster sdk stuff
def initialize_xyzlut(sensor_info):
    return client.XYZLut(sensor_info)

# writing to .bin
def write_scan_binary_file(scan, file_path):
    with open(file_path, 'wb') as f:
        for point in scan:
            # xyz
            f.write(struct.pack('fff', point[0], point[1], point[2]))
            # intensity
            f.write(struct.pack('f', point[3]))
            # time
            f.write(struct.pack('Q', point[4]))
            # reflectivity
            f.write(struct.pack('f', point[5]))
            # ambient
            f.write(struct.pack('f', point[6]))
            # range
            f.write(struct.pack('f', point[7]))

# partially derived from ouster sdk
def process_lidar_packets():
    current_frame_id = None
    current_scan = []

    metadata_str = retrieve_sensor_metadata()
    if not metadata_str:
        print("ERROR: Could not retrieve metadata from the ROS bag.")
        return

    sensor_info = client.SensorInfo(metadata_str)
    xyzlut = initialize_xyzlut(sensor_info)

    # reinitialization
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # while reader.has_next():
    #     topic, data, t = reader.read_next()
    #     if topic == topic_name:
    #         msg = deserialize_message(data, lidar_packet_msg_type)

    #         buf = msg.buf
            
    #         packet = client.LidarPacket(65536)  # fixed buffer size (supposedly 
    #         # accourding to documentation)
            
    #         ##### PROBLEMATIC LINE
    #         packet = client.LidarPacket(buf)
    #         '''
    #         need to find a way of copying buffer from packet msg to pcap packet,
    #         now it breaks cuz size inconsistent (due to corrupted packets)
    #         '''
    #         ##### 

    #         if current_frame_id is None:
    #             current_frame_id = packet.frame_id

    #         if packet.frame_id != current_frame_id:
    #             # write to .bin
    #             timestamp_ns = packet.timestamp
    #             file_path = os.path.join(output_dir, f"{timestamp_ns}.bin")
    #             write_scan_binary_file(current_scan, file_path)
                
    #             current_scan = []
    #             current_frame_id = packet.frame_id

    #         for m_id, timestamp, range, reflectivity, intensity, ambient in zip(
    #                 packet.measurement_id, packet.timestamps, packet.ranges,
    #                 packet.reflectivities, packet.intensities, packet.ambients):
    #             xyz = xyzlut(range, m_id)
    #             current_scan.append((
    #                 xyz[0], xyz[1], xyz[2],
    #                 intensity, timestamp,
    #                 reflectivity, ambient, range))

    # if current_scan:
    #     timestamp_ns = packet.timestamp 
    #     file_path = os.path.join(output_dir, f"{timestamp_ns}.bin")
    #     write_scan_binary_file(current_scan, file_path)
        
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == topic_name:
            msg = deserialize_message(data, lidar_packet_msg_type)
            buf = msg.buf
            
            try:
                ################### PROBLEMATIC LINE
                packet = client.LidarPacket(24832)

                packet = client.LidarPacket(buf) 
                ####################

                if current_frame_id is None:
                    current_frame_id = packet.frame_id

                if packet.frame_id != current_frame_id:
                    timestamp_ns = packet.timestamp  
                    file_path = os.path.join(output_dir, f"{timestamp_ns}.bin")
                    write_scan_binary_file(current_scan, file_path)
                    
                    current_scan = []
                    current_frame_id = packet.frame_id

                for m_id, timestamp, range, reflectivity, intensity, ambient in zip(
                        packet.measurement_id, packet.timestamps, packet.ranges,
                        packet.reflectivities, packet.intensities, packet.ambients):
                    xyz = xyzlut(range, m_id)
                    current_scan.append((
                        xyz[0], xyz[1], xyz[2],
                        intensity, timestamp,
                        reflectivity, ambient, range))
                print("Packet not corrupted: success")
                
            except Exception as e:
                print(f"Warning: Skipping corrupted packet. Error: {str(e)[:400]}")
                print(f"Buffer size: {len(buf)} bytes")
                continue

    if current_scan:
        timestamp_ns = packet.timestamp 
        file_path = os.path.join(output_dir, f"{timestamp_ns}.bin")
        write_scan_binary_file(current_scan, file_path)

process_lidar_packets()

rclpy.shutdown()
