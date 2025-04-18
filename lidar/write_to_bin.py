import os
import json
import struct
import rclpy
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

from ouster.sdk import client
from ouster.sdk.client.core import ScanBatcher, LidarPacket, LidarScan, get_field_types



output_dir = "/home/robot/data/aug_22_field_data/lidar"
# bag_path = "/mnt/goose/rosbag2_2024_08_22-17_32_04"
bag_path = "/home/robot/data/aug_22_field_data/rosbag2_2024_08_22-17_32_04"
#bag_path = "/home/robot/synology/tony/aug_16_test_utm_pond/all_sensors_utm_pond/rosbag2_2024_08_16-19_13_07"
#bag_path = "/home/robot/data/aug_23_field_data/rosbag2_2024_08_23-14_14_04"
topic_name = "/ouster/lidar_packets"
imu_topic_name = "/ouster/imu_packets"
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
            
lidar_dtype = np.dtype([
    ("x", np.float64),         
    ("y", np.float64),         
    ("z", np.float64),         
    ("intensity", np.uint16),  
    ("timestamp", np.uint64),  
    ("reflectivity", np.uint16),  
    ("ambient", np.uint16)     
])  # 32 bytes per point


# partially derived from ouster sdk
def process_lidar_packets():
    current_frame_id = None
    current_scan = []

    metadata_str = retrieve_sensor_metadata()
    if not metadata_str:
        print("ERROR: Could not retrieve metadata from the ROS bag.")
        return

    sensor_info = client.SensorInfo(metadata_str)
    metadata = json.loads(metadata_str)
    packet_format = client.PacketFormat(sensor_info)
    batch = ScanBatcher(metadata["lidar_data_format"]["columns_per_frame"], packet_format)
    h = packet_format.pixels_per_column
    w = metadata["lidar_data_format"]["columns_per_frame"]
    columns_per_packet = packet_format.columns_per_packet
    packets_per_frame = w // columns_per_packet
    field_types = get_field_types(sensor_info)
    print(metadata)
    
    xyzlut = initialize_xyzlut(sensor_info)

    # reinitialization

    
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)


    ls_write = None
    lp = LidarPacket(packet_format.lidar_packet_size)
    count = 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        
        if topic == topic_name:
            msg = deserialize_message(data, lidar_packet_msg_type)
            buf = msg.buf
            try:
                ls_write = ls_write or LidarScan(
                    h, w, field_types, columns_per_packet)
                
                res = batch(buf, ls_write)
                if res:
                    for field in ls_write.fields:
                        print('{0:15} {1}'.format(str(field), ls_write.field(field).dtype))
                    signal = ls_write.field(client.ChanField.SIGNAL)
                    ranges = ls_write.field(client.ChanField.RANGE)
                    timestamps = np.tile(ls_write.timestamp, (signal.shape[0], 1)).reshape(-1, 1)
                    
                    reflectivity = ls_write.field(client.ChanField.REFLECTIVITY)
                    ambient = ls_write.field(client.ChanField.NEAR_IR)

                    ranges_destaggered = client.destagger(sensor_info, ranges).reshape(-1, 1)
                    signal_destaggered = client.destagger(sensor_info, signal).reshape(-1, 1)
                    reflectivity_destaggered = client.destagger(sensor_info, reflectivity).reshape(-1, 1)
                    ambient_destaggered = client.destagger(sensor_info, ambient).reshape(-1, 1)
                    
                    xyz = xyzlut(ranges_destaggered)
                    xyz = xyz.reshape(-1, 3)
                    print("******************* NEW SCAN ******************")
                    print("Points:",  xyz.shape, xyz.dtype)
                    print("Intensity:", signal_destaggered.shape, signal_destaggered.dtype)
                    print("Stamps:", timestamps.shape, timestamps.dtype)
                    print("Reflectivity:", reflectivity_destaggered.shape, reflectivity_destaggered.dtype)
                    print("Ambient:", ambient_destaggered.shape, ambient_destaggered.dtype)
                    
                    data = np.empty(xyz.shape[0], dtype=lidar_dtype)

                    data["x"] = xyz[:, 0]
                    data["y"] = xyz[:, 1]
                    data["z"] = xyz[:, 2]
                    data["intensity"] = signal_destaggered.flatten().astype(np.uint16)
                    data["timestamp"] = timestamps.flatten().astype(np.uint64)
                    data["reflectivity"] = reflectivity_destaggered.flatten().astype(np.uint16)
                    data["ambient"] = ambient_destaggered.flatten().astype(np.uint16)

                    #filename = str(timestamps[timestamps.shape[0]//2][0])+".bin"
                    filename = str(t)+".bin"
                    filename = os.path.join(output_dir, filename)
                    data.tofile(filename)
                
            except Exception as e:
                print(f"Warning: Skipping corrupted packet. Error: {str(e)[:400]}")
                print(f"Buffer size: {len(buf)} bytes")
                continue


process_lidar_packets()

rclpy.shutdown()
