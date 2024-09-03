import os
import struct
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

from ouster.sdk import client

# Paths as provided by you
output_dir = "/mnt/goose"
bag_path = "/mnt/goose/rosbag2_2024_08_22-17_32_04"
topic_name = "/ouster/lidar_packets"
metadata_topic_name = "/ouster/metadata"

# Ensure the output directory exists
os.makedirs(output_dir, exist_ok=True)

# Initialize ROS 2
rclpy.init()

# Open the ROS 2 bag
reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions('', '')
reader.open(storage_options, converter_options)

# Create message types for deserialization
lidar_packet_msg_type = get_message('ouster_sensor_msgs/msg/PacketMsg')
metadata_msg_type = get_message('std_msgs/msg/String')

# Function to retrieve sensor metadata from the bag
def retrieve_sensor_metadata():
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == metadata_topic_name:
            # Deserialize the metadata message
            metadata_msg = deserialize_message(data, metadata_msg_type)
            return metadata_msg.data
    return None

# XYZLut initialization
def initialize_xyzlut(sensor_info):
    return client.XYZLut(sensor_info)

# Prepare a function to write a binary file for each scan
def write_scan_binary_file(scan, file_path):
    with open(file_path, 'wb') as f:
        for point in scan:
            # Write the 3D coordinates (x, y, z)
            f.write(struct.pack('fff', point[0], point[1], point[2]))
            # Write the intensity (photon count)
            f.write(struct.pack('f', point[3]))
            # Write the sensor time (timestamp)
            f.write(struct.pack('Q', point[4]))
            # Write the reflectivity (calibrated reflectivity)
            f.write(struct.pack('f', point[5]))
            # Write the ambient (near-infrared photons)
            f.write(struct.pack('f', point[6]))
            # Write the range (distance)
            f.write(struct.pack('f', point[7]))

# Function to process the Ouster LiDAR packets and create scans
def process_lidar_packets():
    current_frame_id = None
    current_scan = []

    # Retrieve sensor metadata from the ROS bag
    metadata_str = retrieve_sensor_metadata()
    if not metadata_str:
        print("ERROR: Could not retrieve metadata from the ROS bag.")
        return

    # Initialize SensorInfo from metadata
    sensor_info = client.SensorInfo(metadata_str)
    xyzlut = initialize_xyzlut(sensor_info)

    # Reset the reader to start from the beginning
    # Reinitialize the reader to start from the beginning
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)


    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == topic_name:
            # Deserialize the lidar packet
            msg = deserialize_message(data, lidar_packet_msg_type)

            # The actual binary buffer containing the packet data
            buf = msg.buf
            
            # Parse the lidar packet
            packet = client.LidarPacket(65536)  # Fixed buffer size, as per the SDK documentation
            packet = client.LidarPacket(buf)

            if current_frame_id is None:
                current_frame_id = packet.frame_id

            if packet.frame_id != current_frame_id:
                # Write the previous scan to a binary file
                timestamp_ns = packet.timestamp  # Use timestamp of the last packet for naming
                file_path = os.path.join(output_dir, f"{timestamp_ns}.bin")
                write_scan_binary_file(current_scan, file_path)
                
                # Reset for the next scan
                current_scan = []
                current_frame_id = packet.frame_id

            # For each channel data block in the packet, append the data to the current scan
            for m_id, timestamp, range, reflectivity, intensity, ambient in zip(
                    packet.measurement_id, packet.timestamps, packet.ranges,
                    packet.reflectivities, packet.intensities, packet.ambients):
                # Compute 3D coordinates
                xyz = xyzlut(range, m_id)
                # Append point data (x, y, z, intensity, sensor time, reflectivity, ambient, range)
                current_scan.append((
                    xyz[0], xyz[1], xyz[2],
                    intensity, timestamp,
                    reflectivity, ambient, range))

    # Write the last scan if not empty
    if current_scan:
        timestamp_ns = packet.timestamp  # Use timestamp of the last packet for naming
        file_path = os.path.join(output_dir, f"{timestamp_ns}.bin")
        write_scan_binary_file(current_scan, file_path)

# Run the processing function
process_lidar_packets()

# Shutdown ROS 2
rclpy.shutdown()
