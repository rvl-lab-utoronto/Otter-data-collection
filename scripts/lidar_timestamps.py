from ouster.sdk import client
import matplotlib.pyplot as plt
import rosbags
import numpy as np

from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_types_from_msg, register_types, get_typestore, Stores
import argparse
import json
from datetime import datetime
from msg_types import BSCAN_MSG, CFG_MSG, FFT_MSG, PACKETMSG



typestore = get_typestore(Stores.ROS2_HUMBLE)
packet_type = get_types_from_msg(PACKETMSG, 'ouster_sensor_msgs/msg/PacketMsg')
radar_bscan_type = get_types_from_msg(BSCAN_MSG, 'navtech_msgs/msg/RadarBScanMsg')
radar_cfg_type = get_types_from_msg(CFG_MSG, 'navtech_msgs/msg/RadarConfigurationMessage')

parser = argparse.ArgumentParser()
parser.add_argument('--bag', default='', type=str, help='path to bag')
args = parser.parse_args()

typestore.register(packet_type)
typestore.register(radar_cfg_type)
typestore.register(radar_bscan_type)

metadata = None
packet_format = None
with AnyReader([Path(args.bag)]) as reader:
    reader.typestore = typestore 
    
    lidar_stamps = []
    radar_stamps = []
    last_radar_timestamp = 0
    MAX_NUM_RADAR_MSGS = 1000
    num_radar_msgs = 0
    last_lidar_timestamp = 0

    plot_time_graph = False
    
    for connection, timestamp, rawdata in reader.messages(connections=reader.connections):
        if connection.topic == '/ouster/metadata':
            msg = reader.deserialize(rawdata, connection.msgtype)
            metadata = json.loads(msg.data)
            print(metadata)
            #break
            sensor_info = client.SensorInfo(msg.data) 
            packet_format = client.PacketFormat(sensor_info)
            
        if connection.topic == '/ouster/lidar_packets':
            msg = reader.deserialize(rawdata, connection.msgtype)
            if metadata is None:
                continue
            measurement_ids = packet_format.packet_header(client.ColHeader.MEASUREMENT_ID, msg.buf)
            timestamps = packet_format.packet_header(client.ColHeader.TIMESTAMP, msg.buf)
            ranges = packet_format.packet_field(client.ChanField.RANGE, msg.buf)
            
            timestamp_s = timestamps[0] * 1e-9
            if measurement_ids[0] == 0:
                delta = timestamp_s - last_lidar_timestamp
                last_lidar_timestamp = timestamp_s
                #lidar_stamps.append(delta)
                lidar_stamps.append(timestamps[0])
            
            timestamp_ros_s = timestamp * 1e-9
            dt_object = datetime.utcfromtimestamp(timestamp_s)

            formatted_date = dt_object.strftime('%A, %Y-%m-%d %H:%M:%S')

            #print(f'  timestamps = {timestamps}')
            #print(timestamp_s)
            print(formatted_date)

            #print(f'  ranges = {ranges.shape}')

        if connection.topic == '/radar_data/b_scan_msg':
            msg = reader.deserialize(rawdata, connection.msgtype)
            timestamp_ns = msg.timestamps[np.int64(msg.timestamps.shape[0]/2)]
            timestamp_s = msg.timestamps[np.int64(msg.timestamps.shape[0]/2)] * 1e-9
            
            radar_stamps.append(timestamp_ns)
            #print(timestamp_ns)
            
            delta = timestamp_s - last_radar_timestamp
            #radar_stamps.append(delta)


            dt_object = datetime.utcfromtimestamp(timestamp_s)
            formatted_date = dt_object.strftime('%A, %Y-%m-%d %H:%M:%S')
            last_radar_timestamp = timestamp_s

            num_radar_msgs += 1

        #if num_radar_msgs >= MAX_NUM_RADAR_MSGS:
        #    break
    
    if plot_time_graph:
        for t in lidar_stamps:
            plt.axvline(x=t, color='blue', linestyle='-', label='LiDAR' if t == lidar_stamps[0] else "")

        for t in radar_stamps:
            plt.axvline(x=t, color='red', linestyle='--', label='Radar' if t == radar_stamps[0] else "")

        plt.xlabel('Time (seconds)')
        plt.ylabel('Event')
        plt.title('LiDAR and Radar Timestamps')
        plt.legend()
        plt.show()
    
    #lidar_stamps = np.array(lidar_stamps)
    #radar_stamps = np.array(radar_stamps)
    #np.save('lidar_stamps', lidar_stamps)
    #np.save('radar_stamps', radar_stamps)
