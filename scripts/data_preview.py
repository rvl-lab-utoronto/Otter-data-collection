from ouster.sdk import client
import matplotlib.pyplot as plt
import rosbags
import os
import numpy as np
import cv2
import pandas as pd

from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_types_from_msg, register_types, get_typestore, Stores
import argparse
import json
from datetime import datetime
from msg_types import BSCAN_MSG, CFG_MSG, FFT_MSG, PACKETMSG
from cv_bridge import CvBridge
from util import radar_polar_to_cartesian

br = CvBridge()

typestore = get_typestore(Stores.ROS2_HUMBLE)
packet_type = get_types_from_msg(PACKETMSG, 'ouster_sensor_msgs/msg/PacketMsg')
radar_bscan_type = get_types_from_msg(BSCAN_MSG, 'navtech_msgs/msg/RadarBScanMsg')
radar_cfg_type = get_types_from_msg(CFG_MSG, 'navtech_msgs/msg/RadarConfigurationMessage')

parser = argparse.ArgumentParser()
parser.add_argument('--bag', default='', type=str, help='path to bag')
parser.add_argument('--outdir', default='', type=str, help='path to bag')

args = parser.parse_args()

typestore.register(packet_type)
typestore.register(radar_cfg_type)
typestore.register(radar_bscan_type)

metadata = None
packet_format = None

radar_dir = '/home/robot/data/aug_22_field_data/radar'
os.makedirs(radar_dir, exist_ok=True)

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
        if connection.topic == '/ouster/metdata':
            msg = reader.deserialize(rawdata, connection.msgtype)
            metadata = json.loads(msg.data)
            sensor_info = client.SensorInfo(msg.data) 
            packet_format = client.PacketFormat(sensor_info)
            
        if connection.topic == '/ouster/lidar_packts':
            msg = reader.deserialize(rawdata, connection.msgtype)
            measurement_ids = packet_format.packet_header(client.ColHeader.MEASUREMENT_ID, msg.buf)
            timestamps = packet_format.packet_header(client.ColHeader.TIMESTAMP, msg.buf)
            ranges = packet_format.packet_field(client.ChanField.RANGE, msg.buf)
            
            timestamp_s = timestamps[0] * 1e-9
            if measurement_ids[0] == 0:
                delta = timestamp_s - last_lidar_timestamp
                last_lidar_timestamp = timestamp_s
                #lidar_stamps.append(delta)
                lidar_stamps.append(timestamps[0])
                
            dt_object = datetime.utcfromtimestamp(timestamp_s)

            formatted_date = dt_object.strftime('%A, %Y-%m-%d %H:%M:%S')

            #print(f'  timestamps = {timestamps}')
            #print(f'  ranges = {ranges.shape}')

        if connection.topic == '/radar_data/b_scan_msg':
            msg = reader.deserialize(rawdata, connection.msgtype)
            timestamp_ns = np.array([msg.timestamps[np.int32(msg.timestamps.shape[0]/2)]], dtype=np.uint64)
            timestamp_s = msg.timestamps[np.int32(msg.timestamps.shape[0]/2)] * 1e-9
            stamps = msg.timestamps.view(np.uint8).reshape((-1, 8))[:, ::-1]
            print(stamps)
            
            cv_image = br.imgmsg_to_cv2(msg.b_scan_img, desired_encoding="passthrough")
            
            radar_stamps.append(timestamp_ns)
            radar_img_oxford = np.zeros((cv_image.shape[0], cv_image.shape[1]+11))
            radar_img_oxford[:, :8] = stamps
            radar_img_oxford[:, 11:] = cv_image
            radar_img_oxford[:, 8] = msg.encoder_values % 256
            radar_img_oxford[:, 9] = msg.encoder_values / 256
            radar_img_oxford[:, 10] = 255

            
            #delta = timestamp_s - last_radar_timestamp
            #radar_stamps.append(delta)


            #dt_object = datetime.utcfromtimestamp(timestamp_s)
            #formatted_date = dt_object.strftime('%A, %Y-%m-%d %H:%M:%S')
            #last_radar_timestamp = timestamp_s

            num_radar_msgs += 1
            
            azimuths = np.linspace(0, 2*np.pi, cv_image.shape[0], endpoint=False)

            radar_resolution = 0.292
            cartesian_image = radar_polar_to_cartesian(cv_image, azimuths, radar_resolution, cart_resolution=1.5, cart_pixel_width=1080)
            img_name = os.path.join(radar_dir, str(timestamp_ns[0]) + ".png")
            cv2.imwrite(img_name, radar_img_oxford)
            #cv2.imshow("Raw scan", cartesian_image)
            #cv2.imshow("Raw Polar Scan", cv_image)

            #cv2.waitKey(10)

    
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
    
    
    formatted_timestamps = pd.to_datetime(radar_stamps, unit='ns', origin='unix')
    print(formatted_timestamps)
    formatted_times = formatted_timestamps.strftime('%H:%M:%S.%f')
    
    with open("timestamp_files/radar_timestamps_aug_22.txt", "w") as file:
        for time in formatted_times:
            file.write(f"{time}\n")
            
    #lidar_stamps = np.array(lidar_stamps)
    #radar_stamps = np.array(radar_stamps)
    #np.save('lidar_stamps', lidar_stamps)
    #np.save('radar_stamps', radar_stamps)
