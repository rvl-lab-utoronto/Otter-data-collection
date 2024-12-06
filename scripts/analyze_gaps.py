import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import os
import argparse
import pandas as pd



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', default='', type=str, help='path to data root')
    args = parser.parse_args()
    
    plot = False
    radar_dir = os.path.join(args.root, 'radar_scans')
    radar_images = sorted([np.int64(img.split('.')[0]) for img in os.listdir(radar_dir) if img.endswith(('png', 'jpg', 'jpeg'))])
    # Radar stamps are in GPS time
    radar_stamps = np.array(radar_images) #+ 18 * 1e9
    
    print("Radar Results")
    print("total time of survey")
    print(np.float64(radar_stamps[len(radar_stamps)-1] - radar_stamps[0])* 1e-9)
    radar_stamp_diffs = np.diff(radar_stamps)
    print(len(radar_stamp_diffs))
    print(len(radar_stamps))
    radar_stamps = radar_stamps[1:]
    radar_stamps = radar_stamps[radar_stamp_diffs > 1000000000]
    
    if plot == True:
        plt.plot(radar_stamp_diffs * 1e-9)
        plt.xlabel("Frame ID")
        plt.ylabel("Time Delta (s)")
        plt.title("Radar Time Delta vs Frame ID")
        plt.show()
    
    #print("camera stuff")
    #camera_dir = os.path.join(args.root, 'camera/left_images')
    #camera_images = sorted([np.int64(img.split('.')[0]) for img in os.listdir(camera_dir) if img.endswith(('png', 'jpg', 'jpeg'))])
    #camera_stamps = np.array(camera_images)
    #camera_stamp_diffs = np.diff(camera_stamps)

    #if plot == True:
    #    plt.plot(camera_stamp_diffs * 1e-6)
    #    plt.xlabel("Frame ID")
    #    plt.ylabel("Time Delta (s)")
    #    plt.title("Camera Time Delta vs Frame ID")
    #    plt.show()
    #print(len(camera_stamp_diffs[camera_stamp_diffs > 100000]*1e-6))
    #
    #print("sonar stuff")
    #sonar_dir = os.path.join(args.root, 'sonar/images')
    #sonar_images = sorted([np.float64(img.rstrip(".png")) for img in os.listdir(sonar_dir) if img.endswith(('png', 'jpg', 'jpeg'))])
    #sonar_stamps = np.array(sonar_images)
    #sonar_stamp_diffs = np.diff(sonar_stamps)
    #if plot ==  True:
    #    plt.plot(sonar_stamp_diffs * 1e-9)
    #    plt.xlabel("Frame ID")
    #    plt.ylabel("Time Delta (s)")
    #    plt.title("Sonar Time Delta vs Frame ID")
    #    plt.show()
    #print(sonar_stamp_diffs[sonar_stamp_diffs > 1])
    
    file_path_utc = "utc_timestamps.txt"
    
    formatted_timestamps = pd.to_datetime(radar_stamps, unit='ns', origin='unix')
    print(formatted_timestamps)
    formatted_times = formatted_timestamps.strftime('%H:%M:%S.%f')
    
    with open("timestamp_files/radar_timestamps.txt", "w") as file:
        for time in formatted_times:
            file.write(f"{time}\n")
    #np.savetxt(file_path_utc, timestamps_utc, fmt='%s')
    
    


