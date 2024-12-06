import cv2
import os
import argparse
from glob import glob
import numpy as np
from util import radar_polar_to_cartesian
from tqdm import tqdm
import pandas as pd
import sys
from datetime import timedelta
from radar import Radar

parser = argparse.ArgumentParser(description="Display PNG images from a folder sequentially.")

parser.add_argument("--radar", type=str, help="Path to the folder containing PNG images.")
parser.add_argument("--sat", type=str, help="Path to the folder containing PNG images.")
parser.add_argument("--output", type=str, help="Filename for the output timelapse video.")
parser.add_argument("--fps", type=int, default=40, help="Frames per second for the timelapse video.")

data_root = '/media/diskstation/otter-2024-08-23-12/'

args = parser.parse_args()

radar_path = args.radar
sat_root = args.sat
video_name = args.output
fps = np.int32(args.fps)

if not os.path.exists(radar_path):
    print(f"The folder '{radar_path}' does not exist.")
    sys.exit(1)

if not os.path.exists(sat_root):
    print(f"The folder '{sat_root}' does not exist.")
    sys.exit(1)
    
radar_paths = sorted(glob(os.path.join(radar_path, "*.png")))

sat_paths = sorted(glob(os.path.join(sat_root, "*.png")))

radar_images = sorted([np.int64(img.split('.')[0]) for img in os.listdir(radar_path) if img.endswith(('png', 'jpg', 'jpeg'))])
radar_stamps_utc = np.array(radar_images)
radar_stamps_gps = np.array(radar_images) - 18 *1e9

gps_dir = os.path.join(data_root, 'gps')
gps_coords = pd.read_csv((gps_dir+'/aug_23_Radar_poses.txt'))
formatted_timestamps_radar_gps = pd.to_datetime(radar_stamps_gps)

cart_pixel_width = 1000

#first_image = cv2.imread(image_paths[0])

fourcc = cv2.VideoWriter_fourcc(*'mp4v')  
video = cv2.VideoWriter(video_name, fourcc, fps, (cart_pixel_width, cart_pixel_width))

max_frames = None
if max_frames is not None and max_frames < len(sat_paths):
    sat_paths = sat_paths[:max_frames]     

for i, stamp in enumerate(tqdm(formatted_timestamps_radar_gps)):
    microseconds = round(stamp.microsecond / 10000) * 10000
    if microseconds == 1000000:
        stamp = stamp.replace(microsecond=0) + timedelta(seconds=1)
    else:
        stamp = stamp.replace(microsecond=microseconds)
    
    sat_img_name = stamp.strftime('%H:%M:%S.%f')[:-4] + ".png"
    
    full_sat_img_path = os.path.join(sat_root, sat_img_name)
    if os.path.exists(full_sat_img_path) is False:
        print(f"Failed to load image {sat_img_name}")
        continue
    
    sat_image = cv2.imread(full_sat_img_path)
    if sat_image is None:
        print(f"Failed to load image {sat_img_name}")
        continue

    radar_file = radar_path + '/' + str(radar_stamps_utc[i]) + '.png'
    
    if os.path.exists(radar_file) is False:
        print(f"Failed to load image {radar_file}")
        continue
    
    radar_frame = Radar(radar_file)
    radar_frame.load_data()
    radar_frame_cart = radar_frame.polar_to_cart(cart_resolution=1.0, cart_pixel_width=1000)[:,:, None]
    radar_frame_cart = np.concatenate([radar_frame_cart, radar_frame_cart, radar_frame_cart], axis=-1)
    b = 2.0
    a = 0.4
    
    radar_sat_overlay = np.uint8(sat_image*0.4) + np.uint8(radar_frame_cart*255*b)
    
    #cv2.imshow('overlay', radar_sat_overlay)
    radar_frame.unload_data()
    #cv2.imshow('Cartesian Image', cartesian_image)

    key = cv2.waitKey(1)

    if key == ord('q'):
        break
    video.write(radar_sat_overlay)

#cv2.destroyAllWindows()
video.release()

