import cv2
import os
import argparse
import pandas as pd
from glob import glob
import numpy as np
from util import radar_polar_to_cartesian
from tqdm import tqdm

parser = argparse.ArgumentParser(description="Display PNG images from a folder sequentially.")
parser.add_argument("--folder", type=str, help="Path to the folder containing PNG images.")
parser.add_argument("--output", type=str, help="Base filename for the output timelapse videos.")
parser.add_argument("--fps", type=int, default=4, help="Frames per second for the timelapse video.")

args = parser.parse_args()

folder_path = args.folder
base_video_name = args.output
fps = np.int32(args.fps)

if not os.path.exists(folder_path):
    print(f"The folder '{folder_path}' does not exist.")
    sys.exit(1)

image_paths = sorted(glob(os.path.join(folder_path, "*.png")))
cart_pixel_width = 1000

# Helper function to create a new video writer
def create_video_writer(video_name):
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    return cv2.VideoWriter(video_name, fourcc, fps, (cart_pixel_width, cart_pixel_width))

video = None
last_timestamp = None
video_count = 0
max_frames = None

if max_frames is not None and max_frames < len(image_paths):
    image_paths = image_paths[:max_frames]     

def display_images(image_paths):
    global video, last_timestamp, video_count
    
    for image_path in tqdm(image_paths):
        polar_image = cv2.imread(image_path)

        if polar_image is None:
            print(f"Failed to load image {image_path}")
            continue

        azimuths = np.linspace(0, 2*np.pi, polar_image.shape[0], endpoint=False)
        radar_resolution = 0.292
        cartesian_image = radar_polar_to_cartesian(
            polar_image, azimuths, radar_resolution, cart_resolution=1.5, cart_pixel_width=cart_pixel_width
        )
        
        timestamp = int(image_path.split("/")[-1].split(".")[0])
        current_timestamp = pd.to_datetime(timestamp, unit='ns')

        # Check for 5-minute gap
        if last_timestamp is not None and (current_timestamp - last_timestamp).total_seconds() > 200:
            # Close the current video if it exists and start a new one
            if video is not None:
                video.release()
            video_count += 1
            video_name = f"{base_video_name}_{video_count}.mp4"
            video = create_video_writer(video_name)

        # Initialize video writer if it’s the first frame or a new video segment is started
        if video is None:
            video_name = f"{base_video_name}_{video_count}.mp4"
            video = create_video_writer(video_name)

        # Write frame to video and update the last timestamp
        video.write(cartesian_image)
        last_timestamp = current_timestamp

    # Release the last video after finishing all frames
    if video is not None:
        video.release()

display_images(image_paths)
