
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
parser.add_argument("--output", type=str, help="Filename for the output timelapse video.")
parser.add_argument("--fps", type=int, default=4, help="Frames per second for the timelapse video.")

args = parser.parse_args()

folder_path = args.folder
video_name = args.output
fps = np.int32(args.fps)

if not os.path.exists(folder_path):
    print(f"The folder '{folder_path}' does not exist.")
    sys.exit(1)

image_paths = sorted(glob(os.path.join(folder_path, "*.png")))
cart_pixel_width = 1000

#first_image = cv2.imread(image_paths[0])

fourcc = cv2.VideoWriter_fourcc(*'mp4v')  
video = cv2.VideoWriter(video_name, fourcc, fps, (cart_pixel_width, cart_pixel_width))

max_frames = 1000
if max_frames is not None and max_frames < len(image_paths):
    image_paths = image_paths[:max_frames]     
    
def display_images(image_paths):
    for image_path in tqdm(image_paths):
        polar_image = cv2.imread(image_path)

        if polar_image is None:
            print(f"Failed to load image {image_path}")
            continue

        azimuths = np.linspace(0, 2*np.pi, polar_image.shape[0], endpoint=False)

        radar_resolution = 0.292
        cartesian_image = radar_polar_to_cartesian(polar_image, azimuths, radar_resolution, cart_resolution=1.5, cart_pixel_width=cart_pixel_width)
        
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (50,50)
        fontScale              = 2
        fontColor              = (255,255,255)
        thickness              = 1
        lineType               = 2
        
        image_path = str(image_path)
        timestamp = image_path.split("/")[-1]
        timestamp = timestamp.split(".")[0]
        timestamp = np.uint64(timestamp)

        timestamp_formatted = pd.to_datetime(timestamp)
        timestamp_str = timestamp_formatted.strftime('%H:%M:%S.%f')[:-4]

        #cv2.putText(cartesian_image, timestamp_str,
        #    bottomLeftCornerOfText, 
        #    font, 
        #    fontScale,
        #    fontColor,
        #    thickness,
        #    lineType)           
        
        #cv2.imshow('Cartesian Image', cartesian_image)
        #cv2.imshow('Cartesian Image', cartesian_image)

        key = cv2.waitKey(1)

        if key == ord('q'):
            break
        video.write(cartesian_image)
        

    #cv2.destroyAllWindows()
    video.release()

display_images(image_paths)
