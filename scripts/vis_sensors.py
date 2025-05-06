#import pyzed.sl as sl
import cv2
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from oculus_python.files import OculusFileReader

base_dir = "/home/robot/data/aug_22_field_data"
lidar_dir = os.path.join(base_dir, "lidar")
camera_timestamps = []
lidar_timestamps = sorted([np.int64(img.split('.')[0]) for img in os.listdir(lidar_dir) if img.endswith(('png', 'jpg', 'jpeg', '.bin'))])

sonar_file = OculusFileReader(args.filename)
