import numpy as np
import os
from pyboreas.utils.radar import radar_polar_to_cartesian
import os.path as osp
import cv2


def load_radar(example_path):
    """Decode a single Oxford Radar RobotCar Dataset radar example
    Args:
        example_path (AnyStr): Oxford Radar RobotCar Dataset Example png
    Returns:
        timestamps (np.ndarray): Timestamp for each azimuth in int64 (UNIX time)
        azimuths (np.ndarray): Rotation for each polar radar azimuth (radians)
        valid (np.ndarray) Mask of whether azimuth data is an original sensor reading or interpolated from adjacent
            azimuths
        fft_data (np.ndarray): Radar power readings along each azimuth
    """
    # Hard coded configuration to simplify parsing code
    encoder_size = 5600
    resolution = 0.292

    raw_example_data = cv2.imread(example_path, cv2.IMREAD_GRAYSCALE)
    timestamps = raw_example_data[:, :8].copy().view(np.int64)
    azimuths = (
        raw_example_data[:, 8:10].copy().view(np.uint16)
        / float(encoder_size)
        * 2
        * np.pi
    ).astype(np.float32)
    valid = raw_example_data[:, 10:11] == 255
    fft_data = raw_example_data[:, 11:].astype(np.float32)[:, :, np.newaxis] / 255.0
    min_range = int(round(2.5 / resolution))
    fft_data[:, :min_range] = 0
    fft_data = np.squeeze(fft_data)
    return timestamps, azimuths, valid, fft_data, resolution


class Radar():
    def __init__(self, path):
        self.resolution = 0.292
        self.timestamps = None
        self.azimuths = None
        self.polar = None
        self.cartesian = None
        self.mask = None
        data_root = '/media/diskstation/otter-2024-08-23-12'
        radar_dir = os.path.join(data_root, 'radar')
        self.sensor_root = radar_dir
        self.path = path
        self.frame = "1"

    def load_data(self):
        # Loads polar radar data, timestamps, azimuths, and resolution value
        # Additionally, loads a pre-computed cartesian radar image and binary mask if they exist.
        self.timestamps, self.azimuths, _, self.polar, self.resolution = load_radar(
            self.path
        )
        cart_path = osp.join(self.sensor_root, "cart", self.frame + ".png")
        if osp.exists(cart_path):
            self.cartesian = cv2.imread(cart_path, cv2.IMREAD_GRAYSCALE)
        mask_path = osp.join(self.sensor_root, "mask", self.frame + ".png")
        if osp.exists(mask_path):
            self.mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
        return self.timestamps, self.azimuths, self.polar

    def unload_data(self):
        self.timestamps = None
        self.azimuths = None
        self.polar = None
        self.cartesian = None
        self.mask = None

    def polar_to_cart(
        self, cart_resolution, cart_pixel_width, polar=None, in_place=True
    ):
        """Converts a polar scan from polar to Cartesian format
        Args:
            cart_resolution (float): resolution of the output Cartesian image in (m / pixel)
            cart_pixel_width (int): width of the output Cartesian image in pixels
            polar (np.ndarray): if supplied, this function will use this input and not self.polar.
            in_place (bool): if True, self.cartesian is updated.
        """
        if polar is None:
            polar = self.polar
        cartesian = radar_polar_to_cartesian(
            self.azimuths, polar, self.resolution, cart_resolution, cart_pixel_width
        )
        if in_place:
            self.cartesian = cartesian
        return cartesian
    
