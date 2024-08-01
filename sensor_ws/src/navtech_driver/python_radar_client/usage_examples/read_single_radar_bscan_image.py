#####################################################################################################################
# Example to read a rotation of fft data from the radar, and save it as a bscan image
#####################################################################################################################

# ---------------------------------------------------------------------------------------------------------------------
# Copyright 2023 Navtech Radar Limited
# This file is part of IASDK which is released under The MIT License (MIT).
# See file LICENSE.txt in project root or go to https:#opensource.org/licenses/MIT
# for full license details.
#
# Disclaimer:
# Navtech Radar is furnishing this item "as is". Navtech Radar does not provide 
# any warranty of the item whatsoever, whether express, implied, or statutory,
# including, but not limited to, any warranty of merchantability or fitness
# for a particular purpose or any warranty that the contents of the item will
# be error-free.
# In no respect shall Navtech Radar incur any liability for any damages, including,
# but limited to, direct, indirect, special, or consequential damages arising
# out of, resulting from, or any way connected to the use of the item, whether
# or not based upon warranty, contract, tort, or otherwise; whether or not
# injury was sustained by persons or property or otherwise; and whether or not
# loss was sustained from, or arose out of, the results of, the item, or any
# services that may be provided by Navtech Radar.
# ---------------------------------------------------------------------------------------------------------------------

# Imports
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..')))
from client import radar_client
from time import sleep
import numpy as np
import cv2

# Create a callback for handling a radar bscan image
def handle_radar_bscan_image(message):

    print("\n----------Bscan image----------\n")
    print("Shape:      {}".format(message.image_data.shape))
    print("Timestamp:  {}".format(message.timestamp))
    cv2.imwrite('data/radar_bscan_image.png', np.dstack([np.zeros_like(message.image_data), message.image_data, np.zeros_like(message.image_data)]))        
    print("\n")


# Create a radar client
client = radar_client.RadarClient("192.168.0.1", 6317)

# Add a callback for a radar bscan image
# True specifies single shot - run the callback once then remove it
client.add_callback(256, handle_radar_bscan_image, True)

# Connect to the client
client.connect()

# Wait for a while
sleep(5)

# Send the radar a message - in this case, start fft data
client.send_message(21)

# Wait for a while
sleep(10)

# Send the radar a message - in this case, stop fft data
client.send_message(22)

# Wait for a while
sleep(5)

# Remove the callback for a radar bscan image
client.remove_callback(256)

# Stop the radar client
client.disconnect()