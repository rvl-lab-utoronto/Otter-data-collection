#####################################################################################################################
# Example to read a single azimuth of navigation data from the radar
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

# Create a callback for handling a navigation config
def handle_navigation_config_message(message):

    print("\n----------Navigation Config----------\n")
    print("Bins to operate on:      {}".format(message.bins_to_operate_on))
    print("Minimum bin:             {}".format(message.minimum_bin))
    print("Navigation threshold:    {}".format(message.navigation_threshold))
    print("Max peaks per azimuth:   {}".format(message.max_peaks_per_azimuth))
    print("\n")

# Create a callback for handling navigation data
def handle_navigation_data_message(message):

    print("\n----------Navigation Data----------\n")
    print("Azimuth:                 {}".format(message.azimuth))
    print("Seconds:                 {}".format(message.seconds))
    print("Split Seconds:           {}".format(message.split_seconds))
    print("Num navigation pairs:    {}".format(len(message.navigation_data_pairs)))
    print("Navigation pairs data:")
    for p in range(0, len(message.navigation_data_pairs)):
        print("\tRange, Power:        {}, {}".format(message.navigation_data_pairs[p][0], message.navigation_data_pairs[p][1]))
    print("\n")


# Create a radar client
client = radar_client.RadarClient("192.168.0.1", 6317)

# Add a callback for a navigation config message
client.add_callback(204, handle_navigation_config_message)

# Add a callback for a navigation data message
# True specifies single shot - run the callback once then remove it
client.add_callback(123, handle_navigation_data_message, True)

# Connect to the client
client.connect()

# Wait for a while
sleep(5)

# Send the radar a message - in this case, navigation configuration update
bins_to_operate_on = 10
minimum_bin = 1
navigation_threshold = 15
max_peaks_per_azimuth = 10
client.send_message(205, bins_to_operate_on, minimum_bin, navigation_threshold, max_peaks_per_azimuth)

# Wait for a while
sleep(5)

# Send the radar a message - in this case, start navigation data
client.send_message(120)

# Wait for a while
sleep(10)

# Send the radar a message - in this case, stop navigation data
client.send_message(121)

# Wait for a while
sleep(5)

# Wait for a while
sleep(5)

# Stop the radar client read thread
client.disconnect()