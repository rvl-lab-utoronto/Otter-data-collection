#####################################################################################################################
# Example to read an antire rotation of navigation data from the radar
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
from multiprocessing import Value
import numpy as np
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import math


########################################################################
# The below settings can be changed to alter navigation mode behaviour
########################################################################
max_range_of_interest = 200     # in metres: this value is used to discard points beyond a certain range
                                # usually, a customer will set the radar's operating range through the radar's web ui

bins = 5                        # number of bins to operate on (window size) for peakfinding

min_bins = 3                    # closest bin to consider within the peak finding.
                                # this allows for close-in radar returns to be ignored

nav_threshold = 50              # power threshold used to configure the operation of the 
                                # navigationmode. note that setting this value too low can 
                                # result in the processing effort required from the radar
                                # exceeding that which is available. this will result in
                                # points appearing to be "missing" from ranges of azimuths
                                # in the returned dataset

max_peaks = 10                  # the number of peaks per azimuth that navigationmode should report
########################################################################

# Create a callback for handling a configuration message
def handle_configuration_message(message):

    print("\n----------Config Message----------\n")
    print("Azimuth Samples:  {} samples/rotation".format(message.azimuth_samples))
    print("Bin Size:         {} mm/bin".format(message.bin_size/10))
    print("Range in Bins:    {} bins".format(message.range_in_bins))
    print("Encoder Size:     {} counts/rotation".format(message.encoder_size))
    print("Rotation Speed:   {} mHz".format(message.rotation_speed))
    print("Packet Rate:      {} azimuths/second".format(message.packet_rate))
    print("Range Gain:       {} ".format(message.range_gain))
    print("Range Offset:     {} m".format(message.range_offset))
    print("\n")

    azimuth_samples.value = message.azimuth_samples

# Create a callback for handling a navigation config
def handle_navigation_config_message(message):

    print("\n----------Navigation Config----------\n")
    print("Bins to operate on:      {}".format(message.bins_to_operate_on))
    print("Minimum bin:             {}".format(message.minimum_bin))
    print("Navigation threshold:    {}".format(message.navigation_threshold / 10))
    print("Max peaks per azimuth:   {}".format(message.max_peaks_per_azimuth))
    print("\n")

# Create a callback for handling a rotation of navigation data
def handle_rotation_of_navigation_data_message(message):

    print("\n----------Navigation Data----------\n")
    print("Timestamp:                {}".format(message.timestamp))
    print("Num azimuths:             {}".format(len([x for x in message.navigation_data if x is not None])))

    # Save peaks data to file
    range_list = []
    bearing_list = []
    power_list = []

    with open("data/navigation_data.csv",'w') as target:
        target.write("azimuth,range(m),bearing(degrees),power(db)\n")
        for x in range (len(message.navigation_data)):
            if (message.navigation_data[x] != None):
                for nav_pair in message.navigation_data[x]:
                    bearing = 360 * (x / azimuth_samples.value)
                    bearing_list.append(math.radians(bearing))
                    peak_range_metres = nav_pair[0]
                    range_list.append(peak_range_metres)
                    peak_power_db = nav_pair[1]
                    power_list.append(peak_power_db)
                    if (peak_range_metres <= max_range_of_interest):
                        target.write("{},{},{},{}\n".format(x, peak_range_metres, bearing, peak_power_db))

    # Save peaks data as a plot
    print ("Peaks indentified: {}".format(len(power_list)))
    if len(power_list) <= 0:
        print("No data points to plot")
        print("\n")
        return
    fig = plt.figure("One rotation of navigation mode data")
    fig.set_size_inches(9.,10.)
    ax = fig.add_subplot(projection='polar')
    ax.set_ylim(0,max_range_of_interest)
    c = ax.scatter(bearing_list, range_list, s=18, c=power_list, cmap=cm.rainbow,vmin=min(power_list), vmax=max(power_list))
    ax.set_title('Navigation Mode Data\n binstooperateon={}, minbin={}, threshold={}, maxpeaks={}'.format(bins, min_bins, nav_threshold, max_peaks))
    ax.set_theta_direction(-1)
    ax.set_theta_offset(np.pi / 2.0)
    plt.tight_layout()
    plt.savefig("data/navigation_data.png")

# Create shared variables to share (between processes) needed data from the config message
azimuth_samples = Value('i', 0)

# Create a radar client
client = radar_client.RadarClient("192.168.0.1", 6317)

# Add a callback for a config message
# True specifies single shot - run the callback once then remove it
client.add_callback(10, handle_configuration_message, True)

# Add a callback for a navigation config message
client.add_callback(204, handle_navigation_config_message, True)

# Add a callback for handling a rotation of navigation data
# True specifies single shot - run the callback once then remove it
client.add_callback(258, handle_rotation_of_navigation_data_message, True)

# Connect to the client
client.connect()

# Wait for a while
sleep(2)

# Send the radar a message - in this case, a navigation configuration update
client.send_message(205, bins, min_bins, nav_threshold, max_peaks)

# Wait for a while
sleep(2)

# Send the radar a message - in this case, request the navigation configuration from the radar
client.send_message(203)

# Wait for a while
sleep(2)

# Send the radar a message - in this case, start navigation data
client.send_message(120)

# Wait for an entire rotation of navigation data
sleep(8)

# Send the radar a message - in this case, stop navigation data
client.send_message(121)

# Remove the callback for a navigation config message
client.remove_callback(204)

# Wait for a while
sleep(2)

# Stop the radar client
client.disconnect()