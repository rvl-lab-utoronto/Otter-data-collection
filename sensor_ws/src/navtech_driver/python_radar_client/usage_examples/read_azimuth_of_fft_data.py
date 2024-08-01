#####################################################################################################################
# Example to read a single azimuth of fft data from the radar
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
import matplotlib.pyplot as plt


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

    encoder_size.value = message.encoder_size


# Create a callback for handling an FFT message
def handle_fft_message(message):

    print("\n----------FFT Data----------\n")
    print("FFT data offset:  {}".format(message.fft_data_offset))
    print("FFT data bitdepth:{}".format(message.bit_depth))
    print("Sweep counter:    {}".format(message.sweep_counter))
    print("Azimuth:          {}".format(message.azimuth))
    print("Seconds:          {}".format(message.seconds))
    print("Split seconds:    {}".format(message.split_seconds))
    print("Timestamp:        {}".format(message.timestamp))
    print("FFT data:         {}".format(message.fft_data))
    print("\n")

    # Plot the single azimuth of FFT data
    if len(message.fft_data) <= 0:
        print("No data points to plot")
        exit()
    plt.figure("One Azimuth of {}bit FFT Radar Data", figsize=(20, 10)),format(message.bit_depth)
    bearing = 360 * message.azimuth / encoder_size.value if encoder_size.value else 0
    plt.title('{}bit FFT Radar Data from bearing {} at {}'.format(message.bit_depth, round(bearing,2), message.timestamp), fontsize=12)
    plt.plot(message.fft_data, linewidth = 0.5)
    plt.ylabel('Returned power', fontsize=12)
    plt.xlabel('Reporting bin', fontsize=12)
    plt.tight_layout()
    plt.savefig('data/one_azimuth_of_fft.png')


# Create a shared variable to share (between processes) encoder size from config data message
encoder_size = Value('i', 0)

# Create a radar client
client = radar_client.RadarClient("192.168.0.1", 6317)

# Add a callback for a config message
# True specifies single shot - run the callback once then remove it
client.add_callback(10, handle_configuration_message, True)

# Add a callback for an fft message
# True specifies single shot - run the callback once then remove it
client.add_callback(30, handle_fft_message, True)

# Connect to the client
client.connect()

# Wait for a while
sleep(2)

# Send the radar a message - in this case, start fft data
client.send_message(21)

# Wait for a fft data
sleep(1)

# Send the radar a message - in this case, stop fft data
client.send_message(22)

# Wait for a while
sleep(2)

# Stop the radar client
client.disconnect()