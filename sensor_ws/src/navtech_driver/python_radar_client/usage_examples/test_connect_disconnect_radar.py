#####################################################################################################################
# Example to test connection and disconnection from a radar
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

# Create a callback for handling a keep alive message
def handle_keepalive_message(message):
    print("Keepalive Received, payload length {}\n".format(len(message.payload_data)))



# Create a radar client
client = radar_client.RadarClient("192.168.0.1", 6317)

# Add a callback for a configuration message
client.add_callback(10, handle_configuration_message)

# Add a callback for a keepalive message
client.add_callback(1, handle_keepalive_message)

# Connect to the client
client.connect()

# Wait for a while
sleep(10)

# Remove the callback for a configuration message
client.remove_callback(10)

# Remove the callback for a keepalive message
client.remove_callback(1)

# Stop the radar client
client.disconnect()