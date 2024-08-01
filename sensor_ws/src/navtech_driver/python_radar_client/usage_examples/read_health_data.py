#####################################################################################################################
# Example to read a health data from the radar
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


# Create a callback for handling a health message
def handle_health_message(message):

    print("\n----------Health Message----------\n")
    print("Die Temperature:         {} \u2103".format(message.dietemperature))
    print("Soc Temperature:         {} \u2103".format(message.soctemperature))
    print("Vco Temperature:         {} \u2103".format(message.vcotemperature))
    print("Ambient Temperature:     {} \u2103".format(message.ambienttemperature))
    print("Rotation:                {} mHz".format(message.rotation))
    print("Packet Rate:             {} mHz".format(message.packetrate))
    print("Rfhealthcheck:           {} ".format(message.rfhealthcheck))
    print("Transmitting:            {} ".format(message.transmitting))
    print("Expected Rotation:       {} mHz".format(message.expectedrotation))
    print("Expected Packet Rate:    {} mHz".format(message.expectedpacketrate))
    print("Mac Address:             {} ".format(message.macaddress))
    print("Encoder Error Count:     {} ".format(message.encodererrorcount))
    print("System Uptime:           {} ".format(message.systemuptime))
    print("Motor Current:           {} ".format(message.motorcurrent))
    print("Software Uptime:         {} ".format(message.softwareuptime))
    print("Total Uptime:            {} ".format(message.totaluptime))
    print("Network State:           {} ".format(message.networkstate))
    print("Max Clients Allowed:     {} ".format(message.maxclientsallowed))
    print("IP Clients:              {} ".format(message.ipclients))
    print("Expected RX Packet Rate: {} ".format(message.expectedrxpacketrate))
    print("Uplink Errors:           {} ".format(message.uplinkerrors))
    print("Downlink Errors:         {} ".format(message.downlinkerrors))
    print("Uplink Missed:           {} ".format(message.uplinkmissed))
    print("Downloank Missed:        {} ".format(message.downlinkmissed))
    print("\n")


# Create a radar client
client = radar_client.RadarClient("192.168.0.1", 6317)

# Add a callback for a health message
# True specifies single shot - run the callback once then remove it
client.add_callback(40, handle_health_message, True)

# Connect to the client
client.connect()

# Wait for a while
sleep(2)

# Send the radar a message - in this case, start health data
client.send_message(23)

# Wait for a health data
sleep(5)

# Send the radar a message - in this case, stop health data
client.send_message(24)

# Wait for a while
sleep(2)

# Stop the radar client
client.disconnect()