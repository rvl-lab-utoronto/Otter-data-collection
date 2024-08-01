#####################################################################################################################
# Python code to test receiving of nmea pashr messages
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
from client import nmea_client
from time import sleep

# Create a callback for handling a pashr message
def handle_pashr_message(message):

    print("\n----------PASHR Message----------\n")
    print("UTC time:      {}".format(message.utc_time))
    print("Roll:          {}".format(message.roll))
    print("Pitch:         {}".format(message.pitch))

# Create a nmea client
client = nmea_client.NmeaClient("10.77.7.76", 9095)

# Add a handler for a pashr message
client.add_callback(1, handle_pashr_message)

# Connect to the client
client.connect()

# Wait for a while
sleep(10)

# Stop the nmea client
client.disconnect()