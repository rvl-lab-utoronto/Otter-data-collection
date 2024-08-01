#####################################################################################################################
# A python colossus message handling class. Used to handle a configuration message from the radar
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
from message_base import base_radar_message
import struct

# Radar configuration message
class ConfigurationMessage(base_radar_message.BaseRadarMessage):

    # Constructor
    def __init__(self, signature_data, header_data, payload_data):
        self.signature_data = signature_data
        self.header_data = header_data
        self.payload_data = payload_data

        # Standard configuration message data
        self.azimuth_samples = int.from_bytes(self.payload_data[0:2], byteorder='big')
        self.bin_size = int.from_bytes(self.payload_data[2:4], byteorder='big')
        self.range_in_bins = int.from_bytes(self.payload_data[4:6], byteorder='big')
        self.encoder_size = int.from_bytes(self.payload_data[6:8], byteorder='big')
        self.rotation_speed = int.from_bytes(self.payload_data[8:10], byteorder='big')
        self.packet_rate = int.from_bytes(self.payload_data[10:12], byteorder='big')
        self.range_gain = struct.unpack('f', bytes(self.payload_data[12:16][::-1]))[0]
        self.range_offset = struct.unpack('f', bytes(self.payload_data[16:20][::-1]))[0]