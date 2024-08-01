#####################################################################################################################
# A python colossus message handling class. Used to handle a navigation data message from the radar
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

# Navigation data message
class NavigationDataMessage(base_radar_message.BaseRadarMessage):

    # Constructor
    def __init__(self, signature_data, header_data, payload_data):
        self.signature_data = signature_data
        self.header_data = header_data
        self.payload_data = payload_data

        # Standard navigation data message data
        self.azimuth = int.from_bytes(self.payload_data[0:2], byteorder='big')
        self.seconds = int.from_bytes(self.payload_data[2:6], byteorder='big')
        self.split_seconds = int.from_bytes(self.payload_data[6:10], byteorder='big')
        num_navigation_pairs = int((len(self.payload_data) - 10) / 6)
        self.navigation_data_pairs = []
        nav_pairs_start_index = 10
        for p in range(0, num_navigation_pairs):
            nav_pair_range = int.from_bytes(self.payload_data[nav_pairs_start_index:nav_pairs_start_index+4], byteorder='big') / 1000000
            nav_pair_power = int.from_bytes(self.payload_data[nav_pairs_start_index+4:nav_pairs_start_index+6], byteorder='big') / 10
            self.navigation_data_pairs.append((nav_pair_range, nav_pair_power))
            nav_pairs_start_index += 6