#####################################################################################################################
# A python colossus message handling class. Used to handle a high precision FFT message from the radar
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
import datetime
import numpy as np

# Radar FFT message
class FftMessage16Bit(base_radar_message.BaseRadarMessage):

    # Constructor
    def __init__(self, signature_data, header_data, payload_data):
        self.signature_data = signature_data
        self.header_data = header_data
        self.payload_data = payload_data

        # Standard FFT message data
        self.bit_depth = 16
        self.fft_data_offset = int.from_bytes(self.payload_data[0:2], byteorder='big')
        self.sweep_counter = int.from_bytes(self.payload_data[2:4], byteorder='big')
        self.azimuth = int.from_bytes(self.payload_data[4:6], byteorder='big')
        self.seconds = int.from_bytes(self.payload_data[6:10], byteorder='little')
        self.split_seconds = int.from_bytes(self.payload_data[10:14], byteorder='little')
        raw_fft_data = self.payload_data[self.fft_data_offset:]
        converted_fft_data = []
        for f in range(0, len(raw_fft_data), 2):
            converted_fft_data.append(int.from_bytes(raw_fft_data[f:f+2], byteorder='big'))
        self.fft_data = np.asarray(converted_fft_data, dtype=np.uint16)

        # Extra FFT message data
        self.timestamp = "{}.{}".format(datetime.datetime.fromtimestamp(self.seconds).strftime('%d-%m-%Y %H:%M:%S.%f'), 
                                        str(int(round(self.split_seconds/10000,0))).rjust(5,'0'))