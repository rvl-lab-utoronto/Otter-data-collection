#####################################################################################################################
# A python colossus message handling class. Used to handle messages to the radar
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

# Radar messages class for messages to the radar
class MessageToRadar(base_radar_message.BaseRadarMessage):

    # This maps a message type (to the radar) to it's string equivalent
    message_to_radar_type_map =  {
                            1 :   "Keep Alive",
                            20 :  "Configuration Request",
                            21 :  "Start FFT Data",
                            22 :  "Stop FFT Data",
                            23 :  "Start Health Msgs",
                            24 :  "Stop Health Msgs",
                            25 :  "Reset RF Health",
                            50 :  "Contour Update",
                            51 :  "Sector Blanking Update",
                            76 :  "System Restart",
                            90 :  "Logging Levels",
                            100 : "Logging Levels Request",
                            120 : "Start Nav Data",
                            121 : "Stop Nav Data",
                            122 : "Set Nav Threshold",
                            124 : "Set Nav Range Gain and Offset",
                            125 : "Calibrate Accelerometer",
                            126 : "Start Accelerometer",
                            127 : "Stop Accelerometer",
                            144 : "Navigation Area Rules",
                            203 : "Navigation Configuration Request",
                            205 : "Set Navigation Configuration",
                            207 : "Request Time Server Status",
                            209 : "Start radar rotation and transmitter",
                            210 : "Stop radar transmitter and rotation",
                        }


    # Constructor
    def __init__(self, message_type, *args):
        self.message_type = message_type
        try:
            self.message_type_alias = MessageToRadar.message_to_radar_type_map[self.message_type]
        except:
            self.message_type_alias = "Unknown"
        self.version = base_radar_message.BaseRadarMessage.version

        # Parse some common message types

        # Set navigation configuration
        if self.message_type == 205:
            self.payload_size = 12
            self.payload = bytearray(base_radar_message.BaseRadarMessage.check_signature_bytes) + \
            int.to_bytes(self.version, 1, byteorder='big') + \
            int.to_bytes(self.message_type, 1, byteorder='big')  + \
            int.to_bytes(self.payload_size, 4, byteorder='big')  + \
            int.to_bytes(args[0], 2, byteorder='big')   + \
            int.to_bytes(args[1], 2, byteorder='big')   + \
            struct.pack('>f', args[2] * 10)      + \
            int.to_bytes(args[3], 4, byteorder='big')
   
        # General message with no payload
        else:
            self.payload_size = 0
            self.payload = bytearray(base_radar_message.BaseRadarMessage.check_signature_bytes) + \
            int.to_bytes(self.version, 1, byteorder='big') + \
            int.to_bytes(self.message_type, 1, byteorder='big')  + \
            int.to_bytes(self.payload_size, 4, byteorder='big')