#####################################################################################################################
# A python colossus message handling class. Used to handle a health message from the radar
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
from protobuf import health_pb2

# Radar health message
class HealthMessage(base_radar_message.BaseRadarMessage):

    # Constructor
    def __init__(self, signature_data, header_data, payload_data):
        self.signature_data = signature_data
        self.header_data = header_data
        self.payload_data = payload_data

        # Parse the health protobuf data
        message=health_pb2.Health()
        message.ParseFromString(bytearray(self.payload_data))

        # Health message protobuf data
        self.dietemperature = message.dietemperature.value
        self.soctemperature = message.soctemperature.value
        self.vcotemperature = message.vcotemperature.value
        self.ambienttemperature = message.ambienttemperature.value
        self.rotation = message.rotation.value
        self.packetrate = message.packetrate.value
        self.rfhealthcheck = message.rfhealthcheck.value
        self.transmitting = message.transmitting
        self.expectedrotation = message.expectedrotation
        self.expectedpacketrate = message.expectedpacketrate
        self.macaddress = message.macaddress
        self.encodererrorcount = message.encodererrorcount
        self.systemuptime = message.systemuptime
        self.motorcurrent = message.motorcurrent.value
        self.softwareuptime = message.softwareuptime
        self.totaluptime = message.totaluptime
        self.networkstate = message.networkstate.state
        self.maxclientsallowed = message.maxclientsallowed
        self.ipclients = message.ipclients
        self.expectedrxpacketrate = message.expectedrxpacketrate
        self.uplinkerrors = message.uplinkerrors
        self.downlinkerrors = message.downlinkerrors
        self.uplinkmissed = message.uplinkmissed
        self.downlinkmissed = message.downlinkmissed