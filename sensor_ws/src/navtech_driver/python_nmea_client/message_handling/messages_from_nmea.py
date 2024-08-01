#####################################################################################################################
# A python nmea message handling class. Used to handle messages from the nmea sensor
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
from message_base import base_nmea_message
from messages_from_nmea import pashr_message

# Nmea messages class for messages from the nmea sensor
class MessageFromNmea(base_nmea_message.BaseNmeaMessage):

    # This maps a message type (from the nmea sensor) to it's string equivalent
    message_from_nmea_type_map =  {
                            1 :   "PASHR",
                        }

    # This stores callback handles for the above message types
    message_from_nmea_handler_map =  {
                            1 :   (None, False),
                        }


    # Register a callback function in the message handler map
    def register_callback(message_type, function, single_shot):
        MessageFromNmea.message_from_nmea_handler_map.update({message_type:(function, single_shot)})


    # Remove a callback function from the message handler map
    def remove_callback(message_type):
        MessageFromNmea.message_from_nmea_handler_map.update({message_type:(None, False)})


    # Constructor
    def __init__(self, payload_data):
        message_type_string = payload_data[1:payload_data.find(',')]

        if (message_type_string == "PASHR"):
            self.message_type = 1
        else:
            self.message_type = 0

        self.payload_data = payload_data


    # Process a message
    def process_message(self):

        # Parse some common message types
        if (self.message_type == 1):
            message = pashr_message.PashrMessage(self.payload_data)
        else:
            message = base_nmea_message.BaseNmeaMessage(self.payload_data)

        function_data = MessageFromNmea.message_from_nmea_handler_map[self.message_type]
        function = function_data[0]
        single_shot_function_call = function_data[1]
        if function is not None:
            function(message)
        if single_shot_function_call:
            MessageFromNmea.remove_callback(self.message_type)