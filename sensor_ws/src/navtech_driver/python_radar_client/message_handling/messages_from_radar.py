#####################################################################################################################
# A python colossus message handling class. Used to handle messages from the radar
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
from messages_from_radar import keepalive_message
from messages_from_radar import configuration_message
from messages_from_radar import fft_message_8_bit
from messages_from_radar import fft_message_16_bit
from messages_from_radar import health_message
from messages_from_radar import navigation_configuration_message
from messages_from_radar import navigation_data_message
from message_handling import custom_messages

# Radar messages class for messages from the radar
class MessageFromRadar(base_radar_message.BaseRadarMessage):

    # This maps a message type (from the radar) to it's string equivalent
    message_from_radar_type_map =  {
                            1 :   "Keep Alive",
                            10 :  "Configuration",
                            30 :  "FFT Data",
                            31 :  "High Precision FFT Data",
                            40 :  "Health",
                            90 :  "Logging Levels",
                            123 : "Navigation Data",
                            128 : "Accelerometer Data",
                            143 : "Navigation Alarm Data",
                            204 : "Navigation Configuration",
                            208 : "Time Server Status",
                        }

    # This stores callback handles for the above message types
    message_from_radar_handler_map =  {
                            1 :   (None, False),
                            10 :  (None, False),
                            30 :  (None, False),
                            31 :  (None, False),
                            40 :  (None, False),
                            90 :  (None, False),
                            123 : (None, False),
                            128 : (None, False),
                            143 : (None, False),
                            204 : (None, False),
                            208 : (None, False),
                        }


    # Register a callback function in the message handler map
    def register_callback(message_type, function, single_shot):
        if message_type < 256:
            MessageFromRadar.message_from_radar_handler_map.update({message_type:(function, single_shot)})
        else:
            custom_messages.CustomMessages.register_callback(message_type, function, single_shot)


    # Remove a callback function from the message handler map
    def remove_callback(message_type):
        if message_type < 256:
            MessageFromRadar.message_from_radar_handler_map.update({message_type:(None, False)})
        else:
            custom_messages.CustomMessages.remove_callback(message_type)


    # Constructor
    def __init__(self, signature_data, header_data, payload_data):
        self.signature_data = signature_data
        self.header_data = header_data
        self.version = self.header_data[0]
        self.message_type = self.header_data[1]
        try:
            self.message_type_alias = MessageFromRadar.message_from_radar_type_map[self.message_type]
        except:
            self.message_type_alias = "Unknown"
        self.payload_size = int.from_bytes(self.header_data[2:6], byteorder='big')
        self.payload_data = payload_data


    # Process a message
    def process_message(self):

        # Parse some common message types
        if (self.message_type == 1):
            message = keepalive_message.KeepaliveMessage(self.signature_data, self.header_data, self.payload_data)
        if (self.message_type == 10):
            message = configuration_message.ConfigurationMessage(self.signature_data, self.header_data, self.payload_data)
        elif (self.message_type == 30):
            message = fft_message_8_bit.FftMessage8Bit(self.signature_data, self.header_data, self.payload_data)
        elif (self.message_type == 31):
            message = fft_message_16_bit.FftMessage16Bit(self.signature_data, self.header_data, self.payload_data)
        elif (self.message_type == 40):
            message = health_message.HealthMessage(self.signature_data, self.header_data, self.payload_data)
        elif (self.message_type == 123):
            message = navigation_data_message.NavigationDataMessage(self.signature_data, self.header_data, self.payload_data)
        elif (self.message_type == 204):
            message = navigation_configuration_message.NavigationConfigurationMessage(self.signature_data, self.header_data, self.payload_data)
        else:
            message = base_radar_message.BaseRadarMessage(self.signature_data, self.header_data, self.payload_data)

        # Process the message in custom messages class
        custom_messages.CustomMessages.process_custom_message(message)

        function_data = MessageFromRadar.message_from_radar_handler_map[self.message_type]
        function = function_data[0]
        single_shot_function_call = function_data[1]
        if function is not None:
            function(message)
        if single_shot_function_call:
            MessageFromRadar.remove_callback(self.message_type)