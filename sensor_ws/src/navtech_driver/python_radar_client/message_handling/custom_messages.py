#####################################################################################################################
# A python colossus message handling class. Used to handle custom messages from the radar
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
from messages_from_radar import configuration_message
from messages_from_radar import fft_message_8_bit
from messages_from_radar import navigation_data_message
from custom_message_types import radar_bscan_image_message
from custom_message_types import radar_cartesian_image_message
from custom_message_types import navigation_rotation_message
import numpy as np
import cv2

# Radar messages class for custom messages
class CustomMessages():

    rotations_completed = 0
    radar_config_message = None
    radar_bscan_data = None
    cartesian_image_size = 800
    radar_navigation_data = []
    has_rotated_once = False
    previous_azimuth = 0
    
    # This maps a custom message type to it's string equivalent
    custom_functions__type_map =  {
                            256 :   "Radar bscan image",
                            257 :   "Radar cartesian image",
                            258 :   "Rotation of navigation data"
                        }
    
    # This stores callback handles for additional functions
    custom_functions_handler_map =  {
                            256 :   (None, False),
                            257 :   (None, False),
                            258 :   (None, False)
                        }

    # Check if the radar has rotated at least once
    def rotated_once(azimuth):
        if (CustomMessages.has_rotated_once):
            return True
        if (azimuth <= CustomMessages.previous_azimuth):
            CustomMessages.has_rotated_once = True
        CustomMessages.previous_azimuth = azimuth
        return CustomMessages.has_rotated_once

    # Check if the radar has completed another full rotation
    def completed_full_rotation(azimuth):
        if (not CustomMessages.rotated_once(azimuth)):
            return False
        has_completed_rotation = False
        if (azimuth <= CustomMessages.previous_azimuth):
            has_completed_rotation = True
        CustomMessages.previous_azimuth = azimuth
        return has_completed_rotation


    # Register a callback function in the custom message handler map
    def register_callback(message_type, function, single_shot):
        CustomMessages.custom_functions_handler_map.update({message_type:(function, single_shot)})


    # Remove a callback function from the custom message handler map
    def remove_callback(message_type):
        CustomMessages.custom_functions_handler_map.update({message_type:(None, False)})


    # Process a custom message
    def process_custom_message(message):
        
        # Create a bscan or cartesian image, if enabled
        if CustomMessages.custom_functions_handler_map[256][0] is not None or CustomMessages.custom_functions_handler_map[257][0] is not None:
            if (CustomMessages.radar_config_message == None):
                if type(message) == configuration_message.ConfigurationMessage:
                    CustomMessages.radar_config_message = message
                    CustomMessages.radar_bscan_data = np.zeros((message.azimuth_samples, message.range_in_bins), dtype=np.uint8)
            if type(message) == fft_message_8_bit.FftMessage8Bit:
                if (CustomMessages.radar_config_message != None):
                    azimuth_index = int(message.azimuth / CustomMessages.radar_config_message.encoder_size * CustomMessages.radar_config_message.azimuth_samples)
                    CustomMessages.radar_bscan_data[azimuth_index, :len(message.fft_data)] = message.fft_data
                    if (CustomMessages.completed_full_rotation(message.azimuth)):
                        if CustomMessages.rotations_completed < 4:
                            CustomMessages.rotations_completed += 1
                            return
                        if CustomMessages.custom_functions_handler_map[256][0] is not None:
                            bscan_message = radar_bscan_image_message.RadarBscanImageMessage(CustomMessages.radar_bscan_data)
                            CustomMessages.custom_functions_handler_map[256][0](bscan_message)
                        if CustomMessages.custom_functions_handler_map[256][1] == True:
                            CustomMessages.remove_callback(256)
                        if CustomMessages.custom_functions_handler_map[257][0] is not None:
                            data_array_scaled = cv2.resize(CustomMessages.radar_bscan_data, ((CustomMessages.cartesian_image_size,CustomMessages.cartesian_image_size)), cv2.INTER_AREA)
                            polar_data_array =  cv2.linearPolar(data_array_scaled, (int(data_array_scaled.shape[1] / 2), int(data_array_scaled.shape[0] / 2)), int(data_array_scaled.shape[0] / 2), cv2.WARP_FILL_OUTLIERS + cv2.WARP_INVERSE_MAP)
                            cartesian_message = radar_cartesian_image_message.RadarCartesianImageMessage(polar_data_array)
                            CustomMessages.custom_functions_handler_map[257][0](cartesian_message)
                        if CustomMessages.custom_functions_handler_map[257][1] == True:
                            CustomMessages.remove_callback(257)

        
        # Collect together a rotation of navigation data, if enabled
        if CustomMessages.custom_functions_handler_map[258][0] is not None:
            if (CustomMessages.radar_config_message == None):
                if type(message) == configuration_message.ConfigurationMessage:
                    CustomMessages.radar_config_message = message
                    CustomMessages.radar_navigation_data = [None] * message.azimuth_samples
            if type(message) == navigation_data_message.NavigationDataMessage:
                if (CustomMessages.radar_config_message != None):
                    azimuth_index = int(message.azimuth / CustomMessages.radar_config_message.encoder_size * CustomMessages.radar_config_message.azimuth_samples)
                    CustomMessages.radar_navigation_data[azimuth_index] = message.navigation_data_pairs
                    if (CustomMessages.completed_full_rotation(message.azimuth)):
                        if CustomMessages.rotations_completed < 4:
                            CustomMessages.rotations_completed += 1
                            return
                        if CustomMessages.custom_functions_handler_map[258][0] is not None:
                            navigation_rotation_message_data = navigation_rotation_message.NavigationRotationMessage(CustomMessages.radar_navigation_data)
                            CustomMessages.custom_functions_handler_map[258][0](navigation_rotation_message_data)
                            if CustomMessages.custom_functions_handler_map[258][1] == True:
                                CustomMessages.remove_callback(258)