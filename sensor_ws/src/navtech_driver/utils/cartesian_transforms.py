#####################################################################################################################
# Python functions to perform basic cartesian transforms
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
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname( __file__ ), '../..')))
from utils import cartesian_transforms
import math

# Functions
def rotate_point_around_x_axis(point, angle):

    new_point = [0, 0, 0]
    theta = math.radians(angle)
    new_point[0] = point[0]
    new_point[1] = point[1] * math.cos(theta) - point[2] * math.sin(theta)
    new_point[2] = point[1] * math.sin(theta) + point[2] * math.cos(theta)
    return new_point

# Use this one to rotate the radar data
def rotate_point_around_y_axis(point, angle):

    new_point = [0, 0, 0]
    theta = math.radians(angle)
    new_point[0] = point[0] * math.cos(theta) + point[2] * math.sin(theta)
    new_point[1] = point[1]
    new_point[2] = point[2] * math.cos(theta) - point[0] * math.sin(theta)
    return new_point

def rotate_point_around_z_axis(point, angle):

    new_point = [0, 0, 0]
    theta = math.radians(angle)
    new_point[0] = point[0] * math.cos(theta) - point[1] * math.sin(theta)
    new_point[1] = point[0] * math.sin(theta) + point[1] * math.cos(theta)
    new_point[2] = point[2]
    return new_point