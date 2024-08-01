#####################################################################################################################
# A python function to perform cell averaging CFAR
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
import numpy as np

# Perform cell averaging CFAR and return indexes of peak(s)
def get_peak_indexes(data, num_train, num_guard, rate_fa = 1.0):
    data = np.asarray(data).flatten()
    data_shape = data.shape[0]
    num_train_half = int(round(num_train / 2))
    num_guard_half = int(round(num_guard / 2))
    start_index = num_train_half + num_guard_half
    stop_index = data_shape - num_train_half - num_guard_half - 1
    peak_indexes = []

    for test_index in range(start_index, stop_index):
        train_cell_indexes = list(range(test_index - num_guard_half - num_train_half, test_index - num_guard_half)) + list(range(test_index + num_guard_half + 1, test_index + num_guard_half + num_train_half + 1))
        neighbour_cell_indexes = list(range(test_index - num_guard_half - num_train_half, test_index)) + list(range(test_index + 1, test_index + num_guard_half + num_train_half + 1))
        test_threshold = np.mean(data[train_cell_indexes]) * rate_fa
        if data[test_index] >= test_threshold and np.all(data[test_index] > data[neighbour_cell_indexes]):
            peak_indexes.append(test_index)

    return peak_indexes 