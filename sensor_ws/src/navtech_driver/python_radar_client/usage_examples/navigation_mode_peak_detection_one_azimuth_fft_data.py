#####################################################################################################################
# A python example to perform navigation mode peak detection on one azimuth of fft data
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
import matplotlib.pyplot as plt
import numpy as np
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..')))
from utils import navigation_mode

########################################################################
# The below settings need to be changed to match your setup
########################################################################
bins_to_operate_on    = 15          # The original Navtech c++ code sets this between 5 & 15
start_bin             = 10          # Allows for bins really close-in to be ignored
threshold             = 85          # 85 is the value that one customer uses for target identification
range_gain            = 1           # Radar specific value to scale the reported range-in-metres 
range_offset          = 0           # Radar specific parameter to offset the reported range-in-metres
range_resolution_metres = 0.044     # This value is rounded from the /real/ value to go in here!
max_peaks_per_azimuth = 99          # maximum number of peaks 
##########################################################################


########################################################################
# The below section contains the main program code
########################################################################

# Create the navigation mode peak detector
peak_detector = navigation_mode.NavigationMode(bins_to_operate_on)

# Open the file containing the FFT data
data = []
filename = "data/one_azimuth_of_fft_data.csv"
try:
    data = np.genfromtxt(filename, delimiter=',')
except Exception as e:
    print("Cannot open data file {} - {}".format(filename, e))
    exit()

print ("\n######### SETTINGS #########")
print ("Bins to operate on: {}".format(bins_to_operate_on))
print ("Threshold:       {}".format(threshold))
print ("Range resolution: {}".format(range_resolution_metres))
print ("Range gain:      {}".format(range_gain))
print ("Range offset:    {}".format(range_offset))
print ("Max peaks:       {}".format(max_peaks_per_azimuth))

end_bin = len(data)
max_bins_to_operate_on = end_bin
minimum_range = bins_to_operate_on * range_resolution_metres
maximum_range = end_bin * range_resolution_metres
peaks_found = 0
peak_bin = 0
min_bin_to_operate_on = 0
min_bin_to_operate_upon = start_bin

# Plot the data
if len(data) <= 0:
    print("No data points to plot")
    exit()
plt.figure("One Azimuth Analysis - Navigation Mode peak detection")
plt.title("Single Azimuth of FFT Radar Data - 'Navigation Mode' Peak Detection\nPeakBins marked in orange, ResolvedPeaks marked in green")
plt.plot(data, ".", markersize=5, color='b')
plt.hlines(threshold, start_bin, len(data), color='red')
while ((peak_bin != end_bin) and (peaks_found < max_peaks_per_azimuth)):
    peak_bin = peak_detector.find_peak_bin(data, min_bin_to_operate_upon, end_bin, threshold)
    min_bin_to_operate_upon = peak_bin + bins_to_operate_on
    if (peak_bin < end_bin):
        print ("\n####### PEAK FOUND #########")
        print ("Peak bin found:  {}".format(peak_bin))
        plt.plot(peak_bin, data[peak_bin],".-.", color='orange', markersize=12)
        resolved_bin = peak_detector.peak_resolve(data, peak_bin)
        print ("resolved bin at: {:.3f}".format(resolved_bin))
        resolved_range = (resolved_bin * range_gain * range_resolution_metres) + range_offset
        print ("Resolved range:  {:.3f}m".format(resolved_range))
        if ((resolved_range < minimum_range) or (resolved_range > maximum_range)):
            print ("Implausible resolved range")
            continue
        peaks_found += 1
        plt.vlines(resolved_bin, min(data), max(data), color='green')
print ("\n############################")
plt.ylabel('Returned power', fontsize=12)
plt.xlabel('Reporting bin', fontsize=12)
plt.tight_layout()
plt.savefig("data/one_azimuth_of_fft_data_navigation_mode_peaks.png")