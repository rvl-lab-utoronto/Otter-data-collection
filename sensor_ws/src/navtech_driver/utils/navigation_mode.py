#####################################################################################################################
# A python class to perform navigation mode functions
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

# Navigation mode class
class NavigationMode:
    
    # Constructor
    def __init__(self, bins_to_operate_upon):
        self.awaiting_rise = True
        self.bins_to_operate_upon = bins_to_operate_upon

    # Raise a numer to the power 2
    def square(self, number):
        return number ** 2

    # Raise a numer to the power 3
    def cube(self, number):
        return number ** 3

    # Find the peak bin
    def find_peak_bin(self, data, start_bin, end_bin, threshold):
        if (start_bin > (end_bin - self.bins_to_operate_upon)):
            return end_bin 
        for peak_bin in range (start_bin , end_bin - self.bins_to_operate_upon):    
            if (self.awaiting_rise and (data[peak_bin] > data[peak_bin + 1])):
                continue
            elif (self.awaiting_rise):
                self.awaiting_rise = False
            if ((data[peak_bin] > threshold) and (data[peak_bin + 1] < data[peak_bin])) :
                self.awaiting_rise = True
                return peak_bin
        return end_bin 

    # Resolve the peak
    def peak_resolve(self, data, peak_bin):
        x=[0] * self.bins_to_operate_upon
        x_2=[0] * self.bins_to_operate_upon
        x_3=[0] *self.bins_to_operate_upon
        x_4=[0] * self.bins_to_operate_upon
        y=[0] * self.bins_to_operate_upon
        x_y=[0] * self.bins_to_operate_upon
        x_2_y=[0] * self.bins_to_operate_upon
        bins_to_offset = int((self.bins_to_operate_upon - 1) / 2 )   
        index = 0    
        start_value = peak_bin - bins_to_offset    
        for index in range (0, self.bins_to_operate_upon):
            x[index] = start_value + index
        start_bin = peak_bin - bins_to_offset 
        for index in range (0, self.bins_to_operate_upon):
            y[index] = data[start_bin + index]    
        s_x =  0.0
        s_x_2 = 0.0
        s_x_3 = 0.0
        s_x_4 = 0.0    
        s_x=sum(x[0:self.bins_to_operate_upon])
        s_x_2=sum(list(map(self.square,x[0:self.bins_to_operate_upon])))
        x_3=list(map(self.cube,x[0:self.bins_to_operate_upon]))
        s_x_3=sum(x_3[0:self.bins_to_operate_upon])
        x_2=list(map(self.square,x[0:self.bins_to_operate_upon]))
        x_4=list(map(self.square,x_2[0:self.bins_to_operate_upon]))
        s_x_4=sum(x_4[0:self.bins_to_operate_upon])
        s_y =   0.0
        s_x_y =  0.0
        s_x_2_y = 0.0    
        s_y=sum(y[0:self.bins_to_operate_upon])
        x_y=[X*Y for X,Y in zip(x[0:self.bins_to_operate_upon],y[0:self.bins_to_operate_upon])]
        s_x_y=sum(x_y[0:self.bins_to_operate_upon])
        x_2_y=[X*Y for X,Y in zip(x_2[0:self.bins_to_operate_upon],y[0:self.bins_to_operate_upon])]
        s_x_2_y=sum(x_2_y[0:self.bins_to_operate_upon])
        A = [s_x_2, s_x_3, s_x_4, s_x_2_y]
        B =[s_x, s_x_2, s_x_3, s_x_y]  
        C = [ self.bins_to_operate_upon, s_x, s_x_2, s_y ]   
        F = C[0] / A[0]    
        for index in range (0,4):
            C[index] = C[index] - (F * A[index])    
        F = B[0] / A[0]    
        for index in range (0,4):
            B[index] = B[index] - (F * A[index])    
        F = C[1] / B[1]    
        for index in range (0,4):        
            C[index] -= F * B[index]    
        b2 = C[3] / C[2]    
        b1 = (B[3] - B[2] * b2) / B[1]    
        return -b1 / (2 * b2)