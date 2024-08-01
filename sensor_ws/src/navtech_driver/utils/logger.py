#####################################################################################################################
# A python class to handle debug/error logging
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
import os
import glob
from datetime import datetime
import inspect
from threading import Lock

# Radar messages class for messages to the radar
class Logger():

    # Static mutex to lock logging file
    file_write_lock = Lock()

    # Constructor
    def __init__(self, log_directory, log_filename):
        
        self.size_limit_bytes = 1048576
        self.log_directory = log_directory
        self.log_filename = log_filename
        self.log_number = 1
        self.set_up_logging()

    
    # Set up the logging
    def set_up_logging(self):

        # Create a logs folder if it doesn't already exist
        if (not os.path.isdir(self.log_directory)):
            os.mkdir(self.log_directory)

        # Check for the latest log file in the logs folder
        log_files = glob.glob("{}/*{}*".format(self.log_directory, self.log_filename))
        if len(log_files) > 0:
            log_file_numbers = [int(log_file[-7:-4]) for log_file in log_files]
            self.log_number = max(log_file_numbers)

        print("Latest log file: {}/{}_{}.txt".format(self.log_directory, self.log_filename, str(self.log_number).zfill(3)))
    
    
    # Write to log file
    def write_to_log(self, message):

        # Start a new log file if log size is large
        log_file_size = 0
        with Logger.file_write_lock:
            if os.path.isfile("{}/{}_{}.txt".format(self.log_directory, self.log_filename, str(self.log_number).zfill(3))):
                log_file_size = os.path.getsize("{}/{}_{}.txt".format(self.log_directory, self.log_filename, str(self.log_number).zfill(3)))
        if (log_file_size >= self.size_limit_bytes):
            self.log_number += 1
            print("Increased log number: {}".format(self.log_number))
            print("Latest log file: {}/{}_{}.txt".format(self.log_directory, self.log_filename, str(self.log_number).zfill(3)))

        # Get caller details for the log
        stack = inspect.stack()
        calling_class = stack[1][0].f_locals["self"].__class__.__name__
        calling_method = stack[1][0].f_code.co_name

        # Write the message to the log file
        with Logger.file_write_lock:
            with open("{}/{}_{}.txt".format(self.log_directory, self.log_filename, str(self.log_number).zfill(3)), "a") as log_file:
                current_time = datetime.now().strftime("%d.%m.%Y_%H:%M:%S.%f")
                string_to_write = "{} - {} - {}({})\n".format(current_time, message, calling_class, calling_method)
                log_file.write(string_to_write)
