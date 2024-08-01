#####################################################################################################################
# A python nmea client. Used to manage the connection to a nmea sensor, and send/receive the most common nmea message types
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
from utils import logger
import socket
from time import sleep
from multiprocessing import Queue, Process, Event
from message_handling import messages_from_nmea
import copy


# Nmea client class
class NmeaClient:

    # Constructor
    def __init__(self, ip, port, log_directory = "logs", log_filename = "nmea_client_log"):
        self.ip = ip
        self.port = port

        self.log = logger.Logger(log_directory, log_filename)

        self.connection_state = "DISCONNECTED"
        self.client_socket = None
        self.client_socket_timeout = 10.0
        self.incoming_nmea_message_queue = Queue(maxsize = 10)

        self.payload_data = ""

        self.log.write_to_log("Created nmea client")


    # Connection handling FSM
    def connection_handling_fsm(self):

        # Execute connection handling in a loop
        self.log.write_to_log("Connection handling FSM thread started")
        while True:

            # Check for stop event
            if self.event.is_set():
                break

            if self.connection_state == "CONNECTED":
    
                # Attempt to read data
                # If success stay here
                # If fail, move to disconnected
                try:
                    self.handle_messages()
                except:
                    self.log.write_to_log("Disconnected from: {}:{}".format(self.ip, self.port))
                    self.log.write_to_log("Attempting reconnection to: {}:{}".format(self.ip, self.port))
                    self.connection_state = "DISCONNECTED"

            # Run connection handling code
            elif self.connection_state == "DISCONNECTED":

                # Attempt to open the port
                # If success, move to connected
                # If fail, stay here
                try:
                    sleep(2)
                    self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.client_socket.settimeout(self.client_socket_timeout)
                    self.client_socket.connect((self.ip, self.port))
                    self.log.write_to_log("Created connection to: {}:{}".format(self.ip, self.port))
                    self.connection_state = "CONNECTED"
                except:
                    self.connection_state = "DISCONNECTED"

            else:
                self.log.write_to_log("UNKNOWN STATE")

        self.log.write_to_log("Connection handling FSM thread stopped")


    # Handle the actual reading of a message
    def handle_messages(self):

        # Try to read an incoming message
        try:
            self.payload_data = self.payload_data + bytes(self.client_socket.recv(50)).decode("utf-8") 
            start_index = self.payload_data.find("$")
            end_index = self.payload_data.find("\r\n")
            if (start_index >= 0 and end_index >= 0 and end_index > start_index):
                nmea_string = self.payload_data[start_index:end_index+1]
                self.incoming_nmea_message_queue.put(messages_from_nmea.MessageFromNmea(copy.deepcopy(nmea_string)))
                self.payload_data = self.payload_data[end_index+1:]

        except Exception as e:
            self.log.write_to_log("Error reading message - {}".format(e))
            pass


    # Process the response message
    def process_received_message(self):

        # Execute message dispatching in a loop
        self.log.write_to_log("Message dispatching thread started")
        while True:

            # Check for stop event
            if self.event.is_set():
                break

            # Read message type
            if (self.incoming_nmea_message_queue.qsize() > 0):
                try:

                    self.latest_incoming_message = self.incoming_nmea_message_queue.get()
                    self.latest_incoming_message.process_message()
                except:
                    pass

        self.log.write_to_log("Message dispatching thread stopped")


    # Connect to the nmea and start reading messages
    def connect(self):

        # Create an event to signal thread shutdown
        self.event = Event()

        # Create a message dispatching thread
        self.message_dispatch_thread = Process(target=self.process_received_message, args=())
        self.message_dispatch_thread.start()

        # Create a read thread
        self.message_processing_thread = Process(target=self.connection_handling_fsm, args=())
        self.message_processing_thread.start()


    # Stop reading messages and disconnect from the nmea sesnor
    def disconnect(self):

        # Set the event to shutdown threads
        self.event.set()

        # Stop the message dispatching thread
        self.message_dispatch_thread.join()

        # Stop the read thread
        # This also closes the socket
        self.message_processing_thread.join()

    # Add a callback to the message type map
    def add_callback(self, message_type, callback_function, single_shot = False):
        messages_from_nmea.MessageFromNmea.register_callback(message_type, callback_function, single_shot)
        self.log.write_to_log("Registered{}callback function ({}) for message type {}".format(" (single shot) " if single_shot else " " , callback_function.__name__, message_type))
    
    # Remove a callback from the message type map
    def remove_callback(self, message_type):
        messages_from_nmea.MessageFromNmea.remove_callback(message_type)
        self.log.write_to_log("Removed callback for message type {}".format(message_type))