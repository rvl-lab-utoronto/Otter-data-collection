# Python 3 Radar SDK

This folder contains a Python 3 Radar SDK. Which facilitates the conenction to a radar, bi directional message handling, and examples of common functionality of the SDK.

--------------------------------------------------------------------------------------------------------------------

# Requirements

This version of the SDK was developed on Ubuntu 22.04, using Python version 3.11.3. The SDK has been designed with backwards compatability in mind, so previous Python versions may also work.

Additional Python package requirements to run all SDK code and examples:

matplotlib==3.5.3
numpy==1.17.4
opencv_python==4.6.0.66
protobuf==3.19.4

--------------------------------------------------------------------------------------------------------------------

# Directory structure
.<br>
├── client<br>
│   ├── __init__.py<br>
│   └── radar_client.py<br>
├── custom_message_types<br>
│   ├── __init__.py<br>
│   ├── navigation_rotation_message.py<br>
│   ├── radar_bscan_image_message.py<br>
│   └── radar_cartesian_image_message.py<br>
├── LICENSE.txt<br>
├── message_base<br>
│   └── base_radar_message.py<br>
├── message_handling<br>
│   ├── custom_messages.py<br>
│   ├── __init__.py<br>
│   ├── messages_from_radar.py<br>
│   └── messages_to_radar.py<br>
├── messages_from_radar<br>
│   ├── configuration_message.py<br>
│   ├── fft_message_16_bit.py<br>
│   ├── fft_message_8_bit.py<br>
│   ├── health_message.py<br>
│   ├── __init__.py<br>
│   ├── navigation_configuration_message.py<br>
│   └── navigation_data_message.py<br>
├── protobuf<br>
│   ├── healthinfo_pb2.py<br>
│   ├── health_pb2.py<br>
│   ├── __init__.py<br>
│   └── networkinfo_pb2.py<br>
├── README.md<br>
├── usage_examples<br>
│   ├── cfar_peak_detection_one_azimuth_fft_data.py<br>
│   ├── data<br>
│   │   └── one_azimuth_of_fft_data.csv<br>
│   ├── navigation_mode_peak_detection_one_azimuth_fft_data.py<br>
│   ├── read_azimuth_of_fft_data.py<br>
│   ├── read_health_data.py<br>
│   ├── read_rotation_of_navigation_data.py<br>
│   ├── read_single_azimuth_of_navigation_data.py<br>
│   ├── read_single_radar_bscan_image.py<br>
│   ├── read_single_radar_cartesian_image.py<br>
│   ├── test_cfar.py<br>
│   └── test_connect_disconnect_radar.py<br>
└── utils<br>
    ├── cfar.py<br>
    ├── __init__.py<br>
    └── navigation_mode.py<br>

--------------------------------------------------------------------------------------------------------------------

# client
Contains the actual radar client class, which is multi process and handles connection to the radar, as well as message reading and sending.

# custom_message_types
Contains class definitions for custom messages, which are not based on standard radar (Colossus) messages, and are unique to the Python radar SDK.

# message_base
Contains the base class from which the standard radar (Colossus) messages inherit from.

# message_handling
Contains message handling functions, to handle messages to the radar, from the radar, and the generation of custom message types.

# messages_from_radar
Contains implementations of the standard radar (Colossus) messages.

# protobuf
Contains the protobuf definitions for serialised health and network data sent from the radar. These define how health and network information from the radar is parsed.

# usage_examples
Contains examples of how to use the key functionality that the Python radar SDK provides.

# utils
Contains utility functions for use with the radar data. Includes peak detection and CFAR

--------------------------------------------------------------------------------------------------------------------

# Explanation of key SDK functionality

The below worked example shows how to import the radar client into your Python script and use it to

## Importing the SDK

The Python radar SDK can be imported into your project from another location by appending the location of the SDK to your Python system path. The following example allows the Python SDK from location iasdk/python_radar_client to be accessed from an example python script within the examples folder at iasdk/python_radar_client/usage_examples.

```
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..')))
```

## Creating a radar client object

A radar client object can be created by calling the RadarClient constructor from the radar_client package, and specifying the IP address and port of the radar to connect to.

```
client = radar_client.RadarClient("192.168.0.1", 6317)
```

## Connecting to a radar

A connection to a radar can be established by calling connect on the radar_client object instance.

```
client.connect()
```

## Reading messages from the radar

The SDK itself handles the raw message reading, and deliveres a radar message based on it's Colossus message type specification. See https://navtechradar.atlassian.net/wiki/spaces/PROD/pages/2261516289/TCP+Networking for the full list of message types and definitions. To receive a message from the Radar in your own code you must attach a message handling function to the Colossus message type. The radar client will then run your message handling function as a callback, upon receiving the speficifed message type from the radar. You attach your function to a message type by calling the add_callback function on your client object instance, specifying the message type that you want to handle (in this case health), along with the name of the function that you want to call (handle_health_message) to handle the message type.

Note - the optional parameter which is set to True below, specifies whether the function should be run in single shot mode. Single shot mode allows the callback to run only once. Leaving the optional parameter out will default to False, and hence the callback function will run continuously, for every message of that type which is received from the radar.

```
client.add_callback(40, handle_health_message, True)
```

## Stopping the receiving of messages from the radar

The SDK will always receive messages from the radar if it is configured to send them, however you can prevent messages from being received into your Python code. Calling remove_callback on the client object instance and specifying the Colossus message type will detach any registered callback function from that message type. The example code below shows how to detach the handle_health_message function (which was attached in the example above) and prevent it being called when a Colossus message type 40 (health data) is receivd into the SDK. You do not need to specify the function name to detach, as this is already known within the SDK from the message type that it was previously attached to.

Note - If the single shot flag was set to True when the callback function was registered, then there is no need to remove the callback, as this will have already been done internally upon receiving the message type (as long as a message of that type was sucessfully received).

```
client.remove_callback(40)
```

## Sending messages to the radar

The SDK allows you to send messages directly to the radar, by specifying the Colossus message type that you would like to send. There are two variations on the messages that you can send to the radar - these are messages with no payload and messages with a payload.

### Messages with no payload

To send a message with no payload you call the send_message function on the client object instance, and specify the Colossus message type that you want to send to the radar. The SDK handles the formatting of the message and the sending of the raw bytes to the radar. In the below example the Colossus message type is signalling the radar to start sending out health data to the client.

```
client.send_message(23)
```

### Messages with payload

To send a message with a payload you call the send_message function on the client object instance, specify the Colossus message type that you want to send to the radar, and any parameters that the message type requires. The SDK handles the formatting of the message (along with parameters) and the sending of the raw bytes to the radar. In the below example the Colossus message type is signalling the radar to update it's onboard navigation mode paramters, with the values that have been provided.

```
max_range_of_interest = 200
bins = 5
min_bins = 3
nav_threshold = 55
max_peaks = 10
client.send_message(205, bins, min_bins, nav_threshold, max_peaks)
```

## Disconnecting from a radar

A connection to a radar can be relinquished by calling disconnect on the radar_client object instance.

```
client.disconnect()
```

--------------------------------------------------------------------------------------------------------------------

# Example projects
# See "usage_examples" folder
The SDK comes with eight example projects, designed to demonstrate the key functionality of the Python radar SDK.

## cfar_peak_detection_one_azimuth_fft_data
This example does not connect directly to a radar. It demonstrates how to read fft data from a file, and how to apply the cell averaging CFAR algorithm to the data, to detect high return values.

## navigation_mode_peak_detection_one_azimuth_fft_data
This example does not connet directly to a radar. It demonstrates how to read fft data from a file, and how to manually resolve peaks (targets) from the fft data.

## test_connect_disconnect_radar
The most basic usage example - demonstrates how to connect to a radar, handle keepalive messages, and disconnect from the radar.

## read_health_data
This example demonstrates how to connect to a radar and read a health data message from it, including how to decode the protobuf health message.

## read_single_azimuth_of_navigation_data
This example demonstrates how to connect to a radar and read one azimuth of navigation mode data from it.

## read_rotation_of_navigation_data
This example demonstrates how to connect to a radar and read an entire rotation of navigation mode data from it.

## read_azimuth_of_fft_data
This example demonstrates how to connect to a radar and read one azimuth of fft data from it. The fft data is saved as a graph in .png format.

## read_single_radar_bscan_image
This example demonstrates how to connect to a radar and read an entire rotation of fft data, saving it as a bscan style image. This style of image represents the radar data as a 2 dimensional image of size number of azimuths * number of bins.

## read_single_radar_cartesian_image
This example demonstrates how to connect to a radar and read an entire rotation of fft data, saving it as a cartesian style image. This style of image represents the radar data in a true to life coordinate system.