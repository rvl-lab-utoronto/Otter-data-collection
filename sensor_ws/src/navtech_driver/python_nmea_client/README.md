# Python 3 NMEA SDK

This folder contains a Python 3 NMEA GPS Sensor SDK. Which facilitates the conenction to a NMEA sensor, bi directional message handling, and examples of common functionality of the SDK.

--------------------------------------------------------------------------------------------------------------------

# Requirements

This version of the SDK was developed on Ubuntu 22.04, using Python version 3.11.3. The SDK has been designed with backwards compatability in mind, so previous Python versions may also work.

Additional Python package requirements to run all SDK code and examples:

numpy==1.17.4

--------------------------------------------------------------------------------------------------------------------

# Directory structure
.<br>
├── client<br>
│   ├── __init__.py<br>
│   └── nmea_client.py<br>
├── message_base<br>
│   ├── base_nmea_message.py<br>
│   └── __init__.py<br>
├── message_handling<br>
│   ├── __init__.py<br>
│   └── messages_from_nmea.py<br>
├── messages_from_nmea<br>
│   ├── __init__.py<br>
│   └── pashr_message.py<br>
└── usage_examples<br>
    └── test_nmea_pashr_messages.py<br>

--------------------------------------------------------------------------------------------------------------------

# client
Contains the actual NMEA client class, which is multi process and handles connection to the NMEA sensor, as well as message reading and sending.

# message_base
Contains the base class from which the standard NMEA sensor messages inherit from.

# message_handling
Contains message handling functions, to handle messages to and from the NMEA sensor.

# messages_from_nmea
Contains implementations of the standard NMEA messages.

# usage_examples
Contains examples of how to use the key functionality that the Python NMEA sensor SDK provides.

--------------------------------------------------------------------------------------------------------------------

# Explanation of key SDK functionality

The below worked example shows how to import the NMEA sensor client into your Python script and use it to

## Importing the SDK

The Python NMEA sensor SDK can be imported into your project from another location by appending the location of the SDK to your Python system path. The following example allows the Python SDK from location iasdk/python_nmea_client to be accessed from an example python script within the examples folder at iasdk/python_nmea_client/usage_examples.

```
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..')))
```

## Creating a NMEA client object

A NMEA client object can be created by calling the NmeaClient constructor from the nmea_client package, and specifying the IP address and port of the NMEA sensor to connect to.

```
client = nmea_client.NmeaClient("192.168.0.1", 6317)
```

## Connecting to a NMEA sensor

A connection to a NMEA sensor can be established by calling connect on the nmea_client object instance.

```
client.connect()
```

## Reading messages from the NMEA sensor

The SDK itself handles the raw message reading, and deliveres a NMEA sensor message based on it's NMEA message type specification. To receive a message from the NMEA sensor in your own code you must attach a message handling function to the NMEA sensor message type. The NMEA client will then run your message handling function as a callback, upon receiving the speficifed message type from the NMEA sensor. You attach your function to a message type by calling the add_callback function on your client object instance, specifying the message type that you want to handle (in this case PASHR), along with the name of the function that you want to call (handle_pashr_message) to handle the message type.

Note - the optional parameter which is set to True below, specifies whether the function should be run in single shot mode. Single shot mode allows the callback to run only once. Leaving the optional parameter out will default to False, and hence the callback function will run continuously, for every message of that type which is received from the NMEA sensor.

```
client.add_callback(1, handle_pashr_message, True)
```

## Stopping the receiving of messages from the NMEA sensor

The SDK will always receive messages from the NMEA sensor if it is configured to send them, however you can prevent messages from being received into your Python code. Calling remove_callback on the client object instance and specifying the NMEA sensor message type will detach any registered callback function from that message type. The example code below shows how to detach the handle_pashr_message function (which was attached in the example above) and prevent it being called when a NMEA sensor message type 1 (pashr data) is receivd into the SDK. You do not need to specify the function name to detach, as this is already known within the SDK from the message type that it was previously attached to.

Note - If the single shot flag was set to True when the callback function was registered, then there is no need to remove the callback, as this will have already been done internally upon receiving the message type (as long as a message of that type was sucessfully received).

```
client.remove_callback(1)
```

## Disconnecting from a NMEA sensor

A connection to a NMEA sensor can be relinquished by calling disconnect on the nmea_client object instance.

```
client.disconnect()
```

--------------------------------------------------------------------------------------------------------------------

# Example projects
# See "usage_examples" folder
The SDK comes with a single example projects, designed to demonstrate the receiving of a NMEA sensor message.

## test_nmea_pashr_messages
This example connects to the NMEA sensor and reads PASHR messages from it.