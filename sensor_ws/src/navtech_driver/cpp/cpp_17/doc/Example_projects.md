Example projects
================

The SDK comes with a set of supplied projects.  These projects provide minimal examples of how the SDK facilities can be set up and used.  The example projects can be used as examples to build your own applications.

When built, the example project applications can be found in
```
build/<platform>/<build-type>/bin
```
Where

`<platform>` - target platform - either `linux` or `win64`

`<build-type>` - build type - either `Debug` or `Release` 

## colossus_client
This project gives an example of how to create a radar client and connect up free-function handlers for processing various message types.
In this case, the radar client requests, and receives, FFT data from the radar.  The FFT data itself is not processed, but statistics about the data are presented - packet rate, packet size and packet time.

Note, when a client connects to a radar a configuration message is always sent.  The radar client must process this message, before requesting any other types.

The `colossus_client` program has two command-line options, as follows:
```
-i - The IP address of the server   [default: 127.0.0.1] 
-p - The server port                [default: 6317]
```
If an option is not provided, its default value will be used.

## navigation_client
The navigation_client project is a sample application that will peak search and report back upto ten targets per azimuth. 

The class Peak_finder can be used to process FFT data and search for peaks.
The algorithm will sub-resolve within radar bins and return a power at a distance in metres on the azmith being checked.

The algorithm implemented here will slide a window over the FFT data moving forwards by the size of the window, when the FFT has risen and then fallen, the peak resolving algorithm is run to sub-resolve the distance.

See Peak_finder.h for the data structure that is generated per azimuth

* threshold - Threshold in dB
* bins_to_operate_on - Radar bins window size to search for peaks in
* start_bin - Start Bin
* buffer_mode - Buffer mode should only be used with a staring radar
* buffer_length - Buffer length
* max_peaks_per_azimuth - Maximum number of peaks to find in a single azimuth

The `navigation_client` program has two command-line options, as follows:
```
-i - The IP address of the server   [default: 127.0.0.1] 
-p - The server port                [default: 6317] 
```
If an option is not provided, its default value will be used.

## Connection_tester
The Connection tester project is a utility to stress-test the stability of the server.  The program creates a radar client that randomly connects and disconnects from a server.  On each connection, the radar client requests
FFT data.

The `Connection_tester` object runs in its own thread-of-control and can therefore be run in parallel.  Up to three `Connection_tester` objects can be run, all connecting/disconnecting to the same radar.  More than three `Connection_tester` objects can be running, but a server will only allow a maximum of three connections at any one time.

 The `connection_tester` program has three command-line options, as follows:
```
-i - The IP address of the server             [default: 127.0.0.1] 
-p - The server port                          [default: 6317] 
-c - Number of connections to run in parallel [default: 1] 
```
If an option is not provided, its default value will be used.

NOTE:
The connection tester can be terminated with ctrl-c.  This will send an asynchronous message to each `Connection_tester` object.  The `Connection_tester` will only act upon this signal once it has completed its current connect/disconnect sequence.  This means it may take several seconds for the program to end, and you may see additional connect/disconnects after the ctrl-c.


## cat240_client
The `cat240_client` can connect to an ASTERIX CAT-240 source (for example, the `cat240_server` project) and will receive video messages.
This project does not perform any significant processing on incoming video messages.
NOTE:
The `cat240_client` will not (currently) re-form video message packets that may have been split (by the server).

The `cat240_server` program has two command-line options:
```
-i - The IP address UDP packets are read from [default: 127.0.0.1] 
-p - The port UDP packets are read from       [default: 6317]
```

## nmea_client
This project gives a simple example for receiving NMEA messages over a UDP connection.
At present, only the following NMEA messages can be handled:

* GPGGA
* GPRMC
* GPHDT
* PASHR

The messaging classes are currently simplistic and only perform basic message processsing (validation, conversion to string/vector, etc.)
The `nmea_client` program has two command-line options:
```
-i - The IP address UDP packets are read from [default: 127.0.0.1] 
-p - The port UDP packets are read from       [default: 6317]
```


## nmea_server
This project gives a simple example for sending NMEA messages over a UDP connection.
At present, only the following NMEA messages can be handled:

* GPGGA
* GPRMC
* GPHDT
* PASHR

The messaging classes allow construction of NMEA messages from strings or from vectors of `std::uint8_t`.  There are currently no facilities for appending/removing message clauses.
The `nmea_server` program has two command-line options:
```
-i - The IP address UDP packets sent to       [default: 127.0.0.1] 
-p - The port UDP packets are sent to         [default: 6317]
```
If the IP address supplied falls into the range of multicast addresses (224.0.0.1 - 239.255.255.255) the server will be configured to multicast its NMEA messages.


## pointcloud_client
The `pointcloud_client` is used to create a UDP radar client, for receiving radar point-cloud messages via UDP.

NOTE:
The `pointcloud_client` may need to be used with a UDP proxy, if running under WSL. See the readme in the tools folder for an explanation of how to use the UDP proxy.

The `pointcloud_client` program has two command-line options:
```
-i - The IP address UDP packets are read from [default: 127.0.0.1] 
-p - The port UDP packets are read from       [default: 6317]
```

## colossus_protocol_tester
The `colossus_protocol_tester` application tests the Colossus messaging protocol between the radar and a client.  It verifies that the radar is responding to commands and emitting data correctly.
The program runs through a sequence of test cases, and presents the results.  Some tests may run for several seconds (for example, if messages are sent at periodic intervals).

The `colossuse_protocol_tester` program has three command-line options:
```
-i - The IP address UDP packets are read from       [default: 127.0.0.1] 
-p - The port UDP packets are read from             [default: 6317]
-t - The test case (specified as the message enum)  [ default: all]
```

For example:
```
$ ./colossus_protocol_tester -i 192.168.0.1 -t fft_data
```

## Point-cloud Writer
The `pointcloud_writer` application records navigation point-cloud output from a radar and writes it to a file as comma-separated variables (CSV).
The application also retrieves the radar and navigation configuration and stores it in a name-associated file.

The output from the `pointcloud_writer` is two files, the configuration and the data.  The format of the file is:
```
YYYYMMDD_HHMMSS_<tag>.cfg - configuration
YYYYMMDD_HHMMSS_<tag>.csv - data
```

For example:
```
20231123_153106_pointcloud.cfg
20231123_153106_pointcloud.csv

```
The date/time is generated automatically by the application. The `<tag>` can be specified on the command-line and could be used to identify, for example, a particular radar.

Command-line options for the `pointcloud_writer` application are as follows:
```
-i - The TCP address of the radar                                           [default: 192.168.0.1] 
-p - The TCP port of the radar                                              [default: 6317]
-u - The UDP address where point-cloud data is send (usually the host PC)   [default: 127.0.0.1]
-d - The UDP port for point-cloud data                                      [default: 6317]
-f - The file tag for identifying output files                              [default: "pointcloud"]
-m - Turn OFF configuration meta data                                       [default: create radar meta data]
```

For example:
```
$ ./pointcloud_writer -i 10.77.2.211 -p 6317 -u 10.77.2.104 -d 6317 -f school_rd 
```

This will produce two files:
```
20231123_153106_school_rd.cfg
20231123_153106_school_rd.csv
```
(NOTE - the date/time will reflect the actual time the application is run!)


```
$ ./pointcloud_writer -i 10.77.2.211 -p 6317 -u 10.77.2.104 -d 6317 -f school_rd -m 
```

This will produce only one file:
```
20231123_153106_school_rd.csv
```