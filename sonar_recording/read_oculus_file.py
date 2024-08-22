#! /usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import argparse
import datetime

from oculus_python.files import OculusFileReader


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog='OculusFileReader',
        description='Example of how to read and display the content of a .oculus ' +
                    'file. This will display the first ping from a the file.')
    parser.add_argument('filename', type=str,
                        help='Path to a .oculus file to display')
    args = parser.parse_args()

    print('Opening', args.filename)

    f = OculusFileReader(args.filename)
    msg = f.read_next_ping()
    sonar_stamps = [] 
    count = 0
    while msg is not None:
        msg = f.read_next_ping() # this can be called several time to iterate through the pings
                                 # will return None when finished
        count += 1
        if msg is None:
            print('File seems to be empty. Aborting.')
            break
        #print(msg.metadata())
        
        timestamp_seconds = msg.timestamp_micros() / 1e6
        formatted_timestamp = datetime.datetime.utcfromtimestamp(timestamp_seconds)
        formatted_timestamp_str = formatted_timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        print(formatted_timestamp_str)
        
        sonar_stamps.append(np.float64(msg.timestamp_micros())*1e3)

        print("timestamp",           formatted_timestamp_str)
        print("timestamp_micros",    msg.timestamp_micros())
        print("ping_index",          msg.ping_index())
        print("range",               msg.range())
        print("gain_percent",        msg.gain_percent())
        print("frequency",           msg.frequency())
        print("speed_of_sound_used", msg.speed_of_sound_used())
        print("range_resolution",    msg.range_resolution())
        print("temperature",         msg.temperature())
        print("pressure",            msg.pressure())

        bearings     = 0.01*np.array(msg.bearing_data())
        linearAngles = np.linspace(bearings[0], bearings[-1], len(bearings))
        rawPingData = np.array(msg.raw_ping_data())
        gains = np.ones([msg.range_count(),], dtype=np.float32)
        if msg.has_gains():
            gains = np.array(msg.gains())
        pingData = np.array(msg.ping_data()) / np.sqrt(gains)[:,np.newaxis]

        print('has gains   :', msg.has_gains())
        print('sample size :', msg.sample_size())
        print('ping shape  :', pingData.shape)
    print(sonar_stamps)        
    sonar_stamps = np.array(sonar_stamps)
    np.save("sonar_stamps", sonar_stamps)
