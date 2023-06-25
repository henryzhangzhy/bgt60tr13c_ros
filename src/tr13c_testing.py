#!/usr/bin/env python3

import numpy as np
from ifxAvian import Avian
import ifxAlgo
import time

import matplotlib.pyplot as plt


if __name__ == '__main__':
    print("Radar SDK Version: " + Avian.get_version())

    # Create Device
    device = Avian.Device()
    print("Radar firmware: ", device.get_firmware_information())
    print("Sensor Information: ", device.get_sensor_information())
 
    # set device config
    config = device.get_config_defaults()
    print("Device Defaults: ", config)

    print("Setting Device Config")
    device.set_config(config) # Sets the config AND starts data acquisition of time domain data



    print("Getting data")
    # A loop for fetching a finite number of frames comes next..
    for frame_number in range(10):

        # Each frame (rows = # of virtual antennas) (Columns = chirps per frame) (slices = samples per chirp)
        frame = device.get_next_frame()
        print("Got frame with shape: ", frame.shape)
        print("and type: ", frame.dtype)

        num_rx = np.shape(frame)[0]

        # Do some processing with the obtained frame.
        # In this example we just dump it into the console
        print ("Got frame " + format(frame_number) + ", num_antennas={}".format(num_rx))

        for iAnt in range(num_rx):
            mat = frame[iAnt, :, :]
            print("Antenna", iAnt, "\n", mat)





    # Close the device now that we are done
    device.stop_acquisition()
    device.destroy()




    
