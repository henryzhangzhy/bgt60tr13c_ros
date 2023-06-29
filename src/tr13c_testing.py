#!/usr/bin/env python3

import numpy as np
from ifxAvian import Avian
#import ifxAlgo
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

    config.sample_rate_Hz = 1000000
    config.rx_mask = 7
    config.tx_mask = 1
    config.num_chirps_per_frame = 128
    config.num_samples_per_chirp = 64


    print("Setting Device with Config", config)
    device.set_config(config) # Sets the config AND starts data acquisition of time domain data




    # Create a figure and axis for plotting
    fig, ax = plt.subplots()

    # Set up the plot
    line, = ax.plot([], [])  # Empty line for updating data in real-time

    # Customize the plot
    ax.set_xlim(0, 64)
    ax.set_ylim(0, 1)  # Assuming radar frame values are between 0 and 1

    # Initialize empty arrays to store frame data
    #frame_data = np.empty((0, 64))
    frame_counter = 0



    # Animation loop
    while True:
        # Get a new frame (replace this with your actual frame acquisition code)
        frame = device.get_next_frame()

        # Update the plot with the first channel of the frame
        line.set_data(np.arange(frame.shape[0]), frame[0, :])

        # Update the title with the frame number
        ax.set_title(f"Frame {frame_counter+1}")

        # Append the first channel of the frame to the frame_data array
        #frame_data = np.vstack((frame_data, frame[0, :]))

        # Update and display the plot
        plt.draw()
        plt.pause(0.1)  # Pause for a short interval (adjust as needed)

        # Check for termination condition (you can modify this based on your requirements)
        if frame_counter >= 9:
            break

        frame_counter += 1




    # Stop device and end process
    device.stop_acquisition()
    print("acquisition stopped")
    print("Sensor information: ", device.get_sensor_information())   




    
