#!/usr/bin/env python3

import rospy
import numpy as np
from ifxAvian import Avian

from std_msgs.msg import Float32MultiArray, MultiArrayDimension, StampedFloat32MultiArray

def num_rx_antennas_from_rx_mask(rx_mask):

    # popcount for rx_mask
    c = 0
    for i in range(32):
        if rx_mask & (1 << i):
            c += 1
    return c

if __name__ == '__main__':
    rospy.init_node("radar_tr13c_node")

    print("Radar SDK Version: " + Avian.get_version())

    radar_pub = rospy.Publisher('radar_tr13c', Float32MultiArray, queue_size=10)
    r = rospy.Rate(10)

    print("Radar SDK Version: " + Avian.get_version())

    # Create Device
    device = Avian.Device()
    print("Radar firmware: ", device.get_firmware_information())
    print("Sensor Information: ", device.get_sensor_information())
 
    # set device config
    config = device.get_config_defaults()
    print("Device Defaults: ", config)

    # config.sample_rate_Hz = 1000000
    # config.rx_mask = 7
    # config.tx_mask = 1
    # config.num_chirps_per_frame = 128
    # config.num_samples_per_chirp = 64


    # Configuration for "Peaking" / Presence Detection Demo
    # config = Avian.DeviceConfig(
    #     sample_rate_Hz = 1e6,                   # ADC sample rate of 1MHz
    #     rx_mask = 1,                            # RX antenna 1 activated
    #     tx_mask = 1,                            # TX antenna 1 activated
    #     tx_power_level = 31,                    # TX power level of 31
    #     if_gain_dB = 33,                        # 33dB if gain
    #     start_frequency_Hz = 59_133_931_281,    # start frequency: 59.133931281 GHz
    #     end_frequency_Hz = 62_366_068_720,      # end frequency: 62.366068720 GHz
    #     num_samples_per_chirp = 64,             # 64 samples per chirp
    #     num_chirps_per_frame = 32,              # 32 chirps per frame
    #     chirp_repetition_time_s = 0.000411238,  # Chirp repetition time (or pulse repetition time) of 411.238us
    #     frame_repetition_time_s = 0.2, # Frame repetition time default 0.2s (frame rate of 5Hz)
    #     mimo_mode = "off")  
    
    # Configuration for the "Range-Angle Map"
    config = Avian.DeviceConfig(
        sample_rate_Hz = 1_000_000,       # 1MHZ
        rx_mask = 5,                      # activate RX1 and RX3
        tx_mask = 1,                      # activate TX1
        if_gain_dB = 33,                  # gain of 33dB
        tx_power_level = 31,              # TX power level of 31
        start_frequency_Hz = 60e9,        # 60GHz 
        end_frequency_Hz = 61.5e9,        # 61.5GHz
        num_chirps_per_frame = 128,       # 128 chirps per frame
        num_samples_per_chirp = 64,       # 64 samples per chirp
        chirp_repetition_time_s = 0.0005, # 0.5ms
        frame_repetition_time_s = 0.15,   # 0.15s, frame_Rate = 6.667Hz
        mimo_mode = 'off'                 # MIMO disabled
    )
    

    print("Setting Device with Config", config)
    device.set_config(config) # Sets the config AND starts data acquisition of time domain data


    count = 0
    # data_stack = np.zeros((0, 256))
    while not rospy.is_shutdown():
        #frame, metadata = device.get_next_frame() # Frame is a numpy array of shape (num_of_samples, )
        # metadata_dict = metadata.to_dict()
        # print("Chip power mode: ", "active mode " if metadata_dict['active'] else "low power mode")
        # print("Target Detected" if (metadata_dict['motion'] == 0) else "No Target Detected")
        # if metadata_dict['motion'] == 0:
        #     print("Approaching " if metadata_dict['direction'] else " Departing")
        # print("Average Power Consumption is equal to: ", metadata_dict['avg_power'])

        # Publish
        # msg = Complex64Array()
        # msg.header.seq = count
        # count += 1
        # msg.header.stamp = rospy.Time.now()
        # msg.header.frame_id = 'radar_frame'
        # msg.real = frame.real
        # msg.imag = frame.imag

        #radar_pub.publish(msg)

        # sleep
        # Frame is shape (3, 128, 64)
        # Each frame (rows = # of virtual antennas) (Columns = chirps per frame) (slices = samples per chirp)
        frame = device.get_next_frame()
        #print("Before: ", frame[0,:,:])

        #msg = Float32MultiArray()
        msg = StampedFloat32MultiArray()

        msg.header.seq = count
        count += 1
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'radar_frame'


        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim.append(MultiArrayDimension())

        msg.layout.dim[0].label = "Receiver"
        msg.layout.dim[0].size = num_rx_antennas_from_rx_mask(config.rx_mask)
        msg.layout.dim[0].stride = num_rx_antennas_from_rx_mask(config.rx_mask)*config.num_chirps_per_frame*config.num_samples_per_chirp
        msg.layout.dim[1].label = "chirps_per_frame"
        msg.layout.dim[1].size = config.num_chirps_per_frame
        msg.layout.dim[1].stride = config.num_samples_per_chirp*config.num_chirps_per_frame
        msg.layout.dim[2].label = "samples_per_chirp"
        msg.layout.dim[2].size = config.num_samples_per_chirp
        msg.layout.dim[2].stride = config.num_samples_per_chirp

        msg.data = frame.flatten().tolist()
        #print(msg.data)

        radar_pub.publish(msg)


        # size1 = msg.layout.dim[0].size = 3
        # # msg.layout.dim[0].stride = 3*128*64
        # # msg.layout.dim[1].label = "chirps_per_frame"
        # size2 = msg.layout.dim[1].size = 128
        # # msg.layout.dim[1].stride = 64*128
        # # msg.layout.dim[2].label = "samples_per_chirp"
        # size3 = msg.layout.dim[2].size = 64
        # # msg.layout.dim[2].stride = 64

        # radar_data = np.array(msg.data).reshape(size1, size2, size3)
        # #print("After: ", radar_data[0,:,:])
        # if (np.allclose(frame, radar_data)):
        #     print("All are close, good!")
        # else:
        #     print("Bad!")

        r.sleep()


    # Stop device and end process
    device.stop_acquisition()
    print("acquisition stopped")
    print("Sensor information: ", device.get_sensor_information())    


    
