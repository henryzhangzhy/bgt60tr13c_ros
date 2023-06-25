#!/usr/bin/env python3

import rospy
import numpy as np
from ifxAvian import Avian



# open device: The device will be closed at the end of the block. Instead of
# the with-block you can also use:
#   device = Device()
# However, the with block gives you better control when the device is closed.
with Avian.Device() as device:
    # set device config
    config = device.get_config_defaults()
    device.set_config(config)

    # A loop for fetching a finite number of frames comes next..
    for frame_number in range(10):
        frame = device.get_next_frame()

        num_rx = np.shape(frame)[0]

        # Do some processing with the obtained frame.
        # In this example we just dump it into the console
        print ("Got frame " + format(frame_number) + ", num_antennas={}".format(num_rx))

        for iAnt in range(num_rx):
            mat = frame[iAnt, :, :]
            print("Antenna", iAnt, "\n", mat)




if __name__ == '__main__':
    rospy.init_node("radar_tr13c_node")

    print("Radar SDK Version: " + Avian.get_version())

    #radar_pub = rospy.Publisher('radar_ltr11', Complex64Array, queue_size=10)
    r = rospy.Rate(10)

    # Create Device
    device = Avian.Device()

    # Create and Set Config
    config_defaults = device.get_config_defaults()
    #print("Configuration limits", device.get_limits())

    # print("Default Configs: ", config_defaults)

    # mode: 1                           // SPI Continuous Wave Mode or SPI Pulse Mode
    # rf_frequency_Hz: 61044000000      // Operational RF Center Frequency
    # num_of_samples: 256               // Number of samples per frame
    # detector_threshold: 80            // 
    # prt: 1                            // Pulse repetition time expressed as index / enum
    # pulse_width: 0                    // Pulse width expressed as index / enum
    # tx_power_level: 7                 // 
    # rx_if_gain: 8
    # aprt_factor: 4                    // Adaptive prt factor
    # hold_time: 8
    # disable_internal_detector: False

    # Customize config
    #config_defaults.num_of_samples = 1024



    device.set_config(config_defaults)
    print("Device Configured with: ", config_defaults)


    #device.start_acquisition()


    count = 0
    data_stack = np.zeros((0, 256))
    while not rospy.is_shutdown():
        frame, metadata = device.get_next_frame() # Frame is a numpy array of shape (num_of_samples, )
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
        r.sleep()


    # Stop device and end process
    device.stop_acquisition()
    print("acquisition stopped")
    print("Sensor information: ", device.get_sensor_information())    


    
