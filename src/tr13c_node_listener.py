#!/usr/bin/env python3

import rospy
import numpy as np
from ifxAvian import Avian

from std_msgs.msg import Float32MultiArray, MultiArrayDimension


def radar_callback(msg):
    # Need to reshape message back into the normal dimensions to make sure it looks the same
    # msg.layout.dim[0].label = "Receiver"
    size1 = msg.layout.dim[0].size = 3
    # msg.layout.dim[0].stride = 3*128*64
    # msg.layout.dim[1].label = "chirps_per_frame"
    size2 = msg.layout.dim[1].size = 128
    # msg.layout.dim[1].stride = 64*128
    # msg.layout.dim[2].label = "samples_per_chirp"
    size3 = msg.layout.dim[2].size = 64
    # msg.layout.dim[2].stride = 64

    radar_data = np.array(msg.data).reshape(size1, size2, size3)
    #print(radar_data)

if __name__ == '__main__':
    rospy.init_node("radar_tr13c_node_listener")
    radar_sub = rospy.Subscriber('radar_tr13c', Float32MultiArray, radar_callback)

    rospy.spin()


    
