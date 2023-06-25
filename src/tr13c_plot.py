#!/usr/bin/env python3

#import rospy
#from bgt60ltr11aip_ros.msg import Complex64Array
#from std_msgs.msg import Float32

from ifxDopplerLTR11 import DopplerLTR11
import matplotlib.pyplot as plt
import numpy as np

from matplotlib.animation import FuncAnimation
from functools import partial

def plot_callback(msg):
    print("Getting new frame")

    real = np.array(msg.real)
    imag = np.array(msg.imag)

    frame = real + imag * 1j
    if frame.shape[0] >= 100:
        plot1(frame)



def plot1(frame):
    print(frame.shape)
    global fig, ax, line
    # Get the latest raw data in real-time
    raw_data = frame  # Replace with your function to obtain the latest raw data

    # Perform range processing to obtain range data
    range_data = np.fft.fft(raw_data)   # Range FFT
    print(range_data.shape)

    # Convert the data to dB scale for visualization
    range_dB = 20 * np.log10(np.abs(range_data))

    # Update the plot data
    line.set_data(np.arange(0,256), range_dB)

    # Adjust the plot limits if needed
    ax.set_xlim(0, len(range_dB))
    ax.set_ylim(np.min(range_dB), np.max(range_dB))

    # Redraw the plot
    fig.canvas.draw()


if __name__ == '__main__':
    # Enable interactive mode
    plt.ion()
    plt.switch_backend('agg')
    global range_dB

    # Create the figure and axis
    fig, ax = plt.subplots()
    line, = ax.plot([], [])
    ax.set_xlabel('Range Bins')
    ax.set_ylabel('Magnitude (dB)')
    ax.set_title('Range Profile')
    ax.grid(True)

    rospy.init_node("radar_ltr11_plots")
    radar_sub = rospy.Subscriber('radar_ltr11', Complex64Array, plot_callback)
   
    # ------------ Method 1 --------------- #
    # ----------- Range Map --------------- #






    rospy.spin()

    
