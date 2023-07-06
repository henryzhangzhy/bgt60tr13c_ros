#!/usr/bin/env python3

import rospy
import numpy as np
from ifxAvian import Avian

from std_msgs.msg import Float32MultiArray, MultiArrayDimension

import argparse
from ifxAvian import Avian
from examples.internal.fft_spectrum import *
import numpy as np
from scipy import signal
from collections import namedtuple

class PresenceAntiPeekingAlgo:
    def __init__(self, num_samples_per_chirp, num_chirps_per_frame):
        """Presence and Anti-Peeking Algorithm
        
        This is a simple use case of an a presence detection and
        anti-peeking demo.
        
        Parameters:
        num_samples_per_chirp: Number of samples per chirp
        """
        self.num_samples_per_chirp = num_samples_per_chirp
        self.num_chirps_per_frame = num_chirps_per_frame

        # Algorithm Parameters
        self.detect_start_sample = num_samples_per_chirp//8
        self.detect_end_sample = num_samples_per_chirp//2
        self.peek_start_sample = num_samples_per_chirp//2
        self.peek_end_sample = num_samples_per_chirp

        self.threshold_presence = 0.0007
        self.threshold_peeking = 0.0006

        self.alpha_slow = 0.001
        self.alpha_med = 0.05
        self.alpha_fast = 0.6

        # Initialize state
        self.presence_status = False
        self.peeking_status = False
        self.first_run = True
        
        # Use Blackmann-Harris as window function
        self.window = signal.blackmanharris(num_samples_per_chirp).reshape(1,num_samples_per_chirp)

    def presence(self, mat):
        """Run the presence and anti-peeking algorithm on the current frame.
        
        Parameters:
            - mat: Radar data for one antenna as returned by Frame.get_mat_from_antenna
        
        Returns:
            - Tuple consisting of the state for presence detection and peeking.
              The first bool indicates if a target was detected. The second bool
              indicates if peeking was detected.
        """
        # copy values into local variables to keep names short
        alpha_slow = self.alpha_slow
        alpha_med = self.alpha_med
        alpha_fast = self.alpha_fast

        # Compute range FFT
        range_fft = fft_spectrum(mat, self.window)

        # Average absolute FFT values over number of chirps
        fft_spec_abs = abs(range_fft)
        fft_norm = np.divide(fft_spec_abs.sum(axis=0), self.num_chirps_per_frame)

        # Presence sensing
        if self.first_run: # initialize averages
            self.slow_avg = fft_norm
            self.fast_avg = fft_norm
            self.slow_peek_avg = fft_norm
            self.first_run = False

        if self.presence_status == False:
            alpha_used = alpha_med
        else:
            alpha_used = alpha_slow

        self.slow_avg = self.slow_avg*(1-alpha_used) + fft_norm*alpha_used
        self.fast_avg = self.fast_avg*(1-alpha_fast) + fft_norm*alpha_fast
        data = self.fast_avg-self.slow_avg

        self.presence_status = np.max(data[self.detect_start_sample:self.detect_end_sample]) > self.threshold_presence

        # Peeking sensing
        if self.peeking_status == False:
            alpha_used = self.alpha_med
        else:
            alpha_used = self.alpha_slow

        self.slow_peek_avg = self.slow_peek_avg*(1-alpha_used) + fft_norm*alpha_used
        data_peek = self.fast_avg-self.slow_peek_avg

        self.peeking_status = np.max(data_peek[self.peek_start_sample:self.peek_end_sample]) > self.threshold_peeking
        
        return namedtuple("state", ["presence", "peeking"])(self.presence_status, self.peeking_status)




import pprint
import matplotlib.pyplot as plt
import numpy as np
import cv2

from ifxAvian import Avian
from examples.internal.fft_spectrum import *
from examples.DBF import DBF
from examples.doppler import DopplerAlgo

def num_rx_antennas_from_config(config):
    rx_mask = config.rx_mask

    # popcount for rx_mask
    c = 0
    for i in range(32):
        if rx_mask & (1 << i):
            c += 1
    return c


class LivePlot:
    def __init__(self, max_angle_degrees : float, max_range_m : float):
        # max_angle_degrees: maximum supported speed
        # max_range_m:   maximum supported range
        self.h = None
        self.max_angle_degrees = max_angle_degrees
        self.max_range_m = max_range_m

        self._fig, self._ax = plt.subplots(nrows=1, ncols=1)

        self._fig.canvas.manager.set_window_title("Range-Angle-Map using DBF")
        self._fig.canvas.mpl_connect('close_event', self.close)
        self._is_window_open = True

    def _draw_first_time(self, data : np.ndarray):
        # First time draw

        minmin = -60
        maxmax = 0

        self.h = self._ax.imshow(
                   data,
                   vmin=minmin, vmax=maxmax,
                   cmap='viridis',
                   extent=(-self.max_angle_degrees,
                           self.max_angle_degrees,
                           0,
                           self.max_range_m),
                   origin='lower')

        self._ax.set_xlabel("angle (degrees)")
        self._ax.set_ylabel("distance (m)")
        self._ax.set_aspect("auto")

        self._fig.subplots_adjust(right=0.8)
        cbar_ax = self._fig.add_axes([0.85, 0.0, 0.03, 1])

        cbar = self._fig.colorbar(self.h, cax=cbar_ax)
        cbar.ax.set_ylabel("magnitude (a.u.)")

    def _draw_next_time(self, data : np.ndarray):
        # Update data for each antenna

        self.h.set_data(data)

    def draw(self, data : np.ndarray, title : str):
        if self._is_window_open:
            if self.h:
                self._draw_next_time(data)
            else:
                self._draw_first_time(data)
            self._ax.set_title(title)

            # Needed for Matplotlib ver: 3.4.0 and 3.4.1 helps with capture closing event
            plt.draw()
            plt.pause(1e-3)

    def close(self, event = None):
        if not self.is_closed():
            self._is_window_open = False
            plt.close(self._fig)
            plt.close('all')
            print('Application closed!')

    def is_closed(self):
        return not self._is_window_open
    



# def radar_callback(msg):
#     # antenna_active = 1
#     # num_chirps_per_frame = 32
#     # num_samples_per_chirp = 64

#     # Need to reshape message back into the normal dimensions to make sure it looks the same
#     # msg.layout.dim[0].label = "Receiver"
#     antenna_active = 2 #msg.layout.dim[0].size# = antenna_active
#     # msg.layout.dim[0].stride = 3*128*64
#     # msg.layout.dim[1].label = "chirps_per_frame"
#     num_chirps_per_frame = msg.layout.dim[1].size# = num_chirps_per_frame
#     # msg.layout.dim[1].stride = 64*128
#     # msg.layout.dim[2].label = "samples_per_chirp"
#     num_samples_per_chirp = msg.layout.dim[2].size# = num_samples_per_chirp
#     # msg.layout.dim[2].stride = 64

#     frame = np.array(msg.data).reshape(antenna_active, num_chirps_per_frame, num_samples_per_chirp)
#     #print(radar_data)


#     # PEAKING ALGORITHM DEMO ####################################
#     # num_chirps_per_frame = 128
#     # num_samples_per_chirp = 64
#     # algo = PresenceAntiPeekingAlgo(num_samples_per_chirp, num_chirps_per_frame)

#     # # matrix of dimension num_chirps_per_frame x num_samples_per_chirp for RX1
#     # mat = frame[0, :, :]
#     # presence_status, peeking_status = algo.presence(mat)

#     # print(f" Presence: {presence_status}")
#     # print(f"       Peeking: {peeking_status}")
#     ###############################################################


#     # RANGE-ANGLE MAP DEMO ########################################
#     rd_spectrum = np.zeros((config.num_samples_per_chirp, 2*config.num_chirps_per_frame, num_rx_antennas), dtype=complex)

#     beam_range_energy = np.zeros((config.num_samples_per_chirp, num_beams))

#     for i_ant in range(num_rx_antennas): # For each antenna
#         # Current RX antenna (num_samples_per_chirp x num_chirps_per_frame)
#         mat = frame[i_ant, :, :]

#         # Compute Doppler spectrum
#         dfft_dbfs = doppler.compute_doppler_map(mat, i_ant)
#         rd_spectrum[:,:,i_ant] = dfft_dbfs

#     # Compute Range-Angle map
#     rd_beam_formed = dbf.run(rd_spectrum)
#     for i_beam in range(num_beams):
#         doppler_i = rd_beam_formed[:,:,i_beam]
#         beam_range_energy[:,i_beam] += np.linalg.norm(doppler_i, axis=1) / np.sqrt(num_beams)

#     # Maximum energy in Range-Angle map
#     max_energy = np.max(beam_range_energy)

#     # Rescale map to better capture the peak The rescaling is done in a
#     # way such that the maximum always has the same value, independent
#     # on the original input peak. A proper peak search can greatly
#     # improve this algorithm.
#     scale = 150
#     beam_range_energy = scale*(beam_range_energy/max_energy - 1)

#     # Find dominant angle of target
#     _, idx = np.unravel_index(beam_range_energy.argmax(), beam_range_energy.shape)
#     angle_degrees = np.linspace(-max_angle_degrees, max_angle_degrees, num_beams)[idx]

#     # And plot...
#     # plot.draw(beam_range_energy, f"Range-Angle map using DBF, angle={angle_degrees:+02.0f} degrees")

#     ###############################################################
    



class gui_class():
    def __init__(self):
        self.num_beams = 27         # number of beams
        self.max_angle_degrees = 40 # maximum angle, angle ranges from -40 to +40 degrees

        self.config = Avian.DeviceConfig(
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

        self.max_range_m = 3.3533036708831787#metrics.max_range_m

        # Create frame handle
        self.num_rx_antennas = num_rx_antennas_from_config(self.config)

        # Create objects for Range-Doppler, DBF, and plotting.
        self.doppler = DopplerAlgo(self.config, self.num_rx_antennas)
        self.dbf = DBF(self.num_rx_antennas, num_beams = self.num_beams, max_angle_degrees = self.max_angle_degrees)
        self.plot = LivePlot(self.max_angle_degrees, self.max_range_m)

    def radar_callback(self, msg):
        # antenna_active = 1
        # num_chirps_per_frame = 32
        # num_samples_per_chirp = 64

        # Need to reshape message back into the normal dimensions to make sure it looks the same
        # msg.layout.dim[0].label = "Receiver"
        antenna_active = 2 #msg.layout.dim[0].size# = antenna_active
        # msg.layout.dim[0].stride = 3*128*64
        # msg.layout.dim[1].label = "chirps_per_frame"
        num_chirps_per_frame = msg.layout.dim[1].size# = num_chirps_per_frame
        # msg.layout.dim[1].stride = 64*128
        # msg.layout.dim[2].label = "samples_per_chirp"
        num_samples_per_chirp = msg.layout.dim[2].size# = num_samples_per_chirp
        # msg.layout.dim[2].stride = 64

        frame = np.array(msg.data).reshape(antenna_active, num_chirps_per_frame, num_samples_per_chirp)
        #print(radar_data)


        # RANGE-ANGLE MAP DEMO ########################################
        rd_spectrum = np.zeros((self.config.num_samples_per_chirp, 2*self.config.num_chirps_per_frame, self.num_rx_antennas), dtype=complex)

        beam_range_energy = np.zeros((self.config.num_samples_per_chirp, self.num_beams))

        for i_ant in range(self.num_rx_antennas): # For each antenna
            # Current RX antenna (num_samples_per_chirp x num_chirps_per_frame)
            mat = frame[i_ant, :, :]

            # Compute Doppler spectrum
            dfft_dbfs = self.doppler.compute_doppler_map(mat, i_ant)
            rd_spectrum[:,:,i_ant] = dfft_dbfs

        # Compute Range-Angle map
        rd_beam_formed = self.dbf.run(rd_spectrum)
        for i_beam in range(self.num_beams):
            doppler_i = rd_beam_formed[:,:,i_beam]
            beam_range_energy[:,i_beam] += np.linalg.norm(doppler_i, axis=1) / np.sqrt(self.num_beams)

        # Maximum energy in Range-Angle map
        max_energy = np.max(beam_range_energy)

        # Rescale map to better capture the peak The rescaling is done in a
        # way such that the maximum always has the same value, independent
        # on the original input peak. A proper peak search can greatly
        # improve this algorithm.
        scale = 150
        beam_range_energy = scale*(beam_range_energy/max_energy - 1)

        # Find dominant angle of target
        _, idx = np.unravel_index(beam_range_energy.argmax(), beam_range_energy.shape)
        angle_degrees = np.linspace(-self.max_angle_degrees, self.max_angle_degrees, self.num_beams)[idx]

        # And plot...
        self.plot.draw(beam_range_energy, f"Range-Angle map using DBF, angle={angle_degrees:+02.0f} degrees")
        # convert canvas to image


        ###############################################################


if __name__ == '__main__':
    rospy.init_node("radar_tr13c_node_listener")


    radar_plot_class = gui_class()

    print("Test")
    radar_sub = rospy.Subscriber('radar_tr13c', Float32MultiArray, radar_plot_class.radar_callback)

    plt.show(block=True)
    #rospy.spin()


    
