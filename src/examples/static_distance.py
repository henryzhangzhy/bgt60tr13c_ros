# ===========================================================================
# Copyright (C) 2021-2022 Infineon Technologies AG
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# ===========================================================================

import argparse
import numpy as np

from ifxAvian import Avian
from scipy import signal
from scipy import constants

from internal.fft_spectrum import *


# -------------------------------------------------
# Computation
# -------------------------------------------------
class DistanceAlgo:
    # Algorithm for computing distance

    def __init__(self, config):
        # Inits all needed common values
        # config: dictionary with configuration for device used by set_config() as input
        
        # derive the number of Chirps, Samples per chirp from frame shape
        self._chirps_per_frame = config.num_chirps_per_frame
        self._samples_per_chirp = config.num_samples_per_chirp

        # compute Blackman-Harris Window matrix over chirp samples(range)
        self._range_window = signal.blackmanharris(
            self._samples_per_chirp).reshape(1, self._samples_per_chirp)

        bandwidth_hz = abs(config.end_frequency_Hz - config.start_frequency_Hz)

        fft_size = self._samples_per_chirp * 2
        self._range_bin_length = (constants.c) / (2 * bandwidth_hz * fft_size / self._samples_per_chirp)

    def compute_distance(self, chirp_data):
        # Computes distance using chirp data
        # chirp_data: single antenna chirp data

        # Step 1 - calculate fft spectrum for the frame
        range_fft = fft_spectrum(chirp_data, self._range_window)

        # Step 2 - Convert to absolute spectrum
        range_fft_abs = abs(range_fft)

        # Step 3 - Coherent integration of all chirps
        dat = np.divide(range_fft_abs.sum(axis=0), self._chirps_per_frame)

        # Step 4 - Peak search and distance calculation
        skip = 8
        max = np.argmax(dat[skip:])
        return (self._range_bin_length * (max + skip))

# -------------------------------------------------
# Helpers
# -------------------------------------------------
def parse_program_arguments(description, def_nframes, def_frate):
    # Parse all program attributes
    # description:   describes program
    # def_nframes:   default number of frames
    # def_frate:     default frame rate in Hz
    
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('-n', '--nframes', type=int,
                        default=def_nframes, help="number of frames, default "+str(def_nframes))
    parser.add_argument('-f', '--frate', type=int, default=def_frate,
                        help="frame rate in Hz, default "+str(def_frate))
    return parser.parse_args()

# -------------------------------------------------
# Main logic
# -------------------------------------------------
if __name__ == '__main__':

    args = parse_program_arguments(
        '''Displays distance plot from Radar Data'a''', 
        def_nframes=25, 
        def_frate=5)

    with Avian.Device() as device:
        i_ant = 0 #we use here only antenna #0

        metrics = Avian.DeviceMetrics(
            sample_rate_Hz = 1_000_000,
            range_resolution_m = 0.05,
            max_range_m = 1.6,
            max_speed_m_s = 3,
            speed_resolution_m_s = 0.2,
            frame_repetition_time_s = 1/args.frate,
            rx_mask = 1,
            tx_mask = 1,
            tx_power_level = 31,
            if_gain_dB = 33)

        config = device.metrics_to_config(metrics)
        device.set_config(config)
       
        distanceExample = DistanceAlgo(config)
        
        for frame_number in range(args.nframes):
            frame = device.get_next_frame()
            antenna_samples = frame[i_ant, :, :]
            distance = distanceExample.compute_distance(antenna_samples)
            print("Distance:" + format(distance, "^05.3f") + "m")
