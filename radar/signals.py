"""
ECE 239AS Implementation Final Project: RadarSim (FMCW Radar Simulation)
Written by Josh Kushner and Michael Zhou
"""

import numpy as np
from scipy.signal import firwin, lfilter, sawtooth
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

class FMCWDSP:
    """Processes FMCW radar signals for range estimation."""
    def __init__(self, bandwidth, chirp_time, starting_freq, sampling_freq,
                 frame_size, velocity, radar_pos):
        # Copy over the key parameters from scene.py
        self.bandwidth = bandwidth
        self.chirp_time = chirp_time
        self.starting_freq = starting_freq
        self.sampling_freq = sampling_freq
        self.frame_size = frame_size
        self.radar_pos = radar_pos
        self.max_dist = 150
        self.c = 3e8
        self.awgn = 0.1

    ############################################################################
    # Setter Methods                                                           #
    ############################################################################
    def setBandwidth(self, bandwidth):
        self.bandwidth = bandwidth

    def setChirpTime(self, time):
        self.chirp_time = time

    def setStartingFreq(self, freq):
        self.starting_freq = freq
    
    def setFrameSize(self, size):
        self.frame_size = size
    
    def fir_lowpass_filter(self, data, cutoff_hz, numtaps=101):
        """
        Low pass filter applied to the beat signals
        """
        nyq = self.sampling_freq / 2
        normal_cutoff = cutoff_hz / nyq
        fir_coeff = firwin(numtaps, normal_cutoff)
        filtered_data = lfilter(fir_coeff, 1.0, data)
        return filtered_data

    def generate_chirp(self, dist):
        """
        Performs the bulk of the signal processing for our FMCW radar.
        Simulates an FMCW radar signal by leveraging our knowledge of the simulation
        distance of the object. Using the time delay we can find the phase
        difference between the transmit signal and the receive signal. 
        """
        # Calculate time delay for each target
        time_delay = 2 * dist / self.c
        fbeat = 2 * dist * self.bandwidth / (self.c * self.chirp_time) # derived from range formula
        
        # Create linear frequency modulated Tx wave
        t = np.linspace(0, self.chirp_time, int(self.sampling_freq * self.chirp_time))
        phase_diff = 2 * np.pi * (self.starting_freq + self.bandwidth/2) * time_delay
        beat = np.cos(2 * np.pi * fbeat * t + phase_diff)
        
        # Low Pass Filtering
        max_beat = 2 * self.bandwidth * self.max_dist / (self.c * self.chirp_time)
        beat_lpf = self.fir_lowpass_filter(beat, max_beat * 0.95)

        # Simulate AWGN noise
        beat_lpf = beat_lpf + np.random.normal(0, self.awgn, len(beat_lpf))

        # Compute FFT
        N = len(beat_lpf)
        fft_vals = np.fft.fft(beat_lpf)
        freqs = np.fft.fftfreq(N, 1 / self.sampling_freq)

        # Ignore negative frequencies
        pos_mask = freqs > 0
        fft_vals_pos = fft_vals[pos_mask]
        freqs_pos = freqs[pos_mask]

        # Peak frequency
        peak = freqs_pos[np.argmax(np.abs(fft_vals_pos))]

        # Estimate range using FMCW range equation
        fmcw_range = self.c * peak * self.chirp_time / (2 * self.bandwidth)

        f = np.linspace(self.starting_freq, self.starting_freq + self.bandwidth, int(self.chirp_time * self.sampling_freq))
        t = np.linspace(0, 2 * self.chirp_time, len(f))

        tx = (sawtooth(2 * np.pi * (1/self.chirp_time) * t, width=1) + 1) / 2
        rx = (sawtooth(2 * np.pi * (1/self.chirp_time) * (t - time_delay), width=1) + 1) / 2
        
        return {
            'true_range': dist,
            'time_delay' : time_delay,
            'estimated_range': fmcw_range,
            'beat_lpf': beat_lpf,
            'tx' : tx,
            'rx' : rx
        }
    
    def calculateVelocity(self, phase, target):
        lam = self.c / (self.starting_freq + self.bandwidth/2)
        v = (lam / (4 * np.pi)) * (phase / self.chirp_time)
        print(f"TRUE VELOCITY: {target.velocity}, FMCW VELOCITY: {v}")
        return v
