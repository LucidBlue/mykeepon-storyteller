#! /usr/bin/env python

from __future__ import division
import pyaudio
import wave

import sys
import scipy
import numpy as np
import struct

CHUNK = 2**10

#from scikits.audiolab import flacread
from numpy.fft import rfft, irfft
from numpy import argmax, sqrt, mean, diff, log
import matplotlib
from scipy.signal import blackmanharris, fftconvolve
from time import time
from parabolic import parabolic

def freq_power_from_fft(sig, fs):
	# Compute Fourier transform of windowed signal
	windowed = sig * blackmanharris(CHUNK)
	fftData = abs(rfft(windowed))
	# powerData = 20*log10(fftData)
	# Find the index of the peak and interpolate to get a more accurate peak
	i = argmax(fftData) # Just use this for less-accurate, naive version
	# make sure i is not an endpoint of the bin
	if i != len(fftData)-1 and i != 0:
		#print("data: " + str(parabolic(log(abs(fftData)), i)))
		true_i,logmag = parabolic(log(abs(fftData)), i)
		# Convert to equivalent frequency
		freq=  fs * true_i / len(windowed)
		#print("frequency="+ str(freq) + " log of magnitude=" + str(logmag))
		if logmag < 0:
			logmag = 0	
		return freq,logmag
	
	else:
		freq = fs * i / len(windowed)
		logmag = log(abs(fftData))[i]
		if logmag < 0:
			logmag = 0
		#print("frequency="+ str(freq) + "log of magnitude not interp=" + str(logmag))
		return freq,logmag

def main():
	#open wav file
	wf = wave.open(sys.argv[1], 'rb')
	
	(channels,sample_width,rate,frames,comptype,compname) = wf.getparams()
	FPS = 25.0
	
	trimby = 10
	divby = 100
	
	#print("channels: %d sample width: %d rate: %d frames: %d chunk: %d" %(channels, sample_width, rate, frames, chunk))
	
	# instantiate PyAudio
	p = pyaudio.PyAudio()
	
	stream = p.open(format	 = p.get_format_from_width(sample_width),
                	channels = channels,
                	rate	 = rate,
                	output	 = True)

	data = wf.readframes(CHUNK)
	
	freq_sum = 0.0
	freq_count = 0
	freq_max = 0.0
	freq_min = 999999999999
	
	while len(data) == CHUNK*sample_width:
		# unpack data
		indata = np.array(wave.struct.unpack("%dh"%(len(data)/sample_width),data))
		
		#remainder of calculations
		frequency,logmag = freq_power_from_fft(indata, rate)
		print("frequency: " + str(frequency) + " logMagnitude: " + str(logmag))
		
		"""
		if frequency < 1000 and frequency > 500:
			print "frequency: %f Hz" % (frequency)
		"""
			
		if frequency < 1000 and frequency > 0:
			#print "frequency: %f Hz" % (frequency)
			freq_sum += frequency
			freq_count += 1
			if frequency < freq_min:
				freq_min = frequency
			if frequency > freq_max:
				freq_max = frequency
		#print("freq count: " + str(freq_count))
		
		# write data out to the audio stream after first round of calculations
		stream.write(data)
		
		# read some more data
		data = wf.readframes(CHUNK)
	
	avg_freq = freq_sum/freq_count
	print("Average frequency for this clip: %f" %(avg_freq))
	print("Min frequency for this clip: %f" %(freq_min))
	print("Max frequency for this clip: %f" %(freq_max))
	
	
	if data:
		stream.write(data)
	
	wf.close()
	stream.stop_stream()
	stream.close()

	p.terminate()


if __name__ == "__main__":
	main()	

