from __future__ import division
import pyaudio
import wave

import sys
import scipy
import numpy as np
import struct


#from scikits.audiolab import flacread
from numpy.fft import rfft, irfft
from numpy import argmax, sqrt, mean, diff, log
import matplotlib
from scipy.signal import blackmanharris, fftconvolve
from time import time
from parabolic import parabolic

#for frequency estimation using harmonic product spectrum
from pylab import subplot, plot, log, copy, show

chunk = 2**8

def freq_from_fft(sig, fs):
	"""Estimate frequency from peak of FFT
	"""
	# Compute Fourier transform of windowed signal
	windowed = sig * blackmanharris(chunk)
	fftData = rfft(windowed)
	# Find the peak and interpolate to get a more accurate peak
	i = argmax(abs(fftData)) # Just use this for less-accurate, naive version
	if i != len(fftData)-1:
		true_i = parabolic(log(abs(fftData)), i)[0]
		# Convert to equivalent frequency
		return fs * true_i / len(windowed)
	else:
		return fs * i / len(windowed) 

def freq_from_fft_quadratic(sig, rate):
	fftData = np.abs(np.fft.rfft(sig))**2
	#print(fftData[0:4])
	# find the maximum
	fftMax = fftData[1:].argmin() + 1
	#print ("FFT Max: " + str(fftMax))
	# use quadratic interpolation around the max
	if fftMax != len(fftData)-1:
		y0,y1,y2 = np.log(fftData[fftMax-1:fftMax+2:])
		#print("y0: %f y1: %f y2: %f" %(y0, y1, y2))
		x1 = ((y2 - y0) * .5) / (2 * y1 - y2 - y0)
		# find the frequency and output it
		frequency = (fftMax+x1)*rate/chunk
	else:
		frequency = fftMax*rate/chunk
	return frequency

"""
def freq_from_autocorr(sig, fs):
	# Calculate autocorrelation (same thing as convolution, but with
	# one input reversed in time), and throw away the negative lags
	corr = fftconvolve(sig, sig[::-1], mode='full')
	corr = corr[len(corr)/2:]
	# Find the first low point
	d = diff(corr)
	start = find(d > 0)#[0]
	# Find the next peak after the low point (other than 0 lag). This bit is
	# not reliable for long signals, due to the desired peak occurring between
	# samples, and other peaks appearing higher.
	# Should use a weighting function to de-emphasize the peaks at longer lags.
	peak = argmax(corr[start:]) + start
	px, py = parabolic(corr, peak)
	return fs / px
"""

"""
def freq_from_autocorr2(signal, fs):
"""
"""Estimate frequency using autocorrelation
	Pros: Best method for finding the true fundamental of any repeating wave,
	even with strong harmonics or completely missing fundamental
	Cons: Not as accurate, doesn't work for inharmonic things like musical
	instruments, this implementation has trouble with finding the true peak
"""
"""
	# Calculate autocorrelation (same thing as convolution, but with one input
	# reversed in time), and throw away the negative lags
	signal -= mean(signal) # Remove DC offset
	corr = fftconvolve(signal, signal[::-1], mode='full')
	corr = corr[len(corr)/2:]
	# Find the first low point
	d = diff(corr)
	start = matplotlib.mlab.find(d > 0)[0]
	# Find the next peak after the low point (other than 0 lag). This bit is
	# not reliable for long signals, due to the desired peak occurring between
	# samples, and other peaks appearing higher.
	i_peak = argmax(corr[start:]) + start
	i_interp = parabolic(corr, i_peak)[0]
	return fs / i_interp
"""

def capture_audio(filename)
	wf = wave.open(sys.argv[1], 'rb')
	(channels,sample_width,rate,frames,comptype,compname) = wf.getparams()
	p = pyaudio.PyAudio()
	
	stream = p.open(format	 = p.get_format_from_width(sample_width),
                	channels = channels,
                	rate	 = rate,
                	output	 = True)

	data = wf.readframes(chunk)

	
	while len(data) == chunk*sample_width:
		indata = np.array(wave.struct.unpack("%dh"%(len(data)/sample_width),data))
		frequency = freq_from_fft(indata, rate)
		
		if frequency < 1000 and frequency > 500:
			print "frequency: %f Hz" % (frequency)		
		stream.write(data)
		data = wf.readframes(chunk)
	
	#stream may not divide evenly into chunks
	if data:
		stream.write(data)
	
	wf.close()
	stream.stop_stream()
	stream.close()

	p.terminate()


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

	data = wf.readframes(chunk)
	
	freq_sum = 0.0
	freq_count = 0
	freq_max = 0.0
	freq_min = 999999999999
	
	while len(data) == chunk*sample_width:
		# unpack data
		indata = np.array(wave.struct.unpack("%dh"%(len(data)/sample_width),data))
		
		#remainder of calculations
		frequency = freq_from_fft(indata, rate)
		
		if frequency < 1000 and frequency > 500:
			print "frequency: %f Hz" % (frequency)
			
		if frequency < 1000 and frequency >= 0:
			#print "frequency: %f Hz" % (frequency)
			freq_sum += frequency
			freq_count += 1
			if frequency < freq_min:
				freq_min = frequency
			if frequency > freq_max:
				freq_max = frequency
		
		
		# write data out to the audio stream after first round of calculations
		stream.write(data)
		
		# read some more data
		data = wf.readframes(chunk)
	
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




if __name__ == '__main__':
	main()
	
	
	
	
	
	
	
