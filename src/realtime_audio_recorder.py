import matplotlib
matplotlib.use('TkAgg') # <-- THIS MAKES IT FAST!
import numpy
import scipy
import struct
import pyaudio
import threading
import pylab
import struct

from scipy.io import wavfile # get the api

def f(filename):
	fs, data = wavfile.read(filename) # load the data
	a = data # this is a two channel soundtrack, I get the first track

	print(data.dtype)
	print(data.shape)

	b=data/(2.**15) # this is 16-bit track, b is now normalized on [-1,1)

	c = fft(b) # create a list of complex number
	d = len(c)/2  # you only need half of the fft list
	plt.plot(abs(c[:(d-1)]),'r')
	#savefig(filename+'.png',bbox_inches='tight')


class SwhRecorder:
	"""Simple, cross-platform class to record from the microphone."""

	def __init__(self):
		"""minimal garb is executed when class is loaded."""
		self.RATE=48100
		self.BUFFERSIZE=2**12
		self.secToRecord=.1
		self.threadsDieNow=False
		self.newAudio=False

	def setup(self, filename):

		self.buffersToRecord=int(self.RATE*self.secToRecord/self.BUFFERSIZE)
		if self.buffersToRecord==0: self.buffersToRecord=1
		self.samplesToRecord=int(self.BUFFERSIZE*self.buffersToRecord)
		self.chunksToRecord=int(self.samplesToRecord/self.BUFFERSIZE)
		self.secPerPoint=1.0/self.RATE

		self.p = pyaudio.PyAudio()
		self.inStream = self.p.open(format=pyaudio.paInt16,channels=1,rate=self.RATE,input=True,frames_per_buffer=self.BUFFERSIZE)

		sample_rate, wavdata = wavfile.read(filename)

		self.xsBuffer=numpy.arange(self.BUFFERSIZE)*self.secPerPoint
		self.xs=numpy.arange(self.chunksToRecord*self.BUFFERSIZE)*self.secPerPoint
		self.audio=numpy.empty((self.chunksToRecord*self.BUFFERSIZE),dtype=numpy.int16)               


	def record(self,forever=True):
		"""record secToRecord seconds of audio."""
		while True:
			if self.threadsDieNow: break
			for i in range(self.chunksToRecord):
				self.audio[i*self.BUFFERSIZE:(i+1)*self.BUFFERSIZE]=self.getAudio()
			self.newAudio=True 
			if forever==False: break

	def continuousStart(self):
		"""CALL THIS to start running forever."""
		self.t = threading.Thread(target=self.record)
		self.t.start()

	def continuousEnd(self):
		"""shut down continuous recording."""
		self.threadsDieNow=True

		### MATH ###
		
	def downsample(self,data,mult):
		"""Given 1D data, return the binned average."""
		overhang=len(data)%mult
		if overhang: data=data[:-overhang]
		data=numpy.reshape(data,(len(data)/mult,mult))
		data=numpy.average(data,1)
		return data    

	def wavfft(self,trimBy=10,logScale=False,divBy=100):
		normalized=self.wavdata/(2.**15)
		left,right=numpy.split(numpy.abs(numpy.fft.fft(normalized)),2)
		ys=numpy.add(left,right[::-1])
		if logScale:
			ys=numpy.multiply(20,numpy.log10(ys))
		if trimBy:
			i=int((self.BUFFERSIZE/2)/trimBy)
			ys=ys[:i]
			xs=xs[:i]*self.RATE/self.BUFFERSIZE
		if divBy:
			ys=ys/float(divBy)

		return xs,ys
	"""	
	def fft(self,data=None,trimBy=10,logScale=False,divBy=100):
		if data==None: 
			data=self.audio.flatten()
		left,right=numpy.split(numpy.abs(numpy.fft.fft(data)),2)
		ys=numpy.add(left,right[::-1])
		if logScale:
			ys=numpy.multiply(20,numpy.log10(ys))
		xs=numpy.arange(self.BUFFERSIZE/2,dtype=float)
		if trimBy:
			i=int((self.BUFFERSIZE/2)/trimBy)
			ys=ys[:i]
			xs=xs[:i]*self.RATE/self.BUFFERSIZE
		if divBy:
			ys=ys/float(divBy)
		return xs,ys
	"""
		### VISUALIZATION ###

	def plotAudio(self):
		"""open a matplotlib popup window showing audio data."""
		pylab.plot(self.audio.flatten())
		pylab.show()        
		
