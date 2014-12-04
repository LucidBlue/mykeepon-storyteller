#! /usr/bin/env python

# for mykeepon_museum functionality
import roslib; roslib.load_manifest('mykeepon_storyteller')
import rospy
import math
from mykeepon_storyteller.msg import AudioMsg
#from realtime_audio_capture import freq_power_from_fft
from time import sleep

from scipy.stats import rv_discrete

import numpy as np
import pyaudio
import wave
from time import clock, time

from states_and_actions import valence_list, intensity_list, action_list
from convert_labels import convert_labels

#general
import sys

delay = .5
CHUNK = 2**8 #too long and fft will over-smooth freq data

def main():

	
	global audiopub#, launchID
	#launchID = int(sys.argv[3])
	# Initialize ROS node
	rospy.init_node('audio_publisher')
	# Create publisher to send out ControllerMsg messages
	audio_pub = rospy.Publisher('audio_data', AudioMsg)
	
	#open wav file and read parameters
	speechfile = sys.argv[1]
	wf = wave.open(speechfile, 'rb')
	(channels,sample_width,rate,frames,comptype,compname) = wf.getparams()
	print("channels %d, swidth %d, rate %f, frames %d" %(channels, sample_width, rate, frames))
	
	"""
	label_file = speechfile[:-4] + '_labels.txt'
	actions = convert_labels(label_file, CHUNK, frames, rate, delay)
	"""
	#initialize PyAudio
	p = pyaudio.PyAudio()
	
	#start a data stream 
	stream = p.open(format	 = p.get_format_from_width(sample_width),
                	channels = channels,
                	rate	 = rate,
                	output	 = True)
	
	#begin reading data into buffer
	data = wf.readframes(CHUNK)
	#drv = rv_discrete(values=((0, 1, 2), (0.8, 0.1, 0.1)))
	
	
	actions_length = int(float(frames)/(CHUNK)) + 1
	#print(actions_length)
	
	actions = [('none', 'none', 'none') for x in range(actions_length)]
	#print(actions)
	
	#add some actions
	actions[5] 	 = ('none', 	'none',		'none')
	actions[10]  = ('stop', 	'stop',		'stop')
	actions[15]	 = ('stop', 	'stop',		'stop')
	actions[100] = ('none', 	'none',		'tilt left')
	actions[120] = ('none', 	'none',		'stop')
	actions[150] = ('none', 	'none',		'tilt right')
	actions[170] = ('none', 	'none',		'stop')
	#actions[200] = ('none', 	'none',		'look left')
	actions[220] = ('none', 	'none',		'stop')
	#actions[250] = ('none', 	'none',		'look right')
	actions[270] = ('none', 	'none',		'stop')
	actions[300] = ('none', 	'none',		'look up')
	actions[320] = ('none', 	'none',		'stop')
	actions[350] = ('none', 	'none',		'look down')
	actions[370] = ('none', 	'none',		'stop')
	actions[400] = ('none', 	'none',		'extend')
	actions[420] = ('none', 	'none',		'stop')
	actions[440] = ('stop', 	'stop',		'stop')
	actions[442] = ('stop', 	'stop',		'stop')
	#print("randomly generated actions: " + str(actions))
	#print("number of actions: " + str(actions_length))
	#action_iter = 0
	
	
	
	#stream until no more data (i.e. wav file is done)
	#start_time = clock()
	beginning_time = time()
	chunk_counter = 0
	
	current_state = ['stop', 'stop', 'stop']
	
	while len(data) == CHUNK*sample_width:
		valence,intensity,action = actions[chunk_counter]
		#print(valence, intensity, action)
		
		chunk_counter +=1
		#print("chunks read: %d" %(chunk_counter))
		
		stream.write(data)
		data = wf.readframes(CHUNK)
		
		"""
		#convert python wave formate (byte string) into a numpy array to perform calculations
		indata = np.array(wave.struct.unpack("%dh"%(len(data)/sample_width),data))
		
		#math
		#have to find better default values for tempo and loudness until these are extracted
		frequency, loudness = freq_power_from_fft(indata, rate)
		tempo = sys.argv[2] #+-% rate from baseline (-10% = 10% slower)
		
		if frequency < 1000 and frequency > 500:
			#message.ID = launchID
			message.freq = frequency
			message.tempo = float(tempo)
			message.loudness = loudness
			
			audiopub.publish(message)
			#print "frequency: %f Hz loudness %f log(amp)" % (frequency, loudness)
		"""
		if (valence, intensity, action) != ('none', 'none', 'none'):
			message = AudioMsg()
			
			if valence == 'none':
				valence = current_state[0]
			else:
				current_state[0] = valence
			
			
			if intensity == 'none':
				intensity = current_state[1]
			else:
				current_state[1] = intensity
			
			
			if action == 'none':
				action = current_state[2]
			else:
				current_state[2] = action
			
			
			message.valence = valence 
			message.intensity = intensity
			message.action = action
			
			rospy.loginfo('Sending message: valence=' + str(valence) + ' intensity=' + str(intensity) + ' action=' + str(action))
			audio_pub.publish(message)
		
		#play audio after it has been published. will have to work out creating a larger delay
		#end_time = clock() - start_time
		#start_time = clock()
		#print("one chunk has passed. time elapsed: %f" %(end_time))
		

		#rospy.sleep(1.0)
		#print("time so far: %f" %(time() - beginning_time))

	if data:
		stream.write(data)
	
	duration = time() - beginning_time
	print("duration: %f chunks read: %d" %(duration, chunk_counter))
	
	wf.close()
	stream.stop_stream()
	stream.close()
	
	p.terminate()
	
	
	# Allow the program to listen without blocking
	rospy.spin()


if __name__ == "__main__":
	main()



