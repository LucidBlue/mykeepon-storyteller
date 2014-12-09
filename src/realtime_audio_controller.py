#! /usr/bin/env python

# for mykeepon_museum functionality
import roslib; roslib.load_manifest('mykeepon_storyteller')
import rospy
import math
import random
#from mykeepon_storyteller.msg import AudioMsg
from mykeepon_storyteller.msg import ControllerMsg
#from realtime_audio_capture import freq_power_from_fft
from time import sleep

from scipy.stats import rv_discrete

import numpy as np
import pyaudio
import wave
from time import clock, time

from states_and_actions import *
from convert_labels import convert_labels

#general
import sys
import copy

delay = 0
CHUNK = 2**9 #too long and fft will over-smooth freq data

def main():

	
	global audiopub#, launchID
	#launchID = int(sys.argv[3])
	# Initialize ROS node
	rospy.init_node('audio_publisher')
	# Create publisher to send out ControllerMsg messages
	pub = rospy.Publisher('audio_data', ControllerMsg)
	
	#open wav file and read parameters
	speechfile = sys.argv[1]
	choice = sys.argv[2]
	voice = sys.argv[3]
	
	if voice == '-p':
		wf = wave.open(speechfile, 'rb')
	elif voice == '-n':
		speechfile_np = speechfile[:-4] + '_noprosody.wav'
		wf = wave.open(speechfile_np)
	(channels,sample_width,rate,frames,comptype,compname) = wf.getparams()
	print("channels %d, swidth %d, rate %f, frames %d" %(channels, sample_width, rate, frames))
	
	
	label_file = speechfile[:-4] + '_labels.txt'
	actions = convert_labels(label_file, CHUNK, frames, rate, delay)
	actions_length = int(float(frames)/(CHUNK)) + 1
	#print(actions)
	#print(actions_length)
	
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
	
	"""
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
	actions[400] = ('none', 	'none',		'stop')
	actions[420] = ('none', 	'none',		'stop')
	actions[440] = ('none', 	'none',		'stop')
	actions[442] = ('none', 	'none',		'stop')
	#print("randomly generated actions: " + str(actions))
	#print("number of actions: " + str(actions_length))
	#action_iter = 0
	"""
	
	speed = 0.8
	intensity = 0.9
	slight_left = action_list['tilt left'](90.0, intensity, speed)
	slight_right = action_list['tilt right'](90.0, intensity, speed)
	slight_down = action_list['look down'](90.0, intensity, speed)
	slight_up = action_list['look up'](90.0, intensity, speed)
	down_left = [[sum(z) for z in zip(slight_left[x],slight_down[x])] for x in range(len(slight_down))]
	return_from_left = [[-elem for elem in down_left[x]] for x in range(len(down_left))]
	
	down_right = [[sum(z) for z in zip(slight_right[x],slight_down[x])] for x in range(len(slight_down))]
	return_from_right = [[-elem for elem in down_right[x]] for x in range(len(down_right))]
	
	full_left = down_left + return_from_left
	full_right = down_right + return_from_right
	#print(full_left)
	#print(full_right)
	
	background = list()
	random.seed(10)
	while len(background) < len(actions):
		drv = random.randint(0,1)
		print(drv)
		if drv == 0:
			movement = full_left
		else:
			movement = full_right
		background = background + movement
	#print(full_motion)
	#for elem in background: print(elem)
	
	#now add actions
	movements = [[0, 0, 0, 0] for x in range(actions_length)]
	for i,element in enumerate(actions):
		if element[2] != 'none' and element[2] != 'stop':
			movement = action_list[element[2]](90.0, 1.8, 1.6)
			#print(movement)
			reverse = [[-x for x in elem] for elem in movement]
			#for elem in movement:print(elem)
			#for elem in reverse: print(elem)
			
			#print(movement)
			for j in range(len(movement)):
				movements[i-len(movement) + j + 10] = copy.copy(movement[j])
			for j in range(len(reverse)):
				movements[i + j + 10] = reverse[j]
	
	
	movements += [[0,0,0,0]]*(len(actions) - len(movements))
	
	del background[len(actions):]
	del movements[len(actions):]
	#print(len(actions))
	#print(len(background))
	#print(len(movements))
	
	full_motion = list()
	if choice == '-a':
		full_motion = copy.deepcopy(movements)
	elif choice == '-c':
		full_motion = copy.deepcopy(background)
		#print(full_motion)
		#print(len(full_motion))
		#print(len(movements))
		#print(zip(full_motion, movements))
		full_motion = [[pair[0][0]+pair[1][0], pair[0][1]+pair[1][1], pair[0][2]+pair[1][2], pair[0][3]+pair[1][3]] for pair in zip(full_motion, movements)]
	else:
		full_motion = copy.deepcopy(background)
	
	#for elem in background: print(elem)
	#for elem in movements: print(elem)
	#for elem in full_motion:print(elem)
	print(full_motion)
	
	#stream until no more data (i.e. wav file is done)
	#start_time = clock()
	beginning_time = time()
	chunk_counter = 0
	
	current_state = list(default_pos)
	#print(current_state)
	
	while len(data) == CHUNK*sample_width:
		#valence,intensity,action = action[chunk_counter]
		#print(valence, intensity, action)
		
		#only background
		"""
		panD = full_motion[chunk_counter][0]
		tiltD = full_motion[chunk_counter][1]
		rollD = full_motion[chunk_counter][2]
		bopD = full_motion[chunk_counter][3]
		"""
		
		#only movements
		panD = full_motion[chunk_counter][0]
		tiltD = full_motion[chunk_counter][1]
		rollD = full_motion[chunk_counter][2]
		bopD = full_motion[chunk_counter][3]
		
		#print(current_state)
		#print([panD, tiltD, rollD, bopD])
		pan = current_state[0] + panD
		tilt = current_state[1] + tiltD
		roll = current_state[2] + rollD
		bop = current_state[3] + bopD
		
		if pan > 160:
			pan = 160
		if pan < 20:
			pan = 20
		if tilt > 180:
			tilt = 180
		if tilt < 0:
			tilt = 0
		if roll > 180:
			roll = 180
		if roll < 0:
			roll = 0
		
		current_state = [pan, tilt, roll, 0]
		#print(current_state)
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
		"""
		message = ControllerMsg()
		message.ID = 0
		message.pan = pan
		message.tilt = tilt
		message.roll = roll
		message.bop = bop
		
		#rospy.loginfo('Sending message: pan=' + str(pan) + ' tilt=' + str(tilt) + ' roll=' + str(roll) + ' bop=' + str(bop))
		"""
			rospy.loginfo('Sending message: valence=' + str(valence) + ' intensity=' + str(intensity) + ' action=' + str(action))
		"""
		pub.publish(message)
		
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



