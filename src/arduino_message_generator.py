#! /usr/bin/env python

import roslib; roslib.load_manifest('mykeepon_museum')
import rospy
import math
from mykeepon_museum.msg import OmronMsg
from mykeepon_museum.msg import ControllerMsg
from mykeepon_museum.msg import FaceDetectorMsg
from time import sleep
import sys

FACE_X_MAX = 240 #240 # Max x value from face detector
FACE_Y_MAX = 120 #120 # Max y value from face detector
RES_X_MAX = 1280 #1280
RES_Y_MAX = 720 #720
#RES_X_MAX = 800 #1280
#RES_Y_MAX = 600 #720

MIRROR = False # correction for servos that pan in different directions, True for Yale, False for GA Tech as of 3/14

face_detector = [0,0]

i = 10

def main():
	global pub, launchID
	
	print 'camera ID: ' + sys.argv[1]
	launchID = int(sys.argv[1])
	
	# Initialize ROS node
	rospy.init_node('controller')
	
	# Create publisher to send out ControllerMsg messages
	pub = rospy.Publisher('controller_data', ControllerMsg)
	
	# Subscribe to the Omron stream
	rospy.Subscriber('omron_data', OmronMsg, omron_callback, queue_size=1)
	
	# Subscribe to the face detector stream
	rospy.Subscriber('face_detector_data', FaceDetectorMsg, fd_callback, queue_size=1)
	
	rospy.loginfo("Controller node inititalized.")
	
	# Allow the program to listen without blocking
	rospy.spin()
	
def fd_callback(data):
	global face_detector
	global FACE_X_MAX
	global FACE_Y_MAX
	global i
	global launchID
	
	[face_detector[0], face_detector[1]] = [data.x,data.y]
	rospy.loginfo("Received from face detector: x=" + str(data.x) + ", y=" + str(data.y))

	x_center = FACE_X_MAX/2
	y_center = FACE_Y_MAX/2
	
	x = data.x
	alpha = 1 # Scaling factor from face detector to robot pan
	tau = 1 # Scaling factor from face detector to robot tilt
	rho = 1 # Scaling factor from Omron to robot roll
	
	offset = (x_center - x) * alpha # from center of screen
	if math.fabs(offset) > (FACE_X_MAX/2): # large sweeps if big offset
		#pan = math.copysign(10,offset)
		pan = math.copysign(
				max( (int) ((math.fabs(offset)-250)*1.0/3), 10),
					offset)
		rospy.logdebug('offset at ' + str(offset) + ', sending ' + str(pan))

	elif math.fabs(offset) > (FACE_X_MAX/4): # small sweeps if closer than 50
		pan = math.copysign(8,offset)
		rospy.logdebug('offset at ' + str(offset) + ', sending ' + str(pan))

	elif math.fabs(offset) > (FACE_X_MAX/6): # close enough to stop  
		pan = math.copysign(5,offset)
		rospy.logdebug('offset at ' + str(offset) + ', sending ' + str(pan))

	else:
		rospy.logdebug('ignoring pan value of ' + str(offset))
		return
  	
	controller_msg = ControllerMsg()
  	controller_msg.pan = pan
  	controller_msg.tilt = 0
  	controller_msg.roll = 0
	
	pub.publish(controller_msg)

	rospy.loginfo("Sending controller message with pan " + str(pan))
	

	# variable sleep times: the longer the sweep, the more 
	# time the camera needs to settle before a new frame is
	# evaluated
	''' 
	Hopefully solved this with faster face detection
	if pan < 5: 
		return
	elif pan < 10:
		sleep(1.0)
	else:
		sleep(2.0)
	'''


def omron_callback(data):
	'''
	Grab Omron data and convert it into commands to send to MyKeepon.
	If there's no gaze, ignore the message. Otherwise, convert the
	pan, tilt, and roll values from the person's face as well as x 
	and y position values from the face tracker into pan, tilt,
	and roll values for MyKeepon, stuff those into a ControllerMsg, 
	and publish it.
	'''

	global pub
	global face_detector
	global RES_X_MAX
	global RES_Y_MAX
	global launchID
	
	if data.ID != launchID:
		return
	
	p = t = r = pan = tilt = roll = 0
	print("is_gaze = " + str(data.is_gaze))
	if data.is_gaze:
		# Using Omron-detected values of the face's tilt, pan, and roll, 
		# along with the face detector's x and y positions, calculate
		# how MyKeepon should move.
		[p,t,r] = [data.face_pan, data.face_tilt, data.face_roll]
		
		x = data.face_ctr_x
		y = data.face_ctr_y
		
		print ('gaze: ' + str(data.is_gaze) + ', pan: ' + str(p) 
			+ ', tilt: ' + str(t) + ', roll: ' + str(r))
		print ("x " + str(data.face_ctr_x) + " y " + str(data.face_ctr_y))
			
			
		[x_center, y_center] = [RES_X_MAX/2, RES_Y_MAX/2]
		
		alpha = 1.5 # Scaling factor from face detector to robot pan
		tau = 1 # Scaling factor from face detector to robot tilt
		rho = 1 # Scaling factor from Omron to robot roll
		k_p = 0.05 # proportional controller

		# tilt
		# Zhefan change it for testing
		tilt = data.face_tilt
		#if (math.fabs(t) < 20):
		# if (t > 10):
		# 	tilt = 0
		# elif (t < -10):
		# 	tilt = 180
		# else:
		# 	tilt = 90
		'''
		if (math.fabs(y-y_center) < 0.2*RES_Y_MAX):
			tilt = 90
		elif (y > y_center):
			tilt = 180
		else:
			tilt = 0
		'''
				
		rospy.logdebug('sending out tilt of: ' + str(tilt))
		
		# roll
		roll = data.face_roll
		#if (math.fabs(r) <= 15):
		# 	roll = 90
		# elif (r > 15):
		# 	roll = 180
		# elif (r < -15):
		# 	roll = 0
		
		# pan
		offset = (x - x_center) * alpha
		if MIRROR:
			offset = offset * -1
		rospy.logdebug('offset at ' + str(offset))
		
		if math.fabs(offset) > (RES_X_MAX/16):
			pan = k_p * offset;
			rospy.loginfo('pan: ' + str(pan))
		else: # close enough to stop
			rospy.loginfo('ignoring pan value of ' + str(offset))
			
		
	else:
		return
		
		
  	# Create Controller message and publish it
  	rospy.loginfo('sending: pan='+str(pan)+' tilt='+str(tilt)+' roll='+str(roll))
  	controller_msg = ControllerMsg()
  	controller_msg.ID = data.ID
  	controller_msg.pan = pan
  	controller_msg.tilt = tilt
  	controller_msg.roll = roll
  	# emotion: set to 5 to disable
  	# controller_msg.emotion = 5;
  	
  	# emotion filter: disable fear and sadness
  	if data.smile == 1 or data.smile == 3:
  		data.smile = 5
  	
  	controller_msg.emotion = data.smile
	
	pub.publish(controller_msg)
	
	
if __name__ == '__main__':
	main()
