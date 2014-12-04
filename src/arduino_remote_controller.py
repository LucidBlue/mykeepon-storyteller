#! /usr/bin/env python
''' 
 * ArduinoRemoteController
 *
 * Subscriber to the ros stream from Controller.py
 * 
 * Communicates with the Arduino microcontroller to send commands 
 * to make the robot pan to the desired location (when a 'friend' has been
 * identified).
 * Once pan is stabilized, the Arduino is instructed to mimic the yaw, roll 
 * and tilt of the human target as much as possible.
 *
'''
	
import roslib; roslib.load_manifest('mykeepon_museum')
import rospy
import math
#from mykeepon_storyteller.msg import OmronMsg
from mykeepon_storyteller.msg import ControllerMsg

import serial
from serial.tools import list_ports
import math
from time import sleep
import sys
import logging

# --------------------------------------------------------------------
# setting up debug log environment

# -- DEPRECARED --
# logger = logging.getLogger(__name__)
# logger.setLevel (logging.DEBUG)
# handler = logging.FileHandler('LOG_ArdPy.log')
# handler.setLevel(logging.DEBUG)
# formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
# handler.setFormatter(formatter)
# logger.addHandler(handler)


rospy.loginfo('$$$$$ Starting new running instance $$$$$')
rospy.loginfo('===========================================')

# --------------------------------------------------------------------
ard_serial = None	
port_name = "ACM0" # default port for the Arduino
launchID = 0 # default if no argument passed

argc = len(sys.argv)
if (argc > 1):
	port_name = sys.argv[1]
if (argc > 2):
	launchID = int(sys.argv[2])
	
ard_port = "/dev/tty" + port_name

ard_serial = serial.Serial(ard_port, 9600, timeout=None) # no timeout  timeout=None
commandOut = None
print ard_serial
servo_center = 90
last_tilt = 0
last_roll = 0


'''
try:
	port_name = "ACM0" # default port for the Arduino
	if (len(sys.argv) > 2):
		port_name = sys.argv[1]
	
	ard_port = "/dev/tty" + port_name
	
	ard_serial = serial.Serial(ard_port, 9600, timeout=None) # no timeout
	commandOut = None
	print ard_serial
	servo_center = 90
except Exception as e:
	rospy.logerr('(ARD_PY): Cannot connect to Arduino on port: '+ ard_port)
	rospy.logerr(e)
'''
print ard_serial
# --------------------------------------------------------------------
def main():
	# Initialize ROS node
	rospy.init_node('arduino_listener')
	
	# Iinitialize serial port to arduino
	try:
		sleep(1)
		commandOut =  'e' + '5' + 'p' + str(90) \
				+ 't' + str(90) \
				+ 'r' + str(90) \
				+ 'b' + '0' 
		sendToArduino(commandOut)
		
	except Exception as e:
		rospy.logerr('(ARD_PY): Reading failed from arduino')
		rospy.logerr(e)

	# Subscribe to the Controller's stream
	print 'before subscriber'
	rospy.Subscriber('controller_data', ControllerMsg, callback, queue_size=1)
	print 'after subscriber'
	rospy.spin()
	
# --------------------------------------------------------------------
def callback(data):
	print 'callback'
	try:
		global servo_center, last_tilt, last_roll, ard_port, ard_serial, port_name, launchID
		 

		if data.ID != launchID:
			printScreenLog("ignoring message with id " + str(data.ID) + " because launch ID = " + str(launchID))
			return
		
		printScreenLog('(ARD_PY): Received from controller ' + 'p:' + str(data.pan) + ', t:' 
				+ str(data.tilt) + ', r:' + str(data.roll))
		
		"""
		# Zhefan
		new_pan =  servo_center-(data.pan)
		
		# remove jitter
		if math.fabs(data.tilt - last_tilt) < 3:
			data.tilt = last_tilt
		else:
			last_tilt = data.tilt
		
		if math.fabs(data.roll - last_roll) < 3:
			data.roll = last_roll
		else:
			last_roll = data.roll
		
		# amplify motion
		new_tilt = servo_center+(data.tilt*5)
		new_roll = servo_center-(data.roll*3)
		emotion = data.emotion;

		print("new tilt: %f" %(new_tilt))
		print("new roll: %f" %(new_roll))
		print("new pan: %f" %(new_pan))
		"""
		
		commandOut =  'e' + str((int)(data.emotion)) + \
					  'p' + str((int)(data.pan)) + \
					  't' + str((int)(data.tilt)) + \
					  'r' + str((int)(data.roll)) + \
					  'b' + str((int)(data.bop)) +'\0' 
		
		#print(commandOut)
		sendToArduino(commandOut)
	except Exception as e:
		print 'callback exception'
		rospy.logerr ("(ARD_PY) Sending to Arduino failed. Reopening serial connection.")
		rospy.logerr (e)
		
		# Iterate through available ports, try connecting to each one
		for port in list_ports.grep("ACM*"):
			try:
				ard_port = port[0]
				print "trying to connect to port " + str(ard_port)
				ard_serial = serial.Serial(ard_port, 9600, timeout=None)
				break
			except:# serial.SerialException:
				print "failed to connect to port " + str(ard_port)
				continue
				
		#ard_serial = serial.Serial(ard_port, 9600, timeout=None)
	print ''

# --------------------------------------------------------------------
def sendToArduino (commandOut): # reads msgs from arduino, if any
	global servo_center
	global ard_serial
	ard_serial.flushOutput();	# clear the output buffer in case of backlog
	printScreenLog('(ARD_PY) Sending: ' + commandOut)
	ard_serial.write(commandOut)
	#ard_serial.write("hello\0")

	try:

		while ard_serial.inWaiting() == 0:
			#print 'waiting for handshake'
			pass
		while ard_serial.inWaiting() != 0:
			msg = ard_serial.readline()
			tt = '(ARDUINO): ' + msg
			pan_pos = msg.find('pan:')	
			if pan_pos != -1:
				pan_val_end = msg.find(',', pan_pos)
				pan_value = int(msg[pan_pos+4:pan_val_end])
				servo_center = pan_value
			printScreenLog(tt)
	except serial.serialutil.SerialException:
		print "serial exception ? "

# --------------------------------------------------------------------
def printScreenLog (string):
	string = string.strip() + '\n' 
	sys.stdout.write(string)
	rospy.logdebug(string.strip())

# --------------------------------------------------------------------
if __name__ == '__main__':
	main()
