#! /usr/bin/env python

import roslib; roslib.load_manifest('mykeepon_museum')
import rospy
#from mykeepon_museum.msg import OmronMsg
from mykeepon_museum.msg import ControllerMsg

def main():

	# Initialize ROS node
	rospy.init_node('arduino_listener')
	
	# Subscribe to the Controller's stream
	rospy.Subscriber('controller_data', ControllerMsg, callback)
	rospy.spin()
	
def callback(data):
	print ('pan: ' + str(data.pan) 
			+ ', tilt: ' + str(data.tilt) 
			+ ', roll: ' + str(data.roll))

if __name__ == '__main__':
	main()
