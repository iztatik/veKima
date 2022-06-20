#!/usr/bin/env python

import rospy
from dynamic_reconfigure.server import Server
from vekima.cfg import CPGConfig

import json
import serial as pyserial

from os import system
from time import sleep
from WiiProxy import MultiWii

# --------------------------------------------------

controller = None
serial = None

# ------------ Serial Initialization ---------------

serial = pyserial.Serial()

serial.port = "/dev/ttyUSB0"
serial.baudrate = 115200
serial.bytesize = pyserial.EIGHTBITS
serial.parity = pyserial.PARITY_NONE
serial.stopbits = pyserial.STOPBITS_ONE
serial.write_timeout = 3
serial.xonxoff = False
serial.rtscts = False
serial.dsrdtr = False

serial.open()
sleep(3)
system("clear")
controller = MultiWii(serial)
if not controller: exit()


def callback(config,level):
	global speed
	global armed 	
	 
	if config['Arm']!=armed:
		armed = config['Arm']
		if config['Arm']:
			controller.arm()
			print('Armed: ',config['Arm'])
		else:
			controller.disarm()
			print('Armed: ',config['Arm'])
		
	speed = config['Propellers']
	controller.set_channels([1500, 1500, 1500, speed])
	print('Speed: ',speed)
	return config

global speed 
global armed 
speed = 1200
armed = False

# ------------- Main ------------------------
if __name__=='__main__':
	try:
		rospy.init_node('flight', anonymous=False, disable_signals= True)
		srv = Server(CPGConfig, callback)
		controller.disarm()
		while True:
			if armed:
				controller.set_channels([1500, 1500, 1500, speed])
				rospy.sleep(0.08)
			else:
				#print('armed: OFF')
				rospy.sleep(1)	

	except rospy.ROSInterruptException:
		controller.disarm()
		serial.close()
		print('')
		serial = None
		exit()
		
	except KeyboardInterrupt:
		controller.disarm()
		serial.close()
		print('')
		serial = None
		exit()

			
