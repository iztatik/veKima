#!/usr/bin/env python

import json
import serial as pyserial

from os import system
from time import sleep
from WiiProxy import MultiWii

# --------------------------------------------------

controller = None
serial = None

# --------------------------------------------------

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

sleep(4)

system("clear") 

controller = MultiWii(serial)

if not controller: exit()

# ------- PLAYGROUND! Modify this freely -------
print("Arming...")
controller.arm()
print("Armed")

controller.set_channels([1500, 1500, 1500, 1100])
sleep(3)
controller.set_channels([1500, 1500, 1500, 1300])
sleep(3)
controller.set_channels([1500, 1500, 1500, 1500])
sleep(3)
controller.set_channels([1500, 1500, 1500, 1800])
sleep(3)
controller.set_channels([1500, 1500, 1500, 2000])
sleep(3)
controller.set_channels([1500, 1500, 1500, 1000])
sleep(3)



serial.close()
serial = None

# ----------------------------------------------
