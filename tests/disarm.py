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

sleep(6)

system("clear")

controller = MultiWii(serial)

if not controller: exit()

# ------- PLAYGROUND! Modify this freely -------
print('Disarming...')
controller.disarm()
print('Disarmed')

serial.close()
serial = None

# ----------------------------------------------

