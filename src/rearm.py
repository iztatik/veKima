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
try:
    while True:
        input('Press enter to arm ')
        print("Arming...")
        controller.arm()
        controller.set_channels([1500, 1500, 1500, 1100])
        print("Armed")

        input('Press enter to disarm ')
        print("Disarming...")
        controller.disarm()
        print("Disarmed")

except KeyboardInterrupt:
    serial.close()
    serial = None
    exit()

serial.close()
serial = None
