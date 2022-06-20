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
print("Arming...")

controller.arm()

print("Armed")


print('Initial:', controller.get_channels())
while True:
    i = 0
    try:
        for i in range(50):
            controller.set_channels([1500, 1500, 1500, 1000])
            print('Channels_A1:', controller.get_channels())
            sleep(0.01)
        for i in range(50):
            controller.set_channels([1500, 1500, 1500, 1200])
            print('Channels_A1:', controller.get_channels())
            sleep(0.01)
        for i in range(50):
            controller.set_channels([1500, 1500, 1500, 1400])
            print('Channels_A1:', controller.get_channels())
            sleep(0.01)
        for i in range(50):
            controller.set_channels([1500, 1500, 1500, 1600])
            print('Channels_B1:', controller.get_channels())
            sleep(0.01)
        for i in range(50):
            controller.set_channels([1500, 1500, 1500, 1800])
            print('Channels_A1:', controller.get_channels())
            sleep(0.01)
        for i in range(50):
            controller.set_channels([1500, 1500, 1500, 1999])
            print('Channels_A1:', controller.get_channels())
            sleep(0.01)


    except KeyboardInterrupt:
        controller.disarm()
        print('Disarming...')
        exit()

sleep(2)
# ----------------------------------------------

serial.close()

serial = None