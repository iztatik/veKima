#!/usr/bin/env python

import rospy
import time
import numpy as np
import Adafruit_PCA9685
from vekima.msg import cpg_msg
from geometry_msgs.msg import Vector3


# Initialize the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Configure min and max servo pulse lengths (out of 4096)
servo_min = 0.95  # (1ms)
servo_max = 2.1  # (2ms)

# Set frequency to 50hz
freq = 50
pwm.set_pwm_freq(freq)


# Helper function to make setting a servo pulse width simpler.
def set_servo_ms(channel, pulse):
    pulse_length = 1000000     # 1,000,000 us per second
    pulse_length /= freq       # 50 Hz
    pulse_length /= 4096       # 12 bits of resolution
    pulse *= 1000
    pulse /= pulse_length
    pwm.set_pwm(channel, 0,int(pulse))


def set_servo_deg(channel,angle):
    factor = (servo_max - servo_min)/2
    pulse = (angle*factor/90) + servo_min
    set_servo_ms(channel, pulse)

def gait_disarm():
    for i in range(12):
        pwm.set_pwm(i, 0, 0)
        time.sleep(0.05)
    return

class servos():
    def __init__(self):
        # Register subscriber to the master
        self.gait = rospy.Subscriber('cpg_topic', cpg_msg, self.gait_callback)          # Suscriber for the gait
        self.head = rospy.Subscriber('head_topic', Vector3, self.head_callback)         # Suscriber for the head
        self.tc_angles = None
        self.ctr_angles = None
        self.head = None;
    def gait_callback(self, msg):
        # Catch angles from CPG for the legs
        self.tc_angles, self.ctr_angles = msg.tc_angles, msg.ctr_angles
        set_servo_deg(0, self.tc_angles[0])
        set_servo_deg(2, self.tc_angles[1])
        set_servo_deg(4, self.tc_angles[2])
        set_servo_deg(6, self.tc_angles[3])
        set_servo_deg(8, self.tc_angles[4])
        set_servo_deg(10, self.tc_angles[5])
        
        set_servo_deg(1, self.ctr_angles[0])
        set_servo_deg(3, self.ctr_angles[1])
        set_servo_deg(5, self.ctr_angles[2])
        set_servo_deg(7, self.ctr_angles[3])
        set_servo_deg(9, self.ctr_angles[4])
        set_servo_deg(11, self.ctr_angles[5])

        set_servo_deg(12, self.tc_angles[0])
        set_servo_deg(13, self.tc_angles[1])
    def head_callback (self, msg):
        #  Catch angles for the head
        self.head = msg.x, msg.y
        set_servo_deg(12, self.head[0])
        set_servo_deg(13, self.head[1])


if __name__=='__main__':
    try:
        # Initialize the node, register the subscrieber to the master creating a CPG instance and wait until callback
        rospy.init_node('PCA9685', anonymous=False)
        my_servos = servos()
        print('PCA9685: Initializing...')
        print('PCA9685: READY')
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
