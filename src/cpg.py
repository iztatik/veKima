#!/usr/bin/env python

import numpy as np
import time
import matplotlib.pyplot as plt
from numba import jit

from oscillator import myODE
from mapper import tc_joint, ctr_joint

import rospy
from vekima.msg import cpg_msg
from dynamic_reconfigure.server import Server
from vekima.cfg import CPGConfig

# CPG constants
sample_time = 0.018                      # Sample time
sample_freq = 1/sample_time
ctr_offsset = 30


# Define an CPG object
class cpg():
    def __init__(self):
        # Register(topic) to the master
        self.pub = rospy.Publisher('cpg_topic', cpg_msg, queue_size=10)
        # Register the dynamic reconfigure server to the master
        self.srv = Server(CPGConfig, self.reconfigure)
        # Gait activation flag
        self.walk = False

    def reconfigure(self, config, level):
        direction = -90
        if config['Walk'] == True: 
            self.walk = True;
            if config['Direction'] == True: direction = -90
            else: direction = 90
            myODE.hopf_parameters(config['TC_Left_amp'], config['TC_Right_amp'], config['CTr_amp'], 2*np.pi/config['CPG_period'], direction)
        else: 
            self.walk = False
        return config
    
    def arm(self):
        message = cpg_msg()
        right= 90-ctr_offsset
        left = 90+ctr_offsset
        message.tc_angles, message.ctr_angles = np.full(6,90), np.array([right,left,right,left,right,left])
        self.pub.publish(message)
        
    def disarm(self):
        
        message = cpg_msg()
        right= 20
        left = 150
        fact = 10
        message.tc_angles = np.full(6,90)
        message.ctr_angles =  np.array([right+fact,left-fact,right+fact,left-fact,right+fact,left-fact])
        self.pub.publish(message)
        time.sleep(0.1)
        message.ctr_angles =  np.array([right,left,right,left,right,left])
        self.pub.publish(message)
        
    def align(self):
        message = cpg_msg()
        message.ctr_angles = np.array([90-ctr_offsset, 163, 13, 90+ctr_offsset, 90-ctr_offsset, 163])
        message.tc_angles = np.full(6,90)
        self.pub.publish(message)
        


# Integrates the Hopf oscillator and maps to joint angles 
def get_angles(ti):
    ordened = np.zeros([12])
    ordened[:] = myODE.hopf_integrate(ti)
    return tc_joint(ordened[0:6]), ctr_joint(ordened[6:12], ctr_offsset)

# Check stop condition
@jit()
def stop_condition():
    return np.round(np.mean(np.absolute([myODE.raw[9],myODE.raw[11],myODE.raw[13],myODE.raw[15]])),1)!=0
            

# Publish joint angles to a suscriber
def cpg_publish():
    t = sample_time
    message = cpg_msg() 
    while not rospy.is_shutdown():
        try:
            if my_cpg.walk or stop_condition():
                message.tc_angles, message.ctr_angles = get_angles(t)
                my_cpg.pub.publish(message)
                t += sample_time
            else: 
                t = sample_time
                myODE.hopf_restart()
                myODE.raw[8:] = 0
                myODE.hopf_initial(myODE.raw, 0)
                # BORRAR
                time.sleep(1)
                message.ctr_angles = np.array([90-ctr_offsset, 163, 13, 90+ctr_offsset, 90-ctr_offsset, 163])
                time.sleep(1)
                message.tc_angles = np.full(6,90)
                my_cpg.pub.publish(message)
                time.sleep(1)
                my_cpg.disarm()
                # BORRAR
                while my_cpg.walk == False: pass
            rate.sleep()
        except KeyboardInterrupt:
            my_cpg.align()
            #time.sleep(0.5)
            my_cpg.disarm()
            print('CPG: Disarmed')
            rospy.signal_shutdown('Manual shutdowm')


if __name__ == '__main__':
    try:
        print('CPG: Initializing...')
        # Initialize the node, register(topic) to the master creating a CPG instance and set publication rate
        rospy.init_node('CPG', anonymous=False, disable_signals= True) 
        my_cpg = cpg()
        rate = rospy.Rate(sample_freq)
        # Get first values (to avoid the time of C functions compilations)
        get_angles(sample_time)
        stop_condition()
        print('CPG: Armed')
        my_cpg.arm()        
        cpg_publish()
    except rospy.ROSInterruptException:
        pass
    
    
