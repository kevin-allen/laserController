#!/usr/bin/env python
#
# example script to control laser using ROS
# This serves as an example to demonstrate interaction with laser.
#
# set params below and run script
# one could alternatively have argparse handle power, duty cycle, frequency
#
# Pascal, 16/06/2023

import rospy
from std_msgs.msg import Bool, UInt8, UInt32
import time

# set params here ####################
laser_enable = True
laser_power = .25 # 25% of maximum power
freq = 40 # 40 Hertz
duty = .3 # 30 %
######################################

# frequency in Hertz to pulse len in milliseconds
# T = pulse_len_on + pulse_len_off
# f = 1/T
# D = pulse_len_on / T
#
def freq2pulselen(freq, duty_cycle):
    pulse_total = 1000/freq
    pulse_len_on  = pulse_total * duty_cycle
    pulse_len_off = pulse_total * (1-duty_cycle)
    #return int(pulse_len_on),int(pulse_len_off)
    return int(pulse_len_on), int(pulse_total)-int(pulse_len_on) # better for rounding

# convert input to laser ros compatible values
pulse_len_on, pulse_len_off = freq2pulselen(freq, duty) # publish to topic
laser_power_int = int(255*laser_power)

print("CONTROL LASER")
print("=============")
print("")

print("enable =",laser_enable)
print("use frequency =",freq,", duty cycle =",duty)
print("pulse len on/off",pulse_len_on, pulse_len_off)
print("use power",laser_power,", send=",laser_power_int)

rospy.init_node('control_laser')

pub_laserEnable = rospy.Publisher("laser_enable", Bool, queue_size=1)
pub_laserPower = rospy.Publisher("laser_power", UInt8, queue_size=1)
pub_laserPulselenOn = rospy.Publisher("laser_pulselen_on", UInt32, queue_size=1)
pub_laserPulselenOff = rospy.Publisher("laser_pulselen_off", UInt32, queue_size=1)


rospy.loginfo("node started")

time.sleep(1)

pub_laserEnable.publish(laser_enable) # enable or disable
pub_laserPower.publish(laser_power_int) # average power
pub_laserPulselenOn.publish(pulse_len_on)
pub_laserPulselenOff.publish(pulse_len_off)

print("rospy name:", rospy.get_name())
print("Goodbye")
