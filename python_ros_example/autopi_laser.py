#!/usr/bin/env python2
# script to control laser based on AutoPI state
#
# We need to read from the topics related to AutoPI task (see subscribers below) to know what is the current state (search / homing, trial number, ...)
# It could be better to have these information sent from the autopi_task directly as a topic.
# This program mainly defines the condition on WHEN the laser is ON or OFF (condition to start or stop it, or set the power or frequency, all based on the autopi state)
#
# Pascal, 14/06/2023

import rospy
from std_msgs.msg import Bool, UInt8, UInt32
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time

# state variables of the AutoPI task
trial = 0 # trial number
light = None # light on/off
doorstate = None # door open/closed
leverpressed = None # lever pressed (search/homing)
irbroken = None # magazine reward tried to obtained

# laser states
laserisrunning = False # laser running (start,stop)


# frequency in Hertz to pulse len in milliseconds
def freq2pulselen(freq, duty_cycle):
    pulse_total = 1000/freq
    pulse_len_on  = pulse_total * duty_cycle
    pulse_len_off = pulse_total * (1-duty_cycle)
    #return int(pulse_len_on),int(pulse_len_off)
    return int(pulse_len_on), int(pulse_total)-int(pulse_len_on)

power = 0.01 # 1 % of maximum power, low power
freq = 40 # 40 Hertz
duty = .3 # 30 %
#freq=4
#duty=.8
pulse_len_on, pulse_len_off = freq2pulselen(freq, duty) # publish to topic



def print_state():
    print("")
    print("STATE:")
    print(" trial:",trial)
    print(" light:",light)
    print(" doorstate:",doorstate)
    print(" leverpressed:",leverpressed)
    print(" irbroken:",irbroken)
    print(" laserisrunning:",laserisrunning)
    print("")




def set_laser(state):
    print("set laser", "ON" if state else "OFF")
    global laserisrunning
    laserisrunning = state
    pub_laserEnable.publish(state)


"""
Here is the definition of setting LASER either ON or OFF
"""

def trigger_laser():

    """
    # start laser on even trials as soon as the trial starts (door state) and the lever is not yet pressed
    if (trial % 2 == 0) and doorstate==False and leverpressed==False:
	#print("start laser")
	# publish to laser: START
        set_laser(True)

    # stop the laser as soon as the lever is pressed
    if (trial % 2 == 0) and doorstate==False and leverpressed==True:
	#print("stop laser")
	# publish to laser: STOP
        set_laser(False)

    """

    mintrial=1
    everytrial=2
    if trial>=mintrial and ((trial - mintrial)//everytrial)%2==0:
        if doorstate==False and irbroken==False:
            set_laser(True)
        if irbroken==True and leverpressed==True:
            set_laser(False)


"""
..
"""

# These callback functions mainly update the AutoPI state variables

def callbackDoorPosition(data):
    global doorstate
    global trial
    global irbroken
    global leverpressed
    doorstate = data.pose.position.z
    print("door =", doorstate)
    if doorstate==True:
        trial+=1
    else: # reset these states when trial is over, alternatively when new trial starts (door opens)
        irbroken=False
        leverpressed=False
    print_state()
    trigger_laser()

def callbackVisibleLightStatus(data):
    global light
    light = data.frame_id
    print("light =", light)
    print_state()

def callbackIrBeam(data):
    global irbroken
    irbroken = True
    print("irbroken =", irbroken)
    print_state()
    trigger_laser()

def callbackLeverBeam(data):
    global leverpressed
    global irbroken
    leverpressed = True
    irbroken = False
    print("leverpressed =", leverpressed)
    print_state()
    trigger_laser()


print("AUTOPI_LASER")
print("============")
print("")

print("use frequency",freq,", duty cycle",duty)
print("pulse len on/off",pulse_len_on, pulse_len_off)

# subscribe to AutoPI related states
rospy.init_node('autopi_laser')
rospy.Subscriber("door_position", PoseStamped, callbackDoorPosition)
rospy.Subscriber("visible_light_status", Header, callbackVisibleLightStatus)
rospy.Subscriber("ir_beam", Header, callbackIrBeam)
rospy.Subscriber("lever_beam", Header, callbackLeverBeam)

# publish to laser, see laserController repo for details
pub_laserEnable = rospy.Publisher("laser_enable", Bool, queue_size=1)
pub_laserPower = rospy.Publisher("laser_power", UInt8, queue_size=1)
pub_laserPulselenOn = rospy.Publisher("laser_pulselen_on", UInt32, queue_size=1)
pub_laserPulselenOff = rospy.Publisher("laser_pulselen_off", UInt32, queue_size=1)


rospy.loginfo("node started")

# rospy.spin()
def myhook():
    print("\nShutdown")
    set_laser(False) # shutdown laser on exiting this script !!

rospy.on_shutdown(myhook)

time.sleep(1)

# init
pub_laserPower.publish(int(255*power)) # set power
pub_laserPulselenOn.publish(pulse_len_on)
pub_laserPulselenOff.publish(pulse_len_off)

print("rospy name:", rospy.get_name())


print("Starting while loop")
print("\n")

while not rospy.is_shutdown():
    # this keeps looping forever
    rospy.sleep(1)

# program is shutting down
#~ time.sleep(0.5)
print("Goodbye")
