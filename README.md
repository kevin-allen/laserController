# laserController

This repository describes our setup used to perform optogenetic stimulation in behavioral experiments. 

The laser pulses are controlled using a Python script and a TTL signal delivered from an Arduino Uno.

## Material needed

* Arduino Uno
* Arduino Shield (see Eagle files)
* USB cable
* Ethernet cable
* Ethernet socket for the Arduino shield
* Laser head ()
* Laser power ()
* ...

## Arduino shield

The Eagle files of the Arduino sheild are in the folder eagleFiles. You can use these files to generate Gerber files and order the shield from EuroCircuits.
The shield has one Ethernet cable that goes to the laser power box. There are 2 BNC sockets that can be used to send the TTL and power signal to the recording system. The signal will range from 0 to 5V so make sure that the hardware accepting these signals can cope with a 5V signal.

## Installation steps

Ethernet socket:
* pin 2: analog input, set power (0 to 5V defines the ratio of maximum power as defined using the manual poti)
* pin 3: pulse on/off (TTL)
* pin 4 & 5: connect together, apply HIGH for ON, apply LOW for OFF, don't apply any voltage there: key will turn on/off
* pin 6: ground

# Laser controller ROS

These *topics* are subscribed to by the Arduino connected to the Laser controller.

 *   `laser_enable`: bool enabled/disabled
 *   `laser_power`: byte 0-255 relative power of max power set on controller
 *   `laser_pulselen_on`: pulse length [ms] = 1/(2*frequency),  
       duty cycle on [ms] = laser_pulselen_on / (laser_pulselen_on + laser_pulselen_off)
 *   `laser_pulselen_off`: pulse off time [ms], set 0 for constant on

The former (`laser_controller__pulse50dutycycle.old`) sketch only had fixed 50% duty cycle and `laser_pulselen` as corresponding ros topic.


# Usage example
 You can
 - use the example file `python_ros_example/control_laser.py` and set the power, pulse_len and duty_cycle , or
 - use rostopic to publish , see description like in `sketches/laser_controller_ros/laser_controller_ros.ino`
     *   enable: `rostopic pub -1 /laser_enable std_msgs/Bool 1`
     *   set power: `rostopic pub -1 /laser_power std_msgs/UInt8 200`
     *   set pulse len on duration: `rostopic pub -1 /laser_pulselen_on std_msgs/UInt32 5`
     *   set pulse len off duration: `rostopic pub -1 /laser_pulselen_off std_msgs/UInt32 95`

# pulse length, frequency and duty cycle
- $t_\mathrm{on}$: pulse len on duration
- $t_\mathrm{off}$: pulse len off duration
- $T=t_\mathrm{on}+t_\mathrm{off}$: period duration
- $f=\frac{1}{T}$: frequency
- $d = \frac{t_\mathrm{on}}{T}$: duty cycle ( $0 < d \leq 1$ ), <sup><sub>The case `d=1` is treated individually and handled without timers.</sub></sup>

example ( [paper](https://pubmed.ncbi.nlm.nih.gov/35584671/) ):
"stimulation at 10 Hz with a 5 ms duration "
- $f=10\mathrm{Hz},\ T=100\mathrm{ms},\ t_\mathrm{on}=5\mathrm{ms},\ d=0.05$  (this example is acheived by above ros command)

Nota bene: The effective power (measured on average) will scale $\propto d$ as this is the raio of on duration.
