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


