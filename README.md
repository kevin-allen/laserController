# laserController
Repository to describe how to control a laser using a python script and an Arduino Uno

## Material needed

* Arduino Uno
* Arduino Shield (see Eagle files)
* USB cable
* Ethernet cable
* Ethernet socket for the Arduino shield
* Laser head ()
* Laser power ()
* ...

## Installation steps

Ethernet socket:
* pin 2: analog input, set power (0 to 5V defines the ratio of maximum power as defined using the manual poti)
* pin 3: pulse on/off (TTL)
* pin 4 & 5: connect together, apply HIGH for ON, apply LOW for OFF, don't apply any voltage there: key will turn on/off
* pin 6: ground


