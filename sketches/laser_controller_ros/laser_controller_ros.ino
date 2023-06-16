/* 
 *  Laser controller
 *  ROS node
 *  
 *  for CrystaLaser CL-2005 laser power supply
 *  https://neurophysics.ucsd.edu/Manuals/CrystaLaser/Diode%20Pumped%20Solid%20State%20CrystaLaser.pdf
 *
 *
 * Pascal
 * 14/06/2023 - 15/06/2023
 * 
 * 
 * topics: 
 *   laser_enable: bool enabled/disabled
 *   laser_power: byte 0-255 relative power of max power set on controller
 *   laser_pulselen_on: pulse length [ms] = 1/(2*frequency), duty cycle on [ms] = laser_pulselen_on / (laser_pulselen_on + laser_pulselen_off)
 *   laser_pulselen_off: pulse off time [ms], set 0 for constant on
 * (could have a topic laser_pulselen to set both to the same time, that is duty cycle = 50%)
 *
 * running on  
 *   /dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_9513731393235111C1A1-if00
 * 
 * commands:
 *   roscore
 *   rosrun rosserial_python serial_node.py /dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_9513731393235111C1A1-if00 __name:=node_lasercontroller
 *   rostopic pub -1 /laser_enable std_msgs/Bool 1
 *   rostopic pub -1 /laser_power std_msgs/UInt8 123
 *   rostopic pub -1 /laser_pulselen std_msgs/UInt32 99
 *
 */

// ROS includes with messages for topics
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>

// PINOUT
//  PIN 13 pulse: on/off based on frequency
//  Pin  3 power: analog output (ratio of max power)
//  connect interlock
#define PIN_PULSE 13
#define PIN_POWER 3

// laser pulse sending
unsigned long time_last_pulse_on = 0;
unsigned long time_last_pulse_off = 0;
unsigned long time_now;

// laser parameters
bool laser_enabled = false;
uint32_t laser_pulselen_on = 500;
uint32_t laser_pulselen_off = 500;
bool laser_state = false;
bool constant_on = false;


// === ROS ===
ros::NodeHandle nh;

// callback functions for the topics

// callback for topic "laser_enable"
void cb_LaserEnable(const std_msgs::Bool& msg){
  nh.loginfo("callback laser enable");
  bool x = msg.data;
  laser_enabled = x;
  
  if (laser_enabled){
    // start laser and sync to now time, so that loop() will continue with correct time length
    digitalWrite(PIN_PULSE, HIGH);
    laser_state = true;
    time_last_pulse_on = millis();
    nh.loginfo("enable");
  }
  else {
    // disable, directly shut down
    digitalWrite(PIN_PULSE, LOW);
    nh.loginfo("disable");
  }
}

// callback for topic "laser_power"
void cb_LaserPower(const std_msgs::UInt8& msg){
  nh.loginfo("callback laser power");
  uint8_t x = msg.data;
  analogWrite(PIN_POWER, x); // set laser power as defined by ros message
}

// callback for topic "laser_pulselen_on"
void cb_LaserPulselenOn(const std_msgs::UInt32& msg){
  nh.loginfo("callback laser pulselen on");
  uint32_t x = msg.data;
  
  laser_pulselen_on = x;
  nh.loginfo("set pulse len"); 
}

// callback for topic "laser_pulselen_off"
void cb_LaserPulselenOff(const std_msgs::UInt32& msg){
  nh.loginfo("callback laser pulselen off");
  uint32_t x = msg.data;
  
  if (x == 0){
    constant_on = true;
    nh.loginfo("constant on");
  }
  else {
    constant_on = false;
    laser_pulselen_off = x;
    nh.loginfo("set pulse len");
  }
}

// subscribers for the topics
ros::Subscriber<std_msgs::Bool> subLaserEnable("laser_enable", &cb_LaserEnable);
ros::Subscriber<std_msgs::UInt8> subLaserPower("laser_power", &cb_LaserPower);
ros::Subscriber<std_msgs::UInt32> subLaserPulselenOn("laser_pulselen_on", &cb_LaserPulselenOn);
ros::Subscriber<std_msgs::UInt32> subLaserPulselenOff("laser_pulselen_off", &cb_LaserPulselenOff);


// INITIALIZE

void setup()
{
  pinMode(PIN_PULSE, OUTPUT);
  pinMode(PIN_POWER, OUTPUT);
  digitalWrite(PIN_PULSE, LOW);
  digitalWrite(PIN_POWER, LOW);

  nh.initNode();
  nh.subscribe(subLaserEnable);
  nh.subscribe(subLaserPower);
  nh.subscribe(subLaserPulselenOn);
  nh.subscribe(subLaserPulselenOff);

  delay(500);
  nh.loginfo(" === Arduino Laser Controller === ");
}

void loop()
{
  // ROS handling
  nh.spinOnce();
  delay(1);

  // Laser pulses handling
  time_now = millis();
  if (laser_enabled){
    if (constant_on){
      digitalWrite(PIN_PULSE, HIGH);
    }
    else
    {
      if (time_now-time_last_pulse_on > laser_pulselen_on && laser_state==HIGH){
        laser_state = LOW;
        digitalWrite(PIN_PULSE, laser_state);
        time_last_pulse_off = time_now;
      }
      if (time_now-time_last_pulse_off > laser_pulselen_off && laser_state==LOW){
        laser_state = HIGH;
        digitalWrite(PIN_PULSE, laser_state);
        time_last_pulse_on = time_now;
      }
    }
  }

}
