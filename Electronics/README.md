
# Hardware

The gripper is drive by a modified servo. The servo controller board was 
removed and replaced by a Arduino Nano, TB6612FNG motor controller and a ACS712 current sensor.
The two channels of the motor controller a tied together to drive the servo at up to 2.5 Amps continously,
and a 6 Amp peak. 

## Wiring

Arduino Atmel  IO
 D10    PB2    O   PWM, TB6612FNG
 D13    PB5    O   LED

 A0     ADC0   A   Current, ACS712
 A1     ADC1   A   Position 0 - 5V.  Center tap of servo potentiometer. Other two taps connected to 0 and 5V 

 D2     PD2    O   Ain1, TB6612FNG
 D3     PD3    O   Standby, TB6612FNG
 D4     PD4    O   Ain,  TB6612FNG

Note: Connect the grounds should be connected so that the motor current doesn't share the same connection as 
the either the current sensor or servo potentiometer.

## Parts

Servos:  GS-5509MG  9 kg 
Controller:  Arduino Nano v3.0 compatible ATMEGA328 (CH340 USB-Serial) 16MHz, operating at 5V
Motor driver: TB6612FNG Dual Motor Driver
Current sensor: ACS712 Current Sensor Module (5A range)