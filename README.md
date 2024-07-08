# ROS Arduino Encoder

A ros node that reads serial data from an encoder for a variable length slung-payload drone and outputs to a topic for further processing. Also can read a command topic to send a serial message to another Arduino to control a stepper motor. This was used instead of rosserial-arduino to keep the processing load off of the Arduino.

Serial connections
* ttyUSB0 @ baudrate of 115200
* ttyACM0 @ baudrate of 115200

Subscribes to:
* /stepper/setpoint_length
Publishes to:
* /encoder/position_raw
* /encoder/position_payload

Requires:

* Place this in /include https://github.com/wjwwood/serial
