# ROS Arduino Encoder

A ros node that reads serial data from an encoder and outputs to a topic for further processing.

Subscribes to / Input:
* ttyACM0 @ baudrate of 115200

Publishes to:
* /serial/encoder

Requires:

* Place this in /include https://github.com/wjwwood/serial
