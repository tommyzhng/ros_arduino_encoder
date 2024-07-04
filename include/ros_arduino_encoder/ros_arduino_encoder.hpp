#ifndef ROS_ARDUINO_ENCODER_ROS_ARDUINO_ENCODER_HPP
#define ROS_ARDUINO_ENCODER_ROS_ARDUINO_ENCODER_HPP

#include <iostream>
#include <ros/ros.h>
#include <cstdint>
#include <ctime>
#include <cmath>
#include <vector>
#include "serial/serial.h"
#include "geometry_msgs/Vector3Stamped.h"

#include <thread>
#include <chrono>

class RosArduinoEncoderNode
{
public:

    union MsgUint32_t {
        uint32_t value;
        uint8_t bits[4];
    };

    void Update(void);
    RosArduinoEncoderNode(ros::NodeHandle& nh);
    ~RosArduinoEncoderNode() = default; 

private:
    void InitializeSerial(serial::Serial& serial);
    void ReadEncoder(serial::Serial &serial);
    void PubEncoderRaw(void);

    std::unique_ptr<serial::Serial> serialPort;
    std::vector<uint8_t> buffer{13, 0};
    MsgUint32_t posX{0};
    MsgUint32_t posY{0};
    MsgUint32_t posZ{0};

    // ros pubs
    ros::Publisher encoderPub;
    geometry_msgs::Vector3Stamped encoderRawMsg;
};

#endif