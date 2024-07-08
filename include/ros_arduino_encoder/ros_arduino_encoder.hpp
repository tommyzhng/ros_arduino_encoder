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
    void StartEncoderSerial(serial::Serial& serial);
    void ReadEncoder(serial::Serial &serial);
    void StartStepperSerial(serial::Serial& serial);
    void RecieveStepperCommandCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
    void CalculatePosition();
    void PubEncoderRaw(void);
    void PubPayloadPos(void);
    void SendStepperCommand(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

    // encoder serial
    std::unique_ptr<serial::Serial> encoderSerial;
    std::vector<uint8_t> buffer{13, 0};
    MsgUint32_t posX{0};
    MsgUint32_t posY{0};
    MsgUint32_t posZ{0};
    float lengthPerTick = (M_PI * 0.5) / 30;
    // stepper serial
    std::unique_ptr<serial::Serial> stepperSerial;
    // ros subs
    ros::Subscriber stepperSub;
    // ros pubs
    ros::Publisher encoderPub;
    ros::Publisher payloadPosPub;
    geometry_msgs::Vector3Stamped encoderRawMsg;
    geometry_msgs::Vector3Stamped payloadPosMsg;
};

#endif