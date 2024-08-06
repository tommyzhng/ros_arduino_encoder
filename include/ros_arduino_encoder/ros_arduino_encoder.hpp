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
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

#include <thread>
#include <chrono>

class RosArduinoEncoderNode
{
public:

    union MsgUint32_t {
        int32_t value;
        uint8_t bits[4];
    };

    union MsgUfloat {
        float value;
        uint8_t bits[sizeof(float)];
    };

    void Update(void);
    RosArduinoEncoderNode(ros::NodeHandle& nh);
    ~RosArduinoEncoderNode() = default;

private:
    void StartEncoderSerial(serial::Serial& serial);
    void ReadEncoder(serial::Serial &serial);
    void StartStepperSerial(serial::Serial& serial);
    void RecieveStepperCommandCb(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void CalculatePosition();
    void PubEncoderRaw(void);
    void PubPayloadPos(void);
    void PubPayloadVel(void);
    void Send2Serial(float len, float vel);

    // encoder serial
    std::unique_ptr<serial::Serial> encoderSerial;
    std::vector<uint8_t> buffer{13, 0};
    // add stepper BUffer
    const char* stepperBuffer;
    MsgUint32_t posX{0};
    MsgUint32_t posY{0};
    MsgUint32_t posZ{0};
    float lastPayloadX{0};
    float lastPayloadY{0};
    float lastPayloadZ{0};
    float velX{0};
    float velY{0};
    float velZ{0};
    float lengthPerTick = (M_PI * 0.05) / 30;
    // stepper serial
    std::unique_ptr<serial::Serial> stepperSerial;
    // ros subs
    ros::Subscriber stepperSub;
    // ros pubs
    ros::Publisher encoderPub;
    ros::Publisher payloadPosPub;
    ros::Publisher payloadVelPub;
    geometry_msgs::Vector3Stamped encoderRawMsg;
    geometry_msgs::Vector3Stamped payloadPosMsg;
    geometry_msgs::Vector3Stamped encoderRawSpdMsg;
    float temp = 0;
};

#endif