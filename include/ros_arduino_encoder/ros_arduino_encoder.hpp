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
#include <eigen3/Eigen/Dense>

#ifndef window_size
#define window_size 5
#endif

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
    void CalculatePosition(void);
    void CalculateRawVel(void);
    void CalculateRawPayloadVel(void);
    Eigen::Vector3d MovingAverage(Eigen::Vector3d newVel, Eigen::Vector3d velWindow[window_size]);
    void PubEncoderRaw(void);
    void PubEncoderRawVel(void);
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

    float lengthPerTick = (M_PI * 0.052) / 1200;
    // stepper serial
    std::unique_ptr<serial::Serial> stepperSerial;
    // ros subs
    ros::Subscriber stepperSub;
    // ros pubs
    ros::Publisher encoderPub;
    ros::Publisher encoderVelPub;
    ros::Publisher payloadPosPub;
    ros::Publisher payloadVelPub;


    // speed testing
    // variables to store subscriber data
    Eigen::Vector3d encoderRaw{0,0,0};
    Eigen::Vector3d lastEncoderRaw{0,0,0};
    Eigen::Vector3d encoderRawVel{0,0,0};
    Eigen::Vector3d payloadPos{0,0,0};
    Eigen::Vector3d lastPayloadPos{0,0,0};
    Eigen::Vector3d payloadRawVel{0,0,0};

    // for velocity moving average
    Eigen::Vector3d encoderVelWindow_[window_size];
    Eigen::Vector3d encoderVelFiltered_{0,0,0};
    Eigen::Vector3d payloadVelWindow_[window_size];
    Eigen::Vector3d payloadVelFiltered_{0,0,0};
    
};

#endif