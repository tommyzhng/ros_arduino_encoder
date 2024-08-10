#include "ros_arduino_encoder.hpp"


RosArduinoEncoderNode::RosArduinoEncoderNode(ros::NodeHandle& nh)
{
    // serial
    encoderSerial = std::make_unique<serial::Serial>("/dev/ttyUSB0", 115200UL, serial::Timeout::simpleTimeout(10000));
    StartEncoderSerial(*encoderSerial);
    stepperSerial = std::make_unique<serial::Serial>("/dev/ttyACM0", 115200UL, serial::Timeout::simpleTimeout(10000));
    StartStepperSerial(*stepperSerial);

    // subscribe to converted commands
    stepperSub = nh.subscribe("/stepper/serial_command", 1, &RosArduinoEncoderNode::RecieveStepperCommandCb, this);

    // publish
    encoderPub = nh.advertise<geometry_msgs::Vector3Stamped>("/encoder/position_raw", 1);
    encoderVelPub = nh.advertise<geometry_msgs::Vector3Stamped>("/encoder/velocity_raw", 1);
    payloadPosPub = nh.advertise<geometry_msgs::Vector3Stamped>("/encoder/position_payload", 1);
    payloadVelPub = nh.advertise<geometry_msgs::Vector3Stamped>("/encoder/velocity_payload", 1);
}   

void RosArduinoEncoderNode::StartEncoderSerial(serial::Serial& serial)
{
    constexpr uint32_t inter_byte_timeout = 1000;
    constexpr uint32_t read_timeout_constant = 1000;
    constexpr uint32_t read_timeout_multiplier = 200;
    constexpr uint32_t write_timeout_constant = 1000; 
    constexpr uint32_t write_timeout_multiplier = 200;

    serial.setTimeout(inter_byte_timeout,
                      read_timeout_constant,
                      read_timeout_multiplier,
                      write_timeout_constant,
                      write_timeout_multiplier);

    serial.flushInput();
}

void RosArduinoEncoderNode::StartStepperSerial(serial::Serial& serial)
{
    serial.flushOutput();
    serial.flushInput();
}

// callbacks
void RosArduinoEncoderNode::ReadEncoder(serial::Serial &serial)
{
    serial.flushInput();
    auto flag = serial.waitReadable();
    if (!flag) {  // check for message received
        std::cout << "Timeout - No message received \n";
        return;
    }
    [[maybe_unused]] size_t length = serial.read(buffer.data(), 13); // read to buffer, byte length is 13 because header + 3x4 bytes

    if (buffer[0] == 0xEF) { 
        posX.bits[0] = buffer[1];
        posX.bits[1] = buffer[2];
        posX.bits[2] = buffer[3];
        posX.bits[3] = buffer[4];

        posY.bits[0] = buffer[5];
        posY.bits[1] = buffer[6];
        posY.bits[2] = buffer[7];
        posY.bits[3] = buffer[8];

        posZ.bits[0] = buffer[9];
        posZ.bits[1] = buffer[10];
        posZ.bits[2] = buffer[11];
        posZ.bits[3] = buffer[12];

        encoderRaw(0) = posX.value / 10.0;
        encoderRaw(1)= posY.value / 10.0;
        encoderRaw(2) = posZ.value;
        PubEncoderRaw();

        //ROS_INFO("Encoder X: %d, Y: %d, Z: %d", posX.value, posY.value, posZ.value);
    }
}

void RosArduinoEncoderNode::RecieveStepperCommandCb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    //std::cout << "Recieved stepper cmd:"  << msg->data<< std::endl;
    Send2Serial(msg->data[0], msg->data[1]);
}

// calculations
void RosArduinoEncoderNode::CalculatePosition(void)
{
    float angleX = (encoderRaw(0)) * M_PI / 180;
    float angleY = (encoderRaw(1)) * M_PI / 180;
    payloadPos(2) = encoderRaw(2) * lengthPerTick;
    payloadPos(0) = payloadPos(2) * sin(angleX);
    payloadPos(1) = payloadPos(2)* sin(angleY);
    PubPayloadPos();
}
void RosArduinoEncoderNode::CalculateRawVel(void)
{
    ros::Duration duration(0.016666667);
    encoderRawVel(0) = (encoderRaw(0) - lastEncoderRaw(0))/duration.toSec();
    encoderRawVel(1) = (encoderRaw(1) - lastEncoderRaw(1))/duration.toSec();
    encoderRawVel(2) = (encoderRaw(2) - lastEncoderRaw(2))/duration.toSec();
    lastEncoderRaw(0) = encoderRaw(0);
    lastEncoderRaw(1) = encoderRaw(1);
    lastEncoderRaw(2) = encoderRaw(2);
    
    PubEncoderRawVel();
}
void RosArduinoEncoderNode::CalculatePayloadVel(void)
{
    ros::Duration duration(0.016666667);
    payloadVel(0) = (payloadPos(0) - lastPayloadPos(0)) /  duration.toSec();
    payloadVel(1) = (payloadPos(1) - lastPayloadPos(1)) /  duration.toSec();
    payloadVel(2) = (payloadPos(2) - lastPayloadPos(2)) /  duration.toSec();
    lastPayloadPos(0) = payloadPos(0);
    lastPayloadPos(1)= payloadPos(1);
    lastPayloadPos(2) = payloadPos(2);
    PubPayloadVel();
}

// publish msgs
void RosArduinoEncoderNode::PubEncoderRaw(void)
{
    geometry_msgs::Vector3Stamped encoderRawMsg;
    encoderRawMsg.vector.x = encoderRaw(0);
    encoderRawMsg.vector.y = encoderRaw(1);
    encoderRawMsg.vector.z = encoderRaw(2);
    encoderPub.publish(encoderRawMsg);
}
void RosArduinoEncoderNode::PubEncoderRawVel(void)
{
    geometry_msgs::Vector3Stamped encoderRawVelMsg;
    encoderRawVelMsg.header.stamp = ros::Time::now();
    encoderRawVelMsg.vector.x = encoderRawVel(0);
    encoderRawVelMsg.vector.y = encoderRawVel(1);
    encoderRawVelMsg.vector.z = encoderRawVel(2);

    encoderVelPub.publish(encoderRawVelMsg);
}
void RosArduinoEncoderNode::PubPayloadPos(void)
{
    geometry_msgs::Vector3Stamped payloadPosMsg;
    payloadPosMsg.header.stamp = ros::Time::now();
    payloadPosMsg.vector.z = payloadPos(2);
    payloadPosMsg.vector.x = payloadPos(0);
    payloadPosMsg.vector.y = payloadPos(1);
    payloadPosPub.publish(payloadPosMsg);
}
void RosArduinoEncoderNode::PubPayloadVel(void)
{
    geometry_msgs::Vector3Stamped msg;
    encoderRawVelMsg.header.stamp = ros::Time::now();
    msg.vector.x = payloadVel(0);
    msg.vector.y = payloadVel(1);
    msg.vector.z = payloadVel(2);
    payloadVelPub.publish(msg);

}
void RosArduinoEncoderNode::Send2Serial(float len, float vel)
{
    stepperSerial->flush();
    // use string stream for stepperBuffer
    std::stringstream ss;

    // send a float with 5 digits by multiplying by 10000 and rounding
    len = round(len * 10000);
    vel = round(vel * 10000);

    // create stepperBuffer
    ss << len << " " << vel << "\n";
    std::string stepperBuffer = ss.str();
    stepperSerial->write(stepperBuffer.c_str());

    std::cout << "Sent to stepper: " << stepperBuffer << std::endl;

}
void RosArduinoEncoderNode::Update(void)
{
    ReadEncoder(*encoderSerial);
    CalculatePosition();
    CalculateRawVel();
    CalculatePayloadVel();
}

