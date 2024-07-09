#include "ros_arduino_encoder.hpp"


RosArduinoEncoderNode::RosArduinoEncoderNode(ros::NodeHandle& nh)
{
    // serial
    encoderSerial = std::make_unique<serial::Serial>("/dev/ttyUSB0", 115200UL, serial::Timeout::simpleTimeout(10000));
    StartEncoderSerial(*encoderSerial);
    stepperSerial = std::make_unique<serial::Serial>("/dev/ttyACM0", 115200UL, serial::Timeout::simpleTimeout(10000));
    StartStepperSerial(*stepperSerial);

    // subscribe to gui message
    stepperSub = nh.subscribe("/stepper/setpoint_length", 1, &RosArduinoEncoderNode::RecieveStepperCommandCb, this);

    // publish
    encoderPub = nh.advertise<geometry_msgs::Vector3Stamped>("/encoder/position_raw", 1);
    payloadPosPub = nh.advertise<geometry_msgs::Vector3Stamped>("/encoder/position_payload", 1);
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

        encoderRawMsg.vector.x = posX.value;
        encoderRawMsg.vector.y = posY.value;
        encoderRawMsg.vector.z = posZ.value;
    }
}

void RosArduinoEncoderNode::RecieveStepperCommandCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    std::cout << "Recieved stepper cmd: " << msg->vector.x << " " << msg->vector.y << " " << msg->vector.z << std::endl;
    SendStepperCommand(msg);
}

// calculations
void RosArduinoEncoderNode::CalculatePosition()
{
    float angle = (encoderRawMsg.vector.x * 360 / 30) * M_PI / 180;
    payloadPosMsg.vector.z = encoderRawMsg.vector.z * lengthPerTick;
    encoderRawMsg.vector.x = encoderRawMsg.vector.z * sin(angle);
    encoderRawMsg.vector.y = encoderRawMsg.vector.z * cos(angle);
}

// publish msgs
void RosArduinoEncoderNode::PubEncoderRaw(void)
{
    encoderPub.publish(encoderRawMsg);
}
void RosArduinoEncoderNode::PubPayloadPos(void)
{
    payloadPosPub.publish(payloadPosMsg);
}
void RosArduinoEncoderNode::SendStepperCommand(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    // only send z value
    stepperSerial->flush();
    buffer[0] = 0xEF;
    buffer[1] = msg->vector.z;
    stepperSerial->write(buffer);
    // print message sent
    std::cout << "Sent message: " << static_cast<int>(buffer[0]) << " " << static_cast<int>(buffer[1]) << std::endl;
}
void RosArduinoEncoderNode::Update(void)
{
    ReadEncoder(*encoderSerial);
    CalculatePosition();
    PubEncoderRaw();
}

