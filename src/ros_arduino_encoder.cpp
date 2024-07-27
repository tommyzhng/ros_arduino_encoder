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

        //ROS_INFO("Encoder X: %d, Y: %d, Z: %d", posX.value, posY.value, posZ.value);
    }
}

void RosArduinoEncoderNode::RecieveStepperCommandCb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    //std::cout << "Recieved stepper cmd:"  << msg->data<< std::endl;
    Send2Serial(msg->data[0], msg->data[1]);
}

// calculations
void RosArduinoEncoderNode::CalculatePosition()
{
    float angleX = (encoderRawMsg.vector.x * 360 / 30) * M_PI / 180;
    float angleY = (encoderRawMsg.vector.y * 360 / 30) * M_PI / 180;
    payloadPosMsg.vector.z = encoderRawMsg.vector.z * lengthPerTick;
    payloadPosMsg.vector.x = encoderRawMsg.vector.z * sin(angleX);
    payloadPosMsg.vector.y = encoderRawMsg.vector.z * sin(angleY);
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
    PubEncoderRaw();
    PubPayloadPos();
}

