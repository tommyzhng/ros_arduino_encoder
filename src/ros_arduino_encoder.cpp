#include "ros_arduino_encoder.hpp"


RosArduinoEncoderNode::RosArduinoEncoderNode(ros::NodeHandle& nh)
{
    encoder_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/serial/encoder_raw", 1);
    serialPort = std::make_unique<serial::Serial>("/dev/ttyUSB0", 115200UL, serial::Timeout::simpleTimeout(10000));
    InitializeSerial(*serialPort);
}

void RosArduinoEncoderNode::InitializeSerial(serial::Serial& serial)
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
}

void RosArduinoEncoderNode::ReadEncoder(serial::Serial &serial)
{
    auto flag = serial.waitReadable();
    if (!flag) {  // check for message received
        std::cout << "Timeout - No message received \n";
        return;
    }
    [[maybe_unused]] size_t length = serial.read(buffer.data(), 10); // read to buffer

    // print debug

    if (buffer[0] == 0xEF) { 
        pos_x.bits[0] = buffer[1];
        pos_x.bits[1] = buffer[2];
        pos_x.bits[2] = buffer[3];
        pos_x.bits[3] = buffer[4];

        // pos_y.bits[0] = buffer[5];
        // pos_y.bits[1] = buffer[6];
        // pos_y.bits[2] = buffer[7];
        // pos_y.bits[3] = buffer[8];

        // pos_z.bits[0] = buffer[9];
        // pos_z.bits[1] = buffer[10];
        // pos_z.bits[2] = buffer[11];
        // pos_z.bits[3] = buffer[12];

        encoder_raw_msg.vector.x = pos_x.value;
        encoder_raw_msg.vector.y = pos_y.value;
        encoder_raw_msg.vector.z = pos_z.value;
    }
}

// publish msgs

void RosArduinoEncoderNode::Update(void)
{
    ReadEncoder(*serialPort);
    PubEncoderRaw();
}

void RosArduinoEncoderNode::PubEncoderRaw(void)
{
    encoder_pub.publish(encoder_raw_msg);
}
