#include <ros/ros.h>
#include "ros_arduino_encoder.hpp"

// constexpr uint8_t header = 0xEF;

// union MsgUint32_t {
//   uint32_t value;
//   uint8_t bits[4];
// };

// class Encoder2RosNode {
// public:
//     Encoder2RosNode()
//     : nh("~"), encoder_pub(nh.advertise<geometry_msgs::Vector3>("/serial/encoder", 1)), buffer(10, 0)
//     {
//         serial = std::make_unique<serial::Serial>("/dev/ttyUSB0", 9600UL, serial::Timeout::simpleTimeout(10000));
//         configureSerial(*serial);

//         ros::Rate rate(100.0);
//         while (ros::ok())
//         {
//             readEncoder(*serial);
//             ros::spinOnce();
//             rate.sleep();
//         }
//     }

// private:
//     ros::NodeHandle nh;
//     ros::Publisher encoder_pub;
//     std::unique_ptr<serial::Serial> serial;
//     std::vector<uint8_t> buffer;

//     void configureSerial(serial::Serial& serial)
//     {
//         constexpr uint32_t inter_byte_timeout = 1000;
//         constexpr uint32_t read_timeout_constant = 1000;
//         constexpr uint32_t read_timeout_multiplier = 200;
//         constexpr uint32_t write_timeout_constant = 1000; 
//         constexpr uint32_t write_timeout_multiplier = 200;

//         serial.setTimeout(inter_byte_timeout,
//                           read_timeout_constant,
//                           read_timeout_multiplier,
//                           write_timeout_constant,
//                           write_timeout_multiplier);
//     }

//     void readEncoder(serial::Serial &serial)
//     {
//         MsgUint32_t pos_x;
//         MsgUint32_t pos_y;
//         MsgUint32_t pos_z;

//         auto flag = serial.waitReadable();
//         if (!flag) {  // Check for message received
//             std::cout << "Timeout - No message received \n";
//             return;
//         }
//         [[maybe_unused]] size_t length = serial.read(buffer.data(), 10); // read to buffer
//         serial.flushInput();

//         // if (buffer[0] == header) {
//         //     int num_var = buffer[1]/4;
//         //     for (int i=0; i<buffer[1]/num_var;i++) {
//         //         pos_x.bits[i] = buffer[i+2];
//         //     }
//         //     for (int i=0; i<buffer[1]/num_var;i++) {
//         //         pos_y.bits[i] = buffer[i+6];
//         //     }
//         //     std::cout << "pos_x: " << pos_x.value << " pos_y: " << pos_y.value << "\n" ;
//         // } else {
//         //     std::cout <<" Header is wrong... value:" << header << "\n";
//         // }

//         // Read first 4 bytes (integer)
//         pos_x.bits[0] = buffer[2];
//         pos_x.bits[1] = buffer[3];
//         pos_x.bits[2] = buffer[4];
//         pos_x.bits[3] = buffer[5];

//         // Publish 
//         geometry_msgs::Vector3 encoder_msg;
//         encoder_msg.x = pos_x.value;
//         encoder_pub.publish(encoder_msg);
//     }
// };

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_arduino_encoder_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(100.0);
    RosArduinoEncoderNode arduinoEncoder(nh);
    while (ros::ok())
    {
        ros::spinOnce();
        arduinoEncoder.Update();
        rate.sleep();
    }
    return 0;
}
