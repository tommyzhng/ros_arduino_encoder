#include <ros/ros.h>
#include "ros_arduino_encoder.hpp"
#include "ros_stepper_controller.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_arduino_encoder_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(60);
    RosArduinoEncoderNode arduinoEncoder(nh);
    ros::NodeHandle nh2("~");
    //RosStepperController stepperController(nh2);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Wait for Arduino to reset
    while (ros::ok())
    {
        ros::spinOnce();
        arduinoEncoder.Update();
        //stepperController.Update();
        rate.sleep();
    }
    return 0;
}
