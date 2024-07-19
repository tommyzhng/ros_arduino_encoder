#include <ros/ros.h>
#include "ros_arduino_encoder.hpp"
#include "ros_stepper_controller.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_arduino_encoder_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(100);
    RosArduinoEncoderNode arduinoEncoder(nh);
    RosStepperController stepperController(nh);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Wait for Arduino to reset
    while (ros::ok())
    {
        ros::spinOnce();
        arduinoEncoder.Update();
        stepperController.PIDControl();
        rate.sleep();
    }
    return 0;
}
