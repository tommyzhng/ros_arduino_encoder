/*
    * ros_stepper_controller.hpp
    * This file contains a class that converts GUI ROS commands to a stepper motor step ROS Topic.
    * Also includes a sine wave testing function
*/
#ifndef ROS_ARDUINO_ENCODER_ROS_STEPPER_CONTROLLER_HPP
#define ROS_ARDUINO_ENCODER_ROS_STEPPER_CONTROLLER_HPP

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3Stamped.h>

class RosStepperController
{
public:
    RosStepperController(ros::NodeHandle& nh);
    ~RosStepperController() = default;
    void PIDControl(void);
private:
    void RecieveStepperSetpointCb(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void RecieveCurrentLenCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
    void SendStepperCommand(double msg);
    void SineWaveTest(void);

    // ros subs
    ros::Subscriber stepperLenVelSub;
    ros::Subscriber currentLenSub;
    // ros pubs
    ros::Publisher stepperStepsPub;

    std_msgs::Float32 stepperSerialMsg;
    std_msgs::Float32MultiArray sineMsg;

    bool sineTest{false};
    float currentLen{0};
    float targetLen{0};
    float maxVel{0};
    double output;

    // PID params
    float ki{0.0f};
    float kp{0.0f};
    float kd{0.0f};
    double integral{0};
    double lastLen{0};
};

#endif