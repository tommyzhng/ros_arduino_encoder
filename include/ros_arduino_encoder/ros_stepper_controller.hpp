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
#include <chrono>
#include <cmath>

class RosStepperController
{
public:
    RosStepperController(ros::NodeHandle& nh);
    ~RosStepperController() = default;
    void Update(void);
private:
    void RecieveStepperSetpointCb(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void RecieveCurrentLenCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
    void RecieveStepperTestCb(const std_msgs::Float32MultiArray::ConstPtr& msg);
    bool PIDControl(double targetLen, double maxVel); // this PD controller returns true if target is reached, prevents overshoot
    void SineWaveTest(void);
    // pubs
    void SendStepperCommand(double len, double vel);

    // ros subs
    ros::Subscriber stepperLenVelSub;
    ros::Subscriber currentLenSub;
    ros::Subscriber stepperTestSub;
    // ros pubs
    ros::Publisher stepperStepsPub;

    std_msgs::Float32MultiArray stepperSerialMsg;
    std_msgs::Float32MultiArray sineMsg;


    
    // PID params
    float ki{0.0f};
    float kp{0.0f};
    float kd{0.0f};
    double integral{0};
    double lastError{0};
    float currentLen{0};
    float targetLen_{0};
    float maxVel_{0};

    double output;

    // sine wave testing
    bool sineTest{false};
    float amplitude{0};
    float frequency{0};
    double startTime{0};
    bool reachedOffset{false};
};

#endif