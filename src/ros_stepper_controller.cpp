#include "ros_stepper_controller.hpp"

RosStepperController::RosStepperController(ros::NodeHandle& nh)
{
    // subscribers
    stepperLenVelSub = nh.subscribe("/stepper/setpoint", 1, &RosStepperController::RecieveStepperSetpointCb, this);
    currentLenSub = nh.subscribe("/encoder/position_payload", 1, &RosStepperController::RecieveCurrentLenCb, this);

    // publisher to serial node
    stepperStepsPub = nh.advertise<std_msgs::Float32>("/stepper/serial_command", 1);
    nh.param("/kp", kp, 0.0f);
    nh.param("/ki", ki, 0.0f);
    nh.param("/kd", kd, 0.0f);
}

void RosStepperController::RecieveStepperSetpointCb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    targetLen = msg->data[0];
    maxVel = msg->data[1];
}

void RosStepperController::RecieveCurrentLenCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    currentLen = msg->vector.z;
}

void RosStepperController::SendStepperCommand(double msg)
{
    stepperSerialMsg.data = msg;
    stepperStepsPub.publish(stepperSerialMsg);
}

void RosStepperController::PIDControl(void)
{
    // PID control with derivative jerk prevention (feed derivative into feedback)
    double error = targetLen - currentLen;
    integral += error;
    double dError = currentLen - lastLen;

    output = kp * error + ki * integral - kd * dError; // technically PD control (set Ki to 0 pls)
    // preclamped output
    if (output > maxVel) {
         output = maxVel;
    } else if (output < -maxVel) {
         output = -maxVel;
    }
    
    lastLen = currentLen;
    SendStepperCommand(output);
}

