#include "ros_stepper_controller.hpp"

RosStepperController::RosStepperController(ros::NodeHandle& nh)
{
    // subscribers
    stepperLenVelSub = nh.subscribe("/stepper/setpoint", 1, &RosStepperController::RecieveStepperSetpointCb, this);
    currentLenSub = nh.subscribe("/encoder/position_payload", 1, &RosStepperController::RecieveCurrentLenCb, this);

    // publisher to serial node
    stepperStepsPub = nh.advertise<std_msgs::Float32MultiArray>("/stepper/serial_command", 1);
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

void RosStepperController::SendStepperCommand(double len, double vel)
{
    stepperSerialMsg.data = std::vector<float>{static_cast<float>(len), static_cast<float>(vel)};
    stepperStepsPub.publish(stepperSerialMsg);
}

void RosStepperController::PIDControl(void)
{
    // PID control with derivative jerk prevention (feed derivative into feedback)
    double error = targetLen - currentLen;
    integral += error;
    double dError = error - lastError;

    // force no overshoot
    if (abs(error) < 0.01) {
        SendStepperCommand(targetLen, 0);
        return;
    }

    output = kp * error + ki * integral + kd * dError; // technically PD control (set Ki to 0 pls)
    // preclamped output
    if (output > maxVel) {
         output = maxVel;
    } else if (output < -maxVel) {
         output = -maxVel;
    }

    
    lastError = error;
    SendStepperCommand(targetLen, -output);
}

