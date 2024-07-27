#include "ros_stepper_controller.hpp"

RosStepperController::RosStepperController(ros::NodeHandle& nh)
{
    // subscribers
    stepperLenVelSub = nh.subscribe("/stepper/setpoint", 1, &RosStepperController::RecieveStepperSetpointCb, this);
    currentLenSub = nh.subscribe("/encoder/position_payload", 1, &RosStepperController::RecieveCurrentLenCb, this);
    stepperTestSub = nh.subscribe("/stepper/test", 1, &RosStepperController::RecieveStepperTestCb, this);

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

void RosStepperController::RecieveStepperTestCb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data[0] == 1) {
        sineTest = true;
    } else {
        sineTest = false;
    }
    if (sineTest) {
        SineWaveTest();
    }
}

void RosStepperController::SendStepperCommand(double len, double vel)
{
    stepperSerialMsg.data = std::vector<float>{static_cast<float>(len), static_cast<float>(vel)};
    stepperStepsPub.publish(stepperSerialMsg);
}

void RosStepperController::PIDControl(void)
{
    // standard pd controller (bcs if u send len to stepper it blocks any further inputs)
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

void RosStepperController::SineWaveTest(void)
{
    // sine wave test non blocking by sending vel
    double len = 0.0;
    double vel = 0.0;
    double time = ros::Time::now().toSec();
    double dt = 0.0;
    while (sineTest) {
        dt = ros::Time::now().toSec() - time;
        len = 0.1 * sin(2 * M_PI * 0.1 * dt);
        vel = 0.1 * 2 * M_PI * 0.1 * cos(2 * M_PI * 0.1 * dt);
        SendStepperCommand(len, vel);
        ros::spinOnce();
    }

}

