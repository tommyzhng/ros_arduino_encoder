#include "ros_stepper_controller.hpp"


RosStepperController::RosStepperController(ros::NodeHandle& nh) : sineTest(false)
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
    targetLen_ = msg->data[0];
    maxVel_ = msg->data[1];
}

void RosStepperController::RecieveCurrentLenCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    currentLen = msg->vector.z;
}

void RosStepperController::RecieveStepperTestCb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data[0] == 1) {
        sineTest = true;
        reachedOffset = false; // Reset the flag
        amplitude = msg->data[1];
        frequency = msg->data[2];
        startTime = ros::Time::now().toSec(); // Initialize start time for sine wave test
    } else {
        sineTest = false;
        reachedOffset = false;
    }
}

void RosStepperController::SendStepperCommand(double len, double vel)
{
    stepperSerialMsg.data = std::vector<float>{static_cast<float>(len), static_cast<float>(vel)};
    stepperStepsPub.publish(stepperSerialMsg);
}

bool RosStepperController::PIDControl(double targetLen, double maxVel) 
{
    // standard PD controller (because if you send len to stepper it blocks any further inputs)
    double error = targetLen - currentLen;
    integral += error;
    double dError = error - lastError;

    // force no overshoot
    if (abs(error) < 0.01) {
        SendStepperCommand(targetLen, 0);
        return true;
    }

    output = kp * error + ki * integral + kd * dError; // technically PD control (set Ki to 0 please)
    // preclamped output
    if (output > maxVel) {
         output = maxVel;
    } else if (output < -maxVel) {
         output = -maxVel;
    }

    lastError = error;
    SendStepperCommand(targetLen, -output);
    return false;
}

void RosStepperController::SineWaveTest(void)
{
    if (!sineTest) return;

    double offset = amplitude * 1.1; // a bit more than amplitude to avoid hitting the frame

    if (!reachedOffset) {               // move to the starting offset position
        reachedOffset = PIDControl(offset, 0.1);
    } else {                            // perform sine wave oscillation
        double currentTime = ros::Time::now().toSec();
        double elapsedTime = currentTime - startTime;
        double sineValue = offset + amplitude * std::sin(2 * M_PI * frequency * elapsedTime);
        double velocity = 2 * M_PI * frequency * amplitude * std::cos(2 * M_PI * frequency * elapsedTime);
        SendStepperCommand(sineValue, velocity);
    }
}

void RosStepperController::Update()
{
    if (sineTest) {
        SineWaveTest();
    } else {
        PIDControl(targetLen_, maxVel_);
    }
}
