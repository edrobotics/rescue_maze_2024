#pragma once
#include "communicator/communicator.h"
#include "fusion/MotorControllers.h"
#include "GlobalConstants.h"
// #include "localNav/PIDController.h"

#include <chrono>
#include <thread>

// Input translational and rotational speed, and then it sends motor output values to hardware interface.
// Note: This is only a theoretical calculation. A separate control loop should be used in conjunction with this.

class KinematicDriver
{
    public:
        KinematicDriver(communication::Communicator* globComm);

        // Set the speeds to the hardware interface
        void setSpeeds();

        // Calculate motor speeds from robot translational and rotational speed.
        // tSpeed - translational speed, mm/s
        // rSpeed - rotational speed, rad/s
        void calcSpeeds(double tSpeed, double rSpeed);

        // Stops the robot
        void stop();

        // Test communication
        void testComm();


    private:
        communication::Communicator* globComm {};
        MotorControllers::MotorSpeeds motorSpeeds {};

};