#pragma once

#include <mutex>
#include <iostream>
// #include <chrono>

#include "fusion/MotorControllers.h"


namespace communication
{
    class MotorControllerCommunicator
    {
        public:
            // Sets the speeds that should be set to the motors
            void setSpeeds(MotorControllers::MotorSpeeds speeds);

            // Reads the speeds that should be set to the motors
            MotorControllers::MotorSpeeds getSpeeds();

        private:
            MotorControllers::MotorSpeeds speeds {};
            std::mutex mtx_speeds;

    };
}