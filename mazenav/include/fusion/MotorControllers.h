#pragma once

#include "fusion/teensyCommunicator.h"
#include "fusion/mutexes.h"

class MotorControllers
{
    public:
        MotorControllers(TeensyCommunicator* communicator);

        class MotorSpeeds
        {
            public:
            MotorSpeeds();
            MotorSpeeds(int rf, int lf, int rb, int lb);

            int rf {0};
            int lf {0};
            int rb {0};
            int lb {0};

            void toRadians();
            void toRpm();
        };

        struct Distances
        {
            int rf {0};
            int lf {0};
            int rb {0};
            int lb {0};
        };

        enum MotorID
        {
            motor_rf,
            motor_lf,
            motor_rb,
            motor_lb,
            motor_num,
        };

        // Sets the latest control speeds and gets new data from teensy
        void updateVals();

        // Variables to store the latest values
        MotorSpeeds setSpeeds {};
        MotorSpeeds readSpeeds {};
        Distances readDistances {};

        void printSpeeds();
        void printDistances();

    private:
        TeensyCommunicator* communicator;
        int16_t distances[motor_num] {};
        int16_t speeds[motor_num] {};

};