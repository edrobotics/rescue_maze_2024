#pragma once

#include "fusion/TeensyCommunicator.h"
// #include "fusion/mutexes.h"
#include <mutex>
#include <cmath>

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

            static double toRadians(int rpm);
            static int toRpm(double radians);
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

        // // Sets the latest control speeds and gets new data from teensycommunicator
        // bool updateVals();
        // Sets the latest control speeds to teensycommunicator
        void setVals();
        // Gets the latest data from teensycommunicator
        bool getVals();

        void setSpeeds(MotorSpeeds speeds);
        // MotorSpeeds getSpeeds();
        // Distances getDistances();

        Distances getDistances();
        MotorSpeeds getSpeeds();


        void printSpeeds();
        void printDistances();

    private:
        // Variables to store the latest values
        MotorSpeeds controlSpeeds {};
        MotorSpeeds motorSpeeds {};
        Distances motorDistances {};
    
        // For TS access control
        std::mutex mtx_general {};
        std::mutex mtx_speedSetter;
        std::mutex mtx_speedGetter;
        std::mutex mtx_distanceGetter;

        TeensyCommunicator* communicator;
        int16_t distances[motor_num] {};
        int16_t speeds[motor_num] {};


        // Whether or not values were updated
        bool setUpdated {false};
        bool speedUpdated {false};
        bool posUpdated {false};



};