#pragma once
#include <Arduino.h>
#include "MotorController.h"
#include "Communicator.h"
#include "Imu.h"
#include "TofCollection.h"
#include <DFRobot_MCP23017.h>
#include "BatMeas.h"

class Robot
{
    public:
        Robot();

        bool init();

        void updateLoop();
        // void testDrive();

        void calibrateMotorPid();

        // Should be private but there was an issue (flexible member array though it is not)
        static constexpr int MOTOR_NUM = 4;
        MotorController* motorControllers[MOTOR_NUM] {&motorRF, &motorLF, &motorRB, &motorLB};
    private:
        // Misc.
        Communicator communicator {1};
        DFRobot_MCP23017 ioExpander {};

        // Motors
        //                     ENCA, ENCB, PWM, DIR, ENCRATIO, CPR
        MotorController motorRF {3,     2,   7,   6,    46.85,  48};
        MotorController motorLF {39,   38,  28,  29,    46.85,  48};
        MotorController motorRB {1,     0,   5,   4,    46.85,  48};
        MotorController motorLB {23,   22,   37, 36,    46.85,  48};

        // Sensors
        BatMeas batMeas {20, 10000, 2200};
        Imu imu {};
        TofCollection tof {&ioExpander};

        // Refresh values
        // Get from sensors and put in communicator
        void updateBatVol();
        void updateImu();
        void updateTof();
        void updateCol();
        void getMotors();
        // Get from communicator and set to sensors/components
        void setMotors();

        // Run the motor PID loop
        void updateMotors();

};