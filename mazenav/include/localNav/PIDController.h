#pragma once

#include <chrono>

class PIDController
{
    public:

        PIDController(double kP, double kI, double kD);

        void setCoeff(double kP, double kI, double kD);
        void setkP(double kP);
        void setkI(double kI);
        void setkD(double kD);

        // Set the setpoint
        void setSetpoint(double setpoint);
        // Set the actual value

        // Gets the correction for the given value (computes the correction)
        // If not enough time has passed, the last known correction is returned
        double getCorrection(double value);

        double getCorrectionFromError(double error);

        // Restarts the PID controller (removes time dependencies, keeps coefficients and setpoint). Useful if you have a pause in operation and want the controller to be fresh when starting up again.
        void restartPID();

    private:
        double kP {0};
        double kI {0};
        double kD {0};

        double setpoint {0};
        double lastCorr {0};

        // Minimum time between loops in milliseconds
        int minDurationMillis {10};

        std::chrono::steady_clock::time_point lastTime {std::chrono::steady_clock::now()};
        double integralSum {};
};