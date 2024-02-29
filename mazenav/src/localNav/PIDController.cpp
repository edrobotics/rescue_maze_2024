#include "localNav/PIDController.h"


PIDController::PIDController(double kP, double kI, double kD)
{
    setCoeff(kP, kI, kD);
}

void PIDController::setCoeff(double kP, double kI, double kD)
{
    setkP(kP);
    setkI(kI);
    setkD(kD);
}

void PIDController::setkP(double kP)
{
    this->kP = kP;
}

void PIDController::setkI(double kI)
{
    this->kI = kI;
}

void PIDController::setkD(double kD)
{
    this->kD = kD;
}


void PIDController::setSetpoint(double setpoint)
{
    this->setpoint = setpoint;
}

double PIDController::getCorrection(double value)
{
    std::chrono::steady_clock::time_point curTime {std::chrono::steady_clock::now()};
    long int loopTime {std::chrono::duration_cast<std::chrono::milliseconds>(curTime-lastTime).count()};

    // If not enough time has passed
    if (loopTime < minDurationMillis)
    {
        return lastCorr;
    }
    // Otherwise continue

    // Compute terms
    double error {setpoint-value}; // Negative when above, positive when below
    integralSum += error*static_cast<double>(loopTime)/1000.0;
    double derivative {error/(static_cast<double>(loopTime)/1000.0)};

    // Compute correction
    double correction {(kI*error + kI*integralSum + kD*derivative)};

    // Update variables for next iteration
    lastTime = curTime;
    lastCorr = correction;
    
    return correction;
}