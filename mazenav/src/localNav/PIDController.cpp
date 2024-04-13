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
    double error {setpoint-value}; // Negative when above, positive when below
    return getCorrectionFromError(error);
}

double PIDController::getCorrectionFromError(double error)
{
    std::chrono::steady_clock::time_point curTime {std::chrono::steady_clock::now()};
    double loopTime {std::chrono::duration_cast<std::chrono::milliseconds>(curTime-lastTime).count()/1000.0};

    // If not enough time has passed
    if (loopTime < MIN_DURATION)
    {
        return lastCorr;
    }
    // Otherwise continue

    // Compute terms
    integralSum += error*loopTime;
    double derivative {(error-lastError)/loopTime};
    // Catch the case where reset has been done or first time, where we do not know the lastError (thus it is set to 0)
    if (lastError==0)
    {
        derivative = 0;
    }

    // Compute correction
    double correction {(kP*error + kI*integralSum + kD*derivative)};

    // Update variables for next iteration
    lastTime = curTime;
    lastCorr = correction;
    lastError = error;


    return correction;
}


void PIDController::restartPID()
{
    integralSum = 0;
    lastTime = std::chrono::steady_clock::now();
    lastCorr = 0;
    lastError = 0;
}