#include "fusion/MotorControllers.h"

MotorControllers::MotorControllers(TeensyCommunicator* communicator)
{
    mtx_general.lock();
    this->communicator = communicator;
    mtx_general.unlock();
}

MotorControllers::MotorSpeeds::MotorSpeeds(int rf, int lf, int rb, int lb)
{
    this->rf = rf;
    this->lf = lf;
    this->rb = rb;
    this->lb = lb;
}

MotorControllers::MotorSpeeds::MotorSpeeds()
{
    MotorSpeeds {0, 0, 0, 0};
}

double MotorControllers::MotorSpeeds::toRadians(int rpm)
{
    return static_cast<double>(rpm)*M_PI/30;
}

int MotorControllers::MotorSpeeds::toRpm(double radians)
{
    return static_cast<int>(radians*30/M_PI);
}

// bool MotorControllers::updateVals()
// {
//     setVals();
//     return getVals();
// }

void MotorControllers::setVals()
{
    int16_t values[motor_num] {};
    mtx_speedSetter.lock();
    values[motor_rf] = controlSpeeds.rf;
    values[motor_lf] = controlSpeeds.lf;
    values[motor_rb] = controlSpeeds.rb;
    values[motor_lb] = controlSpeeds.lb;
    mtx_speedSetter.unlock();

    communicator->transData.tsSetRpmControl(values);
}

bool MotorControllers::getVals()
{


    if (!speedUpdated)
    {
        // Speeds
        mtx_speedGetter.lock();

        speedUpdated = communicator->transData.tsGetRPM(speeds);

        motorSpeeds.rf = speeds[motor_rf];
        motorSpeeds.lf = speeds[motor_lf];
        motorSpeeds.rb = speeds[motor_rb];
        motorSpeeds.lb = speeds[motor_lb];

        mtx_speedGetter.unlock();
    }

    if (!posUpdated)
    {
        // Distances
        mtx_distanceGetter.lock();

        posUpdated = communicator->transData.tsGetPos(distances);

        motorDistances.rf = distances[motor_rf];
        motorDistances.lf = distances[motor_lf];
        motorDistances.rb = distances[motor_rb];
        motorDistances.lb = distances[motor_lb];

        // if (posUpdated)
        // {
        //     std::cout << "motorDistances: lf=" << motorDistances.lf << "  ";
        //     std::cout << "lb=" << motorDistances.lb << "  ";
        //     std::cout << "rf=" << motorDistances.rf << "  ";
        //     std::cout << "rb=" << motorDistances.rb << "\n";
        // }
        
        mtx_distanceGetter.unlock();
    }

    if (speedUpdated && posUpdated)
    {
        // Reset variables for next use and return that both were updated
        speedUpdated = false;
        posUpdated = false;
        return true;
    }
    else
    {
        return false;
    }

}


void MotorControllers::printSpeeds()
{
    mtx_speedGetter.lock();
    std::cout << "rf=" << motorSpeeds.rb << "  ";
    std::cout << "lf=" << motorSpeeds.lf << "  ";
    std::cout << "rb=" << motorSpeeds.rb << "  ";
    std::cout << "lb=" << motorSpeeds.lb << "  ";
    std::cout << "\n";
    mtx_speedGetter.unlock();
}


// MotorControllers::MotorSpeeds MotorControllers::getSpeeds()
// {
//     return motorSpeeds;
// }

// MotorControllers::Distances MotorControllers::getDistances()
// {
//     return motorDistances;
// }

void MotorControllers::setSpeeds(MotorSpeeds speeds)
{
    mtx_speedSetter.lock();
    this->controlSpeeds = speeds;
    mtx_speedSetter.unlock();
}

MotorControllers::Distances MotorControllers::getDistances()
{
    std::lock_guard<std::mutex> lock(mtx_distanceGetter);
    return motorDistances;
}

MotorControllers::MotorSpeeds MotorControllers::getSpeeds()
{
    std::lock_guard<std::mutex> lock(mtx_speedGetter);
    return motorSpeeds;
}