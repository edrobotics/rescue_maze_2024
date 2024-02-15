#include "fusion/MotorControllers.h"

MotorControllers::MotorControllers(TeensyCommunicator* communicator)
{
    this->communicator = communicator;
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

void MotorControllers::updateVals()
{
    setVals();
    getVals();
}

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

void MotorControllers::getVals()
{
    // Speeds
    mtx_speedGetter.lock();

    communicator->transData.tsGetRPM(speeds);

    motorSpeeds.rf = speeds[motor_rf];
    motorSpeeds.lf = speeds[motor_lf];
    motorSpeeds.rb = speeds[motor_rb];
    motorSpeeds.lb = speeds[motor_lb];

    mtx_speedGetter.unlock();

    // Distances
    mtx_distanceGetter.lock();

    communicator->transData.tsGetPos(distances);

    motorDistances.rf = distances[motor_rf];
    motorDistances.lf = distances[motor_lf];
    motorDistances.rb = distances[motor_rb];
    motorDistances.lb = distances[motor_lb];
    
    mtx_distanceGetter.unlock();
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


MotorControllers::MotorSpeeds MotorControllers::getSpeeds()
{
    return motorSpeeds;
}

MotorControllers::Distances MotorControllers::getDistances()
{
    return motorDistances;
}

void MotorControllers::setSpeeds(MotorSpeeds speeds)
{
    this->controlSpeeds = speeds;
}