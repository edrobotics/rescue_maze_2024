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

void MotorControllers::updateVals()
{
    setVals();
    getVals();
}

void MotorControllers::setVals()
{
    int16_t values[motor_num] {};
    values[motor_rf] = setSpeeds.rf;
    values[motor_lf] = setSpeeds.lf;
    values[motor_rb] = setSpeeds.rb;
    values[motor_lb] = setSpeeds.lb;

    communicator->transData.tsSetRpmControl(values);
}

void MotorControllers::getVals()
{
    communicator->transData.tsGetRPM(speeds);
    communicator->transData.tsGetPos(distances);

    readSpeeds.rf = speeds[motor_rf];
    readSpeeds.lf = speeds[motor_lf];
    readSpeeds.rb = speeds[motor_rb];
    readSpeeds.lb = speeds[motor_lb];

    readDistances.rf = distances[motor_rf];
    readDistances.lf = distances[motor_lf];
    readDistances.rb = distances[motor_rb];
    readDistances.lb = distances[motor_lb];
}


void MotorControllers::printSpeeds()
{
    std::cout << "rf=" << readSpeeds.rb << "  ";
    std::cout << "lf=" << readSpeeds.lf << "  ";
    std::cout << "rb=" << readSpeeds.rb << "  ";
    std::cout << "lb=" << readSpeeds.lb << "  ";
    std::cout << "\n";
}