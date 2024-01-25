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
    int16_t values[] {};
    values[motor_rf] = readSpeeds.rf;
    values[motor_lf] = readSpeeds.lf;
    values[motor_rb] = readSpeeds.rb;
    values[motor_lb] = readSpeeds.lb;

    mtx_transData_controlData.lock();
    communicator->transData.setRpmControl(values);
    mtx_transData_controlData.unlock();
    mtx_transData_freqData.lock();
    communicator->transData.getRPM(speeds);
    communicator->transData.getPos(distances);
    mtx_transData_freqData.unlock();

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