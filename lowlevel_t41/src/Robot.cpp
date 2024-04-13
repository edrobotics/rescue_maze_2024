#include "Robot.h"


Robot::Robot()
{

}

bool Robot::init()
{
    int somethingFailed {0};
    communicator.init();
    for (int i=0;i<MOTOR_NUM;++i)
    {
        if (!motorControllers[i]->init())
        {
            ++somethingFailed;
        }
    }
    motorRF.isReversed = true;
    motorRB.isReversed = true;

    if (!imu.init())
    {
        Serial.println("Failed to init IMUs");
        ++somethingFailed;
    }
    if (tof.init()>0)
    {
        Serial.println("Failed to init ToF");
        ++somethingFailed;
    }

    delay(1000);
    if (somethingFailed>0)
    {
        return false;
    }
    return true;
}


void Robot::updateLoop()
{
    // Wait until control data is written
    while(!communicator.checkWrite())
    {
        updateMotors();
        delay(1);
    }
    // Update the settings once new data is received (also use this data for motors)
    communicator.updateSettings();
    setMotors();

    // Update all sensors while also updating the motors every so often
        updateMotors();
    updateBatVol();
        updateMotors();
    updateImu();
        updateMotors();
    updateTof();
        updateMotors();
    getMotors();
        updateMotors();

    // Set the newly recorded data as accessible to the pi
    communicator.updateByteArray();
}


void Robot::setMotors()
{
    int16_t rpmVals[MOTOR_NUM] {0};
    communicator.transData.getRpmControl(rpmVals);
    for (int i=0;i<MOTOR_NUM;++i)
    {
        motorControllers[i]->setSpeed(rpmVals[i]);
    }
}

void Robot::getMotors()
{
    int16_t speeds[MOTOR_NUM];
    int16_t positions[MOTOR_NUM];
    for (int i=0;i<MOTOR_NUM;++i)
    {
        speeds[i] = motorControllers[i]->getOutputSpeed();
        positions[i] = motorControllers[i]->getDistance();
    }
    communicator.transData.setRPM(speeds);
    communicator.transData.setPos(positions);
}

void Robot::updateMotors()
{
    for (int i=0;i<MOTOR_NUM;++i)
    {
        motorControllers[i]->update();
    }
}

void Robot::updateBatVol()
{
    communicator.transData.setVoltage(batMeas.getVoltage());
}

void Robot::updateImu()
{
    imu.runLoop();

    communicator.transData.setIMU(0, imu.rotationVector.floats);
}

void Robot::updateTof()
{
    while (!tof.update())
    {
        delayMicroseconds(100);
    }

    communicator.transData.setTof(tof.distances);

}

void Robot::updateCol()
{
    uint16_t tempColVal[] {0, 0, 0, 0};

    communicator.transData.setCol(0, tempColVal);
}

// void Robot::testDrive()
// {
//     for (int i=0;i<MOTOR_NUM;++i)
//     {
//         motorControllers[i]->setSpeed(90);
//     }

//     long timeflag {millis()};
//     while (millis()-timeflag < 600)
//     {
//         updateMotors();
//     }

//     for (int i=0;i<MOTOR_NUM;++i)
//     {
//         motorControllers[i]->setPWM(0);
//     }
// }


void Robot::calibrateMotorPid()
{
    Serial.println("Calibrating motorRF:");
    motorRF.pwmRPMCalibration();
    Serial.println("Calibrating motorLF:");
    motorLF.pwmRPMCalibration();
    Serial.println("Calibrating motorRB:");
    motorRB.pwmRPMCalibration();
    Serial.println("Calibrating motorLB:");
    motorLB.pwmRPMCalibration();
}