#include "localNav/KinematicDriver.h"

KinematicDriver::KinematicDriver(communication::Communicator* globComm)
{
    this->globComm = globComm;
}

void KinematicDriver::calcSpeeds(int tSpeed, int rSpeed)
{
    // Base driving speed
    double baseSpeed {tSpeed};
    double baseWheelspeedContribution {baseSpeed/WHEEL_RADIUS};

    // Contribution of rotation
    double rotationSpeedAtWheel {WHEEL_TURN_CONTRIBUTION_FACTOR*(static_cast<double>(rSpeed)/WHEEL_TURN_RADIUS)};
    double rotationWheelspeedContribution {rotationSpeedAtWheel/WHEEL_RADIUS};

    // Computation of wheel rotation speeds
    double leftSpeed {baseWheelspeedContribution - rotationSpeedAtWheel};
    double rightSpeed {baseWheelspeedContribution - rotationSpeedAtWheel};

    // Convert from rad/s to rpm
    int leftSpeedrpm = MotorControllers::MotorSpeeds::toRpm(leftSpeed);
    int rightSpeedrpm = MotorControllers::MotorSpeeds::toRpm(rightSpeed);

    // Set the speeds to the motorspeeds variable
    motorSpeeds.lf = leftSpeedrpm;
    motorSpeeds.lb = leftSpeedrpm;
    motorSpeeds.rf = rightSpeedrpm;
    motorSpeeds.rb = rightSpeedrpm;

}

void KinematicDriver::setSpeeds()
{
    globComm->motors.setSpeeds(motorSpeeds);
}

void KinematicDriver::testComm()
{
    MotorControllers::MotorSpeeds speeds {};
    motorSpeeds = speeds;
    setSpeeds();
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    speeds.rf = 100;
    speeds.lf = 100;
    speeds.rb = 100;
    speeds.lb = 100;
    motorSpeeds = speeds;
    setSpeeds();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}