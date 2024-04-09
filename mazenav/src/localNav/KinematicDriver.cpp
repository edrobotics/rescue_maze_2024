#include "localNav/KinematicDriver.h"

KinematicDriver::KinematicDriver(communication::Communicator* globComm)
{
    this->globComm = globComm;
}

void KinematicDriver::calcSpeeds(int tSpeed, int rSpeed)
{
    // First, modify the tSpeed and rSpeed according to PID algorithm
    // Set the PID setpoints
    tPID.setSetpoint(tSpeed);
    rPID.setSetpoint(rSpeed);
    
    // Get the corrections and apply them
    #warning have to get actual speeds. From where?
    tSpeed += tPID.getCorrection(0);
    rSpeed += rPID.getCorrection(0);

    // Then do the actual motor speed calculations. The updated PID values "trick" the system

    // Base driving speed
    double baseSpeed {tSpeed};
    double baseWheelspeedContribution {baseSpeed/WHEEL_RADIUS};

    // Contribution of rotation
    double rotationSpeedAtWheel {WHEEL_TURN_CONTRIBUTION_FACTOR*(static_cast<double>(rSpeed)/WHEEL_TURN_RADIUS)};
    double rotationWheelspeedContribution {rotationSpeedAtWheel/WHEEL_RADIUS};

    // Computation of wheel rotation speeds
    double leftSpeed {baseWheelspeedContribution - rotationWheelspeedContribution};
    double rightSpeed {baseWheelspeedContribution + rotationWheelspeedContribution};

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

void KinematicDriver::stop()
{
    motorSpeeds.lf = 0;
    motorSpeeds.lb = 0;
    motorSpeeds.rf = 0;
    motorSpeeds.rb = 0;
    setSpeeds();
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