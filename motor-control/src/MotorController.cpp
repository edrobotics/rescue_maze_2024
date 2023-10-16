#include "MotorController.h"


MotorController::MotorController(int enA, int enB, int pwmPin, int dirPin, double encRatio, int cpr) : encoder(enA, enB)
{
    this->pwmPin = pwmPin;
    this->dirPin = dirPin;
    this->encOutputRatio = encRatio;
    this->cpr = cpr;
}

bool MotorController::init()
{

    // PID Setup
    pid.SetMode(QuickPID::Control::automatic);
    pid.SetOutputLimits(-255, 255); // Limits for analogWrite(), positive and negative
    #warning untuned sample time
    pid.SetSampleTimeUs(1000); // 1ms sample time (?) Probably too large?
    #warning PID setup not done!


    // Nothing went wrong, so we return true (otherwise we returned false earlier)
    return true;
}

void MotorController::setSpeed(double speed)
{
    outputSpeed = speed;
    motorSpeed = outputSpeed*encOutputRatio;
}

void MotorController::setPWM(int pwmSpeed)
{
    if (pwmSpeed>255)
    {
        pwmSpeed = 255;
    }
    else if (pwmSpeed<-255)
    {
        pwmSpeed = 255;
    }

    #warning unsure if high or low means forwards or backwards
    if (pwmSpeed>0)
    {
        digitalWrite(dirPin, (!isReversed) ? HIGH : LOW);
    }
    else
    {
        digitalWrite(dirPin, (isReversed) ? HIGH : LOW);
    }
    analogWrite(pwmPin, abs(pwmSpeed));
}

void MotorController::update()
{
    long updateTime = millis();
    static long lastUpdateTime = 0;
    static long lastEncoderPos = 0;

    int read = encoder.read();
    encoderPos = (isReversed) ? -read : read;

    // Calculate speeds
    #warning possible overflows and truncations ahead
    if (updateTime==lastUpdateTime) return; // Would result in division by 0
    curMotorSpeed = double(encoderPos-lastEncoderPos)/double(cpr) / double(updateTime-lastUpdateTime) * double(MILLIS_PER_MINUTE);
    // curMotorOutSpeed = curMotorSpeed/encOutputRatio;

    // PID Loop calculation
    // Updates pwmCorrection
    pid.Compute();
    
    // Speed correction by PWM
    setPWM(pwmCorrection);

    // Set variables for next time
    lastEncoderPos = encoderPos;
    lastUpdateTime = updateTime;
}

void MotorController::startDistanceMeasure()
{
    tickStartDistance = encoderPos;
}


double MotorController::getDistance()
{
    return (encoderPos-tickStartDistance)/encOutputRatio;
}

double MotorController::getOtputSpeed()
{
    return curMotorSpeed/encOutputRatio;
}