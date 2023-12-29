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
    pid.SetControllerDirection(QuickPID::Action::direct);
    pid.SetOutputLimits(-255, 255); // Limits for setPwm(), positive and negative. Should be narrower?
    pid.SetSampleTimeUs(pidSampleTimeUs);

    pinMode(dirPin, OUTPUT);
    pinMode(pwmPin, OUTPUT);
    analogWriteFrequency(pwmPin, 80000); // Sets the frequency higher to reduce noise and make more like real analogue voltage?

    // Nothing went wrong, so we return true (otherwise we returned false earlier)
    return true;
}

void MotorController::setSpeed(double speed)
{
    motorSpeed = speed*encOutputRatio;
}

void MotorController::setPWM(int pwmSpeed)
{
    if (pwmSpeed>255)
    {
        pwmSpeed = 255;
    }
    else if (pwmSpeed<-255)
    {
        pwmSpeed = -255;
    }

    if (pwmSpeed>0)
    {
        digitalWrite(dirPin, (!isReversed) ? LOW : HIGH);
    }
    else
    {
        digitalWrite(dirPin, (isReversed) ? LOW : HIGH);
    }
    analogWrite(pwmPin, abs(pwmSpeed));

    curPWM = pwmSpeed;
}

bool MotorController::update()
{
    return update(true);
}

bool MotorController::update(bool usePIDCorr)
{
    unsigned long updateTime = micros();
    
    // Check that enough time has passed, so that we do not calculate stuff unnecessarily
    if (updateTime-lastUpdateTime < pidSampleTimeUs+300)
    {
        return false;
    }

    long read = encoder.read();
    encoderPos = (isReversed) ? -read : read;


    // Calculate speeds
    if (updateTime==lastUpdateTime) return false; // Would result in division by 0
    curMotorSpeed = double(encoderPos-lastEncoderPos)/double(cpr*(updateTime-lastUpdateTime)) * double(MICROS_PER_MINUTE);
    // Check for unreasonable values?

    // PID Loop calculation
    // Updates speedCorrection
    // Returns if not enough time has passed
    if (!pid.Compute())
    {
        Serial.println("----------------------ABORTED----------------------------");
        return false;
    }


    float newPWMVal = curPWM + speedCorrection;

    if (abs(motorSpeed)<10 && abs(curMotorSpeed)<1000 && abs(newPWMVal) < 30)
    {
        newPWMVal = 0;
    }

    // Protection against stalling and getting stuck (motors cannot be driven above 252 in duty cycle)
    if (newPWMVal>250)
    {
        newPWMVal = 250;
    }
    else if (newPWMVal<-250)
    {
        newPWMVal = -250;
    }
    
    // Speed correction by PWM
    if (usePIDCorr==true)
    {
        setPWM(newPWMVal);
    }

    // printValues();

    // Set variables for next time
    lastEncoderPos = encoderPos;
    lastUpdateTime = updateTime;
    return true;
}

void MotorController::startDistanceMeasure()
{
    tickStartDistance = encoderPos;
}


double MotorController::getDistance()
{
    return (encoderPos-tickStartDistance)/(cpr*encOutputRatio);
}

double MotorController::getOutputSpeed()
{
    return curMotorSpeed/encOutputRatio;
}

void MotorController::printValues()
{
    Serial.print("motorSpeed:");Serial.print(motorSpeed);Serial.print(",");
    Serial.print("curMotorSpeed:");Serial.print(curMotorSpeed);Serial.print(",");
    Serial.print("curPWM:");Serial.print(curPWM);Serial.print(",");
    Serial.print("speedCorrection:");Serial.print(speedCorrection);Serial.print(",");
    // Serial.print("error:");Serial.print(motorSpeed-curMotorSpeed);Serial.print(",");
    Serial.print("outputSpeed:");Serial.print(getOutputSpeed());Serial.print(",");
    // Serial.print("encoderPos:");Serial.print(encoderPos);Serial.print(",");
    Serial.println("");
}