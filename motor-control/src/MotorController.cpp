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
    pid.SetOutputLimits(-255, 255); // Limits for setPwm(), positive and negative
    #warning untuned sample time
    pid.SetSampleTimeUs(pidSampleTimeUs);
    #warning PID setup not done!(?)

    pinMode(dirPin, OUTPUT);
    pinMode(pwmPin, OUTPUT);

    // Nothing went wrong, so we return true (otherwise we returned false earlier)
    return true;
}

void MotorController::setSpeed(double speed)
{
    // outputSpeed = speed; // Should not be updated here? Not needed?
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

    #warning unsure if high or low means forwards or backwards
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

double MotorController::rpmToPwm(double rpmSpeed)
{
    // return rpmSpeed/encOutputRatio;
    return rpmSpeed/33.4154675813246;
}

bool MotorController::update()
{
    update(true);
}

bool MotorController::update(bool usePIDCorr)
{
    long updateTime = micros();
    static long lastUpdateTime = 0;
    static long lastEncoderPos = 0;
    
    // Check that enough time has passed, so that we do not calculate stuff unnecessarily
    if (updateTime-lastUpdateTime < pidSampleTimeUs+300)
    {
        return false;
    }

    long read = encoder.read();
    encoderPos = (isReversed) ? -read : read;

    // Calculate speeds
    #warning possible overflows and truncations ahead. Fixed?
    if (updateTime==lastUpdateTime) return; // Would result in division by 0
    curMotorSpeed = double(encoderPos-lastEncoderPos)/double(cpr*(updateTime-lastUpdateTime)) * double(MICROS_PER_MINUTE);
    // If the value is this large, something is wrong. Should fix problems with very negative numbers.
    if (abs(curMotorSpeed)>15000)
    {
        #warning dirty fix for the negative values? Yes, it is dirty and it blocks the PID for long periods when the value is negative
        return false;
    }

    // PID Loop calculation
    // Updates speedCorrection
    // Returns if not enough time has passed
    if (!pid.Compute())
    {
        Serial.println("----------------------ABORTED----------------------------");
        return false;
    }

    if (motorSpeed==0)
    {
        #warning crude solution for integral term problem that will not work later
        pid.Reset(); // Reset the controller to prevent integral windup.
    }
    
    // Speed correction by PWM
    if (usePIDCorr==true)
    {
        setPWM(rpmToPwm(motorSpeed)+speedCorrection);
    }

    // Serial.print("encoderPos:");Serial.print(encoderPos);Serial.print(",");
    // Serial.print(";Serial.print(encoderPos);Serial.print(",");
    // Serial.print("lastEncoderPolastEncoderPos:");Serial.print(lastEncoderPos);Serial.print(",");
    // Serial.print("deltaEncoderPos:");Serial.print(encoderPos-lastEncoderPos);Serial.print(",");
    // Serial.print("updateTime:");Serial.print(updateTime);Serial.print(",");
    // Serial.print("lastUpdateTime:");Serial.print(lastUpdateTime);Serial.print(",");
    // Serial.print("deltaTime:");Serial.print(updateTime-lastUpdateTime);Serial.print(",");
    // Serial.print("curMotorSpeed:");Serial.print(curMotorSpeed);Serial.print(",");
    // Serial.println("");

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
    // Serial.print("curPWM:");Serial.print(curPWM);Serial.print(",");
    Serial.print("speedCorrection:");Serial.print(speedCorrection);Serial.print(",");
    // Serial.print("error:");Serial.print(motorSpeed-curMotorSpeed);Serial.print(",");
    Serial.print("outputSpeed:");Serial.print(getOutputSpeed());Serial.print(",");
    Serial.print("encoderPos:");Serial.print(encoderPos);Serial.print(",");
    Serial.println("");
}

void MotorController::printMotorSpeed()
{
    Serial.println(curMotorSpeed);
}
















void MotorController::pwmRPMCalibration()
{
    // Input parameters
    int startPWM = 40;
    int endPWM = 255;
    int pwmStep = 20;
    int toTest = 2*((endPWM-startPWM)/pwmStep + 2)+1; // Truncation on purpose
    int middle = toTest/2; // Truncation on purpose
    
    double rpms[toTest];
    int pwms[toTest] {};

    // Set pwms to test
    pwms[middle] = 0;
    for (int i=startPWM;i<=endPWM;i+=pwmStep)
    {
        static int num = middle+1;
        pwms[num] = i;
        pwms[2*middle - num] = -i;

        ++num;
    }
    pwms[toTest-1] = endPWM;
    pwms[0] = -endPWM;

    // Run the motors and calculate the rpms
    for (int i=0;i<toTest;++i)
    {
        rpms[i] = calibrateSinglePWM_RPM(pwms[i]);
        delay(1000);
    }

    // Print the data to serial (to be replaced with calculation later)
    Serial.println("---------------Calibration data for motor---------------");
    for (int i=0;i<toTest;++i)
    {
        printCsvVars(pwms[i], rpms[i]);
    }
    Serial.println("---------------END---------------");
}

double MotorController::calibrateSinglePWM_RPM(int pwmToCal)
{
    if (pwmToCal<0)
    {
        setPWM(-200);
    }
    else if (pwmToCal==0)
    {
        setPWM(0);
    }
    else
    {
        setPWM(200);
    }
    delay(42);
    setPWM(pwmToCal);
    delay(169);
    double motorSpeedSum = 0;
    for (int i=0;i<calibrationIterations;++i)
    {
        while(!update(false))
        {
            delay(1);
        }
        motorSpeedSum+=curMotorSpeed;
    }
    setPWM(0);
    return motorSpeedSum/double(calibrationIterations);

}

void MotorController::printCsvVars(int PWM, double RPM)
{
    Serial.print(PWM);Serial.print(",");Serial.print(RPM);Serial.println("");
}