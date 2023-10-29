#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <QuickPID.h>

// Settings for Encoder library
// #define ENCODER_OPTIMIZE_INTERRUPTS // Leads to a linking error
#include <Encoder.h>

class MotorController
{
    public:
    MotorController(int enA, int enB, int pwmPin, int dirPin, double encRatio, int cpr);
    
    bool init();
    void update(); // Read encoder positions and set new PID loop value
    void setSpeed(double speed); // Sets the speed in rpm
    void setPWM(int pwmSpeed); // Sets the PWM output. Clamps if too great.

    void startDistanceMeasure(); // Begin distance measuring
    // Maybe replace these with passing pointers in the beginning and then updating the values through pointers?
    double getDistance(); // Returns the distance turned on output shaft since beginning distance measuring. Measured in rounds. Maybe int where every step is part of full revolution?
    double getOutputSpeed(); // Returns the current speed of the output shaft, measuren in rpm.
    bool isReversed {false}; // If the direction of the motor is inverted. Should this apply to the encoderPosition as well?

    void printValues();


    private:

    // Data about the motor and encoder
    double encOutputRatio; // How many turns the encoder makes for one turn of the output shaft.
    int cpr; // Counts per revolution on the encoder shaft. Only counts rising/falling edges, not both!

    // Encoder tracking
    Encoder encoder;
    long encoderPos {0}; // Position of the encoder, in ticks. Reversing applies here already. A long should be sufficient for unreasonably large distances...
    float curMotorSpeed {0}; // Current motor speed in rpm
    // double curMotorOutSpeed {0}; // Current output shaft speed in rpm. Not really used?
    // const long MILLIS_PER_MINUTE {long(60000)};
    #define MILLIS_PER_MINUTE 60000 // Bad to use #define?

    // Distance measuring
    long tickStartDistance {0}; // Ticks driven since begin of distancemeasure

    // Motor controlling
    int pwmPin;
    int dirPin;
    double outputSpeed {0}; // Wanted output speed in rpm
    float motorSpeed {0}; // Wanted motor speed in rpm
    float speedCorrection;

    // PID
    #warning untuned constants
    float Kp {0.04};
    float Ki {0};
    float Kd {0};
    QuickPID pid {&curMotorSpeed, &speedCorrection, &motorSpeed, Kp, Ki, Kd, QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas, QuickPID::iAwMode::iAwCondition, QuickPID::Action::direct};
    

};


#endif //MOTOR_CONTROLLER_H