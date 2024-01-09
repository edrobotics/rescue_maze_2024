#pragma once
#include "VL53L0X.h"
#include <DFRobot_MCP23017.h>

class TofVl53l0x
{
    public:

        TofVl53l0x();

        // Disable sensor by pulling XSHUT low
        void disable();
        // Enable sensor by releasing XSHUT OBS!!!!!!!!! Do not pull high! Set to input instead!!!
        void enable();
        // Reset the sensor (OBS: Blocking)
        void reset();
        // Set the necessary variables
        void setVars(int addr, DFRobot_MCP23017::ePin_t pin, int samplingTime, DFRobot_MCP23017* ioExpander);
        // Perform all necessary initialization (includes setting address)
        bool init();

        // Updates the sensor value.
        // Returns:
        // - Distance in mm if successful
        // - -1 if unsuccessful (sensor not ready (too little time since last measurement) or some other issue)
        int update();

    private:
        // I2C address of the sensor
        int i2cAddr {};
        // IO Expander pin which the sensor XSHUT pin is connected to
        DFRobot_MCP23017::ePin_t xShutPin {};
        const int TIMEOUT_TIME {500};
        // Time for 1 measurement (microseconds)
        const int TIMING_BUDGET {20000};
        // Time between measurements
        int samplingTime {};

        VL53L0X sensor {};
        DFRobot_MCP23017* ioExpander {};
        
};