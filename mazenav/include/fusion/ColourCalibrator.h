#pragma once

#include <thread>
#include <chrono>
#include <vector>
#include <algorithm>

#include "fusion/PiAbstractor.h"
#include "fusion/Sensors.h"
#include "fusion/ColourSensor.h"
#include "fusion/ColourIdentifier.h"

enum class PanelState
{

};

enum class ButtonState
{
    black,
    blue,
    reflective,
    white,
    none,
};


class ColourCalibrator
{
    private:

        Sensors* sensors {nullptr};
        ColourIdentifier* colId {nullptr};
        PiAbstractor* piAbs {nullptr};

        #warning maybe need to change pin numbers around
        static constexpr int PIN_WHITE {30};
        static constexpr int PIN_RELECTIVE {22};
        static constexpr int PIN_BLUE {21};
        static constexpr int PIN_BLACK {23};
        static constexpr int PIN_CONN {24};
        ButtonState getButtonStates();
        bool getIsPanelConnected();


        // Colour sample vectors
        std::vector<ColourSample> blackSamples {};
        ColourThreshold blackThreshold {TileColours::Black};

        std::vector<ColourSample> blueSamples {};
        ColourThreshold blueThreshold {TileColours::Blue};
        
        std::vector<ColourSample> reflectiveSamples {};
        ColourThreshold reflectiveThreshold {TileColours::Checkpoint};
        
        std::vector<ColourSample> whiteSamples {};
        ColourThreshold whiteThreshold {TileColours::White};


        void calculate(std::vector<ColourSample> samples, ColourThreshold& threshold);

        ColourSample calcAverage(std::vector<ColourSample> samples);
        int findMinValue(std::vector<ColourSample> samples, FundamentalColour fCol);
        int findMaxValue(std::vector<ColourSample> samples, FundamentalColour fCol);

    public:

        // Init the hardware communication pins
        void init(PiAbstractor* piAbs);

        // True if calibration device is plugged in
        bool checkActivated();

        // Run the calibration sequence - returns once device is unplugged
        void calibrate(Sensors* sensors, ColourIdentifier* colId);

};