#pragma once

#include <array>
#include <thread>
#include <chrono>

#include "fusion/i2cCommunicator.h"
#include "fusion/TCS34725.h"

enum class TileColours
{
    Checkpoint=0,
    Blue=1,
    Black=2,
    White=3,
    RealNum=4,
    NotUpdated,
    Unknown,
};

enum FundamentalColour
{
    fcol_r,
    fcol_g,
    fcol_b,
    fcol_c,
    fcol_num,
};

class ColourSample
{
    public:
    std::array<int, fcol_num> values {};
    bool wasDone {false};
    TileColours classification {TileColours::Unknown};
    static double calcColourDistance(ColourSample s1, ColourSample s2);
};

class ColourSensor
{
    private:
        // Initialized in constructor
        Adafruit_TCS34725 colSens;

        std::chrono::steady_clock::time_point lastReadTime {std::chrono::steady_clock::now()};
        ColourSample getValuesFromSensor();

        // Also set in aggregate initialization i .cpp file
        static constexpr double INTEGRATION_TIME_MS {50};

    public:
        ColourSensor(i2cCommunicator* i2cComm);

        // Initialize sensor
        void init();

        // Read the sensor and store in colSample
        // true=updated, false=not updated
        bool updateVals();

        ColourSample colSample {};

};