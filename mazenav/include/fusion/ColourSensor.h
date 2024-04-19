#pragma once

#include <array>
#include <thread>
#include <chrono>
#include <cmath>
#include <iostream>

#include "fusion/i2cCommunicator.h"
#include "fusion/TCS34725.h"

enum class TileColours
{
    Checkpoint=0,
    Blue=1,
    Black=2,
    White=3,
    Unknown=4,
    ColNum=5,
    NotUpdated,
};

std::ostream& operator<< (std::ostream& out, const TileColours& tileColour);
std::string stringFromTileColours(TileColours tileColour);

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
    // The actual values
    std::array<int, fcol_num> values {};
    // Wether value was updated by sensor or not
    bool wasDone {false};
    // What TileColour it was classified as
    TileColours classification {TileColours::Unknown};
    // Robot x rotation at time of measurement
    double rotX {};
    // Robot y rotation at time of measurement
    double rotY {};


    static double calcColourDistance(ColourSample s1, ColourSample s2);

    friend std::ostream& operator<< (std::ostream& out, const ColourSample& sample);
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