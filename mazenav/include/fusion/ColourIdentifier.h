#pragma once

#include <vector>

#include "fusion/ColourSensor.h"


class ColourThreshold
{
    private:
        // The sample
        ColourSample sample {};
        // Radius used for closeness calculation
        double radius {};
        // Maximal distance outside of radius that can still count as identification
        double maxDetectionDistance {};

    public:

        bool isWithinThreshold(ColourSample otherSample);
};


class ColourSampleCollection
{
    private:
    
    public:
        std::vector<ColourSample> samples {};
        // Clear the samples to prepare for new ones.
        // void clear();
        // Add a sample
        // void pushBackSample(ColourSample sample);

};












class ColourIdentifier
{
    private:
        // Determine which tile we are on
        bool onNextTile {};
        // Store the colour samples
        ColourSampleCollection samplesOnStartTile {};
        ColourSampleCollection samplesOnNext {};

        std::array<ColourThreshold, static_cast<int>(TileColours::RealNum)> thresholds {};

        // Identify the colours on each sample, independently of eachother
        void classifySamples(ColourSampleCollection& samples);
        // Determine the colour of one sample
        void classifySample(ColourSample& sample);
        // Determine the closest colour when taking the radius into account
        TileColours getClosestColour(ColourSample sample);

        // Mix the colour values to give you one colour for the whole tile
        TileColours getTileColourFromClassifiedSamples(ColourSampleCollection samples);


    public:
        // Input ---------------------------------------------------------

        // Add a colour sample to the list of samples being judged.
        void registerColourSample(ColourSample sample);

        // Clear all stored colour samples. Do when starting a new move, for example.
        void clearColourSamples();

        // Call with true when the sensor passes over to the next tile
        void setSensorOnNextTile(bool nextTile);

        // Colour output ---------------------------------------------------------

        // Get the tile colour from the collected samples
        TileColours getTileColour();



        // Calibration ------------------------------------------------------------



};