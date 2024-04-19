#pragma once

#include <vector>
#include <fstream>

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
        // Stores which colour the threshold is for.
        TileColours type {TileColours::Unknown};


        // Thresholds for maximum allowed distance outside of radius that can still count as identification
        #warning untuned constants
        static constexpr double MAX_DETECTION_DISTANCE_BLACK {1000};
        static constexpr double MAX_DETECTION_DISTANCE_BLUE {1000};
        static constexpr double MAX_DETECTION_DISTANCE_REFLECTIVE {1000};
        static constexpr double MAX_DETECTION_DISTANCE_WHITE {1000};

        // Thresholds for what percentage of detections are needed on a tile
        // Black is not needed as it is not averaged
        #warning untuned constants
        static constexpr double TILE_DETECTION_THRESHOLD_WHITE {0.5};
        static constexpr double TILE_DETECTION_THRESHOLD_BLUE {0.6};
        static constexpr double TILE_DETECTION_THRESHOLD_REFLECTIVE {0.3};
        // The actual threshold in use
        double tileAvgDetectionThreshold {};

        // Minimum radiuses for the colour
        #warning untuned constants
        static constexpr double STANDARD_RADIUS_BLACK {0};
        static constexpr double STANDARD_RADIUS_BLUE {0};
        static constexpr double STANDARD_RADIUS_REFLECTIVE {0};
        static constexpr double STANDARD_RADIUS_WHITE {0};
        double standardRadius {};


        // For file storage
        // Get the filename for this type
        std::string getFileNameFromType(TileColours type);

    public:
        ColourThreshold();
        ColourThreshold(TileColours type);
        // True if the sample is within the threshold
        bool isWithinThreshold(ColourSample otherSample);
        // Gets the colour distance to the sample
        double getDistanceToSample(ColourSample sample);
        // Gets the colour distance to the radius (to sample minus the radius)
        double getDistanceToRadius(ColourSample sample);

        bool isAboveTileAvgThresh(double share);

        void setType(TileColours tileColour);
        void setSample(ColourSample sample);
        void setRadius(double radius);
        friend std::ostream& operator<<(std::ostream& out, const ColourThreshold& thresh);

        void writeToFile();
        void readFromFile();

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
        // ColourSampleCollection samplesOnStartTile {};
        ColourSampleCollection samplesOnNext {};
        ColourSample curSample {};

        // The last known current tile colour
        TileColours lastKnownCurTileColour {TileColours::White};

        std::array<ColourThreshold, static_cast<int>(TileColours::ColNum)> thresholds {};

        // Identify the colours on each sample, independently of eachother
        void classifySamples(ColourSampleCollection& samples);
        // Determine the colour of one sample
        void classifySample(ColourSample& sample);
        // Determine the closest colour when taking the radius into account
        TileColours getClosestColour(ColourSample sample);

        // Mix the colour values to give you one colour for the whole tile
        TileColours getTileColourFromClassifiedSamples(ColourSampleCollection& samples);
        std::array<double, static_cast<int>(TileColours::ColNum)> getNewTileColourShares(ColourSampleCollection& collection);
        TileColours determineColourFromColourShares(std::array<double, static_cast<int>(TileColours::ColNum)> shares);


    public:
        ColourIdentifier();
        // Input ---------------------------------------------------------

        // Add a colour sample to the list of samples being judged.
        void registerColourSample(ColourSample sample);

        // Clear all stored colour samples. Do when starting a new move, for example.
        void clearColourSamples();

        // Call with true when the sensor passes over to the next tile
        void setSensorOnNextTile(bool nextTile);

        // Set the threshold to use
        void setThreshold(TileColours type, ColourThreshold threshold);

        // Colour output ---------------------------------------------------------

        // Get the tile colour from the collected samples (i.e. of the next tile)
        TileColours getTileColour();
        // Get the tile colour of the tile directly beneath the sensor.
        TileColours getCurTileColour();

        // Reads stored thresholds from files
        void readThresholdsFromFiles();
        // Stores thresholds to files
        void storeThresholds();




        // Calibration ------------------------------------------------------------



};