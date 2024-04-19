#include "fusion/ColourIdentifier.h"


bool ColourThreshold::isWithinThreshold(ColourSample otherSample)
{
    return (ColourSample::calcColourDistance(sample, otherSample) <= radius+maxDetectionDistance);
}







void ColourIdentifier::registerColourSample(ColourSample sample)
{
    if (onNextTile)
    {
        samplesOnNext.samples.push_back(sample);
    }
    else
    {
        samplesOnStartTile.samples.push_back(sample);
    }
}


void ColourIdentifier::clearColourSamples()
{
    samplesOnStartTile.samples.clear();
    samplesOnNext.samples.clear();
}


void ColourIdentifier::setSensorOnNextTile(bool nextTile)
{
    onNextTile = nextTile;
}

TileColours ColourIdentifier::getTileColour()
{
    // Calculate colour vlaues for all the individual values
    classifySamples(samplesOnNext);

    return getTileColourFromClassifiedSamples(samplesOnNext);
}


void ColourIdentifier::classifySamples(ColourSampleCollection& samples)
{
    for (auto& sample : samples.samples)
    {
        classifySample(sample);
    }
}


void ColourIdentifier::classifySample(ColourSample& sample)
{
    TileColours closestCol {getClosestColour(sample)};
    ColourThreshold threshold {};
    // Set threshold to the closest colour
    switch (closestCol)
    {
        case TileColours::Black:
        threshold = thresholds.at(static_cast<int>(TileColours::Black));
            break;
        case TileColours::Blue:
        threshold = thresholds.at(static_cast<int>(TileColours::Blue));
            break;
        case TileColours::Checkpoint:
            // Also check if white was detected. If it was, use that instead as it is safer. Remember to change closestCol here also.
            #error reflective and white colour detecting not yet
            break;
        case TileColours::White:
            threshold = thresholds.at(static_cast<int>(TileColours::White));
            break;
        default:
            break;
    }

    if (threshold.isWithinThreshold(sample))
    {
        sample.classification = closestCol;
    }
    else
    {
        sample.classification =  TileColours::Unknown;
    }
}