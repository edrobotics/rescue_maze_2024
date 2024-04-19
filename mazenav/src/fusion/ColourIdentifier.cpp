#include "fusion/ColourIdentifier.h"

ColourThreshold::ColourThreshold()
{
    ColourThreshold(TileColours::Unknown);
}

ColourThreshold::ColourThreshold(TileColours type)
{
    setType(type);
}

bool ColourThreshold::isWithinThreshold(ColourSample otherSample)
{
    return (ColourSample::calcColourDistance(sample, otherSample) <= radius+maxDetectionDistance);
}

void ColourThreshold::setType(TileColours tileColour)
{
    type = tileColour;
    switch (tileColour)
    {
        case TileColours::Blue:
            tileAvgDetectionThreshold = TILE_DETECTION_THRESHOLD_BLUE;
            break;
        
        case TileColours::White:
            tileAvgDetectionThreshold = TILE_DETECTION_THRESHOLD_WHITE;
            break;

        case TileColours::Checkpoint:
            tileAvgDetectionThreshold = TILE_DETECTION_THRESHOLD_REFLECTIVE;
            break;
        
        default:
            tileAvgDetectionThreshold = 1;
            break;
    }
}


bool ColourThreshold::isAboveTileAvgThresh(double share)
{
    return share>tileAvgDetectionThreshold;
}


double ColourThreshold::getDistanceToSample(ColourSample sample)
{
    return ColourSample::calcColourDistance(this->sample, sample);
}







ColourIdentifier::ColourIdentifier()
{
    thresholds.at(static_cast<int>(TileColours::Black)).setType(TileColours::Black);
    thresholds.at(static_cast<int>(TileColours::Blue)).setType(TileColours::Blue);
    thresholds.at(static_cast<int>(TileColours::White)).setType(TileColours::White);
    thresholds.at(static_cast<int>(TileColours::Checkpoint)).setType(TileColours::Checkpoint);
}


void ColourIdentifier::registerColourSample(ColourSample sample)
{
    if (onNextTile)
    {
        samplesOnNext.samples.push_back(sample);
    }
    else
    {
        // samplesOnStartTile.samples.push_back(sample);
    }
}


void ColourIdentifier::clearColourSamples()
{
    // samplesOnStartTile.samples.clear();
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

TileColours ColourIdentifier::getCurTileColour()
{
    classifySample(samplesOnNext.samples.back());
    ColourSample curSample {samplesOnNext.samples.back()};
    std::cout << "Current tile colour is: " << curSample.classification << '\n';
    if (curSample.classification == TileColours::Unknown || curSample.classification == TileColours::NotUpdated)
    {
        std::cout << "unknown or notUpdated\n";
        return lastKnownCurTileColour;
    }
    else
    {
        lastKnownCurTileColour = curSample.classification;
        return curSample.classification;
    }
}


TileColours ColourIdentifier::getTileColourFromClassifiedSamples(ColourSampleCollection& collection)
{
    classifySamples(collection);

    TileColours  tileColour {determineColourFromColourShares(getNewTileColourShares(collection))};

    return tileColour;
}


std::array<double, static_cast<int>(TileColours::ColNum)> ColourIdentifier::getNewTileColourShares(ColourSampleCollection& collection)
{
     // The number of samples of each colour
    double blackNum {0};
    double blueNum {0};
    double whiteNum {0};
    double reflectiveNum {0};
    double unknownNotUpdatedNum {0};
    int sampleNum {collection.samples.size()};

    for (auto& sample : collection.samples)
    {
        switch (sample.classification)
        {
            case TileColours::Black:
                ++blackNum;
                break;
            
            case TileColours::Blue:
                ++blueNum;
                break;

            case TileColours::White:
                ++blueNum;
                break;

            case TileColours::Checkpoint:
                ++blueNum;
                break;
            
            default:
                ++unknownNotUpdatedNum;
                break;
        }
    }

    std::array<double, static_cast<int>(TileColours::ColNum)> retArr {};
    retArr.at(static_cast<int>(TileColours::Black)) = blackNum/static_cast<double>(sampleNum);
    retArr.at(static_cast<int>(TileColours::Blue)) = blueNum/static_cast<double>(sampleNum);
    retArr.at(static_cast<int>(TileColours::White)) = whiteNum/static_cast<double>(sampleNum);
    retArr.at(static_cast<int>(TileColours::Checkpoint)) = reflectiveNum/static_cast<double>(sampleNum);
    retArr.at(static_cast<int>(TileColours::Unknown)) = unknownNotUpdatedNum/static_cast<double>(sampleNum);

    return retArr;
}


TileColours ColourIdentifier::determineColourFromColourShares(std::array<double, static_cast<int>(TileColours::ColNum)> shares)
{
    TileColours tilecolour {TileColours::Blue};
    if (thresholds.at(static_cast<int>(tilecolour)).isAboveTileAvgThresh(shares.at(static_cast<int>(tilecolour))))
    {
        return tilecolour;
    }

    tilecolour = TileColours::Checkpoint;
    if (thresholds.at(static_cast<int>(tilecolour)).isAboveTileAvgThresh(shares.at(static_cast<int>(tilecolour))))
    {
        return tilecolour;
    }

    tilecolour = TileColours::White;
    if (thresholds.at(static_cast<int>(tilecolour)).isAboveTileAvgThresh(shares.at(static_cast<int>(tilecolour))))
    {
        return tilecolour;
    }

    // Nothing matched
    return TileColours::Unknown;

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
    ColourThreshold threshold {TileColours::Unknown};
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
            
            // If also detecting white, use white as it is safer.
            if (thresholds.at(static_cast<int>(TileColours::White)).isWithinThreshold(sample))
            {
                // Detect white
                threshold = thresholds.at(static_cast<int>(TileColours::White));
                closestCol = TileColours::White;
            }
            else
            {
                threshold = thresholds.at(static_cast<int>(TileColours::Checkpoint));
            }
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


TileColours ColourIdentifier::getClosestColour(ColourSample sample)
{
    std::array<double, static_cast<int>(TileColours::ColNum)-1> colDistances {};
    // Fill the array
    for (int i=0;i<static_cast<int>(TileColours::ColNum)-1;++i)
    {
        colDistances.at(i) = thresholds.at(i).getDistanceToSample(sample);
    }

    // Find the max and its index
    double maxVal {};
    TileColours closestCol {TileColours::Unknown};
    for (int i=0;i<static_cast<int>(TileColours::ColNum)-1;++i)
    {
        if (colDistances.at(i) > maxVal)
        {
            maxVal = colDistances.at(i);
            closestCol = static_cast<TileColours>(i);
        }
    }

    return closestCol;
}