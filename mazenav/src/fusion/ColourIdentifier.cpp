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
            standardRadius = STANDARD_RADIUS_BLUE;
            maxDetectionDistance = MAX_DETECTION_DISTANCE_BLUE;
            break;
        
        case TileColours::White:
            tileAvgDetectionThreshold = TILE_DETECTION_THRESHOLD_WHITE;
            standardRadius = STANDARD_RADIUS_WHITE;
            maxDetectionDistance = MAX_DETECTION_DISTANCE_WHITE;
            break;

        case TileColours::Checkpoint:
            tileAvgDetectionThreshold = TILE_DETECTION_THRESHOLD_REFLECTIVE;
            standardRadius = STANDARD_RADIUS_REFLECTIVE;
            maxDetectionDistance = MAX_DETECTION_DISTANCE_REFLECTIVE;
            break;

        case TileColours::Black:
            tileAvgDetectionThreshold = 1;
            standardRadius = STANDARD_RADIUS_BLACK;
            maxDetectionDistance = MAX_DETECTION_DISTANCE_BLACK;
            break;
        
        default:
            tileAvgDetectionThreshold = 1;
            standardRadius = 0;
            maxDetectionDistance = 0;
            break;
    }
}

void ColourThreshold::setSample(ColourSample sample)
{
    this->sample = sample;
}

void ColourThreshold::setRadius(double radius)
{
    if (radius>standardRadius)
    {
        this->radius = radius;
    }
    else
    {
        this->radius = standardRadius;
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

double ColourThreshold::getDistanceToRadius(ColourSample sample)
{
    return getDistanceToSample(sample)-radius;
}

std::ostream& operator<<(std::ostream& out, const ColourThreshold& thresh)
{
    out << thresh.sample << ", " << "radius=" << thresh.radius;
    return out;
}

std::string ColourThreshold::getFileNameFromType(TileColours type)
{
    return "colCal_" + stringFromTileColours(type) + ".txt";
}



void ColourThreshold::writeToFile()
{
    std::ofstream storeFile(getFileNameFromType(type));
    if (!storeFile)
    {
        // std::cerr << "could not open file: " << getFileNameFromType(type) << "\n";
    }
    
    storeFile
    << sample.values.at(fcol_r) << " "
    << sample.values.at(fcol_g) << " "
    << sample.values.at(fcol_b) << " "
    << sample.values.at(fcol_c) << " "
    << radius << std::endl;

    storeFile.close();
}

void ColourThreshold::readFromFile()
{
    std::ifstream storeFile(getFileNameFromType(type));
    if (!storeFile)
    {
        std::cerr << "could not open file: " << getFileNameFromType(type) << "\n";
        return;
    }
    storeFile >> sample.values.at(fcol_r);
    storeFile >> sample.values.at(fcol_g);
    storeFile >> sample.values.at(fcol_b);
    storeFile >> sample.values.at(fcol_c);
    storeFile >> radius;

    storeFile.close();

}




ColourIdentifier::ColourIdentifier()
{
    thresholds.at(static_cast<int>(TileColours::Black)).setType(TileColours::Black);
    thresholds.at(static_cast<int>(TileColours::Blue)).setType(TileColours::Blue);
    thresholds.at(static_cast<int>(TileColours::White)).setType(TileColours::White);
    thresholds.at(static_cast<int>(TileColours::Checkpoint)).setType(TileColours::Checkpoint);
    readThresholdsFromFiles();
}


void ColourIdentifier::registerColourSample(ColourSample sample)
{
    curSample = sample;
    if (onNextTile)
    {
        samplesOnNext.samples.push_back(curSample);
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


void ColourIdentifier::setThreshold(TileColours type, ColourThreshold threshold)
{
    if (type==TileColours::Unknown || type==TileColours::NotUpdated || type==TileColours::ColNum)
    {
        std::cout << "could not update threshold with this type\n";
        return;
    }
    else
    {
        thresholds.at(static_cast<int>(type)) = threshold;
        std::cout << "Wrote threshold: " << threshold << "\n";
    }
}

TileColours ColourIdentifier::getTileColour()
{
    // Calculate colour vlaues for all the individual values
    classifySamples(samplesOnNext);

    return getTileColourFromClassifiedSamples(samplesOnNext);
}

TileColours ColourIdentifier::getCurTileColour()
{
    classifySample(curSample);
    std::cout << "Current sample is: " << curSample << "\n";
    if (curSample.classification == TileColours::Unknown || curSample.classification == TileColours::NotUpdated)
    {
        // std::cout << "unknown or notUpdated\n";
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
    std::cout << "closestCol: " << closestCol << '\n';
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
            threshold = thresholds.at(static_cast<int>(TileColours::Checkpoint));
            #warning disabled white failsafe for reflective tiles
            // If also detecting white, use white as it is safer.
            // if (thresholds.at(static_cast<int>(TileColours::White)).isWithinThreshold(sample))
            // {
            //     // Detect white
            //     threshold = thresholds.at(static_cast<int>(TileColours::White));
            //     closestCol = TileColours::White;
            // }
            // else
            // {
            //     threshold = thresholds.at(static_cast<int>(TileColours::Checkpoint));
            // }
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
        colDistances.at(i) = thresholds.at(i).getDistanceToRadius(sample);
    }

    // Find the min and its index
    TileColours closestCol {TileColours::White};
    double minVal {colDistances.at(static_cast<int>(closestCol))};
    for (int i=0;i<static_cast<int>(TileColours::ColNum)-1;++i)
    {
        if (colDistances.at(i) < minVal)
        {
            minVal = colDistances.at(i);
            closestCol = static_cast<TileColours>(i);
        }
    }

    return closestCol;
}


void ColourIdentifier::readThresholdsFromFiles()
{
    for (auto& threshold : thresholds)
    {
        threshold.readFromFile();
    }
}

void ColourIdentifier::storeThresholds()
{
    for (auto& threshold : thresholds)
    {
        threshold.writeToFile();
    }
}
