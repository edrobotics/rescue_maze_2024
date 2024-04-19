#include "fusion/ColourSensor.h"


std::ostream& operator<< (std::ostream& out, const TileColours& tileColour)
{
    switch (tileColour)
    {
        case TileColours::Black:
            out << "black";
            break;
        case TileColours::Blue:
            out << "blue";
            break;
        case TileColours::Checkpoint:
            out << "checkpoint/reflective";
            break;
        case TileColours::White:
            out << "white";
            break;
        case TileColours::Unknown:
            out << "unknown";
            break;
        case TileColours::NotUpdated:
            out << "notUpdated";
            break;
        default:
            out << "(invalid TileColours)";
    }

    return out;
}

std::string stringFromTileColours(TileColours tileColour)
{
    std::string out {};
    switch (tileColour)
    {
        case TileColours::Black:
            out = "black";
            break;
        case TileColours::Blue:
            out = "blue";
            break;
        case TileColours::Checkpoint:
            out = "checkpoint_reflective";
            break;
        case TileColours::White:
            out = "white";
            break;
        case TileColours::Unknown:
            out = "unknown";
            break;
        case TileColours::NotUpdated:
            out = "notUpdated";
            break;
        default:
            // out = "(invalid TileColours)";
            break;
    }

    return out;

}

std::ostream& operator<< (std::ostream& out, const ColourSample& sample)
{
    out << "r=" << sample.values.at(fcol_r)
    << " g=" << sample.values.at(fcol_g)
    << " b=" << sample.values.at(fcol_b)
    << " c=" << sample.values.at(fcol_c)
    << " wasDone=" << sample.wasDone
    << " classification=" << sample.classification
    << " rotX=" << sample.rotX
    << " rotY=" << sample.rotY;

    return out;

}

ColourSensor::ColourSensor(i2cCommunicator* i2cComm)
: colSens {TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X, i2cComm}
{
}


void ColourSensor::init()
{
    while(!colSens.begin())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        std::cout << "TCS34725 not found\n";
    }
}


ColourSample ColourSensor::getValuesFromSensor()
{
    ColourSample retVal {};
    long milsSinceRead {std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lastReadTime).count()};
    if (!(milsSinceRead>INTEGRATION_TIME_MS))
    {
        retVal.wasDone = false;
        return retVal;
    }

    uint16_t r {};
    uint16_t g {};
    uint16_t b {};
    uint16_t c {};
    colSens.getRawData(&r, &g, &b, &c);

    retVal.values.at(fcol_r) = r;
    retVal.values.at(fcol_g) = g;
    retVal.values.at(fcol_b) = b;
    retVal.values.at(fcol_c) = c;
    retVal.wasDone = true;
    
    lastReadTime = std::chrono::steady_clock::now();

    return retVal;
}


bool ColourSensor::updateVals()
{
    colSample = getValuesFromSensor();
    return colSample.wasDone;
}

double ColourSample::calcColourDistance(ColourSample s1, ColourSample s2)
{
    return sqrt(pow(s1.values.at(fcol_r)-s2.values.at(fcol_r), 2)+pow(s1.values.at(fcol_g)-s2.values.at(fcol_g), 2)+pow(s1.values.at(fcol_b)-s2.values.at(fcol_b), 2)+pow(s1.values.at(fcol_c)-s2.values.at(fcol_c), 2));
}