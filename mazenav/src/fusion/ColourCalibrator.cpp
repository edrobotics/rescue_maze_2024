#include "fusion/ColourCalibrator.h"

void ColourCalibrator::init(PiAbstractor* piAbs)
{
    this->piAbs = piAbs;

    piAbs->pinModeTS(PIN_WHITE, PiAbstractor::PinMode::inputPullup);
    piAbs->pinModeTS(PIN_BLUE, PiAbstractor::PinMode::inputPullup);
    piAbs->pinModeTS(PIN_RELECTIVE, PiAbstractor::PinMode::inputPullup);
    piAbs->pinModeTS(PIN_BLACK, PiAbstractor::PinMode::inputPullup);
    piAbs->pinModeTS(PIN_CONN, PiAbstractor::PinMode::inputPullup);
}


void ColourCalibrator::calibrate(Sensors* sensors, ColourIdentifier* colId)
{
    this->sensors = sensors;
    this->colId = colId;

    bool connected {true};

    while (connected)
    {
        sensors->update(false);

        ButtonState buttState {getButtonStates()};
        switch (buttState)
        {
            case ButtonState::black:
                std::cout << "Calibrating black\n";
                blackSamples.push_back(sensors->colSens.colSample);
                calculate(blackSamples, blackThreshold);
                colId->setThreshold(TileColours::Black, blackThreshold);
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                break;
            case ButtonState::blue:
                std::cout << "Calibrating blue\n";
                blueSamples.push_back(sensors->colSens.colSample);
                calculate(blueSamples, blueThreshold);
                colId->setThreshold(TileColours::Blue, blueThreshold);
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                break;
            case ButtonState::reflective:
                std::cout << "Calibrating reflective\n";
                reflectiveSamples.push_back(sensors->colSens.colSample);
                calculate(reflectiveSamples, reflectiveThreshold);
                colId->setThreshold(TileColours::Checkpoint, reflectiveThreshold);
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                break;
            case ButtonState::white:
                std::cout << "Calibrating white\n";
                whiteSamples.push_back(sensors->colSens.colSample);
                calculate(whiteSamples, whiteThreshold);
                colId->setThreshold(TileColours::White, whiteThreshold);
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                break;
            default:
                colId->registerColourSample(sensors->colSens.colSample);
                std::cout << "Tile colour: " << colId->getCurTileColour() << "\n";
                colId->clearColourSamples();
                break;
        
        }


        // If we have disconnected, leave the loop
        connected = checkActivated();
    }

}

bool ColourCalibrator::checkActivated()
{
    bool activated {false};
    activated = piAbs->digitalReadTS(PIN_CONN);
    std::this_thread::sleep_for(std::chrono::milliseconds(42));
    if (activated && piAbs->digitalReadTS(PIN_CONN))
    {
        return true;
    }
    else
    {
        return false;
    }
}

ButtonState ColourCalibrator::getButtonStates()
{
    if (piAbs->digitalReadTS(PIN_BLACK))
    {
        return ButtonState::black;
    }
    if (piAbs->digitalReadTS(PIN_BLUE))
    {
        return ButtonState::blue;
    }
    if (piAbs->digitalReadTS(PIN_RELECTIVE))
    {
        return ButtonState::reflective;
    }
    if (piAbs->digitalReadTS(PIN_WHITE))
    {
        return ButtonState::white;
    }

    return ButtonState::none;
}

bool ColourCalibrator::getIsPanelConnected()
{
    return piAbs->digitalReadTS(PIN_CONN);
}


void ColourCalibrator::calculate(std::vector<ColourSample> samples, ColourThreshold& threshold)
{
    ColourSample averageSample {calcAverage(samples)};
    threshold.setSample(averageSample);


    // Find min and max values of each type to determine good radius
    int rDiff {abs(findMaxValue(samples, fcol_r) - findMinValue(samples, fcol_r))};
    int gDiff {abs(findMaxValue(samples, fcol_g) - findMinValue(samples, fcol_g))};
    int bDiff {abs(findMaxValue(samples, fcol_b) - findMinValue(samples, fcol_b))};
    int cDiff {abs(findMaxValue(samples, fcol_c) - findMinValue(samples, fcol_c))};


    
    threshold.setRadius((rDiff+gDiff+bDiff+cDiff)/4.0); // Yes, divided by 4 gives the diameter. No, I do not know why it is like this. Yes, it worked last year.

}


int ColourCalibrator::findMinValue(std::vector<ColourSample> samples, FundamentalColour fCol)
{
    std::vector<int> bareValues {};

    for (auto& sample : samples)
    {
        bareValues.push_back(sample.values.at(fCol));
    }


    return *std::min_element(bareValues.begin(), bareValues.end());
}

int ColourCalibrator::findMaxValue(std::vector<ColourSample> samples, FundamentalColour fCol)
{
    std::vector<int> bareValues {};

    for (auto& sample : samples)
    {
        bareValues.push_back(sample.values.at(fCol));
    }

    return *std::max_element(bareValues.begin(), bareValues.end());

}



ColourSample ColourCalibrator::calcAverage(std::vector<ColourSample> samples)
{
    ColourSample resultSample {};
    int iterations {samples.size()};

    for (auto& sample : samples)
    {
        ++iterations;
        resultSample.values.at(fcol_r)+=sample.values.at(fcol_r);
        resultSample.values.at(fcol_g)+=sample.values.at(fcol_g);
        resultSample.values.at(fcol_b)+=sample.values.at(fcol_b);
        resultSample.values.at(fcol_c)+=sample.values.at(fcol_c);
    }

    resultSample.values.at(fcol_r) = static_cast<double>(resultSample.values.at(fcol_r))/static_cast<double>(iterations);
    resultSample.values.at(fcol_g) = static_cast<double>(resultSample.values.at(fcol_g))/static_cast<double>(iterations);
    resultSample.values.at(fcol_b) = static_cast<double>(resultSample.values.at(fcol_b))/static_cast<double>(iterations);
    resultSample.values.at(fcol_c) = static_cast<double>(resultSample.values.at(fcol_c))/static_cast<double>(iterations);

    return resultSample;
}
