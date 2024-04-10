#include "fusion/Tof.h"

Tof::Tof(TeensyCommunicator* communicator)
{
    this->communicator = communicator;
}

bool Tof::updateVals()
{
    mtx_general.lock();
    // Read vals
    bool retVal {communicator->transData.tsGetTof(vals)};

    tofData.b = vals[tof_b];
    tofData.lb = vals[tof_lb];
    tofData.lf = vals[tof_lf];
    tofData.fl = vals[tof_fl];
    tofData.fr = vals[tof_fr];
    tofData.rf = vals[tof_rf];
    tofData.rb = vals[tof_rb];
    mtx_general.unlock();

    return retVal;
}

void Tof::printVals(bool newline)
{
    mtx_general.lock();
    std::cout << "b=" << std::fixed << std::setprecision(1) << tofData.b << "  ";
    std::cout << "lb=" << std::fixed << std::setprecision(1) << tofData.lb << "  ";
    std::cout << "lf=" << std::fixed << std::setprecision(1) << tofData.lf << "  ";
    std::cout << "fl=" << std::fixed << std::setprecision(1) << tofData.fl << "  ";
    std::cout << "fr=" << std::fixed << std::setprecision(1) << tofData.fr << "  ";
    std::cout << "rf=" << std::fixed << std::setprecision(1) << tofData.rf << "  ";
    std::cout << "rb=" << std::fixed << std::setprecision(1) << tofData.rb << "  ";
    if (newline)
    {
        std::cout << "\n";
    }
    mtx_general.unlock();
}