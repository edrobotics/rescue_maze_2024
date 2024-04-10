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

    tofData.b.setCur(vals[tof_b]);
    tofData.lb.setCur(vals[tof_lb]);
    tofData.lf.setCur(vals[tof_lf]);
    tofData.fl.setCur(vals[tof_fl]);
    tofData.fr.setCur(vals[tof_fr]);
    tofData.rf.setCur(vals[tof_rf]);
    tofData.rb.setCur(vals[tof_rb]);
    mtx_general.unlock();

    return retVal;
}

void Tof::printVals(bool newline)
{
    mtx_general.lock();
    std::cout << "cur: ";
    std::cout << "b=" << std::fixed << std::setprecision(1) << tofData.b.cur << "  ";
    std::cout << "lb=" << std::fixed << std::setprecision(1) << tofData.lb.cur << "  ";
    std::cout << "lf=" << std::fixed << std::setprecision(1) << tofData.lf.cur << "  ";
    std::cout << "fl=" << std::fixed << std::setprecision(1) << tofData.fl.cur << "  ";
    std::cout << "fr=" << std::fixed << std::setprecision(1) << tofData.fr.cur << "  ";
    std::cout << "rf=" << std::fixed << std::setprecision(1) << tofData.rf.cur << "  ";
    std::cout << "rb=" << std::fixed << std::setprecision(1) << tofData.rb.cur << "  ";

    std::cout << "  avg: ";
    std::cout << "b="  << std::fixed << std::setprecision(1) << tofData.b.avg << "  ";
    std::cout << "lb=" << std::fixed << std::setprecision(1) << tofData.lb.avg << "  ";
    std::cout << "lf=" << std::fixed << std::setprecision(1) << tofData.lf.avg << "  ";
    std::cout << "fl=" << std::fixed << std::setprecision(1) << tofData.fl.avg << "  ";
    std::cout << "fr=" << std::fixed << std::setprecision(1) << tofData.fr.avg << "  ";
    std::cout << "rf=" << std::fixed << std::setprecision(1) << tofData.rf.avg << "  ";
    std::cout << "rb=" << std::fixed << std::setprecision(1) << tofData.rb.avg << "  ";

    if (newline)
    {
        std::cout << "\n";
    }
    mtx_general.unlock();
}