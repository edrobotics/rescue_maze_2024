#pragma once
#include <mutex>

// Mutexes for reading and writing withing fusion threads
std::mutex mtx_transData_freqData;
std::mutex mtx_transData_controlData;
std::mutex mtx_transData_infreqData;