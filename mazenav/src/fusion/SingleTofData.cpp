#include "fusion/SingleTofData.h"



void SingleTofData::setCur(int newVal)
{
    cur = newVal;
    updateAvg(newVal);
}


void SingleTofData::updateAvg(int newVal)
{
    // Change value
    history.at(historyIndex) = newVal;
    // Update index
    incrementHistoryIndex();

    int sum {0};

    for (auto& value : history)
    {
        sum+=value;
    }
    avg = static_cast<double>(sum)/static_cast<double>(HISTORY_NUM);
}


void SingleTofData::incrementHistoryIndex()
{
    ++historyIndex;
    // Wrap to range: 0 - (HISTORY_NUM-1)
    historyIndex = historyIndex % HISTORY_NUM;
}