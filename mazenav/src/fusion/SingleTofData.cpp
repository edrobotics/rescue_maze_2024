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
    // std::cout << "historyIndex: " << historyIndex << "\n";
    // Update index
    incrementHistoryIndex();
    // Index should now be one ahead of the newest added value = on the oldest value

    int sum {0};

    for (int i=0; i<HISTORY_NUM-HISTORY_EXCLUSION_NUM; ++i)
    {
        sum += history.at(wrapIndex(historyIndex+i));
    }

    // for (auto& value : history)
    // {
    //     sum+=value;
    // }
    avg = static_cast<double>(sum)/static_cast<double>(HISTORY_NUM-HISTORY_EXCLUSION_NUM);
}


void SingleTofData::incrementHistoryIndex()
{
    ++historyIndex;
    // Wrap to range: 0 - (HISTORY_NUM-1)
    historyIndex = wrapIndex(historyIndex);
    // std::cout << "historyIndex: " << historyIndex << "\n";
}

int SingleTofData::wrapIndex(int index)
{
    return index % HISTORY_NUM;
}