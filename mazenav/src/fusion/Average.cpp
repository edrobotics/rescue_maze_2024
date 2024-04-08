#include "fusion/Average.h"

ConditionalAverageTerm::ConditionalAverageTerm(double value, double weight)
{
    this->value = value;
    this->weight = weight;
}


double Average::calc(std::vector<ConditionalAverageTerm> terms)
{
    double value {0};
    double totalWeight {0};
    for (auto& term : terms)
    {
        if (term.weight > 0)
        {
            value += term.value*term.weight;
            totalWeight += term.weight;
        }
    }

    if (totalWeight==0)
    {
        throw std::runtime_error {"Division by 0"};
    }

    double result {value/totalWeight};
    return result;
}

double Average::calc()
{
    return calc(terms);
}