#pragma once

#include <vector>
#include <stdexcept>

class ConditionalAverageTerm
{
    public:
    ConditionalAverageTerm(double value, double weight);
    // The value
    double value {};
    // Weight in calculation. 1 is standard. 0 means do not use.
    double weight {1};
};


class Average
{
    public:
    std::vector<ConditionalAverageTerm> terms {};

    // Calculates the average given the terms
    // If no terms are set to calculate (through their weight or lack of terms), throws runtime_error exception
    double calc(std::vector<ConditionalAverageTerm> terms);
    double calc();

    private:

};