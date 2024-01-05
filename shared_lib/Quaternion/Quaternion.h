#pragma once

struct Quaternion
{
    enum Terms
        {
            term_real,
            term_i,
            term_j,
            term_k,
            term_num,
        };

    float floats[term_num] {0};   
};