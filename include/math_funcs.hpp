#pragma once

#include <math.h>

/**
 * Produce random double in range.
 * 
 * https://stackoverflow.com/questions/2704521/generate-random-double-numbers-in-c
 */
double randomDouble(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}