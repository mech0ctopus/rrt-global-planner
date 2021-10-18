#pragma once

#include <math.h>

/**
 *  @brief Produces a random double in range.
 *
 *  @details
 *   Produces a random double in range (inclusive).
 *
 *  @see   https://stackoverflow.com/questions/2704521/generate-random-double-numbers-in-c
 *
 *  @param fMin Minimum possible value.
 *  @param fMax Maximum possible value.
 *  @return     Random double in [fMin, fMax].
 *
 */
double randomDouble(double fMin, double fMax)
{
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}