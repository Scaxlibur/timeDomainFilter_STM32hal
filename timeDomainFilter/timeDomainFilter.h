#ifndef TIME_DOMAIN_FILTER_H
#define TIME_DOMAIN_FILTER_H

#include <stdio.h>
#include <stdlib.h>
#include "arm_math.h"

int cmp_float(const void* e1, const void* e2);
void amplitude_limiting_filter(float* getValue, float amp);
float medianValueFilter(float* getValue);
float arithmeticAverageFilter(float* getValue);
float geometricMeanfilter(float* samples);
float firstOrderLagFilter(float* data_buffer, float a);
float recursiveMovingAverageFilter(float* data_buffer, uint16_t window_size);
float amplitudeLimitedMovingAverage(float* data_buffer, float amp, uint16_t window_size);
float debounce_filter(float* samples, uint16_t threshold);

#endif