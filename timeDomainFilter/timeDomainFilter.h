#ifndef TIME_DOMAIN_FILTER_H
#define TIME_DOMAIN_FILTER_H

#include <stdio.h>
#include <stdlib.h>
#include "arm_math.h"

int cmp_float(const void* e1, const void* e2);
void amplitudeLimitingFilter(float* getValue, float amp, uint16_t array_size);
float medianValueFilter(float* samples, uint16_t array_size);
float arithmeticAverageFilter(float* samples, uint16_t array_size);
float geometricMeanFilter(float* samples, uint16_t array_size);
float firstOrderLagFilter(float* data_buffer, float a, uint16_t array_size);
float recursiveMovingAverageFilter(float* data_buffer, uint16_t window_size, uint16_t array_size);
float amplitudeLimitedMovingAverage(float* data_buffer, float amp, uint16_t window_size, uint16_t array_size);
float debounceFilter(float* samples, uint16_t threshold, uint16_t array_size);
void kalmanInit(KalmanFilter* kf, float init_x, float init_P, float process_noise, float measure_noise);
float kalmanUpdate(KalmanFilter* kf, float samples);

#endif