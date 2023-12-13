#pragma once
#include "consts.h"

DTMF findDTMF(int numSamples, int SAMPLING_RATE, float data[],
              float minMagnitude);

float goertzel_mag(int numSamples, float TARGET_FREQUENCY, int SAMPLING_RATE,
                   float *data);
