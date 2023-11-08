#pragma once

char findDTMF(int numSamples, int SAMPLING_RATE, float data[]);

float goertzel_mag(int numSamples, float TARGET_FREQUENCY, int SAMPLING_RATE,
                   float *data);
