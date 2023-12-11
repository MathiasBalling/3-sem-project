#include "goertzel.h"
#include "consts.h"
#include <cmath>

float goertzel_mag(int numSamples, float TARGET_FREQUENCY, int SAMPLING_RATE,
                   float *data) {
  // Calculate the coefficients
  int k = (int)(0.5 + (((float)numSamples * TARGET_FREQUENCY) /
                       (float)SAMPLING_RATE));
  float omega = (2.0 * M_PI * k) / (float)numSamples;
  float cosine = cos(omega);
  float coeff = 2.0 * cosine;
  float q0 = 0, q1 = 0, q2 = 0;

  // Run the algorithm
  for (int i = 0; i < numSamples; i++) {
    q0 = coeff * q1 - q2 + data[i];
    q2 = q1;
    q1 = q0;
  }
  float magnitude = q1 * q1 + q2 * q2 - q1 * q2 * coeff;

  return magnitude;
}

DTMF findDTMF(int numSamples, int SAMPLING_RATE, float data[]) {
  // DTMF frequencies
  float dtmfFreqMag[8];
  for (int i = 0; i < 8; i++) {
    dtmfFreqMag[i] =
        goertzel_mag(numSamples, dtmf_freqs[i], SAMPLING_RATE, data);
  }

  // Find the two largest magnitudes for low and high frequencies
  int indexLowFreqs = 0;
  int indexHighFreqs = 4;
  for (int i = 1; i < 4; i++) {
    if (dtmfFreqMag[i] > dtmfFreqMag[indexLowFreqs]) {
      indexLowFreqs = i;
    }
    if (dtmfFreqMag[i + 4] > dtmfFreqMag[indexHighFreqs]) {
      indexHighFreqs = i + 4;
    }
  }

  // Find the corresponding DTMF tone
  float meanMag =
      (dtmfFreqMag[indexLowFreqs] + dtmfFreqMag[indexHighFreqs - 4]) * 0.5;
  // Only consider the tone if the meanMag is greater than the threshold.
  if (meanMag > (float)THRESHOLD_MAG_DETECTION) {
    return DTMF(indexLowFreqs * 4 + indexHighFreqs % 4);
  }
  return DTMF::ERROR;
}
