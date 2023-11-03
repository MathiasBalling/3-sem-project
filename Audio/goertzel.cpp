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
char findDTMF(int numSamples, int SAMPLING_RATE, float *data) {
  // DTMF frequencies
  float dtmf_mag[8];
  for (int i = 0; i < 8; i++) {
    dtmf_mag[i] = goertzel_mag(numSamples, dtmf_freqs[i], SAMPLING_RATE, data);
  }

  // Find the two largest magnitudes
  int index_low_freqs = 0;
  int index_high_freqs = 4;
  for (int i = 1; i < 4; i++) {
    if (dtmf_mag[i] > dtmf_mag[index_low_freqs]) {
      index_low_freqs = i;
    }
    if (dtmf_mag[i + 4] > dtmf_mag[index_high_freqs]) {
      index_high_freqs = i + 4;
    }
  }

  // Find the corresponding key
  char dtmf_char = indexToDtmf[index_low_freqs % 4 + index_high_freqs / 4 * 4];
  float mean = (dtmf_mag[index_low_freqs] + dtmf_mag[index_high_freqs - 4]) / 2;
  if (mean > 10) {
    return dtmf_char;
  }
  return -1;
}