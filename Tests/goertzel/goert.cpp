#include "PAsound.h"
#include "consts.h"
#include "goertzel.h"
#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
  float sampleRate = 48000;
  float numSamples = 650;
  float data[(int)numSamples];
  float result_mag[8];
  std::cout << "Bin width: " << sampleRate / numSamples << std::endl;
  for (int i = 0; i < 16; i++) {
    std::array<float, 2> dtmf = DTMFtoFreq((DTMF)i);
    for (int i = 0; i < numSamples; i++) {
      // make dtmf signal
      data[i] = 0.5 * sin(2 * M_PI * dtmf[0] * i / sampleRate) +
                0.5 * sin(2 * M_PI * dtmf[1] * i / sampleRate);
    }

    for (int i = 0; i < 8; i++) {
      result_mag[i] = goertzel_mag(numSamples, dtmf_freqs[i], sampleRate, data);
    }
    // Normalize the magnitude
    int max = 0;
    for (int i = 0; i < 8; i++) {
      if (result_mag[i] > max) {
        max = result_mag[i];
      }
    }
    /* std::cout << "DTMF: " << dtmf[0] << " " << dtmf[1] << std::endl; */
    std::cout << std::fixed;
    std::cout << std::setprecision(2);
    for (int i = 0; i < 8; i++) {
      result_mag[i] = result_mag[i] / max;
      std::cout << result_mag[i] << " ";
    }
    std::cout << std::endl;
  }
  return 0;
}
