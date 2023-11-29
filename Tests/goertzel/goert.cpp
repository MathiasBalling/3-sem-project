#include "consts.h"
#include "goertzel.h"
#include <cmath>
#include <iostream>

int main() {
  int sampleRate = 48000;
  int numSamples = 500;
  float data[numSamples];
  for (int i = 0; i < numSamples; i++) {
    // make dtmf signal
    data[i] = 0.5 * sin(2 * M_PI * 697 * i / sampleRate) +
              0.5 * sin(2 * M_PI * 1209 * i / sampleRate);
  }

  DTMF res = findDTMF(500, 48000, data);
  std::cout << "DTMF= " << (int)res << std::endl;
  return 0;
}
