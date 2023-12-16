#include "PAsound.h"
#include "consts.h"
#include "goertzel.h"
#include "protocol.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <queue>
#include <vector>

int main() {
  float sampleRate = 48000;
  float result_mag[8];
  for (float size = 300; size < 1500; size += 1) {
    int countCorrect = 0;
    float sum = 0;
    for (int i = 0; i < 16; i++) {
      float data[(int)size];
      std::array<float, 2> dtmf = DTMFtoFreq((DTMF)i);
      for (int i = 0; i < size; i++) {
        // make dtmf signal
        data[i] = 0.5 * sin(2 * M_PI * dtmf[0] * i / sampleRate) +
                  0.5 * sin(2 * M_PI * dtmf[1] * i / sampleRate);
      }
      for (int i = 0; i < 8; i++) {
        result_mag[i] = goertzel_mag(size, dtmf_freqs[i], sampleRate, data);
      }
      // Normalize the magnitude
      int maxLow = 0;
      int maxHigh = 4;
      float tempSumLow = 0;
      float tempSumHigh = 0;
      for (int i = 1; i < 4; i++) {
        if (result_mag[i] > result_mag[maxLow]) {
          maxLow = i;
        }
        if (result_mag[i + 4] > result_mag[maxHigh]) {
          maxHigh = i + 4;
        }
      }
      int index = maxLow * 4 + maxHigh % 4;
      if (index == i) {
        countCorrect++;
      }

      float maxMagLow = result_mag[maxLow];
      float maxMagHigh = result_mag[maxHigh];
      for (int i = 0; i < 4; i++) {
        result_mag[i] /= maxMagLow;
        tempSumLow += result_mag[i];
        result_mag[i + 4] /= maxMagHigh;
        tempSumHigh += result_mag[i + 4];
      }
      tempSumLow -= result_mag[maxLow];
      tempSumHigh -= result_mag[maxHigh];
      sum += tempSumHigh + tempSumLow;
    }
    if (countCorrect == 16) {
      std::cout << "Bin width: " << sampleRate / size << std::endl;
      std::cout << "Size: " << size << " Correct: " << countCorrect
                << "  Average magnitude of not max: " << sum / 16 << std::endl;
      std::cout << std::endl;
    }
  }
  /* std::vector<float> data = {(float)70}; */
  /* auto dtmftones = dataToDTMF(Operation::UPDATE_MAG_THRESHOLD, data); */
  /* std::queue<DTMF> result; */
  /* for (auto dtmf : dtmftones) { */
  /*   std::cout << indexToDtmf[(int)dtmf]; */
  /*   result.push(dtmf); */
  /* } */
  /* std::cout << std::endl; */
  /* auto res = DTMFtoData(result); */
  /* auto op = getOperation(res); */
  return 0;
}
