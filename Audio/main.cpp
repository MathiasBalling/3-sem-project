#include "consts.h"

std::array<int, 2> DTMFtoFreq(DTMF dt) {
  std::array<int, 2> result;
  result[0] = dtmf_freqs[dt % 4];
  result[1] = dtmf_freqs[dt / 4 + 4];
  return result;
};

int main() {
  std::array<int, 2> result;
  result = DTMFtoFreq(DTMF::start);
  printf("%d %d\n", result[0], result[1]);
  return 0;
}
