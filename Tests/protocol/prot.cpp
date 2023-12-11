#include "consts.h"
#include "protocol.h"
#include <cstddef>
#include <iostream>
#include <queue>

int main() {
  std::vector<float> data = {0.22, 2.84};
  std::vector<DTMF> dtmf = dataToDTMF(Operation::FORWARD);
  std::queue<DTMF> output;
  for (size_t i = 0; i < dtmf.size(); i++) {
    output.push(dtmf[i]);
    std::cout << (int)dtmf[i] << " ";
  }

  return 0;
}
