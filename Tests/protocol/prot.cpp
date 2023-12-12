#include "consts.h"
#include "protocol.h"
#include <cstddef>
#include <iostream>
#include <queue>

int main() {
  std::vector<float> data = {0.22, 2.84};
  std::vector<DTMF> dtmf = dataToDTMF(Operation::MOVEMENT, data);
  std::queue<DTMF> sampleInput;
  for (size_t i = 0; i < dtmf.size(); i++) {
    sampleInput.push(dtmf[i]);
    std::cout << (int)dtmf[i] << " ";
  }

  std::cout << std::endl;
  auto res = DTMFtoData(sampleInput);
  std::cout << indexToOperation[(int)getOperation(res)] << std::endl;

  return 0;
}
