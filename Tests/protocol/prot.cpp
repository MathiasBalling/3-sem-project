#include "consts.h"
#include "protocol.h"
#include <iostream>

int main() {
  std::vector<float> data = {0.22, 2.84};
  std::vector<DTMF> dtmf = dataToDTMF(Operation::MOVEMENT, data);
  for (int i = 0; i < dtmf.size(); i++) {
    std::cout << (int)dtmf[i] << " ";
  }
  return 0;
}
