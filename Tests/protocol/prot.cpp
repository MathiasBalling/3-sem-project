#include "consts.h"
#include "protocol.h"
#include <deque>
#include <iostream>

int main() {
  std::vector<float> data = {0, 0};
  std::vector<DTMF> dtmf = dataToDTMF(Operation::MOVEMENT, data);
  std::deque<DTMF> input = {};
  for (int i = 0; i < dtmf.size(); i++) {
    std::cout << (int)dtmf[i] << " ";
    input.push_back(dtmf[i]);
  }
  std::pair<Operation, std::vector<float>> in = DTMFtoData(input);
  std::cout << "Operation: " << (int)in.first << std::endl;
  for (int i = 0; i < in.second.size(); i++) {
    std::cout << in.second[i] << " ";
  }
  return 0;
}
