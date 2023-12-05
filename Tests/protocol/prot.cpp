#include "consts.h"
#include "protocol.h"
#include <deque>
#include <iostream>

int main() {
  std::vector<float> data = {0.22, 2.84};
  std::vector<DTMF> dtmf = dataToDTMF(Operation::BACKWARD, data);
  std::deque<DTMF> output;
  for (int i = 0; i < dtmf.size(); i++) {
    output.push_back(dtmf[i]);
    std::cout << (int)dtmf[i] << " ";
  }

  auto res = DTMFtoData(output);
  std::cout << "\nOperation: " << indexToOperation[(int)res.first] << "\n";
  /* << "Data: " << res.second[0] << " " << res.second[1] << std::endl; */

  return 0;
}
