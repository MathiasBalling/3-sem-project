#include "consts.h"
#include "protocol.h"
#include <cstddef>
#include <iostream>
#include <queue>

int main() {
  std::vector<float> data = {-0.22, 0.22};
  std::string dataString = "Mathias Balling Christiansen";
  std::vector<DTMF> dtmf = dataToDTMF(Operation::MOVEMENT, data);
  std::queue<DTMF> sampleInput;
  for (size_t i = 0; i < dtmf.size(); i++) {
    sampleInput.push(dtmf[i]);
    std::cout << (int)dtmf[i] << " ";
  }

  std::cout << std::endl;
  auto res = DTMFtoData(sampleInput);
  for (auto data : res) {
    std::cout << data << " ";
  }
  std::cout << indexToOperation[(int)getOperation(res)] << std::endl;
  if (getOperation(res) == Operation::MOVEMENT) {
    auto resfloat = dataFloatDecode(res);
    for (auto data : resfloat) {
      std::cout << data << std::endl;
    }
  } else if (getOperation(res) == Operation::STRING) {
    auto resstring = dataStringDecode(res);
    std::cout << resstring << std::endl;
  }

  /* std::vector<unsigned int> test{5}; */
  /* test.push_back(14); */
  /* auto test2 = dataBitsToDTMF(test); */
  /* std::queue<DTMF> sampleInput2; */
  /* for (auto data : test2) { */
  /*   sampleInput2.push(data); */
  /*   std::cout << (int)data << " "; */
  /* } */
  /* std::cout << std::endl; */
  /* DTMFtoData(sampleInput2); */

  return 0;
}
