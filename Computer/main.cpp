#include "PAsound.h"
#include "consts.h"
#include "protocol.h"

int main() {
  /* printData(dataEncode(2.5), 2); */
  /* printData(dataEncode(2.5), 10); */
  auto out = dataToDTMF(Operation::FORWARD, {12.5, 25.25});
  std::deque<DTMF> outqueue;
  for (auto i : out) {
    outqueue.push_back(i);
    std::cout << indexToDtmf[(int)i] << " ";
  }
  std::cout << std::endl;
  auto in = DTMFtoData(outqueue);
  std::cout << (int)in.first << std::endl;
  for (auto i : in.second) {
    std::cout << i << std::endl;
  }
  return 0;
}
