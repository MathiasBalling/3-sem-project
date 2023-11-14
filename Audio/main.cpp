#include "PAsound.h"
#include "consts.h"
#include "protocol.h"
#include <iostream>

int main() {
  Operation op = Operation::COORDINATE;
  std::vector<float> data = {1.2, 3.4};
  PAsound sound;
  sound.init(0);
  while (1) {
    sound.play(op, data);
    std::cin.get();
  }
  /* sound.processInput(); */
  return 0;
}
