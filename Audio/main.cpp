#include "PAsound.h"
#include "protocol.h"
#include <iostream>

int main() {
  PAsound sound;
  sound.init(0);
  sound.play({697, 1209}, 1000);
  std::cin.get();
  sound.play({697, 1336}, 1000);
  std::cin.get();
  sound.setListening(1);

  std::cin.get();
  return 0;
}
