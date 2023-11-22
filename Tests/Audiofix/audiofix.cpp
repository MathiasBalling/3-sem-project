#include "PAsound.h"
#include "consts.h"
#include "protocol.h"
#include <chrono>
#include <iostream>
#include <thread>

int main(int argc, char **argv) {
  PAsound sound;
  sound.init(0);
  sound.setState(State::SENDING);
  sound.play(DTMF::ZERO);
  sound.play(DTMF::ONE);
  sound.play(DTMF::TWO);
  sound.play(DTMF::THREE);
  sound.play(DTMF::FOUR);
  sound.play(DTMF::FIVE);
  sound.play(DTMF::SIX);
  sound.play(DTMF::SEVEN);
  sound.play(DTMF::EIGHT);
  sound.play(DTMF::NINE);
  sound.play(DTMF::A);
  sound.play(DTMF::B);
  sound.play(DTMF::C);
  sound.play(DTMF::D);
  sound.play(DTMF::WALL);
  sound.play(DTMF::DIVIDE);

  std::this_thread::sleep_for(std::chrono::seconds(1));
  return 0;
}
