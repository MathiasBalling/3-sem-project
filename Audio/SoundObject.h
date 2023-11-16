#pragma once
#include <array>

// Sound object
struct SoundObject {
  std::array<float, 2> freqs;
  int samplesLeft;
  float time;
};
