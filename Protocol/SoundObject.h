#pragma once
#include <array>

// Sound object
struct SoundObject {
  // Frequency of DTMF tone
  std::array<float, 2> freqs;
  // Used to make fade and duration
  int samplesLeft;
  int samplesDone;
  // Used to make the sound
  float time;
};
