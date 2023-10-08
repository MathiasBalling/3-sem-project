#include "PAsound.h"
#include "portaudio.h"
  // Enum for dtmf tones
  enum dtmf_tones {
    DTMF_1,
    DTMF_2,
    DTMF_3,
    DTMF_A,
    DTMF_4,
    DTMF_5,
    DTMF_6,
    DTMF_B,
    DTMF_7,
    DTMF_8,
    DTMF_9,
    DTMF_C,
    DTMF_star,
    DTMF_0,
    DTMF_hash,
    DTMF_D
  };

int main(int argc, char *argv[]) {
  int duration = 500;
  PAsound sound;
  sound.play(100, 0, duration);
  // sound.play(100, 0, duration);
  // sound.play(700, 10, duration);
  // sound.play(1300, 10, duration);
  // sound.play(697, 1209, duration); // 1
  // sound.play(697, 1336, duration); //2
  // sound.play(697, 1477, duration); //3
  // sound.play(697, 1633, duration); //A
  // sound.play(770, 1209, duration); //4
  // sound.play(770, 1336, duration); //5
  // sound.play(770, 1477, duration); //6
  // sound.play(770, 1633, duration); //sound
  // sound.play(852, 1209, duration); //7
  // sound.play(852, 1336, duration); //8
  // sound.play(852, 1477, duration); //9
  // sound.play(852, 1633, duration); //C
  // sound.play(941, 1209, duration); //*
  // sound.play(941, 1336, duration); //0
  // sound.play(941, 1477, duration); //#
  // sound.play(941, 1633, duration); //D

  sound.wait();
  return 0;
}