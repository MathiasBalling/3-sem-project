#include "PAsound.h"

int main(int argc, char *argv[]) {
  DTMF dtmf;
  PAsound sound;
  sound.init(0);
  int duration = 100;
  sound.play(dtmf.One, duration);   // 1
  sound.play(dtmf.Two, duration);   // 2
  sound.play(dtmf.Three, duration); // 3
  sound.play(dtmf.A, duration);     // *
  sound.play(dtmf.Four, duration);  // A
  sound.play(dtmf.Five, duration);  // 4
  sound.play(dtmf.Six, duration);   // 5
  sound.play(dtmf.B, duration);     // 0
  sound.play(dtmf.Seven, duration); // 6
  sound.play(dtmf.Eight, duration); // B
  sound.play(dtmf.Nine, duration);  // 7
  sound.play(dtmf.C, duration);     // #
  sound.play(dtmf.Star, duration);  // C
  sound.play(dtmf.Zero, duration);  // 8
  sound.play(dtmf.Hash, duration);  // 9
  sound.play(dtmf.D, duration);     // D
  sound.wait();
  return 0;
}
