#include "portaudio.h"
#include <math.h>
#include <stdio.h>

#define SAMPLE_RATE (44100)
#define FRAMES_PER_BUFFER (64)

class PAsound {
  PaStreamParameters outputParameters;
  PaStream *stream;
  PaError err;

  struct paData {
    float sine[1000];
    int phase;
  };
  paData data;

  static int audioCallback(const void *inputBuffer, void *outputBuffer,
                           unsigned long framesPerBuffer,
                           const PaStreamCallbackTimeInfo *timeInfo,
                           PaStreamCallbackFlags statusFlags, void *userData);
  void StreamFinished(void *userData);
  void makeSineWave(float freq1, float freq2);
  void generateSamples(float *buffer, int numSamples);
public:
  PAsound();
  ~PAsound();
  void playtest();
  void play(float freq1, float freq2, int duration);
  void wait();
};