#include "portaudio.h"
#include <math.h>
#include <queue>
#include <stdio.h>
#include <sys/types.h>

#define SAMPLE_RATE (44100)
#define BUFFER_SIZE (1024)

struct SoundObject {
  double freq1;
  double freq2;
  int samplesLeft;
  float time;
};

// Callbacks
static void StreamFinished(void *userData);
static int audioCallback(const void *inputBuffer, void *outputBuffer,
                         unsigned long framesPerBuffer,
                         const PaStreamCallbackTimeInfo *timeInfo,
                         PaStreamCallbackFlags statusFlags, void *userData);

class PAsound {
  // PortAudio variables
  PaStreamParameters outputParameters;
  PaStream *stream;
  PaError err;

  // Queue for sound objects
  std::queue<SoundObject> soundQueue;

public:
  PAsound();
  ~PAsound();
  void play(float freq1, float freq2, int duration);
  void wait();
  bool isQueueEmpty();
  std::queue<SoundObject> *getQueue();
};