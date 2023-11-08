#pragma once
#include "consts.h"
#include "portaudio.h"
#include <queue>

std::array<int, 2> DTMFtoFreq(DTMF dt);

// Sound object
struct SoundObject {
  std::vector<float> freqs;
  int samplesLeft;
  float time;
};

// Callbacks
static void StreamFinished(void *userData);
static int audioCallback(const void *inputBuffer, void *outputBuffer,
                         unsigned long framesPerBuffer,
                         const PaStreamCallbackTimeInfo *timeInfo,
                         PaStreamCallbackFlags statusFlags, void *userData);

// PAsound class
class PAsound {
  // PortAudio variables
  PaStreamParameters outputParameters;
  PaStreamParameters inputParameters;
  PaStream *stream;
  PaError err;
  bool isStreamActive = false;
  bool listening = false;

  // Queue for sound objects
  std::queue<SoundObject> soundQueue;

  // Device variables
  int outputDevice;
  int inputDevice;

public:
  PAsound();
  ~PAsound();
  void init(bool verbose = false);
  void findDevices();
  void setOutputDevice(int outDevice);
  void setInputDevice(int inDevice);
  void play(std::vector<float> freqs, int duration);
  void stop();
  bool isQueueEmpty();
  std::queue<SoundObject> *getQueue();
  bool isListening();
  void setListening(bool listen);
};
