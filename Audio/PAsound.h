#pragma once
#include "consts.h"
#include "portaudio.h"
#include <deque>
#include <queue>

std::array<float, 2> DTMFtoFreq(DTMF dt);

// Sound object
struct SoundObject {
  std::array<float, 2> freqs;
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
public:
  PAsound();
  ~PAsound();
  void init(bool verbose = false);
  void findDevices();
  void play(Operation op, std::vector<float> data);
  void stop();
  bool isQueueEmpty();
  std::queue<SoundObject> *getQueue();
  bool isListening();
  void setListening(bool listen);
  int getSampleRate();
  float getDTime();
  void insertInputBuffer(char input);
  char getLastChar();
  /* std::pair() processInput(); */

private:
  // PortAudio variables
  PaStreamParameters outputParameters;
  PaStreamParameters inputParameters;
  PaStream *stream;
  PaError err;
  bool isStreamActive = false;
  bool listening = false;

  int SampleRate = 44800;
  float dTime = 1. / SampleRate;
  int duration = 50;
  // Queue for sound objects
  std::queue<SoundObject> soundQueue;

  // Device variables
  int outputDevice;
  int inputDevice;

  // For input
  std::deque<char> inputBuffer;
  char lastChar = -1;
};
