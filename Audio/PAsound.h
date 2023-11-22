#pragma once
#include "SoundObject.h"
#include "consts.h"
#include "portaudio.h"
#include <deque>
#include <queue>

std::array<float, 2> DTMFtoFreq(DTMF dt);

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
  State getState();
  void setState(State state);
  int getSampleRate();
  float getDTime();
  void insertInputBuffer(DTMF input);
  DTMF getLastDTMF();
  std::pair<Operation, std::vector<float>> processInput();

private:
  // PortAudio variables
  PaStreamParameters outputParameters;
  PaStreamParameters inputParameters;
  PaStream *stream;
  PaError err;
  bool isStreamActive = false;
  State state = State::WAITING;
  int SampleRate = SAMPLE_RATE;
  float dTime = 1. / SampleRate;
  int duration = DURATION;
  int samples = duration * SampleRate / 1000.;
  // Queue for sound objects
  std::queue<SoundObject> soundQueue;

  // Device variables
  int outputDevice;
  int inputDevice;

  // For input
  std::deque<DTMF> inputBuffer;
  DTMF lastDTMF = DTMF::ERROR;

  std::array<float, 2> DTMFtoFreq(DTMF dt);
};
