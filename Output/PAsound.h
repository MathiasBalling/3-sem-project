#include "portaudio.h"
#include <chrono>
#include <cstddef>
#include <iostream>
#include <math.h>
#include <ostream>
#include <queue>
#include <stdio.h>
#include <sys/types.h>
#include <vector>

// Constants
#define BUFFER_SIZE (500)
static double inputSampleRate = 10000;
static double outputSampleRate = 44100;
static float dTime = 1. / outputSampleRate;

// DTMS frequencies
struct DTMF {
  std::vector<float> One = {697, 1209};
  std::vector<float> Two = {697, 1336};
  std::vector<float> Three = {697, 1477};
  std::vector<float> A = {697, 1633};
  std::vector<float> Four = {770, 1209};
  std::vector<float> Five = {770, 1336};
  std::vector<float> Six = {770, 1477};
  std::vector<float> B = {770, 1633};
  std::vector<float> Seven = {852, 1209};
  std::vector<float> Eight = {852, 1336};
  std::vector<float> Nine = {852, 1477};
  std::vector<float> C = {852, 1633};
  std::vector<float> Star = {941, 1209};
  std::vector<float> Zero = {941, 1336};
  std::vector<float> Hash = {941, 1477};
  std::vector<float> D = {941, 1633};
};

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
  PaStream *stream;
  PaError err;
  bool isStreamActive;

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
  void wait();
  void stop();
  bool isQueueEmpty();
  std::queue<SoundObject> *getQueue();
};