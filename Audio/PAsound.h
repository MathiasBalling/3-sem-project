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
  bool init(bool verbose = false, State state = State::WAITING);
  void findDevices();
  void play(Operation op, std::vector<float> data = {},
            int duration = (int)DURATION_MS);
  void play(DTMF dtmf, int duration = (int)DURATION_MS);
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
  PaStreamParameters m_outputParameters;
  PaStreamParameters m_inputParameters;
  PaStream *m_stream;
  PaError m_err;
  bool m_isStreamActive = false;
  State m_state = State::WAITING;
  int m_sampleRate = SAMPLE_RATE;
  float m_dTime = 1. / m_sampleRate;
  // Queue for sound objects
  std::queue<SoundObject> m_soundQueue;

  // Device variables
  int m_outputDevice;
  int m_inputDevice;

  // For input
  std::deque<DTMF> m_inputBuffer;
  DTMF m_lastDTMF = DTMF::ERROR;

  std::array<float, 2> DTMFtoFreq(DTMF dt);
};
