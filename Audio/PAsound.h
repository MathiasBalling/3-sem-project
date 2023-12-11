#pragma once
#include "SoundObject.h"
#include "consts.h"
#include "portaudio.h"
#include <queue>

std::array<float, 2> DTMFtoFreq(DTMF dt);

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

  // For input
  void insertInputBuffer(DTMF input);
  DTMF getLastInput() const;
  DTMF getLastDTMF() const;
  void setLastDTMF(DTMF dtmf);
  int getLastDTMFCount() const;
  void setLastDTMFCount(int count);
  std::pair<Operation, std::vector<int>> processInput();

private:
  // PortAudio variables
  PaStreamParameters m_outputParameters;
  PaStreamParameters m_inputParameters;
  PaStream *m_stream;
  PaError m_err;
  State m_state = State::WAITING;
  int m_sampleRate = SAMPLE_RATE;
  float m_dTime = 1. / m_sampleRate;
  // Queue for sound objects
  std::queue<SoundObject> m_soundQueue;

  // Device variables
  int m_outputDevice;
  int m_inputDevice;

  // For input
  std::queue<DTMF> m_inputBuffer;
  DTMF m_lastDTMF = DTMF::ERROR;
  int m_lastDTMFCount = 0;
};
