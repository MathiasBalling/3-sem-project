#include "PAsound.h"
#include "consts.h"
#include "goertzel.h"
#include "protocol.h"
#include <cmath>
#include <iostream>

PAsound::PAsound() {
  // PortAudio Initialization
  Pa_Initialize();
  std::cout << "PortAudio initialized" << std::endl;
  m_outputDevice = Pa_GetDefaultOutputDevice();
  m_inputDevice = Pa_GetDefaultInputDevice();
}

PAsound::~PAsound() {
  Pa_CloseStream(m_stream);
  Pa_Terminate();
}

void PAsound::setState(State newState) { m_state = newState; }

State PAsound::getState() { return m_state; }

int PAsound::getSampleRate() { return m_sampleRate; }

float PAsound::getDTime() { return m_dTime; }

void PAsound::insertInputBuffer(DTMF input) {
  m_inputBuffer.push_back(input);
  m_lastDTMF = input;
}

DTMF PAsound::getLastDTMF() { return m_lastDTMF; }

std::array<float, 2> PAsound::DTMFtoFreq(DTMF dt) {
  std::array<float, 2> result;
  result[0] = dtmf_freqs[(int)dt / 4];
  result[1] = dtmf_freqs[(int)dt % 4 + 4];
  return result;
};

void PAsound::findDevices() {
  int numDevices = Pa_GetDeviceCount();
  std::cout << std::endl;
  const PaDeviceInfo *deviceInfo;
  for (int i = 0; i < numDevices; i++) {
    deviceInfo = Pa_GetDeviceInfo(i);
    std::cout << "Device " << i << ": " << deviceInfo->name << std::endl;
    std::cout << "Max output channels: " << deviceInfo->maxOutputChannels
              << std::endl;
    std::cout << "Max input channels: " << deviceInfo->maxInputChannels
              << std::endl;
    std::cout << "Default sample rate: " << deviceInfo->defaultSampleRate
              << std::endl;
    std::cout << std::endl;
  }
}

void PAsound::init(bool verbose, State state) {
  if (verbose) {
    findDevices();
    int input;
    std::cout << "Default output device: " << m_outputDevice << std::endl;
    std::cout << "Enter output device (-1 for default): ";
    std::cin >> input;
    if (input != -1) {
      m_outputDevice = input;
    }
    std::cout << std::endl;
    std::cout << "Default sample rate: "
              << Pa_GetDeviceInfo(m_outputDevice)->defaultSampleRate
              << std::endl;
    std::cout << "Enter sample rate (-1 for default): ";
    std::cin >> input;
    if (input != -1) {
      m_sampleRate = input;
    } else {
      m_sampleRate = Pa_GetDeviceInfo(m_outputDevice)->defaultSampleRate;
    }
    std::cout << std::endl;
    std::cout << "Default input device: " << m_inputDevice << std::endl;
    std::cout << "Enter input device (-1 for default): ";
    std::cin >> input;
    if (input != -1) {
      m_inputDevice = input;
    }
    std::cout << std::endl;
  }
  m_state = state;
  // Set up output m_stream
  m_outputParameters.device = m_outputDevice;
  if (m_outputParameters.device == paNoDevice) {
    std::cout << "Error: No default output device." << std::endl;
  }
  m_outputParameters.channelCount = 1; /* mono output */
  m_outputParameters.sampleFormat =
      paFloat32; /* 32 bit floating point output */
  m_outputParameters.suggestedLatency =
      Pa_GetDeviceInfo(m_outputParameters.device)->defaultLowOutputLatency;
  m_outputParameters.hostApiSpecificStreamInfo = NULL;
  // Set up input m_stream
  m_inputParameters.device = m_inputDevice;
  if (m_inputParameters.device == paNoDevice) {
    std::cout << "Error: No default input device." << std::endl;
  }
  m_inputParameters.channelCount = 1;         /* mono input */
  m_inputParameters.sampleFormat = paFloat32; /* 32 bit floating point input */
  m_inputParameters.suggestedLatency =
      Pa_GetDeviceInfo(m_inputParameters.device)->defaultLowInputLatency;
  m_inputParameters.hostApiSpecificStreamInfo = NULL;
  // Open m_stream
  m_err =
      Pa_OpenStream(&m_stream, &m_inputParameters, &m_outputParameters,
                    m_sampleRate, BUFFER_SIZE, paClipOff, audioCallback, this);
  if (m_err != paNoError) {
    std::cout << "Error: " << Pa_GetErrorText(m_err) << std::endl;
  }
  m_err = Pa_StartStream(m_stream);
  if (m_err != paNoError) {
    std::cout << "Error: " << Pa_GetErrorText(m_err) << std::endl;
  }
}

bool PAsound::isQueueEmpty() { return m_soundQueue.empty(); }

std::queue<SoundObject> *PAsound::getQueue() { return &m_soundQueue; }

void PAsound::play(Operation op, std::vector<float> data, int duration) {
  std::vector<DTMF> send = dataToDTMF(op, data);
  for (auto dtmf : send) {
    std::array<float, 2> freqs = DTMFtoFreq(dtmf);
    int samples = duration * m_sampleRate / 1000;
    SoundObject sound({freqs, samples, 0, 0});
    m_soundQueue.push(sound);
  }
}

void PAsound::play(DTMF dtmf, int duration) {
  std::array<float, 2> freqs = DTMFtoFreq(dtmf);
  int samples = duration * m_sampleRate / 1000;
  SoundObject sound({freqs, samples, 0, 0});
  m_soundQueue.push(sound);
}
void PAsound::stop() {
  while (!m_soundQueue.empty())
    m_soundQueue.pop();
}

std::pair<Operation, std::vector<float>> PAsound::processInput() {
  std::pair<Operation, std::vector<float>> res = DTMFtoData(m_inputBuffer);
  m_inputBuffer.clear();
  return res;
}

int audioCallback(const void *m_inputBuffer, void *outputBuffer,
                  unsigned long framesPerBuffer,
                  const PaStreamCallbackTimeInfo *timeInfo,
                  PaStreamCallbackFlags statusFlags, void *userData) {
  float *out = (float *)outputBuffer;
  /* Prevent unused variable warning. */
  (void)timeInfo;
  (void)statusFlags;
  PAsound *sound = (PAsound *)userData;
  DTMF dtmf =
      findDTMF(framesPerBuffer, sound->getSampleRate(), (float *)m_inputBuffer);
  switch (sound->getState()) {
  case State::WAITING: {
    dtmf = findDTMF(framesPerBuffer, sound->getSampleRate(),
                    (float *)m_inputBuffer);
    if (dtmf == DTMF::WALL) {
      sound->insertInputBuffer(dtmf);
      sound->setState(State::LISTENING);
    }
    break;
  }
  case State::LISTENING: {
    dtmf = findDTMF(framesPerBuffer, sound->getSampleRate(),
                    (float *)m_inputBuffer);
    if (sound->getLastDTMF() == dtmf)
      break;
    switch (dtmf) {
    case DTMF::ERROR:
      break;
    case DTMF::WALL:
      sound->insertInputBuffer(dtmf);
      sound->setState(State::PROCESSING);
      break;
    default:
      sound->insertInputBuffer(dtmf);
      break;
    }
    break;
  }
  case State::SENDING: {
    // Fill the buffer with silence if the queue is empty
    if (sound->isQueueEmpty()) {
      for (int i = 0; i < framesPerBuffer; i++)
        *out++ = 0;
      return paContinue;
    }
    SoundObject &soundObject = sound->getQueue()->front();
    unsigned long i;
    // FIX: Match the amplitude of the concurrent sounds
    for (i = 0; i < framesPerBuffer; i++) {
      if (soundObject.samplesLeft == 0) {
        sound->getQueue()->pop();
        if (sound->isQueueEmpty()) {
          *out++ = 0;
          return paContinue;
        }
        soundObject = sound->getQueue()->front();
      }
      float sample = 0;
      for (auto freq : soundObject.freqs) {
        sample += (float)(sin(freq * soundObject.time * 2 * M_PI) * 1. /
                          soundObject.freqs.size());
      }
      // Fade out the sound when it is about to end
      float fade = 200;
      if (soundObject.samplesLeft < fade)
        sample *= soundObject.samplesLeft / fade;
      if (soundObject.samplesDone < fade)
        sample *= soundObject.samplesDone / fade;
      *out++ = sample;
      soundObject.time += sound->getDTime();
      soundObject.samplesLeft--;
      soundObject.samplesDone++;
    }

    break;
  }
  case State::PROCESSING: {
    break;
  }
  default: {
    break;
  }
  }

  return paContinue;
}
