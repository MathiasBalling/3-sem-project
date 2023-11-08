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
  outputDevice = Pa_GetDefaultOutputDevice();
  inputDevice = Pa_GetDefaultInputDevice();
}

PAsound::~PAsound() {
  Pa_CloseStream(stream);
  Pa_Terminate();
}

void PAsound::setListening(bool listen) { listening = listen; }

bool PAsound::isListening() { return listening; }

int PAsound::getSampleRate() { return SampleRate; }

float PAsound::getDTime() { return dTime; }

void PAsound::insertInputBuffer(char input) { inputBuffer.push_back(input); }

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

void PAsound::init(bool verbose) {
  if (verbose) {
    findDevices();
    int input;
    std::cout << "Default output device: " << outputDevice << std::endl;
    std::cout << "Enter output device (-1 for default): ";
    std::cin >> input;
    if (input != -1) {
      outputDevice = input;
    }
    std::cout << std::endl;
    std::cout << "Default sample rate: "
              << Pa_GetDeviceInfo(outputDevice)->defaultSampleRate << std::endl;
    std::cout << "Enter sample rate (-1 for default): ";
    std::cin >> input;
    if (input != -1) {
      SampleRate = input;
    } else {
      SampleRate = Pa_GetDeviceInfo(outputDevice)->defaultSampleRate;
    }
    std::cout << std::endl;

    std::cout << "Default input device: " << inputDevice << std::endl;
    std::cout << "Enter input device (-1 for default): ";
    std::cin >> input;
    if (input != -1) {
      inputDevice = input;
    }
    std::cout << std::endl;
  }

  // Set up output stream
  outputParameters.device = outputDevice;

  if (outputParameters.device == paNoDevice) {
    std::cout << "Error: No default output device." << std::endl;
  }
  outputParameters.channelCount = 1;         /* mono output */
  outputParameters.sampleFormat = paFloat32; /* 32 bit floating point output */
  outputParameters.suggestedLatency =
      Pa_GetDeviceInfo(outputParameters.device)->defaultLowOutputLatency;
  outputParameters.hostApiSpecificStreamInfo = NULL;

  // Set up input stream
  inputParameters.device = inputDevice;
  if (inputParameters.device == paNoDevice) {
    std::cout << "Error: No default input device." << std::endl;
  }
  inputParameters.channelCount = 1;         /* mono input */
  inputParameters.sampleFormat = paFloat32; /* 32 bit floating point input */
  inputParameters.suggestedLatency =
      Pa_GetDeviceInfo(inputParameters.device)->defaultLowInputLatency;
  inputParameters.hostApiSpecificStreamInfo = NULL;

  // Open stream
  Pa_OpenStream(&stream, &inputParameters, &outputParameters, SampleRate,
                BUFFER_SIZE, paClipOff, audioCallback, this);
  Pa_StartStream(stream);
}

bool PAsound::isQueueEmpty() { return soundQueue.empty(); }

std::queue<SoundObject> *PAsound::getQueue() { return &soundQueue; }

void PAsound::play(Operation op, std::vector<float> data) {
  int samples = duration * SampleRate / 1000.;
  std::vector<DTMF> send = dataToDTMF(op, data);
  for (auto dtmf : send) {
    std::array<float, 2> freqs = DTMFtoFreq(dtmf);
    SoundObject sound({freqs, samples, 0});
    soundQueue.push(sound);
  }
}

void PAsound::stop() {
  std::cout << "Stopping Stream" << std::endl;
  PaError err = Pa_AbortStream(stream);
  if (err != paNoError) {
    std::cout << "Error: " << Pa_GetErrorText(err) << std::endl;
  }
  isStreamActive = false;
}

int audioCallback(const void *inputBuffer, void *outputBuffer,
                  unsigned long framesPerBuffer,
                  const PaStreamCallbackTimeInfo *timeInfo,
                  PaStreamCallbackFlags statusFlags, void *userData) {
  float *out = (float *)outputBuffer;
  /* Prevent unused variable warning. */
  (void)timeInfo;
  (void)statusFlags;
  PAsound *sound = (PAsound *)userData;
  if (sound->isListening()) {
    char dtmf =
        findDTMF(framesPerBuffer, sound->getSampleRate(), (float *)inputBuffer);
    switch (dtmf) {
    case -1:
      /* for (int i = 0; i < framesPerBuffer; i++) */
      /*   *out++ = 0; */
      break;
    case '#':
      sound->insertInputBuffer(dtmf);
      sound->setListening(false);
      break;
    default:
      sound->insertInputBuffer(dtmf);
      break;
    }
    return paContinue;
  }

  // Tell the main thread to stop playing when the queue is empty
  if (sound->isQueueEmpty()) {
    for (int i = 0; i < framesPerBuffer; i++)
      *out++ = 0;
    return paContinue;
  }

  SoundObject &soundObject = sound->getQueue()->front();
  unsigned long i;
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
    *out++ = sample;
    soundObject.time += sound->getDTime();
    soundObject.samplesLeft--;
  }

  return paContinue;
}
