#include "PAsound.h"
#include "consts.h"
#include "goertzel.h"
#include <cmath>
#include <iostream>

std::array<int, 2> DTMFtoFreq(DTMF dt) {
  std::array<int, 2> result;
  result[0] = dtmf_freqs[dt % 4];
  result[1] = dtmf_freqs[dt / 4 + 4];
  return result;
};

PAsound::PAsound() {
  // PortAudio Initialization
  Pa_Initialize();
  std::cout << "PortAudio initialized" << std::endl;
  outputDevice = Pa_GetDefaultOutputDevice();
  inputDevice = Pa_GetDefaultInputDevice();
  /* SampleRate = Pa_GetDeviceInfo(outputDevice)->defaultSampleRate; */
}

PAsound::~PAsound() {
  Pa_CloseStream(stream);
  Pa_Terminate();
}

void PAsound::setListening(bool listen) { listening = listen; }

bool PAsound::isListening() { return listening; }

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
}

bool PAsound::isQueueEmpty() { return soundQueue.empty(); }

std::queue<SoundObject> *PAsound::getQueue() { return &soundQueue; }

void PAsound::play(std::vector<float> freqs, int ms_duration) {
  int samples = ms_duration * SampleRate / 1000.;
  SoundObject sound({freqs, samples, 0});
  soundQueue.push(sound);

  if (!isStreamActive) {
    isStreamActive = true;
    std::cout << "Starting Stream" << std::endl;
    Pa_StartStream(stream);
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
    char dtmf = findDTMF(framesPerBuffer, SampleRate, (float *)inputBuffer);
    if (dtmf != -1)
      printf("%c\n", dtmf);
    for (int i = 0; i < framesPerBuffer; i++)
      *out++ = 0;
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
    soundObject.time += dTime;
    soundObject.samplesLeft--;
  }

  return paContinue;
}
