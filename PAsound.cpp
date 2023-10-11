#include "PAsound.h"
#include "portaudio.h"

PAsound::PAsound() {
  // PortAudio Initialization
  Pa_Initialize();
  std::cout << "PortAudio initialized" << std::endl;
  outputDevice = Pa_GetDefaultOutputDevice();
  inputDevice = Pa_GetDefaultInputDevice();
  outputSampleRate = Pa_GetDeviceInfo(outputDevice)->defaultSampleRate;
}

PAsound::~PAsound() {
  Pa_CloseStream(stream);
  Pa_Terminate();
}

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
    outputSampleRate = Pa_GetDeviceInfo(outputDevice)->defaultSampleRate;
    std::cout << std::endl;
    std::cout << "Default sample rate: " << outputSampleRate << std::endl;
    std::cout << "Enter sample rate (-1 for default): ";
    std::cin >> input;
    if (input != -1) {
      outputSampleRate = input;
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
  double sampleRate = Pa_GetDeviceInfo(outputParameters.device)
                          ->defaultSampleRate; /* default sample rate */

  // Set up input stream
  PaStreamParameters inputParameters;
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
  Pa_OpenStream(&stream, NULL, &outputParameters, sampleRate, BUFFER_SIZE,
                paClipOff, audioCallback, this);
  Pa_SetStreamFinishedCallback(stream, &StreamFinished);
}

bool PAsound::isQueueEmpty() { return soundQueue.empty(); }

std::queue<SoundObject> *PAsound::getQueue() { return &soundQueue; }

void PAsound::play(std::vector<float> freqs, int ms_duration) {
  int samples = ms_duration * outputSampleRate / 1000.;
  SoundObject sound({freqs, samples, 0});
  soundQueue.push(sound);

  if (Pa_IsStreamStopped(stream)) {
    std::cout << "Starting Stream" << std::endl;
    Pa_StartStream(stream);
  }
}

void PAsound::stop() {
  std::cout << "Stopping Stream" << std::endl;
  Pa_StopStream(stream);
  while (!soundQueue.empty()) {
    soundQueue.pop();
  }
}

void PAsound::wait() {
  std::cout << "Waiting for stream to finish" << std::endl;
  bool streamActive;
  do {
    Pa_Sleep(5);
    streamActive = Pa_IsStreamActive(stream);
  } while (streamActive);
}
int audioCallback(const void *inputBuffer, void *outputBuffer,
                  unsigned long framesPerBuffer,
                  const PaStreamCallbackTimeInfo *timeInfo,
                  PaStreamCallbackFlags statusFlags, void *userData) {
  float *out = (float *)outputBuffer;
  /* Prevent unused variable warning. */
  (void)inputBuffer;
  (void)statusFlags;
  (void)timeInfo;

  // Tell the main thread to stop playing when the queue is empty
  PAsound *sound = (PAsound *)userData;
  if (sound->isQueueEmpty()) {
    for (int i = 0; i < framesPerBuffer; i++) {
      *out++ = 0;
    }
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
    if (soundObject.samplesLeft < 50) {
      sample *= (float)(soundObject.samplesLeft / 50.);
    }
    *out++ = sample;
    soundObject.time += dTime;
    soundObject.samplesLeft--;
  }

  return paContinue;
}

void StreamFinished(void *userData) {
  std::cout << "Stream Completed" << std::endl;
}
