#include "PAsound.h"
#include "portaudio.h"
#include <chrono>
#include <cstddef>
#include <iostream>
#include <ostream>

PAsound::PAsound() {
  // PortAudio Initialization
  Pa_Initialize();
  // Set up output stream
  outputParameters.device = Pa_GetDefaultOutputDevice();
  if (outputParameters.device == paNoDevice) {
    std::cout << "Error: No default output device." << std::endl;
  }
  outputParameters.channelCount = 1;         /* mono output */
  outputParameters.sampleFormat = paFloat32; /* 32 bit floating point output */
  outputParameters.suggestedLatency =
      Pa_GetDeviceInfo(outputParameters.device)->defaultLowOutputLatency;
  outputParameters.hostApiSpecificStreamInfo = NULL;
  // Open stream
  Pa_OpenStream(&stream, NULL, &outputParameters, SAMPLE_RATE, BUFFER_SIZE,
                paClipOff, audioCallback, this);
  Pa_SetStreamFinishedCallback(stream, &StreamFinished);
  std::cout << "PAsound initialized" << std::endl;
}

PAsound::~PAsound() {
  Pa_CloseStream(stream);
  Pa_Terminate();
}

std::queue<SoundObject> *PAsound::getQueue() { return &soundQueue; }

static float dTime= 1./SAMPLE_RATE;

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
    // std::cout << "empty" << std::endl;
    return paComplete;
  }

  // unsigned long i;
  // SoundObject &soundObject = sound->getQueue()->front();
  // std::cout << "FramesPerBuffer: " << framesPerBuffer
  //           << " samplesLeft: " << soundObject.samplesLeft << std::endl;
  // int index;
  // for (i = 0; i < framesPerBuffer; i++) {
  //   if (soundObject.samplesLeft == 0) {
  //     sound->getQueue()->pop();
  //     if (sound->isQueueEmpty()) {
  //       return paComplete;
  //     }
  //     soundObject = sound->getQueue()->front();
  //   }
  //   // Use micro to determine which sample to play
  //   index = (soundObject.index + i) % BUFFER_SIZE;
  //   std::cout << "index: " << index << std::endl;
  //   Pa_Sleep(100);
  //   *out++ = soundObject.buffer[index];
  //   soundObject.samplesLeft--;
  // }
  // soundObject.index = index;

  SoundObject &soundObject = sound->getQueue()->front();
  unsigned long i;
  for (i = 0; i < framesPerBuffer; i++) {
    if (soundObject.samplesLeft == 0) {
      sound->getQueue()->pop();
      if (sound->isQueueEmpty()) {
        return paComplete;
      }
      soundObject = sound->getQueue()->front();
    }
    *out++ = (float)((sin(soundObject.freq1 * soundObject.time * 2 * M_PI) +
                      sin(soundObject.freq2 * soundObject.time * 2 * M_PI)) /
                     2);
    soundObject.time += dTime;
    soundObject.samplesLeft--;
  }

  return paContinue;
}

void StreamFinished(void *userData) { printf("Stream Completed"); }

void PAsound::wait() {
  bool streamActive;
  do {
    Pa_Sleep(50);
    streamActive = Pa_IsStreamActive(stream);
  } while (streamActive);
}

void PAsound::play(float freq1, float freq2, int ms_duration) {
  SoundObject sound({freq1, freq2, ms_duration * SAMPLE_RATE / 1000, 0});
  soundQueue.push(sound);
  if (Pa_IsStreamStopped(stream)) {
    std::cout << "starting stream" << std::endl;
    Pa_StartStream(stream);
  }
}

bool PAsound::isQueueEmpty() { return soundQueue.empty(); }