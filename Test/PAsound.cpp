#include "PAsound.h"
#include "portaudio.h"
#include <cstddef>
#include <iostream>

PAsound::PAsound() {
  Pa_Initialize();
  outputParameters.device = Pa_GetDefaultOutputDevice();
  if (outputParameters.device == paNoDevice) {
    std::cout << "Error: No default output device." << std::endl;
  }
  outputParameters.channelCount = 1;         /* mono output */
  outputParameters.sampleFormat = paFloat32; /* 32 bit floating point output */
  outputParameters.suggestedLatency =
      Pa_GetDeviceInfo(outputParameters.device)->defaultLowOutputLatency;
  outputParameters.hostApiSpecificStreamInfo = NULL;
  
  Pa_OpenStream(&stream, NULL, &outputParameters, SAMPLE_RATE,
                FRAMES_PER_BUFFER, paClipOff, audioCallback, &data);
}

PAsound::~PAsound() {
  Pa_CloseStream(stream);
  Pa_Terminate();
}

int PAsound::audioCallback(const void *inputBuffer, void *outputBuffer, unsigned long framesPerBuffer, const PaStreamCallbackTimeInfo *timeInfo, PaStreamCallbackFlags statusFlags, void *userData){
  paData *data = (paData *)userData;
  float *out = (float *)outputBuffer;
  unsigned long i;
  (void)inputBuffer; /* Prevent unused variable warning. */
  for (int i = 0; i < framesPerBuffer; i++) {
    *out++ = (float)i*framesPerBuffer*2/framesPerBuffer-1;
  }
  return paContinue;
}

void PAsound::playtest(){
    Pa_StartStream(stream);
    Pa_Sleep(1000);
    Pa_StopStream(stream);
}

void PAsound::wait() {
  Pa_Sleep(1000);
}

// float *makeSineWave(float freq1, float freq2) {
//   float *sine = (float *)malloc(sizeof(float) * TABLE_SIZE);
//   for (int i = 0; i < TABLE_SIZE; i++) {
//     sine[i] = (float)sin(((double)i / (double)TABLE_SIZE) * M_PI * 2. *
//     freq1); sine[i] += (float)sin(((double)i / (double)TABLE_SIZE) * M_PI
//     * 2. * freq2);
//   }
//   return sine;
// }