#include "portaudio.h"
#include <iostream>
#include <math.h>
#include <stdio.h>

#define NUM_SECONDS (2)
#define SAMPLE_RATE (44100)
#define FRAMES_PER_BUFFER (64)

#ifndef M_PI
#define M_PI (3.14159265)
#endif

#define TABLE_SIZE (1000)
typedef struct {
  float sine[TABLE_SIZE];
  int phase;
} paTestData;

int totalFrames = 0;

static int patestCallback(const void *inputBuffer, void *outputBuffer,
                          unsigned long framesPerBuffer,
                          const PaStreamCallbackTimeInfo *timeInfo,
                          PaStreamCallbackFlags statusFlags, void *userData) {
  paTestData *data = (paTestData *)userData;
  float *out = (float *)outputBuffer;
  unsigned long i;

  (void)timeInfo; /* Prevent unused variable warnings. */
  (void)statusFlags;
  (void)inputBuffer;
  totalFrames++;

  for (i = 0; i < framesPerBuffer; i++) {
    *out++ = data->sine[data->phase];
    data->phase += 1;
    if (data->phase >= TABLE_SIZE)
      data->phase -= TABLE_SIZE;
  }

  return paContinue;
}

static void StreamFinished(void *userData) {
  paTestData *data = (paTestData *)userData;
}

/* Sinusoidal wavetable with frequency*/
void makeSineWave(paTestData *data, float freq) {
  for (int i = 0; i < TABLE_SIZE; i++) {
    data->sine[i] =
        (float)sin(((double)i / (double)TABLE_SIZE) * M_PI * 2. * freq);
  }
};

int main(void) {
  PaStreamParameters outputParameters;
  PaStream *stream;
  PaError err;
  paTestData data;
  int i;

  printf("PortAudio Test: output sine wave. SR = %d, BufSize = %d\n",
         SAMPLE_RATE, FRAMES_PER_BUFFER);

  /* initialise sinusoidal wavetable with frequency*/
  // for (i = 0; i < TABLE_SIZE; i++) {
  //   data.sine[i] = (float)sin(((double)i / (double)TABLE_SIZE) * M_PI * 2.);
  // }
  makeSineWave(&data, 100.);
  for (int i = 0; i < TABLE_SIZE; i++) {
    printf("%f\n", data.sine[i]);
  }

  data.phase = 0;

  err = Pa_Initialize();
  if (err != paNoError)
    goto error;

  outputParameters.device =
      Pa_GetDefaultOutputDevice(); /* default output device */
  if (outputParameters.device == paNoDevice) {
    fprintf(stderr, "Error: No default output device.\n");
    goto error;
  }
  outputParameters.channelCount = 1;         /* stereo output */
  outputParameters.sampleFormat = paFloat32; /* 32 bit floating point output */
  outputParameters.suggestedLatency =
      Pa_GetDeviceInfo(outputParameters.device)->defaultLowOutputLatency;
  outputParameters.hostApiSpecificStreamInfo = NULL;

  err = Pa_OpenStream(&stream, NULL, /* no input */
                      &outputParameters, SAMPLE_RATE, FRAMES_PER_BUFFER,
                      paClipOff, /* we won't output out of range samples so
                                    don't bother clipping them */
                      patestCallback, &data);

  if (err != paNoError)
    goto error;

  // err = Pa_SetStreamFinishedCallback(stream, &StreamFinished);
  // if (err != paNoError)
  //   goto error;

  // err = Pa_StartStream(stream);
  // if (err != paNoError)
  //   goto error;

  printf("Play for %d seconds.\n", NUM_SECONDS);
  Pa_Sleep(NUM_SECONDS * 1000);
  std::cout << "timeinfo: " << totalFrames << std::endl;

  // err = Pa_StopStream(stream);
  // if (err != paNoError)
  //   goto error;

  // err = Pa_CloseStream(stream);
  if (err != paNoError)
    goto error;

  Pa_Terminate();
  printf("Test finished.\n");

  return err;
error:
  Pa_Terminate();
  fprintf(stderr, "An error occured while using the portaudio stream\n");
  fprintf(stderr, "Error number: %d\n", err);
  fprintf(stderr, "Error message: %s\n", Pa_GetErrorText(err));
  return err;
}