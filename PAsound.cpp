#include "portaudio.h"
#include <math.h>
#include <stdio.h>

#define NUM_SECONDS (2)
#define SAMPLE_RATE (44100)
#define FRAMES_PER_BUFFER (64)
#define TABLE_SIZE (200)

// Make the code for 2 sine waves into a function
// that takes the 2 frequency of both the sine wave as a parameter
// and returns the sine wave as a float array

float *makeSineWave(float freq1, float freq2) {
  float *sine = (float *)malloc(sizeof(float) * TABLE_SIZE);
  for (int i = 0; i < TABLE_SIZE; i++) {
    sine[i] = (float)sin(((double)i / (double)TABLE_SIZE) * M_PI * 2. * freq1);
    sine[i] += (float)sin(((double)i / (double)TABLE_SIZE) * M_PI * 2. * freq2);
  }
  return sine;
}