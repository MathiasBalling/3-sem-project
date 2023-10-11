#include <chrono>
#include <cstdio>
#include <fstream>
#include <math.h>
#include <string>

// Explanation of the Goertzel algorithm:
// https://www.embedded.com/the-goertzel-algorithm/

float goertzel_mag(int numSamples, float TARGET_FREQUENCY, int SAMPLING_RATE,
                   float *data) {
  // Calculate the coefficients
  int k = (int)(0.5 + (((float)numSamples * TARGET_FREQUENCY) /
                       (float)SAMPLING_RATE));
  float omega = (2.0 * M_PI * k) / (float)numSamples;
  float cosine = cos(omega);
  // float sine = sin(omega);
  float coeff = 2.0 * cosine;
  float q0 = 0, q1 = 0, q2 = 0;

  // Run the algorithm
  for (int i = 0; i < numSamples; i++) {
    q0 = coeff * q1 - q2 + data[i];
    q2 = q1;
    q1 = q0;
  }

  // With phase information
  // Calculate the real and imaginary results with the final coefficients
  // float real = (q1-q2*cosine);
  // float imag = (q2 * sine);
  // Calculate the overall magnitude^2 for the target frequency
  // float magnitude = real * real + imag * imag;

  // Without phase information
  // Calculate the overall magnitude^2 for the target frequency
  float magnitude = q1 * q1 + q2 * q2 - q1 * q2 * coeff;

  return magnitude;
}

char findDTMF(int numSamples, int SAMPLING_RATE, float *data) {
  // DTMF frequencies
  int dtmf_freqs[8] = {697, 770, 852, 941, 1209, 1336, 1477, 1633};
  float dtmf_mag[9];

  // Use chrono to time the goertzel_mag function
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < 8; i++) {
    dtmf_mag[i] = goertzel_mag(numSamples, dtmf_freqs[i], SAMPLING_RATE, data);
  }
  auto end = std::chrono::high_resolution_clock::now();
  auto elapsed =
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
  printf("Goertzel algorithm time: %dns\n", (int)elapsed.count());

  // Find the two largest magnitudes
  printf("The magnitudes of the frequencies are: \n");
  dtmf_mag[8] = 0;
  int index1 = 8;
  int index2 = 8;
  for (int i = 0; i < 8; i++) {
    printf("\tFrequency: %dHz\tMagnitude: %f\n", dtmf_freqs[i], dtmf_mag[i]);
    if (dtmf_mag[i] > dtmf_mag[index1]) {
      index2 = index1;
      index1 = i;
    } else if (dtmf_mag[i] > dtmf_mag[index2]) {
      index2 = i;
    }
  }

  printf("The two dominant frequencies are: \n\t%dHz\n\t%dHz\n",
         dtmf_freqs[index1], dtmf_freqs[index2]);

  // Find the corresponding key
  char dtmf[4][4] = {{'1', '2', '3', 'A'},
                     {'4', '5', '6', 'B'},
                     {'7', '8', '9', 'C'},
                     {'*', '0', '#', 'D'}};
  if (index1 > index2) {
    int temp = index1;
    index1 = index2;
    index2 = temp;
  }
  int row = index1;
  int col = index2 % 4;
  char dtmf_char = dtmf[row][col];

  printf("The DTMF sound is: %c\n", dtmf_char);
  return dtmf_char;
}

int main() {
  // Sample rate and size
  int sample_rate = 10000;
  int sample_size = 50;
  float binWidth = (float)sample_rate / (float)sample_size;
  printf("Max bin width: %fHz\n", binWidth);

  // DTMF frequencies
  int dtmf_freqs[8] = {697, 770, 852, 941, 1209, 1336, 1477, 1633};

  // Generate a test signal with two DTMF frequencies
  float data[sample_size];
  for (int i = 0; i < sample_size; i++) {
    data[i] = sin(2 * M_PI * dtmf_freqs[0] * i / sample_rate) +
              sin(2 * M_PI * dtmf_freqs[7] * i / sample_rate);
  }

  // Find the two dominant frequencies
  findDTMF(sample_size, sample_rate, data);

  return 0;
}