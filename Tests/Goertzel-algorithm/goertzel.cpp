#include <cstdio>
#include <math.h>
#include <string>
#include <sys/_types/_clock_t.h>

// Explanation of the Goertzel algorithm:
// https://www.embedded.com/the-goertzel-algorithm/

// Find the corresponding key
const char dtmf[4][4] = {{'1', '2', '3', 'A'},
                         {'4', '5', '6', 'B'},
                         {'7', '8', '9', 'C'},
                         {'*', '0', '#', 'D'}};

const int dtmf_freqs[8] = {697, 770, 852, 941, 1209, 1336, 1477, 1633};

float goertzel_mag(int numSamples, float TARGET_FREQUENCY, int SAMPLING_RATE,
                   float *data) {
  // Calculate the coefficients
  int k = (int)(0.5 + (((float)numSamples * TARGET_FREQUENCY) /
                       (float)SAMPLING_RATE));
  float omega = (2.0 * M_PI * k) / (float)numSamples;
  float cosine = cos(omega);
  float coeff = 2.0 * cosine;
  float q0 = 0, q1 = 0, q2 = 0;

  // Run the algorithm
  for (int i = 0; i < numSamples; i++) {
    q0 = coeff * q1 - q2 + data[i];
    q2 = q1;
    q1 = q0;
  }

  // Without phase information
  // Calculate the overall magnitude^2 for the target frequency
  float magnitude = q1 * q1 + q2 * q2 - q1 * q2 * coeff;

  return magnitude;
}

char findDTMF(int numSamples, int SAMPLING_RATE, float *data) {
  // DTMF frequencies
  float dtmf_mag[8];

  // Use chrono to time the goertzel_mag function
  for (int i = 0; i < 8; i++) {
    dtmf_mag[i] = goertzel_mag(numSamples, dtmf_freqs[i], SAMPLING_RATE, data);
  }

  // Find the two largest magnitudes
  int index_low_freqs = 0;
  int index_high_freqs = 4;
  for (int i = 1; i < 4; i++) {
    if (dtmf_mag[i] > dtmf_mag[index_low_freqs]) {
      index_low_freqs = i;
    }
    if (dtmf_mag[i + 4] > dtmf_mag[index_high_freqs]) {
      index_high_freqs = i + 4;
    }
  }

  printf("The two dominant frequencies are: \n\t%dHz\n\t%dHz\n",
         dtmf_freqs[index_low_freqs], dtmf_freqs[index_high_freqs]);

  char dtmf_char = dtmf[index_low_freqs][index_high_freqs - 4];
  float mean = (dtmf_mag[index_low_freqs] + dtmf_mag[index_high_freqs - 4]) / 2;
  if (mean > 10) {
    return dtmf_char;
  }
  return -1;
}

int main() {
  // Sample rate and size
  int sample_rate = 10000;
  int sample_size = 100;
  float binWidth = (float)sample_rate / (float)sample_size;
  printf("Max bin width: %fHz\n", binWidth);

  // Generate a test signal with two DTMF frequencies
  float data[sample_size];
  for (int i = 0; i < sample_size; i++) {
    data[i] = sin(2 * M_PI * dtmf_freqs[0] * i / sample_rate) +
              sin(2 * M_PI * dtmf_freqs[7] * i / sample_rate);
  }

  // Find the two dominant frequencies
  clock_t start = clock();
  char dtmfchar = findDTMF(sample_size, sample_rate, data);
  clock_t end = clock();
  double time_taken = double(end - start) / double(CLOCKS_PER_SEC);
  printf("The corresponding key is: %c\n", dtmfchar);
  printf("Time taken: %fs\n", time_taken);
  return 0;
}