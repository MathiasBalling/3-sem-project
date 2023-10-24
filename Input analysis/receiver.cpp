#include <cmath>
#include <iostream>
#include <portaudio.h>
#include <vector>

// To stop terminal from running program press Enter
const int SAMPLE_RATE = 10000;
const int FRAME_SIZE = 1000;

int dtmf_freqs[8] = {697, 770, 852, 941, 1209, 1336, 1477, 1633};
float dtmf_mag[9];
char dtmf[4][4] = {{'1', '2', '3', 'A'},
                   {'4', '5', '6', 'B'},
                   {'7', '8', '9', 'C'},
                   {'*', '0', '#', 'D'}};

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
  float magnitude = q1 * q1 + q2 * q2 - q1 * q2 * coeff;

  return magnitude;
}
char findDTMF(int numSamples, int SAMPLING_RATE, float *data) {
  // DTMF frequencies
  for (int i = 0; i < 8; i++) {
    dtmf_mag[i] = goertzel_mag(numSamples, dtmf_freqs[i], SAMPLING_RATE, data);
  }

  // Find the two largest magnitudes
  dtmf_mag[8] = 0;
  int index1 = 8;
  int index2 = 8;
  for (int i = 0; i < 8; i++) {
    if (dtmf_mag[i] > dtmf_mag[index1]) {
      index2 = index1;
      index1 = i;
    } else if (dtmf_mag[i] > dtmf_mag[index2]) {
      index2 = i;
    }
  }

  // Find the corresponding key
  if (index1 > index2) {
    int temp = index1;
    index1 = index2;
    index2 = temp;
  }
  int row = index1;
  int col = index2 % 4;
  float mean = (dtmf_mag[index1] + dtmf_mag[index1]) / 2;
  char dtmf_char = dtmf[row][col];
  std::vector<char> byte;
  byte.push_back(dtmf_char);
  for (int k = 0; k < 1; k++) {
    if (mean > 10000) {
      std::cout << byte[k] << " " << mean << std::endl;
    }
  }

  byte.clear();
  return 0;
}

static int audioCallback(const void *inputBuffer, void *outputBuffer,
                         unsigned long framesPerBuffer,
                         const PaStreamCallbackTimeInfo *timeInfo,
                         PaStreamCallbackFlags statusFlags, void *userData) {
  float *inputData = (float *)inputBuffer;
  findDTMF(FRAME_SIZE, SAMPLE_RATE, inputData);

  return paContinue;
}

int main() {
  PaError err;
  PaStream *stream;

  err = Pa_Initialize();
  if (err != paNoError) {
    std::cerr << "PortAudio error: " << Pa_GetErrorText(err) << std::endl;
    return 1;
  }

  int numDevices = Pa_GetDeviceCount();
  if (numDevices < 0) {
    std::cerr << "Error getting the number of audio devices." << std::endl;
    return 1;
  }

  std::cout << "Available audio input devices:" << std::endl;
  for (int i = 0; i < numDevices; i++) {
    const PaDeviceInfo *deviceInfo = Pa_GetDeviceInfo(i);
    if (deviceInfo->maxInputChannels > 0) {
      std::cout << i << ": " << deviceInfo->name << std::endl;
    }
  }

  int selectedDevice;
  std::cout << "Enter the number of the input device you want to use: ";
  std::cin >> selectedDevice;

  if (selectedDevice < 0 || selectedDevice >= numDevices) {
    std::cerr << "Invalid device number." << std::endl;
    return 1;
  }

  PaStreamParameters inputParameters;
  inputParameters.device = selectedDevice;
  inputParameters.channelCount = 1; // Mono recording
  inputParameters.sampleFormat = paFloat32;
  inputParameters.suggestedLatency =
      Pa_GetDeviceInfo(inputParameters.device)->defaultLowInputLatency;
  inputParameters.hostApiSpecificStreamInfo = NULL;

  err = Pa_OpenStream(&stream, &inputParameters, NULL, SAMPLE_RATE, FRAME_SIZE,
                      paClipOff, audioCallback, NULL);
  if (err != paNoError) {
    std::cerr << "PortAudio error: " << Pa_GetErrorText(err) << std::endl;
    return 1;
  }

  err = Pa_StartStream(stream);
  if (err != paNoError) {
    std::cerr << "PortAudio error: " << Pa_GetErrorText(err) << std::endl;
    return 1;
  }

  std::cout << "DTMF tones INCOMING. Press Enter to stop" << std::endl;
  std::cin.get();
  std::cin.get(); // Wait for user input

  err = Pa_StopStream(stream);
  if (err != paNoError) {
    std::cerr << "PortAudio error: " << Pa_GetErrorText(err) << std::endl;
    return 1;
  }

  err = Pa_CloseStream(stream);
  if (err != paNoError) {
    std::cerr << "PortAudio error: " << Pa_GetErrorText(err) << std::endl;
    return 1;
  }

  Pa_Terminate();
  return 0;
}
