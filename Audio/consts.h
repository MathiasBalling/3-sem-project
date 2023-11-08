#pragma once
#include <string>

// Constants
#define BUFFER_SIZE (500)
static double SampleRate = 20000;
static float dTime = 1. / SampleRate;

enum Operation { FORWARD, BACKWARD, LEFT, RIGHT, STOP, COORDINATE, LIDAR };

enum DTMF {
  zero = 0, // 1
  one,      // 2
  two,      // 3
  three,    // A
  four,     // 4
  five,     // 5
  six,      // 6
  seven,    // B
  eight,    // 7
  nine,     // 8
  comma,    // 9
  negative, // C
  start,    // *
  divide,   // 0
  end,      // #
  wall      // D
};

const std::string indexToOperation[7] = {
    "Forward", "Backward", "Left", "Right", "Stop", "Coordinate", "lidar"};

const char indexToDtmf[16] = {'1', '2', '3', 'A', '4', '5', '6', 'B',
                              '7', '8', '9', 'C', '*', '0', '#', 'D'};

const int dtmf_freqs[8] = {697, 770, 852, 941, 1209, 1336, 1477, 1633};

struct DTMFfreqs {
  std::vector<float> One = {697, 1209};
  std::vector<float> Two = {697, 1336};
  std::vector<float> Three = {697, 1477};
  std::vector<float> A = {697, 1633};
  std::vector<float> Four = {770, 1209};
  std::vector<float> Five = {770, 1336};
  std::vector<float> Six = {770, 1477};
  std::vector<float> B = {770, 1633};
  std::vector<float> Seven = {852, 1209};
  std::vector<float> Eight = {852, 1336};
  std::vector<float> Nine = {852, 1477};
  std::vector<float> C = {852, 1633};
  std::vector<float> Star = {941, 1209};
  std::vector<float> Zero = {941, 1336};
  std::vector<float> Hash = {941, 1477};
  std::vector<float> D = {941, 1633};
};
