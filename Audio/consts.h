#pragma once
#include <string>

// Constants
#define BUFFER_SIZE (500)
#define SAMPLE_RATE (48000)
#define DURATION (200)

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
  wall,     // D
  error     // If no DTMF is detected
};

enum Operation {
  ERROR,
  ACKNOWLEDGE,
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  STOP,
  MOVEMENT,
  COORDINATE,
  LIDAR
};

const std::string indexToOperation[10] = {
    "Error", "Acknowledge", "Forward",  "Backward",   "Left",
    "Right", "Stop",        "Movement", "Coordinate", "lidar"};

const char indexToDtmf[16] = {'1', '2', '3', 'A', '4', '5', '6', 'B',
                              '7', '8', '9', 'C', '*', '0', '#', 'D'};

const float dtmf_freqs[8] = {697, 770, 852, 941, 1209, 1336, 1477, 1633};
