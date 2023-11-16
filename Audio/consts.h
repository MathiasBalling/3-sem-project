#pragma once
#include <string>

// Constants
#define BUFFER_SIZE (500)
#define SAMPLE_RATE (48000)
#define DURATION (200)

enum class DTMF {
  ZERO = 0, // 1
  ONE,      // 2
  TWO,      // 3
  THREE,    // A
  FOUR,     // 4
  FIVE,     // 5
  SIX,      // 6
  SEVEN,    // B
  EIGHT,    // 7
  NINE,
  A,      // 9
  B,      // C
  C,      // *
  D,      // 0
  WALL,   // #
  DIVIDE, // D
  ERROR   // If no DTMF is detected
};

enum class Operation {
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
