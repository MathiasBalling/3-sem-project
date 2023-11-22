#pragma once
#include <string>

// Constants
#define BUFFER_SIZE (500)
#define SAMPLE_RATE (48000)
#define DURATION_MS (100)

#define BASE (14)
#define HEADERSIZE (6)
#define OPERATIONSIZE (4)
#define FLOATSIZE (24)

enum class DTMF {
  ZERO,   // 1 // 0
  ONE,    // 2 // 1
  TWO,    // 3 // 2
  THREE,  // A // 3
  FOUR,   // 4 // 4
  FIVE,   // 5 // 5
  SIX,    // 6 // 6
  SEVEN,  // B // 7
  EIGHT,  // 7 // 8
  NINE,   // 8 // 9
  A,      // 9 // 10
  B,      // C // 11
  C,      // * // 12
  D,      // 0 // 13
  WALL,   // # // Start/stop
  DIVIDE, // D // between dublicate data e.g. 1 1
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

enum class State { WAITING, LISTENING, SENDING, PROCESSING };

const std::string indexToOperation[10] = {
    "Error", "Acknowledge", "Forward",  "Backward",   "Left",
    "Right", "Stop",        "Movement", "Coordinate", "lidar"};

const char indexToDtmf[16] = {'1', '2', '3', 'A', '4', '5', '6', 'B',
                              '7', '8', '9', 'C', '*', '0', '#', 'D'};

const float dtmf_freqs[8] = {697, 770, 852, 941, 1209, 1336, 1477, 1633};
