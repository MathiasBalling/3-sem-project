#pragma once
#include <string>

// Constants for PAsound
#define BUFFER_SIZE (500)
// The amount of sound samples to be processed by goertzel.
// Time to sample: 500 / 48000 = 10.4167 ms
#define SAMPLE_RATE (48000)
// Sample rate for send and receiving
#define DURATION_MS (30)
// Duration of each DTMF tone in ms
// Samples to send: 30 / (1 / 48000) = 1440 samples
// Can be modified from GUI!
#define SOUND_FADE_STEPS (150)
// Fade output sound in and out to avoid stratching
// Time: 150/48000 = 3.125 ms
#define THRESHOLD_MAG_DETECTION (50)
// Threshold magnetude for detemining if a DTMF is sampled.

// Constants for protocol
#define BASE (14)
// Base system
// TODO: Change to Bytes instead of bits
#define HEADERSIZE (7)    // TO 1
#define OPERATIONSIZE (4) // TO 1
#define FLOATSIZE (24)    // TO 3

enum class State { WAITING, LISTENING, SENDING, PROCESSING };

enum class DTMF {
  ZERO,   // 1
  ONE,    // 2
  TWO,    // 3
  THREE,  // A
  FOUR,   // 4
  FIVE,   // 5
  SIX,    // 6
  SEVEN,  // B
  EIGHT,  // 7
  NINE,   // 8
  A,      // 9
  B,      // C
  C,      // *
  D,      // 0
  WALL,   // # // Start/Stop flags
  DIVIDE, // D // between dublicate DTMF tones e.g. A A -> A D A
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
  UPDATE_MAG_THRESHOLD,
  COORDINATE_REL,
  COORDINATE_ABS,
  LIDAR
};

const std::string indexToOperation[12] = {"Error",
                                          "Acknowledge",
                                          "Forward",
                                          "Backward",
                                          "Left",
                                          "Right",
                                          "Stop",
                                          "Movement",
                                          "Update Magnetude Threshold",
                                          "Relative Coordinate",
                                          "Absolute Coordinate",
                                          "LIDAR"};

const char indexToDtmf[16] = {'1', '2', '3', 'A', '4', '5', '6', 'B',
                              '7', '8', '9', 'C', '*', '0', '#', 'D'};

const float dtmf_freqs[8] = {697, 770, 852, 941, 1209, 1336, 1477, 1633};
