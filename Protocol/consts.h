#include <string>
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

const int dtmf[16][2] = {{697, 1209}, {697, 1336}, {697, 1477}, {697, 1633},
                         {770, 1209}, {770, 1336}, {770, 1477}, {770, 1633},
                         {852, 1209}, {852, 1336}, {852, 1477}, {852, 1633},
                         {941, 1209}, {941, 1336}, {941, 1477}, {941, 1633}};


const std::string indexToOperation[7] = {
    "Forward", "Backward", "Left", "Right", "Stop", "Coordinate", "lidar"};

const char indexToDtmf[16] = {'1', '2', '3', 'A', '4', '5', '6', 'B',
                              '7', '8', '9', 'C', '*', '0', '#', 'D'};