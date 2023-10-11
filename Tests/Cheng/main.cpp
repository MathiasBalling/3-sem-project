#include <iostream>
#include <vector>

enum Operation {
  forward,
  backward,
  left,
  right,
  coordinate,
};

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
  A,        // 9
  B,        // C
  start,    //*
  divide,   // 0
  end,      // #
  wall      // D
};

static int dtmf[4][4][2] = {
    {{697, 1209}, {697, 1336}, {697, 1477}, {697, 1633}},
    {{770, 1209}, {770, 1336}, {770, 1477}, {770, 1633}},
    {{852, 1209}, {852, 1336}, {852, 1477}, {852, 1633}},
    {{941, 1209}, {941, 1336}, {941, 1477}, {941, 1633}}};

std::vector<int> freqsDTMF(DTMF dt) {
  int row = (int)dt / 4;
  int col = (int)dt % 4;
  std::vector<int> result;
  result.push_back(dtmf[row][col][0]);
  result.push_back(dtmf[row][col][1]);
  return result;
};

std::vector<DTMF> dataToDTMF(Operation op, int data) {
  std::vector<DTMF> result;
  // Send start and operation
  result.push_back(DTMF::start);
  result.push_back(DTMF::divide);
  result.push_back((DTMF)op);
  result.push_back(DTMF::divide);
  result.push_back(DTMF::wall);
  result.push_back(DTMF::divide);

  // Send data in 12 base
  int base = 12;
  while (data > 0) {
    result.push_back((DTMF)(data % base));
    result.push_back(DTMF::divide);
    data /= base;
  }

  // Send end
  result.push_back(DTMF::end);
  return result;
}

int main() {
  // With 12 bits:0123456789AB
  //* start
  // # end
  // 0 Divider
  // D Wall

  // * 1 0 B 0 3 #
  // * 5 D 55#

  int data = 1283;
  std::vector<DTMF> result = dataToDTMF(Operation::coordinate, data);
  for (auto i : result) {
    std::cout << i << " ";
  };
  return 0;
}