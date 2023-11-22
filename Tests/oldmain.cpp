#include "PAsound.h"
#include "consts.h"
#include "protocol.h"
#include <iostream>

int main() {
  Operation op = Operation::COORDINATE;
  std::vector<float> data = {};
  PAsound sound;
  sound.init(0);
  char c;
  bool quit = false;
  while (!quit) {
    std::cout << sizeof(long long) << std::endl;
    std::cout << "Press:"
              << "\n"
              << "'q' quit!"
              << "\n"
              << "'a' move left!"
              << "\n"
              << "'d' move right!"
              << "\n"
              << "'w' move forward!"
              << "\n"
              << "'s' move backward!"
              << "\n"
              << "'x' stop!"
              << "\n"
              << "'m' linear and angular velocity!"
              << "\n"
              << "'c' to move to coordinate!"
              << "\n";
    std::cin >> c;
    switch (c) {
    case 'a':
      op = Operation::LEFT;
      break;
    case 'd':
      op = Operation::RIGHT;
      break;
    case 'w':
      op = Operation::FORWARD;
      break;
    case 's':
      op = Operation::BACKWARD;
      break;
    case 'x':
      op = Operation::STOP;
      break;
    case 'c':
      op = Operation::COORDINATE;
      std::cout << "Enter x and y coordinate: ";
      std::cin >> data[0];
      std::cin >> data[1];
      break;
    case 'm':
      op = Operation::MOVEMENT;
      std::cout << "Enter linear and angular velocity: ";
      std::cin >> data[0] >> data[1];
      break;
    case 'q':
      quit = true;
      continue;
    default:
      std::cout << "Invalid input!" << std::endl;
      continue;
    };
    sound.play(op, data);
  }
  return 0;
}
