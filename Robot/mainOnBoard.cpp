#include "PAsound.h"
#include "RB3_cpp_publisher.h"
#include "consts.h"
#include "protocol.h"
#include <iostream>
#include <memory>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto publisher = std::make_shared<RB3_cpp_publisher>();
  PAsound sound;
  sound.init(1);

  float currentLinear = 0.0;
  float currentAngular = 0.0;
  while (1) {
    if (sound.getState() == State::PROCESSING) {

      std::pair<Operation, std::vector<float>> input = sound.processInput();
      std::cout << indexToOperation[(int)input.first] << std::endl;
      switch (input.first) {
      case Operation::FORWARD: {
        currentLinear += 0.1;
        break;
      }
      case Operation::BACKWARD: {
        currentLinear -= 0.1;
        break;
      }
      case Operation::LEFT: {
        currentAngular -= 0.1;
        break;
      }
      case Operation::RIGHT: {
        currentAngular += 0.1;
        break;
      }
      case Operation::STOP: {
        currentAngular = 0.0;
        currentLinear = 0.0;
        break;
      }
      case Operation::MOVEMENT: {
        currentLinear = input.second[0];
        currentAngular = input.second[1];
        break;
      }
      case Operation::ERROR: {
        ;
      }

      default:
        break;
      }
      if (input.first != Operation::ERROR)
        publisher->publish_vel(currentLinear, currentAngular);
    }
  }
  rclcpp::shutdown();
  return 0;
}
