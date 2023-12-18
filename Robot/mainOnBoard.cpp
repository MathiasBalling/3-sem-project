#include "PAsound.h"
#include "RB3_cpp_publisher.h"
#include "consts.h"
#include "protocol.h"
#include <cstdint>
#include <iostream>
#include <memory>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto publisher = std::make_shared<RB3_cpp_publisher>();
  PAsound sound;
  sound.init(1);

  float currentLinear = 0.f;
  float currentAngular = 0.f;
  std::vector<uint64_t> data{};
  Operation operation = Operation::ERROR;
  std::vector<float> dataFloats{0.f, 0.f};
  while (1) {
    data = sound.processInput(); // Fill data with DTMF data if ready
    if (data.empty()) {
      continue; // If no data, continue
    }
    operation = getOperation(data);
    std::cout << indexToOperation[(int)operation] << std::endl;
    switch (operation) {
    case Operation::FORWARD: {
      currentLinear += 0.03f;
      if (currentLinear > 0.22f)
        currentLinear = 0.22f;
      break;
    }
    case Operation::BACKWARD: {
      currentLinear -= 0.03f;
      if (currentLinear < -0.22f)
        currentLinear = -0.22f;
      break;
    }
    case Operation::RIGHT: {
      currentAngular -= 0.25f;
      if (currentAngular < -2.84f)
        currentAngular = -2.84f;
      break;
    }
    case Operation::LEFT: {
      currentAngular += 0.25f;
      if (currentAngular > 2.84f)
        currentAngular = 2.84f;
      break;
    }
    case Operation::STOP: {
      currentAngular = 0.0f;
      currentLinear = 0.0f;
      break;
    }
    case Operation::MOVEMENT: {
      dataFloats = dataFloatDecode(data);
      currentLinear = dataFloats[0];
      if (currentLinear < -0.22f)
        currentLinear = -0.22f;
      else if (currentLinear > 0.22f)
        currentLinear = 0.22f;
      currentAngular = dataFloats[1];
      if (currentAngular < -2.84f)
        currentAngular = -2.84f;
      else if (currentAngular > 2.84f)
        currentAngular = 2.84f;
      break;
    }
    case Operation::UPDATE_MAG_THRESHOLD: {
      dataFloats = dataFloatDecode(data);
      if (dataFloats[0] > 1.f && dataFloats[0] < 5000.f)
        sound.setMinMagnitude(dataFloats[0]);
      break;
    }
    case Operation::STRING: {
      std::cout << dataStringDecode(data) << std::endl;
      break;
    }
    case Operation::ERROR: {
      continue;
      break;
    }
    default:
      continue;
      break;
    }
    publisher->publish_vel(currentLinear, currentAngular);
  }
  rclcpp::shutdown();
  return 0;
}
