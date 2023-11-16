#include "protocol.h"

std::vector<DTMF> dataToDTMF(Operation op, std::vector<float> inputData = {}) {
  // TODO: Update header with new specs
  // Send direction in DTMF
  std::vector<DTMF> result;
  result.push_back(DTMF::WALL);

  result.push_back(DTMF::WALL);
  return result;
}
std::vector<float> DTMFdecode(const std::deque<DTMF> &input, int start,
                              int end) {
  // TODO: Update header with new specs
  std::vector<float> result;
  return result;
}

std::pair<Operation, std::vector<float>> DTMFtoData(std::deque<DTMF> input) {
  // TODO: Update header with new specs
  while (1) {
    if (input.front() == DTMF::WALL) {
      break;
    }
    input.erase(input.begin());
    if (input.empty()) {
      return std::make_pair(Operation::ERROR, std::vector<float>());
    }
  }
  std::vector<float> result;
  Operation op = (Operation)input[1];
  return std::make_pair(op, result);
}
