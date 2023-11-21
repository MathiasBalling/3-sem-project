#include "protocol.h"
#include "consts.h"

std::vector<DTMF> dataToDTMF(Operation op, std::vector<float> inputData = {}) {
  std::vector<DTMF> result;
  long long int data = 0;
  int base = 14;
  // A long long int is 64 bits
  // First 6 bits is the size of the data 2^6 = 64
  // The operation is the next 4 bits
  int sizeOfData = 4;
  data += pow(2, 6) * (int)op;
  if (op <= Operation::STOP) {
    data += sizeOfData;
  } else if (op > Operation::STOP) {
    data += inputData.size();
  }
  printData(data, 2);
  printData(data, 10);
  printData(data, 14);

  // Convert to DTMF
  result.push_back(DTMF::WALL); // Start flag

  while (data > 0) {
    result.push_back((DTMF)(data % base));
    data /= base;
  }
  result.push_back(DTMF::WALL); // End flag
  return result;
}

long long int dataEncode(std::vector<float> inputData) {
  long long int data = 0;
  for (int i = 0; i < inputData.size(); i++) {
    // Convert to binary
  }

  return data;
}

std::pair<Operation, std::vector<float>>
DTMFdecode(const std::deque<DTMF> &input, int start, int end) {
  // TODO: Update header with new specs
  std::vector<float> result;
  Operation op = Operation::ACKNOWLEDGE;
  return std::make_pair(op, result);
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
  Operation op = Operation::ACKNOWLEDGE;
  return std::make_pair(op, result);
}

void printData(long long int data, int base) {
  std::cout << "Data: ";
  while (data > 0) {
    std::cout << data % base << " ";
    data /= base;
  }
  std::cout << std::endl;
}
