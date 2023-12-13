#include "protocol.h"
#include "consts.h"
#include <cmath>
#include <cstddef>
#include <iostream>
#include <sys/types.h>

// ---------------------------- Encode -------------------------------
std::vector<DTMF> dataToDTMF(Operation op) {
  std::vector<unsigned int> output{};
  size_t baseIndex{1}; // Start after parity bit
  size_t dataSize{1};  // Always operation code
  unsigned int temp{};
  if (op <= Operation::STOP) {
    temp += pow(2, baseIndex) * dataSize;
    baseIndex = (size_t)HEADER_SIZE_BYTES * 8;
    temp += pow(2, baseIndex) * (unsigned int)op;
  } else if (op > Operation::STOP) {
    temp = 0; // Error: Every operation over STOP need inputData or is not
    // defiend yet
  }

  // Ensure even parity
  output.push_back(temp);
  if (!evenParity(output)) {
    output.at(0) += 1;
  }
  return dataBitsToDTMF(output);
}

std::vector<DTMF> dataToDTMF(Operation op, std::vector<float> inputData) {
  std::vector<unsigned int> output{};
  unsigned int temp{};
  if (op <= Operation::STOP) {
    temp = 0; // Wrong function call
    output.push_back(temp);
  } else if (op <= Operation::COORDINATE_REL) {
    // Operation that require floats
    output = dataFloatEncode(op, inputData);
  } else {
    temp = 0; // Not defined or e.g. strings
    output.push_back(temp);
  }

  // Ensure even parity
  if (!evenParity(output)) {
    output.at(0) += 1;
  }
  for (auto res : output) {
    std::cout << res << std::endl;
  }
  return dataBitsToDTMF(output);
}

std::vector<DTMF> dataToDTMF(Operation op, std::string inputData) {
  std::vector<unsigned int> output{};
  unsigned int temp{};
  if (op <= Operation::COORDINATE_REL) {
    temp = 0; // Wrong function call
  } else if (op <= Operation::STRING) {
    // Operation that require floats
    output = dataStringEncode(op, inputData);
  } else {
    temp = 0; // Not defined
  }

  // Ensure even parity
  output.push_back(temp);
  if (!evenParity(output)) {
    output.at(0) += 1;
  }
  return dataBitsToDTMF(output);
}

std::vector<unsigned int> dataFloatEncode(const Operation op,
                                          const std::vector<float> &inputData) {
  std::vector<unsigned int> outputBits{};
  if ((inputData.size() * COSTUM_FLOAT_SIZE_BYTES) > (pow(2, 7) - 1)) {
    return outputBits; // To long data input
  }
  size_t baseIndex{1}; // Start after parity bit
  size_t dataSize = 1 + inputData.size() * COSTUM_FLOAT_SIZE_BYTES; // Operation
  unsigned int temp{};
  size_t maxIndex = sizeof(temp) * 8;
  if (op <= Operation::STOP) {
    return outputBits; // wrong function call
  } else if (op <= Operation::COORDINATE_REL) {
    temp += pow(2, baseIndex) * dataSize;
    baseIndex = (size_t)HEADER_SIZE_BYTES * 8;
    temp += pow(2, baseIndex) * (unsigned int)op;
    baseIndex = (HEADER_SIZE_BYTES + OPERATION_SIZE_BYTES) * 8;
  } else {
    return outputBits; // Not yet defined og a string
  }

  // Costum float 3 bytes
  // XXX is exponent (3 bits)10^-N for 0<=N<=7
  // E is the sign bit (1 bit) E=1 means -2^20=-1048576
  // 00000000000000000000 is the mantissa (20 bits) 2^20 = 1048576
  // XXX E00000000000000000000
  for (auto data : inputData) {
    bool negative = false;
    int exponent = 0;
    if (data < 0) {
      negative = true;
      data *= -1;
    }
    while ((data - (int)data != 0) && exponent < 8) {
      data *= 10;
      exponent++;
    }
    unsigned int res = (int)data;
    res += negative << 20;
    res += exponent << 21;
    // Insert into outputBits
    for (int i = 0; i < (COSTUM_FLOAT_SIZE_BYTES * 8); ++i) {
      if (baseIndex % (sizeof(unsigned int) * 8) == 0) {
        outputBits.push_back(temp);
        temp = 0;
        baseIndex = 0;
      }
      temp += pow(2, baseIndex) * (res & 1);
      res >>= 1;
      baseIndex++;
    }
  }
  if (baseIndex != 0)
    outputBits.push_back(temp);

  return outputBits;
}

std::vector<unsigned int> dataStringEncode(const Operation op,
                                           const std::string &inputData) {
  return std::vector<unsigned int>{};
}

std::vector<DTMF> dataBitsToDTMF(const std::vector<unsigned int> &inputData) {
  std::vector<DTMF> output;
  // Convert to DTMF
  output.push_back(DTMF::WALL); // Start flag
  DTMF lastDtmf = DTMF::WALL;
  auto tempData = inputData;
  size_t count{};
  while (!tempData.empty()) {
    DTMF dtmf = (DTMF)(tempData.at(0) % (unsigned int)BASE);
    if (dtmf == lastDtmf) {
      output.push_back(DTMF::DIVIDE);
    }
    output.push_back(dtmf);
    lastDtmf = dtmf;
    tempData.at(0) /= (unsigned int)BASE;
    count++;
    if (tempData.at(0) == 0 && tempData.size() == 1) {
      tempData.erase(tempData.begin());
    } else if (count == 9) {
      // Log14(uintMax)=8.405≈9
      // We need 9 DTMF tones to represent a uintMax
      tempData.erase(tempData.begin());
      count = 0;
    }
  }
  output.push_back(DTMF::DIVIDE); // End flag
  output.push_back(DTMF::WALL);   // End flag
  return output;
}
// ---------------------------- Decode -------------------------------
std::vector<unsigned int> DTMFtoData(std::queue<DTMF> sampleInput) {
  // Find the start flag
  while (1) {
    if (sampleInput.front() == DTMF::WALL) {
      break;
    }
    sampleInput.pop();
    if (sampleInput.empty()) {
      return std::vector<unsigned int>{0};
    }
  }
  std::vector<unsigned int> sampleBits;
  unsigned int tempBits = 0;
  size_t baseIndex = 0;

  while (!sampleInput.empty()) {
    if (sampleInput.front() != DTMF::DIVIDE &&
        sampleInput.front() != DTMF::WALL) {
      tempBits += (unsigned int)sampleInput.front() *
                  (unsigned int)pow(BASE, baseIndex);
      baseIndex++;

      if (baseIndex % 9 == 0) {
        // Log14(uintMax)=8.405≈9
        // We need 9 DTMF tones to represent a uintMax
        sampleBits.push_back(tempBits);
        tempBits = 0;
        baseIndex = 0;
      }
    }
    sampleInput.pop();
  }
  if (baseIndex != 0) {
    sampleBits.push_back(tempBits);
  }
  if (!evenParity(sampleBits)) {
    return std::vector<unsigned int>{0};
  }

  return sampleBits;
}

Operation getOperation(const std::vector<unsigned int> dataInput) {

  // Check the length of the data
  size_t expectedLength = dataInput.at(0) & 0xff;
  expectedLength >>= 1; // Remove parity bit

  unsigned int operation = dataInput.at(0);
  operation >>= HEADER_SIZE_BYTES * 8;
  operation &= 0xff;

  size_t lengthByte{};
  if (operation <= (size_t)Operation::STOP) {
    lengthByte = OPERATION_SIZE_BYTES; // Only 1 byte for these operation
  } else if (operation <= (size_t)Operation::UPDATE_MAG_THRESHOLD) {
    lengthByte = OPERATION_SIZE_BYTES +
                 (COSTUM_FLOAT_SIZE_BYTES); // The operation + float
  } else if (operation <= (size_t)Operation::COORDINATE_REL) {
    lengthByte = OPERATION_SIZE_BYTES +
                 (COSTUM_FLOAT_SIZE_BYTES * 2); // The operation + 2*float
  } else {
    lengthByte = expectedLength; // Variable length for e.g. strings
  }

  size_t maxBytesReceived =
      dataInput.size() * sizeof(unsigned int) - HEADER_SIZE_BYTES;
  // 4 bytes per unsigned int. Header is not included in data length

  if (lengthByte != expectedLength && lengthByte < maxBytesReceived) {
    return Operation::ERROR;
  } else {
    return (Operation)operation;
  }
}

std::vector<float> dataFloatDecode(std::vector<unsigned int> data) {
  return std::vector<float>{};
}
std::string dataStringDecode(std::vector<unsigned int> data) { return ""; }

// ---------------------------- Utils -------------------------------
void printData(const std::vector<unsigned int> &data, int base) {
  std::cout << "Data: ";
  for (auto data : data) {
    while (data > 0) {
      std::cout << (int)(data % base) << " ";
      data /= base;
    }
  }
  std::cout << std::endl;
}

bool evenParity(const std::vector<unsigned int> &data) {
  // Calcurate the number of 1's
  int oneCount = 0;
  for (auto temp : data) {
    while (temp) {
      oneCount += temp & 1;
      temp >>= 1;
    }
  }
  // Test for even number of 1's
  if (oneCount % 2) {
    return false;
  }
  return true;
}
/*
std::vector<DTMF> dataToDTMF(Operation op, std::vector<float> inputData = {}) {
  long long int data = 0;
  int insertIndex = 1;
  // A long long int is 64 bits
  // First 7 bits is the size of the data up to 2^8-1 = 255
  int sizeOfData = 4; // The operation is the next 4 bits 2^4 = 16
  if (op <= Operation::STOP) {
    data += pow(2, insertIndex) * sizeOfData;
  } else if (op > Operation::STOP) {
    sizeOfData += 2 * FLOATSIZE;
    data += pow(2, insertIndex) * sizeOfData;
  }
  insertIndex += HEADER_SIZE_BYTES;
  // Next 4 bits is the operation
  data += pow(2, insertIndex) * (int)op;
  insertIndex += OPERATIONSIZE;
  if (Operation::STOP < op) {
    for (size_t i = 0; i < inputData.size(); i++) {
      data += (long long int)(pow(2, insertIndex) * dataEncode(inputData[i]));
      insertIndex += FLOATSIZE;
    }
  }

  // Calcurate the parity bit
  int oneCount = 0;
  long long int tempData = data;
  while (tempData) {
    oneCount += tempData & 1;
    tempData >>= 1;
  }
  // Make the data even parity
  if (oneCount % 2) {
    data += 1;
  }

  // Convert to DTMF
  std::vector<DTMF> output;
  output.push_back(DTMF::WALL); // Start flag
  DTMF lastDtmf = DTMF::WALL;
  while (data) {
    DTMF dtmf = (DTMF)(data % BASE);
    if (dtmf == lastDtmf) {
      output.push_back(DTMF::DIVIDE);
    }
    output.push_back(dtmf);
    lastDtmf = dtmf;
    data /= BASE;
  }
  output.push_back(DTMF::DIVIDE); // End flag
  output.push_back(DTMF::WALL);   // End flag
  return output;
}

int dataEncode(float &inputData) {
  int data = 0;
  // XXX is exponent (3 bits)10^-N for 0<=N<=7
  // E is the sign bit (1 bit) E=1 means -2^20=-1048576
  // 00000000000000000000 is the mantissa (20 bits) 2^20 = 1048576
  // XXX E00000000000000000000
  bool negative = false;
  if (inputData < 0) {
  int exponent = 0;
    negative = true;
    inputData *= -1;
  }
  while ((inputData - (int)inputData != 0) && exponent < 8) {
    inputData *= 10;
    exponent++;
  }
  data = (int)inputData;
  data += negative << 20;
  data += exponent << 21;

  return data;
}

std::pair<Operation, std::vector<float>> DTMFdecode(long long int &data) {
  std::vector<float> output;
  // Even parity check
  // Calcurate the parity bit
  int oneCount = 0;
  long long int tempData = data;
  while (tempData) {
    oneCount += tempData & 1;
    tempData >>= 1;
  }
  // Make the data even parity
  if ((oneCount % 2) == 1) {
    return std::make_pair(Operation::ERROR, std::vector<float>());
  }

  data >>= 1;

  // Extract the data size from the first 6 bits of the header
  short dataSize = data & 0b111111;
  data >>= (int)HEADER_SIZE_BYTES;

  // Extract the operation from the first 4 bits of the data
  Operation op = (Operation)(data & 0b1111);
  dataSize -= (int)OPERATIONSIZE;
  data >>= (int)OPERATIONSIZE;

  // Extract the floats from the rest of the data
  for (int i = 0; i < dataSize; i++) {
    float dataInFloat = 0;
    dataInFloat = data & 0xFFFFF;
    short exponent = (data >> 21) & 0b111;
    dataInFloat *= pow(10, -exponent);
    if (data >> 20 & 1) {
      dataInFloat *= -1;
    }
    output.push_back(dataInFloat);
    data >>= FLOATSIZE;
    dataSize -= FLOATSIZE;
  }

  // Check if there is any data left
  // If there is, then the data is corrupted
  if (data != 0 || dataSize != 0) {
    return std::make_pair(Operation::ERROR, std::vector<float>());
  }

  return std::make_pair(op, output);
}

std::pair<Operation, std::vector<float>> DTMFtoData(std::deque<DTMF> input) {
  // Find the start flag
  while (1) {
    if (input.front() == DTMF::WALL) {
      break;
    }
    input.erase(input.begin());
    if (input.empty()) {
      return std::make_pair(Operation::ERROR, std::vector<float>());
    }
  }
  long long int data = 0;
  int baseIndex = 0;
  // Extract the data
  for (size_t i = 0; i < input.size(); i++) {
    if (input[i] != DTMF::DIVIDE && input[i] != DTMF::WALL) {
      data += (long long int)(pow(BASE, baseIndex) * (long long int)input[i]);
      baseIndex++;
    }
  }
  return DTMFdecode(data);
}

void printData(long long int &data, int base) {
  std::cout << "Data: ";
  while (data > 0) {
    std::cout << (int)(data % base) << " ";
    data /= base;
  }
  std::cout << std::endl;
}
*/
