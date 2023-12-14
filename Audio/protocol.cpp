#include "protocol.h"
#include "consts.h"
#include <cmath>
#include <cstddef>
#include <iostream>
#include <sys/types.h>

// ---------------------------- Encode -------------------------------
std::vector<DTMF> dataToDTMF(Operation op) {
  std::vector<uint64_t> output{};
  size_t baseIndex{1}; // Start after parity bit
  size_t dataSize{1};  // Always operation code
  uint64_t temp{};
  if (op <= Operation::STOP) {
    temp += (uint64_t)pow(2, baseIndex) * dataSize;
    baseIndex = (size_t)HEADER_SIZE_BYTES * 8;
    temp += (uint64_t)pow(2, baseIndex) * (uint64_t)op;
  } else if (op > Operation::STOP) {
    temp = 0; // Error: Every operation over STOP need inputData or is not
    // defiend yet
  }
  output.push_back(temp);

  // Ensure even parity
  if (!evenParity(output)) {
    output.at(0) += 1;
  }
  return dataBitsToDTMF(output);
}

std::vector<DTMF> dataToDTMF(Operation op,
                             const std::vector<float> &inputData) {
  std::vector<uint64_t> output{};
  uint64_t temp{};
  if (op <= Operation::STOP) {
    temp = 0; // Wrong function call
    output.push_back(temp);
  } else if (op <= Operation::LIDAR) {
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
  return dataBitsToDTMF(output);
}

std::vector<DTMF> dataToDTMF(Operation op, const std::string &inputData) {
  std::vector<uint64_t> output{};
  uint64_t temp{};
  if (op <= Operation::COORDINATE_REL) {
    temp = 0; // Wrong function call
    output.push_back(temp);
  } else if (op <= Operation::STRING) {
    output = dataStringEncode(op, inputData);
  } else {
    temp = 0; // Not defined
    output.push_back(temp);
  }

  // Ensure even parity
  if (!evenParity(output)) {
    output.at(0) += 1;
  }
  return dataBitsToDTMF(output);
}

std::vector<uint64_t> dataFloatEncode(const Operation op,
                                      const std::vector<float> &inputData) {
  std::vector<uint64_t> outputBits{};
  if ((inputData.size() * COSTUM_FLOAT_SIZE_BYTES) > (pow(2, 7) - 1)) {
    return outputBits; // To long data input
  }
  size_t baseIndex{1}; // Start after parity bit
  size_t dataSize = 1 + inputData.size() * COSTUM_FLOAT_SIZE_BYTES; // Operation
  uint64_t temp{};
  size_t maxIndex = sizeof(temp) * 8;
  if (op <= Operation::STOP) {
    return outputBits; // wrong function call
  } else if (op <= Operation::LIDAR) {
    temp += (uint64_t)pow(2, baseIndex) * dataSize;
    baseIndex = (size_t)HEADER_SIZE_BYTES * 8;
    temp += (uint64_t)pow(2, baseIndex) * (uint64_t)op;
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
    while ((data - (int)data > 0) && exponent < 8 &&
           (int)data * 10 < (pow(2, 20) - 1)) {
      data *= 10;
      exponent++;
    }
    uint64_t res = (int)data;
    std::cout << "data:" << (int)(pow(2, 20) - 1) << std::endl;
    std::cout << "Test:" << res << std::endl;
    res += negative << 20;
    res += exponent << 21;
    std::cout << "negative:" << negative << std::endl;
    std::cout << "exponent:" << exponent << std::endl;
    // Insert into outputBits
    for (int i = 0; i < (COSTUM_FLOAT_SIZE_BYTES * 8); ++i) {
      if (baseIndex % maxIndex == 0) {
        outputBits.push_back(temp);
        temp = 0;
        baseIndex = 0;
      }
      temp += (uint64_t)pow(2, baseIndex) * (res & 1);
      res >>= 1;
      baseIndex++;
    }
  }
  if (baseIndex != 0)
    outputBits.push_back(temp);

  return outputBits;
}

std::vector<uint64_t> dataStringEncode(const Operation op,
                                       const std::string &inputData) {
  std::vector<uint64_t> outputBits{};
  if (inputData.size() > (pow(2, 7) - 1)) {
    return outputBits; // To long data input
  }
  size_t baseIndex{1}; // Start after parity bit
  size_t dataSize = 1 + inputData.size() * sizeof(char); // Operation
  uint64_t temp{};
  size_t maxIndex = sizeof(temp) * 8;
  if (op <= Operation::COORDINATE_REL) {
    return outputBits; // wrong function call
  } else if (op == Operation::STRING) {
    temp += (uint64_t)pow(2, baseIndex) * dataSize;
    baseIndex = (size_t)HEADER_SIZE_BYTES * 8;
    temp += (uint64_t)pow(2, baseIndex) * (uint64_t)op;
    baseIndex = (HEADER_SIZE_BYTES + OPERATION_SIZE_BYTES) * 8;
  } else {
    return outputBits; // Not yet defined
  }
  for (auto data : inputData) {
    for (size_t i = 0; i < (sizeof(char) * 8); ++i) {
      if (baseIndex % maxIndex == 0) {
        outputBits.push_back(temp);
        temp = 0;
        baseIndex = 0;
      }
      temp += (uint64_t)pow(2, baseIndex) * (data & 1);
      data >>= 1;
      baseIndex++;
    }
  }
  if (baseIndex != 0)
    outputBits.push_back(temp);
  return outputBits;
}

std::vector<DTMF> dataBitsToDTMF(const std::vector<uint64_t> &inputData) {
  std::vector<DTMF> output;
  // Convert to DTMF
  output.push_back(DTMF::WALL); // Start flag
  DTMF lastDtmf = DTMF::WALL;
  auto tempData = inputData;
  size_t count{};
  while (!tempData.empty()) {
    DTMF dtmf = (DTMF)(tempData.at(0) % (uint64_t)BASE);
    if (dtmf == lastDtmf) {
      output.push_back(DTMF::DIVIDE);
    }
    output.push_back(dtmf);
    lastDtmf = dtmf;
    tempData.at(0) /= (uint64_t)BASE;
    count++;
    if (tempData.at(0) == 0 && tempData.size() == 1) {
      tempData.erase(tempData.begin());
    } else if (count == 17) {
      // Log14(uint64_tMax)=16.81≈17
      // We need 17 DTMF tones to represent a uint64_tMax
      tempData.erase(tempData.begin());
      count = 0;
    }
  }
  output.push_back(DTMF::DIVIDE); // End flag
  output.push_back(DTMF::WALL);   // End flag
  return output;
}
// ---------------------------- Decode -------------------------------
std::vector<uint64_t> DTMFtoData(std::queue<DTMF> sampleInput) {
  // Find the start flag
  while (1) {
    if (sampleInput.front() == DTMF::WALL) {
      break;
    }
    sampleInput.pop();
    if (sampleInput.empty()) {
      return std::vector<uint64_t>{0};
    }
  }
  std::vector<uint64_t> sampleBits;
  uint64_t tempBits = 0;
  size_t baseIndex = 0;

  while (!sampleInput.empty()) {
    if (sampleInput.front() != DTMF::DIVIDE &&
        sampleInput.front() != DTMF::WALL) {
      tempBits +=
          (uint64_t)sampleInput.front() * (uint64_t)pow(BASE, baseIndex);
      baseIndex++;

      if (baseIndex % 17 == 0) {
        // Log14(uint64_tMax)=16.81≈17
        // We need 17 DTMF tones to represent a uint64_tMax
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
    return std::vector<uint64_t>{0};
  }

  return sampleBits;
}

Operation getOperation(const std::vector<uint64_t> dataInput) {

  // Check the length of the data
  size_t expectedLength = dataInput.at(0) & 0xff;
  expectedLength >>= 1; // Remove parity bit

  uint64_t operation = dataInput.at(0);
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
      dataInput.size() * sizeof(uint64_t) - HEADER_SIZE_BYTES;
  // 8 bytes per uint64_t. Header is not included in data length

  if (lengthByte != expectedLength && lengthByte < maxBytesReceived) {
    return Operation::ERROR;
  } else {
    return (Operation)operation;
  }
}

std::vector<float> dataFloatDecode(std::vector<uint64_t> data) {
  std::vector<float> output;
  float tempFloat{};
  int exponent{};
  size_t index{};
  size_t floatIndex{};
  size_t maxIndex = sizeof(uint64_t) * 8;
  // Remove parity bit
  data.at(0) >>= 1;
  size_t dataLength =
      data.at(0) & (uint64_t)(pow(2, HEADER_SIZE_BYTES * 7) - 1);
  data.at(0) >>= HEADER_SIZE_BYTES * 8 - 1;
  data.at(0) >>= OPERATION_SIZE_BYTES * 8;
  dataLength -= OPERATION_SIZE_BYTES;
  index = (HEADER_SIZE_BYTES + OPERATION_SIZE_BYTES) * 8;
  std::cout << "Data: " << data.at(0) << std::endl;

  while (!data.empty() && dataLength > 0) {

    int bit = data.at(0) & 1;

    if (floatIndex < 20) {
      // Mantissa
      tempFloat += (uint64_t)pow(2, floatIndex) * bit;
    } else if (floatIndex == 20) {
      // Sign
      if (bit)
        tempFloat *= -1;
    } else if (floatIndex < 24) {
      // Exponent
      exponent += (uint64_t)pow(2, floatIndex - 21) * bit;
    }
    if (floatIndex == (COSTUM_FLOAT_SIZE_BYTES * 8 - 1)) {
      tempFloat *= pow(10, -exponent);
      output.push_back(tempFloat);
      floatIndex = 0;
      tempFloat = 0;
      exponent = 0;
      dataLength -= COSTUM_FLOAT_SIZE_BYTES;
    } else {
      floatIndex++;
    }
    index++;
    data.at(0) >>= 1;
    if (index % maxIndex == 0) {
      index = 0;
      data.erase(data.begin());
    }
  }
  return output;
}

std::string dataStringDecode(std::vector<uint64_t> data) {
  std::string output{};
  size_t index{};
  // Remove parity bit
  data.at(0) >>= 1;
  size_t dataLength =
      data.at(0) & (uint64_t)(pow(2, HEADER_SIZE_BYTES * 7) - 1);
  data.at(0) >>= HEADER_SIZE_BYTES * 8 - 1;
  data.at(0) >>= OPERATION_SIZE_BYTES * 8;
  dataLength -= OPERATION_SIZE_BYTES;
  index = (HEADER_SIZE_BYTES + OPERATION_SIZE_BYTES) * 8;
  while (!data.empty() && dataLength > 0) {
    char tempChar{};
    for (size_t j = 0; j < 8; j++) {
      tempChar += (data.at(0) & 1) << j;
      data.at(0) >>= 1;
      index++;
      if (index % (sizeof(uint64_t) * 8) == 0) {
        index = 0;
        data.erase(data.begin());
      }
    }
    output.push_back(tempChar);
  }
  return output;
}

// ---------------------------- Utils -------------------------------
void printData(const std::vector<uint64_t> &data, int base) {
  std::cout << "Data: ";
  size_t count{};
  for (auto d : data) {
    while (d > 0) {
      count++;
      std::cout << (int)(d % base) << " ";
      d /= base;
    }
  }
  std::cout << std::endl;
  std::cout << count << std::endl;
}

bool evenParity(const std::vector<uint64_t> &data) {
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
