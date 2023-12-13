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
  output.push_back(temp);

  // Ensure even parity
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
  return dataBitsToDTMF(output);
}

std::vector<DTMF> dataToDTMF(Operation op, std::string inputData) {
  std::vector<unsigned int> output{};
  unsigned int temp{};
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
      if (baseIndex % maxIndex == 0) {
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
  std::vector<unsigned int> outputBits{};
  if (inputData.size() > (pow(2, 7) - 1)) {
    return outputBits; // To long data input
  }
  size_t baseIndex{1}; // Start after parity bit
  size_t dataSize = 1 + inputData.size() * sizeof(char); // Operation
  unsigned int temp{};
  size_t maxIndex = sizeof(temp) * 8;
  if (op <= Operation::COORDINATE_REL) {
    return outputBits; // wrong function call
  } else if (op == Operation::STRING) {
    temp += pow(2, baseIndex) * dataSize;
    baseIndex = (size_t)HEADER_SIZE_BYTES * 8;
    temp += pow(2, baseIndex) * (unsigned int)op;
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
      temp += pow(2, baseIndex) * (data & 1);
      data >>= 1;
      baseIndex++;
    }
  }
  if (baseIndex != 0)
    outputBits.push_back(temp);
  return outputBits;
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
  std::vector<float> output;
  float tempFloat{};
  int exponent{};
  size_t index{};
  size_t floatIndex{};
  // Remove parity bit
  data.at(0) >>= 1;
  size_t dataLength =
      data.at(0) & (unsigned int)(pow(2, HEADER_SIZE_BYTES * 7) - 1);
  data.at(0) >>= HEADER_SIZE_BYTES * 8 - 1;
  data.at(0) >>= OPERATION_SIZE_BYTES * 8;
  dataLength -= OPERATION_SIZE_BYTES;
  index = (HEADER_SIZE_BYTES + OPERATION_SIZE_BYTES) * 8;

  while (!data.empty() && dataLength > 0) {

    int bit = data.at(0) & 1;

    if (floatIndex < 20) {
      // Mantissa
      tempFloat += pow(2, floatIndex) * bit;
    } else if (floatIndex == 20) {
      // Sign
      if (bit)
        tempFloat *= -1;
    } else if (floatIndex < 24) {
      // Exponent
      exponent += pow(2, floatIndex - 21) * bit;
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
    if (index % (sizeof(unsigned int) * 8) == 0) {
      index = 0;
      data.erase(data.begin());
    }
  }
  return output;
}

std::string dataStringDecode(std::vector<unsigned int> data) {
  std::string output{};
  size_t index{};
  // Remove parity bit
  data.at(0) >>= 1;
  size_t dataLength =
      data.at(0) & (unsigned int)(pow(2, HEADER_SIZE_BYTES * 7) - 1);
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
      if (index % (sizeof(unsigned int) * 8) == 0) {
        index = 0;
        data.erase(data.begin());
      }
    }
    output.push_back(tempChar);
  }
  return output;
}

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
