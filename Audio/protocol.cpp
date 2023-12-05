#include "protocol.h"
#include "consts.h"
#include <cmath>
#include <iostream>

std::vector<DTMF> dataToDTMF(Operation op, std::vector<float> inputData = {}) {
  long long int data = 0;
  int insertIndex = 1;
  // A long long int is 64 bits
  // First 6 bits is the size of the data up to 2^6 = 64
  int sizeOfData = 4; // The operation is the next 4 bits 2^4 = 16
  if (op <= Operation::STOP) {
    data += pow(2, insertIndex) * sizeOfData;
  } else if (op > Operation::STOP) {
    sizeOfData += 2 * FLOATSIZE;
    data += pow(2, insertIndex) * sizeOfData;
  }
  insertIndex += HEADERSIZE;
  // Next 4 bits is the operation
  data += pow(2, insertIndex) * (int)op;
  insertIndex += OPERATIONSIZE;
  if (Operation::STOP < op) {
    for (int i = 0; i < inputData.size(); i++) {
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
  std::vector<DTMF> result;
  result.push_back(DTMF::WALL); // Start flag
  DTMF lastDtmf = DTMF::WALL;
  while (data) {
    DTMF dtmf = (DTMF)(data % BASE);
    if (dtmf == lastDtmf) {
      result.push_back(DTMF::DIVIDE);
    }
    result.push_back(dtmf);
    lastDtmf = dtmf;
    data /= BASE;
  }
  result.push_back(DTMF::DIVIDE); // End flag
  result.push_back(DTMF::WALL);   // End flag
  return result;
}

int dataEncode(float &inputData) {
  int data = 0;
  // XXX is exponent (3 bits)10^-N for 0<=N<=7
  // E is the sign bit (1 bit) E=1 means -2^20=-1048576
  // 00000000000000000000 is the mantissa (20 bits) 2^20 = 1048576
  // XXX E00000000000000000000
  bool negative = false;
  int exponent = 0;
  if (inputData < 0) {
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
  std::vector<float> result;
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
  data >>= (int)HEADERSIZE;

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
    result.push_back(dataInFloat);
    data >>= FLOATSIZE;
    dataSize -= FLOATSIZE;
  }

  // Check if there is any data left
  // If there is, then the data is corrupted
  if (data != 0 || dataSize != 0) {
    return std::make_pair(Operation::ERROR, std::vector<float>());
  }

  return std::make_pair(op, result);
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
  for (int i = 0; i < input.size(); i++) {
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
