#include "consts.h"
#include <array>
#include <iostream>
#include <string>
#include <vector>

std::vector<DTMF> dataToDTMF(Operation op, std::vector<float> inputData = {}) {
  // Send direction in DTMF
  std::vector<DTMF> result;
  result.push_back(DTMF::start);
  result.push_back(static_cast<DTMF>(op));
  // TODO: Update header with new specs
  if (op <= Operation::STOP) {
    result.push_back(DTMF::end);
    return result;
  }

  // Send other operations with data in DTMF
  switch (static_cast<int>(op)) {
  case Operation::COORDINATE: {
    result.push_back(DTMF::wall);
    int base = 10;
    int index = 0;
    for (auto data : inputData) {
      if (index != 0)
        result.push_back(DTMF::wall);
      index++;
      // Convert negative number to positive
      if (data < 0) {
        result.push_back(DTMF::negative);
        data = -data;
      }
      int intdata = data;
      int remainderdata = round((data - intdata) * 1000);

      // Push the whole part into the vector
      while (intdata > 0) {
        result.push_back(DTMF(intdata % base));
        intdata /= base;
        if (intdata != 0)
          result.push_back(DTMF::divide);
      }

      // Push the decimal part into the vector
      if (remainderdata > 0) {
        result.push_back(DTMF::comma);
        std::vector<int> temp;
        bool nonZeroBefore = false;
        while (remainderdata > 0) {
          int digit = remainderdata % base;
          if (digit != 0) {
            nonZeroBefore = true;
          }
          if (nonZeroBefore) {
            temp.push_back(digit);
          }
          remainderdata /= base;
        }
        for (int i = temp.size() - 1; i >= 0; i--) {
          result.push_back(static_cast<DTMF>(temp[i]));
          if (i != 0)
            result.push_back(DTMF::divide);
        }
      }
    }

    result.push_back(DTMF::end);
    return result;
  }
  case Operation::LIDAR: {

    result.push_back(DTMF::end);
    return result;
  }
  default:
    return result;
  }
}
std::vector<float> DTMFdecode(const std::vector<DTMF> &input, int start,
                              int end) {
  std::vector<float> result;
  int index = start;
  while (index < end) {
    float temp = 0;
    bool isNegative = false;
    bool isDecimal = false;
    int decimalPlace = 10;
    // Loop through a data element
    while ((input[index] != DTMF::wall) && (input[index] != DTMF::end)) {
      if (input[index] == DTMF::divide) {
        // Do nothing
      } else if (input[index] == DTMF::negative) {
        isNegative = true;
      } else if (input[index] == DTMF::comma) {
        isDecimal = true;
      } else if (isDecimal) {
        temp +=
            static_cast<float>(input[index]) / static_cast<float>(decimalPlace);
        decimalPlace *= 10;
      } else {
        temp *= 10;
        temp += input[index];
      }
      index++;
    }

    if (isNegative) {
      temp = -temp;
    }
    result.push_back(temp);
    index++;
  }
  return result;
}

std::pair<Operation, std::vector<float>> DTMFtoData(std::vector<DTMF> input) {
  //! Error dectection shold happen before this function

  Operation op = (Operation)input[1];
  // TODO: Update header with new specs
  if (op <= Operation::STOP) {
    // Return operation with empty vector
    return std::make_pair(op, std::vector<float>());
  }
  std::vector<float> result;
  switch (static_cast<int>(input[1])) {
  case Operation::COORDINATE: {
    // Start from index 3 to skip start, operation and wall/end
    int index = 3;
    result = DTMFdecode(input, index, input.size());

    return std::make_pair(op, result);
  }
  case Operation::LIDAR: {
    // Start from index 3 to skip start, operation and wall/end
    int index = 3;
    result = DTMFdecode(input, index, input.size());

    return std::make_pair(op, result);
  }
  default:
    return std::make_pair(op, result);
  }
  return std::make_pair(op, result);
}
