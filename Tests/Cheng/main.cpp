#include <array>
#include <iostream>
#include <vector>

// Coord (-1.23, -1.1)
// * 5 D C 2 9 3 0 A D C 2 9 2 #
// st  | -   ,       | -   ,   end
// Direction (left)
// * 2 #
// Stop moving
// * 4 #

enum Operation { forward, backward, left, right, stop, coordinate, lidar };

enum DTMF {
  zero = 0, // 1
  one,      // 2
  two,      // 3
  three,    // A
  four,     // 4
  five,     // 5
  six,      // 6
  seven,    // B
  eight,    // 7
  nine,     // 8
  comma,    // 9 change to comma?
  negative, // C change to negative?
  start,    //*
  divide,   // 0
  end,      // #
  wall      // D
};
char indexToDtmf[16] = {'1', '2', '3', 'A', '4', '5', '6', 'B',
                        '7', '8', '9', 'C', '*', '0', '#', 'D'};

static int dtmf[16][2] = {{697, 1209}, {697, 1336}, {697, 1477}, {697, 1633},
                          {770, 1209}, {770, 1336}, {770, 1477}, {770, 1633},
                          {852, 1209}, {852, 1336}, {852, 1477}, {852, 1633},
                          {941, 1209}, {941, 1336}, {941, 1477}, {941, 1633}};

std::array<int, 2> freqsDTMF(DTMF dt) {
  std::array<int, 2> result;
  result[0] = dtmf[dt][0];
  result[1] = dtmf[dt][1];
  return result;
};

std::vector<DTMF> dataToDTMF(Operation op, std::vector<float> inputData = {}) {
  // Send direction in DTMF
  if (op <= Operation::stop) {
    std::vector<DTMF> result;
    result.push_back(DTMF::start);
    result.push_back((DTMF)op);
    result.push_back(DTMF::end);
    return result;
  }

  // Send other operations with data in DTMF
  std::vector<DTMF> result;
  switch ((int)op) {
  case Operation::coordinate: {
    result.push_back(DTMF::start);
    result.push_back((DTMF)op);
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
          result.push_back((DTMF)temp[i]);
          if (i != 0)
            result.push_back(DTMF::divide);
        }
      }
    }

    result.push_back(DTMF::end);
    return result;
  }
  case Operation::lidar: {
    result.push_back(DTMF::start);

    result.push_back(DTMF::end);
    return result;
  }
  default:
    return result;
  }
}

void DTMFtoData(std::vector<DTMF> input) {
  // Error dectection shold happen before this function

  Operation op = (Operation)input[1];
  std::cout << "Operation: " << op << std::endl;
  if (op <= Operation::stop) {
    return;
  }

  std::vector<float> result;
  switch ((int)op) {
  case Operation::coordinate: {
    int index = 3;
    while (index < input.size()) {
      float temp = 0;
      bool isNegative = false;
      bool isDecimal = false;
      int decimalPlace = 1;
      while (input[index] != DTMF::wall || input[index] != DTMF::end) {
        if (input[index] == DTMF::negative) {
          isNegative = true;
        } else if (input[index] == DTMF::comma) {
          isDecimal = true;
        } else if (isDecimal) {
          temp += (float)input[index] / (float)decimalPlace;
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
    for (auto i : result) {
      std::cout << i << " ";
    }
    std::cout << std::endl;
    return;
  }
  }
}

int main() {
  std::vector<float> data = {11.11};
  std::vector<DTMF> result = dataToDTMF(Operation::coordinate, data);
  for (auto i : result) {
    std::cout << indexToDtmf[i] << " ";
  };
  std::cout << std::endl;
  DTMFtoData(result);
  return 0;
}