#include <iostream>

int main(int, char **) {
  int base = 10;
  float data = 150.301;
  int intdata = data;
  int remainderdata = round((data - intdata) * 1000);
  std::vector<int> result;

  // Push the whole part into the vector
  while (intdata > 0) {
    result.push_back(intdata % base);
    intdata /= base;
  }

  // Push the decimal part into the vector
  if (remainderdata > 0) {
    result.push_back(9); // Push the decimal point
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
      result.push_back(temp[i]);
    }
  };

  for (auto res : result) {
    std::cout << res << std::endl;
  }
  return 0;
}
