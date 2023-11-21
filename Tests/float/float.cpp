#include <iostream>

int main() {
  float a = -12.25;
  int data = 0;
  // XXX is exponent (3 bits)10^-N for 0<=N<=7
  // E is the sign bit (1 bit) E=1 means -2^20=-1048576
  // 00000000000000000000 is the mantissa (20 bits) 2^20 = 1048576
  // XXX E00000000000000000000
  bool negative = false;
  int exponent = 0;
  if (a < 0) {
    negative = true;
    a *= -1;
  }
  while ((a - (int)a != 0) && exponent < 8) {
    a *= 10;
    exponent++;
  }
  data = (int)a;
  data += negative << 21;
  data += exponent << 22;
  std::cout << "Data:" << data << std::endl;

  float dataInFloat = 0;
  dataInFloat = data & 0x1FFFFF;
  exponent = (data >> 22) & 111;
  dataInFloat *= pow(10, -exponent);
  if (data >> 21 & 1) {
    dataInFloat *= -1;
  }
  std::cout << "DataInput: " << dataInFloat << std::endl;

  return 0;
};
