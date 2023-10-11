#include <fstream>
#include <iostream>

int main(void) {
  std::ofstream myfile;
  myfile.open("../Sine.txt");
  int freq1 = 1209;
  int freq2 = 697;
  int sampleRate = 4500;
  int N = 64;
  for (int i = 0; i < N; i++) {
    myfile << sin(2 * M_PI * freq1 * i / sampleRate) +
                  sin(2 * M_PI * freq2 * i / sampleRate)
           << std::endl;
  }

  myfile.close();
  return 0;
}