#pragma once
#include "consts.h"
#include <array>
#include <deque>
#include <vector>

std::array<float, 2> DTMFtoFreq(DTMF dt);

std::vector<DTMF> dataToDTMF(Operation op, std::vector<float> inputData);
int dataEncode(float &inputData);

std::pair<Operation, std::vector<float>> DTMFtoData(std::deque<DTMF> input);
std::pair<Operation, std::vector<float>> DTMFdecode(long long int &data);

void printData(const long long int &data, int base);
