#pragma once
#include "consts.h"
#include <array>
#include <deque>
#include <iostream>
#include <string>
#include <vector>

std::array<float, 2> DTMFtoFreq(DTMF dt);

std::vector<DTMF> dataToDTMF(Operation op, std::vector<float> inputData);

std::pair<Operation, std::vector<float>> DTMFtoData(std::deque<DTMF> input);

std::pair<Operation, std::vector<float>>
DTMFdecode(const std::deque<DTMF> &input);

int dataEncode(float inputData);

void printData(long long int data, int base);
