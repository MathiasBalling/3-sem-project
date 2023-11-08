#pragma once
#include "consts.h"
#include <array>
#include <iostream>
#include <string>
#include <vector>

std::array<float, 2> DTMFtoFreq(DTMF dt);

std::vector<DTMF> dataToDTMF(Operation op, std::vector<float> inputData);

std::pair<Operation, std::vector<float>> DTMFtoData(std::vector<DTMF> input);

std::vector<float> DTMFdecode(const std::vector<DTMF> &input, int start,
                              int end);