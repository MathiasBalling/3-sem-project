#pragma once
#include "consts.h"
#include <array>
#include <cstdint>
#include <queue>
#include <string>
#include <vector>

std::array<float, 2> DTMFtoFreq(DTMF dt);

// Handle operations and data from application
std::vector<DTMF> dataToDTMF(Operation op, std::vector<float> inputData);
std::vector<DTMF> dataToDTMF(Operation op, std::string inputData);
std::vector<DTMF> dataToDTMF(Operation op);

// Data in bits to DTMF
std::vector<DTMF> dataBitsToDTMF(const std::vector<uint64_t> &inputData);

// Translate float and string to bits
std::vector<uint64_t> dataFloatEncode(const Operation op,
                                      const std::vector<float> &inputData);
std::vector<uint64_t> dataStringEncode(const Operation op,
                                       const std::string &inputData);

//------------------------------------------
// Handle sampled DTMF and present to application
std::vector<uint64_t> DTMFtoData(std::queue<DTMF> sampleInput);

// Translate bits to operation, floats, string
Operation getOperation(const std::vector<uint64_t> dataInput);
std::vector<float> dataFloatDecode(std::vector<uint64_t> data);
std::string dataStringDecode(std::vector<uint64_t> data);

//-----------------------------------
void printData(const std::vector<uint64_t> &data, int base);
bool evenParity(const std::vector<uint64_t> &data);
