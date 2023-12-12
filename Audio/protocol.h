#pragma once
#include "consts.h"
#include <array>
#include <queue>
#include <string>
#include <vector>

std::array<float, 2> DTMFtoFreq(DTMF dt);

// Handle operations and data from application
std::vector<DTMF> dataToDTMF(Operation op, std::vector<float> inputData);
std::vector<DTMF> dataToDTMF(Operation op, std::string inputData);
std::vector<DTMF> dataToDTMF(Operation op);

// Data in bits to DTMF
std::vector<DTMF> dataBitsToDTMF(const std::vector<unsigned int> &inputData);

// Translate float and string to bits
std::vector<unsigned int> dataFloatEncode(const Operation op,
                                          const std::vector<float> &inputData);
std::vector<unsigned int> dataStringEncode(const Operation op,
                                           const std::string &inputData);

//------------------------------------------
// Handle sampled DTMF and present to application
std::vector<unsigned int> DTMFtoData(std::queue<DTMF> sampleInput);

// Translate bits to operation, floats, string
Operation getOperation(const std::vector<unsigned int> dataInput);
std::vector<float> dataFloatDecode(std::vector<unsigned int> data);
std::string dataStringDecode(std::vector<unsigned int> data);

//-----------------------------------
void printData(const std::vector<unsigned int> &data, int base);
bool evenParity(const std::vector<unsigned int> &data);

// Old functions

/* std::vector<DTMF> dataToDTMF(Operation op, std::vector<float> inputData);
 */
/* int dataEncode(float &inputData); */
/* std::pair<Operation, std::vector<float>> DTMFtoData(std::deque<DTMF>
 * input);
 */
/* std::pair<Operation, std::vector<float>> DTMFdecode(long long int &data);
 */
/* void printData(const long long int &data, int base); */
