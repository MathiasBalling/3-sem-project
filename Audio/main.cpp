#include "PAsound.h"
#include "consts.h"
#include "protocol.h"

int main() {
  dataToDTMF(Operation::FORWARD, {1, 2, 3, 4, 5, 6, 7, 8, 9, 0});
  return 0;
}
