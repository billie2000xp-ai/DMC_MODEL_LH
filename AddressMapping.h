#include "Transaction.h"

namespace LPDDRSim {
    void addressMapping(Transaction &trans);
    uint8_t bit_xor(uint64_t matrix, uint64_t address);
    void CalcMatrixNum();
}