#ifndef _LP_SIMULATOROBJ_H
#define _LP_SIMULATOROBJ_H

#include <stdint.h>

namespace LPDDRSim {

class SimulatorObject {
    private:
    uint64_t currentClockCycle;

    public:
    SimulatorObject() { currentClockCycle = 0; }
    void step() { currentClockCycle ++; }
    inline uint64_t now() { return currentClockCycle; }
    virtual void update() = 0;
};
}
#endif