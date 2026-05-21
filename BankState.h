#ifndef LP_BANKSTATE_H
#define LP_BANKSTATE_H

#include "SystemConfiguration.h"
#include "ddr_common.h"
#ifdef SYSARCH_PLATFORM
#include "dmi_trans.h"
#endif
using namespace std;

namespace LPDDRSim {

enum CurrentBankState {
    Idle,
    RowActive,
    Precharging,
    Refreshing,
    PowerDown
};

class BankState {
    ostream &DDRSim_log;
public:
    //Fields
    CurrentBankState currentBankState;
    unsigned openRowAddress;
    uint64_t nextRead;
    uint64_t nextReadAp;
    uint64_t nextWrite;
    uint64_t nextWriteAp;
    uint64_t nextWriteRmw;
    uint64_t nextWriteApRmw;
    uint64_t nextWriteMask;
    uint64_t nextWriteMaskAp;
    uint64_t nextActivate1;
    uint64_t nextActivate2;
    uint64_t nextPrecharge;
    uint64_t nextPerBankRefresh;
    uint64_t nextAllBankRefresh;
    TransactionCmd lastCommand;
    bool stateChangeEn;
    uint32_t stateChangeCountdown;
    unsigned rwIntlvCountdown;
    TransactionType lastCmdType;
    unsigned lastCmdPri;
    unsigned lastRow;
    unsigned lastCmdSource;
    unsigned lastPrechargeSource; // set true if page timeout and ap command
    unsigned lastMatgrp;
    //Functions
    BankState(ostream &DDRSim_log_);
    void print();
    uint64_t pageOpenTime;
    bool act_executing;
    bool fg_ref;
};
}

#endif