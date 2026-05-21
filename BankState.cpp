#include "BankState.h"

using namespace std;
using namespace LPDDRSim;

//All banks start precharged
BankState::BankState(ostream &DDRSim_log_):
    DDRSim_log(DDRSim_log_),
    currentBankState(Idle),
    openRowAddress(0),
    nextRead(0),
    nextReadAp(0),
    nextWrite(0),
    nextWriteAp(0),
    nextWriteRmw(0),
    nextWriteApRmw(0),
    nextWriteMask(0),
    nextWriteMaskAp(0),
    nextActivate1(0),
    nextActivate2(0),
    nextPrecharge(0),
    nextPerBankRefresh(0),
    nextAllBankRefresh(0),
    lastCommand(INVALID),
    stateChangeEn(false),
    stateChangeCountdown(0),
    rwIntlvCountdown(0),
    lastCmdType(DATA_READ),
    lastCmdPri(0),
    lastRow(0x80000000),
    lastCmdSource(0),
    lastPrechargeSource(0),
    lastMatgrp(0xFFFFFFFF),
    pageOpenTime(0),
    act_executing(false),
    fg_ref(false)
{}

void BankState::print() {
    PRINT(" == Bank State ");
    if (currentBankState == Idle) {
        PRINT("    State : Idle");
    }
    else if (currentBankState == RowActive) {
        PRINT("    State : Active");
    }
    else if (currentBankState == Refreshing) {
        PRINT("    State : Refreshing");
    }
    else if (currentBankState == PowerDown) {
        PRINT("    State : Power Down");
    }

    PRINT("    OpenRowAddress  : " << openRowAddress);
    PRINT("    nextRead        : " << nextRead);
    PRINT("    nextReadAp      : " << nextReadAp);
    PRINT("    nextWrite       : " << nextWrite);
    PRINT("    nextWriteAp     : " << nextWriteAp);
    PRINT("    nextWriteRmw    : " << nextWriteRmw);
    PRINT("    nextWriteApRmw  : " << nextWriteApRmw);
    PRINT("    nextActivate1   : " << nextActivate1);
    PRINT("    nextActivate2   : " << nextActivate2);
    PRINT("    nextPrecharge   : " << nextPrecharge);
    PRINT("    nextWriteMask   : " << nextWriteMask);
    PRINT("    nextWriteMaskAp : " << nextWriteMaskAp);
}