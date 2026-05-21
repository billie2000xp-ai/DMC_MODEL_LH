#ifndef _LPDDR_COMMOB_H
#define _LPDDR_COMMOB_H

namespace LPDDRSim {

#ifndef SYSARCH_PLATFORM
enum TransactionType {
    DATA_READ,
    DATA_WRITE
};
#endif

enum TransactionCmd {
    PRECHARGE_PB_CMD, // 0
    PRECHARGE_SB_CMD, // 1
    PRECHARGE_AB_CMD, // 2
    ACTIVATE1_CMD, // 3
    ACTIVATE2_CMD, // 4
    WRITE_CMD, // 5
    WRITE_P_CMD, // 6
    WRITE_MASK_CMD, // 7
    WRITE_MASK_P_CMD, // 8
    READ_CMD, // 9
    READ_P_CMD, // 10
    REFRESH_CMD, // 11
    PER_BANK_REFRESH_CMD, // 12
    INVALID, // 13
    PDE_CMD, // 14
    PD_CMD, // 15
    PDX_CMD, // 16
    ASREFE_CMD, // 17
    ASREF_CMD, // 18
    ASREFX_CMD, // 19
    SRPDE_CMD, // 20
    SRPD_CMD, // 21
    SRPDX_CMD, // 22
    ACTIVATE1_DST_CMD, // 23
    ACTIVATE2_DST_CMD, // 24
    PRECHARGE_PB_DST_CMD, // 25
    CAS, // 26
    CAS_FS, // 27
    CAS_OFF // 28
};

const std::string LPDDR4 = "lpddr4";
const std::string LPDDR5 = "lpddr5";
const std::string LPDDR6 = "lpddr6";
const std::string GUARDIANS_D1 = "guardians-d1";
const std::string GUARDIANS_D2 = "guardians-d2";
const std::string GUARDIANS_3D = "guardians-3d";
const std::string DDR5 = "ddr5";
const std::string DDR4 = "ddr4";
const std::string HBM2E = "hbm2e";
const std::string HBM3 = "hbm3";

enum WR_GROUP_TYPE {
    NO_GROUP = 2, READ_GROUP = 0, WRITE_GROUP = 1
};

enum RANK_GROUP_TYPE {
    NO_RGRP = 0x10, R0RD_GROUP = 0, R0WR_GROUP = 1, R1RD_GROUP = 2, R1WR_GROUP = 3
};
}
#endif