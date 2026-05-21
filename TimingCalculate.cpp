#include "SystemConfiguration.h"
#include "TimingCalculate.h"
#include "ddr_common.h"
#include <assert.h>
#include <math.h>

using namespace std;
namespace LPDDRSim {

void TimingInit() {
#if 1
    unsigned CKR = 4;
    bool READ_LINK_ECC = false;
    bool WRITE_LINK_ECC = false;
    bool DVFSC = false;
    bool BYTE_MODE = false;
    bool READ_DBI = false;
#endif
    // --------------------------------------------------
    // RL
    // --------------------------------------------------
    RL_SPEC.clear();
    if (IS_LP4) {
        if (BYTE_MODE) {
            if (READ_DBI) {
                RL_SPEC[532]  = { 6};
                RL_SPEC[1066] = {12};
                RL_SPEC[1600] = {18};
                RL_SPEC[2132] = {24};
                RL_SPEC[2666] = {30};
                RL_SPEC[3200] = {36};
                RL_SPEC[3732] = {40};
                RL_SPEC[4266] = {44};
            } else { // No Read DBI
                RL_SPEC[532]  = { 6};
                RL_SPEC[1066] = {10};
                RL_SPEC[1600] = {16};
                RL_SPEC[2132] = {22};
                RL_SPEC[2666] = {26};
                RL_SPEC[3200] = {32};
                RL_SPEC[3732] = {36};
                RL_SPEC[4266] = {40};
            }
        } else { // X16 Mode
            if (READ_DBI) {
                RL_SPEC[532]  = { 6};
                RL_SPEC[1066] = {12};
                RL_SPEC[1600] = {16};
                RL_SPEC[2132] = {22};
                RL_SPEC[2666] = {28};
                RL_SPEC[3200] = {32};
                RL_SPEC[3732] = {36};
                RL_SPEC[4266] = {40};
            } else { // No Read DBI
                RL_SPEC[532]  = { 6};
                RL_SPEC[1066] = {10};
                RL_SPEC[1600] = {14};
                RL_SPEC[2132] = {20};
                RL_SPEC[2666] = {24};
                RL_SPEC[3200] = {28};
                RL_SPEC[3732] = {32};
                RL_SPEC[4266] = {36};
            }
        }
    } else if (IS_LP5 || IS_GD2) {
        if (!DVFSC && !READ_LINK_ECC) {
            if (CKR == 2) {
                RL_SPEC[533]  = { 6,  6,  6};
                RL_SPEC[1067] = { 8,  8,  8};
                RL_SPEC[1600] = {10, 10, 12};
                RL_SPEC[2133] = {12, 14, 14};
                RL_SPEC[2750] = {16, 16, 18};
                RL_SPEC[3200] = {18, 20, 20};
            } else if (CKR == 4) {
                RL_SPEC[533]  = { 3,  3,  3};
                RL_SPEC[1067] = { 4,  4,  4};
                RL_SPEC[1600] = { 5,  5,  6};
                RL_SPEC[2133] = { 6,  7,  7};
                RL_SPEC[2750] = { 8,  9,  9};
                RL_SPEC[3200] = { 9, 10, 10};
                RL_SPEC[3733] = {10, 11, 12};
                RL_SPEC[4267] = {12, 13, 14};
                RL_SPEC[4800] = {13, 14, 15};
                RL_SPEC[5500] = {15, 16, 17};
                RL_SPEC[6000] = {16, 17, 19};
                RL_SPEC[6400] = {17, 18, 20};
                RL_SPEC[7500] = {20, 22, 24};
                RL_SPEC[8533] = {23, 25, 26};
            }
        } else if (DVFSC && !READ_LINK_ECC) {
            if (CKR == 2) {
                RL_SPEC[533]  = { 6,  6,  6};
                RL_SPEC[1067] = { 8, 10, 10};
                RL_SPEC[1600] = {12, 12, 14};
            } else if (CKR == 4) {
                RL_SPEC[533]  = { 3,  3,  3};
                RL_SPEC[1067] = { 4,  5,  5};
                RL_SPEC[1600] = { 6,  6,  7};
            }
        } else if (!DVFSC && READ_LINK_ECC) {
            RL_SPEC[3733] = {12, 13};
            RL_SPEC[4267] = {13, 14};
            RL_SPEC[4800] = {15, 16};
            RL_SPEC[5500] = {17, 18};
            RL_SPEC[6000] = {18, 20};
            RL_SPEC[6400] = {19, 21};
            RL_SPEC[7500] = {23, 24};
            RL_SPEC[8533] = {26, 28};
        }
    } else if (IS_LP6) {
        RL_SPEC[533]  = { 6,  6,  6};
        RL_SPEC[1067] = { 8,  8,  8};
        RL_SPEC[1600] = {10, 10, 12};
        RL_SPEC[2133] = {12, 14, 14};
        RL_SPEC[2750] = {16, 18, 18};
        RL_SPEC[3200] = {18, 20, 20};
        RL_SPEC[3733] = {20, 22, 24};
        RL_SPEC[4267] = {24, 26, 28};
        RL_SPEC[4800] = {26, 28, 30};
        RL_SPEC[5500] = {30, 32, 34};
        RL_SPEC[6000] = {32, 34, 38};
        RL_SPEC[6400] = {34, 36, 40};
        RL_SPEC[7500] = {40, 44, 48};
        RL_SPEC[8533] = {46, 50, 52};
        RL_SPEC[9600] = {52, 56, 56};
    } else if (IS_GD1) {
    } else if (IS_GD2) {
    } else if (IS_DDR4) {
    } else if (IS_DDR5) {
    } else if (IS_HBM2E || IS_HBM3) {
        RL_SPEC[3200] = {40};
        RL_SPEC[3600] = {46};
    }

    // --------------------------------------------------
    // WL
    // --------------------------------------------------
    if (IS_LP4) {
        if (BYTE_MODE) {
            WL_SPEC[532]  = { 4, 4};
            WL_SPEC[1066] = { 6, 8};
            WL_SPEC[1600] = { 8,12};
            WL_SPEC[2132] = {10,18};
            WL_SPEC[2666] = {12,22};
            WL_SPEC[3200] = {14,26};
            WL_SPEC[3732] = {16,30};
            WL_SPEC[4266] = {18,34};
        } else { // X16 Mode
            WL_SPEC[532]  = { 4, 4};
            WL_SPEC[1066] = { 6, 8};
            WL_SPEC[1600] = { 8,12};
            WL_SPEC[2132] = {10,18};
            WL_SPEC[2666] = {12,22};
            WL_SPEC[3200] = {14,26};
            WL_SPEC[3732] = {16,30};
            WL_SPEC[4266] = {18,34};
        }
    } else if (IS_LP5 || IS_GD2) {
        if (!DVFSC) {
            if (CKR == 2) {
                WL_SPEC[533]  = { 4,  4};
                WL_SPEC[1067] = { 4,  6};
                WL_SPEC[1600] = { 6,  8};
                WL_SPEC[2133] = { 8, 10};
                WL_SPEC[2750] = { 8, 14};
                WL_SPEC[3200] = {10, 16};
            } else if (CKR == 4) {
                WL_SPEC[533]  = { 2,  2};
                WL_SPEC[1067] = { 2,  3};
                WL_SPEC[1600] = { 3,  4};
                WL_SPEC[2133] = { 4,  5};
                WL_SPEC[2750] = { 4,  7};
                WL_SPEC[3200] = { 5,  8};
                WL_SPEC[3733] = { 6,  9};
                WL_SPEC[4267] = { 6, 11};
                WL_SPEC[4800] = { 7, 12};
                WL_SPEC[5500] = { 8, 14};
                WL_SPEC[6000] = { 9, 15};
                WL_SPEC[6400] = { 9, 16};
                WL_SPEC[7500] = {11, 19};
                WL_SPEC[8533] = {12, 22};
            }
        } else { // DVFSC Enable
            if (CKR == 2) {
                WL_SPEC[533]  = {4, 4};
                WL_SPEC[1067] = {4, 6};
                WL_SPEC[1600] = {6, 8};
            } else if (CKR == 4) {
                WL_SPEC[533]  = {2, 2};
                WL_SPEC[1067] = {2, 3};
                WL_SPEC[1600] = {3, 4};
            }
        }
    } else if (IS_LP6) {
        WL_SPEC[533]  = { 4,  4};
        WL_SPEC[1067] = { 4,  6};
        WL_SPEC[1600] = { 6,  8};
        WL_SPEC[2133] = { 8, 10};
        WL_SPEC[2750] = { 8, 14};
        WL_SPEC[3200] = {10, 16};
        WL_SPEC[3733] = {12, 18};
        WL_SPEC[4267] = {12, 22};
        WL_SPEC[4800] = {14, 24};
        WL_SPEC[5500] = {16, 28};
        WL_SPEC[6000] = {18, 30};
        WL_SPEC[6400] = {18, 32};
        WL_SPEC[7500] = {22, 38};
        WL_SPEC[8533] = {24, 44};
    } else if (IS_GD1) {
    } else if (IS_GD2) {
    } else if (IS_DDR4) {
    } else if (IS_DDR5) {
    } else if (IS_HBM2E || IS_HBM3) {
        WL_SPEC[3200] = {10};
        WL_SPEC[3600] = {10};
    }

    // --------------------------------------------------
    // nWR
    // --------------------------------------------------
    if (IS_LP4) {
        if (BYTE_MODE) {
            NWR_SPEC[532]  = { 6};
            NWR_SPEC[1066] = {12};
            NWR_SPEC[1600] = {16};
            NWR_SPEC[2132] = {22};
            NWR_SPEC[2666] = {28};
            NWR_SPEC[3200] = {32};
            NWR_SPEC[3732] = {38};
            NWR_SPEC[4266] = {44};
        } else { // X16 Mode
            NWR_SPEC[532]  = { 6};
            NWR_SPEC[1066] = {10};
            NWR_SPEC[1600] = {16};
            NWR_SPEC[2132] = {20};
            NWR_SPEC[2666] = {24};
            NWR_SPEC[3200] = {30};
            NWR_SPEC[3732] = {34};
            NWR_SPEC[4266] = {40};
        }
    } else if (IS_LP5 || IS_GD2) {
        if (CKR == 2) {
            if (!DVFSC && !WRITE_LINK_ECC) {
                if (BYTE_MODE) {
                    NWR_SPEC[533]  = { 5};
                    NWR_SPEC[1067] = {10};
                    NWR_SPEC[1600] = {15};
                    NWR_SPEC[2133] = {20};
                    NWR_SPEC[2750] = {25};
                    NWR_SPEC[3200] = {29};
                } else { // X16 Mode
                    NWR_SPEC[533]  = { 5};
                    NWR_SPEC[1067] = {10};
                    NWR_SPEC[1600] = {14};
                    NWR_SPEC[2133] = {19};
                    NWR_SPEC[2750] = {24};
                    NWR_SPEC[3200] = {28};
                }
            } else if (DVFSC && !WRITE_LINK_ECC) {
                if (BYTE_MODE) {
                    NWR_SPEC[533]  = { 6};
                    NWR_SPEC[1067] = {12};
                    NWR_SPEC[1600] = {18};
                } else { // X16 Mode
                    NWR_SPEC[533]  = { 6};
                    NWR_SPEC[1067] = {11};
                    NWR_SPEC[1600] = {17};
                }
            }
        } else if (CKR == 4) {
            if (!DVFSC && !WRITE_LINK_ECC) {
                if (BYTE_MODE) {
                    NWR_SPEC[533]  = { 3};
                    NWR_SPEC[1067] = { 5};
                    NWR_SPEC[1600] = { 8};
                    NWR_SPEC[2133] = {10};
                    NWR_SPEC[2750] = {13};
                    NWR_SPEC[3200] = {15};
                    NWR_SPEC[3733] = {17};
                    NWR_SPEC[4267] = {20};
                    NWR_SPEC[4800] = {22};
                    NWR_SPEC[5500] = {25};
                    NWR_SPEC[6000] = {28};
                    NWR_SPEC[6400] = {29};
                    NWR_SPEC[7500] = {34};
                    NWR_SPEC[8533] = {39};
                } else { // X16 Mode
                    NWR_SPEC[533]  = { 3};
                    NWR_SPEC[1067] = { 5};
                    NWR_SPEC[1600] = { 7};
                    NWR_SPEC[2133] = {10};
                    NWR_SPEC[2750] = {12};
                    NWR_SPEC[3200] = {14};
                    NWR_SPEC[3733] = {16};
                    NWR_SPEC[4267] = {19};
                    NWR_SPEC[4800] = {21};
                    NWR_SPEC[5500] = {24};
                    NWR_SPEC[6000] = {26};
                    NWR_SPEC[6400] = {28};
                    NWR_SPEC[7500] = {32};
                    NWR_SPEC[8533] = {37};
                }
            } else if (!DVFSC && WRITE_LINK_ECC) {
                if (BYTE_MODE) {
                    NWR_SPEC[3733] = {19};
                    NWR_SPEC[4267] = {22};
                    NWR_SPEC[4800] = {24};
                    NWR_SPEC[5500] = {28};
                    NWR_SPEC[6000] = {31};
                    NWR_SPEC[6400] = {32};
                    NWR_SPEC[7500] = {38};
                    NWR_SPEC[8533] = {43};
                } else { // X16 Mode
                    NWR_SPEC[3733] = {18};
                    NWR_SPEC[4267] = {21};
                    NWR_SPEC[4800] = {23};
                    NWR_SPEC[5500] = {27};
                    NWR_SPEC[6000] = {29};
                    NWR_SPEC[6400] = {31};
                    NWR_SPEC[7500] = {36};
                    NWR_SPEC[8533] = {41};
                }
            } else if (DVFSC && !WRITE_LINK_ECC) {
                NWR_SPEC[533]  = { 3};
                NWR_SPEC[1067] = { 6};
                NWR_SPEC[1600] = { 9};
            }
        }
    } else if (IS_LP6) {
        NWR_SPEC[533]  = { 6};
        NWR_SPEC[1067] = {10};
        NWR_SPEC[1600] = {14};
        NWR_SPEC[2133] = {20};
        NWR_SPEC[2750] = {24};
        NWR_SPEC[3200] = {28};
        NWR_SPEC[3733] = {32};
        NWR_SPEC[4267] = {38};
        NWR_SPEC[4800] = {42};
        NWR_SPEC[5500] = {48};
        NWR_SPEC[6000] = {52};
        NWR_SPEC[6400] = {56};
        NWR_SPEC[7500] = {64};
        NWR_SPEC[8533] = {74};
        NWR_SPEC[9600] = {86};
    } else if (IS_GD1) {
    } else if (IS_GD2) {
    } else if (IS_DDR4) {
    } else if (IS_DDR5) {
    } else if (IS_HBM2E || IS_HBM3) {
    }

    // --------------------------------------------------
    // nRTP
    // --------------------------------------------------
    if (IS_LP4) {
        NRTP_SPEC[532]  = { 8};
        NRTP_SPEC[1066] = { 8};
        NRTP_SPEC[1600] = { 8};
        NRTP_SPEC[2132] = { 8};
        NRTP_SPEC[2666] = {10};
        NRTP_SPEC[3200] = {12};
        NRTP_SPEC[3732] = {14};
        NRTP_SPEC[4266] = {16};
    } else if (IS_LP5 || IS_GD2) {
        if (!DVFSC && !READ_LINK_ECC) {
            if (CKR == 2) {
                NRTP_SPEC[533]  = {0};
                NRTP_SPEC[1067] = {0};
                NRTP_SPEC[1600] = {0};
                NRTP_SPEC[2133] = {0};
                NRTP_SPEC[2750] = {2};
                NRTP_SPEC[3200] = {2};
            } else if (CKR == 4) {
                NRTP_SPEC[533]  = {0};
                NRTP_SPEC[1067] = {0};
                NRTP_SPEC[1600] = {0};
                NRTP_SPEC[2133] = {0};
                NRTP_SPEC[2750] = {1};
                NRTP_SPEC[3200] = {1};
                NRTP_SPEC[3733] = {2};
                NRTP_SPEC[4267] = {2};
                NRTP_SPEC[4800] = {3};
                NRTP_SPEC[5500] = {4};
                NRTP_SPEC[6000] = {4};
                NRTP_SPEC[6400] = {4};
                NRTP_SPEC[7500] = {6};
                NRTP_SPEC[8533] = {6};
            }
        } else if (DVFSC && !READ_LINK_ECC) {
            NRTP_SPEC[533]  = {0};
            NRTP_SPEC[1067] = {0};
            NRTP_SPEC[1600] = {0};
        } else if (!DVFSC && READ_LINK_ECC) {
            NRTP_SPEC[3733] = {2};
            NRTP_SPEC[4267] = {2};
            NRTP_SPEC[4800] = {3};
            NRTP_SPEC[5500] = {4};
            NRTP_SPEC[6000] = {4};
            NRTP_SPEC[6400] = {4};
            NRTP_SPEC[7500] = {6};
            NRTP_SPEC[8533] = {6};
        }
    } else if (IS_LP6) {
        NRTP_SPEC[533]  = { 0};
        NRTP_SPEC[1067] = { 0};
        NRTP_SPEC[1600] = { 0};
        NRTP_SPEC[2133] = { 0};
        NRTP_SPEC[2750] = { 1};
        NRTP_SPEC[3200] = { 1};
        NRTP_SPEC[3733] = { 4};
        NRTP_SPEC[4267] = { 4};
        NRTP_SPEC[4800] = { 6};
        NRTP_SPEC[5500] = { 8};
        NRTP_SPEC[6000] = { 8};
        NRTP_SPEC[6400] = { 8};
        NRTP_SPEC[7500] = {12};
        NRTP_SPEC[8533] = {12};
        NRTP_SPEC[9600] = {14};
    } else if (IS_GD1) {
    } else if (IS_GD2) {
    } else if (IS_DDR4) {
    } else if (IS_DDR5) {
    } else if (IS_HBM2E || IS_HBM3) {
    }

    // --------------------------------------------------
    // BLn
    // --------------------------------------------------
    if (IS_LP4) {
        BLN_SPEC[16] = {8};
        BLN_SPEC[32] = {16};
    } else if (IS_LP5 || IS_GD2) {
        if (NUM_GROUPS == 1) { // 16B Mode
            BLN_SPEC[16] = {16/2/CKR};
            BLN_SPEC[32] = {32/2/CKR};
        } else { // BG Mode(CKR = 4), {Same BG BLn, Diff BG BLn, BLn_min, BLn_max}
            BLN_SPEC[16] = {4, 2, 2, 4};
            BLN_SPEC[32] = {8, 2, 6, 8};
        }
    } else if (IS_LP6) {
            BLN_SPEC[16] = {8, 4, 4, 8};
            BLN_SPEC[32] = {16, 4, 12, 16};
    } else if (IS_GD1) {
    } else if (IS_GD2) {
    } else if (IS_DDR4) {
    } else if (IS_DDR5) {
    } else if (IS_HBM2E || IS_HBM3) {
    }

    // --------------------------------------------------
    // WL
    // --------------------------------------------------
    if (IS_LP4) {
    } else if (IS_LP5 || IS_GD2) {
    } else if (IS_LP6) {
    } else if (IS_GD1) {
    } else if (IS_GD2) {
    } else if (IS_DDR4) {
    } else if (IS_DDR5) {
    } else if (IS_HBM2E || IS_HBM3) {
    }








}

unsigned Max(float ns, unsigned nck, float tck, unsigned ratio, float ofreq) {
    return unsigned(ceil(ceil(max(ns / tck, float(nck))) / float(ratio)) * ofreq);
}

void TimingCalc() {
#if 1
    unsigned CKR = 4;
    float OFREQ = 1;
    unsigned WL_SET = 0;
    bool BYTE_MODE = false;
    bool DVFSC = false;
    bool WRITE_LINK_ECC = false;
    std::map <unsigned, unsigned> tWR;
    std::map <unsigned, unsigned> tWRap;
    std::map <unsigned, unsigned> tRTP;
    std::map <unsigned, unsigned> tRTPap;
#endif
    if (IS_LP4) {
        float tCK = 1000 * 2 / DMC_RATE;
        // tREFI
        tREFI = unsigned(floor(3902 * OFREQ / tCK / CKR));
        // tRRD
        if (DMC_RATE > 3733) tRRD_S = Max(7.5, 4, tCK, CKR, OFREQ);
        else tRRD_S = Max(10.0, 4, tCK, CKR, OFREQ);
        tRRD_L = tRRD_S;
        // tFAW
        if (DMC_RATE > 3733) tFAW = Max(30.0, 0, tCK, CKR, OFREQ);
        else tFAW = Max(40.0, 0, tCK, CKR, OFREQ);
        // tRCD
        tRCD = Max(18.0, 4, tCK, CKR, OFREQ);
        tRCD_WR = tRCD;
        // tRP
        tRPpb = Max(18.0, 4, tCK, CKR, OFREQ);
        tRPab = Max(21.0, 4, tCK, CKR, OFREQ);
        // tPPD
        tPPD = Max(0.0, 4, tCK, CKR, OFREQ);
        tPPD_L = tPPD;
        // tRAS
        tRAS = Max(42.0, 3, tCK, CKR, OFREQ);
        // tRFC
        if (DRAM_CAPACITY <= 2) {
            tRFCab = Max(130.0, 0, tCK, CKR, OFREQ);
            tRFCpb = Max(60.0, 0, tCK, CKR, OFREQ);
        } else if (DRAM_CAPACITY <= 4) {
            tRFCab = Max(180.0, 0, tCK, CKR, OFREQ);
            tRFCpb = Max(90.0, 0, tCK, CKR, OFREQ);
        } else if (DRAM_CAPACITY <= 8) {
            tRFCab = Max(280.0, 0, tCK, CKR, OFREQ);
            tRFCpb = Max(140.0, 0, tCK, CKR, OFREQ);
        } else if (DRAM_CAPACITY <= 16) {
            tRFCab = Max(380.0, 0, tCK, CKR, OFREQ);
            tRFCpb = Max(190.0, 0, tCK, CKR, OFREQ);
        }
        // tXSR
        tXSR = tRFCab + Max(7.5, 2, tCK, CKR, OFREQ);
        // tWR(Write To Precharge)
        unsigned twr = 0;
        if (BYTE_MODE) twr = Max(20.0, 6, tCK, CKR, 1);
        else twr = Max(18.0, 6, tCK, CKR, 1);
        unsigned twr_16 = WL_SPEC[DMC_RATE][WL_SET] + 1 + BLN_SPEC[16][0] + twr;
        unsigned twr_32 = WL_SPEC[DMC_RATE][WL_SET] + 1 + BLN_SPEC[32][0] + twr;
        tWR[16] = Max(0.0, twr_16, tCK, 1, OFREQ);
        tWR[32] = Max(0.0, twr_32, tCK, 1, OFREQ);
        // tWRap(Write AP To Precharge)
        unsigned twrap_16 = WL_SPEC[DMC_RATE][WL_SET] + 1 + BLN_SPEC[16][0] + NWR_SPEC[16][0];
        unsigned twrap_32 = WL_SPEC[DMC_RATE][WL_SET] + 1 + BLN_SPEC[32][0] + NWR_SPEC[32][0];
        tWRap[16] = Max(0.0, twrap_16, tCK, 1, OFREQ);
        tWRap[32] = Max(0.0, twrap_32, tCK, 1, OFREQ);
        // tRTP(Read To Precharge)
        unsigned trtp_16 = BLN_SPEC[16][0] + Max(7.5, 8, tCK, CKR, 1) - 8;
        unsigned trtp_32 = BLN_SPEC[32][0] + Max(7.5, 8, tCK, CKR, 1) - 8;
        tRTP[16] = Max(0.0, trtp_16, tCK, 1, OFREQ);
        tRTP[32] = Max(0.0, trtp_32, tCK, 1, OFREQ);
        // tRTPap(Read AP To Precharge)
        unsigned trtpap_16 = BLN_SPEC[16][0] + NRTP_SPEC[16][0] - 8;
        unsigned trtpap_32 = BLN_SPEC[32][0] + NRTP_SPEC[32][0] - 8;
        tRTPap[16] = Max(0.0, trtpap_16, tCK, 1, OFREQ);
        tRTPap[32] = Max(0.0, trtpap_32, tCK, 1, OFREQ);
    } else if (IS_LP5) {
        float tCK = 1000 * 2 / DMC_RATE * CKR;
        // tREFI
        tREFI = unsigned(floor(3902 * OFREQ / tCK));
        // tRRD
        if (DMC_RATE > 6400) tRRD_S = Max(3.75, 2, tCK, 1, OFREQ);
        else tRRD_S = Max(5.0, 2, tCK, 1, OFREQ);
        // tFAW
        if (DMC_RATE > 6400) tFAW = Max(15.0, 2, tCK, 1, OFREQ);
        else tFAW = Max(20.0, 2, tCK, 1, OFREQ);
        // tRCD
        tRCD = Max(18.0, 2, tCK, 1, OFREQ);
        if (DMC_RATE > 6400) tRCD_WR = Max(8.0, 2, tCK, 1, OFREQ);
        else tRCD_WR = tRCD;
        // tRP
        tRPpb = Max(18.0, 2, tCK, 1, OFREQ);
        tRPab = Max(21.0, 2, tCK, 1, OFREQ);
        // tPPD
        tPPD = Max(0.0, 2, tCK, 1, OFREQ);
        tPPD_L = tPPD;
        // tRAS
        tRAS = Max(42.0, 3, tCK, 1, OFREQ);
        // tRFC
        if (DRAM_CAPACITY * NUM_RANKS <= 2) {
            tRFCab = Max(130.0, 0, tCK, 1, OFREQ);
            tRFCpb = Max(60.0, 0, tCK, 1, OFREQ);
        } else if (DRAM_CAPACITY * NUM_RANKS <= 4) {
            tRFCab = Max(180.0, 0, tCK, 1, OFREQ);
            tRFCpb = Max(90.0, 0, tCK, 1, OFREQ);
        } else if (DRAM_CAPACITY * NUM_RANKS <= 8) {
            tRFCab = Max(210.0, 0, tCK, 1, OFREQ);
            tRFCpb = Max(120.0, 0, tCK, 1, OFREQ);
        } else if (DRAM_CAPACITY * NUM_RANKS <= 16) {
            tRFCab = Max(280.0, 0, tCK, 1, OFREQ);
            tRFCpb = Max(140.0, 0, tCK, 1, OFREQ);
        } else if (DRAM_CAPACITY * NUM_RANKS <= 32) {
            tRFCab = Max(380.0, 0, tCK, 1, OFREQ);
            tRFCpb = Max(190.0, 0, tCK, 1, OFREQ);
        }
        // tXSR
        tXSR = tRFCab + Max(7.5, 2, tCK, 1, OFREQ);
        // tWR(Write To Precharge)
        unsigned twr = 0;
        if (BYTE_MODE) { // Byte Mode
            if (NUM_GROUPS == 1) { // 16B Mode
                if (!DVFSC && !WRITE_LINK_ECC) twr = Max(36.0, 3, tCK, 1, 1);
                else if (DVFSC && !WRITE_LINK_ECC) twr = Max(43.0, 3, tCK, 1, 1);
            } else { // BG Mode
                if (!DVFSC && !WRITE_LINK_ECC) twr = Max(36.0, 3, tCK, 1, 1);
                else if (!DVFSC && WRITE_LINK_ECC) twr = Max(40.0, 3, tCK, 1, 1);
            }
        } else { // x16 Mode
            if (NUM_GROUPS == 1) { // 16B Mode
                if (!DVFSC && !WRITE_LINK_ECC) twr = Max(34.0, 3, tCK, 1, 1);
                else if (DVFSC && !WRITE_LINK_ECC) twr = Max(41.0, 3, tCK, 1, 1);
            } else { // BG Mode
                if (!DVFSC && !WRITE_LINK_ECC) twr = Max(34.0, 3, tCK, 1, 1);
                else if (!DVFSC && WRITE_LINK_ECC) twr = Max(38.0, 3, tCK, 1, 1);
            }
        }
        unsigned twr_16 = 0, twr_32 = 0;
        if (NUM_GROUPS == 1) { // 16B Mode
            twr_16 = WL_SPEC[DMC_RATE][WL_SET] + 1 + BLN_SPEC[16][0] + twr;
            twr_32 = WL_SPEC[DMC_RATE][WL_SET] + 1 + BLN_SPEC[32][0] + twr;
        } else {
            twr_16 = WL_SPEC[DMC_RATE][WL_SET] + 1 + BLN_SPEC[16][2] + twr;
            twr_32 = WL_SPEC[DMC_RATE][WL_SET] + 1 + BLN_SPEC[32][2] + twr;
        }
        tWR[16] = Max(0.0, twr_16, tCK, 1, OFREQ);
        tWR[32] = Max(0.0, twr_32, tCK, 1, OFREQ);
        // tWRap(Write AP To Precharge)
        tWRap[16] = tWR[16];
        tWRap[32] = tWR[32];
        // tRTP(Read To Precharge)
        unsigned trbtp = 0, trtp_16 = 0, trtp_32 = 0;
        if (NUM_GROUPS == 1) { // 16B Mode
            trbtp = Max(8.5, 8/CKR, tCK, CKR, 1) - 8/CKR;
            trtp_16 = BLN_SPEC[16][0] + trbtp;
            trtp_32 = BLN_SPEC[32][0] + trbtp;
        } else { // BG Mode
            trbtp = Max(7.5, 8/CKR, tCK, CKR, 1) - 8/CKR;
            trtp_16 = BLN_SPEC[16][2] + trbtp;
            trtp_32 = BLN_SPEC[32][2] + trbtp;
        }
        tRTP[16] = Max(0.0, trtp_16, tCK, 1, OFREQ);
        tRTP[32] = Max(0.0, trtp_32, tCK, 1, OFREQ);
        // tRTPap(Read AP To Precharge)
        unsigned trtpap_16 = 0, trtpap_32 = 0;
        if (NUM_GROUPS == 1) { // 16B Mode
            trtpap_16 = BLN_SPEC[16][0] + NRTP_SPEC[16][0];
            trtpap_32 = BLN_SPEC[32][0] + NRTP_SPEC[32][0];
        } else { // BG Mode
            trtpap_16 = BLN_SPEC[16][2] + NRTP_SPEC[16][0];
            trtpap_32 = BLN_SPEC[32][2] + NRTP_SPEC[32][0];
        }
        tRTPap[16] = Max(0.0, trtpap_16, tCK, 1, OFREQ);
        tRTPap[32] = Max(0.0, trtpap_32, tCK, 1, OFREQ);
    } else if (IS_LP6) {
        float tCK = 1000 * 2 / DMC_RATE * CKR;
        // tREFI
        tREFI = unsigned(floor(3902 * OFREQ / tCK));
        // tRRD
        if (DMC_RATE > 6400) tRRD_S = Max(3.75, 2, tCK, 1, OFREQ);
        else tRRD_S = Max(5.0, 2, tCK, 1, OFREQ);
        // tFAW
        if (DMC_RATE > 6400) tFAW = Max(15.0, 2, tCK, 1, OFREQ);
        else tFAW = Max(20.0, 2, tCK, 1, OFREQ);
        // tRCD
        tRCD = Max(18.0, 2, tCK, 1, OFREQ);
        if (DMC_RATE > 6400) tRCD_WR = Max(8.0, 2, tCK, 1, OFREQ);
        else tRCD_WR = tRCD;
        // tRP
        tRPpb = Max(18.0, 2, tCK, 1, OFREQ);
        tRPab = Max(21.0, 2, tCK, 1, OFREQ);
        // tPPD
        tPPD = Max(0.0, 2, tCK, 1, OFREQ);
        tPPD_L = tPPD;
        // tRAS
        tRAS = Max(42.0, 3, tCK, 1, OFREQ);
        // tRFC
        if (DRAM_CAPACITY * NUM_RANKS <= 2) {
            tRFCab = Max(130.0, 0, tCK, 1, OFREQ);
            tRFCpb = Max(60.0, 0, tCK, 1, OFREQ);
        } else if (DRAM_CAPACITY * NUM_RANKS <= 4) {
            tRFCab = Max(180.0, 0, tCK, 1, OFREQ);
            tRFCpb = Max(90.0, 0, tCK, 1, OFREQ);
        } else if (DRAM_CAPACITY * NUM_RANKS <= 8) {
            tRFCab = Max(210.0, 0, tCK, 1, OFREQ);
            tRFCpb = Max(120.0, 0, tCK, 1, OFREQ);
        } else if (DRAM_CAPACITY * NUM_RANKS <= 16) {
            tRFCab = Max(280.0, 0, tCK, 1, OFREQ);
            tRFCpb = Max(140.0, 0, tCK, 1, OFREQ);
        } else if (DRAM_CAPACITY * NUM_RANKS <= 32) {
            tRFCab = Max(380.0, 0, tCK, 1, OFREQ);
            tRFCpb = Max(190.0, 0, tCK, 1, OFREQ);
        }
        // tXSR
        tXSR = tRFCab + Max(7.5, 2, tCK, 1, OFREQ);
        // tWR(Write To Precharge)
        unsigned twr = 0;
        if (BYTE_MODE) { // Byte Mode
            if (NUM_GROUPS == 1) { // 16B Mode
                if (!DVFSC && !WRITE_LINK_ECC) twr = Max(36.0, 3, tCK, 1, 1);
                else if (DVFSC && !WRITE_LINK_ECC) twr = Max(43.0, 3, tCK, 1, 1);
            } else { // BG Mode
                if (!DVFSC && !WRITE_LINK_ECC) twr = Max(36.0, 3, tCK, 1, 1);
                else if (!DVFSC && WRITE_LINK_ECC) twr = Max(40.0, 3, tCK, 1, 1);
            }
        } else { // x16 Mode
            if (NUM_GROUPS == 1) { // 16B Mode
                if (!DVFSC && !WRITE_LINK_ECC) twr = Max(34.0, 3, tCK, 1, 1);
                else if (DVFSC && !WRITE_LINK_ECC) twr = Max(41.0, 3, tCK, 1, 1);
            } else { // BG Mode
                if (!DVFSC && !WRITE_LINK_ECC) twr = Max(34.0, 3, tCK, 1, 1);
                else if (!DVFSC && WRITE_LINK_ECC) twr = Max(38.0, 3, tCK, 1, 1);
            }
        }
        unsigned twr_16 = 0, twr_32 = 0;
        if (NUM_GROUPS == 1) { // 16B Mode
            twr_16 = WL_SPEC[DMC_RATE][WL_SET] + 1 + BLN_SPEC[16][0] + twr;
            twr_32 = WL_SPEC[DMC_RATE][WL_SET] + 1 + BLN_SPEC[32][0] + twr;
        } else {
            twr_16 = WL_SPEC[DMC_RATE][WL_SET] + 1 + BLN_SPEC[16][2] + twr;
            twr_32 = WL_SPEC[DMC_RATE][WL_SET] + 1 + BLN_SPEC[32][2] + twr;
        }
        tWR[16] = Max(0.0, twr_16, tCK, 1, OFREQ);
        tWR[32] = Max(0.0, twr_32, tCK, 1, OFREQ);
        // tWRap(Write AP To Precharge)
        tWRap[16] = tWR[16];
        tWRap[32] = tWR[32];
        // tRTP(Read To Precharge)
        unsigned trbtp = 0, trtp_16 = 0, trtp_32 = 0;
        if (NUM_GROUPS == 1) { // 16B Mode
            trbtp = Max(8.5, 8/CKR, tCK, CKR, 1) - 8/CKR;
            trtp_16 = BLN_SPEC[16][0] + trbtp;
            trtp_32 = BLN_SPEC[32][0] + trbtp;
        } else { // BG Mode
            trbtp = Max(7.5, 8/CKR, tCK, CKR, 1) - 8/CKR;
            trtp_16 = BLN_SPEC[16][2] + trbtp;
            trtp_32 = BLN_SPEC[32][2] + trbtp;
        }
        tRTP[16] = Max(0.0, trtp_16, tCK, 1, OFREQ);
        tRTP[32] = Max(0.0, trtp_32, tCK, 1, OFREQ);
        // tRTPap(Read AP To Precharge)
        unsigned trtpap_16 = 0, trtpap_32 = 0;
        if (NUM_GROUPS == 1) { // 16B Mode
            trtpap_16 = BLN_SPEC[16][0] + NRTP_SPEC[16][0];
            trtpap_32 = BLN_SPEC[32][0] + NRTP_SPEC[32][0];
        } else { // BG Mode
            trtpap_16 = BLN_SPEC[16][2] + NRTP_SPEC[16][0];
            trtpap_32 = BLN_SPEC[32][2] + NRTP_SPEC[32][0];
        }
        tRTPap[16] = Max(0.0, trtpap_16, tCK, 1, OFREQ);
        tRTPap[32] = Max(0.0, trtpap_32, tCK, 1, OFREQ);
    } else if (IS_GD1) {
    } else if (IS_GD2) {
    } else if (IS_DDR4) {
    } else if (IS_DDR5) {
    } else if (IS_HBM2E || IS_HBM3) {
    }




}

void TimingPrint() {




}

};