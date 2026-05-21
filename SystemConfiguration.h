#ifndef _LP_SYSCONFIG_H
#define _LP_SYSCONFIG_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <stdint.h>
#include <map>
#include "Macros.h"
#include <sys/types.h>

namespace LPDDRSim {
extern std::ofstream cmd_verify_out; //used by BusPacket.cpp if VERIFICATION_OUTPUT is enabled
extern bool DEBUG_BUS;
extern bool RD_APRE_EN;
extern bool WR_APRE_EN;
extern bool ENHAN_RD_AP_EN;
extern bool ENHAN_WR_AP_EN;
extern bool AREF_EN;
extern std::string SYSTEM_CONFIG;
extern bool STATE_LOG;
extern bool PRINT_TRACE;
extern bool PRINT_READ;
extern bool PRINT_RDATA;
extern bool PRINT_SCH;
extern bool PRINT_EXEC;
extern bool PRINT_IDLE_LAT;
extern bool PRINT_LATENCY;
extern bool PRINT_DRAM_TRACE;
extern bool PRINT_UT_TRACE;
extern bool PRINT_CMD_NUM;
extern uint64_t PRINT_CH_OHOT;
extern bool PRINT_BW;
extern uint32_t PRINT_BW_WIN;
extern bool PERFECT_DMC_EN;
extern unsigned PERFECT_DMC_DELAY;
extern bool DROP_WRITE_CMD;
extern bool LAT_INC_BP;
extern bool FORCE_BAINTLV_EN;
extern bool POWER_EN;
extern bool BA_SHIFT_DIR;
extern unsigned BA_SHIFT_BIT;
extern bool SLOT_FIFO;
extern bool TIME_ASSERT_EN;
extern unsigned EXEC_NUMBER;
extern unsigned NUM_SIDS;
extern unsigned NUM_GROUPS;
extern unsigned NUM_BANKS;
extern unsigned NUM_RANKS;
extern unsigned NUM_CHANS;
extern unsigned NUM_ROWS;
extern unsigned NUM_COLS;
extern unsigned NUM_MATGRPS;
extern unsigned DRAM_CAPACITY;
extern unsigned JEDEC_DATA_BUS_BITS;
extern unsigned DMC_DATA_BUS_BITS;
extern bool BUSYSTATE_INC_WCMD;
extern unsigned BUSYSTATE_TH;
extern bool PRI_ADPT_ENABLE;
extern unsigned LP_QOS2_TIMEOUT;
extern unsigned LP_QOS3_TIMEOUT;
extern unsigned STATE_TIME;
extern bool MPAM_MAPPING_TIMEOUT;
#ifdef SYSARCH_PLATFORM
extern std::map <std::string, std::vector<int64_t>> MAP_CONFIG;
#else
extern std::map <std::string, std::vector<unsigned>> MAP_CONFIG;
#endif
extern bool PRIORITY_PASS_ENABLE;
extern unsigned LAST_BUSRT_LEN;
extern unsigned DMC_RATE;
extern std::string DDR_TYPE;
extern std::string DDR_MODE;
extern std::string DRAM_VENDOR;
extern std::string DRAM_MODE;
extern bool IS_LP4;
extern bool IS_LP5;
extern bool IS_LP6;
extern bool IS_GD1;
extern bool IS_GD2;
extern bool IS_G3D;
extern bool IS_DDR4;
extern bool IS_DDR5;
extern bool IS_HBM2E;
extern bool IS_HBM3;
extern unsigned ABR_PSTPND_LEVEL;
extern unsigned PBR_PSTPND_LEVEL;
extern unsigned PRE_NUM_SEND_PBR;
extern unsigned RANK_TIMEOUT;
extern bool PBR_EN;
extern bool ENH_PBR_EN;
extern bool ENH_PBR_CEIL_EN;
extern bool DERATING_EN;
extern unsigned DERATING_RATIO;
extern bool PBR_PARA_EN;
extern bool AREF_OFFSET_EN;
extern bool SBR_IDLE_EN;
extern unsigned SBR_REQ_MODE;
extern unsigned SBR_FRCST_NUM;
extern unsigned SBR_WEIGHT_MODE;
extern unsigned SBR_WEIGHT_ENH_MODE;
extern unsigned SBR_GAP_CNT;
extern bool SBR_IDLE_ADAPT_EN;
extern unsigned SBR_IDLE_ADAPT_WIN;
extern unsigned SBR_IDLE_ADAPT_LEVEL;
extern bool SEND_DSTREF_SERIAL;
extern bool PD_PBR_EN;
extern unsigned FG_REFRESH_TH;
extern bool DEBUG_STATE;
extern bool DEBUG_GBUF_STATE;
extern bool DEBUG_RMW_STATE;
extern bool DEBUG_PDU;
extern bool ECC_MERGE_ENABLE;
extern uint64_t DEBUG_START_TIME;
extern uint64_t DEBUG_END_TIME;
extern unsigned DMC_READ_FF_TH;
extern unsigned DMC_WRITE_FF_TH;

extern unsigned tREFI;
extern float PAM_RATIO;
extern float tDFI;
extern bool PAGE_ADAPT_EN;
extern bool PAGE_TIMEOUT_EN;
extern unsigned OPENPAGE_TIME_RD;
extern unsigned OPENPAGE_TIME_WR;
extern bool ENH_PAGE_ADPT_EN;
extern unsigned ENH_PAGE_ADPT_WIN;
extern unsigned DMC_BW_WIN;
extern bool PAGE_ADPT_EN;
extern unsigned PAGE_WIN_MODE;
extern unsigned PAGE_WIN_SIZE;
extern unsigned PAGE_OPC_TH;
extern unsigned PAGE_PPC_TH;
extern unsigned PAGE_ADPT_STEP;
extern unsigned PAGE_TIME_MAX;
extern unsigned DRESP_BP_TH;
extern unsigned RL;
extern unsigned WL;
extern unsigned BLEN;
extern unsigned tRAS;
extern unsigned tRCD_WR;
extern unsigned tRCD;
extern unsigned tRRD_L;
extern unsigned tRRD_S;
extern unsigned tRPpb;
extern unsigned tRPab;
extern unsigned tRPfg;
extern unsigned tPPD;
extern unsigned tPPD_L;
extern unsigned tFPW;
extern unsigned tCCD_S;
extern unsigned tCCD_L;
extern unsigned tCCD_R;
extern unsigned tCCD_M;
extern unsigned tCCD_M_WR;
extern unsigned tCCD_L_WR;
extern unsigned tCCD_L_WR2;
extern unsigned tCCD_NSR;
extern unsigned tCCD_NSW;
extern unsigned tCCDMW;
extern unsigned tCCD_L24;
extern unsigned tCCD_L48;
extern unsigned PCFG_TWR;
extern unsigned PCFG_TRTP;
extern unsigned PCFG_TRTW;
extern unsigned PCFG_TRTW_L;
extern unsigned PCFG_TWTR;
extern unsigned PCFG_TWTR_L;
extern unsigned PCFG_TWTR_SB;
extern unsigned PCFG_RANKTRTR;
extern unsigned PCFG_RANKTRTW;
extern unsigned PCFG_RANKTWTR;
extern unsigned PCFG_RANKTWTW;
extern unsigned tRFCab;
extern unsigned tRFCpb;
extern unsigned tPBR2PBR;
extern unsigned tPBR2PBR_L;
extern unsigned tPBR2ACT;
extern unsigned tFAW;
extern unsigned tCMD2SCH;
extern unsigned tCMD2SCH_BYPACT;
extern unsigned tCMD_CONF;
extern unsigned tD_D;//dmc read data delay
extern unsigned tPIPE_PRE_DMC;
extern unsigned tCMD_PHY;
extern unsigned tDAT_PHY;
extern unsigned tCMD_RASC;
extern unsigned tCMD_ADAPT;
extern unsigned tWDATA_DMC;
extern unsigned tDAT_RASC;
extern unsigned tCMD_WAKEUP;
extern unsigned tXP;
extern unsigned tXP_V570;
extern unsigned tXP_V580;
extern unsigned tXP_V590;
extern unsigned tXSR;
extern unsigned tCMDPD;
extern unsigned tnACU;
extern unsigned tRDPD;
extern unsigned tRDAPPD;
extern unsigned tWRPD;
extern unsigned tWRAPPD;
extern unsigned tPDE;
extern unsigned tCSPD;
extern unsigned tPDLP;
extern unsigned tPHYLPE;
extern unsigned tPHYLPX;
extern unsigned tASREFE;
extern bool HPC_ROW_HIT_PRI;
extern bool FASTWAKEUP_EN;
extern bool PREDICT_FASTWAKEUP;
extern bool QOS_INV;
extern bool GREEN_PATH_DIS;
extern bool LQOS_BP_EN;
extern unsigned LQOS_BP_LEVEL;
extern bool TIMEOUT_ENABLE;
extern unsigned QOS_POLICY;
extern unsigned TIMEOUT_MODE;
extern unsigned RDATA_TYPE;
extern bool IECC_ENABLE;
extern unsigned PDU_DEPTH;
extern unsigned IECC_CONFLICT_CNT;
extern bool PD_ENABLE;
extern bool ASREF_ENABLE;
extern bool ASREF_ADAPT_EN;
extern bool IECC_PARTIAL_BYPASS;
extern uint64_t IECC_BYPASS_ADDRESS;
extern bool IECC_BL32_MODE;
extern unsigned IECC_PRI;
extern unsigned IECC_CAP_RATIO;

extern float POWER_RDINC_K;
extern float POWER_RDWRAP_K;
extern float POWER_WRINC_K;
extern float POWER_WRWRAP_K;
extern float POWER_RDATA_K;
extern float POWER_WDATA_K;
extern float POWER_ACT_K;
extern float POWER_PREP_K;
extern float POWER_PRES_K;
extern float POWER_PREA_K;
extern float POWER_PBR_K;
extern float POWER_ABR_K;
extern float POWER_R2W_K;
extern float POWER_W2R_K;
extern float POWER_RNKSW_K;
extern float POWER_PDE_K;
extern float POWER_ASREFE_K;
extern float POWER_SRPDE_K;
extern float POWER_PDX_K;
extern float POWER_ASREFX_K;
extern float POWER_SRPDX_K;
extern float POWER_IDLE_K;
extern float POWER_PDCC_K;
extern uint64_t MATRIX_ROW23;
extern uint64_t MATRIX_ROW22;
extern uint64_t MATRIX_ROW21;
extern uint64_t MATRIX_ROW20;
extern uint64_t MATRIX_ROW19;
extern uint64_t MATRIX_ROW18;
extern uint64_t MATRIX_ROW17;
extern uint64_t MATRIX_ROW16;
extern uint64_t MATRIX_ROW15;
extern uint64_t MATRIX_ROW14;
extern uint64_t MATRIX_ROW13;
extern uint64_t MATRIX_ROW12;
extern uint64_t MATRIX_ROW11;
extern uint64_t MATRIX_ROW10;
extern uint64_t MATRIX_ROW9;
extern uint64_t MATRIX_ROW8;
extern uint64_t MATRIX_ROW7;
extern uint64_t MATRIX_ROW6;
extern uint64_t MATRIX_ROW5;
extern uint64_t MATRIX_ROW4;
extern uint64_t MATRIX_ROW3;
extern uint64_t MATRIX_ROW2;
extern uint64_t MATRIX_ROW1;
extern uint64_t MATRIX_ROW0;
extern uint64_t MATRIX_CH;
extern uint64_t MATRIX_RA2;
extern uint64_t MATRIX_RA1;
extern uint64_t MATRIX_RA0;
extern uint64_t MATRIX_BA6;
extern uint64_t MATRIX_BA5;
extern uint64_t MATRIX_BA4;
extern uint64_t MATRIX_BA3;
extern uint64_t MATRIX_BA2;
extern uint64_t MATRIX_BA1;
extern uint64_t MATRIX_BA0;
extern uint64_t MATRIX_BG4;
extern uint64_t MATRIX_BG3;
extern uint64_t MATRIX_BG2;
extern uint64_t MATRIX_BG1;
extern uint64_t MATRIX_BG0;
extern uint64_t MATRIX_SID2;
extern uint64_t MATRIX_SID1;
extern uint64_t MATRIX_SID0;
extern uint64_t MATRIX_COL10;
extern uint64_t MATRIX_COL9;
extern uint64_t MATRIX_COL8;
extern uint64_t MATRIX_COL7;
extern uint64_t MATRIX_COL6;
extern uint64_t MATRIX_COL5;
extern uint64_t MATRIX_COL4;
extern uint64_t MATRIX_COL3;
extern uint64_t MATRIX_COL2;
extern uint64_t MATRIX_COL1;
extern uint64_t MATRIX_COL0;

extern unsigned DMC_THEORY_BW;
extern unsigned FLOW_STAT_TIME;
extern bool MPAM_PUSH_EN;
extern bool MID_PUSH_EN;
extern unsigned PD_PRD;
extern unsigned ASREF_PRD;
extern unsigned ASREF_ADAPT_WIN;
/* For power parameters (current and voltage), see externs in MemoryController.cpp */

//Memory Controller related parameters
extern unsigned TRANS_QUEUE_DEPTH;
extern unsigned TABLE_DEPTH;
extern std::string ROW_BUFFER_POLICY;
extern std::string BANK_TABLE_MAPPING;
extern float WCK2DFI_RATIO;
extern bool OFREQ_EN;
extern float OFREQ_RATIO;
extern bool GRP_RW_EN;
extern unsigned ENGRP_LEVEL;
extern unsigned EXGRP_LEVEL;
extern unsigned CMD_WLEVELL;
extern unsigned CMD_WLEVELH;
extern unsigned SERIAL_RLEVELL;
extern unsigned SERIAL_RLEVELH;
extern unsigned SERIAL_WLEVELL;
extern unsigned SERIAL_WLEVELH;
extern unsigned RLEVEL_R2W;
extern unsigned ALEVEL_W2R;
extern bool HARDWARE_RHIT_EN;
extern unsigned CMD_RLEVELL;
extern unsigned CMD_RLEVELH;
extern unsigned RHIT_RW_CMD_NUM;
extern unsigned RHIT_ACT_CMD_NUM;
extern bool RWGRP_TRANS_BY_TOUT;
extern unsigned RWGRP_AUTO_BW;
extern bool GRP_RANK_EN;
extern unsigned GRP_RANK_MODE;
extern bool GRP_RANK_PREPB;
extern bool GRP_RANK_PREAB;
extern unsigned GRP_RANK_LEVEL;
extern bool SIMPLE_GRP_RANK_EN;
extern unsigned READ_RANK_GRP_LEVEL0_H;
extern unsigned READ_RANK_GRP_LEVEL0_L;
extern unsigned READ_RANK_GRP_LEVEL1_H;
extern unsigned READ_RANK_GRP_LEVEL1_L;
extern bool SIMPLE_GRP_SID_EN;
extern bool RCMD_HQOS_EN;
extern unsigned RCMD_HQOS_LEVEL;
extern unsigned SWITCH_HQOS_LEVEL;
extern unsigned CPU_MID_START;
extern unsigned CPU_MID_END;
extern bool RCMD_HQOS_W2R_SWITCH_EN;
extern unsigned RCMD_HQOS_W2R_RLEVELH;
extern unsigned RCMD_HQOS_RANK_SWITCH_LEVELH;
extern unsigned RCMD_HQOS_RANK_SWITCH_LEVELL;
extern unsigned RCMD_HQOS_RANK_SWITCH_ROWHIT_LEVEL;
extern bool RCMD_HQOS_RANK_SWITCH_EN;
extern bool RHIT_HQOS_BREAK_EN;
extern unsigned RHIT_HQOS_BREAK_OTH_RCMD_LEVEL;
extern bool BYP_ACT_EN;
extern bool RHIT_BREAK_EN;
extern unsigned RHIT_BREAK_LEVEL;
extern unsigned RW_GRPCHG_W2R_TH;
extern unsigned RW_GRPCHG_R2W_TH;
extern bool DMC_V580;
extern bool DMC_V590;
extern bool DMC_V596;
extern bool EARLY_TCCDL_EN;
extern bool TRFC_CC_EN;
extern unsigned WB_WDEPTH;
extern unsigned WB_RDEPTH;
extern bool MERGE_ENABLE;
extern bool FORWARD_ENABLE;
extern bool DYN_BYP_EN;
extern bool GBUF_RCMD_BLOCK_PBR;
extern bool GBUF_RD_BRK_WFLUSH_CMD;
extern unsigned GBUF_RCMD_TIMEOUT;
extern unsigned GBUF_WCMD_PRIADAPT;
extern bool RQ_RNKCONF_EN;
extern bool RQ_ADCONF_PUSH_EN;
extern unsigned CMD_ROW_ORDER;
extern unsigned TOTAL_RCMD_MODE;
extern unsigned RQ_BACONF_MODE;
extern unsigned WQ_BACONF_MODE;
extern bool RO_HIT_EN;
extern bool WRITE_BUFFER_ENABLE;
extern bool EM_ENABLE;
extern unsigned EM_MODE;
extern bool RMW_ENABLE;
extern unsigned RMW_CMD_MODE;
extern unsigned RMW_QUE_DEPTH;
extern unsigned RMW_CONF_SIZE;
extern unsigned GRT_FIFO_DEPTH;
extern unsigned TRANS_FIFO_DEPTH;
extern unsigned ROW_SEL;
extern bool BG_ROTATE_EN;
extern bool RCMD_BANK_ARB_EN;
extern bool WCMD_BANK_ARB_EN;
extern unsigned RCMD_BANK_ARB_PRI;
extern unsigned WCMD_BANK_ARB_PRI;
extern bool WCK_ALWAYS_ON;
extern bool CAS_FS_EN;
extern unsigned CAS_FS_TH;
extern unsigned CORE_CONCURR;
extern unsigned CORE_CONCURR_PRD;
extern bool WCMD_MERGE_EN;
extern unsigned WRITE_MERGE_BUFFER_DEPTH;
extern bool UNPAIRED_TO_RMW_EN;

extern bool DRAM_POWER_EN;
extern float IDD01, IDD02H, IDD02L, IDD0Q;
extern float IDD2P1, IDD2P2H, IDD2P2L, IDD2PQ, IDD2PS1, IDD2PS2H, IDD2PS2L, IDD2PSQ;
extern float IDD2N1, IDD2N2H, IDD2N2L, IDD2NQ, IDD2NS1, IDD2NS2H, IDD2NS2L, IDD2NSQ;
extern float IDD3P1, IDD3P2H, IDD3P2L, IDD3PQ, IDD3PS1, IDD3PS2H, IDD3PS2L, IDD3PSQ;
extern float IDD3N1, IDD3N2H, IDD3N2L, IDD3NQ, IDD3NS1, IDD3NS2H, IDD3NS2L, IDD3NSQ;
extern float IDD4R1_BG, IDD4R2H_BG, IDD4R2L_BG, IDD4RQ_BG, IDD4R1_BK, IDD4R2H_BK, IDD4R2L_BK, IDD4RQ_BK;
extern float IDD4W1_BG, IDD4W2H_BG, IDD4W2L_BG, IDD4WQ_BG, IDD4W1_BK, IDD4W2H_BK, IDD4W2L_BK, IDD4WQ_BK;
extern float IDD51, IDD52H, IDD52L, IDD5Q, IDD5AB1, IDD5AB2H;
extern float IDD5AB2L, IDD5ABQ, IDD5PB1, IDD5PB2H, IDD5PB2L, IDD5PBQ;
extern float IDD61, IDD62H, IDD62L, IDD6Q, IDD6DS1, IDD6DS2H, IDD6DS2L, IDD6DSQ;
extern float VDD1, VDD2H, VDD2L, VDDQH, VDDQL;

enum TraceType {
    k6,
    mase,
    misc
};

enum BankTableMappingPolicy {
    FULL_MAPPING ,
    PARTIAL_MAPPING
};

// Only used in CommandQueue
enum QueuingStructure {
    PerRank,
    PerRankPerBank
};

enum SchedulingPolicy {
    RankThenBankRoundRobin,
    BankThenRankRoundRobin
};

// set by IniReader.cpp

typedef void (*returnCallBack_t)(unsigned id, uint64_t addr, uint64_t clockcycle);
typedef void (*powerCallBack_t)(double bgpower, double burstpower, double refreshpower, double actprepower);

extern SchedulingPolicy schedulingPolicy;
extern BankTableMappingPolicy BankTableMode;
extern QueuingStructure queuingStructure;
};
#endif