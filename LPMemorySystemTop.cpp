#include <errno.h>
#include <sstream> //stringstream
#include <stdlib.h> // getenv()
#include <sys/stat.h>
#include <sys/types.h>
#include <iomanip>
#include <math.h>
#include "LPMemorySystemTop.h"
#include "AddressMapping.h"
#include "TimingCalculate.h"
#include "IniReader.h"
#include <algorithm>

using namespace LPDDRSim;
//==============================================================================
namespace LPDDRSim {
#ifdef SYSARCH_PLATFORM
LPMemorySystemTop::LPMemorySystemTop(unsigned hhaId, string IniFilePath, string LogPath,
        HALib::Configurable* cfg) : hhaId(hhaId) {
#else
LPMemorySystemTop::LPMemorySystemTop(unsigned hhaId, string IniFilePath, string LogPath,
        int argc, char *argv[]) : hhaId(hhaId) {
#endif

#ifdef SYSARCH_PLATFORM
    IniFilename = "parameter/public.ini";
    if (hhaId == 0) DEBUG("== Loading public model file '"<<IniFilename<<"' ==");
    mininf::EmbededFile embed;
    string lpddr_pub_str = embed.getString(IniFilename);
    std::istringstream lpddr_sys_stream(lpddr_pub_str);
    IniReader::ReadIniFile(lpddr_pub_stream, true);

    SYSTEM_CONFIG = cfg->getString("SYSTEM_CONFIG");
    STATE_LOG = cfg->getBool("STATE_LOG");
    STATE_TIME = cfg->getNumber("STATE_TIME");
    DEBUG_BUS = cfg->getBool("DEBUG_BUS");
    DEBUG_STATE = cfg->getBool("DEBUG_STATE");
    DEBUG_GBUF_STATE = cfg->getBool("DEBUG_GBUF_STATE");
    DEBUG_RMW_STATE = cfg->getBool("DEBUG_RMW_STATE");
    DEBUG_PDU = cfg->getBool("DEBUG_PDU");
    ECC_MERGE_ENABLE = cfg->getBool("ECC_MERGE_ENABLE");
    DEBUG_START_TIME = cfg->getNumber("DEBUG_START_TIME");
    DEBUG_END_TIME = cfg->getNumber("DEBUG_END_TIME");
    PRINT_TRACE = cfg->getBool("PRINT_TRACE");
    PRINT_READ = cfg->getBool("PRINT_READ");
    PRINT_RDATA = cfg->getBool("PRINT_RDATA");
    PRINT_SCH = cfg->getBool("PRINT_SCH");
    PRINT_EXEC = cfg->getBool("PRINT_EXEC");
    PRINT_IDLE_LAT = cfg->getBool("PRINT_IDLE_LAT");
    PRINT_LATENCY = cfg->getBool("PRINT_LATENCY");
    PRINT_DRAM_TRACE = cfg->getBool("PRINT_DRAM_TRACE");
    PRINT_UT_TRACE = cfg->getBool("PRINT_UT_TRACE");
    PRINT_CMD_NUM = cfg->getBool("PRINT_CMD_NUM");
    PRINT_CH_OHOT = cfg->getNumber("PRINT_CH_OHOT");
    PRINT_BW = cfg->getBool("PRINT_BW");
    PRINT_BW_WIN = cfg->getNumber("PRINT_BW_WIN");
    PERFECT_DMC_EN = cfg->getBool("PERFECT_DMC_EN");
    PERFECT_DMC_DELAY = cfg->getNumber("PERFECT_DMC_DELAY");
    DROP_WRITE_CMD = cfg->getBool("DROP_WRITE_CMD");
    LAT_INC_BP = cfg->getBool("LAT_INC_BP");
    FORCE_BAINTLV_EN = cfg->getBool("FORCE_BAINTLV_EN");
    POWER_EN = cfg->getBool("POWER_EN");
    BA_SHIFT_DIR = cfg->getBool("BA_SHIFT_DIR");
    BA_SHIFT_BIT = cfg->getNumber("BA_SHIFT_BIT");
    SLOT_FIFO = cfg->getBool("SLOT_FIFO");
    TIME_ASSERT_EN = cfg->getBool("TIME_ASSERT_EN");

    IniFilename = "parameter/" + SYSTEM_CONFIG + ".ini";
    if (hhaId == 0) DEBUG("== Loading system model file '"<<IniFilename<<"' ==");
    string lpddr_sys_str = embed.getString(IniFilename);
    std::istringstream lpddr_sys_stream(lpddr_sys_str);
    IniReader::ReadIniFile(lpddr_sys_stream, true);

    // Modify Parameter From Config File
    NUM_CHANS = cfg->getNumber("NUM_CHANS");
    DDR_TYPE = cfg->getString("DDR_TYPE");
    DDR_MODE = cfg->getString("DDR_MODE");
    DMC_RATE = cfg->getNumber("DMC_RATE");
    DRAM_VENDOR = cfg->getString("DRAM_VENDOR");
    DRAM_MODE = cfg->getString("DRAM_MODE");
    TRANS_QUEUE_DEPTH = cfg->getNumber("TRANS_QUEUE_DEPTH");
    TABLE_DEPTH = cfg->getNumber("TABLE_DEPTH");
    EXEC_NUMBER = cfg->getNumber("EXEC_NUMBER");
    RD_APRE_EN = cfg->getBool("RD_APRE_EN");
    WR_APRE_EN = cfg->getBool("WR_APRE_EN");
    ENHAN_RD_AP_EN = cfg->getBool("ENHAN_RD_AP_EN");
    ENHAN_WR_AP_EN = cfg->getBool("ENHAN_WR_AP_EN");
    AREF_EN = cfg->getBool("AREF_EN");
    PBR_EN = cfg->getBool("PBR_EN");
    ENH_PBR_EN = cfg->getBool("ENH_PBR_EN");
    ENH_PBR_CEIL_EN = cfg->getBool("ENH_PBR_CEIL_EN");
    DERATING_EN = cfg->getBool("DERATING_EN");
    DERATING_RATIO = cfg->getNumber("DERATING_RATIO");
    PBR_PARA_EN = cfg->getBool("PBR_PARA_EN");
    AREF_OFFSET_EN = cfg->getBool("AREF_OFFSET_EN");
    SBR_IDLE_EN = cfg->getBool("SBR_IDLE_EN");
    SBR_REQ_MODE = cfg->getNumber("SBR_REQ_MODE");
    SBR_FRCST_NUM = cfg->getNumber("SBR_FRCST_NUM");
    SBR_WEIGHT_MODE = cfg->getNumber("SBR_WEIGHT_MODE");
    SBR_WEIGHT_ENH_MODE = cfg->getNumber("SBR_WEIGHT_ENH_MODE");
    SBR_GAP_CNT = cfg->getNumber("SBR_GAP_CNT");
    SBR_IDLE_ADAPT_EN = cfg->getBool("SBR_IDLE_ADAPT_EN");
    SBR_IDLE_ADAPT_WIN = cfg->getNumber("SBR_IDLE_ADAPT_WIN");
    SBR_IDLE_ADAPT_LEVEL = cfg->getNumber("SBR_IDLE_ADAPT_LEVEL");
    SEND_DSTREF_SERIAL = cfg->getBool("SEND_DSTREF_SERIAL");
    PD_PBR_EN = cfg->getBool("PD_PBR_EN");
    FG_REFRESH_TH = cfg->getNumber("FG_REFRESH_TH");
    FASTWAKEUP_EN = cfg->getBool("FASTWAKEUP_EN");
    PREDICT_FASTWAKEUP = cfg->getBool("PREDICT_FASTWAKEUP");
    QOS_INV = cfg->getBool("QOS_INV");
    GREEN_PATH_DIS = cfg->getBool("GREEN_PATH_DIS");
    LQOS_BP_EN = cfg->getBool("LQOS_BP_EN");
    LQOS_BP_LEVEL = cfg->getNumber("LQOS_BP_LEVEL");
    TIMEOUT_ENABLE = cfg->getBool("TIMEOUT_ENABLE");
    MAP_CONFIG["TIMEOUT_PRI_RD"] = cfg->getNumberArray("TIMEOUT_PRI_RD");
    MAP_CONFIG["TIMEOUT_PRI_WR"] = cfg->getNumberArray("TIMEOUT_PRI_WR");
    PRI_ADPT_ENABLE = cfg->getBool("PRI_ADPT_ENABLE");
    MAP_CONFIG["ADAPT_PRI_RD"] = cfg->getNumberArray("ADAPT_PRI_RD");
    MAP_CONFIG["ADAPT_PRI_WR"] = cfg->getNumberArray("ADAPT_PRI_WR");
    MPAM_MAPPING_TIMEOUT = cfg->getBool("MPAM_MAPPING_TIMEOUT");
    MAP_CONFIG["MPAM_TIMEOUT_RD"] = cfg->getNumberArray("MPAM_TIMEOUT_RD");
    MAP_CONFIG["MPAM_TIMEOUT_WR"] = cfg->getNumberArray("MPAM_TIMEOUT_WR");
    MAP_CONFIG["MPAM_ADAPT_RD"] = cfg->getNumberArray("MPAM_ADAPT_RD");
    MAP_CONFIG["MPAM_ADAPT_WR"] = cfg->getNumberArray("MPAM_ADAPT_WR");
    PRIORITY_PASS_ENABLE = cfg->getBool("PRIORITY_PASS_ENABLE");
    QOS_POLICY = cfg->getNumber("QOS_POLICY");
    RDATA_TYPE = cfg->getNumber("RDATA_TYPE");
    PAGE_ADAPT_EN = cfg->getBool("PAGE_ADAPT_EN");
    PAGE_TIMEOUT_EN = cfg->getBool("PAGE_TIMEOUT_EN");
    OPENPAGE_TIME_RD = cfg->getNumber("OPENPAGE_TIME_RD");
    OPENPAGE_TIME_WR = cfg->getNumber("OPENPAGE_TIME_WR");
    ENH_PAGE_ADPT_EN = cfg->getBool("ENH_PAGE_ADPT_EN");
    ENH_PAGE_ADPT_WIN = cfg->getNumber("ENH_PAGE_ADPT_WIN");
    MAP_CONFIG["ENH_PAGE_ADPT_LVL"] = cfg->getNumberArray("ENH_PAGE_ADPT_LVL");
    MAP_CONFIG["ENH_PAGE_ADPT_TIME"] = cfg->getNumberArray("ENH_PAGE_ADPT_TIME");
    DMC_BW_WIN = cfg->getNumber("DMC_BW_WIN");
    MAP_CONFIG["DMC_BW_LEVEL"] = cfg->getNumberArray("DMC_BW_LEVEL");
    PAGE_ADPT_EN = cfg->getBool("PAGE_ADPT_EN");
    PAGE_WIN_MODE = cfg->getNumber("PAGE_WIN_MODE");
    PAGE_WIN_SIZE = cfg->getNumber("PAGE_WIN_SIZE");
    PAGE_OPC_TH = cfg->getNumber("PAGE_OPC_TH");
    PAGE_PPC_TH = cfg->getNumber("PAGE_PPC_TH");
    PAGE_ADPT_STEP = cfg->getNumber("PAGE_ADPT_STEP");
    PAGE_TIME_MAX = cfg->getNumber("PAGE_TIME_MAX");
    DRESP_BP_TH = cfg->getNumber("DRESP_BP_TH");
    BUSYSTATE_INC_WCMD = cfg->getBool("BUSYSTATE_INC_WCMD");
    BUSYSTATE_TH = cfg->getNumber("BUSYSTATE_TH");
    IECC_ENABLE = cfg->getBool("IECC_ENABLE");
    PDU_DEPTH = cfg->getNumber("PDU_DEPTH");
    IECC_CONFLICT_CNT = cfg->getNumber("IECC_CONFLICT_CNT");
    IECC_PARTIAL_BYPASS = cfg->getBool("IECC_PARTIAL_BYPASS");
    IECC_BYPASS_ADDRESS = cfg->getNumber("IECC_BYPASS_ADDRESS");
    IECC_BL32_MODE = cfg->getBool("IECC_BL32_MODE");
    IECC_PRI = cfg->getNumber("IECC_PRI");
    IECC_CAP_RATIO = cfg->getNumber("IECC_CAP_RATIO");
    DMC_THEORY_BW = cfg->getNumber("DMC_THEORY_BW");
    FLOW_STAT_TIME = cfg->getNumber("FLOW_STAT_TIME");
    MPAM_PUSH_EN = cfg->getBool("MPAM_PUSH_EN");
    MID_PUSH_EN = cfg->getBool("MID_PUSH_EN");
    PD_ENABLE = cfg->getBool("PD_ENABLE");
    PD_PRD = cfg->getNumber("PD_PRD");
    ASREF_ENABLE = cfg->getBool("ASREF_ENABLE");
    ASREF_PRD = cfg->getNumber("ASREF_PRD");
    ASREF_ADAPT_EN = cfg->getBool("ASREF_ADAPT_EN");
    ASREF_ADAPT_WIN = cfg->getNumber("ASREF_ADAPT_WIN");
    MAP_CONFIG["ASREF_ADAPT_LEVEL"] = cfg->getNumberArray("ASREF_ADAPT_LEVEL");
    MAP_CONFIG["ASREF_ADAPT_PRD"] = cfg->getNumberArray("ASREF_ADAPT_PRD");
    BG_ROTATE_EN = cfg->getBool("BG_ROTATE_EN");
    RCMD_BANK_ARB_EN = cfg->getBool("RCMD_BANK_ARB_EN");
    WCMD_BANK_ARB_EN = cfg->getBool("WCMD_BANK_ARB_EN");
    RCMD_BANK_ARB_PRI = cfg->getNumber("RCMD_BANK_ARB_PRI");
    WCMD_BANK_ARB_PRI = cfg->getNumber("WCMD_BANK_ARB_PRI");
    WCK_ALWAYS_ON = cfg->getBool("WCK_ALWAYS_ON");
    CAS_FS_EN = cfg->getBool("CAS_FS_EN");
    CAS_FS_TH = cfg->getNumber("CAS_FS_TH");
    CORE_CONCURR = cfg->getNumber("CORE_CONCURR");
    CORE_CONCURR_PRD = cfg->getNumber("CORE_CONCURR_PRD");
    GRP_RW_EN = cfg->getBool("GRP_RW_EN");
    ENGRP_LEVEL = cfg->getNumber("ENGRP_LEVEL");
    EXGRP_LEVEL = cfg->getNumber("EXGRP_LEVEL");
    CMD_WLEVELL = cfg->getNumber("CMD_WLEVELL");
    CMD_WLEVELH = cfg->getNumber("CMD_WLEVELH");
    SERIAL_RLEVELL = cfg->getNumber("SERIAL_RLEVELL");
    SERIAL_RLEVELH = cfg->getNumber("SERIAL_RLEVELH");
    SERIAL_WLEVELL = cfg->getNumber("SERIAL_WLEVELL");
    SERIAL_WLEVELH = cfg->getNumber("SERIAL_WLEVELH");
    RLEVEL_R2W = cfg->getNumber("RLEVEL_R2W");
    ALEVEL_W2R = cfg->getNumber("ALEVEL_W2R");
    HARDWARE_RHIT_EN = cfg->getBool("HARDWARE_RHIT_EN");
    CMD_RLEVELL = cfg->getNumber("CMD_RLEVELL");
    CMD_RLEVELH = cfg->getNumber("CMD_RLEVELH");
    RHIT_RW_CMD_NUM = cfg->getNumber("RHIT_RW_CMD_NUM");
    RHIT_ACT_CMD_NUM = cfg->getNumber("RHIT_ACT_CMD_NUM");
    RWGRP_TRANS_BY_TOUT = cfg->getBool("RWGRP_TRANS_BY_TOUT");
    RWGRP_AUTO_BW = cfg->getNumber("RWGRP_AUTO_BW");
    GRP_RANK_EN = cfg->getBool("GRP_RANK_EN");
    GRP_RANK_MODE = cfg->getNumber("GRP_RANK_MODE");
    GRP_RANK_PREPB = cfg->getBool("GRP_RANK_PREPB");
    GRP_RANK_PREAB = cfg->getBool("GRP_RANK_PREAB");
    GRP_RANK_LEVEL = cfg->getNumber("GRP_RANK_LEVEL");
    SIMPLE_GRP_RANK_EN = cfg->getBool("SIMPLE_GRP_RANK_EN");
    READ_RANK_GRP_LEVEL0_H = cfg->getNumber("READ_RANK_GRP_LEVEL0_H");
    READ_RANK_GRP_LEVEL0_L = cfg->getNumber("READ_RANK_GRP_LEVEL0_L");
    READ_RANK_GRP_LEVEL1_H = cfg->getNumber("READ_RANK_GRP_LEVEL1_H");
    READ_RANK_GRP_LEVEL1_L = cfg->getNumber("READ_RANK_GRP_LEVEL1_L");
    SIMPLE_GRP_SID_EN = cfg->getBool("SIMPLE_GRP_SID_EN");
    MAP_CONFIG["GRP_SID_LEVEL"] = cfg->getNumberArray("GRP_SID_LEVEL");
    RCMD_HQOS_EN = cfg->getBool("RCMD_HQOS_EN");
    RCMD_HQOS_LEVEL = cfg->getNumber("RCMD_HQOS_LEVEL");
    SWITCH_HQOS_LEVEL = cfg->getNumber("SWITCH_HQOS_LEVEL");
    CPU_MID_START = cfg->getNumber("CPU_MID_START");
    CPU_MID_END = cfg->getNumber("CPU_MID_END");
    RCMD_HQOS_W2R_SWITCH_EN = cfg->getBool("RCMD_HQOS_W2R_SWITCH_EN");
    RCMD_HQOS_W2R_RLEVELH = cfg->getNumber("RCMD_HQOS_W2R_RLEVELH");
    RCMD_HQOS_RANK_SWITCH_EN = cfg->getBool("RCMD_HQOS_RANK_SWITCH_EN");
    RCMD_HQOS_RANK_SWITCH_LEVELH = cfg->getNumber("RCMD_HQOS_RANK_SWITCH_LEVELH");
    RCMD_HQOS_RANK_SWITCH_LEVELL = cfg->getNumber("RCMD_HQOS_RANK_SWITCH_LEVELL");
    RCMD_HQOS_RANK_SWITCH_ROWHIT_LEVEL = cfg->getNumber("RCMD_HQOS_RANK_SWITCH_ROWHIT_LEVEL");
    RHIT_HQOS_BREAK_EN = cfg->getBool("RHIT_HQOS_BREAK_EN");
    RHIT_HQOS_BREAK_OTH_RCMD_LEVEL = cfg->getNumber("RHIT_HQOS_BREAK_OTH_RCMD_LEVEL");
    BYP_ACT_EN = cfg->getBool("BYP_ACT_EN");
    RHIT_BREAK_EN = cfg->getBool("RHIT_BREAK_EN");
    RHIT_BREAK_LEVEL = cfg->getNumber("RHIT_BREAK_LEVEL");
    RW_GRPCHG_W2R_TH = cfg->getNumber("RW_GRPCHG_W2R_TH");
    RW_GRPCHG_R2W_TH = cfg->getNumber("RW_GRPCHG_R2W_TH");
    DMC_V580 = cfg->getBool("DMC_V580");
    DMC_V590 = cfg->getBool("DMC_V590");
    DMC_V596 = cfg->getBool("DMC_V596");
    EARLY_TCCDL_EN = cfg->getBool("EARLY_TCCDL_EN");
    TRFC_CC_EN = cfg->getBool("TRFC_CC_EN");
    WB_WDEPTH = cfg->getNumber("WB_WDEPTH");
    WB_RDEPTH = cfg->getNumber("WB_RDEPTH");
    MERGE_ENABLE = cfg->getBool("MERGE_ENABLE");
    FORWARD_ENABLE = cfg->getBool("FORWARD_ENABLE");
    DYN_BYP_EN = cfg->getBool("DYN_BYP_EN");
    GBUF_RCMD_BLOCK_PBR = cfg->getBool("GBUF_RCMD_BLOCK_PBR");
    GBUF_RD_BRK_WFLUSH_CMD = cfg->getBool("GBUF_RD_BRK_WFLUSH_CMD");
    GBUF_RCMD_TIMEOUT = cfg->getNumber("GBUF_RCMD_TIMEOUT");
    GBUF_WCMD_PRIADAPT = cfg->getNumber("GBUF_WCMD_PRIADAPT");
    RQ_RNKCONF_EN = cfg->getBool("RQ_RNKCONF_EN");
    RQ_ADCONF_PUSH_EN = cfg->getBool("RQ_ADCONF_PUSH_EN");
    CMD_ROW_ORDER = cfg->getNumber("CMD_ROW_ORDER");
    TOTAL_RCMD_MODE = cfg->getNumber("TOTAL_RCMD_MODE");
    RQ_BACONF_MODE = cfg->getNumber("RQ_BACONF_MODE");
    WQ_BACONF_MODE = cfg->getNumber("WQ_BACONF_MODE");
    RO_HIT_EN = cfg->getBool("RO_HIT_EN");
    MAP_CONFIG["GRP_MODE_LEVEL0"] = cfg->getNumberArray("GRP_MODE_LEVEL0");
    MAP_CONFIG["GRP_MODE_LEVEL1"] = cfg->getNumberArray("GRP_MODE_LEVEL1");
    MAP_CONFIG["GRP_MODE_LEVEL2"] = cfg->getNumberArray("GRP_MODE_LEVEL2");
    MAP_CONFIG["GRP_MODE_LEVEL3"] = cfg->getNumberArray("GRP_MODE_LEVEL3");
    MAP_CONFIG["BANK_CMD_TH"] = cfg->getNumberArray("BANK_CMD_TH");
    MAP_CONFIG["NO_CMD_SCH_TH"] = cfg->getNumberArray("NO_CMD_SCH_TH");
    MAP_CONFIG["WGRP_RANK_MODE"] = cfg->getNumberArray("WGRP_RANK_MODE");
    MAP_CONFIG["WR_LEVEL0"] = cfg->getNumberArray("WR_LEVEL0");
    MAP_CONFIG["WR_LEVEL1"] = cfg->getNumberArray("WR_LEVEL1");
    MAP_CONFIG["WR_LEVEL2"] = cfg->getNumberArray("WR_LEVEL2");
    MAP_CONFIG["WR_LEVEL3"] = cfg->getNumberArray("WR_LEVEL3");
    MAP_CONFIG["WR_MOST_LEVEL0"] = cfg->getNumberArray("WR_MOST_LEVEL0");
    MAP_CONFIG["WR_MOST_LEVEL1"] = cfg->getNumberArray("WR_MOST_LEVEL1");
    MAP_CONFIG["WR_MOST_LEVEL2"] = cfg->getNumberArray("WR_MOST_LEVEL2");
    MAP_CONFIG["WR_MOST_LEVEL3"] = cfg->getNumberArray("WR_MOST_LEVEL3");
    MAP_CONFIG["RD_LEVEL0"] = cfg->getNumberArray("RD_LEVEL0");
    MAP_CONFIG["RD_LEVEL1"] = cfg->getNumberArray("RD_LEVEL1");
    MAP_CONFIG["RD_LEVEL2"] = cfg->getNumberArray("RD_LEVEL2");
    MAP_CONFIG["RD_LEVEL3"] = cfg->getNumberArray("RD_LEVEL3");
    WRITE_BUFFER_ENABLE = cfg->getBool("WRITE_BUFFER_ENABLE");
    EM_ENABLE = cfg->getBool("EM_ENABLE");
    EM_MODE = cfg->getNumber("EM_MODE");
    RMW_ENABLE = cfg->getBool("RMW_ENABLE");
    RMW_CMD_MODE = cfg->getNumber("RMW_CMD_MODE");
    RMW_QUE_DEPTH = cfg->getNumber("RMW_QUE_DEPTH");
    RMW_CONF_SIZE = cfg->getNumber("RMW_CONF_SIZE");
    GRT_FIFO_DEPTH = cfg->getNumber("GRT_FIFO_DEPTH");
    TRANS_FIFO_DEPTH = cfg->getNumber("TRANS_FIFO_DEPTH");
    ROW_SEL = cfg->getNumber("ROW_SEL");
    WCMD_MERGE_EN = cfg->getBool("WCMD_MERGE_EN");
    WRITE_MERGE_BUFFER_DEPTH = cfg->getNumber("WRITE_MERGE_BUFFER_DEPTH");
    UNPAIRED_TO_RMW_EN = cfg->getBool("UNPAIRED_TO_RMW_EN");

    IniFilename = "parameter/" + DDR_TYPE + "_" + to_string(DMC_RATE) + "M" + DDR_MODE + ".ini";
    if (hhaId == 0) DEBUG("== Loading device model file '"<<IniFilename<<"' ==");
    string lpddr_dev_str = embed.getString(IniFilename);
    std::istringstream lpddr_dev_stream(lpddr_dev_str);
    IniReader::ReadIniFile(lpddr_dev_stream, false);

    DMC_DATA_BUS_BITS = cfg->getNumber("DMC_DATA_BUS_BITS");

    CalcMatrixNum();
    IniFilename = "parameter/" + DRAM_VENDOR + "_" + to_string(DRAM_CAPACITY) +
            "gb_" + to_string(DMC_RATE) + "M" + DRAM_MODE + ".ini";
    if (hhaId == 0) DEBUG("== Loading DRAM power model file '"<<IniFilename<<"' ==");
    string drampower_str = embed.getString(IniFilename);
    std::istringstream drampower_stream(drampower_str);
    IniReader::ReadIniFile(drampower_stream, false);
#else
    IniFilename = IniFilePath + "/public.ini";
    if (hhaId == 0) DEBUG("== Loading public model file '"<<IniFilename<<"' ==");
    ifstream iniFile;
    iniFile.open(IniFilename.c_str());
    if (iniFile.is_open()) {
        IniReader::ReadIniFile(iniFile, true);
    } else {
        ERROR("Unable to load ini file "<<IniFilename);
        abort();
    }
    iniFile.close();

    Configurable cfg;
    for (int i = 1; i < argc; i ++) {
        cfg.getString(argv[i]);
    }

    GET_PARAM(SYSTEM_CONFIG, "SYSTEM_CONFIG", get);
    GET_PARAM(STATE_LOG, "STATE_LOG", getBool);
    GET_PARAM(STATE_TIME, "STATE_TIME", getUint);
    GET_PARAM(DEBUG_BUS, "DEBUG_BUS", getBool);
    GET_PARAM(DEBUG_STATE, "DEBUG_STATE", getBool);
    GET_PARAM(DEBUG_GBUF_STATE, "DEBUG_GBUF_STATE", getBool);
    GET_PARAM(DEBUG_RMW_STATE, "DEBUG_RMW_STATE", getBool);
    GET_PARAM(DEBUG_PDU, "DEBUG_PDU", getBool);
    GET_PARAM(ECC_MERGE_ENABLE, "ECC_MERGE_ENABLE", getBool);
    GET_PARAM(DEBUG_START_TIME, "DEBUG_START_TIME", getUint64);
    GET_PARAM(DEBUG_END_TIME, "DEBUG_END_TIME", getUint64);
    GET_PARAM(PRINT_TRACE, "PRINT_TRACE", getBool);
    GET_PARAM(PRINT_READ, "PRINT_READ", getBool);
    GET_PARAM(PRINT_RDATA, "PRINT_RDATA", getBool);
    GET_PARAM(PRINT_SCH, "PRINT_SCH", getBool);
    GET_PARAM(PRINT_EXEC, "PRINT_EXEC", getBool);
    GET_PARAM(PRINT_IDLE_LAT, "PRINT_IDLE_LAT", getBool);
    GET_PARAM(PRINT_LATENCY, "PRINT_LATENCY", getBool);
    GET_PARAM(PRINT_DRAM_TRACE, "PRINT_DRAM_TRACE", getBool);
    GET_PARAM(PRINT_UT_TRACE, "PRINT_UT_TRACE", getBool);
    GET_PARAM(PRINT_CMD_NUM, "PRINT_CMD_NUM", getBool);
    GET_PARAM(PRINT_CH_OHOT, "PRINT_CH_OHOT", getUint64);
    GET_PARAM(PRINT_BW, "PRINT_BW", getBool);
    GET_PARAM(PRINT_BW_WIN, "PRINT_BW_WIN", getUint);
    GET_PARAM(PERFECT_DMC_EN, "PERFECT_DMC_EN", getBool);
    GET_PARAM(PERFECT_DMC_DELAY, "PERFECT_DMC_DELAY", getUint);
    GET_PARAM(DROP_WRITE_CMD, "DROP_WRITE_CMD", getBool);
    GET_PARAM(LAT_INC_BP, "LAT_INC_BP", getBool);
    GET_PARAM(FORCE_BAINTLV_EN, "FORCE_BAINTLV_EN", getBool);
    GET_PARAM(POWER_EN, "POWER_EN", getBool);
    GET_PARAM(BA_SHIFT_DIR, "BA_SHIFT_DIR", getBool);
    GET_PARAM(BA_SHIFT_BIT, "BA_SHIFT_BIT", getUint);
    GET_PARAM(SLOT_FIFO, "SLOT_FIFO", getBool);
    GET_PARAM(TIME_ASSERT_EN, "TIME_ASSERT_EN", getBool);

    IniFilename = IniFilePath + "/" + SYSTEM_CONFIG + ".ini";
    if (hhaId == 0) DEBUG("== Loading system model file '"<<IniFilename<<"' ==");
    iniFile.open(IniFilename.c_str());
    if (iniFile.is_open()) {
        IniReader::ReadIniFile(iniFile, true);
    } else {
        ERROR("Unable to load ini file "<<IniFilename);
        abort();
    }
    iniFile.close();

    GET_PARAM(NUM_CHANS, "NUM_CHANS", getUint);
    GET_PARAM(DDR_TYPE, "DDR_TYPE", get);
    GET_PARAM(DDR_MODE, "DDR_MODE", get);
    GET_PARAM(DMC_RATE, "DMC_RATE", getUint);
    GET_PARAM(DRAM_VENDOR, "DRAM_VENDOR", get);
    GET_PARAM(DRAM_MODE, "DRAM_MODE", get);
    GET_PARAM(TRANS_QUEUE_DEPTH, "TRANS_QUEUE_DEPTH", getUint);
    GET_PARAM(TABLE_DEPTH, "TABLE_DEPTH", getUint);
    GET_PARAM(EXEC_NUMBER, "EXEC_NUMBER", getUint);
    GET_PARAM(RD_APRE_EN, "RD_APRE_EN", getBool);
    GET_PARAM(WR_APRE_EN, "WR_APRE_EN", getBool);
    GET_PARAM(ENHAN_RD_AP_EN, "ENHAN_RD_AP_EN", getBool);
    GET_PARAM(ENHAN_WR_AP_EN, "ENHAN_WR_AP_EN", getBool);
    GET_PARAM(AREF_EN, "AREF_EN", getBool);
    GET_PARAM(PBR_EN, "PBR_EN", getBool);
    GET_PARAM(ENH_PBR_EN, "ENH_PBR_EN", getBool);
    GET_PARAM(ENH_PBR_CEIL_EN, "ENH_PBR_CEIL_EN", getBool);
    GET_PARAM(DERATING_EN, "DERATING_EN", getBool);
    GET_PARAM(DERATING_RATIO, "DERATING_RATIO", getUint);
    GET_PARAM(PBR_PARA_EN, "PBR_PARA_EN", getBool);
    GET_PARAM(AREF_OFFSET_EN, "AREF_OFFSET_EN", getBool);
    GET_PARAM(SBR_IDLE_EN, "SBR_IDLE_EN", getBool);
    GET_PARAM(SBR_REQ_MODE, "SBR_REQ_MODE", getUint);
    GET_PARAM(SBR_FRCST_NUM, "SBR_FRCST_NUM", getUint);
    GET_PARAM(SBR_WEIGHT_MODE, "SBR_WEIGHT_MODE", getUint);
    GET_PARAM(SBR_WEIGHT_ENH_MODE, "SBR_WEIGHT_ENH_MODE", getUint);
    GET_PARAM(SBR_GAP_CNT, "SBR_GAP_CNT", getUint);
    GET_PARAM(SBR_IDLE_ADAPT_EN, "SBR_IDLE_ADAPT_EN", getBool);
    GET_PARAM(SBR_IDLE_ADAPT_WIN, "SBR_IDLE_ADAPT_WIN", getUint);
    GET_PARAM(SBR_IDLE_ADAPT_LEVEL, "SBR_IDLE_ADAPT_LEVEL", getUint);
    GET_PARAM(SEND_DSTREF_SERIAL, "SEND_DSTREF_SERIAL", getBool);
    GET_PARAM(PD_PBR_EN, "PD_PBR_EN", getBool);
    GET_PARAM(FG_REFRESH_TH, "FG_REFRESH_TH", getUint);
    GET_PARAM(FASTWAKEUP_EN, "FASTWAKEUP_EN", getBool);
    GET_PARAM(PREDICT_FASTWAKEUP, "PREDICT_FASTWAKEUP", getBool);
    GET_PARAM(QOS_INV, "QOS_INV", getBool);
    GET_PARAM(GREEN_PATH_DIS, "GREEN_PATH_DIS", getBool);
    GET_PARAM(LQOS_BP_EN, "LQOS_BP_EN", getBool);
    GET_PARAM(LQOS_BP_LEVEL, "LQOS_BP_LEVEL", getUint);
    GET_PARAM(TIMEOUT_ENABLE, "TIMEOUT_ENABLE", getBool);
    GET_PARAM(MAP_CONFIG["TIMEOUT_PRI_RD"], "TIMEOUT_PRI_RD", getUintArray);
    GET_PARAM(MAP_CONFIG["TIMEOUT_PRI_WR"], "TIMEOUT_PRI_WR", getUintArray);
    GET_PARAM(PRI_ADPT_ENABLE, "PRI_ADPT_ENABLE", getBool);
    GET_PARAM(MAP_CONFIG["ADAPT_PRI_RD"], "ADAPT_PRI_RD", getUintArray);
    GET_PARAM(MAP_CONFIG["ADAPT_PRI_WR"], "ADAPT_PRI_WR", getUintArray);
    GET_PARAM(MPAM_MAPPING_TIMEOUT, "MPAM_MAPPING_TIMEOUT", getBool);
    GET_PARAM(MAP_CONFIG["MPAM_TIMEOUT_RD"], "MPAM_TIMEOUT_RD", getUintArray);
    GET_PARAM(MAP_CONFIG["MPAM_TIMEOUT_WR"], "MPAM_TIMEOUT_WR", getUintArray);
    GET_PARAM(MAP_CONFIG["MPAM_ADAPT_RD"], "MPAM_ADAPT_RD", getUintArray);
    GET_PARAM(MAP_CONFIG["MPAM_ADAPT_WR"], "MPAM_ADAPT_WR", getUintArray);
    GET_PARAM(PRIORITY_PASS_ENABLE, "PRIORITY_PASS_ENABLE", getBool);
    GET_PARAM(QOS_POLICY, "QOS_POLICY", getUint);
    GET_PARAM(TIMEOUT_MODE, "TIMEOUT_MODE", getUint);
    GET_PARAM(RDATA_TYPE, "RDATA_TYPE", getUint);
    GET_PARAM(PAGE_ADAPT_EN, "PAGE_ADAPT_EN", getBool);
    GET_PARAM(PAGE_TIMEOUT_EN, "PAGE_TIMEOUT_EN", getBool);
    GET_PARAM(OPENPAGE_TIME_RD, "OPENPAGE_TIME_RD", getUint);
    GET_PARAM(OPENPAGE_TIME_WR, "OPENPAGE_TIME_WR", getUint);
    GET_PARAM(ENH_PAGE_ADPT_EN, "ENH_PAGE_ADPT_EN", getBool);
    GET_PARAM(ENH_PAGE_ADPT_WIN, "ENH_PAGE_ADPT_WIN", getUint);
    GET_PARAM(MAP_CONFIG["ENH_PAGE_ADPT_LVL"], "ENH_PAGE_ADPT_LVL", getUintArray);
    GET_PARAM(MAP_CONFIG["ENH_PAGE_ADPT_TIME"], "ENH_PAGE_ADPT_TIME", getUintArray);
    GET_PARAM(DMC_BW_WIN, "DMC_BW_WIN", getUint);
    GET_PARAM(MAP_CONFIG["DMC_BW_LEVEL"], "DMC_BW_LEVEL", getUintArray);
    GET_PARAM(PAGE_ADPT_EN, "PAGE_ADPT_EN", getBool);
    GET_PARAM(PAGE_WIN_MODE, "PAGE_WIN_MODE", getUint);
    GET_PARAM(PAGE_WIN_SIZE, "PAGE_WIN_SIZE", getUint);
    GET_PARAM(PAGE_OPC_TH, "PAGE_OPC_TH", getUint);
    GET_PARAM(PAGE_PPC_TH, "PAGE_PPC_TH", getUint);
    GET_PARAM(PAGE_ADPT_STEP, "PAGE_ADPT_STEP", getUint);
    GET_PARAM(PAGE_TIME_MAX, "PAGE_TIME_MAX", getUint);
    GET_PARAM(DRESP_BP_TH, "DRESP_BP_TH", getUint);
    GET_PARAM(BUSYSTATE_INC_WCMD, "BUSYSTATE_INC_WCMD", getBool);
    GET_PARAM(BUSYSTATE_TH, "BUSYSTATE_TH", getUint);
    GET_PARAM(IECC_ENABLE, "IECC_ENABLE", getBool);
    GET_PARAM(PDU_DEPTH, "PDU_DEPTH", getUint);
    GET_PARAM(IECC_CONFLICT_CNT, "IECC_CONFLICT_CNT", getUint);
    GET_PARAM(IECC_PARTIAL_BYPASS, "IECC_PARTIAL_BYPASS", getBool);
    GET_PARAM(IECC_BYPASS_ADDRESS, "IECC_BYPASS_ADDRESS", getUint);
    GET_PARAM(IECC_BL32_MODE, "IECC_BL32_MODE", getBool);
    GET_PARAM(IECC_PRI, "IECC_PRI", getUint);
    GET_PARAM(IECC_CAP_RATIO, "IECC_CAP_RATIO", getUint);
    GET_PARAM(DMC_THEORY_BW, "DMC_THEORY_BW", getUint);
    GET_PARAM(FLOW_STAT_TIME, "FLOW_STAT_TIME", getUint);
    GET_PARAM(MPAM_PUSH_EN, "MPAM_PUSH_EN", getBool);
    GET_PARAM(MID_PUSH_EN, "MID_PUSH_EN", getBool);
    GET_PARAM(PD_ENABLE, "PD_ENABLE", getBool);
    GET_PARAM(PD_PRD, "PD_PRD", getUint);
    GET_PARAM(ASREF_ENABLE, "ASREF_ENABLE", getBool);
    GET_PARAM(ASREF_PRD, "ASREF_PRD", getUint);
    GET_PARAM(ASREF_ADAPT_EN, "ASREF_ADAPT_EN", getBool);
    GET_PARAM(ASREF_ADAPT_WIN, "ASREF_ADAPT_WIN", getUint);
    GET_PARAM(MAP_CONFIG["ASREF_ADAPT_LEVEL"], "ASREF_ADAPT_LEVEL", getUintArray);
    GET_PARAM(MAP_CONFIG["ASREF_ADAPT_PRD"], "ASREF_ADAPT_PRD", getUintArray);
    GET_PARAM(BG_ROTATE_EN, "BG_ROTATE_EN", getBool);
    GET_PARAM(RCMD_BANK_ARB_EN, "RCMD_BANK_ARB_EN", getBool);
    GET_PARAM(WCMD_BANK_ARB_EN, "WCMD_BANK_ARB_EN", getBool);
    GET_PARAM(RCMD_BANK_ARB_PRI, "RCMD_BANK_ARB_PRI", getUint);
    GET_PARAM(WCMD_BANK_ARB_PRI, "WCMD_BANK_ARB_PRI", getUint);
    GET_PARAM(WCK_ALWAYS_ON, "WCK_ALWAYS_ON", getBool);
    GET_PARAM(CAS_FS_EN, "CAS_FS_EN", getBool);
    GET_PARAM(CAS_FS_TH, "CAS_FS_TH", getUint);
    GET_PARAM(CORE_CONCURR, "CORE_CONCURR", getUint);
    GET_PARAM(CORE_CONCURR_PRD, "CORE_CONCURR_PRD", getUint);
    GET_PARAM(GRP_RW_EN, "GRP_RW_EN", getBool);
    GET_PARAM(ENGRP_LEVEL, "ENGRP_LEVEL", getUint);
    GET_PARAM(EXGRP_LEVEL, "EXGRP_LEVEL", getUint);
    GET_PARAM(CMD_WLEVELL, "CMD_WLEVELL", getUint);
    GET_PARAM(CMD_WLEVELH, "CMD_WLEVELH", getUint);
    GET_PARAM(SERIAL_RLEVELL, "SERIAL_RLEVELL", getUint);
    GET_PARAM(SERIAL_RLEVELH, "SERIAL_RLEVELH", getUint);
    GET_PARAM(SERIAL_WLEVELL, "SERIAL_WLEVELL", getUint);
    GET_PARAM(SERIAL_WLEVELH, "SERIAL_WLEVELH", getUint);
    GET_PARAM(RLEVEL_R2W, "RLEVEL_R2W", getUint);
    GET_PARAM(ALEVEL_W2R, "ALEVEL_W2R", getUint);
    GET_PARAM(HARDWARE_RHIT_EN, "HARDWARE_RHIT_EN", getBool);
    GET_PARAM(CMD_RLEVELL, "CMD_RLEVELL", getUint);
    GET_PARAM(CMD_RLEVELH, "CMD_RLEVELH", getUint);
    GET_PARAM(RHIT_RW_CMD_NUM, "RHIT_RW_CMD_NUM", getUint);
    GET_PARAM(RHIT_ACT_CMD_NUM, "RHIT_ACT_CMD_NUM", getUint);
    GET_PARAM(RWGRP_TRANS_BY_TOUT, "RWGRP_TRANS_BY_TOUT", getBool);
    GET_PARAM(RWGRP_AUTO_BW, "RWGRP_AUTO_BW", getUint);
    GET_PARAM(GRP_RANK_EN, "GRP_RANK_EN", getBool);
    GET_PARAM(GRP_RANK_MODE, "GRP_RANK_MODE", getUint);
    GET_PARAM(GRP_RANK_PREPB, "GRP_RANK_PREPB", getBool);
    GET_PARAM(GRP_RANK_PREAB, "GRP_RANK_PREAB", getBool);
    GET_PARAM(GRP_RANK_LEVEL, "GRP_RANK_LEVEL", getUint);
    GET_PARAM(SIMPLE_GRP_RANK_EN, "SIMPLE_GRP_RANK_EN", getBool);
    GET_PARAM(READ_RANK_GRP_LEVEL0_H, "READ_RANK_GRP_LEVEL0_H", getUint);
    GET_PARAM(READ_RANK_GRP_LEVEL0_L, "READ_RANK_GRP_LEVEL0_L", getUint);
    GET_PARAM(READ_RANK_GRP_LEVEL1_H, "READ_RANK_GRP_LEVEL1_H", getUint);
    GET_PARAM(READ_RANK_GRP_LEVEL1_L, "READ_RANK_GRP_LEVEL1_L", getUint);
    GET_PARAM(SIMPLE_GRP_SID_EN, "SIMPLE_GRP_SID_EN", getBool);
    GET_PARAM(MAP_CONFIG["GRP_SID_LEVEL"], "GRP_SID_LEVEL", getUintArray);
    GET_PARAM(RCMD_HQOS_EN, "RCMD_HQOS_EN", getBool);
    GET_PARAM(RCMD_HQOS_LEVEL, "RCMD_HQOS_LEVEL", getUint);
    GET_PARAM(SWITCH_HQOS_LEVEL, "SWITCH_HQOS_LEVEL", getUint);
    GET_PARAM(CPU_MID_START, "CPU_MID_START", getUint);
    GET_PARAM(CPU_MID_END, "CPU_MID_END", getUint);
    GET_PARAM(RCMD_HQOS_W2R_SWITCH_EN, "RCMD_HQOS_W2R_SWITCH_EN", getBool);
    GET_PARAM(RCMD_HQOS_W2R_RLEVELH, "RCMD_HQOS_W2R_RLEVELH", getUint);
    GET_PARAM(RCMD_HQOS_RANK_SWITCH_EN, "RCMD_HQOS_RANK_SWITCH_EN", getBool);
    GET_PARAM(RCMD_HQOS_RANK_SWITCH_LEVELH, "RCMD_HQOS_RANK_SWITCH_LEVELH", getUint);
    GET_PARAM(RCMD_HQOS_RANK_SWITCH_LEVELL, "RCMD_HQOS_RANK_SWITCH_LEVELL", getUint);
    GET_PARAM(RCMD_HQOS_RANK_SWITCH_ROWHIT_LEVEL, "RCMD_HQOS_RANK_SWITCH_ROWHIT_LEVEL", getUint);
    GET_PARAM(RHIT_HQOS_BREAK_EN, "RHIT_HQOS_BREAK_EN", getBool);
    GET_PARAM(RHIT_HQOS_BREAK_OTH_RCMD_LEVEL, "RHIT_HQOS_BREAK_OTH_RCMD_LEVEL", getUint);
    GET_PARAM(BYP_ACT_EN, "BYP_ACT_EN", getBool);
    GET_PARAM(RHIT_BREAK_EN, "RHIT_BREAK_EN", getBool);
    GET_PARAM(RHIT_BREAK_LEVEL, "RHIT_BREAK_LEVEL", getUint);
    GET_PARAM(RW_GRPCHG_W2R_TH, "RW_GRPCHG_W2R_TH", getUint);
    GET_PARAM(RW_GRPCHG_R2W_TH, "RW_GRPCHG_R2W_TH", getUint);
    GET_PARAM(DMC_V580, "DMC_V580", getBool);
    GET_PARAM(DMC_V590, "DMC_V590", getBool);
    GET_PARAM(DMC_V596, "DMC_V596", getBool);
    GET_PARAM(EARLY_TCCDL_EN, "EARLY_TCCDL_EN", getBool);
    GET_PARAM(TRFC_CC_EN, "TRFC_CC_EN", getBool);
    GET_PARAM(WB_WDEPTH, "WB_WDEPTH", getUint);
    GET_PARAM(WB_RDEPTH, "WB_RDEPTH", getUint);
    GET_PARAM(MERGE_ENABLE, "MERGE_ENABLE", getBool);
    GET_PARAM(FORWARD_ENABLE, "FORWARD_ENABLE", getBool);
    GET_PARAM(DYN_BYP_EN, "DYN_BYP_EN", getBool);
    GET_PARAM(GBUF_RCMD_BLOCK_PBR, "GBUF_RCMD_BLOCK_PBR", getBool);
    GET_PARAM(GBUF_RD_BRK_WFLUSH_CMD, "GBUF_RD_BRK_WFLUSH_CMD", getBool);
    GET_PARAM(GBUF_RCMD_TIMEOUT, "GBUF_RCMD_TIMEOUT", getUint);
    GET_PARAM(GBUF_WCMD_PRIADAPT, "GBUF_WCMD_PRIADAPT", getUint);
    GET_PARAM(RQ_RNKCONF_EN, "RQ_RNKCONF_EN", getBool);
    GET_PARAM(RQ_ADCONF_PUSH_EN, "RQ_ADCONF_PUSH_EN", getBool);
    GET_PARAM(CMD_ROW_ORDER, "CMD_ROW_ORDER", getUint);
    GET_PARAM(TOTAL_RCMD_MODE, "TOTAL_RCMD_MODE", getUint);
    GET_PARAM(RQ_BACONF_MODE, "RQ_BACONF_MODE", getUint);
    GET_PARAM(WQ_BACONF_MODE, "WQ_BACONF_MODE", getUint);
    GET_PARAM(RO_HIT_EN, "RO_HIT_EN", getBool);
    GET_PARAM(MAP_CONFIG["GRP_MODE_LEVEL0"], "GRP_MODE_LEVEL0", getUintArray);
    GET_PARAM(MAP_CONFIG["GRP_MODE_LEVEL1"], "GRP_MODE_LEVEL1", getUintArray);
    GET_PARAM(MAP_CONFIG["GRP_MODE_LEVEL2"], "GRP_MODE_LEVEL2", getUintArray);
    GET_PARAM(MAP_CONFIG["GRP_MODE_LEVEL3"], "GRP_MODE_LEVEL3", getUintArray);
    GET_PARAM(MAP_CONFIG["BANK_CMD_TH"], "BANK_CMD_TH", getUintArray);
    GET_PARAM(MAP_CONFIG["NO_CMD_SCH_TH"], "NO_CMD_SCH_TH", getUintArray);
    GET_PARAM(MAP_CONFIG["WGRP_RANK_MODE"], "WGRP_RANK_MODE", getUintArray);
    GET_PARAM(MAP_CONFIG["WR_LEVEL0"], "WR_LEVEL0", getUintArray);
    GET_PARAM(MAP_CONFIG["WR_LEVEL1"], "WR_LEVEL1", getUintArray);
    GET_PARAM(MAP_CONFIG["WR_LEVEL2"], "WR_LEVEL2", getUintArray);
    GET_PARAM(MAP_CONFIG["WR_LEVEL3"], "WR_LEVEL3", getUintArray);
    GET_PARAM(MAP_CONFIG["WR_MOST_LEVEL0"], "WR_MOST_LEVEL0", getUintArray);
    GET_PARAM(MAP_CONFIG["WR_MOST_LEVEL1"], "WR_MOST_LEVEL1", getUintArray);
    GET_PARAM(MAP_CONFIG["WR_MOST_LEVEL2"], "WR_MOST_LEVEL2", getUintArray);
    GET_PARAM(MAP_CONFIG["WR_MOST_LEVEL3"], "WR_MOST_LEVEL3", getUintArray);
    GET_PARAM(MAP_CONFIG["RD_LEVEL0"], "RD_LEVEL0", getUintArray);
    GET_PARAM(MAP_CONFIG["RD_LEVEL1"], "RD_LEVEL1", getUintArray);
    GET_PARAM(MAP_CONFIG["RD_LEVEL2"], "RD_LEVEL2", getUintArray);
    GET_PARAM(MAP_CONFIG["RD_LEVEL3"], "RD_LEVEL3", getUintArray);
    GET_PARAM(WRITE_BUFFER_ENABLE, "WRITE_BUFFER_ENABLE", getBool);
    GET_PARAM(EM_ENABLE, "EM_ENABLE", getBool);
    GET_PARAM(EM_MODE, "EM_MODE", getUint);
    GET_PARAM(RMW_ENABLE, "RMW_ENABLE", getBool);
    GET_PARAM(RMW_CMD_MODE, "RMW_CMD_MODE", getUint);
    GET_PARAM(RMW_QUE_DEPTH, "RMW_QUE_DEPTH", getUint);
    GET_PARAM(RMW_CONF_SIZE, "RMW_CONF_SIZE", getUint);
    GET_PARAM(GRT_FIFO_DEPTH, "GRT_FIFO_DEPTH", getUint);
    GET_PARAM(TRANS_FIFO_DEPTH, "TRANS_FIFO_DEPTH", getUint);
    GET_PARAM(ROW_SEL, "ROW_SEL", getUint);
    GET_PARAM(WCMD_MERGE_EN, "WCMD_MERGE_EN", getBool);
    GET_PARAM(WRITE_MERGE_BUFFER_DEPTH, "WRITE_MERGE_BUFFER_DEPTH", getUint);
    GET_PARAM(UNPAIRED_TO_RMW_EN, "UNPAIRED_TO_RMW_EN", getBool);

    IniFilename = IniFilePath+"/"+DDR_TYPE+"_"+to_string(DMC_RATE)+"M"+DDR_MODE+".ini";
    if (hhaId == 0) DEBUG("== Loading device model file '"<<IniFilename<<"' ==");
    iniFile.open(IniFilename.c_str());
    if (iniFile.is_open()) {
        IniReader::ReadIniFile(iniFile, false);
    } else {
        ERROR("Unable to load ini file "<<IniFilename);
        abort();
    }
    iniFile.close();

    //TimingInit();
    //TimingCalc();

    GET_PARAM(JEDEC_DATA_BUS_BITS, "JEDEC_DATA_BUS_BITS", getUint);
    GET_PARAM(DMC_DATA_BUS_BITS, "DMC_DATA_BUS_BITS", getUint);
    GET_PARAM(WCK2DFI_RATIO, "WCK2DFI_RATIO", getFloat);
    GET_PARAM(OFREQ_EN, "OFREQ_EN", getBool);
    GET_PARAM(OFREQ_RATIO, "OFREQ_RATIO", getFloat);
    GET_PARAM(PAM_RATIO, "PAM_RATIO", getFloat);
    GET_PARAM(tDFI, "tDFI", getFloat);
    GET_PARAM(MAP_CONFIG["BL"], "BL", getUintArray);
    GET_PARAM(tREFI, "tREFI", getUint);
    GET_PARAM(WL, "WL", getUint);
    GET_PARAM(RL, "RL", getUint);
    GET_PARAM(tRAS, "tRAS", getUint);
    GET_PARAM(tRCD_WR, "tRCD_WR", getUint);
    GET_PARAM(tRCD, "tRCD", getUint);
    GET_PARAM(tRRD_L, "tRRD_L", getUint);
    GET_PARAM(tRRD_S, "tRRD_S", getUint);
    GET_PARAM(tRPpb, "tRPpb", getUint);
    GET_PARAM(tRPab, "tRPab", getUint);
    GET_PARAM(tRPfg, "tRPfg", getUint);
    GET_PARAM(tPPD, "tPPD", getUint);
    GET_PARAM(tPPD_L, "tPPD_L", getUint);
    GET_PARAM(tFPW, "tFPW", getUint);
    GET_PARAM(tCCD_S, "tCCD_S", getUint);
    GET_PARAM(tCCD_L, "tCCD_L", getUint);
    GET_PARAM(tCCD_R, "tCCD_R", getUint);
    GET_PARAM(tCCD_M, "tCCD_M", getUint);
    GET_PARAM(tCCD_M_WR, "tCCD_M_WR", getUint);
    GET_PARAM(tCCD_L_WR, "tCCD_L_WR", getUint);
    GET_PARAM(tCCD_L_WR2, "tCCD_L_WR2", getUint);
    GET_PARAM(tCCD_NSR, "tCCD_NSR", getUint);
    GET_PARAM(tCCD_NSW, "tCCD_NSW", getUint);
    GET_PARAM(tCCDMW, "tCCDMW", getUint);
    GET_PARAM(tCCD_L24, "tCCD_L24", getUint);
    GET_PARAM(tCCD_L48, "tCCD_L48", getUint);
    GET_PARAM(PCFG_TWR, "PCFG_TWR", getUint);
    GET_PARAM(PCFG_TRTP, "PCFG_TRTP", getUint);
    GET_PARAM(PCFG_TRTW, "PCFG_TRTW", getUint);
    GET_PARAM(PCFG_TRTW_L, "PCFG_TRTW_L", getUint);
    GET_PARAM(PCFG_TWTR, "PCFG_TWTR", getUint);
    GET_PARAM(PCFG_TWTR_L, "PCFG_TWTR_L", getUint);
    GET_PARAM(PCFG_TWTR_SB, "PCFG_TWTR_SB", getUint);
    GET_PARAM(PCFG_RANKTRTR, "PCFG_RANKTRTR", getUint);
    GET_PARAM(PCFG_RANKTRTW, "PCFG_RANKTRTW", getUint);
    GET_PARAM(PCFG_RANKTWTR, "PCFG_RANKTWTR", getUint);
    GET_PARAM(PCFG_RANKTWTW, "PCFG_RANKTWTW", getUint);
    GET_PARAM(tFAW, "tFAW", getUint);
    GET_PARAM(tCMD2SCH, "tCMD2SCH", getUint);
    GET_PARAM(tCMD2SCH_BYPACT, "tCMD2SCH_BYPACT", getUint);
    GET_PARAM(tCMD_CONF, "tCMD_CONF", getUint);
    GET_PARAM(tD_D, "tD_D", getUint);
    GET_PARAM(tPIPE_PRE_DMC, "tPIPE_PRE_DMC", getUint);
    GET_PARAM(tCMD_PHY, "tCMD_PHY", getUint);
    GET_PARAM(tDAT_PHY, "tDAT_PHY", getUint);
    GET_PARAM(tCMD_RASC, "tCMD_RASC", getUint);
    GET_PARAM(tDAT_RASC, "tDAT_RASC", getUint);
    GET_PARAM(tCMD_ADAPT, "tCMD_ADAPT", getUint);
    GET_PARAM(tWDATA_DMC, "tWDATA_DMC", getUint);
    GET_PARAM(tRFCab, "tRFCab", getUint);
    GET_PARAM(tRFCpb, "tRFCpb", getUint);
    GET_PARAM(tPBR2PBR, "tPBR2PBR", getUint);
    GET_PARAM(tPBR2PBR_L, "tPBR2PBR_L", getUint);
    GET_PARAM(tPBR2ACT, "tPBR2ACT", getUint);
    GET_PARAM(tCMD_WAKEUP, "tCMD_WAKEUP", getUint);
    GET_PARAM(tXP_V570, "tXP_V570", getUint);
    GET_PARAM(tXP_V580, "tXP_V580", getUint);
    GET_PARAM(tXP_V590, "tXP_V590", getUint);
    GET_PARAM(tXSR, "tXSR", getUint);
    GET_PARAM(tCMDPD, "tCMDPD", getUint);
    GET_PARAM(tnACU, "tnACU", getUint);
    GET_PARAM(tRDPD, "tRDPD", getUint);
    GET_PARAM(tRDAPPD, "tRDAPPD", getUint);
    GET_PARAM(tWRPD, "tWRPD", getUint);
    GET_PARAM(tWRAPPD, "tWRAPPD", getUint);
    GET_PARAM(tPDE, "tPDE", getUint);
    GET_PARAM(tCSPD, "tCSPD", getUint);
    GET_PARAM(tPDLP, "tPDLP", getUint);
    GET_PARAM(tPHYLPE, "tPHYLPE", getUint);
    GET_PARAM(tPHYLPX, "tPHYLPX", getUint);
    GET_PARAM(tASREFE, "tASREFE", getUint);
    GET_PARAM(ABR_PSTPND_LEVEL, "ABR_PSTPND_LEVEL", getUint);
    GET_PARAM(PBR_PSTPND_LEVEL, "PBR_PSTPND_LEVEL", getUint);
    GET_PARAM(PRE_NUM_SEND_PBR, "PRE_NUM_SEND_PBR", getUint);
    GET_PARAM(POWER_RDINC_K, "POWER_RDINC_K", getFloat);
    GET_PARAM(POWER_RDWRAP_K, "POWER_RDWRAP_K", getFloat);
    GET_PARAM(POWER_WRINC_K, "POWER_WRINC_K", getFloat);
    GET_PARAM(POWER_WRWRAP_K, "POWER_WRWRAP_K", getFloat);
    GET_PARAM(POWER_RDATA_K, "POWER_RDATA_K", getFloat);
    GET_PARAM(POWER_WDATA_K, "POWER_WDATA_K", getFloat);
    GET_PARAM(POWER_ACT_K, "POWER_ACT_K", getFloat);
    GET_PARAM(POWER_PREP_K, "POWER_PREP_K", getFloat);
    GET_PARAM(POWER_PRES_K, "POWER_PRES_K", getFloat);
    GET_PARAM(POWER_PREA_K, "POWER_PREA_K", getFloat);
    GET_PARAM(MAP_CONFIG["POWER_RD_K"], "POWER_RD_K", getUintArray);
    GET_PARAM(MAP_CONFIG["POWER_WR_K"], "POWER_WR_K", getUintArray);
    GET_PARAM(POWER_PBR_K, "POWER_PBR_K", getFloat);
    GET_PARAM(POWER_ABR_K, "POWER_ABR_K", getFloat);
    GET_PARAM(POWER_R2W_K, "POWER_R2W_K", getFloat);
    GET_PARAM(POWER_W2R_K, "POWER_W2R_K", getFloat);
    GET_PARAM(POWER_RNKSW_K, "POWER_RNKSW_K", getFloat);
    GET_PARAM(POWER_PDE_K, "POWER_PDE_K", getFloat);
    GET_PARAM(POWER_ASREFE_K, "POWER_ASREFE_K", getFloat);
    GET_PARAM(POWER_SRPDE_K, "POWER_SRPDE_K", getFloat);
    GET_PARAM(POWER_PDX_K, "POWER_PDX_K", getFloat);
    GET_PARAM(POWER_ASREFX_K, "POWER_ASREFX_K", getFloat);
    GET_PARAM(POWER_SRPDX_K, "POWER_SRPDX_K", getFloat);
    GET_PARAM(POWER_IDLE_K, "POWER_IDLE_K", getFloat);
    GET_PARAM(POWER_PDCC_K, "POWER_PDCC_K", getFloat);
    GET_PARAM(MAP_CONFIG["POWER_QUEUE_K"], "POWER_QUEUE_K", getUintArray);
    GET_PARAM(MATRIX_ROW23, "MATRIX_ROW23", getUint64);
    GET_PARAM(MATRIX_ROW22, "MATRIX_ROW22", getUint64);
    GET_PARAM(MATRIX_ROW21, "MATRIX_ROW21", getUint64);
    GET_PARAM(MATRIX_ROW20, "MATRIX_ROW20", getUint64);
    GET_PARAM(MATRIX_ROW19, "MATRIX_ROW19", getUint64);
    GET_PARAM(MATRIX_ROW18, "MATRIX_ROW18", getUint64);
    GET_PARAM(MATRIX_ROW17, "MATRIX_ROW17", getUint64);
    GET_PARAM(MATRIX_ROW16, "MATRIX_ROW16", getUint64);
    GET_PARAM(MATRIX_ROW15, "MATRIX_ROW15", getUint64);
    GET_PARAM(MATRIX_ROW14, "MATRIX_ROW14", getUint64);
    GET_PARAM(MATRIX_ROW13, "MATRIX_ROW13", getUint64);
    GET_PARAM(MATRIX_ROW12, "MATRIX_ROW12", getUint64);
    GET_PARAM(MATRIX_ROW11, "MATRIX_ROW11", getUint64);
    GET_PARAM(MATRIX_ROW10, "MATRIX_ROW10", getUint64);
    GET_PARAM(MATRIX_ROW9, "MATRIX_ROW9", getUint64);
    GET_PARAM(MATRIX_ROW8, "MATRIX_ROW8", getUint64);
    GET_PARAM(MATRIX_ROW7, "MATRIX_ROW7", getUint64);
    GET_PARAM(MATRIX_ROW6, "MATRIX_ROW6", getUint64);
    GET_PARAM(MATRIX_ROW5, "MATRIX_ROW5", getUint64);
    GET_PARAM(MATRIX_ROW4, "MATRIX_ROW4", getUint64);
    GET_PARAM(MATRIX_ROW3, "MATRIX_ROW3", getUint64);
    GET_PARAM(MATRIX_ROW2, "MATRIX_ROW2", getUint64);
    GET_PARAM(MATRIX_ROW1, "MATRIX_ROW1", getUint64);
    GET_PARAM(MATRIX_ROW0, "MATRIX_ROW0", getUint64);
    GET_PARAM(MATRIX_CH, "MATRIX_CH", getUint64);
    GET_PARAM(MATRIX_RA2, "MATRIX_RA2", getUint64);
    GET_PARAM(MATRIX_RA1, "MATRIX_RA1", getUint64);
    GET_PARAM(MATRIX_RA0, "MATRIX_RA0", getUint64);
    GET_PARAM(MATRIX_BA6, "MATRIX_BA6", getUint64);
    GET_PARAM(MATRIX_BA5, "MATRIX_BA5", getUint64);
    GET_PARAM(MATRIX_BA4, "MATRIX_BA4", getUint64);
    GET_PARAM(MATRIX_BA3, "MATRIX_BA3", getUint64);
    GET_PARAM(MATRIX_BA2, "MATRIX_BA2", getUint64);
    GET_PARAM(MATRIX_BA1, "MATRIX_BA1", getUint64);
    GET_PARAM(MATRIX_BA0, "MATRIX_BA0", getUint64);
    GET_PARAM(MATRIX_BG4, "MATRIX_BG4", getUint64);
    GET_PARAM(MATRIX_BG3, "MATRIX_BG3", getUint64);
    GET_PARAM(MATRIX_BG2, "MATRIX_BG2", getUint64);
    GET_PARAM(MATRIX_BG1, "MATRIX_BG1", getUint64);
    GET_PARAM(MATRIX_BG0, "MATRIX_BG0", getUint64);
    GET_PARAM(MATRIX_COL10, "MATRIX_COL10", getUint64);
    GET_PARAM(MATRIX_COL9, "MATRIX_COL9", getUint64);
    GET_PARAM(MATRIX_COL8, "MATRIX_COL8", getUint64);
    GET_PARAM(MATRIX_COL7, "MATRIX_COL7", getUint64);
    GET_PARAM(MATRIX_COL6, "MATRIX_COL6", getUint64);
    GET_PARAM(MATRIX_COL5, "MATRIX_COL5", getUint64);
    GET_PARAM(MATRIX_COL4, "MATRIX_COL4", getUint64);
    GET_PARAM(MATRIX_COL3, "MATRIX_COL3", getUint64);
    GET_PARAM(MATRIX_COL2, "MATRIX_COL2", getUint64);
    GET_PARAM(MATRIX_COL1, "MATRIX_COL1", getUint64);
    GET_PARAM(MATRIX_COL0, "MATRIX_COL0", getUint64);

    CalcMatrixNum();
    IniFilename = IniFilePath + "/" + DRAM_VENDOR + "_" + to_string(DRAM_CAPACITY) +
            "gb_" + DDR_TYPE + "_" + to_string(DMC_RATE) + "M" + DRAM_MODE + ".ini";
    iniFile.open(IniFilename.c_str());
    if (iniFile.is_open()) {
        if (hhaId == 0) DEBUG("== Loading DRAM power model file '"<<IniFilename<<"' ==");
        IniReader::ReadIniFile(iniFile, false);
        DRAM_POWER_EN = true;
    } else {
        if (hhaId == 0) DEBUG("== Unable to load "<<IniFilename<<", DRAM power model not enable ==");
        DRAM_POWER_EN = false;
    }
    iniFile.close();

    GET_PARAM(IDD01, "IDD01", getFloat);
    GET_PARAM(IDD02H, "IDD02H", getFloat);
    GET_PARAM(IDD02L, "IDD02L", getFloat);
    GET_PARAM(IDD0Q, "IDD0Q", getFloat);
    GET_PARAM(IDD2P1, "IDD2P1", getFloat);
    GET_PARAM(IDD2P2H, "IDD2P2H", getFloat);
    GET_PARAM(IDD2P2L, "IDD2P2L", getFloat);
    GET_PARAM(IDD2PQ, "IDD2PQ", getFloat);
    GET_PARAM(IDD2PS1, "IDD2PS1", getFloat);
    GET_PARAM(IDD2PS2H, "IDD2PS2H", getFloat);
    GET_PARAM(IDD2PS2L, "IDD2PS2L", getFloat);
    GET_PARAM(IDD2PSQ, "IDD2PSQ", getFloat);
    GET_PARAM(IDD2N1, "IDD2N1", getFloat);
    GET_PARAM(IDD2N2H, "IDD2N2H", getFloat);
    GET_PARAM(IDD2N2L, "IDD2N2L", getFloat);
    GET_PARAM(IDD2NQ, "IDD2NQ", getFloat);
    GET_PARAM(IDD2NS1, "IDD2NS1", getFloat);
    GET_PARAM(IDD2NS2H, "IDD2NS2H", getFloat);
    GET_PARAM(IDD2NS2L, "IDD2NS2L", getFloat);
    GET_PARAM(IDD2NSQ, "IDD2NSQ", getFloat);
    GET_PARAM(IDD3P1, "IDD3P1", getFloat);
    GET_PARAM(IDD3P2H, "IDD3P2H", getFloat);
    GET_PARAM(IDD3P2L, "IDD3P2L", getFloat);
    GET_PARAM(IDD3PQ, "IDD3PQ", getFloat);
    GET_PARAM(IDD3PS1, "IDD3PS1", getFloat);
    GET_PARAM(IDD3PS2H, "IDD3PS2H", getFloat);
    GET_PARAM(IDD3PS2L, "IDD3PS2L", getFloat);
    GET_PARAM(IDD3PSQ, "IDD3PSQ", getFloat);
    GET_PARAM(IDD3N1, "IDD3N1", getFloat);
    GET_PARAM(IDD3N2H, "IDD3N2H", getFloat);
    GET_PARAM(IDD3N2L, "IDD3N2L", getFloat);
    GET_PARAM(IDD3NQ, "IDD3NQ", getFloat);
    GET_PARAM(IDD3NS1, "IDD3NS1", getFloat);
    GET_PARAM(IDD3NS2H, "IDD3NS2H", getFloat);
    GET_PARAM(IDD3NS2L, "IDD3NS2L", getFloat);
    GET_PARAM(IDD3NSQ, "IDD3NSQ", getFloat);
    GET_PARAM(IDD4R1_BG, "IDD4R1_BG", getFloat);
    GET_PARAM(IDD4R2H_BG, "IDD4R2H_BG", getFloat);
    GET_PARAM(IDD4R2L_BG, "IDD4R2L_BG", getFloat);
    GET_PARAM(IDD4RQ_BG, "IDD4RQ_BG", getFloat);
    GET_PARAM(IDD4R1_BK, "IDD4R1_BK", getFloat);
    GET_PARAM(IDD4R2H_BK, "IDD4R2H_BK", getFloat);
    GET_PARAM(IDD4R2L_BK, "IDD4R2L_BK", getFloat);
    GET_PARAM(IDD4RQ_BK, "IDD4RQ_BK", getFloat);
    GET_PARAM(IDD4W1_BG, "IDD4W1_BG", getFloat);
    GET_PARAM(IDD4W2H_BG, "IDD4W2H_BG", getFloat);
    GET_PARAM(IDD4W2L_BG, "IDD4W2L_BG", getFloat);
    GET_PARAM(IDD4WQ_BG, "IDD4WQ_BG", getFloat);
    GET_PARAM(IDD4W1_BK, "IDD4W1_BK", getFloat);
    GET_PARAM(IDD4W2H_BK, "IDD4W2H_BK", getFloat);
    GET_PARAM(IDD4W2L_BK, "IDD4W2L_BK", getFloat);
    GET_PARAM(IDD4WQ_BK, "IDD4WQ_BK", getFloat);
    GET_PARAM(IDD51, "IDD51", getFloat);
    GET_PARAM(IDD52H, "IDD52H", getFloat);
    GET_PARAM(IDD52L, "IDD52L", getFloat);
    GET_PARAM(IDD5Q, "IDD5Q", getFloat);
    GET_PARAM(IDD5AB1, "IDD5AB1", getFloat);
    GET_PARAM(IDD5AB2H, "IDD5AB2H", getFloat);
    GET_PARAM(IDD5AB2L, "IDD5AB2L", getFloat);
    GET_PARAM(IDD5ABQ, "IDD5ABQ", getFloat);
    GET_PARAM(IDD5PB1, "IDD5PB1", getFloat);
    GET_PARAM(IDD5PB2H, "IDD5PB2H", getFloat);
    GET_PARAM(IDD5PB2L, "IDD5PB2L", getFloat);
    GET_PARAM(IDD5PBQ, "IDD5PBQ", getFloat);
    GET_PARAM(IDD61, "IDD61", getFloat);
    GET_PARAM(IDD62H, "IDD62H", getFloat);
    GET_PARAM(IDD62L, "IDD62L", getFloat);
    GET_PARAM(IDD6Q, "IDD6Q", getFloat);
    GET_PARAM(IDD6DS1, "IDD6DS1", getFloat);
    GET_PARAM(IDD6DS2H, "IDD6DS2H", getFloat);
    GET_PARAM(IDD6DS2L, "IDD6DS2L", getFloat);
    GET_PARAM(IDD6DSQ, "IDD6DSQ", getFloat);
    GET_PARAM(VDD1, "VDD1", getFloat);
    GET_PARAM(VDD2H, "VDD2H", getFloat);
    GET_PARAM(VDD2L, "VDD2L", getFloat);
    GET_PARAM(VDDQH, "VDDQH", getFloat);
    GET_PARAM(VDDQL, "VDDQL", getFloat);
#endif

    IniReader::ModifyParameter(cfg);
    CalcMatrixNum();
    // If we have any overrides, set them now before creating all of the memory objects
    IniReader::InitEnumsFromStrings();
    if (DRAM_POWER_EN && !IniReader::CheckIfAllSet()) {
        assert(0);
    }
    IniReader::CheckParameter();

    for (size_t dmcId=0; dmcId<NUM_CHANS; dmcId++) {
        MemorySystem *channel = new MemorySystem(dmcId, hhaId, DDRSim_log ,LogPath);
        channels.push_back(channel);
    }

//    rmw = new Rmw(this, hhaId, DDRSim_log, LogPath);

    string ddr_type;
    ddr_type = DDR_TYPE;
    transform(ddr_type.begin(), ddr_type.end(), ddr_type.begin(), ::tolower);
    if (hhaId == 0) DEBUG("== "<<ddr_type<<" "<<DMC_RATE<<" Mbps, "<<NUM_CHANS<<" Channels, "<<NUM_RANKS
            <<" Ranks, Gbuf "<<boolalpha<<WRITE_BUFFER_ENABLE<<", RMW "<<RMW_ENABLE<<", IECC "<<IECC_ENABLE<<noboolalpha<<" ==");
}

bool fileExists(string &path) {
    struct stat stat_buf;
    if (stat(path.c_str(), &stat_buf) != 0) {
        if (errno == ENOENT) {
            return false;
        }
        ERROR("Some other kind of error happened with stat(), should probably check");
    }
    return true;
}

string FilenameWithNumberSuffix(const string &filename, const string &extension,
        unsigned maxNumber=100) {
    string currentFilename = filename+extension;
    if (!fileExists(currentFilename)) {
        return currentFilename;
    }

    // otherwise, add the suffixes and test them out until we find one that works
    stringstream tmpNum;
    tmpNum<<"."<<1;
    for (unsigned i=1; i<maxNumber; i++) {
        currentFilename = filename+tmpNum.str()+extension;
        if (fileExists(currentFilename)) {
            currentFilename = filename;
            tmpNum.seekp(0);
            tmpNum << "." << i;
        } else {
            return currentFilename;
        }
    }
    // if we can't find one, just give up and return whatever is the current filename
    ERROR("Warning: Couldn't find a suitable suffix for "<<filename);
    return currentFilename;
}

LPMemorySystemTop::~LPMemorySystemTop() {
    
//    rmw->register_write(4,0);
//    rmw->register_write(0,0);
//    delete rmw;

    for (size_t i=0; i<NUM_CHANS; i++) {
        delete channels[i];
    }
    channels.clear();

    // flush our streams and close them up
#ifdef LOG_OUTPUT
    DDRSim_log.flush();
    DDRSim_log.close();
#endif

}

void LPMemorySystemTop::update() {
//    if (RMW_ENABLE) {
//        rmw->update();
//    }
    for (size_t i=0; i<NUM_CHANS; i++) {
        channels[i]->update();
    }

//    if (RMW_ENABLE) {
//        rmw->step();
//    }
//    step();

//    //statistics for RMW
//    if (STATE_TIME != 0) {
//        if ((now() % STATE_TIME) == 0) {
//            rmw->register_write(4,0);
//            rmw->register_write(0,0);
//        }
//    }
//    rmw->check_cnt();
}

uint8_t LPMemorySystemTop::get_occ(uint8_t chl) {
    if (EM_ENABLE && EM_MODE==0) return channels[0]->get_occ();
    return channels[chl]->get_occ();
}

uint8_t LPMemorySystemTop::get_bandwidth(uint8_t chl) {
    if (EM_ENABLE && EM_MODE==0) return uint8_t(channels[0]->memoryController->availability);
    return uint8_t(channels[chl]->memoryController->availability);
}

void LPMemorySystemTop::noc_read_inform(uint8_t channel, bool fast_wakeup_rank0,
        bool fast_wakeup_rank1, bool bus_rempty) {
    if (EM_ENABLE && EM_MODE==0) { 
        channels[0]->noc_read_inform(fast_wakeup_rank0, fast_wakeup_rank1, bus_rempty);
    } else {
        channels[channel]->noc_read_inform(fast_wakeup_rank0, fast_wakeup_rank1, bus_rempty);
    }
}

bool LPMemorySystemTop::addTransaction(const hha_command &command) {
    command_check(command);
    uint8_t ch = addr_map_ch(command);

    //GRANT FIFO BP under LP6 Nomal mode
    if (!EM_ENABLE && IS_LP6 && channels[1]->memoryController->grt_fifo_bp) {
        return false;
    }

//    if (RMW_ENABLE && rmw->pre_req_time == rmw->now()) {
//        return false;
//    }

    bool ret = false;
    Transaction *trans = new Transaction(command);
//    trans_init(trans, now());

    addressMapping(*trans);
    trans_check(trans);
    
//    if (RMW_ENABLE && !trans->pre_act) {
//        ret = rmw->addTransaction(trans);
//    } else {
        if (EM_ENABLE && EM_MODE==0) {
            ret = channels[0]->addTransaction(trans);
        } else {
            ret = channels[ch]->addTransaction(trans);
        }

    if (!ret){
        delete trans;
    }
    
    return ret;
}

//void LPMemorySystemTop::trans_init(Transaction *trans, uint64_t inject_time) {
//    trans->data_size = (trans->burst_length + 1) * DMC_DATA_BUS_BITS / 8;
//    trans->inject_time = inject_time;
//    if (!LAT_INC_BP) trans->reqEnterDmcBufTime = now() * tDFI;
//}

void LPMemorySystemTop::trans_check(Transaction *t) {
    if (IS_HBM2E || IS_HBM3) {
        if (NUM_SIDS == 3 && t->sid == 3) {
            ERROR(setw(10)<<now()<<" Error Sid! task="<<t->task<<" address="<<hex<<t->address<<dec<<" sid="<<t->sid);
            assert(0);
        }
    }
    if (IS_LP6 && EM_ENABLE && EM_MODE==2) {
        if (t->sc == 1 && t->rank == 1) {
            ERROR(setw(10)<<now()<<" Error SC under Combo e-mode! task="<<t->task<<" address="<<hex<<t->address<<dec<<" sc="<<t->sc<<", rank="<<t->rank);
            assert(0);
        }
    }
//    if (IS_LP6 && !EM_ENABLE) {
//        if (t->sc != t->channel) {
//            ERROR(setw(10)<<now()<<" Mismatch SC value under n-mode! task="<<t->task<<" address="<<hex<<t->address<<dec<<" sc="<<t->sc<<", channel="<<t->channel);
//            assert(0);
//        }
//    }
}

uint32_t LPMemorySystemTop::getDmcPressureLevel() {
    uint32_t maxPressureLevel = 0;
    uint32_t pressureLevel = 0;
    for (uint32_t i=0; i< NUM_CHANS; i++) {
        pressureLevel = channels[i]->getFlowPressureLevel();
        if (pressureLevel > maxPressureLevel) {
            maxPressureLevel = pressureLevel;
        }
    }
    return maxPressureLevel;
}

bool LPMemorySystemTop::addData(uint32_t *data,uint32_t channel,uint64_t id) {
//    if (RMW_ENABLE && rmw->pre_req_data_time == rmw->now()) {
//        return false;
//    }
    
    if (EM_ENABLE && EM_MODE==0) channel = 0;
    
    bool ret = false;
//    if (RMW_ENABLE) {
//        ret = rmw->addData(data, channel, id); 
//    } else {
//        ret = channels[channel]->addData(data, id);
//    }
    ret = channels[channel]->addData(data, id, false);
    return ret;
}


void LPMemorySystemTop::RegisterCallbacks(
    TransactionCompleteCB *readData,
    TransactionCompleteCB *writeDone,
    TransactionCompleteCB *readDone,
    TransactionCompleteCB *cmdDone ) {
    for (size_t i=0; i<NUM_CHANS; i++) {
        channels[i]->RegisterCallbacks(readData, writeDone, readDone, cmdDone);
    }
}

uint32_t LPMemorySystemTop::getTransQueSize(uint32_t dmc_id, bool isRd) {
    if (EM_ENABLE && EM_MODE==0) return channels.at(0)->getTransQueSize(isRd);
    return channels.at(dmc_id)->getTransQueSize(isRd);
}

void LPMemorySystemTop::dfs_backpress(unsigned ch, bool backpress) {
    if (EM_ENABLE && EM_MODE==0) channels[0]->dfs_backpress(backpress);
    channels[ch]->dfs_backpress(backpress);
}

void LPMemorySystemTop::GetQueueCmdNum(uint8_t channel, unsigned *dmc_rd_num,
        unsigned *dmc_wr_num, unsigned *gbuf_rd_num, unsigned *gbuf_wr_num) {
    if (EM_ENABLE && EM_MODE==0) {
        channels[0]->GetQueueCmdNum(dmc_rd_num, dmc_wr_num, gbuf_rd_num, gbuf_wr_num);
    } else {
        channels[channel]->GetQueueCmdNum(dmc_rd_num, dmc_wr_num, gbuf_rd_num, gbuf_wr_num);
    }
}

void LPMemorySystemTop::GetDmcBusyStatus(uint8_t channel, bool *dmc_busy) {
    if (EM_ENABLE && EM_MODE==0) {
        channels[0]->GetDmcBusyStatus(dmc_busy);
    } else {
        channels[channel]->GetDmcBusyStatus(dmc_busy);
    }
}

uint32_t LPMemorySystemTop::getRmwQueueCmdNum(uint8_t ch) const{
    return channels[ch]->memoryController->rmw->rmw_cmd_cnt;
}

uint8_t LPMemorySystemTop::addr_map_ch(const hha_command &c) {
    if (IS_LP6) {
        return c.channel;
    } else {
        if (MATRIX_CH == 0) return c.channel;
        return (bit_xor(MATRIX_CH, c.address));
    }
}

void LPMemorySystemTop::command_check(const hha_command &c) {
    if (c.type != DATA_READ && c.type != DATA_WRITE) {
        ERROR("Error command type! task="<<c.task<<", type="<<c.type);
        assert(0);
    }
    if (c.pf_type >= 4) {
        ERROR("Error pf_type! task="<<c.task<<", pf_type="<<c.pf_type);
        assert(0);
    }
    if (c.sub_pftype >= 13) {
        ERROR("Error sub_pftype! task="<<c.task<<", sub_pftype="<<c.sub_pftype);
        assert(0);
    }
    if (c.sub_src >= 4) {
        ERROR("Error sub_src! task="<<c.task<<", sub_src="<<c.sub_src);
        assert(0);
    }
    if (c.qos >= 8) {
        ERROR("Error qos value! task="<<c.task<<", qos="<<c.qos);
        assert(0);
    }
    if (c.mid >= MidMax) {
        ERROR("Error mid value! task="<<c.task<<", mid="<<c.mid);
        assert(0);
    }
    if (c.channel > NUM_CHANS) {
        ERROR("Error command channel number! task="<<c.task<<", channel="<<c.channel);
        assert(0);
    }
}

void LPMemorySystemTop::wdata_check(uint64_t task, uint8_t channel) {
    if (channel > NUM_CHANS) {
        ERROR("Error wdata channel number! task="<<task<<", channel="<<channel);
        assert(0);
    }
}

/**
 * This function creates up to 3 output files:
 *     - The .log file if LOG_OUTPUT is set
 *     - the .vis file where csv data for each epoch will go
 *     - the .tmp file if verification output is enabled
 * The results directory is setup to be in PWD/TRACEFILENAME.[SIM_DESC]/DRAM_PARTNAME/PARAMS.vis
 * The environment variable SIM_DESC is also appended to output files/directories
 *
 * TODO: verification info needs to be generated per channel so it has to be
 * moved back to MemorySystem
 **/
}