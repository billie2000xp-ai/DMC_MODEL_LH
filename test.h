#include <string>
#include <stdint.h>
#include "LPMemorySystemTop.h"
#include <iomanip>

namespace LPDDRSim {

bool LATENCY_MODE;
bool MATCH_MODE;
unsigned BKD_OSTD;
unsigned T_OSTD;
unsigned T_NUM;
unsigned BKD_DLY;
unsigned T_DLY;
unsigned WDATA_DLY;
bool TRACE_EN;
bool DOU_TRACE_EN;
bool CH_XOR_EN;
unsigned CHINTLV_START;
unsigned CHINTLV_BIT;
unsigned VLD_CH_NUM;
unsigned TRACE_CMD_NUM;
string TRACE;
unsigned TRACE_Q_MAX_CNT;
float GEAR_RATIO;
unsigned SEQ_NUM;
unsigned DATA_SIZE;
unsigned SEQ_DATA_SIZE;
unsigned RK_SW_RATIO;
unsigned WR_RATIO;
unsigned MASK_WR_RATIO;
unsigned FASTWAKEUP_CYCLE;
bool SAME_ADDR_MODE;
bool SAME_BA_RAND_ROW_MODE;
string PARAM_PATH;
string LOG_PATH;
bool STOP_WITH_STATETIME;
unsigned STOP_WINDOW;

struct fastwakeup {
    uint64_t send_time;
    unsigned rank;
};

struct wdata {
    uint64_t task;
    uint64_t delay;
    unsigned ch;
};

class some_object {
	public:
		bool read_data(unsigned, uint64_t ,double, double, double);
		bool write_response(unsigned, uint64_t ,double, double, double);
        bool read_response(unsigned, uint64_t ,double, double, double);
        bool cmd_response(unsigned, uint64_t ,double, double, double);
};

void build_cfg() {
    string path = PARAM_PATH + "/main.ini";
    SimpleConfig cfg(path.c_str());
    LATENCY_MODE = cfg.getBool("LATENCY_MODE");
    MATCH_MODE = cfg.getBool("MATCH_MODE");
    BKD_OSTD = cfg.getUint("BKD_OSTD");
    T_OSTD = cfg.getUint("T_OSTD");
    T_NUM = cfg.getUint("T_NUM");
    BKD_DLY = cfg.getUint("BKD_DLY");
    T_DLY = cfg.getUint("T_DLY");
    WDATA_DLY = cfg.getUint("WDATA_DLY");
    TRACE_EN = cfg.getBool("TRACE_EN");
    CHINTLV_START = cfg.getUint("CHINTLV_START");
    CHINTLV_BIT = cfg.getUint("CHINTLV_BIT");
    VLD_CH_NUM = cfg.getUint("VLD_CH_NUM");
    TRACE_CMD_NUM = cfg.getUint("TRACE_CMD_NUM");
    DOU_TRACE_EN = cfg.getBool("DOU_TRACE_EN");
    CH_XOR_EN = cfg.getBool("CH_XOR_EN");
    TRACE = cfg.get("TRACE");
    TRACE_Q_MAX_CNT = cfg.getUint("TRACE_Q_MAX_CNT");
    GEAR_RATIO = cfg.getFloat("GEAR_RATIO");
    SEQ_NUM = cfg.getUint("SEQ_NUM");
    DATA_SIZE = cfg.getUint("DATA_SIZE");
    SEQ_DATA_SIZE = cfg.getUint("SEQ_DATA_SIZE");
    RK_SW_RATIO = cfg.getUint("RK_SW_RATIO");
    WR_RATIO = cfg.getUint("WR_RATIO");
    MASK_WR_RATIO = cfg.getUint("MASK_WR_RATIO");
    FASTWAKEUP_CYCLE = cfg.getUint("FASTWAKEUP_CYCLE");
    SAME_ADDR_MODE = cfg.getBool("SAME_ADDR_MODE");
    SAME_BA_RAND_ROW_MODE = cfg.getBool("SAME_BA_RAND_ROW_MODE");
    PARAM_PATH = cfg.get("PARAM_PATH");
    LOG_PATH = cfg.get("LOG_PATH");
    STOP_WITH_STATETIME = cfg.getBool("STOP_WITH_STATETIME");
    STOP_WINDOW = cfg.getUint("STOP_WINDOW");
}

void update_cfg(int argc, char *argv[]) {
    Configurable cfg;
    for (int i = 1; i < argc; i ++) {
        cfg.getString(argv[i]);
    }
    GET_PARAM(LATENCY_MODE, "LATENCY_MODE", getBool);
    GET_PARAM(MATCH_MODE, "MATCH_MODE", getBool);
    GET_PARAM(BKD_OSTD, "BKD_OSTD", getUint);
    GET_PARAM(T_OSTD, "T_OSTD", getUint);
    GET_PARAM(T_NUM, "T_NUM", getUint);
    GET_PARAM(BKD_DLY, "BKD_DLY", getUint);
    GET_PARAM(T_DLY, "T_DLY", getUint);
    GET_PARAM(WDATA_DLY, "WDATA_DLY", getUint);
    GET_PARAM(TRACE_EN, "TRACE_EN", getBool);
    GET_PARAM(DOU_TRACE_EN, "DOU_TRACE_EN", getBool);
    GET_PARAM(CH_XOR_EN, "CH_XOR_EN", getBool);
    GET_PARAM(CHINTLV_START, "CHINTLV_START", getUint);
    GET_PARAM(CHINTLV_BIT, "CHINTLV_BIT", getUint);
    GET_PARAM(VLD_CH_NUM, "VLD_CH_NUM", getUint);
    GET_PARAM(TRACE_CMD_NUM, "TRACE_CMD_NUM", getUint);
    GET_PARAM(TRACE, "TRACE", get);
    GET_PARAM(TRACE_Q_MAX_CNT, "TRACE_Q_MAX_CNT", getUint);
    GET_PARAM(GEAR_RATIO, "GEAR_RATIO", getFloat);
    GET_PARAM(SEQ_NUM, "SEQ_NUM", getUint);
    GET_PARAM(DATA_SIZE, "DATA_SIZE", getUint);
    GET_PARAM(SEQ_DATA_SIZE, "SEQ_DATA_SIZE", getUint);
    GET_PARAM(RK_SW_RATIO, "RK_SW_RATIO", getUint);
    GET_PARAM(WR_RATIO, "WR_RATIO", getUint);
    GET_PARAM(MASK_WR_RATIO, "MASK_WR_RATIO", getUint);
    GET_PARAM(FASTWAKEUP_CYCLE, "FASTWAKEUP_CYCLE", getUint);
    GET_PARAM(SAME_ADDR_MODE, "SAME_ADDR_MODE", getBool);
    GET_PARAM(SAME_BA_RAND_ROW_MODE, "SAME_BA_RAND_ROW_MODE", getBool);
    GET_PARAM(PARAM_PATH, "PARAM_PATH", get);
    GET_PARAM(LOG_PATH, "LOG_PATH", get);
    GET_PARAM(STOP_WITH_STATETIME, "STOP_WITH_STATETIME", getBool);
    GET_PARAM(STOP_WINDOW, "STOP_WINDOW", getUint);
}

void get_param_path(int argc, char *argv[]) {
    Configurable cfg;
    for (int i = 1; i < argc; i ++) {
        cfg.getString(argv[i]);
    }
    PARAM_PATH = "./parameter/";
    if (cfg.has("PARAM_PATH")) PARAM_PATH = cfg.get("PARAM_PATH");
}

};