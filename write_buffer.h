#ifndef _WRITE_BUFFER_H
#define _WRITE_BUFFER_H

#include "Transaction.h"
#include <memory>
#include <stdint.h>
#include <ostream>
#include <cstring>
#include "SimulatorObject.h"
#include "SystemConfiguration.h"

using namespace std;

namespace LPDDRSim {
typedef Transaction& t_ptr;
class MemoryController;
class WriteBuff:public SimulatorObject {
    enum WB_State {
        WBUFF_IDLE,
        WBUFF_WRITE
    };
    ofstream buf_log;
    struct arb_cmd {
        uint64_t task;
        TransactionType type;
        unsigned rank;
        unsigned row;
        unsigned bank;
        unsigned group;
        unsigned bankIndex;
        unsigned pri;
        arb_cmd() {
            task = 0;
            type = DATA_READ;
            rank = 0;
            row = 0;
            bank = 0;
            group = 0;
            bankIndex = 0;
            pri = 0;
        }
        void creat(Transaction *t) {
            task = t->task;
            type = t->transactionType;
            rank = t->rank;
            row = t->row;
            bank = t->bank;
            group = t->group;
            bankIndex = t->bankIndex;
            pri = t->pri;
        }
    };
    struct conf_cnt {
        unsigned bg_conf_cnt;
        unsigned ba_conf_cnt;
        unsigned ad_conf_cnt;
        conf_cnt() {
            bg_conf_cnt = 0;
            ba_conf_cnt = 0;
            ad_conf_cnt = 0;
        }
    };
    struct conf_state {
        conf_cnt gbuf;
        conf_cnt dmc;
        bool rowhit;
        conf_state() {
            rowhit = false;
        }
    };
    struct Grp_Mode {
        uint8_t grp_mode0;
        uint8_t grp_mode1;
        uint8_t grp_mode23;
        uint8_t grp_mode4;
        uint8_t grp_mode5;
        uint8_t grp_mode6;
        uint8_t grp_mode7;
        Grp_Mode() {
            grp_mode0 = 0;
            grp_mode1 = 0;
            grp_mode23 = 0;
            grp_mode4 = 0;
            grp_mode5 = 0;
            grp_mode6 = 0;
            grp_mode7 = 0;
        }
    };

    public:
    WriteBuff(MemoryController *_top,unsigned id, ostream &DDRSim_log_);
    //virtual ~WriteBuff() {};
    bool addTransaction(Transaction * trans);
    void addData(uint32_t *data ,uint64_t task);
    void rcmd_set_conflict(Transaction *trans);
    void wcmd_set_conflict(Transaction *trans);
    void rcmd_release_conflict(Transaction *trans);
    void wcmd_release_conflict(Transaction *trans);
    void dmc_release_conflict(Transaction *trans);
    bool read_forward(Transaction * trans);
    bool write_merge(Transaction * trans);
    void update();
    void update_state_trig();
    void sch_rque();
    void sch_wque();
    void arb_node();
    void send_wdata();
    unsigned priority(arb_cmd *cmd);
    inline bool full() {return (wcmd_cnt >= WB_WDEPTH || rcmd_cnt >= WB_RDEPTH);}
    void func_check();
    void rcmd_push_wcmd(Transaction *trans);
    unsigned read_cnt;
    unsigned write_cnt;
    inline unsigned rcmd_num() {return rcmd_cnt;}
    inline unsigned wcmd_num() {return wcmd_cnt;}
    unsigned get_max_rank();
    void rcmd_set_timeout(Transaction *trans);
    void check_timeout();
    void update_state();
    void trans_state_clr(Transaction *trans);
    uint64_t pre_req_time;
    std::vector <bool> rcmd_bank_state;
    std::vector<uint64_t> WdataToSend;

    private:
    std::vector<Transaction *> Rbuff;
    std::vector<conf_state *> RbuffConfCnt;
    std::vector<Transaction *> Wbuff;
    std::vector<conf_state *> WbuffConfCnt;
    std::vector<std::vector<unsigned>>wb_bank_cnt;
    std::vector<std::vector<unsigned>>rb_bank_cnt;
    std::vector<unsigned> wb_rank_cnt;
    std::vector<unsigned> rb_rank_cnt;
    MemoryController *top;
    unsigned channel;
    uint64_t channel_ohot;
    ostream &DDRSim_log;
    unsigned wcmd_cnt;
    unsigned rcmd_cnt;
    unsigned ser_write_cnt;
    WB_State wbuff_state;
    unsigned wbuff_state_gap;
    unsigned max_rank;
    unsigned ser_sch_write;
    unsigned same_bank_cnt;
    unsigned no_cmd_sch_cnt;
    unsigned no_cmd_sch_th;
    unsigned bytes_per_col;
    string log_path;

    public:
    unsigned forward_cnt;
    unsigned merge_cnt;
    unsigned push_cnt;
    vector<uint8_t> rb_qos_cnt;
    vector<uint8_t> wb_qos_cnt;
    vector<unsigned> sch_level_cnt;

    vector<vector<unsigned *>> wr_level;
    vector<vector<unsigned *>> wr_most_level;
    vector<vector<unsigned *>> rd_level;
    vector<vector<unsigned *>> grp_mode;

    unsigned state_trig = 0;
    unsigned idle_trig_time = 0;
    arb_cmd LastArbCmd;
    std::vector <arb_cmd *> ArbCmd;
    Grp_Mode GrpMode;
    unsigned rd_sch_rhit_cnt = 0;
    unsigned rd_timeout_cnt = 0;
    unsigned wr_timeout_cnt = 0;
};
}
#endif