#ifndef _INLINE_ECC_H_
#define _INLINE_ECC_H_

#include "Transaction.h"
#include "assert.h"
#include <map>
#include <deque>
#include "SimulatorObject.h"
using namespace std;
namespace LPDDRSim {

enum BUF_STATE {
    BUF_IDLE,
    WAIT_RDECC_BACK,
    WAIT_WRECC_BACK
};

enum ECC_MODEL_STATE {
    TRY_HIT_ECC_BUF,
    GET_ECC_BUF,
    ECC_ADD_TRANS,
    ADD_RD_ECC_TRANS,
    ADD_WR_ECC_TRANS,
    WAIT_ECC_DATA_FINISH,
    ECC_RD_HIT_DIRTY,
    ECC_RD_MISS_DIRTY
};

enum ECC_STRATEGY {
    ID_ORDER,
    LRU
};

struct ECC_BUF_Entry {
    uint32_t buf_id = 0; // should not be changed
    uint32_t ecc_buf_addr = 0; // there are 64 entry for ecc_buf;
    bool     vld;
    uint32_t slt_cnt;
    uint64_t pdu_addr; // echo pdu cover 512Byte addr space
    bool     eor; // end of row flag
    uint32_t wr_ecc_pos;
    bool     rd_ecc_pos;
    uint64_t wr_ecc_info[16];
    uint64_t rd_ecc_info[16];

    uint32_t ecc_pri;//replace priority
    uint32_t wr_merge_cnt;//24/9/6
    

    BUF_STATE buf_ctrl_state;
    bool ecc_dirty = false;
    bool rd_ecc    = false;
    bool wr_ecc    = false;
    std::vector<uint64_t>task_list;
};

class MemoryController;
class Inline_ECC:public SimulatorObject {
    public :
    Inline_ECC(MemoryController *_top,unsigned id, ostream &DDRSim_log_);
    virtual ~Inline_ECC();
    MemoryController *top;
    unsigned channel;
    uint64_t channel_ohot;
    ostream &DDRSim_log;

    uint32_t hit_ecc_buf(uint64_t pdu_addr);
    uint32_t get_avail_rd_ecc_buf_id();
    uint32_t get_avail_wr_ecc_buf_id();

    bool proc_iecc(Transaction * trans, uint64_t inject_time);
    bool addTransaction(Transaction * trans);
    void update_rd_ecc_buf();
    void update_wr_ecc_buf();
    bool ecc_try_add_rd_trans(Transaction * trans, uint32_t ecc_buf_id);
    bool ecc_try_add_wr_trans(Transaction * trans, uint32_t ecc_buf_id);
    bool try_add_ecc_rd(Transaction * trans, uint32_t ecc_buf_id);
    bool try_add_ecc_wr(Transaction * trans, uint32_t ecc_buf_id);
    void update();
    void get_pdu_addr(ECC_BUF_Entry *buf, Transaction *trans);

    uint64_t ecc_task;
    uint32_t ecc_conflict_cnt;
    uint32_t iecc_add_cnt;

    std::vector<ECC_BUF_Entry> rd_ecc_buf;
    std::vector<ECC_BUF_Entry> wr_ecc_buf;
    std::map<uint32_t, bool> index_recycle_rd_ecc_buf;
    std::map<uint32_t, bool> index_recycle_wr_ecc_buf;
    uint32_t avail_rd_ecc_buf_id;
    uint32_t avail_wr_ecc_buf_id;
    uint32_t counter;
    uint32_t wdata_full_counter;
    uint32_t wdata_256bit_num;
    uint32_t try_count;
    uint32_t rhit;
    uint32_t whit;
    uint32_t merge_cnt;
    uint32_t merge_id;
    uint32_t wpos[17];
    ECC_MODEL_STATE ecc_model_state;
    uint64_t pdu_addr = 0;
    uint32_t ecc_delay = 0;
    ECC_MODEL_STATE ecc_pre_state;

    unsigned iecc_cmd_cnt;
    unsigned iecc_wdata_cnt;
    unsigned backpress_cnt;

    //bool MERGE_ENABLE;
    bool ecc_merge_flag;
    bool PREFETCH_ENABLE;
    ECC_STRATEGY strategy;


    std::deque<Transaction *> IeccCmdQueue;
    struct wdata {
        uint64_t wdata_delay;
        uint64_t task;
        wdata() {
            wdata_delay = 0;
            task = 0;
        }
    };
    std::deque<wdata> IeccWdata;
    bool addData(uint32_t *data ,uint64_t task);
    void addr_exp(Transaction * trans);
    void addr_ecc_map(Transaction * trans);
    void iecc_alct();
    void send_command();
    void send_wdata();
    bool full() {return (iecc_cmd_cnt >= 3);};
    bool wdata_full() {return (iecc_wdata_cnt >= 1);};
    std::string trans_type_opcode_iecc(Transaction * trans);
    void show_buf_state();

    void replace_pri_update(uint32_t id,uint32_t cmd_type);
};
}
#endif