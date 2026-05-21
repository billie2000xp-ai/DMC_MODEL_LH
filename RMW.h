#ifndef _RMW_H
#define _RMW_H

#include "Transaction.h"
#include "MemorySystem.h"
#include <memory>
#include <stdint.h>
#include <ostream>
#include <cstring>
#include <deque>
#include <map>
#include "Callback.h"
#include "SimulatorObject.h"
#include "SystemConfiguration.h"

using namespace std;

namespace LPDDRSim {
//typedef Transaction& t_ptr;
class MemoryController;
//class LPMemorySystemTop;
class Rmw:public SimulatorObject {
//    ofstream DDRSim_log;
//    ofstream state_log;

    enum RMW_State {
        QUE_WAITING,
        MERGE_READ,
        SEND_READY
    };
    ofstream buf_log;
    struct conf_state {
        uint64_t task;
        unsigned ad_conf_cnt;
        conf_state() {
            ad_conf_cnt = 0;
            task = 0;
        }
    };
    struct cmd_state {
        uint64_t task;
        uint64_t rmwTimeAdded;
        RMW_State rmwState;
        cmd_state() {
            task = 0;
            rmwState = QUE_WAITING;
            rmwTimeAdded = 0;
        }
    };

    public:
    Rmw(MemoryController *_top, unsigned id, ostream &DDRSim_log_);
    virtual ~Rmw() {};
    bool addTransaction(Transaction * trans);
    bool addMergedWriteTransaction(Transaction *trans);
    void update_cresp();
//    bool addData(uint32_t *data, uint32_t channel, uint64_t task);
    void addData(uint32_t *data, uint64_t task);
//    void RmwInitOutputFiles();
    void cmd_set_conflict(Transaction *trans);
    void cmd_release_conflict(Transaction *trans);
    bool address_conf(Transaction *t, Transaction *cmd);
    void update();
    void sch_que();
    void arb_node();
    void send_wdata();
    unsigned rmw_cmd_cnt;
    inline bool full() {return (rmw_cmd_cnt >= RMW_QUE_DEPTH);}
    void func_check();
    void update_state();
//    void trans_state_clr(Transaction *trans);
    uint64_t pre_req_time;          // time point for last cmd
    uint64_t pre_req_data_time;    // time point for last wdata 
    std::vector<uint64_t> WdataToSend;
    std::vector<uint32_t> WdataChannel;
    std::map<uint64_t, unsigned> MaskWriteDataBeats;
    std::map<uint64_t, unsigned> MaskWriteSendBeats;
    std::vector<Transaction *> RmwQue;

//    string rmw_log;
//    string log_path;
//    unsigned channel;
//    uint64_t channel_ohot;
    

    uint64_t pre_cresp_time;
    vector <uint64_t> RmwCmdResp;
    vector <uint8_t> RmwCmdRespCh;
    void gen_cresp(uint64_t task);
    bool cmd_response(uint64_t task,uint64_t address, uint8_t ch);
    
    unsigned GetRmwQsize();
    void check_cnt();
//    void register_write(uint64_t address, uint32_t data);
//    void statistics();

    //statistics
    unsigned totalReads; 
    unsigned totalBypassReads; 
    unsigned totalWrites; 
    unsigned totalBypassWrites; 
    unsigned totalFullWrites; 
    unsigned totalMaskWrites; 
    unsigned totalTransactions;
    uint64_t totalWriteMergeInput;
    uint64_t totalWriteMergePair;
    uint64_t totalWriteMergeUnpairedToRmw;
    uint64_t totalWriteMergeUnpairedDirect;
    uint64_t totalWriteMergeBufferFull;
//    unsigned pre_reads; 
//    unsigned pre_bypass_reads; 
//    unsigned pre_bypass_writes; 
//    unsigned pre_writes; 
//    unsigned pre_full_writes; 
//    unsigned pre_mask_writes; 
//    unsigned pre_totals; 
    
    vector<uint32_t> rmw_que_cnt;
    
    uint64_t start_cycle;
    uint64_t end_cycle;

    private:
//    std::vector<Transaction *> RmwQue;
    struct BufferedWriteEntry {
        Transaction *first_trans;
        Transaction *second_trans;
        bool has_second;
        unsigned first_data_ready_cnt;
        unsigned second_data_ready_cnt;

        BufferedWriteEntry() {
            first_trans = NULL;
            second_trans = NULL;
            has_second = false;
            first_data_ready_cnt = 0;
            second_data_ready_cnt = 0;
        }
    };

    struct WriteDataRoute {
        uint64_t internal_task;
        unsigned remaining_beats;
        uint64_t resp_task;
        unsigned resp_channel;
        bool has_resp;

        WriteDataRoute() {
            internal_task = 0;
            remaining_beats = 0;
            resp_task = 0;
            resp_channel = 0;
            has_resp = false;
        }
    };

    struct PendingWriteResp {
        uint64_t external_task;
        unsigned upstream_channel;

        PendingWriteResp() {
            external_task = 0;
            upstream_channel = 0;
        }
    };

    std::deque<BufferedWriteEntry> write_merge_buffer;
    std::map<uint64_t, std::deque<WriteDataRoute> > write_data_routes;
    std::deque<PendingWriteResp> pending_write_merge_resps;
    std::map<uint64_t, unsigned> prefetched_write_data_beats;
    bool is_write_merge_candidate(const Transaction *trans) const;
    bool can_merge_write_pair(const Transaction *first, const Transaction *second) const;
    Transaction *build_merged_transaction(const BufferedWriteEntry &entry, uint64_t merged_task) const;
    Transaction *build_rmw_256b_transaction(Transaction *trans) const;
    bool dispatch_buffered_write(size_t entry_index);
    void pump_write_merge_buffer();
    bool is_task_buffered(uint64_t task) const;
    void enqueue_pending_write_resp(uint64_t external_task, unsigned upstream_channel);
    void pump_pending_write_resps();
    bool handle_write_merge_transaction(Transaction *trans);
    bool addTransactionWithoutWriteMerge(Transaction *trans);
    std::vector<conf_state *> RmwConfCnt;
    std::vector<cmd_state *> RmwCmdState;
//    LPMemorySystemTop *top;
    MemoryController *top;
    unsigned channel;
    uint64_t channel_ohot;
    ostream &DDRSim_log;
    RMW_State rmw_state;
//    unsigned same_bank_cnt;
//    unsigned no_cmd_sch_cnt;
//    unsigned no_cmd_sch_th;
//    unsigned bytes_per_col;
    string log_path;
    unsigned rcmd_cnt;
    unsigned wcmd_cnt;

//    public:
//    unsigned push_cnt;


//    arb_cmd LastArbCmd;
//    std::vector <arb_cmd *> ArbCmd;
};
}
#endif