#ifndef _LPDDR_MEMORYSYSTEM_H_
#define _LPDDR_MEMORYSYSTEM_H_

#include "SimulatorObject.h"
#include "SystemConfiguration.h"
#include "MemoryController.h"
#include "Transaction.h"
#include "Callback.h"
#include <deque>
#ifdef SYSARCH_PLATFORM
#include "HALib/event.h"
#endif

namespace LPDDRSim {

typedef CallbackBase<bool,unsigned,uint64_t,double,double,double> Callback_t;

class MemorySystem : public SimulatorObject {
    ofstream DDRSim_log;
    ofstream trace_log;
    ofstream cmdnum_log;
    vector <ofstream> dram_log;
public:
    //functions
    MemorySystem(unsigned id,unsigned _dmc_id,ostream &DDRSim_log_,string LogPath);
    virtual ~MemorySystem();
    void update();
    void communicate();

    bool addTransaction(Transaction *trans);
    bool addTransaction(bool isWrite, uint64_t addr);
    void noc_read_inform(bool fast_wakeup_rank0, bool fast_wakeup_rank1, bool bus_rempty);
    bool addData(uint32_t *data ,uint64_t id, bool ecc_flag);
    bool WillAcceptTransaction();
    void RegisterCallbacks(Callback_t *readData, Callback_t *writeDone, Callback_t *readDone, Callback_t *cmdDone);
    void check_bank(uint32_t *dmc_2up_bank, uint32_t rank, uint32_t type);
    void InitOutputFiles();
    uint8_t get_occ();
    uint32_t GetTransactionLen();
    uint32_t getTransQueSize(bool isRd);
    void dfs_backpress(bool backpress);
//    void dfs_backpress(bool backpress) {memoryController->dfs_backpress(backpress);};
    //fields
    MemoryController *memoryController;
    vector<Rank *> *ranks;

    //function pointers
    Callback_t* ReturnReadData;
    Callback_t* WriteResp;
    Callback_t* ReadResp;
    Callback_t* CmdResp;
    unsigned dmc_id;
    unsigned systemID;
    string log_path;
    //TODO: make this a functor as well?
    std::map<uint64_t,write_msg>write_map;
    Transaction* get_resource();
    void release_resource(Transaction* t);
    vector <Transaction *> PreDmcPipeQueue;

    string dmc_log;
    void update_print();

    ofstream state_log;
    ofstream inputs;
    ofstream ms_sim;
    uint64_t start_cycle;
    uint64_t end_cycle;
    uint64_t flow_statis_start_cycle;
    uint64_t flow_statis_end_cycle;

    uint32_t curFlowPressureLevel;

    uint64_t totalBytes;
    uint64_t totalWriteBytes;
    uint64_t totalReadBytes;
    uint64_t task_cnt;
    uint64_t total_task_cnt;
    bool enable_statistics;
    uint64_t access_cnt;
    uint64_t bp_cnt;
    uint64_t total_access_cnt;
    uint64_t total_bp_cnt;
    
    //added for trans_fifo
    unsigned trans_fifo_data_cnt;
    bool trans_fifo_full;

    void statistics();
    float flowStatistic();
    uint32_t updateFlowState(float total_bw);

    uint32_t getFlowPressureLevel();
    void GetQueueCmdNum(unsigned *dmc_rd_num, unsigned *dmc_wr_num,
            unsigned *gbuf_rd_num, unsigned *gbuf_wr_num);
    void GetDmcBusyStatus(bool *dmc_busy);
    void UnitConvert(double *oenergy, string *ouint, double ienergy);

    vector<uint32_t> que_cnt;
    vector<uint32_t> que_rd_cnt;
    vector<uint32_t> que_wr_cnt;
    vector<uint32_t> pre_acc_rank_cnt;
    vector<uint32_t> pre_acc_bank_cnt;
    vector<uint32_t> pre_racc_rank_cnt;
    vector<uint32_t> pre_racc_bank_cnt;
    vector<uint32_t> pre_wacc_rank_cnt;
    vector<uint32_t> pre_wacc_bank_cnt;

    uint32_t occ_1_cnt;
    uint32_t occ_2_cnt;
    uint32_t occ_3_cnt;
    uint32_t occ_4_cnt;
    vector<unsigned> pre_sch_level_cnt;

    void check_cnt();
    void register_write(uint64_t address, uint32_t data);
    void register_read(uint64_t address, uint32_t &data);

    unsigned pre_reads;
    unsigned pre_writes;
    unsigned pre_totals;
    unsigned pre_gbuf_reads;
    unsigned pre_gbuf_writes;
    unsigned pre_address_conf_cnt;
    unsigned pre_id_conf_cnt;
    unsigned pre_ba_conf_cnt;
    unsigned pre_total_conf;
    unsigned pre_act_cnt;
    unsigned pre_act_dst_cnt;
    unsigned pre_pre_sb_cnt;
    unsigned pre_pre_pb_cnt;
    unsigned pre_pre_pb_dst_cnt;
    unsigned pre_pre_ab_cnt;
    unsigned pre_read_cnt;
    unsigned pre_write_cnt;
    unsigned pre_pde_cnt;
    unsigned pre_asrefe_cnt;
    unsigned pre_srpde_cnt;
    unsigned pre_pdx_cnt;
    unsigned pre_asrefx_cnt;
    unsigned pre_srpdx_cnt;
    float pre_power;
    vector<unsigned> PreBankRowActCnt;

    unsigned pre_rmw_reads; 
    unsigned pre_rmw_bypass_reads; 
    unsigned pre_rmw_bypass_writes; 
    unsigned pre_rmw_writes; 
    unsigned pre_rmw_full_writes; 
    unsigned pre_rmw_mask_writes; 
    unsigned pre_rmw_totals;

    unsigned pre_merge_read_cnt;
    unsigned pre_ecc_read_cnt;
    unsigned pre_ecc_write_cnt;

    unsigned Total_func_pre_cnt;

    unsigned pre_read_p_cnt;
    unsigned pre_write_p_cnt;
    unsigned pre_mwrite_cnt;
    unsigned pre_mwrite_p_cnt;
    unsigned pre_timeout_cnt;
    unsigned pre_rt_timeout_cnt;
    unsigned pre_row_hit_cnt;
    unsigned pre_row_miss_cnt;
    unsigned pre_rw_switch_cnt;
    unsigned pre_rank_switch_cnt;
    unsigned pre_sid_switch_cnt;
    unsigned pre_refresh_pb_cnt;
    unsigned pre_refresh_ab_cnt;
    unsigned pre_r2w_switch_cnt;
    unsigned pre_w2r_switch_cnt;
    unsigned pre_phy_notlp_cnt;
    unsigned pre_phy_lp_cnt;
    map <unsigned, unsigned> PreRdCntBl;
    map <unsigned, unsigned> PreWrCntBl;

    bool rd_one = true;
    bool rd_two = false;
    bool rd_three = false;

    bool wr_one = true;
    bool wr_two = false;
    bool wr_three = false;

    uint64_t pre_total_latency;
    unsigned pre_com_read_cnt;
    unsigned channel;
    uint64_t channel_ohot;
    void trans_init(Transaction *trans, uint64_t inject_time);
//    void trans_check(Transaction *trans);

private:
    vector <uint32_t> prePdTime;
    vector <uint32_t> preAsrefTime;
    vector <uint32_t> preSrpdTime;
    vector <uint32_t> preWakeUpTime;
    vector <bus_state> BusStateAsync;
    vector <vector<unsigned>> pre_abr_cnt;
    vector <unsigned> pre_pbr_cnt;
};
}
#endif