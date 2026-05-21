#ifndef _LPDDR_LPMEMORYSYSTEMTOP_H_
#define _LPDDR_LPMEMORYSYSTEMTOP_H_

#include "SimulatorObject.h"
#include "Transaction.h"
#include "SystemConfiguration.h"
#include "MemorySystem.h"
#include "Rmw.h"
#include "IniReader.h"
#include "assert.h"

namespace LPDDRSim {
struct cmd_message {
    Transaction * trans;
    size_t delay;
    bool can_send;

    cmd_message() {
        trans = NULL;
        delay = 0;
        can_send = false;
    }
};

struct data_message {
    uint32_t * trans;
    size_t delay;
};


class LPMemorySystemTop : public SimulatorObject {
public:
#ifdef SYSARCH_PLATFORM
    LPMemorySystemTop(unsigned id, string IniFilePath = "./parameter", string LogPath = "./log",
            HALib::Configurable* cfg = NULL);
#else
    LPMemorySystemTop(unsigned id, string IniFilePath = "./parameter", string LogPath = "./log",
            int argc = 0, char *argv[] = NULL);
#endif
    virtual ~LPMemorySystemTop();
    bool addTransaction(const hha_command &command);
//    void trans_init(Transaction *trans, uint64_t inject_time);
    void trans_check(Transaction *trans);
    void noc_read_inform(uint8_t channel, bool fast_wakeup_rank0, bool fast_wakeup_rank1, bool bus_rempty);
    void update();
    void RegisterCallbacks(TransactionCompleteCB *readData,
            TransactionCompleteCB *writeDone,TransactionCompleteCB *readDone, TransactionCompleteCB *cmdDone);
    void InitOutputFiles();
    bool addData(uint32_t *data,uint32_t channel,uint64_t id);
    void read_complete(unsigned id, uint64_t address, uint64_t clk);
    void write_complete(unsigned id, uint64_t address, uint64_t clk);
    void register_read(uint64_t address ,uint32_t &data);
    void register_write(uint64_t address , uint32_t data ,uint32_t chl_id);
    uint8_t get_occ(uint8_t chl);
    uint8_t get_bandwidth(uint8_t chl);
    void getDMCBGPar(uint32_t &bg_pos ,uint32_t &xor_mode);

    uint32_t getDmcPressureLevel();

    uint32_t getTransQueSize(uint32_t dmc_id, bool isRd);
    uint32_t getRmwQueueCmdNum(uint8_t ch) const;

    //output file
    ofstream DDRSim_log;
    uint32_t curr_index;
    uint32_t pre_index;
    vector<uint32_t> clock_que;
    vector<uint32_t> over_clock_que;
    vector<uint32_t> start_clock_que;
    vector<uint32_t> data_clk_que;
    getfile param;
    vector<cmd_message>transaction;
    vector<data_message>data_que;
    void addData();
    uint32_t task_cnt;
    uint32_t com_cnt;
    void statistics(uint32_t channel_id);
    unsigned hhaId;
    void dfs_backpress(unsigned ch, bool backpress);
    void GetQueueCmdNum(uint8_t channel, unsigned *dmc_rd_num, unsigned *dmc_wr_num,
            unsigned *gbuf_rd_num, unsigned *gbuf_wr_num);
    void GetDmcBusyStatus(uint8_t channel, bool *dmc_busy);
    vector<MemorySystem*> channels;
//    Rmw *rmw;

private:
    vector <hha_command> CommandDelay;
    string IniFilename;

    TransactionCompleteCB *read_cb;
    TransactionCompleteCB *write_cb;

    void command_check(const hha_command &c);
    void wdata_check(uint64_t task, uint8_t channel);
    uint8_t addr_map_ch(const hha_command &c);

#ifdef DDRC_NEED_DEBUG
    uint32_t test_mode;
    uint32_t cin_num;
    uint32_t send_cnt;
    bool fifo_not_empty;
#define FIFO_DEPTH (3)
#define EEROR_VALUE (3)
    vector<Transaction*>  TransactionFifo;
#endif
    };
}
#endif