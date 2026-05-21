#ifndef _LP_TRANSACTION_H
#define _LP_TRANSACTION_H

#include "SystemConfiguration.h"
#include "BankState.h"
#include "ddr_common.h"
#include "math.h"
using std::ostream;

namespace LPDDRSim {

struct TRANS_MSG {
    unsigned long long time;
    double reqAddToDmcTime;
    double reqEnterDmcBufTime;
    uint8_t data_size;
    uint8_t burst_cnt;
    uint8_t channel;
    uint8_t rank;
    uint8_t hpc_time;
    uint8_t qos;
    unsigned mid;
    uint8_t burst_length;
    uint8_t pf_type;
    uint8_t sub_pftype;
    uint8_t sub_src;

    TRANS_MSG() {
        time = 0;
        reqAddToDmcTime = 0.0;
        reqEnterDmcBufTime = 0.0;
        data_size = 0;
        burst_cnt = 0;
        channel = 0;
        rank = 0;
        hpc_time = 0;
        qos = 0;
        mid = 0;
        burst_length = 0;
        pf_type = 0;
        sub_pftype = 0;
        sub_src = 0;
    }
};

struct data_packet {
    uint64_t task;
    uint32_t delay;
    uint32_t cnt;
    uint32_t channel;
    double readDataEnterDmcTime;
    bool mask_wcmd;
    TRANS_MSG msg;

    data_packet() {
        task  = 0;
        delay = 0;
        cnt   = 0;
        channel = 0;
        readDataEnterDmcTime = 0.0;
        mask_wcmd = false;
    }
};

struct COUNTER {
    bool enable;
    uint32_t cnt;

    COUNTER() {
        enable = false;
        cnt = 0;
    }
};


struct BankTableState {
    BankState *state;
    unsigned bank;
    unsigned rank;
    unsigned sid;
    unsigned group;
    unsigned bankIndex;
//    uint32_t channel;
    uint32_t last_activerow;
    bool     hold_precharge;
    bool     en_refresh_pb;
    bool     hold_refresh_pb;
    bool     finish_refresh_pb;
    bool     write_hold;//this is a flag for hold precharge
    unsigned ser_rhit_cnt;
    bool     has_rhit_break;
    bool     has_cmd_rowhit;
    bool     has_highqos_cmd_rowhit;
    bool     has_timeout;
    unsigned row_hit_cnt;
    unsigned row_miss_cnt;

    BankTableState(ostream &_log) {
        state = new BankState(_log);
        hold_precharge = false;
        write_hold = false;
        bank = 0;
        rank = 0;
        sid = 0;
        group = 0;
        last_activerow = 0;
        en_refresh_pb = false;
        hold_refresh_pb = false;
        finish_refresh_pb = false;
        ser_rhit_cnt = 0;
        has_rhit_break = false;
        has_cmd_rowhit = false;
        has_highqos_cmd_rowhit = false;
        has_timeout = false;
        row_hit_cnt = 0;
        row_miss_cnt = 0;
    }

    BankTableState(ostream &_log, size_t r, size_t s, size_t g, size_t b) {
        state = new BankState(_log);
        hold_precharge = false;
        write_hold = false;
        en_refresh_pb = false;
        hold_refresh_pb = false;
        finish_refresh_pb = false;
        bank = b;
        sid = s;
        group = g;
        rank = r;
        bankIndex = s * NUM_BANKS /NUM_SIDS + r * NUM_BANKS + g * NUM_BANKS /NUM_SIDS / NUM_GROUPS + b;
//        channel = c;
        last_activerow = 0;
        ser_rhit_cnt = 0;
        has_rhit_break = false;
        has_cmd_rowhit = false;
        has_highqos_cmd_rowhit = false;
        has_timeout = false;
        row_hit_cnt = 0;
        row_miss_cnt = 0;
    }
};

enum cmd_state {
    idle,
    working
};

#ifndef SYSARCH_PLATFORM
struct hha_command {
    uint32_t type;
    uint64_t address;
    uint32_t burst_length;
    uint32_t pri;
    uint32_t qos;
    uint32_t id;
    uint32_t channel;
    uint64_t task;
    double   reqEnterDmcBufTime;
    uint32_t mid;
    uint32_t mpam_id;
    bool     cmd_rt_type;
    bool     pre_act;
    bool     ecc_flag;
    bool     ap_cmd;
    bool     mask_wcmd;
    bool     wrap_cmd;
    unsigned pf_type;//0:DMD, 1:PFL1, 2:PFL2, 3:PFL3
    unsigned sub_pftype;//0:NonPf, 1:SEQ, 2:STRIDE, 3:SMS, 4:NL, 5:PCP, 6:BBOP,
                        //7:IFUPF, 8:META, 9:BOF, 10:PF_MAX, 11:CONTEXT, 12:SPP
    unsigned sub_src;//0:SS_Inst, 1:SS_Data, 2:SS_MMU

    hha_command() {
        reset();
    }

    void reset() {
        type = DATA_READ;
        address = 0;
        burst_length = 1;
        pri = 0;
        qos = 0;
        id = 0;
        channel = 0;
        task = 0;
        reqEnterDmcBufTime = 0.0;
        mid = 0;
        mpam_id = 0;
        cmd_rt_type = false;
        pre_act = false;
        ecc_flag = false;
        ap_cmd = false;
        mask_wcmd = false;
        wrap_cmd = false;
        pf_type = 0;
        sub_pftype = 0;
        sub_src = 0;
    }
};
#endif

struct DataPacket {
    unsigned long long task;
    unsigned delay;
    unsigned bl;
    bool mask_wcmd;

    DataPacket() {
        task = 0;
        delay = WL;
        bl = 0;
        mask_wcmd = false;
    }
};

struct FORALLREFRESH {
    unsigned refresh_cnt;
    unsigned rank;
    bool refreshWaiting;
    bool refreshing;

    FORALLREFRESH() {
    }

    FORALLREFRESH(unsigned index) {
        refreshWaiting = false;
        refresh_cnt = 0;
        rank = index;
        refreshing = false;
    }
};

struct PERBANKREFRESH {
    unsigned bank;
    bool refreshWaiting;
    bool refreshing;
    bool refreshWaitingPre;         // added for solving problems: refreshWaiting block precharge of other banks

    PERBANKREFRESH() {
    }

    PERBANKREFRESH(unsigned index) {
        bank = index;
        refreshWaiting = false;
        refreshing = false;
        refreshWaitingPre = false;
    }
};

//enum RmwState {
//    QUE_WAITING,
//    MERGE_READ,
//    SEND_READY
//};

class Transaction {
public:
    Transaction();
    ~Transaction() {}
    Transaction(const hha_command &c);
    void print(ostream &os);
    void print_inputs(ostream &os,uint64_t clk);
    void reset();

public:
    //fields
    TransactionType transactionType;
    TransactionCmd nextCmd;
//    RmwState rmwState;
    uint64_t arb_time;
    uint64_t enter_que_time;
    uint64_t address;
    uint64_t timeAdded;
    double reqAddToDmcTime;
    double reqEnterDmcBufTime;
    uint64_t async_delay_time;
    uint32_t improve_cnt;
    uint32_t pri;
    uint32_t qos;
    uint32_t sid;
    uint32_t group;
    uint32_t rank;
    uint32_t sc;
    uint32_t bank;
    uint32_t row;
    uint32_t col;
    uint32_t group_ini;           // initial group with system_meta
    uint32_t bank_ini;            // initial bank with system_meta
    uint32_t row_ini;             // initial row with system_meta
    uint32_t col_ini;             // initial col with system_meta
    uint32_t channel;             // subchannel for lpddr6
    uint32_t data_size;
    uint32_t burst_length;
    uint32_t issue_size;
    uint32_t trans_size;
    uint32_t addr_col;
    uint32_t addr_col_ini;        // initial addr_col with system_meta
    uint32_t bankIndex;
    uint64_t addr_block_source_id;
    uint64_t task;
    uint32_t data_ready_cnt;
    uint32_t timeout_th;
    uint32_t pri_adapt_th;
    bool     addrconf;
    bool     has_active; //just for statistics
    bool     timeout;
    bool     timeout_dec_once;
    uint64_t inject_time;
    bool     exec_wr;
    unsigned improve_time;
    bool     merge;
    bool     is_src_conf;
    unsigned block_num;
    unsigned position;
    uint32_t mid;
    uint32_t mpam_id;
    bool     cmd_rt_type;
    bool     cmd_hqos_type;
    bool     pre_act;
    bool     act_executing;
    bool     has_send_act;
    bool     ecc_flag;
    bool     ap_cmd;
    bool     mask_wcmd;
    bool     wrap_cmd;
    unsigned pf_type;
    unsigned sub_pftype;
    unsigned sub_src;
    unsigned bg_rotate_pri;
    uint64_t trcd_met;
    bool     bp_by_tout;
    unsigned bl;
    uint32_t arb_group;
    bool     lqos_bp;

    friend ostream &operator<<(ostream &os, const Transaction &t);
    //functions
    Transaction(const Transaction &t);
    Transaction(const Transaction *t);

};

struct Cmd {
    TransactionCmd      cmd_type;
    cmd_state           state;
    unsigned            row;
    unsigned            pri;
    unsigned            qos;
    unsigned            sid;
    unsigned            group;
    unsigned            rank;
    unsigned            bank;
    uint64_t            task;
    uint32_t            channel;     // subchannel for lpddr6
    unsigned            bankIndex;
    unsigned            fst_bankIndex;
    unsigned            lst_bankIndex;
    bool                bg_interleave;
    TransactionType     type;
    uint64_t            timeAdded;
    uint64_t            inject_time;
    unsigned            position;
    bool                timeout;
    bool                timeout_dec_once;
    bool                force_pbr;
    bool                pre_act;
    bool                mask_wcmd;
    uint32_t            issue_size;
    uint32_t            trans_size;
    uint32_t            addr_col;
    uint32_t            data_size;
    unsigned            bl;
    unsigned            bg_rotate_pri;
    unsigned            cmd_source; // 0->from Queue, 1->from page timeout, 2->from func
    bool                fg_ref;
    uint32_t            arb_group;

    Cmd() {
        reset();
    }

    void reset() {
        state = idle;
        cmd_type = INVALID;
        row = 0;
        pri = 0;
        qos = 0;
        sid = 0;
        group = 0;
        rank = 0;
        bank = 0;
        task = 0;
        channel = 0;
        bankIndex = 0;
        fst_bankIndex = 0;
        lst_bankIndex = 0;
        bg_interleave = false;
        type = DATA_READ;
        timeAdded = 0;
        inject_time = bankIndex = 0;
        position = INVALID_POSITION;
        timeout = false;
        timeout_dec_once = false;
        force_pbr = false;
        pre_act = false;
        mask_wcmd = false;
        issue_size = 0;
        trans_size = 0;
        addr_col = 0;
        data_size = 0;
        bl = 0;
        bg_rotate_pri = 0;
        cmd_source = 0;
        fg_ref = false;
        arb_group = 0;
    }

    Cmd(const Transaction &trans,bool first) {
        state = (first ? idle : working);
        cmd_type = trans.nextCmd;
        row = trans.row;
        pri = trans.pri;
        qos = trans.qos;
        sid = trans.sid;
        group = trans.group;
        rank = trans.rank;
        bank = trans.bank;
        channel = trans.channel;
        bankIndex = trans.bankIndex;
        fst_bankIndex = 0;
        lst_bankIndex = 0;
        task = trans.task;
        bg_interleave = (((trans.burst_length+1)*DMC_DATA_BUS_BITS/DmcLog2(BLEN,JEDEC_DATA_BUS_BITS)/PAM_RATIO)>1);
        type = trans.transactionType;
        timeAdded = trans.timeAdded;
        inject_time = trans.inject_time;
        position = trans.position;
        timeout = trans.timeout;
        timeout_dec_once = trans.timeout_dec_once;
        force_pbr = false;
        pre_act = trans.pre_act;
        mask_wcmd = trans.mask_wcmd;
        issue_size = trans.issue_size;
        trans_size = trans.trans_size;
        addr_col = trans.addr_col;
        data_size = trans.data_size;
        bl = trans.bl;
        bg_rotate_pri = trans.bg_rotate_pri;
        cmd_source = 0;
        fg_ref = false;
        arb_group = trans.arb_group;
    }
};

struct BusPacket {
    TransactionCmd type;
    unsigned row;
    unsigned bank;
    unsigned rank;
    unsigned sid;
    unsigned group;
//    uint32_t channel;
    unsigned bankIndex;
    unsigned fst_bankIndex;
    unsigned lst_bankIndex;
    uint64_t task;
    unsigned bl;
    unsigned cmd_source;
    bool fg_ref;
    bool mask_wcmd;
    unsigned trans_size;
    unsigned addr_col;

    BusPacket() {
        reset();
    }

    void reset() {
        type = INVALID;
        row = 0;
        bank = 0;
        rank = 0;
        sid = 0;
        group = 0;
//        channel = 0;
        task = 0;
        bankIndex = 0;
        fst_bankIndex = 0;
        lst_bankIndex = 0;
        bl = 0;
        cmd_source = 0;
        fg_ref = false;
        mask_wcmd = false;
        trans_size = 0;
        addr_col = 0;
    }

    void creat(Cmd *c) {
        row        = c->row;
        bank       = c->bank;
        rank       = c->rank;
        sid        = c->sid;
        group      = c->group;
//        channel    = c->channel;
        task       = c->task;
        type       = c->cmd_type;
        bankIndex  = c->bankIndex;
        fst_bankIndex  = c->fst_bankIndex;
        lst_bankIndex  = c->lst_bankIndex;
        bl         = c->bl;
        cmd_source = c->cmd_source;
        fg_ref     = c->fg_ref;
        mask_wcmd  = c->mask_wcmd;
        trans_size = c->trans_size;
        addr_col   = c->addr_col;
    }
};

struct phy {
    BusPacket command;
    uint32_t delay;

    phy() {
        command.reset();
        delay = 0;
    }
};

struct pbr_weight {
    unsigned bank_pair_cnt;
    bool send_pbr;
    unsigned bankIndex;
    unsigned fst_bankIndex;            // one bank of a bank pair
    unsigned lst_bankIndex;            // another bank of a bank pair
    unsigned bagroup;                // no of groups, in which banks have same ba1ba0
    bool block_pbr;

    pbr_weight(size_t index, size_t fst_index, size_t lst_index) {
        bank_pair_cnt = 0;
        send_pbr = false;
        bankIndex = index;
        fst_bankIndex = fst_index;
        lst_bankIndex = lst_index;
        bagroup = 0;
        block_pbr = false;
    }
};

enum LP_STATE {
    IDLE, PDE, PD, PDLP, PDX, ASREFE, ASREF, ASREFX, SRPDE, SRPD, SRPDLP, SRPDX
//  0     1    2   3     4    5       6      7       8      9     10      11
};

struct RankStatus {
    LP_STATE lp_state;
    unsigned pd_cnt;
    unsigned asref_cnt;
    unsigned state_cnt;
    unsigned rankIndex;
    uint64_t wck_off_time;
    bool wck_on;

    RankStatus(size_t index) {
        lp_state = IDLE;
        pd_cnt = 0;
        asref_cnt = 0;
        state_cnt = 0;
        wck_off_time = 0;
        wck_on = false;
        rankIndex = index;
    }
};

enum PHYLP_STATE {
    PHYLP_IDLE, PHYLPE, PHYLP, PHYLPX
//  0           1       2      3
};

struct PhyLpStatus {
    PHYLP_STATE phylp_state;
    unsigned lp_cnt;

    PhyLpStatus() {
        phylp_state = PHYLP_IDLE;
        lp_cnt = 0;
    }
};

struct FuncState {
    bool wakeup;
    uint64_t nextPde;

    FuncState() {
        wakeup = false;
        nextPde = 0;
    }
};

struct bus_state {
    bool fast_wakeup_rank0;
    bool fast_wakeup_rank1;
    bool bus_rempty;
    uint64_t valid_time;

    bus_state() {
        fast_wakeup_rank0 = false;
        fast_wakeup_rank1 = false;
        bus_rempty = false;
        valid_time = 0;
    }
};

enum BL_NUM {
    BL64 = 64, BL48 = 48, BL32 = 32, BL24 = 24, BL16 = 16,
    BL12 = 12, BL8 = 8, BL6 = 6, BL4 = 4, BL2 = 2
};

enum path {
    DMC_PATH,
    HPC_PATH
};

struct write_msg {
    path pt;
    uint8_t num_256bit;

    write_msg() {
        pt = DMC_PATH;
        num_256bit = 0;
    }
};
}
#endif