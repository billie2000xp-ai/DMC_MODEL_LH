#include "write_buffer.h"
#include "MemoryController.h"
#include "MemorySystem.h"
#include <assert.h>
#include <iomanip>
using namespace LPDDRSim;

#define PROTECT_SUB(a) a = (a > 0) ? (a - 1) : 0;

WriteBuff::WriteBuff(MemoryController *_top,unsigned id, ostream &DDRSim_log_) :
    top(_top),channel(id),DDRSim_log(DDRSim_log_) {
    channel_ohot = 1ull << channel;
    wb_bank_cnt.resize(NUM_RANKS);
    rb_bank_cnt.resize(NUM_RANKS);
    for (auto &wb_bank:wb_bank_cnt) {
        wb_bank.resize(NUM_BANKS);
    }
    for (auto &rb_bank:rb_bank_cnt) {
        rb_bank.resize(NUM_BANKS);
    }
    no_cmd_sch_cnt = 0;
    no_cmd_sch_th = 0;
    wbuff_state_gap = 0;
    wbuff_state = WBUFF_IDLE;
    wb_rank_cnt.resize(NUM_RANKS);
    rb_rank_cnt.resize(NUM_RANKS);
    wb_qos_cnt.resize(NUM_RANKS);
    rb_qos_cnt.resize(NUM_RANKS);
    for (size_t i = 0; i < 8; i ++) {
        wb_qos_cnt.push_back(0);
        rb_qos_cnt.push_back(0);
    }
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        wb_rank_cnt.push_back(0);
        rb_rank_cnt.push_back(0);
    }
    ser_write_cnt = 0;
    wcmd_cnt = 0;
    rcmd_cnt = 0;
    max_rank = 0;
    read_cnt = 0;
    write_cnt = 0;
    bytes_per_col = JEDEC_DATA_BUS_BITS / 8;
    forward_cnt = 0;
    merge_cnt = 0;
    push_cnt = 0;
    same_bank_cnt = 0;
    log_path = top->log_path;
    rcmd_bank_state.resize(NUM_RANKS * NUM_BANKS);
    WdataToSend.clear();
    pre_req_time = 0xFFFFFFFFFFFFFFFF;

    sch_level_cnt.clear(); sch_level_cnt.resize(7);
    for (size_t i = 0; i < 7; i ++) {
        sch_level_cnt.push_back(0);
    }

    wr_level.clear(); wr_level.resize(4);
    wr_most_level.clear(); wr_most_level.resize(4);
    rd_level.clear(); rd_level.resize(4);
    grp_mode.clear(); grp_mode.resize(4);
    for (size_t i = 0; i < 4; i ++) {
        wr_level[i].clear(); wr_level[i].resize(5);
        wr_most_level[i].clear(); wr_most_level[i].resize(5);
        rd_level[i].clear(); rd_level[i].resize(5);
        grp_mode[i].clear(); grp_mode[i].resize(5);
        for (size_t j = 0; j < 5; j ++) {
            wr_level[i].push_back(NULL);
            wr_most_level[i].push_back(NULL);
            rd_level[i].push_back(NULL);
            grp_mode[i].push_back(NULL);
        }
    }
    for (size_t i = 0; i < 5; i ++) {
        wr_level[0][i] = &MAP_CONFIG["WR_LEVEL0"][i];
        wr_level[1][i] = &MAP_CONFIG["WR_LEVEL1"][i];
        wr_level[2][i] = &MAP_CONFIG["WR_LEVEL2"][i];
        wr_level[3][i] = &MAP_CONFIG["WR_LEVEL3"][i];
        wr_most_level[0][i] = &MAP_CONFIG["WR_MOST_LEVEL0"][i];
        wr_most_level[1][i] = &MAP_CONFIG["WR_MOST_LEVEL1"][i];
        wr_most_level[2][i] = &MAP_CONFIG["WR_MOST_LEVEL2"][i];
        wr_most_level[3][i] = &MAP_CONFIG["WR_MOST_LEVEL3"][i];
        rd_level[0][i] = &MAP_CONFIG["RD_LEVEL0"][i];
        rd_level[1][i] = &MAP_CONFIG["RD_LEVEL1"][i];
        rd_level[2][i] = &MAP_CONFIG["RD_LEVEL2"][i];
        rd_level[3][i] = &MAP_CONFIG["RD_LEVEL3"][i];
        grp_mode[0][i] = &MAP_CONFIG["GRP_MODE_LEVEL0"][i];
        grp_mode[1][i] = &MAP_CONFIG["GRP_MODE_LEVEL1"][i];
        grp_mode[2][i] = &MAP_CONFIG["GRP_MODE_LEVEL2"][i];
        grp_mode[3][i] = &MAP_CONFIG["GRP_MODE_LEVEL3"][i];
    }
}

void WriteBuff::rcmd_push_wcmd(Transaction * t) {
    for (auto &w : Wbuff) {
        if (t->bankIndex == w->bankIndex && t->row == w->row && t->col == w->col) {
            push_cnt ++;
            if (GrpMode.grp_mode6) { // priority is 1
                w->pri = 1;
            } else { // high priority
                w->timeout = true;
                wr_timeout_cnt ++;
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- ADD_PUSH :: task="<<t->task<<hex<<" address="<<t->address<<dec
                        <<" rank="<<t->rank<<" bank="<<t->bankIndex<<" row="<<t->row<<" w_task="<<w->task<<endl);
            }
        }
    }
}

void WriteBuff::rcmd_set_conflict(Transaction * t) {
    conf_state *c = new conf_state;

    for (auto &w : Wbuff) {
        if (t->group == w->group) c->gbuf.bg_conf_cnt ++;
        if (t->bankIndex == w->bankIndex) c->gbuf.ba_conf_cnt ++;
        if (t->bankIndex == w->bankIndex && t->row == w->row && t->col == w->col) c->gbuf.ad_conf_cnt ++;
    }
    for (auto &q : top->transactionQueue) {
        if (t->group == q->group) c->dmc.bg_conf_cnt ++;
        if (t->bankIndex == q->bankIndex) c->dmc.ba_conf_cnt ++;
        if (t->bankIndex == q->bankIndex && t->row == q->row && t->col == q->col) c->dmc.ad_conf_cnt ++;
    }

    RbuffConfCnt.push_back(c);
}

void WriteBuff::rcmd_set_timeout(Transaction * trans) {
    if (GBUF_RCMD_TIMEOUT == 0) return;

    trans->timeout_th = now() + GBUF_RCMD_TIMEOUT;
}

void WriteBuff::wcmd_set_conflict(Transaction * t) {
    conf_state *c = new conf_state;

    if (GrpMode.grp_mode1 == 0) {
        for (auto &r : Rbuff) {
            if (t->group == r->group) c->gbuf.bg_conf_cnt ++;
            if (t->bankIndex == r->bankIndex) c->gbuf.ba_conf_cnt ++;
            if (t->bankIndex == r->bankIndex && t->row == r->row && t->col == r->col) c->gbuf.ad_conf_cnt ++;
        }
    }
    for (auto &q : top->transactionQueue) {
        if (GrpMode.grp_mode1 == 1 && GrpMode.grp_mode7 == 1 && q->transactionType == DATA_READ) continue;
        if (t->group == q->group) c->dmc.bg_conf_cnt ++;
        if (t->bankIndex == q->bankIndex) c->dmc.ba_conf_cnt ++;
        if (t->bankIndex == q->bankIndex && t->row == q->row && t->col == q->col) c->dmc.ad_conf_cnt ++;
    }

    WbuffConfCnt.push_back(c);
}

void WriteBuff::rcmd_release_conflict(Transaction *trans) {
    unsigned size = Rbuff.size();
    for (unsigned i = 0; i < size; i ++) {
        if (trans->task == Rbuff[i]->task) continue;
        if (trans->group == Rbuff[i]->group) RbuffConfCnt[i]->dmc.bg_conf_cnt ++;
        if (trans->bankIndex == Rbuff[i]->bankIndex) RbuffConfCnt[i]->dmc.ba_conf_cnt ++;
        if (trans->bankIndex == Rbuff[i]->bankIndex && trans->row == Rbuff[i]->row && trans->col == Rbuff[i]->col)
            PROTECT_SUB(RbuffConfCnt[i]->dmc.ad_conf_cnt);
    }

    size = Wbuff.size();
    for (unsigned i = 0; i < size; i ++) {
        if (trans->group == Wbuff[i]->group) PROTECT_SUB(WbuffConfCnt[i]->gbuf.bg_conf_cnt);
        if (trans->bankIndex == Wbuff[i]->bankIndex) PROTECT_SUB(WbuffConfCnt[i]->gbuf.ba_conf_cnt);
        if (trans->bankIndex == Wbuff[i]->bankIndex && trans->row == Wbuff[i]->row && trans->col == Wbuff[i]->col)
            PROTECT_SUB(WbuffConfCnt[i]->gbuf.ad_conf_cnt);
    }
}

void WriteBuff::wcmd_release_conflict(Transaction *trans) {
    unsigned size = Rbuff.size();
    for (unsigned i = 0; i < size; i ++) {
        if (trans->group == Rbuff[i]->group) PROTECT_SUB(RbuffConfCnt[i]->gbuf.bg_conf_cnt);
        if (trans->bankIndex == Rbuff[i]->bankIndex) PROTECT_SUB(RbuffConfCnt[i]->gbuf.ba_conf_cnt);
        if (trans->bankIndex == Rbuff[i]->bankIndex && trans->row == Rbuff[i]->row && trans->col == Rbuff[i]->col)
            PROTECT_SUB(RbuffConfCnt[i]->gbuf.ad_conf_cnt);
    }

    size = Wbuff.size();
    for (unsigned i = 0; i < size; i ++) {
        if (trans->task == Wbuff[i]->task) continue;
        if (trans->group == Wbuff[i]->group) WbuffConfCnt[i]->dmc.bg_conf_cnt ++;
        if (trans->bankIndex == Wbuff[i]->bankIndex) WbuffConfCnt[i]->dmc.ba_conf_cnt ++;
        if (trans->bankIndex == Wbuff[i]->bankIndex && trans->row == Wbuff[i]->row && trans->col == Wbuff[i]->col)
            PROTECT_SUB(WbuffConfCnt[i]->dmc.ad_conf_cnt);
    }
}

void WriteBuff::dmc_release_conflict(Transaction *trans) {
    unsigned size = Rbuff.size();
    for (unsigned i = 0; i < size; i ++) {
        if (trans->group == Rbuff[i]->group) PROTECT_SUB(RbuffConfCnt[i]->dmc.bg_conf_cnt);
        if (trans->bankIndex == Rbuff[i]->bankIndex) PROTECT_SUB(RbuffConfCnt[i]->dmc.ba_conf_cnt);
        if (trans->bankIndex == Rbuff[i]->bankIndex && trans->row == Rbuff[i]->row && trans->col == Rbuff[i]->col)
            PROTECT_SUB(RbuffConfCnt[i]->dmc.ad_conf_cnt);
    }

    size = Wbuff.size();
    for (unsigned i = 0; i < size; i ++) {
        if (trans->group == Wbuff[i]->group) PROTECT_SUB(WbuffConfCnt[i]->dmc.bg_conf_cnt);
        if (trans->bankIndex == Wbuff[i]->bankIndex) PROTECT_SUB(WbuffConfCnt[i]->dmc.ba_conf_cnt);
        if (trans->bankIndex == Wbuff[i]->bankIndex && trans->row == Wbuff[i]->row && trans->col == Wbuff[i]->col)
            PROTECT_SUB(WbuffConfCnt[i]->dmc.ad_conf_cnt);
    }
}

void WriteBuff::trans_state_clr(Transaction * trans) {
    trans->timeout = false;
    trans->has_active = false;
}

bool WriteBuff::read_forward(Transaction * trans) {
    for (auto &wbuf : Wbuff) {
        if (trans->rank != wbuf->rank) continue;
        if (trans->bankIndex != wbuf->bankIndex) continue;
        if (trans->row != wbuf->row) continue;
        if (wbuf->data_ready_cnt <= wbuf->burst_length) continue;
        if (trans->col >= wbuf->col && (trans->col + 64/bytes_per_col) <= (wbuf->col + 64/bytes_per_col)) {
            for (size_t i = 0; i <= trans->burst_length; i ++) {
                unsigned cnt = i % 2 + 1;
                top->gen_rdata(trans->task, cnt, 0, trans->mask_wcmd);
            }
            if (trans->data_size == 64) forward_cnt ++;
            top->gen_wresp(trans->task);

            if (FASTWAKEUP_EN && !PREDICT_FASTWAKEUP) {
                if (top->fast_wakeup_cnt[trans->rank] == 0 && now() >= 100) {
                    ERROR(setw(10)<<now()<<" -- GBUF["<<channel<<"] Error fast wakeup count!");
                    assert(0);
                }
                top->fast_wakeup_cnt[trans->rank] -= 1;
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- GBUF_FORWARD :: task="<<trans->task<<hex<<" address="
                        <<trans->address<<dec<<" (R:"<<rcmd_cnt<<"|W:"<<wcmd_cnt<<")"<<endl);
            }
            return true;
        }
    }
    return false;
}

bool WriteBuff::write_merge(Transaction * trans) {
    for (auto &wbuf : Wbuff) {
        if (wbuf->data_size != 64) continue;
        if (trans->rank != wbuf->rank) continue;
        if (trans->bankIndex != wbuf->bankIndex) continue;
        if (wbuf->data_ready_cnt <= wbuf->burst_length) continue;
        if ((trans->address & ALIGNED_DATA_64B) == (wbuf->address & ALIGNED_DATA_64B)
                && (trans->address & ALIGNED_NUMB_64B) != (wbuf->address & ALIGNED_NUMB_64B)) {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- GBUF_MERGE :: Send task="<<trans->task<<" address="<<hex<<trans->address
                        <<dec<<", Merge task="<<wbuf->task<<" address="<<hex<<wbuf->address<<dec<<endl);
            }
            merge_cnt ++;
            wbuf->burst_length = trans->burst_length * 2 + 1;
            wbuf->data_ready_cnt = wbuf->burst_length + 1;
            wbuf->data_size = 128;
            wbuf->address = wbuf->address & ~ALIGNED_NUMB_64B;
            wbuf->col = wbuf->col & trans->col;
            wbuf->addr_col = wbuf->addr_col & trans->addr_col;
            return true;
        }
    }

    return false;
}

void WriteBuff::addData(uint32_t *data ,uint64_t task) {
    for (auto &wbuf : Wbuff) {
        if (task != wbuf->task) continue;
        if ((wbuf->transactionType == DATA_WRITE)
                && wbuf->data_ready_cnt <= wbuf->burst_length) {
            wbuf->data_ready_cnt ++;
            if (DEBUG_BUS) {
                 PRINTN(setw(10)<<now()<<" -- GBUF_MATCH :: data_ready_cnt:"<<wbuf->data_ready_cnt
                         <<", "<<wbuf->data_size<<", task="<<wbuf->task<<endl);
            }
            return;
        }
    }
}

bool WriteBuff::addTransaction(Transaction * trans) {
    bool rbuf_full = rcmd_cnt >= WB_RDEPTH && WB_RDEPTH != 0;
    bool wbuf_full = wcmd_cnt >= WB_WDEPTH && WB_WDEPTH != 0;

    if (rbuf_full || wbuf_full) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- GBUF_DROP :: task="<<trans->task<<" type="<<trans->transactionType
                    <<" qos="<<trans->qos<<" burst_length:"<<trans->burst_length<<" address="<<hex<<trans->address
                    <<dec<<" rank="<<trans->rank<<" bank="<<trans->bankIndex<<" row="<<trans->row<<" (R:"<<rcmd_cnt
                    <<"|W:"<<wcmd_cnt<<")"<<endl);
        }
        return false;
    }

    pre_req_time = now();
    if (trans->transactionType == DATA_READ) {
        if (FORWARD_ENABLE && trans->data_size == 64 && read_forward(trans)) return true;
        if (RQ_ADCONF_PUSH_EN) rcmd_push_wcmd(trans);
        rcmd_set_conflict(trans);
        rcmd_set_timeout(trans);
        if (DYN_BYP_EN && !top->full()/* && rcmd_cnt == 0*/) {
            trans->arb_time = now() + 1;
            trans->has_active = true; // use as bypass read
        } else {
            trans->arb_time = now() + 6;
        }

        read_cnt ++;
        rcmd_cnt ++;
        Rbuff.push_back(trans);

        rb_bank_cnt[trans->rank][trans->bankIndex % NUM_BANKS] ++;
        rb_rank_cnt[trans->rank] ++;
        rb_qos_cnt[trans->qos] ++;
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- GBUF_ADD :: [R]B["<<trans->burst_length<<"]"<<"QOS["<<trans->qos<<"] addr="<<hex
                    <<trans->address<<dec<<" task="<<trans->task<<" rank="<<trans->rank<<" group="<<trans->group<<" bank="
                    <<trans->bankIndex<<" row="<<trans->row<<" (R:"<<rcmd_cnt<<"|W:"<<wcmd_cnt<<")"<<endl);
        }
        return true;
    } else {
        if (MERGE_ENABLE && trans->data_size == 64 && write_merge(trans)) return true;
        wcmd_set_conflict(trans);
        write_cnt ++;
        wcmd_cnt ++;
        trans->arb_time = now() + 8;

        Wbuff.push_back(trans);
        assert(wb_rank_cnt[trans->rank] <= WB_WDEPTH && "rank counter error");
        wb_rank_cnt[trans->rank] ++;
        wb_qos_cnt[trans->qos] ++;
        wb_bank_cnt[trans->rank][trans->bankIndex % NUM_BANKS] ++;
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- GBUF_ADD :: [W]B["<<trans->burst_length<<"]"<<"QOS["<<trans->qos<<"] addr="<<hex
                    <<trans->address<<dec<<" task="<<trans->task<<" rank="<<trans->rank<<" group="<<trans->group<<" bank="
                    <<trans->bankIndex<<" row="<<trans->row<<" (R:"<<rcmd_cnt<<"|W:"<<wcmd_cnt<<")"<<endl);
        }
        return true;
    }
}

unsigned WriteBuff::get_max_rank() {
    unsigned ret_rank = 0;
    unsigned size = wb_rank_cnt[0];
    unsigned rank = 0;
    for (auto &cnt : wb_rank_cnt) {
        if (cnt > size) {
            ret_rank = rank;
            size = cnt;
        }
        rank ++;
    }
    return ret_rank;
}

void WriteBuff::func_check() {
    if (Wbuff.size() != wcmd_cnt) {
        ERROR(setw(10)<<now()<<" -- Wbuff Unmatch, Wbuff="<<Wbuff.size()<<", wcmd_cnt="<<wcmd_cnt);
        assert(0);
    }
    if (Rbuff.size() != rcmd_cnt) {
        ERROR(setw(10)<<now()<<" -- Rbuff Unmatch, Rbuff="<<Rbuff.size()<<", rcmd_cnt="<<rcmd_cnt);
        assert(0);
    }
}

void WriteBuff::update() {
    if (DEBUG_GBUF_STATE) {
        PRINTN(setw(10)<<now()<<" -- GBUF_STATE wbuff_state="<<wbuff_state<<", gbuf_rd="<<+rcmd_cnt<<", dmc_rd="
                <<top->Read_Cnt()<<", gbuf_wr="<<+wcmd_cnt<<", dmc_wr="<<top->Write_Cnt()<<endl);
    }
#if 0
    func_check();
#endif
    update_state();
    update_state_trig();
    check_timeout();
    sch_rque();
    sch_wque();
    arb_node();
    send_wdata();
}

void WriteBuff::update_state() {
    if (!DEBUG_GBUF_STATE) return;
    unsigned size = Rbuff.size();
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl)
    PRINTN("Gbuf Total Status: R:"<<rcmd_cnt<<" W:"<<wcmd_cnt<<" wbuff_state"<<wbuff_state<<" occ="<<top->occ
            <<" state_trig="<<state_trig<<endl);
    for (unsigned i = 0; i < size; i ++) {
        auto t = Rbuff[i];
        auto r = RbuffConfCnt[i];
        auto rg = RbuffConfCnt[i]->gbuf;
        auto rd = RbuffConfCnt[i]->dmc;
        PRINTN("Rbuf time: "<<now()<<" | task="<<t->task<<" | bank="<<t->bankIndex<<" | rank="<<t->rank<<" | row="
                <<t->row<<" | address="<<hex<<t->address<<dec<<" | len="<<t->burst_length<<" | data_size="<<t->data_size
                <<" | data_ready_cnt="<<t->data_ready_cnt<<" | timeout="<<t->timeout<<" | qos="<<t->qos<<" | pri="<<t->pri
                <<" rd_byp="<<t->has_active<<" | rhit="<<r->rowhit<<" | g_bg_cc="<<rg.bg_conf_cnt<<" | g_ba_cc="
                <<rg.ba_conf_cnt<<" | g_ad_cc="<<rg.ad_conf_cnt<<" | d_bg_cc="<<rd.bg_conf_cnt<<" | d_ba_cc="
                <<rd.ba_conf_cnt<<" | d_ad_cc="<<rd.ad_conf_cnt<<" | r_bank_cnt="<<top->r_bank_cnt[t->bankIndex]
                <<" | w_bank_cnt="<<top->w_bank_cnt[t->bankIndex]<<endl);
    }
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl)
    size = Wbuff.size();
    for (unsigned i = 0; i < size; i ++) {
        auto t = Wbuff[i];
        auto w = WbuffConfCnt[i];
        auto wg = WbuffConfCnt[i]->gbuf;
        auto wd = WbuffConfCnt[i]->dmc;
        PRINTN("Wbuf time: "<<now()<<" | task="<<t->task<<" | bank="<<t->bankIndex<<" | rank="<<t->rank<<" | row="
                <<t->row<<" | address="<<hex<<t->address<<dec<<" | len="<<t->burst_length<<" | data_size="<<t->data_size
                <<" | data_ready_cnt="<<t->data_ready_cnt<<" | timeout="<<t->timeout<<" | qos="<<t->qos<<" | pri="<<t->pri
                <<" rd_byp="<<t->has_active<<" | rhit="<<w->rowhit<<" | g_bg_cc="<<wg.bg_conf_cnt<<" | g_ba_cc="
                <<wg.ba_conf_cnt<<" | g_ad_cc="<<wg.ad_conf_cnt<<" | d_bg_cc="<<wd.bg_conf_cnt<<" | d_ba_cc="
                <<wd.ba_conf_cnt<<" | d_ad_cc="<<wd.ad_conf_cnt<<" | r_bank_cnt="<<top->r_bank_cnt[t->bankIndex]
                <<" | w_bank_cnt="<<top->w_bank_cnt[t->bankIndex]<<endl);
    }
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl)
}

void WriteBuff::update_state_trig() {
    if (wbuff_state == WBUFF_IDLE) {
        unsigned rd_cnt = 0;
        if (TOTAL_RCMD_MODE == 0) rd_cnt = top->Read_Cnt() + rcmd_cnt;
        else if (TOTAL_RCMD_MODE == 1) rd_cnt = rcmd_cnt;
        else rd_cnt = top->Read_Cnt();

        state_trig = 0;
        if (wcmd_cnt > *wr_level[top->occ][4] && rd_cnt < *rd_level[top->occ][4]) state_trig = 5;
        else if (wcmd_cnt <= *wr_level[top->occ][4] && wcmd_cnt > *wr_level[top->occ][3]
                && rd_cnt < *rd_level[top->occ][3]) state_trig = 4;
        else if (wcmd_cnt <= *wr_level[top->occ][3] && wcmd_cnt > *wr_level[top->occ][2]
                && rd_cnt < *rd_level[top->occ][2]) state_trig = 3;
        else if (wcmd_cnt <= *wr_level[top->occ][2] && wcmd_cnt > *wr_level[top->occ][1]
                && rd_cnt < *rd_level[top->occ][1]) state_trig = 2;
        else if (wcmd_cnt <= *wr_level[top->occ][1] && wcmd_cnt > *wr_level[top->occ][0]
                && rd_cnt < *rd_level[top->occ][0]) state_trig = 1;

        if (state_trig == 0 && wcmd_cnt > 0 && (top->Read_Cnt() + rcmd_cnt) == 0) {
            if (idle_trig_time == 255) state_trig = 1; // idle trig
            else idle_trig_time ++;
        } else {
            idle_trig_time = 0;
        }

        if (state_trig == 0 && wr_timeout_cnt > 0) state_trig = 1;
    }

    if (state_trig != 0) {
        GrpMode.grp_mode0 = BIT_GET(*grp_mode[top->occ][state_trig - 1], 0, 1);
        GrpMode.grp_mode1 = BIT_GET(*grp_mode[top->occ][state_trig - 1], 1, 1);
        GrpMode.grp_mode23 = BIT_GET(*grp_mode[top->occ][state_trig - 1], 2, 2);
        GrpMode.grp_mode4 = BIT_GET(*grp_mode[top->occ][state_trig - 1], 4, 1);
        GrpMode.grp_mode5 = BIT_GET(*grp_mode[top->occ][state_trig - 1], 5, 1);
        GrpMode.grp_mode6 = BIT_GET(*grp_mode[top->occ][state_trig - 1], 6, 1);
        GrpMode.grp_mode7 = BIT_GET(*grp_mode[top->occ][state_trig - 1], 7, 1);
    }

    if (wbuff_state_gap > 0) wbuff_state_gap --;

    if (GBUF_RCMD_BLOCK_PBR) {
        for (size_t i = 0; i < NUM_RANKS * NUM_BANKS; i++) rcmd_bank_state[i] = false;
        if (rcmd_cnt != 0) for (auto &rbuf : Rbuff) rcmd_bank_state[rbuf->bankIndex] = true;
    }
}

void WriteBuff::check_timeout() {
    if (GBUF_RCMD_TIMEOUT != 0) {
        for (auto rbuf : Rbuff) {
            if ((now() >= rbuf->timeout_th || rbuf->pri == 0) && !rbuf->timeout) {
                rbuf->timeout = true;
                rd_timeout_cnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- GBUF_RTO, task="<<rbuf->task<<" pri="<<rbuf->pri<<endl);
                }
            }
        }
    }

    if (GBUF_WCMD_PRIADAPT != 0 && now() != 0 && (now() % GBUF_WCMD_PRIADAPT) == 0) {
        for (auto wbuf : Wbuff) {
            if (wbuf->pri > 0) wbuf->pri --;
            if (wbuf->pri == 0 && !wbuf->timeout) {
                wbuf->timeout = true;
                wr_timeout_cnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- GBUF_WTO, task="<<wbuf->task<<" pri="<<wbuf->pri<<endl);
                }
            }
        }
    }
}

void WriteBuff::sch_rque() {
    if (top->full()) return;
    if (wbuff_state == WBUFF_IDLE) {
        unsigned size = Rbuff.size();
        for (unsigned i = 0; i < size; i ++) {
            if (now() < Rbuff[i]->arb_time) continue;
            //if (top->r_bank_cnt[Rbuff[i]->bankIndex] > 0) continue; // OPT
            if ((RbuffConfCnt[i]->dmc.ad_conf_cnt + RbuffConfCnt[i]->gbuf.ad_conf_cnt) > 0) continue;

            arb_cmd *cmd = new arb_cmd;
            cmd->creat(Rbuff[i]);

            unsigned bank_cnt = 0;
            if (RQ_BACONF_MODE == 0) bank_cnt = top->bank_cnt[cmd->bankIndex];
            else if (RQ_BACONF_MODE == 1) bank_cnt = top->r_bank_cnt[cmd->bankIndex];
            else bank_cnt = top->w_bank_cnt[cmd->bankIndex];

            if (Rbuff[i]->timeout || Rbuff[i]->has_active) cmd->pri = 0;
            else if (RbuffConfCnt[i]->rowhit && LastArbCmd.bankIndex == Rbuff[i]->bankIndex
                    && rd_sch_rhit_cnt == 1) cmd->pri = 1;
            else if (RbuffConfCnt[i]->rowhit && bank_cnt > 0) cmd->pri = 6;
            else if (RQ_RNKCONF_EN && cmd->rank != top->PreCmd.rank) cmd->pri = 7;
            //else if (RbuffConfCnt[i]->dmc.ba_conf_cnt > 0) cmd->pri = 6;
            else if (bank_cnt > 0) cmd->pri = 7;
            else cmd->pri = Rbuff[i]->pri;

            ArbCmd.push_back(cmd);
        }
    }
}

void WriteBuff::sch_wque() {
    switch (wbuff_state) {
        case WBUFF_IDLE : {
            if (wbuff_state_gap == 0 && state_trig != 0) {
                max_rank = get_max_rank();
                no_cmd_sch_cnt = 0;
                ser_sch_write = *wr_most_level[top->occ][state_trig - 1];
                same_bank_cnt = MAP_CONFIG["BANK_CMD_TH"][state_trig - 1];
                no_cmd_sch_th = MAP_CONFIG["NO_CMD_SCH_TH"][state_trig - 1];
                sch_level_cnt[state_trig] ++;
                wbuff_state = WBUFF_WRITE;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- GBUF_ST go to WRITE, WB[wcmd_cnt="<<wcmd_cnt<<" rcmd_cnt="
                            <<rcmd_cnt<<"] DMC[R:"<<top->Read_Cnt()<<" W:"<<top->Write_Cnt()
                            <<"] availability="<<top->availability<<" state_trig="<<state_trig<<endl);
                }
            }
            break;
        }
        case WBUFF_WRITE : {
            if (ser_write_cnt >= ser_sch_write || no_cmd_sch_cnt > no_cmd_sch_th) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- GBUF_ST go to IDLE, WB[wcmd_cnt="<<wcmd_cnt<<" rcmd_cnt="
                            <<rcmd_cnt<<"] DMC[R:"<<top->Read_Cnt()<<" W:"<<top->Write_Cnt()
                            <<"] availability="<<top->availability<<" ser_write="<<ser_write_cnt
                            <<" no_cmd_sch="<<no_cmd_sch_cnt<<endl);
                }
                if (state_trig == 5) wbuff_state_gap = 8; // 4 is rtl gap
                else wbuff_state_gap = 16 + ser_write_cnt;

                no_cmd_sch_cnt = 0;
                ser_write_cnt = 0;
                wbuff_state = WBUFF_IDLE;
            }
            unsigned size = Wbuff.size();
            for (unsigned i = 0; i < size; i ++) {
                if (now() < Wbuff[i]->arb_time) continue;
                if (Wbuff[i]->data_ready_cnt < (Wbuff[i]->burst_length + 1)) continue;

                unsigned bank_cnt = 0;
                if (WQ_BACONF_MODE == 0) bank_cnt = top->bank_cnt[Wbuff[i]->bankIndex];
                else if (WQ_BACONF_MODE == 1) bank_cnt = top->r_bank_cnt[Wbuff[i]->bankIndex];
                else bank_cnt = top->w_bank_cnt[Wbuff[i]->bankIndex];

                //added for lpddr6 e-mode
                unsigned sub_channel = (Wbuff[i]->bankIndex % NUM_BANKS) / top->sc_bank_num; 
                if (!Wbuff[i]->timeout) {
                    if (top->refreshPerBank[Wbuff[i]->bankIndex].refreshing) continue;
                    if (top->refreshALL[Wbuff[i]->rank][sub_channel].refreshing) continue;

                    bool bg_conf_vld = false;
                    //unsigned bg_conf_cnt = WbuffConfCnt[i]->dmc.bg_conf_cnt;
                    unsigned bg_conf_cnt = top->bg_cnt[Wbuff[i]->rank][Wbuff[i]->group];
                    if (GrpMode.grp_mode23 <= 1) bg_conf_vld = bg_conf_cnt >= 2;
                    else if (GrpMode.grp_mode23 == 2) bg_conf_vld = bg_conf_cnt >= 3;
                    else bg_conf_vld = bg_conf_cnt >= 4;
                    if (bg_conf_vld && !WbuffConfCnt[i]->rowhit) continue;

                    if (MAP_CONFIG["WGRP_RANK_MODE"][state_trig-1] == 0 && Wbuff[i]->rank != max_rank) continue;

                    if (GrpMode.grp_mode0 == 1) {
                        //if (WbuffConfCnt[i]->dmc.ba_conf_cnt > same_bank_cnt) continue;
                        if (bank_cnt > same_bank_cnt) continue;
                    }
                }

                arb_cmd *cmd = new arb_cmd;
                cmd->creat(Wbuff[i]);

                if (Wbuff[i]->timeout || Wbuff[i]->pri == 0) cmd->pri = 0;
                else if (WbuffConfCnt[i]->rowhit) cmd->pri = 0;
                //else if (WbuffConfCnt[i]->dmc.ba_conf_cnt > 0) cmd->pri = 3;
                else if (bank_cnt > 0) cmd->pri = 3;
                //else if (WbuffConfCnt[i]->dmc.bg_conf_cnt > 0) cmd->pri = 2;
                else if (top->bg_cnt[Wbuff[i]->rank][Wbuff[i]->group] > 0) cmd->pri = 2;
                else cmd->pri = 1;

                if (MAP_CONFIG["WGRP_RANK_MODE"][state_trig-1]==1 && Wbuff[i]->rank != max_rank)
                    if (cmd->pri != 0) cmd->pri += 10;
                    
                ArbCmd.push_back(cmd);
            }
            break;
        }
        default : break;
    }
}

void WriteBuff::arb_node() {
    if (wbuff_state == WBUFF_WRITE) {
        if (ArbCmd.empty()) no_cmd_sch_cnt ++;
        else no_cmd_sch_cnt = 0;
    }
    if (ArbCmd.empty()) return;

    arb_cmd *cur_cmd = NULL;
    for (auto &cmd : ArbCmd) {
        if (cur_cmd == NULL) {cur_cmd = cmd; continue;}
        if (priority(cmd) < priority(cur_cmd)) {
            cur_cmd = cmd;
        }
    }

    if (cur_cmd->type == DATA_READ) { // read command
        unsigned size = Rbuff.size();
        for (unsigned i = 0; i < size; i ++) {
            if (Rbuff[i]->task != cur_cmd->task) continue;

            if (top->addTransaction(Rbuff[i])) {
                if (rd_sch_rhit_cnt >= 1) rd_sch_rhit_cnt = 0;
                else if (RbuffConfCnt[i]->rowhit && LastArbCmd.bankIndex == Rbuff[i]->bankIndex) rd_sch_rhit_cnt ++;
                if (CMD_ROW_ORDER == 2) {
                    for (unsigned j = 0; j < size; j ++) {
                        if (RbuffConfCnt[j]->rowhit) continue;
                        if (Rbuff[i]->task == Rbuff[j]->task) continue;
                        if (Rbuff[i]->bankIndex != Rbuff[j]->bankIndex) continue;
                        if (Rbuff[i]->row != Rbuff[j]->row) continue;
                        RbuffConfCnt[j]->rowhit = true;
                    }
                }
                LastArbCmd.creat(Rbuff[i]);
                if (rcmd_cnt == 0) {
                    ERROR(setw(10)<<now()<<" -- rcmd_cnt is 0.");
                    assert(0);
                }
                rcmd_cnt --;
                rb_bank_cnt[cur_cmd->rank][cur_cmd->bankIndex % NUM_BANKS] --;
                rb_rank_cnt[cur_cmd->rank] --;
                rb_qos_cnt[Rbuff[i]->qos] --;
                if (Rbuff[i]->timeout) rd_timeout_cnt --;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- GBUF_SCH :: READ (R:"<<rcmd_cnt<<"|W:"<<wcmd_cnt<<"|QR:"<<top->Read_Cnt()
                            <<"|RW:"<<top->Write_Cnt()<<") task=" <<Rbuff[i]->task<<" addr="<<hex<<Rbuff[i]->address<<dec
                            <<" rank="<<Rbuff[i]->rank<<" bank="<<Rbuff[i]->bankIndex<<" row="<<Rbuff[i]->row<<" i="<<i<<" qos="
                            <<Rbuff[i]->qos<<" blen="<<Rbuff[i]->burst_length<<" rhit="<<RbuffConfCnt[i]->rowhit<<" rbc="
                            <<top->r_bank_cnt[Rbuff[i]->bankIndex]<<" wbc="<<top->w_bank_cnt[Rbuff[i]->bankIndex]
                            <<" timeout="<<Rbuff[i]->timeout<<endl);
                }
                rcmd_release_conflict(Rbuff[i]);
                trans_state_clr(Rbuff[i]);
                delete RbuffConfCnt[i];
                RbuffConfCnt.erase(RbuffConfCnt.begin() + i);
                Rbuff.erase(Rbuff.begin() + i);
            }
            break;
        }
    } else { // write command
        unsigned size = Wbuff.size();
        for (unsigned i = 0; i < size; i ++) {
            if (Wbuff[i]->task != cur_cmd->task) continue;

            if (top->addTransaction(Wbuff[i])) {
                if (RO_HIT_EN) {
                    for (unsigned j = 0; j < size; j ++) {
                        if (WbuffConfCnt[j]->rowhit) continue;
                        if (Wbuff[i]->task == Wbuff[j]->task) continue;
                        if (Wbuff[i]->bankIndex != Wbuff[j]->bankIndex) continue;
                        if (Wbuff[i]->row != Wbuff[j]->row) continue;
                        WbuffConfCnt[j]->rowhit = true;
                    }
                }
                LastArbCmd.creat(Wbuff[i]);
                for (size_t j = 0; j <= Wbuff[i]->burst_length; j ++) {
                    WdataToSend.push_back(Wbuff[i]->task);
                }
                if (Wbuff[i]->timeout) wr_timeout_cnt --;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- GBUF_SCH :: WRITE (R:"<<rcmd_cnt<<"|W:"<<wcmd_cnt<<"|QR:"<<top->Read_Cnt()
                            <<"|RW:"<<top->Write_Cnt()<<") task="<<Wbuff[i]->task<<" addr="<<hex<<Wbuff[i]->address<<dec
                            <<" rank="<<Wbuff[i]->rank<<" bank="<<Wbuff[i]->bankIndex<<" row="<<Wbuff[i]->row<<" i="<<i<<" qos="
                            <<Wbuff[i]->qos<<" blen="<<Wbuff[i]->burst_length<<" rhit="<<WbuffConfCnt[i]->rowhit<<" rbc="
                            <<top->r_bank_cnt[Wbuff[i]->bankIndex]<<" wbc="<<top->w_bank_cnt[Wbuff[i]->bankIndex]
                            <<" timeout="<<Wbuff[i]->timeout<<endl);
                }
                ser_write_cnt ++;
                if (wcmd_cnt == 0) {
                    ERROR(setw(10)<<now()<<" -- wcmd_cnt is 0.");
                    assert(0);
                }
                wcmd_cnt --;
                wb_rank_cnt[Wbuff[i]->rank] --;
                wb_qos_cnt[Wbuff[i]->qos] --;
                wb_bank_cnt[Wbuff[i]->rank][Wbuff[i]->bankIndex % NUM_BANKS] --;
                wcmd_release_conflict(Wbuff[i]);
                trans_state_clr(Wbuff[i]);
                delete WbuffConfCnt[i];
                WbuffConfCnt.erase(WbuffConfCnt.begin() + i);
                Wbuff.erase(Wbuff.begin() + i);
            }
            break;
        }
    }

    for (auto &cmd : ArbCmd) delete cmd;
    ArbCmd.clear();
}

unsigned WriteBuff::priority(arb_cmd *cmd) {
    return cmd->pri;
}

void WriteBuff::send_wdata() {
    if (!WdataToSend.empty()) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- GBUF_SEND_WDATA :: task="<<WdataToSend[0]<<endl);
        }
        top->receiveFromCPU(0, WdataToSend[0]);
        WdataToSend.erase(WdataToSend.begin());
    }
}