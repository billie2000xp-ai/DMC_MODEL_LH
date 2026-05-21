#include <stdio.h>
#include "Inline_ECC.h"
#include "MemoryController.h"
#include "MemorySystem.h"
#include <assert.h>
#include <iomanip>
#include "SystemConfiguration.h"
#include "AddressMapping.h"
using namespace std;
namespace LPDDRSim {
#define PDU_ROW 5ull
#define PDU_BG 32ull
#define PDU_BA 40ull
#define PDU_RA 48ull

#define MERGE_DEPTH 64ull //update 24/9/5
//============================================================================
Inline_ECC::Inline_ECC(MemoryController *_top,unsigned id, ostream &DDRSim_log_):
    top(_top),channel(id),DDRSim_log(DDRSim_log_) {
    channel_ohot = 1ull << channel;
    rd_ecc_buf.clear();
    wr_ecc_buf.reserve(PDU_DEPTH);
    ECC_BUF_Entry buf_entry;

    for (size_t i=0; i<PDU_DEPTH; i++) {
        buf_entry.buf_id        = i;
        buf_entry.ecc_buf_addr  = i;
        buf_entry.vld           = false;
        buf_entry.slt_cnt       = 0;
//        buf_entry.pdu_addr      = 0xffffffffffffffff;
        buf_entry.pdu_addr      = 0;
        buf_entry.eor           = false;
        buf_entry.wr_ecc_pos    = 0;
        buf_entry.rd_ecc_pos    = false;
        for (size_t k=0; k<16; k++) {
            buf_entry.wr_ecc_info[k]   = 0;
            buf_entry.rd_ecc_info[k]   = 0;
        }
        buf_entry.buf_ctrl_state = BUF_IDLE;
        buf_entry.ecc_dirty = false;
        buf_entry.rd_ecc = false;
        buf_entry.wr_ecc = false;

        buf_entry.ecc_pri=PDU_DEPTH;
        buf_entry.wr_merge_cnt=0;

        rd_ecc_buf.push_back(buf_entry);
        wr_ecc_buf.push_back(buf_entry);

    }

    counter = 0;
    wdata_full_counter = 0;
    wdata_256bit_num = 1;
    try_count=0;
    rhit=0;
    whit=0;
    merge_cnt=0;
    merge_id = PDU_DEPTH;
    wpos[17]={0};
    ecc_merge_flag=false;
    ecc_model_state = TRY_HIT_ECC_BUF;
    ecc_pre_state = TRY_HIT_ECC_BUF;
    avail_rd_ecc_buf_id = 0xffff;
    avail_wr_ecc_buf_id = 0xffff;
    pdu_addr = 0;
    ecc_delay = 0;
    //================ init ecc unoccupy buf and recycle buf ===============//
    index_recycle_rd_ecc_buf.clear();
    index_recycle_wr_ecc_buf.clear();
    assert(this->index_recycle_rd_ecc_buf.empty());//make sure clear the index_recycle_ecc_buf
    assert(this->index_recycle_wr_ecc_buf.empty());//make sure clear the index_recycle_ecc_buf

    ecc_task = 0x80000000;
    ecc_conflict_cnt = 0;
    iecc_add_cnt = 0;

    //IeccCmdQueue.reserve(3);
    //IeccWdata.reserve(3);
    iecc_cmd_cnt = 0;
    iecc_wdata_cnt = 0;
    backpress_cnt = 0;
    strategy= LRU;
    PREFETCH_ENABLE=true;
}
//============================================================================
Inline_ECC::~Inline_ECC() {
    rd_ecc_buf.clear();
    wr_ecc_buf.clear();
    index_recycle_rd_ecc_buf.clear();
    index_recycle_wr_ecc_buf.clear();
    assert(this->index_recycle_rd_ecc_buf.empty());//make sure clear the index_recycle_ecc_buf
    assert(this->index_recycle_wr_ecc_buf.empty());//make sure clear the index_recycle_ecc_buf

    return;
}
//==============================================================================
// try hit
// 0：rd_buf miss wr_buf miss
// 1：rd_buf hit  wr_buf miss
// 2: rd_buf miss wr_buf hit
// 3: rd_buf hit  wr_buf hit
uint32_t Inline_ECC::hit_ecc_buf(uint64_t pdu_address) {
    uint32_t rd_buf_hit = 0;
    uint32_t wr_buf_hit = 0;
    this->avail_rd_ecc_buf_id = 0xffff;
    this->avail_wr_ecc_buf_id = 0xffff;

    try_count++;
    for (size_t index=0; index<PDU_DEPTH; index++) {
        if (this->rd_ecc_buf.at(index).vld) {
            //======================check if hit ecc_buf=========================//
            if (pdu_address == this->rd_ecc_buf.at(index).pdu_addr) {
                rd_buf_hit = 1; // hit the index buf
                rhit++;
                this->avail_rd_ecc_buf_id = index;
                replace_pri_update(index,0);
            }
        }
    }

    for (size_t index=0; index<PDU_DEPTH; index++) {
        if (this->wr_ecc_buf.at(index).vld) {
            //======================check if hit ecc_buf=========================//
            if (pdu_address == this->wr_ecc_buf.at(index).pdu_addr) {
                wr_buf_hit = 1; // hit the index buf
                whit++;
                this->avail_wr_ecc_buf_id = index;
                this->wr_ecc_buf.at(index).wr_merge_cnt = 0;//24/9/6
                replace_pri_update(index,1);
            }
        }
    }
    return (wr_buf_hit*2 + rd_buf_hit);
}
//=================================================================================
// get ecc buf id
//=================================================================================
uint32_t Inline_ECC::get_avail_rd_ecc_buf_id() {
    uint32_t max_pri=0;
    this->avail_rd_ecc_buf_id = 0xffff;
    
    for (std::map<uint32_t,bool>::iterator it=index_recycle_rd_ecc_buf.begin();
            it!=index_recycle_rd_ecc_buf.end(); ++it) {
        if (it->second) {
            if(rd_ecc_buf[it->first].ecc_pri>=max_pri){
                max_pri=rd_ecc_buf[it->first].ecc_pri;
                this->avail_rd_ecc_buf_id = it->first;
            }
            //this->avail_rd_ecc_buf_id = it->first;
            //index_recycle_rd_ecc_buf.erase(it->first);
            //break;
        }
    }
    if(this->avail_rd_ecc_buf_id!=0xffff){
        index_recycle_rd_ecc_buf.erase(this->avail_rd_ecc_buf_id);;//9/11
    }
    return this->avail_rd_ecc_buf_id; // find the avail buf indx
}
//=================================================================================
uint32_t Inline_ECC::get_avail_wr_ecc_buf_id() {
    uint32_t max_pri=0;
    uint32_t max_full_pri=0;
    uint32_t full_pre_id = 0xffff;
    bool full_flag=false;
    bool idle_flag=false;
    this->avail_wr_ecc_buf_id = 0xffff;
    if(!ECC_MERGE_ENABLE){
        if (DEBUG_PDU) {
            PRINTN(setw(10)<<now()<<" -- CHECK_ECC_BUF_WR :: , avail_wr_ecc_buf_id="<<avail_wr_ecc_buf_id<<", WR_BUFF_SIZE="<<index_recycle_wr_ecc_buf.size()<<endl);
        }
        for (std::map<uint32_t,bool>::iterator it=index_recycle_wr_ecc_buf.begin();
            it!=index_recycle_wr_ecc_buf.end(); ++it) {
            if (it->second) {
                if(wr_ecc_buf[it->first].wr_ecc_pos==0xffff){//full wr_buff first 24/9/5
                    if(!full_flag){
                        full_flag=true;
                        max_pri = wr_ecc_buf[it->first].ecc_pri;
                        this->avail_wr_ecc_buf_id = it->first;
                        wdata_full_counter++;
                    }
                    if(wr_ecc_buf[it->first].ecc_pri>=max_pri){//update 24/9/5
                        max_pri=wr_ecc_buf[it->first].ecc_pri;
                        this->avail_wr_ecc_buf_id = it->first;
                    }
                }
                else if(!full_flag){
                    if (wr_ecc_buf[it->first].wr_ecc_pos == 0){    // select idle
                        if(!idle_flag){
                            idle_flag=true;
                            max_pri = wr_ecc_buf[it->first].ecc_pri;
                            this->avail_wr_ecc_buf_id = it->first;
                        }
                        if(wr_ecc_buf[it->first].ecc_pri>=max_pri){//update 24/9/5
                            max_pri=wr_ecc_buf[it->first].ecc_pri;
                            this->avail_wr_ecc_buf_id = it->first;
                        }
                    } else if (!idle_flag) {
                        if(wr_ecc_buf[it->first].ecc_pri>=max_pri){//update 24/9/5
                            max_pri=wr_ecc_buf[it->first].ecc_pri;
                            this->avail_wr_ecc_buf_id = it->first;
                        }
                    }
                }
            }
            else {
                if (DEBUG_PDU) {
                    PRINTN(setw(10)<<now()<<" -- NO AVAILABLE ECC_BUF "<<endl);
                }
            }
        }
    }
    else{//ecc write merge
        for (std::map<uint32_t,bool>::iterator it=index_recycle_wr_ecc_buf.begin();
            it!=index_recycle_wr_ecc_buf.end(); ++it){
                if(it->second){
                    if(wr_ecc_buf[it->first].wr_ecc_pos==0xffff){
                        for(std::map<uint32_t,bool>::iterator it2=index_recycle_wr_ecc_buf.begin();
                        it2!=index_recycle_wr_ecc_buf.end(); ++it2){
                            if(it2->second){
                                if((wr_ecc_buf[it->first].pdu_addr-wr_ecc_buf[it2->first].pdu_addr==0xffffffffffffffff
                                || wr_ecc_buf[it->first].pdu_addr-wr_ecc_buf[it2->first].pdu_addr==0x1)
                                && wr_ecc_buf[it2->first].wr_ecc_pos==0xffff){
                                    ecc_merge_flag=true;
                                    if(wr_ecc_buf[it->first].ecc_pri>wr_ecc_buf[it2->first].ecc_pri){
                                        merge_id=it2->first;
                                        this->avail_wr_ecc_buf_id = it->first;
                                    }
                                    else{
                                        merge_id=it->first;
                                        this->avail_wr_ecc_buf_id = it2->first;
                                    }
                                    merge_cnt++;
                                    return this->avail_wr_ecc_buf_id;
                                }
                            }
                        }
                        if(!full_flag && wr_ecc_buf[it->first].ecc_pri>max_full_pri){//update 24/9/5
                            max_full_pri=wr_ecc_buf[it->first].ecc_pri;
                            full_pre_id = it->first;
                        }
                        if(wr_ecc_buf[it->first].wr_merge_cnt<MERGE_DEPTH){
                            wr_ecc_buf[it->first].wr_merge_cnt++;
                            continue;
                        }
                        else if(!full_flag){
                            full_flag=true;
                            max_pri=wr_ecc_buf[it->first].ecc_pri;
                            this->avail_wr_ecc_buf_id = it->first;
                        }
                        if(wr_ecc_buf[it->first].ecc_pri>=max_pri){//update 24/9/5
                            max_pri=wr_ecc_buf[it->first].ecc_pri;
                            this->avail_wr_ecc_buf_id = it->first;
                        }
                        //continue;        
                    }
                    else if(!full_flag){
                        if(wr_ecc_buf[it->first].ecc_pri>=max_pri){//update 24/9/5
                            max_pri=wr_ecc_buf[it->first].ecc_pri;
                            this->avail_wr_ecc_buf_id = it->first;
                        }
                    }
                }
            }
        if(full_flag) wdata_full_counter++;
        if(!full_flag && this->avail_wr_ecc_buf_id == 0xffff){
            this->avail_wr_ecc_buf_id = full_pre_id;
            wdata_full_counter++;
        }
    }
    if(this->avail_wr_ecc_buf_id!=0xffff){
        index_recycle_wr_ecc_buf.erase(this->avail_wr_ecc_buf_id);;//10.18
        if (DEBUG_PDU) {
            PRINTN(setw(10)<<now()<<" -- DELETE_ECC_BUF_WR :: , avail_wr_ecc_buf_id="<<avail_wr_ecc_buf_id<<endl);
        }
    }
    return this->avail_wr_ecc_buf_id; // find the avail buf indx
}
//==============================================================================
void Inline_ECC::update_rd_ecc_buf() {
    for (uint32_t index=0; index<PDU_DEPTH; index++) {
        for (uint32_t i=0; i<rd_ecc_buf.at(index).task_list.size(); i++) {
            auto it = top->tasks_info.find(rd_ecc_buf.at(index).task_list.at(i));
            if (it != top->tasks_info.end()) {
                if (it->second.rd_finish) {
                    rd_ecc_buf.at(index).slt_cnt--;
                    top->tasks_info.erase(rd_ecc_buf.at(index).task_list.at(i));
                    rd_ecc_buf.at(index).task_list.erase(
                            rd_ecc_buf.at(index).task_list.begin()+i);
                }
            }
        }
        if (rd_ecc_buf.at(index).slt_cnt == 0) {
            index_recycle_rd_ecc_buf[rd_ecc_buf.at(index).buf_id] = true;
        }
    }
}
//==============================================================================
void Inline_ECC::update_wr_ecc_buf() {
    for (uint32_t index=0; index<PDU_DEPTH; index++) {
        for (uint32_t i=0; i<wr_ecc_buf.at(index).task_list.size(); i++) {
            auto it = top->tasks_info.find(wr_ecc_buf.at(index).task_list.at(i));
            if (it != top->tasks_info.end()) {
                if (it->second.wr_finish) {
                    wr_ecc_buf.at(index).slt_cnt--;
                    top->tasks_info.erase(wr_ecc_buf.at(index).task_list.at(i));
                    wr_ecc_buf.at(index).task_list.erase(wr_ecc_buf.at(
                                index).task_list.begin()+i);
                    //if (DEBUG_BUS) {
                    //    PRINTN(setw(10)<<now()<<" -- TASK_ERASE :: wr_ecc_buf.at(index).task_list.at(i)="<<wr_ecc_buf.at(index).task_list.at(i)<<endl);
                    //}
                }
            
            }
        }
        if (wr_ecc_buf.at(index).slt_cnt == 0) {
            index_recycle_wr_ecc_buf[wr_ecc_buf.at(index).buf_id] = true;
            if (DEBUG_PDU) {
                PRINTN(setw(10)<<now()<<" -- RELEASE_ECC_BUF_WR :: index="<<index<<", wr_ecc_buf.at(index).buf_id="<<wr_ecc_buf.at(index).buf_id<<endl);
            }
        }
    }
}
//==============================================================================
bool Inline_ECC::ecc_try_add_rd_trans(Transaction * trans, uint32_t rd_ecc_buf_id) {
    write_msg msg;
    msg.pt = DMC_PATH;
    msg.num_256bit = trans->burst_length + 1;
    
    bool ret = false;
    if (RMW_ENABLE) {
        ret = top->rmw->addTransaction(trans);
    } else {
        ret = top->addTransaction(trans);
    }
    if (ret) {
        top->tasks_info[trans->task].rd_finish = false;
        top->tasks_info[trans->task].wr_finish = false;
        //=========rd_data add success========//
        rd_ecc_buf.at(rd_ecc_buf_id).slt_cnt++;
        rd_ecc_buf.at(rd_ecc_buf_id).buf_ctrl_state = BUF_IDLE;
        rd_ecc_buf.at(rd_ecc_buf_id).task_list.push_back(trans->task);
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- IECC SCH :: READ_CMD task="<<trans->task<<" type="<<trans->transactionType<<" mask_write="<<trans->mask_wcmd<<" ecc_flag="<<trans->ecc_flag 
                    <<" qos="<<trans->qos<<" burst_length="<<trans->burst_length<<" channel="<<trans->channel<<" data_ready_cnt="<<trans->data_ready_cnt<<" address="<<hex<<trans->address
                    <<dec<<" rank="<<trans->rank<<" bank="<<trans->bankIndex<<" row="<<trans->row<<" col="<<trans->col<<" addr_col="<<trans->addr_col<<endl);
        }
    }
    return ret;
}
//==============================================================================
bool Inline_ECC::ecc_try_add_wr_trans(Transaction * trans, uint32_t wr_ecc_buf_id) {
    write_msg msg;
    unsigned int n_bit=0;
    msg.pt = DMC_PATH;
    msg.num_256bit = trans->burst_length + 1;
    bool ret = false;
    if (RMW_ENABLE) {
        ret = top->rmw->addTransaction(trans);
    } else {
        ret = top->addTransaction(trans);
    }
    if (ret) {
        top->tasks_info[trans->task].rd_finish = false;
        top->tasks_info[trans->task].wr_finish = false;
        //=========rd_data add success========//
        wr_ecc_buf.at(wr_ecc_buf_id).slt_cnt++;
        wr_ecc_buf.at(wr_ecc_buf_id).buf_ctrl_state = BUF_IDLE;
        wr_ecc_buf.at(wr_ecc_buf_id).task_list.push_back(trans->task);
        //for (uint32_t index=0; index<PDU_DEPTH; index++) {
        //    for (uint32_t i=0; i<wr_ecc_buf.at(index).task_list.size(); i++) {
        //        if (DEBUG_BUS) {
        //            PRINTN(setw(10)<<now()<<" -- WBUF PUSH :: task="<<wr_ecc_buf.at(index).task_list[i]<<", index="<<index<<endl);
        //        }
        //    }
        //}
        top->parentMemorySystem->write_map[trans->task] = msg;
        wr_ecc_buf.at(wr_ecc_buf_id).ecc_dirty = true;
        n_bit = trans->data_size/32;
        wr_ecc_buf.at(wr_ecc_buf_id).wr_ecc_pos |= (uint32_t(pow(2,n_bit))-1)<<(trans->col/16)%16;//update 24/9/4
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- IECC SCH :: WRITE_CMD task="<<trans->task<<", wr_ecc_buf_id="<<wr_ecc_buf_id<<" type="<<trans->transactionType<<" mask_write="<<trans->mask_wcmd<<" ecc_flag="<<trans->ecc_flag 
                    <<" qos="<<trans->qos<<" burst_length="<<trans->burst_length<<" channel="<<trans->channel<<" data_ready_cnt="<<trans->data_ready_cnt<<" address="<<hex<<trans->address
                    <<dec<<" rank="<<trans->rank<<" bank="<<trans->bankIndex<<" row="<<trans->row<<" col="<<trans->col<<" addr_col="<<trans->addr_col<<endl);
        }

    }
    return ret;
}
//==============================================================================
void Inline_ECC::get_pdu_addr(ECC_BUF_Entry *buf, Transaction *trans) {
    trans->row = (buf->pdu_addr >> PDU_ROW) & ((1ull << (PDU_BG - PDU_ROW)) - 1ull);
    trans->group = (buf->pdu_addr >> PDU_BG) & ((1ull << (PDU_BA - PDU_BG)) - 1ull);
    trans->bank = (buf->pdu_addr >> PDU_BA) & ((1ull << (PDU_RA - PDU_BA)) - 1ull);
    trans->rank = buf->pdu_addr >> PDU_RA;
//    trans->col = 0;
    trans->col = (buf->pdu_addr & ((1ull << (PDU_ROW)) - 1ull)) << (8ull + unsigned(IECC_BL32_MODE));
    trans->bankIndex = trans->rank * NUM_BANKS + trans->group * (NUM_BANKS / NUM_GROUPS) + trans->bank;
    trans->address = buf->pdu_addr;
}
//==============================================================================
bool Inline_ECC::try_add_ecc_rd(Transaction * trans, uint32_t rd_ecc_buf_id) {
    bool ret = false;
    Transaction *trans_rd_ecc = new Transaction(trans);
//    *trans_rd_ecc = Transaction(*trans);
    trans_rd_ecc->transactionType = DATA_READ;
    trans_rd_ecc->ecc_flag = true;
    //trans_rd_ecc->burst_length = unsigned(IECC_BL32_MODE) * 2 + 1;
    trans_rd_ecc->data_size = 32;
    trans_rd_ecc->burst_length = (trans_rd_ecc->data_size)/32 - 1;
    trans_rd_ecc->task = ecc_task;
    trans_rd_ecc->inject_time = now();
    top->parentMemorySystem->trans_init(trans_rd_ecc, now());
//    trans_rd_ecc->qos = IECC_PRI;
//    trans_rd_ecc->pri = IECC_PRI;
    trans_rd_ecc->qos = trans->qos;
    trans_rd_ecc->pri = trans->pri;
//    addressMapping(*trans_rd_ecc);
//    addr_exp(trans_rd_ecc);
//    if (RMW_ENABLE) {
    addr_ecc_map(trans_rd_ecc);
//    }
    if (RMW_ENABLE) {
        ret = top->rmw->addTransaction(trans_rd_ecc);
    } else {
        ret = top->addTransaction(trans_rd_ecc);
    }
    if (ret) {
        top->tasks_info[trans_rd_ecc->task].rd_finish = false;
        top->tasks_info[trans_rd_ecc->task].wr_finish = false;
        top->tasks_info[trans_rd_ecc->task].rd_ecc = true;
        rd_ecc_buf.at(rd_ecc_buf_id).task_list.push_back(trans_rd_ecc->task);
        rd_ecc_buf.at(rd_ecc_buf_id).slt_cnt++;
        rd_ecc_buf.at(rd_ecc_buf_id).buf_ctrl_state = WAIT_RDECC_BACK;
        ecc_task ++;
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- IECC SCH :: ECC_RD_CMD task="<<trans_rd_ecc->task<<" type="<<trans_rd_ecc->transactionType<<" mask_write="<<trans_rd_ecc->mask_wcmd<<" ecc_flag="<<trans_rd_ecc->ecc_flag 
                    <<" qos="<<trans_rd_ecc->qos<<" burst_length="<<trans_rd_ecc->burst_length<<" channel="<<trans_rd_ecc->channel<<" data_ready_cnt="<<trans_rd_ecc->data_ready_cnt<<" address="<<hex<<trans_rd_ecc->address
                    <<dec<<" rank="<<trans_rd_ecc->rank<<" bank="<<trans_rd_ecc->bankIndex<<" row="<<trans_rd_ecc->row<<" col="<<trans_rd_ecc->col<<" addr_col="<<trans_rd_ecc->addr_col<<endl);
        }
    } else {
        delete trans_rd_ecc;
    }
    return ret;
}
//==============================================================================
bool Inline_ECC::try_add_ecc_wr(Transaction * trans, uint32_t wr_ecc_buf_id) {
    bool ret = false;
    uint64_t req_time;
    Transaction *trans_wr_ecc = new Transaction(trans);
    write_msg msg_wr_ecc;
//    *trans_wr_ecc = Transaction(*trans);
    trans_wr_ecc->transactionType = DATA_WRITE;
    trans_wr_ecc->ecc_flag = true;
    if(ecc_merge_flag){
        trans_wr_ecc->data_size = 64;
    }
    else{
        trans_wr_ecc->data_size = 32;
    }
    //trans_wr_ecc->burst_length = unsigned(IECC_BL32_MODE) * 2 + 1;
    trans_wr_ecc->burst_length = (trans_wr_ecc->data_size)/32 - 1;
    trans_wr_ecc->task = ecc_task;
    trans_wr_ecc->inject_time = now();
    top->parentMemorySystem->trans_init(trans_wr_ecc, now());
    trans_wr_ecc->qos = IECC_PRI;
    trans_wr_ecc->pri = IECC_PRI;
//    addressMapping(*trans_wr_ecc);
    get_pdu_addr(&wr_ecc_buf[wr_ecc_buf_id], trans_wr_ecc);
//    if (RMW_ENABLE) {
    addr_ecc_map(trans_wr_ecc);
//    }
    msg_wr_ecc.pt = DMC_PATH;
    msg_wr_ecc.num_256bit = trans_wr_ecc->burst_length + 1;
    if(wr_ecc_buf[wr_ecc_buf_id].wr_ecc_pos!=0xffff){
        trans_wr_ecc->mask_wcmd = true;
    }
    if (RMW_ENABLE) {
        ret = top->rmw->addTransaction(trans_wr_ecc);
    } else {
        ret = top->addTransaction(trans_wr_ecc);
    }
    if (ret) {
        top->tasks_info[trans_wr_ecc->task].rd_finish = false;
        top->tasks_info[trans_wr_ecc->task].wr_finish = false;
        top->tasks_info[trans_wr_ecc->task].wr_ecc = true;
        wr_ecc_buf.at(wr_ecc_buf_id).task_list.push_back(trans_wr_ecc->task);
        wr_ecc_buf.at(wr_ecc_buf_id).slt_cnt++;
        wr_ecc_buf.at(wr_ecc_buf_id).buf_ctrl_state = WAIT_WRECC_BACK;
        top->parentMemorySystem->write_map[trans_wr_ecc->task] = msg_wr_ecc;
        ecc_task++;
        req_time=top->pre_req_data_time;
        for(uint32_t i=0; i<msg_wr_ecc.num_256bit;i++){
            if(DEBUG_BUS){
                PRINTN(setw(10)<<now()<<" -- IECC WR AddData"<<endl);
            }
            top->pre_req_data_time =req_time;
            top->parentMemorySystem->addData(nullptr, trans_wr_ecc->task, trans_wr_ecc->ecc_flag);
        }
        ecc_conflict_cnt = IECC_CONFLICT_CNT;
        ecc_merge_flag = false;
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- IECC SCH :: ECC_WRITE_CMD task="<<trans_wr_ecc->task<<" wr_ecc_buf_id="<<wr_ecc_buf_id<<" type="<<trans_wr_ecc->transactionType<<" mask_write="<<trans_wr_ecc->mask_wcmd<<" ecc_flag="<<trans_wr_ecc->ecc_flag 
                    <<" qos="<<trans_wr_ecc->qos<<" burst_length="<<trans_wr_ecc->burst_length<<" channel="<<trans_wr_ecc->channel<<" data_ready_cnt="<<trans_wr_ecc->data_ready_cnt<<" address="<<hex<<trans_wr_ecc->address
                    <<dec<<" rank="<<trans_wr_ecc->rank<<" bank="<<trans_wr_ecc->bankIndex<<" row="<<trans_wr_ecc->row<<" col="<<trans_wr_ecc->col<<" addr_col="<<trans_wr_ecc->addr_col<<endl);
        }
    } 
    else{
        delete trans_wr_ecc;
    }

    return ret;
}
//==============================================================================
bool Inline_ECC::proc_iecc(Transaction * trans, uint64_t inject_time) {
    addr_exp(trans);
    counter++;
    bool ret = false;
    update_rd_ecc_buf();
    update_wr_ecc_buf();

    switch (ecc_model_state) {
        case TRY_HIT_ECC_BUF: {
            uint32_t hit_result;
            // pdu_addr: [2:0]->col, [20:3]->row, [22:21]->bg, [25:23]->ba, [26]->rank
            // Redundant bits are added to increase the signal bit width.
            pdu_addr = trans->col >> (8ull + unsigned(IECC_BL32_MODE));
            pdu_addr |= uint64_t(trans->row) << PDU_ROW;
            pdu_addr |= uint64_t(trans->group) << PDU_BG;
            pdu_addr |= uint64_t(trans->bank) << PDU_BA;
            pdu_addr |= uint64_t(trans->rank) << PDU_RA;
            hit_result = hit_ecc_buf(pdu_addr);
            if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- TRY_HIT_ECC_BUF :: pdu_address=0x"<<hex<<pdu_addr<<endl);
            }
            if ((hit_result == 0) && (trans->transactionType == DATA_READ)) {
                ecc_model_state = GET_ECC_BUF;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- TRY_HIT_ECC_BUF :: address=0x"<<hex<<trans->address
                            <<", task="<<dec<<trans->task<<", nxt_state=GET_ECC_BUF"<<endl);
                }
            } else if ((hit_result == 1) && (trans->transactionType == DATA_READ)) {
                ecc_model_state = ECC_ADD_TRANS;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- TRY_HIT_ECC_BUF :: address=0x"<<hex<<trans->address
                            <<", task="<<dec<<trans->task<<", nxt_state=ECC_ADD_TRANS"<<endl);
                }
            } else if ((hit_result == 2) && (trans->transactionType == DATA_READ)) {
                ecc_model_state = ECC_RD_MISS_DIRTY;// new state ecc write back and data read back
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- TRY_HIT_ECC_BUF :: address=0x"<<hex<<trans->address
                            <<", task="<<dec<<trans->task<<", nxt_state=ECC_RD_MISS_DIRTY"<<endl);
                }
            } else if ((hit_result == 3) && (trans->transactionType == DATA_READ)) {
                ecc_model_state = ECC_RD_HIT_DIRTY;// new state ecc write back and data read back
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- TRY_HIT_ECC_BUF :: address=0x"<<hex<<trans->address
                            <<", task="<<dec<<trans->task<<", nxt_state=ECC_RD_HIT_DIRTY"<<endl);
                }
            } else if (((hit_result == 0) || (hit_result == 1)) && (trans->transactionType == DATA_WRITE)) {
                ecc_model_state = GET_ECC_BUF;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- TRY_HIT_ECC_BUF :: address=0x"<<hex<<trans->address
                            <<", task="<<dec<<trans->task<<", nxt_state=GET_ECC_BUF"<<endl);
                }
            } else if (((hit_result == 2) || (hit_result == 3)) && (trans->transactionType == DATA_WRITE)) {
                ecc_model_state = ECC_ADD_TRANS;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- TRY_HIT_ECC_BUF :: address=0x"<<hex<<trans->address
                            <<", task="<<dec<<trans->task<<", nxt_state=ECC_ADD_TRANS"<<endl);
                }
            } else {
                assert(0 && "IECC Access Error !");
            }
            break;
        }
        case GET_ECC_BUF: {
            update_rd_ecc_buf();
            update_wr_ecc_buf();
            if (trans->transactionType == DATA_READ) {
                avail_rd_ecc_buf_id = get_avail_rd_ecc_buf_id();
                if (avail_rd_ecc_buf_id >= PDU_DEPTH) {
                    ecc_model_state = GET_ECC_BUF;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- GET_ECC_BUF_RD :: address=0x"<<hex<<trans->address
                                <<", task="<<dec<<trans->task<<", nxt_state=GET_ECC_BUF"<<endl);
                    }
                    return false;
                }
            } else if (trans->transactionType == DATA_WRITE) {
                avail_wr_ecc_buf_id = get_avail_wr_ecc_buf_id();
                if (avail_wr_ecc_buf_id >= PDU_DEPTH) {
                    ecc_model_state = GET_ECC_BUF;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- GET_ECC_BUF_WR :: address=0x"<<hex<<trans->address
                                <<", task="<<dec<<trans->task<<", nxt_state=GET_ECC_BUF"<<", avail_wr_ecc_buf_id="<<avail_wr_ecc_buf_id<<endl);
                    }
                    return false;
                }
            } else {
                assert(0 && "Transaction Type Error!");
            }

            if (trans->transactionType == DATA_READ) {
                rd_ecc_buf.at(avail_rd_ecc_buf_id).rd_ecc = true;
                rd_ecc_buf.at(avail_rd_ecc_buf_id).pdu_addr = pdu_addr;
                rd_ecc_buf.at(avail_rd_ecc_buf_id).vld = true;
                replace_pri_update(this->avail_rd_ecc_buf_id,0);//update 24/9/5
            } else {//WRITE
//                wr_ecc_buf.at(avail_wr_ecc_buf_id).pdu_addr = pdu_addr;
                wr_ecc_buf.at(avail_wr_ecc_buf_id).vld = true;
                if(trans->col>=3*256){//update 24/9/9
                    wr_ecc_buf.at(avail_wr_ecc_buf_id).wr_ecc_pos = 0xf000;
                }
                else{
                    wr_ecc_buf.at(avail_wr_ecc_buf_id).wr_ecc_pos = 0;
                }
                wr_ecc_buf.at(avail_wr_ecc_buf_id).wr_merge_cnt = 0;//update 24/9/6
                replace_pri_update(this->avail_wr_ecc_buf_id,1);//update 24/9/5
                if(ECC_MERGE_ENABLE && ecc_merge_flag){
                    if((wr_ecc_buf.at(merge_id).pdu_addr & 0x3) == 0x3){
                        wr_ecc_buf.at(merge_id).wr_ecc_pos = 0xf000;//update 24/9/6
                    }
                    else{
                        wr_ecc_buf.at(merge_id).wr_ecc_pos = 0;//update 24/9/6
                    }
                    wr_ecc_buf.at(merge_id).wr_merge_cnt = 0;//update 24/9/6
                    wr_ecc_buf.at(merge_id).ecc_dirty = false;//update 24/9/6
                    replace_pri_update(merge_id,2);//update 24/9/6
                    merge_id=PDU_DEPTH;
                }
                if (wr_ecc_buf.at(avail_wr_ecc_buf_id).ecc_dirty){
                    wr_ecc_buf.at(avail_wr_ecc_buf_id).wr_ecc = true;
                    wr_ecc_buf.at(avail_wr_ecc_buf_id).ecc_dirty=false;
                }
            }
            ecc_model_state = ECC_ADD_TRANS;//update 24/10/21
            
            if (trans->transactionType == DATA_READ) {
                if (rd_ecc_buf.at(avail_rd_ecc_buf_id).rd_ecc) {
                        ecc_model_state = ADD_RD_ECC_TRANS;
                        rd_ecc_buf.at(avail_rd_ecc_buf_id).rd_ecc = false;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- GET_ECC_BUF :: address=0x"<<hex<<trans->address
                            <<", task="<<dec<<trans->task<<", nxt_state=ADD_RD_ECC_TRANS"<<endl);
                        }   
                    }
                else{
                    ecc_model_state = ECC_ADD_TRANS;
                    if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- GET_ECC_BUF :: address=0x"<<hex<<trans->address
                            <<", task="<<dec<<trans->task<<", nxt_state=ECC_ADD_TRANS"<<endl);
                        }  
                }
            }
            else{
                if (wr_ecc_buf.at(avail_wr_ecc_buf_id).wr_ecc) {
                        ecc_model_state = ADD_WR_ECC_TRANS;
                        wr_ecc_buf.at(avail_wr_ecc_buf_id).wr_ecc = false;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- GET_ECC_BUF :: address=0x"<<hex<<trans->address
                            <<", task="<<dec<<trans->task<<", nxt_state=ADD_WR_ECC_TRANS"<<endl);
                        } 
                    }
                else{
                    ecc_model_state = ECC_ADD_TRANS;
                    if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- GET_ECC_BUF :: address=0x"<<hex<<trans->address
                            <<", task="<<dec<<trans->task<<", nxt_state=ECC_ADD_TRANS"<<endl);
                        } 
                }
            }

            break;
        }
        case ECC_ADD_TRANS: {
            Transaction *trans_cmd = new Transaction(trans);
//            *trans_cmd = Transaction(*trans);
            if (trans_cmd->transactionType == DATA_READ) {
                if (ecc_try_add_rd_trans(trans_cmd, avail_rd_ecc_buf_id)) {
                    ecc_model_state = TRY_HIT_ECC_BUF;
                    /*
                    if (rd_ecc_buf.at(avail_rd_ecc_buf_id).rd_ecc) {
                        ecc_model_state = ADD_RD_ECC_TRANS;
                        rd_ecc_buf.at(avail_rd_ecc_buf_id).rd_ecc = false;
                    }
                    */
                    if (ecc_model_state == TRY_HIT_ECC_BUF) {
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- ECC_ADD_TRANS :: address=0x"<<hex<<trans->address
                                    <<", task="<<dec<<trans->task<<", nxt_state=TRY_HIT_ECC_BUF"<<endl);
                        }
                    } else {
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- ECC_ADD_TRANS :: address=0x"<<hex<<trans->address
                                    <<", task="<<dec<<trans->task<<", nxt_state=ADD_RD_ECC_TRANS"<<endl);
                        }
                    }

                    return (ecc_model_state == TRY_HIT_ECC_BUF);
                }
            } else if (trans_cmd->transactionType == DATA_WRITE) {
                if (ecc_try_add_wr_trans(trans_cmd, avail_wr_ecc_buf_id)) {
                    ecc_model_state = TRY_HIT_ECC_BUF;
                    wr_ecc_buf.at(avail_wr_ecc_buf_id).pdu_addr = pdu_addr;//update 24/10/21
                    /*
                    if (wr_ecc_buf.at(avail_wr_ecc_buf_id).wr_ecc) {
                        ecc_model_state = ADD_WR_ECC_TRANS;
                        wr_ecc_buf.at(avail_wr_ecc_buf_id).wr_ecc = false;
                    }
                    */                  
                    if (ecc_model_state == TRY_HIT_ECC_BUF) {
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- ECC_ADD_TRANS :: address=0x"<<hex<<trans->address
                                    <<", task="<<dec<<trans->task<<", nxt_state=TRY_HIT_ECC_BUF"<<endl);
                        }
                    } else {
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- ECC_ADD_TRANS :: address=0x"<<hex<<trans->address
                                    <<", task="<<dec<<trans->task<<", nxt_state=ADD_WR_ECC_TRANS"<<endl);
                        }
                    }
                    return (ecc_model_state == TRY_HIT_ECC_BUF);
                }
            }
            break;
        }
        case ADD_RD_ECC_TRANS: {
            if (try_add_ecc_rd(trans, avail_rd_ecc_buf_id)) {
                //ecc_model_state = TRY_HIT_ECC_BUF;
                ecc_model_state = ECC_ADD_TRANS;//update 24/10/21
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- ADD_RD_ECC_TRANS :: address=0x"<<hex<<trans->address
                            <<", task="<<dec<<trans->task<<", nxt_state=ECC_ADD_TRANS"<<endl);
                }
            } else {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- ADD_RD_ECC_TRANS :: address=0x"<<hex<<trans->address
                            <<", task="<<dec<<trans->task<<", nxt_state=ADD_RD_ECC_TRANS"<<endl);
                }
            }
            return (ecc_model_state == TRY_HIT_ECC_BUF);
            break;
        }
        case ADD_WR_ECC_TRANS: {
            if (try_add_ecc_wr(trans, avail_wr_ecc_buf_id)) {
                ecc_model_state = WAIT_ECC_DATA_FINISH;
                ecc_pre_state = ADD_WR_ECC_TRANS;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- ADD_WR_ECC_TRANS :: address=0x"<<hex<<trans->address
                            <<", task="<<dec<<trans->task<<", nxt_state=WAIT_ECC_DATA_FINISH"<<endl);
                }
                wr_ecc_buf.at(avail_wr_ecc_buf_id).pdu_addr = pdu_addr;//update 24/10/21
//                wr_ecc_buf[avail_wr_ecc_buf_id].vld = false;
            } else {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- ADD_WR_ECC_TRANS :: address=0x"<<hex<<trans->address
                            <<", task="<<dec<<trans->task<<", nxt_state=ADD_WR_ECC_TRANS"<<endl);
                }
            }
            return (ecc_model_state == TRY_HIT_ECC_BUF);
            break;
        }
        case WAIT_ECC_DATA_FINISH: {
            if (ecc_delay > IECC_CONFLICT_CNT) {
                //ecc_model_state = TRY_HIT_ECC_BUF;
                if(ecc_pre_state == ADD_WR_ECC_TRANS){
                    ecc_model_state = ECC_ADD_TRANS;
                }
                else {
                    ecc_model_state = TRY_HIT_ECC_BUF;//update 24/10/21
                }
                ecc_delay = 0;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- WAIT_ECC_DATA_FINISH :: address=0x"<<hex<<trans->address
                            <<", task="<<dec<<trans->task<<", nxt_state=ECC_ADD_TRANS"<<endl);
                }
            } else {
                ecc_model_state = WAIT_ECC_DATA_FINISH;
                ecc_delay++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- WAIT_ECC_DATA_FINISH :: address=0x"<<hex<<trans->address
                            <<", task="<<dec<<trans->task<<", nxt_state=WAIT_ECC_DATA_FINISH"<<endl);
                }
            }
            return (ecc_model_state == TRY_HIT_ECC_BUF);
            break;
        }
        case ECC_RD_HIT_DIRTY: {
            bool ret0 = false;
            bool ret1 = false;
            bool ret2 = false;
            if (iecc_add_cnt == 0) {
                ret1 = try_add_ecc_wr(trans, avail_wr_ecc_buf_id);
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- ECC_RD_HIT_DIRTY :: address=0x"<<hex<<trans->address
                            <<", task="<<dec<<trans->task<<", iecc_add_cnt="<<iecc_add_cnt
                            <<", nxt_state=ECC_RD_HIT_DIRTY"<<endl);
                }
                if (ret1) {
                    iecc_add_cnt = 1;
                    wr_ecc_buf.at(avail_wr_ecc_buf_id).ecc_dirty=false;
                    wr_ecc_buf[avail_wr_ecc_buf_id].vld = false;
                }
            } else if (iecc_add_cnt == 1) {
                ret2 = try_add_ecc_rd(trans, avail_rd_ecc_buf_id);
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- ECC_RD_HIT_DIRTY :: address=0x"<<hex<<trans->address
                            <<", task="<<dec<<trans->task<<", iecc_add_cnt="<<iecc_add_cnt
                            <<", nxt_state=ECC_RD_HIT_DIRTY"<<endl);
                }
                if (ret2) {
                    iecc_add_cnt = 2;
                    rd_ecc_buf[avail_rd_ecc_buf_id].pdu_addr = pdu_addr;
                    rd_ecc_buf[avail_rd_ecc_buf_id].vld = true;
                    replace_pri_update(this->avail_rd_ecc_buf_id,0);
                }
            } else if (iecc_add_cnt == 2) {
                Transaction *trans_cmd = new Transaction(trans);
//                *trans_cmd = Transaction(*trans);
                ret0 = ecc_try_add_rd_trans(trans_cmd, avail_rd_ecc_buf_id);
                if (ret0) {
                    iecc_add_cnt = 0;
                    ecc_model_state = WAIT_ECC_DATA_FINISH;
                    ecc_pre_state = ECC_RD_HIT_DIRTY;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- ECC_RD_HIT_DIRTY :: address=0x"<<hex<<trans->address
                                <<", task="<<dec<<trans->task<<", nxt_state=WAIT_ECC_DATA_FINISH"<<endl);
                    }
                } else {
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- ECC_RD_HIT_DIRTY :: address=0x"<<hex<<trans->address
                                <<", task="<<dec<<trans->task<<", nxt_state=ECC_RD_HIT_DIRTY"<<endl);
                    }
                }
            } else {
                assert("iecc add cnt has been wrong" && false);
            }

            return (ecc_model_state == TRY_HIT_ECC_BUF);
            break;
        }
        case ECC_RD_MISS_DIRTY: {//need fixing
            bool ret0 = false;
            bool ret1 = false;
            bool ret2 = false;
            update_rd_ecc_buf();
            avail_rd_ecc_buf_id = get_avail_rd_ecc_buf_id();
            if (avail_rd_ecc_buf_id >= PDU_DEPTH) {
                ecc_model_state = GET_ECC_BUF;
                return false;
            }

            if (iecc_add_cnt == 0) {
                ret1 = try_add_ecc_wr(trans, avail_wr_ecc_buf_id);
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- ECC_RD_MISS_DIRTY :: address=0x"<<hex<<trans->address
                            <<", task="<<dec<<trans->task<<", iecc_add_cnt="<<iecc_add_cnt
                            <<", nxt_state=ECC_RD_MISS_DIRTY"<<endl);
                }
                if (ret1) {
                    iecc_add_cnt = 1;
                    wr_ecc_buf.at(avail_wr_ecc_buf_id).ecc_dirty=false;
                    wr_ecc_buf[avail_wr_ecc_buf_id].vld = false;
                }
            } else if (iecc_add_cnt == 1) {
                ret2 = try_add_ecc_rd(trans, avail_rd_ecc_buf_id);
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- ECC_RD_MISS_DIRTY :: address=0x"<<hex<<trans->address
                            <<", task="<<dec<<trans->task<<", iecc_add_cnt="<<iecc_add_cnt
                            <<", nxt_state=ECC_RD_MISS_DIRTY"<<endl);
                }
                if (ret2) {
                    iecc_add_cnt = 2;
                }
            } else if (iecc_add_cnt == 2) {
                Transaction *trans_cmd = new Transaction(trans);
//                *trans_cmd = Transaction(*trans);
                ret0 = ecc_try_add_rd_trans(trans_cmd, avail_rd_ecc_buf_id);
                if (ret0) {
                    iecc_add_cnt = 0;
                    ecc_model_state = WAIT_ECC_DATA_FINISH;
                    ecc_pre_state = ECC_RD_MISS_DIRTY;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- ECC_RD_MISS_DIRTY :: address=0x"<<hex<<trans->address
                                <<", task="<<dec<<trans->task<<", nxt_state=WAIT_ECC_DATA_FINISH"<<endl);
                    }
                } else {
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- ECC_RD_MISS_DIRTY :: address=0x"<<hex<<trans->address
                                <<", task="<<dec<<trans->task<<", nxt_state=ECC_RD_HIT_DIRTY"<<endl);
                    }
                }
            } else {
                assert("iecc add cnt has been wrong" && false);
            }
            return (ecc_model_state == TRY_HIT_ECC_BUF);
            break;
        }
        default:
            assert(ret && "No Such State");
            return false;
    }
    return false;
}

bool Inline_ECC::addTransaction(Transaction * trans) {
    return (proc_iecc(trans, now())); // Delete

    if (WRITE_BUFFER_ENABLE) {
        if (top->wb->full()) return false;
    } else {
        if (top->full()) return false;
    }
    if (full()) return false;
    if (backpress_cnt > 0) return false;

    iecc_cmd_cnt ++;
    addr_exp(trans);
    trans->async_delay_time = now() + 3;
    IeccCmdQueue.push_back(trans);
    return true;
}

bool Inline_ECC::addData(uint32_t *data ,uint64_t task) {
    if (wdata_full()) return false;

    iecc_wdata_cnt ++;
    wdata wd;
    wd.task = task;
    wd.wdata_delay = now() + 3;
    IeccWdata.push_back(wd);
    return true;
}

void Inline_ECC::update() {
    show_buf_state();
    return; // Delete

    iecc_alct();

    send_command();
    send_wdata();
}

// normal command and ecc command use the same column
void Inline_ECC::addr_exp(Transaction * trans) {
    //unsigned capacity_ratio = 960;
    unsigned capacity_ratio = IECC_CAP_RATIO;
    unsigned col_per_row = 1024;
    unsigned bank_per_rank = NUM_BANKS;
    unsigned row = trans->row;
    unsigned bank = trans->bankIndex;
    unsigned col = trans->col;
    trans->col_ini = col;
    trans->row = ((col_per_row * (row * bank_per_rank + bank) + col) / capacity_ratio) / bank_per_rank;
    trans->bankIndex = ((col_per_row * (row * bank_per_rank + bank) + col) / capacity_ratio) % bank_per_rank;
    unsigned banks_per_bg = NUM_BANKS / NUM_GROUPS;
    trans->group = trans->bankIndex / banks_per_bg;
    trans->bank = trans->bankIndex % banks_per_bg;
    trans->col = (col_per_row * (row * bank_per_rank + bank) + col) % capacity_ratio;
    trans->bankIndex = trans->bankIndex + trans->rank * NUM_BANKS;
    if (IS_LP6) {
        trans->addr_col = trans->col * 16 / 8;
        trans->addr_col_ini = trans->col_ini * 16 / 8;
    } else {
        trans->addr_col = trans->col * JEDEC_DATA_BUS_BITS / 8;
        trans->addr_col_ini = trans->col_ini * JEDEC_DATA_BUS_BITS / 8;
    }
    if (DEBUG_BUS) {
        PRINTN(setw(10)<<now()<<" -- ADD_EXP :: task="<<trans->task<<", rank="<<trans->rank<<", row="<<row<<", bank="
                <<bank<<", col="<<col<<", addr_col="<<trans->addr_col_ini<<", exp_row="<<trans->row<<", exp_bank="<<trans->bankIndex
                <<", exp_col="<<trans->col<<", exp_addr_col="<<trans->addr_col<<endl);
    }
}


// transform ecc address to real physical address
void Inline_ECC::addr_ecc_map(Transaction * trans) {
    //unsigned capacity_ratio = 960;
    unsigned capacity_ratio = IECC_CAP_RATIO;
//    unsigned col_per_row = 1024;
    unsigned byte_per_col = 2;
    unsigned ecc_data_size = 32;
    unsigned col = trans->col;
    trans->col_ini = col;
    trans->col = capacity_ratio + (col/256)* ecc_data_size/byte_per_col;
    if (IS_LP6) {
        trans->addr_col = trans->col * 16 / 8;
        trans->addr_col_ini = trans->col_ini * 16 / 8;
    } else {
        trans->addr_col = trans->col * JEDEC_DATA_BUS_BITS / 8;
        trans->addr_col_ini = trans->col_ini * JEDEC_DATA_BUS_BITS / 8;
    }
    if (DEBUG_BUS) {
        PRINTN(setw(10)<<now()<<" -- ADD_ECC_MAP :: task="<<trans->task<<", rank="<<trans->rank<<", row="<<trans->row<<", bank="
                <<trans->bankIndex<<", col="<<col<<", addr_col="<<trans->addr_col_ini<<", exp_row="<<trans->row<<", exp_bank="<<trans->bankIndex
                <<", exp_col="<<trans->col<<", exp_addr_col="<<trans->addr_col<<endl);
    }
}

void Inline_ECC::iecc_alct() {
    if (IeccCmdQueue.empty()) return;
    for (auto iecc : IeccCmdQueue) {
        if (iecc->ecc_flag) continue;
        if (now() < iecc->async_delay_time) continue;


        break;
    }
}

void Inline_ECC::send_command() {
    bool ret = false;
    if (!IeccCmdQueue.empty()) {
        if (now() >= IeccCmdQueue[0]->async_delay_time) {
            if (WRITE_BUFFER_ENABLE) {
                ret = top->wb->addTransaction(IeccCmdQueue[0]);
            } else {
                ret = top->addTransaction(IeccCmdQueue[0]);
            }
        }
        if (ret) {
            iecc_cmd_cnt --;
            IeccCmdQueue.erase(IeccCmdQueue.begin());
        }
    }
}

void Inline_ECC::send_wdata() {
    if (!IeccWdata.empty()) {
        if (now() >= IeccWdata[0].wdata_delay) {
            if (WRITE_BUFFER_ENABLE) {
                top->wb->addData(NULL, IeccWdata[0].task);
            } else {
                top->receiveFromCPU(NULL, IeccWdata[0].task);
            }
        }
        iecc_wdata_cnt --;
        IeccWdata.erase(IeccWdata.begin());
    }
}

std::string Inline_ECC::trans_type_opcode_iecc(Transaction * trans) {
    switch (trans->transactionType) {
        case DATA_READ       : {return "READ"; break;}
        case DATA_WRITE      : {return "WRITE"; break;}
        default              : break;
    }
    return "Unkown opcode";
}

void Inline_ECC::replace_pri_update(uint32_t id, uint32_t cmd_type){
    uint32_t tar_pri;
    switch(strategy){
        case ID_ORDER:{
            return;
        }
        case LRU:{
            if(cmd_type==0){
                tar_pri=rd_ecc_buf[id].ecc_pri;
                for(auto &bufs : rd_ecc_buf){
                    if(bufs.buf_id==id){
                        bufs.ecc_pri=1;
                    }
                    else{
                        if(bufs.ecc_pri<tar_pri){
                            bufs.ecc_pri++;
                        }
                    }
                }
            }
            else if(cmd_type==1){
                tar_pri=wr_ecc_buf[id].ecc_pri;
                for(auto &bufs : wr_ecc_buf){
                    if(bufs.buf_id==id){
                        bufs.ecc_pri=1;
                    }
                    else{
                        if(bufs.ecc_pri<tar_pri){
                            bufs.ecc_pri++;
                        }
                    }
                }
            }
            else if(cmd_type==2){
                tar_pri=wr_ecc_buf[id].ecc_pri;
                for(auto &bufs : wr_ecc_buf){
                    if(bufs.buf_id==id){
                        bufs.ecc_pri=PDU_DEPTH;
                    }
                    else{
                        if(bufs.ecc_pri>tar_pri){
                            bufs.ecc_pri--;
                        }
                    }
                }
            }
            return;
        }
    }
}

void Inline_ECC::show_buf_state(){
    if(!DEBUG_PDU) return;
    if(!DEBUG_BUS) return;
 //   float ratio;
    
    //PRINTN("--------------------------------------------------------------------------------------------------"<<endl);
    //for (auto &bufs : rd_ecc_bufwr_ecc_buf.at(index).task_list.at(i)) {
    //    PRINTN("BUF time: "<<now()<<" | buffer="<<bufs.buf_id<<" | pdu_addr=0x"<<hex<<bufs.pdu_addr<<dec
    //    <<" | rd_ecc_pos="<<bufs.rd_ecc_pos<<" | rd_ecc="<<bufs.rd_ecc
    //    <<" | wr_ecc="<<bufs.wr_ecc <<" | slt_cnt="<<bufs.slt_cnt<<" | vld="<<bufs.vld<<" | pri="<<bufs.ecc_pri<<endl);
    //}
    
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl);
    for (auto &bufs : wr_ecc_buf) {
        PRINTN("BUF time: "<<now()<<" | buffer="<<bufs.buf_id<<" | pdu_addr=0x"<<hex<<bufs.pdu_addr<<dec
        <<" | wr_ecc_pos="<< hex << bufs.wr_ecc_pos<<dec<<" | slt_cnt="<<bufs.slt_cnt<<" | vld="<<bufs.vld<<endl);
    }
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl);
    for (std::map<uint32_t,bool>::iterator it=index_recycle_wr_ecc_buf.begin(); it!=index_recycle_wr_ecc_buf.end(); ++it) {
        PRINTN("RECYCLE time: "<<now()<<" | buffer="<<it->second<<" | valid="<<it->first<<endl);
    }
    PRINTN("ECC_BUF_SIZE: "<< index_recycle_wr_ecc_buf.size() <<endl);
    PRINTN("ECC_MERGE_ENABLE: "<< ECC_MERGE_ENABLE <<endl);
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl);
    for (uint32_t index=0; index<PDU_DEPTH; index++) {
        for (uint32_t i=0; i<wr_ecc_buf.at(index).task_list.size(); i++) {
            auto it = top->tasks_info.find(wr_ecc_buf.at(index).task_list.at(i));
            if (DEBUG_PDU) {
                PRINTN(setw(10)<<now()<<" -- IECC UPDATE :: WR_ECC task="<<wr_ecc_buf.at(index).task_list.at(i)<<", wr_finish="<<it->second.wr_finish<<", index="<<index<<endl);
            }
        }
    }
    //for (uint32_t index=0; index<PDU_DEPTH; index++) {
    //    for (uint32_t i=0; i<wr_ecc_buf.at(index).task_list.size(); i++) {
    //        if (DEBUG_BUS) {
    //            PRINTN(setw(10)<<now()<<" -- WBUF ERASE :: task="<<wr_ecc_buf.at(index).task_list[i]<<", index="<<index<<endl);
    //        }
    //    }
    //}
    //for(size_t i = 0;i<top->tasks_info.size();i++){
    //    if(DEBUG_BUS){
    //       PRINTN(setw(10)<<now()<<dec<<" -- TASK_IFNOS :: index="<<i<<", wr_finish="<<top->tasks_info[i].wr_finish<<endl); 
    //    }
    //}
    /*
    for (std::map<uint32_t,bool>::iterator it=index_recycle_rd_ecc_buf.begin();
            it!=index_recycle_rd_ecc_buf.end(); ++it) {
        PRINTN("BUF time: "<<now()<<" READ index="<<it->first<<" | state="<<it->second<<endl)
    }
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl)
    for (std::map<uint32_t,bool>::iterator it=index_recycle_wr_ecc_buf.begin();
            it!=index_recycle_wr_ecc_buf.end(); ++it) {
        PRINTN("BUF time: "<<now()<<" WRITE index="<<it->first<<" | state="<<it->second<<endl)
    }
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl)
    */
}


}