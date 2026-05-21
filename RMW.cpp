#include "Rmw.h"
//#include "LPMemorySystemTop.h"
#include "MemorySystem.h"
#include "MemoryController.h"
#include "AddressMapping.h"
#include <assert.h>
#include <iomanip>
#include <algorithm>
using namespace std;

namespace LPDDRSim {
//#define PROTECT_SUB(a) a = (a > 0) ? (a - 1) : 0;

//Rmw::Rmw(LPMemorySystemTop *_top, unsigned id, ostream &DDRSim_log_, string LogPath) :
Rmw::Rmw(MemoryController *_top, unsigned id, ostream &DDRSim_log_) :
        top(_top), 
        channel(id), 
        DDRSim_log(DDRSim_log_) { 
//        log_path(LogPath) {
    channel_ohot = 1ull << channel;
    rmw_cmd_cnt = 0;
//    bytes_per_col = JEDEC_DATA_BUS_BITS / 8;
//    push_cnt = 0;
    log_path = top->log_path;
    WdataToSend.clear();
    WdataChannel.clear();
    MaskWriteDataBeats.clear();
    MaskWriteSendBeats.clear();
    write_data_routes.clear();
    pending_write_merge_resps.clear();
    pre_req_time = 0xFFFFFFFFFFFFFFFF;
    pre_req_data_time = 0xFFFFFFFFFFFFFFFF;
    pre_cresp_time = 0xFFFFFFFFFFFFFFFF;
    start_cycle = 0;
    end_cycle = 0;
    rcmd_cnt = 0;
    wcmd_cnt = 0;
    totalReads = 0; 
    totalBypassReads = 0; 
    totalWrites = 0; 
    totalBypassWrites = 0; 
    totalFullWrites = 0; 
    totalMaskWrites = 0; 
    totalTransactions = 0; 
    totalWriteMergeInput = 0;
    totalWriteMergePair = 0;
    totalWriteMergeUnpairedToRmw = 0;
    totalWriteMergeUnpairedDirect = 0;
    totalWriteMergeBufferFull = 0;
//    pre_reads = 0; 
//    pre_bypass_reads = 0; 
//    pre_bypass_writes = 0; 
//    pre_bypass_writes = 0; 
//    pre_full_writes = 0; 
//    pre_mask_writes = 0; 
//    pre_totals = 0; 
    for (uint32_t index = 0; index < RMW_QUE_DEPTH+1; index++) {
        rmw_que_cnt.push_back(0);
    }
//    RmwInitOutputFiles();

    }

//void WriteBuff::rcmd_push_wcmd(Transaction * t) {
//    for (auto &w : Wbuff) {
//        if (t->bankIndex == w->bankIndex && t->row == w->row && t->col == w->col) {
//            push_cnt ++;
//            if (GrpMode.grp_mode6) { // priority is 1
//                w->pri = 1;
//            } else { // high priority
//                w->timeout = true;
//                wr_timeout_cnt ++;
//            }
//            if (DEBUG_BUS) {
//                PRINTN(setw(10)<<now()<<" -- ADD_PUSH :: task="<<t->task<<hex<<" address="<<t->address<<dec
//                        <<" rank="<<t->rank<<" bank="<<t->bankIndex<<" row="<<t->row<<" w_task="<<w->task<<endl);
//            }
//        }
//    }
//}

//void Rmw::RmwInitOutputFiles() {
//    if ((DEBUG_BUS || DEBUG_STATE || DEBUG_RMW_STATE) && (channel_ohot == (channel_ohot & PRINT_CH_OHOT))) {
////        dmc_log = log_path + "/lpddr_sim" + std::to_string(channel) + ".log";
//        rmw_log = log_path + "/lpddr_sim" + "_rmw" + std::to_string(channel) + ".log";
//        DDRSim_log.open(rmw_log.c_str(),ios_base::out | ios_base::trunc);
//        if (!DDRSim_log) {
//            ERROR("Cannot open "<<rmw_log);
//            assert(0);
//        }
//    }
//
//    if (STATE_LOG) {
//        string st_log = log_path + "/lpddr_state" + "_rmw" + std::to_string(channel) + ".log";
//        state_log.open(st_log.c_str(),ios_base::out | ios_base::trunc);
//        if (!state_log) {
//             ERROR("Cannot open "<<st_log);
//             assert(0);
//        }
//    }
//
//}



void Rmw::cmd_set_conflict(Transaction * t) {
    conf_state *c = new conf_state;
    c->task = t->task;

    for (auto &cmd : RmwQue) {
        if (cmd->task == t->task) continue;
        if (t->transactionType == DATA_READ && cmd->transactionType == DATA_READ) continue;
        bool col_conflict = address_conf(t, cmd);
        if (t->channel==cmd->channel && t->bankIndex == cmd->bankIndex && t->row == cmd->row && col_conflict) {
            c->ad_conf_cnt ++;
        }
    }
    RmwConfCnt.push_back(c);
}


void Rmw::cmd_release_conflict(Transaction *trans) {
    unsigned size = RmwQue.size();
    unsigned trans_rmwque_index = 0;
    
    for (unsigned i = 0; i < size; i ++) {
        if (trans->task == RmwQue[i]->task) {
            trans_rmwque_index = i;
            break;
        }
    }

    //index chk
    if (trans_rmwque_index >= size) {
        ERROR(setw(10)<<now()<<" -- Impossible index in Rmw Queue, index="<<trans_rmwque_index<<", task="<<trans->task);
        assert(0);
    }

//    for (unsigned i = 0; i < size; i ++) {
    for (unsigned i = trans_rmwque_index; i < size; i ++) {
        if (trans->task == RmwQue[i]->task) continue;
        if (trans->transactionType==DATA_READ && RmwQue[i]->transactionType==DATA_READ) continue;
        bool col_conflict = address_conf(trans, RmwQue[i]);
        if (trans->channel == RmwQue[i]->channel && trans->bankIndex == RmwQue[i]->bankIndex && trans->row == RmwQue[i]->row && col_conflict) {
            if (RmwConfCnt[i]->ad_conf_cnt > 0) {
                RmwConfCnt[i]->ad_conf_cnt--;
            }
        }
    }

}

bool Rmw::address_conf(Transaction *t, Transaction *cmd) {
        
        // data size cal
        unsigned t_data_size = (t->burst_length + 1) * DMC_DATA_BUS_BITS / 8;
        unsigned cmd_data_size = (cmd->burst_length + 1) * DMC_DATA_BUS_BITS / 8;
        // wrap or inc
//        bool t_wrap = ((t->address % t_data_size) == 0) ? false : true;
//        bool cmd_wrap = ((cmd->address % cmd_data_size) == 0) ? false : true;
        bool t_wrap = t->wrap_cmd;
        bool cmd_wrap = cmd->wrap_cmd;
//        if (t_wrap) {
//            DEBUG(now()<<"wrap cmd, task="<<t->task);
//        }
//        if (cmd_wrap) {
//            DEBUG(now()<<"wrap cmd in RMWQUE, task="<<cmd->task);
//        }
        // addr of trans && cmd
        unsigned t_start_addr_col = t_wrap ? ((t->addr_col / t_data_size) * t_data_size) : t->addr_col;
        unsigned t_end_addr_col = t_wrap ? ((t->addr_col / t_data_size + 1) * t_data_size) : t->addr_col + t_data_size;
        unsigned cmd_start_addr_col = cmd_wrap ? ((cmd->addr_col / cmd_data_size) * cmd_data_size) : cmd->addr_col;
        unsigned cmd_end_addr_col = cmd_wrap ? ((cmd->addr_col / cmd_data_size + 1) * cmd_data_size) : cmd->addr_col + cmd_data_size;
        // addr used for address conflict
        unsigned t_addr_left = 0;
        unsigned t_addr_right = 0;
        unsigned cmd_addr_left = 0;
        unsigned cmd_addr_right = 0;
        bool     ret = false;
         
//        if ((t_start_addr_col % RMW_CONF_SIZE == 0) && (t_end_addr_col % RMW_CONF_SIZE == 0)){       // address aligned with RMW_CONF_SIZE
        if (t_end_addr_col % RMW_CONF_SIZE == 0){       // address aligned with RMW_CONF_SIZE
            t_addr_left = t_start_addr_col / RMW_CONF_SIZE;
            t_addr_right = t_end_addr_col / RMW_CONF_SIZE;
        } else {       // address not aligned with RMW_CONF_SIZE 
            t_addr_left = t_start_addr_col / RMW_CONF_SIZE;
            t_addr_right = (t_end_addr_col / RMW_CONF_SIZE) + 1;
        }
        
//        if ((cmd_start_addr_col % RMW_CONF_SIZE == 0) && (cmd_end_addr_col % RMW_CONF_SIZE == 0)){       // address aligned with RMW_CONF_SIZE
        if (cmd_end_addr_col % RMW_CONF_SIZE == 0){       // address aligned with RMW_CONF_SIZE
            cmd_addr_left = cmd_start_addr_col / RMW_CONF_SIZE;
            cmd_addr_right = cmd_end_addr_col / RMW_CONF_SIZE;
        } else {       // address not aligned with RMW_CONF_SIZE 
            cmd_addr_left = cmd_start_addr_col / RMW_CONF_SIZE;
            cmd_addr_right = (cmd_end_addr_col / RMW_CONF_SIZE) + 1;
        }
        
        if ((t_addr_left >= cmd_addr_left && t_addr_left < cmd_addr_right) ||
                (t_addr_right > cmd_addr_left && t_addr_right <= cmd_addr_right)) {
//            DEBUG(now()<<" address conflict, transaction task="<<t->task<<" t_addr_left="<<t_addr_left<<" t_addr_right"<<t_addr_right
//                    <<" cmd in RMWQUE, task="<<cmd->task<<" cmd_addr_left"<<cmd_addr_left<<" cmd_addr_right"<<cmd_addr_right);
            ret = true;
        }
        return ret;

    
}

//void WriteBuff::trans_state_clr(Transaction * trans) {
//    trans->timeout = false;
//    trans->has_active = false;
//}


//void Rmw::addData(uint32_t *data, uint32_t channel, uint64_t task) {
bool Rmw::is_write_merge_candidate(const Transaction *trans) const {
    if (trans == NULL) return false;
    if (!WCMD_MERGE_EN) return false;
    if (trans->transactionType != DATA_WRITE) return false;
    if (trans->mask_wcmd) return false;
    return ((trans->burst_length + 1) * DMC_DATA_BUS_BITS / 8) == 128;
}

bool Rmw::can_merge_write_pair(const Transaction *first, const Transaction *second) const {
    if (!is_write_merge_candidate(first) || !is_write_merge_candidate(second)) return false;
    if (first->channel != second->channel) return false;

    uint64_t lower = std::min(first->address, second->address);
    uint64_t upper = std::max(first->address, second->address);
    return (upper - lower) == 128 && (lower / 256) == (upper / 256);
}

Transaction *Rmw::build_merged_transaction(const BufferedWriteEntry &entry, uint64_t merged_task) const {
    if (!entry.has_second) return NULL;

    const Transaction *lower = (entry.first_trans->address <= entry.second_trans->address)
            ? entry.first_trans : entry.second_trans;

    Transaction *merged = new Transaction(*lower);
    merged->task = merged_task;
    merged->mask_wcmd = false;
    merged->burst_length = (entry.first_trans->burst_length + 1)
            + (entry.second_trans->burst_length + 1) - 1;
    merged->data_size = (merged->burst_length + 1) * DMC_DATA_BUS_BITS / 8;
    merged->reqEnterDmcBufTime = std::min(entry.first_trans->reqEnterDmcBufTime,
            entry.second_trans->reqEnterDmcBufTime);
    merged->qos = std::max(entry.first_trans->qos, entry.second_trans->qos);
    merged->address = lower->address;
    merged->timeAdded = 0;
    merged->inject_time = 0;
    merged->issue_size = 0;
    merged->data_ready_cnt = 0;
    merged->nextCmd = INVALID;
    merged->has_active = false;
    merged->act_executing = false;
    merged->has_send_act = false;
    addressMapping(*merged);
    return merged;
}

Transaction *Rmw::build_rmw_256b_transaction(Transaction *trans) const {
    Transaction *rmw_trans = new Transaction(trans);
    uint64_t base = (trans->address / 256) * 256;
    rmw_trans->address = base;
    rmw_trans->mask_wcmd = true;
    rmw_trans->burst_length = (256 * 8 / DMC_DATA_BUS_BITS) - 1;
    rmw_trans->data_size = 256;
    rmw_trans->timeAdded = 0;
    rmw_trans->inject_time = 0;
    rmw_trans->issue_size = 0;
    rmw_trans->data_ready_cnt = 0;
    rmw_trans->nextCmd = INVALID;
    rmw_trans->has_active = false;
    rmw_trans->act_executing = false;
    rmw_trans->has_send_act = false;
    addressMapping(*rmw_trans);
    return rmw_trans;
}

bool Rmw::dispatch_buffered_write(size_t entry_index) {
    if (entry_index >= write_merge_buffer.size()) return false;

    BufferedWriteEntry entry = write_merge_buffer[entry_index];
    bool ret = false;

    if (entry.has_second) {
        unsigned first_beats = entry.first_trans->burst_length + 1;
        unsigned second_beats = entry.second_trans->burst_length + 1;
        if (entry.first_data_ready_cnt < first_beats || entry.second_data_ready_cnt < second_beats) return false;
        uint64_t merged_task = entry.second_trans->task;
        unsigned ready_beats = entry.first_data_ready_cnt + entry.second_data_ready_cnt;
        Transaction *merged = build_merged_transaction(entry, merged_task);
        if (merged == NULL) return false;
        merged->data_ready_cnt = 0;
        ret = addMergedWriteTransaction(merged);
        if (!ret) {
            delete merged;
            return false;
        }
        totalWriteMergePair++;
        for (unsigned i = 0; i < ready_beats; i++) {
            top->receiveFromCPU(0, merged_task);
        }
        enqueue_pending_write_resp(entry.first_trans->task, entry.first_trans->channel);
        top->tasks_info[entry.first_trans->task].wr_finish = true;

        delete entry.first_trans;
        delete entry.second_trans;
    } else {
        if (UNPAIRED_TO_RMW_EN) {
            uint64_t task = entry.first_trans->task;
            unsigned ready_beats = entry.first_data_ready_cnt;
            unsigned expected_beats = entry.first_trans->burst_length + 1;
            if (ready_beats < expected_beats) {
                write_merge_buffer[entry_index] = entry;
                return false;
            }
            Transaction *rmw_trans = build_rmw_256b_transaction(entry.first_trans);
            rmw_trans->data_ready_cnt = 0;
            ret = addTransactionWithoutWriteMerge(rmw_trans);
            if (!ret) {
                delete rmw_trans;
                return false;
            }
            totalWriteMergeUnpairedToRmw++;
            MaskWriteDataBeats[rmw_trans->task] = entry.first_trans->burst_length + 1;
            MaskWriteSendBeats[rmw_trans->task] = rmw_trans->burst_length + 1;
            if (ready_beats > 0) {
                prefetched_write_data_beats[rmw_trans->task] += ready_beats;
            }

            unsigned remaining = entry.first_trans->burst_length + 1 - ready_beats;
            if (remaining > 0) {
                WriteDataRoute route;
                route.internal_task = rmw_trans->task;
                route.remaining_beats = remaining;
                route.resp_task = task;
                route.resp_channel = entry.first_trans->channel;
                route.has_resp = true;
                write_data_routes[task].push_back(route);
            }

            for (size_t i = 0; i < RmwQue.size(); i++) {
                if (RmwQue[i] == rmw_trans) {
                    RmwQue[i]->data_ready_cnt = 0;
                    break;
                }
            }
            delete entry.first_trans;
        } else {
            uint64_t task = entry.first_trans->task;
            unsigned ready_beats = entry.first_data_ready_cnt;
            entry.first_trans->data_ready_cnt = 0;
            ret = addMergedWriteTransaction(entry.first_trans);
            if (!ret) {
                return false;
            }
            totalWriteMergeUnpairedDirect++;
            for (unsigned i = 0; i < ready_beats; i++) {
                top->receiveFromCPU(0, task);
            }
        }
    }

    write_merge_buffer.erase(write_merge_buffer.begin() + entry_index);
    return true;
}

void Rmw::pump_write_merge_buffer() {
    bool progress = true;
    while (progress) {
        progress = false;
        for (size_t i = 0; i < write_merge_buffer.size(); i++) {
            if (write_merge_buffer[i].has_second) {
                progress = dispatch_buffered_write(i);
                break;
            }
        }
    }
}

bool Rmw::is_task_buffered(uint64_t task) const {
    for (size_t i = 0; i < write_merge_buffer.size(); i++) {
        if (write_merge_buffer[i].first_trans != NULL && write_merge_buffer[i].first_trans->task == task) {
            return true;
        }
        if (write_merge_buffer[i].has_second && write_merge_buffer[i].second_trans != NULL
                && write_merge_buffer[i].second_trans->task == task) {
            return true;
        }
    }
    return false;
}

void Rmw::enqueue_pending_write_resp(uint64_t external_task, unsigned upstream_channel) {
    PendingWriteResp resp;
    resp.external_task = external_task;
    resp.upstream_channel = upstream_channel;
    pending_write_merge_resps.push_back(resp);
}

void Rmw::pump_pending_write_resps() {
    while (!pending_write_merge_resps.empty()) {
        const PendingWriteResp &resp = pending_write_merge_resps.front();
        // if (!cmd_response(resp.external_task, 0, resp.upstream_channel)) {
        if (!((*(top->parentMemorySystem)->WriteResp)(resp.upstream_channel,resp.external_task,0,0,0))) {
            return;
        }
        pending_write_merge_resps.pop_front();
    }
}

bool Rmw::handle_write_merge_transaction(Transaction *trans) {
    pump_write_merge_buffer();

    for (size_t i = 0; i < write_merge_buffer.size(); i++) {
        if (write_merge_buffer[i].has_second) continue;

        if (can_merge_write_pair(write_merge_buffer[i].first_trans, trans)) {
            write_merge_buffer[i].second_trans = trans;
            write_merge_buffer[i].has_second = true;
            pump_write_merge_buffer();
            return true;
        }
    }

    if (write_merge_buffer.size() >= WRITE_MERGE_BUFFER_DEPTH) {
        totalWriteMergeBufferFull++;
        if (!dispatch_buffered_write(0)) {
            return false;
        }
    }

    BufferedWriteEntry entry;
    entry.first_trans = trans;
    write_merge_buffer.push_back(entry);
    totalWriteMergeInput++;
    pump_write_merge_buffer();
    return true;
}

bool Rmw::addMergedWriteTransaction(Transaction *trans) {
    if (trans == NULL) return false;
    if (trans->transactionType != DATA_WRITE) return false;
    if (trans->mask_wcmd) return false;

    if (trans->data_size == 256) {
        trans->burst_length = (256 * 8 / DMC_DATA_BUS_BITS) - 1;
    }
    trans->mask_wcmd = false;
    trans->issue_size = 0;
    trans->nextCmd = INVALID;
    trans->has_active = false;
    trans->act_executing = false;
    trans->has_send_act = false;
    return addTransactionWithoutWriteMerge(trans);
}

void Rmw::addData(uint32_t *data, uint64_t task) {

    pre_req_data_time = now();

    for (size_t i = 0; i < write_merge_buffer.size(); i++) {
        BufferedWriteEntry &entry = write_merge_buffer[i];
        if (entry.first_trans != NULL && entry.first_trans->task == task) {
            if (entry.first_data_ready_cnt < entry.first_trans->burst_length + 1) {
                entry.first_data_ready_cnt++;
            }
            pump_write_merge_buffer();
            return;
        }
        if (entry.has_second && entry.second_trans != NULL && entry.second_trans->task == task) {
            if (entry.second_data_ready_cnt < entry.second_trans->burst_length + 1) {
                entry.second_data_ready_cnt++;
            }
            pump_write_merge_buffer();
            return;
        }
    }

    map<uint64_t, deque<WriteDataRoute> >::iterator route_it = write_data_routes.find(task);
    uint64_t target_task = task;
    if (route_it != write_data_routes.end() && !route_it->second.empty()) {
        target_task = route_it->second.front().internal_task;
    }
    map<uint64_t, unsigned>::iterator prefetched_route_it = prefetched_write_data_beats.find(target_task);
    if (prefetched_route_it != prefetched_write_data_beats.end()) {
        bool consumed_prefetched = false;
        for (auto &rmwq : RmwQue) {
            unsigned expected_data_beats = rmwq->burst_length + 1;
            map<uint64_t, unsigned>::iterator expected_it = MaskWriteDataBeats.find(rmwq->task);
            if (expected_it != MaskWriteDataBeats.end()) expected_data_beats = expected_it->second;
            if ((((rmwq->transactionType == DATA_WRITE)&&(rmwq->mask_wcmd==true)) || ((rmwq->transactionType == DATA_WRITE)&&(RMW_CMD_MODE==1)&&(rmwq->mask_wcmd==false))
                    || ((rmwq->transactionType == DATA_WRITE)&&(rmwq->mask_wcmd==false)&&(rmwq->ecc_flag==true)))
                    && rmwq->data_ready_cnt < expected_data_beats && target_task==rmwq->task) {
                rmwq->data_ready_cnt++;
                consumed_prefetched = true;
                if (DEBUG_BUS) {
                     PRINTN(setw(10)<<now()<<" -- RMW_MATCH :: data_ready_cnt:"<<rmwq->data_ready_cnt
                             <<", data_size="<<rmwq->data_size<<", task="<<rmwq->task<<endl);
                }
                break;
            }
        }
        if (consumed_prefetched) {
            prefetched_route_it->second--;
            if (prefetched_route_it->second == 0) {
                prefetched_write_data_beats.erase(prefetched_route_it);
            }
            return;
        }
    }
    
    bool task_match = false;
    for (auto &rmwq : RmwQue) {
        if (target_task == rmwq->task) {
            task_match=true;
        };
        unsigned expected_data_beats = rmwq->burst_length + 1;
        map<uint64_t, unsigned>::iterator expected_it = MaskWriteDataBeats.find(rmwq->task);
        if (expected_it != MaskWriteDataBeats.end()) expected_data_beats = expected_it->second;
        if ((((rmwq->transactionType == DATA_WRITE)&&(rmwq->mask_wcmd==true)) || ((rmwq->transactionType == DATA_WRITE)&&(RMW_CMD_MODE==1)&&(rmwq->mask_wcmd==false))
                || ((rmwq->transactionType == DATA_WRITE)&&(rmwq->mask_wcmd==false)&&(rmwq->ecc_flag==true)))  // ecc full_write, must collect all data
                && rmwq->data_ready_cnt < expected_data_beats && target_task==rmwq->task) {
            rmwq->data_ready_cnt ++;
            if (route_it != write_data_routes.end() && !route_it->second.empty()) {
                WriteDataRoute &route = route_it->second.front();
                if (route.remaining_beats > 0) {
                    route.remaining_beats--;
                }
                if (route.remaining_beats == 0) {
                    if (route.has_resp) {
                        enqueue_pending_write_resp(route.resp_task, route.resp_channel);
                    }
                    route_it->second.pop_front();
                    if (route_it->second.empty()) {
                        write_data_routes.erase(route_it);
                    }
                }
            }
            if (DEBUG_BUS) {
                 PRINTN(setw(10)<<now()<<" -- RMW_MATCH :: data_ready_cnt:"<<rmwq->data_ready_cnt
                         <<", data_size="<<rmwq->data_size<<", task="<<rmwq->task<<endl);
            }
//            return true;
            break;
        }
    }

    if (task_match==false) {
        if (is_task_buffered(task)) {
            if (DEBUG_BUS) {
                 PRINTN(setw(10)<<now()<<" -- RMW_WDATA_WAIT_MERGE :: task="<<task<<endl);
            }
            return;
        }
        if (DEBUG_BUS) {
             PRINTN(setw(10)<<now()<<" -- RMW_WDATA_BYPASS :: task="<<target_task<<endl);
        }
//        bool ret = false;
//        if (EM_ENABLE) {
//            ret = top->channels[0]->addData(data ,task);
//        } else {
//            ret = top->channels[channel]->addData(data ,task);
//        }
        top->receiveFromCPU(data ,target_task);
        if (route_it != write_data_routes.end() && !route_it->second.empty()) {
            WriteDataRoute &route = route_it->second.front();
            if (route.remaining_beats > 0) {
                route.remaining_beats--;
            }
            if (route.remaining_beats == 0) {
                if (route.has_resp) {
                    enqueue_pending_write_resp(route.resp_task, route.resp_channel);
                }
                route_it->second.pop_front();
                if (route_it->second.empty()) {
                    write_data_routes.erase(route_it);
                }
            }
        }
//        return ret;
    }

//    ERROR(setw(10)<<now()<<" -- Impossible wdata, task="<<task);
//    assert(0);

}

bool Rmw::addTransactionWithoutWriteMerge(Transaction * trans) {

//    if (trans->data_size == 0) {
//         DEBUG(" task="<<trans->task<<" data size="<<trans->data_size);
//    }

    if ((trans->transactionType==DATA_READ)&&(trans->mask_wcmd==true)){
        ERROR(setw(10)<<now()<<" -- No mask Flag In Read, task="<<trans->task<<" type="<<trans->transactionType
               <<" address="<<hex<<trans->address<<" mask_write="<<trans->mask_wcmd);
        assert(0);
    }


//    uint8_t ch = trans->channel;
    bool rmw_que_full  = rmw_cmd_cnt >= RMW_QUE_DEPTH && RMW_QUE_DEPTH != 0;
    bool rmw_que_empty = rmw_cmd_cnt == 0;

    if (rmw_que_full) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- RMW BP CMD :: task="<<trans->task<<" type="<<trans->transactionType<<" mask_write="<<trans->mask_wcmd
                    <<" qos="<<trans->qos<<" burst_length:"<<trans->burst_length<<" address="<<hex<<trans->address
                    <<dec<<" rank="<<trans->rank<<" bank="<<trans->bankIndex<<" row="<<trans->row<<" channel="<<trans->channel<<" (rmw_cmd_cnt:"<<rmw_cmd_cnt
                    <<")"<<endl);
        }
        return false;
    }

    if (rmw_que_empty) {
        if ((trans->transactionType==DATA_READ)||((trans->transactionType==DATA_WRITE)&&(RMW_CMD_MODE==0)&&(trans->mask_wcmd==false))) {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- RMW BYPASS :: task="<<trans->task<<" type="<<trans->transactionType<<" mask_write="<<trans->mask_wcmd
                        <<" ecc_flag="<<trans->ecc_flag<<" qos="<<trans->qos<<" burst_length="<<trans->burst_length<<" address="<<hex<<trans->address
                        <<dec<<" rank="<<trans->rank<<" bank="<<trans->bankIndex<<" row="<<trans->row<<" channel="<<trans->channel<<" col="<<trans->col
                        <<" addr_col="<<trans->addr_col<<" (rmw_cmd_cnt:"<<rmw_cmd_cnt<<")"<<endl);
            }

//            bool ret = top->channels[ch]->addTransaction(trans); 
            bool ret = top->addTransaction(trans);
            
            if (ret) {
                if (trans->transactionType==DATA_READ){
                    totalBypassReads++;
                } else {
                    totalBypassWrites++;
                    totalFullWrites ++;
                }
            }
            
            return ret;
        }
    }

    pre_req_time = now();
    cmd_state *c = new cmd_state;
    c->task = trans->task;
    c->rmwTimeAdded = now();
    if (trans->transactionType == DATA_READ) {
        cmd_set_conflict(trans);
        if (RMW_CONF_SIZE == 32) {
            trans->arb_time = now() + 2;
        } else {
            trans->arb_time = now() + 1;
        }

        rcmd_cnt ++;
        rmw_cmd_cnt ++;
        totalReads ++;
        totalTransactions ++;
        RmwQue.push_back(trans);
        RmwCmdState.push_back(c);

        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- RMW_ADD ::[R]B["<<trans->burst_length<<"]"<<"QOS["<<trans->qos<<"] addr="<<hex
                    <<trans->address<<dec<<" col="<<trans->col<<" addr_col="<<trans->addr_col<<" task="<<trans->task<<" type="<<trans->transactionType
                    <<" mask_write="<<trans->mask_wcmd<<" ecc_flag="<<trans->ecc_flag<<" rank="<<trans->rank<<" group="<<trans->group<<" bank="
                    <<trans->bankIndex<<" row="<<trans->row<<" channel="<<trans->channel<<" (rmw_cmd_cnt:"<<rmw_cmd_cnt<<")"<<endl);
        }
        return true;
    } else {
        cmd_set_conflict(trans);
        if (RMW_CONF_SIZE == 32) {
            trans->arb_time = now() + 3;
        } else {
            trans->arb_time = now() + 2;
        }
        if (trans->mask_wcmd == true) {
//            top->channels[ch]->memoryController->rmw_rd_finish[trans->task] = false;
            top->rmw_rd_finish[trans->task] = false;
            totalMaskWrites ++;
        } else {
            totalFullWrites ++;
        }

        rmw_cmd_cnt ++;
        wcmd_cnt ++;
        totalWrites ++;
        totalTransactions ++;
        RmwQue.push_back(trans);
        RmwCmdState.push_back(c);
        
        if (RMW_CMD_MODE==1 && !trans->ecc_flag) {
            gen_cresp(trans->task);
            RmwCmdRespCh.push_back(trans->channel);
        }

//        if (!RmwCmdResp.empty()) {
//            if (pre_cresp_time != now()) {
//                if (cmd_response(RmwCmdResp[0], 0, ch)) {
//                    if (DEBUG_BUS) {
//                        PRINTN(setw(10)<<now()<<" -- Rmw Cresp Received :: task="<<RmwCmdResp[0]<<" RMW MODE="<<RMW_CMD_MODE<<endl);
//                    }
//                    pre_cresp_time = now();
//                    RmwCmdResp.erase(RmwCmdResp.begin());
//                } else {
//                    if (DEBUG_BUS) {
//                        PRINTN(setw(10)<<now()<<" -- Rmw Cresp Back Pressure :: task="<<RmwCmdResp[0]<<" RMW MODE="<<RMW_CMD_MODE<<endl);
//                    }
//                }
//            }
//        }
        
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- RMW_ADD :: [W]B["<<trans->burst_length<<"]"<<"QOS["<<trans->qos<<"] addr="<<hex
                    <<trans->address<<dec<<" col="<<trans->col<<" addr_col="<<trans->addr_col<<" task="<<trans->task<<" type="<<trans->transactionType
                    <<" mask_write="<<trans->mask_wcmd<<" ecc_flag="<<trans->ecc_flag<<" rank="<<trans->rank<<" group="<<trans->group<<" bank="
                    <<trans->bankIndex<<" row="<<trans->row<<" channel="<<trans->channel<<" (rmw_cmd_cnt:"<<rmw_cmd_cnt<<")"<<endl);
        }
        return true;
    }
}

bool Rmw::addTransaction(Transaction * trans) {
    if ((trans->transactionType==DATA_WRITE) && is_write_merge_candidate(trans)) {
        return handle_write_merge_transaction(trans);
    }
    return addTransactionWithoutWriteMerge(trans);
}

void Rmw::gen_cresp(uint64_t task) {
    RmwCmdResp.push_back(task);
}

bool Rmw::cmd_response(uint64_t task,uint64_t address, uint8_t ch) {
//     return ((*(top->channels[ch])->CmdResp)(ch,task,0,0,0));
     return ((*(top->parentMemorySystem)->CmdResp)(ch,task,0,0,0));
}

void Rmw::update_cresp() {
    // RMW cresp return to ha 
    if (!RmwCmdResp.empty()) {
        if (pre_cresp_time != now()) {
            if (cmd_response(RmwCmdResp[0], 0, RmwCmdRespCh[0])) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- Rmw Cresp Received :: task="<<RmwCmdResp[0]<<" RMW MODE="<<RMW_CMD_MODE<<endl);
                }
                pre_cresp_time = now();
                RmwCmdResp.erase(RmwCmdResp.begin());
                RmwCmdRespCh.erase(RmwCmdRespCh.begin());
            } else {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- Rmw Cresp Back Pressure :: task="<<RmwCmdResp[0]<<" RMW MODE="<<RMW_CMD_MODE<<endl);
                }
            }
        }
    }
}

    void Rmw::func_check() {
    if (RmwQue.size() != rmw_cmd_cnt) {
        ERROR(setw(10)<<now()<<" -- RmwQue Unmatch, RmwQue="<<RmwQue.size()<<", rmw_cmd_cnt="<<rmw_cmd_cnt);
        assert(0);
    }
}

void Rmw::update() {
    pump_write_merge_buffer();
    pump_pending_write_resps();
    update_cresp();
#if 0
    func_check();
#endif
    update_state();
//    check_timeout();
    sch_que();
    arb_node();
    send_wdata();
}

void Rmw::update_state() {
    if (!DEBUG_RMW_STATE) return;
    unsigned size = RmwQue.size();
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl)
    PRINTN("Rmw Total Status: R:"<<rcmd_cnt<<" W:"<<wcmd_cnt<<" R+W:"<<rmw_cmd_cnt<<endl);
    for (unsigned i = 0; i < size; i ++) {
        auto t = RmwQue[i];
        auto r = RmwConfCnt[i];
        auto s = RmwCmdState[i];
        PRINTN("Rmw time: "<<now()<<" | type="<<t->transactionType<<" | task="<<t->task<<" | bank="<<t->bankIndex<<" | rank="<<t->rank<<" | row="
                <<t->row<<" | address="<<hex<<t->address<<dec<<" | addr_col="<<t->addr_col<<" | len="<<t->burst_length<<" | channel="<<t->channel<<" | data_size="<<t->data_size
                <<" | data_ready_cnt="<<t->data_ready_cnt<<" | timeout="<<t->timeout<<" | qos="<<t->qos<<" | pri="<<t->pri
                <<" | rd_byp="<<t->has_active<<" | mask_wcmd="<<t->mask_wcmd<<" | addr_conf_cnt="<<r->ad_conf_cnt<<" | cmd_state="
                <<s->rmwState<<" | RMW MODE="<<RMW_CMD_MODE<<endl);
    }
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl)
}


void Rmw::sch_que() {
    if (RmwQue.empty()) return;

    // command number check
    if ((rcmd_cnt+wcmd_cnt)!=rmw_cmd_cnt) {
        ERROR(setw(10)<<now()<<" -- Cmd Number Chk Failed, No.Read="<<rcmd_cnt<<" No.Write="<<wcmd_cnt<<" No.Total="<<rmw_cmd_cnt);
        assert(0);
    }
    
    unsigned size = RmwQue.size();
    for (unsigned i = 0; i < size; i++) {

        //prevent big latency in RMW QUE
        if ((now() - RmwCmdState[i]->rmwTimeAdded) > 1000000) {
            ERROR(setw(10)<<now()<<" -- task="<<RmwQue[i]->task<<" address="<<hex<<RmwQue[i]->address<<dec
                    <<" rank="<<RmwQue[i]->rank<<" bank="<<RmwQue[i]->bankIndex<<" row="<<RmwQue[i]->row<<" type="
                    <<RmwQue[i]->transactionType<<" mask_wcmd="<<RmwQue[i]->mask_wcmd);
            ERROR(setw(10)<<now()<<" -- error, qos="<<RmwQue[i]->qos<<", pri="<<RmwQue[i]->pri);
            ERROR(setw(10)<<now()<<" -- RMW FATAL ERROR == big latency"<<", chnl:"<<RmwQue[i]->channel);
            assert(0);
        }
        //prevent wdata lost in RMW QUE
        if ((RmwQue[i]->transactionType == DATA_WRITE)&&(RmwQue[i]->mask_wcmd || (!RmwQue[i]->mask_wcmd && RMW_CMD_MODE))) {
            unsigned expected_data_beats = RmwQue[i]->burst_length + 1;
            map<uint64_t, unsigned>::iterator expected_it = MaskWriteDataBeats.find(RmwQue[i]->task);
            if (expected_it != MaskWriteDataBeats.end()) expected_data_beats = expected_it->second;
            map<uint64_t, unsigned>::iterator prefetched_it = prefetched_write_data_beats.find(RmwQue[i]->task);
            unsigned prefetched_beats = prefetched_it == prefetched_write_data_beats.end() ? 0 : prefetched_it->second;
            if (now() - RmwCmdState[i]->rmwTimeAdded > 100000 && RmwQue[i]->data_ready_cnt + prefetched_beats < expected_data_beats) {
                ERROR(setw(10)<<now()<<" -- RMW_DMC["<<RmwQue[i]->channel<<"] task="<<RmwQue[i]->task<<" Wdata number miss match, EXP="
                        <<expected_data_beats<<", ACT="<<RmwQue[i]->data_ready_cnt + prefetched_beats);
                assert(0);
            }
        }


//        uint8_t ch = EM_ENABLE ? 0 : RmwQue[i]->channel;
        //Merge between write data and read data (Mask Write)
        if ((RmwQue[i]->transactionType==DATA_WRITE) && (RmwQue[i]->mask_wcmd==true)) {
//            auto it = top->channels[ch]->memoryController->rmw_rd_finish.find(RmwQue[i]->task);
//            if (it != top->channels[ch]->memoryController->rmw_rd_finish.end()) {
            auto it = top->rmw_rd_finish.find(RmwQue[i]->task);
            if (it != top->rmw_rd_finish.end()) {
                unsigned expected_data_beats = RmwQue[i]->burst_length + 1;
                map<uint64_t, unsigned>::iterator expected_it = MaskWriteDataBeats.find(RmwQue[i]->task);
                if (expected_it != MaskWriteDataBeats.end()) expected_data_beats = expected_it->second;
                map<uint64_t, unsigned>::iterator prefetched_it = prefetched_write_data_beats.find(RmwQue[i]->task);
                unsigned prefetched_beats = prefetched_it == prefetched_write_data_beats.end() ? 0 : prefetched_it->second;
                if((RmwQue[i]->data_ready_cnt + prefetched_beats >= expected_data_beats) && (it->second==true)){
                    RmwCmdState[i]->rmwState = SEND_READY;
                    top->rmw_rd_finish.erase(it);
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- RMW WDATA RDATA MERGE ::[MaskW]B["<<RmwQue[i]->burst_length<<"]"<<"QOS["<<RmwQue[i]->qos<<"] addr="<<hex
                            <<RmwQue[i]->address<<dec<<" task="<<RmwQue[i]->task<<" type="<<RmwQue[i]->transactionType<<" mask_write="<<RmwQue[i]->mask_wcmd
                            <<" rank="<<RmwQue[i]->rank<<" group="<<RmwQue[i]->group<<" bank="
                            <<RmwQue[i]->bankIndex<<" row="<<RmwQue[i]->row<<" mode="<<RMW_CMD_MODE<<" (rmw_cmd_cnt:"<<rmw_cmd_cnt<<")"<<endl);
                    } 
                }
            }
        }

        //Full Write under non fast cmd mode: waiting for write data
        if (((RmwQue[i]->transactionType==DATA_WRITE) && (RmwQue[i]->mask_wcmd==false) && (RMW_CMD_MODE==1)) 
                || ((RmwQue[i]->transactionType==DATA_WRITE) && (RmwQue[i]->mask_wcmd==false) && (RmwQue[i]->ecc_flag==true))) {
            unsigned expected_data_beats = RmwQue[i]->burst_length + 1;
            map<uint64_t, unsigned>::iterator expected_it = MaskWriteDataBeats.find(RmwQue[i]->task);
            if (expected_it != MaskWriteDataBeats.end()) expected_data_beats = expected_it->second;
            map<uint64_t, unsigned>::iterator prefetched_it = prefetched_write_data_beats.find(RmwQue[i]->task);
            unsigned prefetched_beats = prefetched_it == prefetched_write_data_beats.end() ? 0 : prefetched_it->second;
            if(RmwQue[i]->data_ready_cnt + prefetched_beats >= expected_data_beats) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- RMW WDATA MATCH ::[FullW]B["<<RmwQue[i]->burst_length<<"]"<<"QOS["<<RmwQue[i]->qos<<"] addr="<<hex
                        <<RmwQue[i]->address<<dec<<" task="<<RmwQue[i]->task<<" type="<<RmwQue[i]->transactionType<<" mask_write="<<RmwQue[i]->mask_wcmd
                        <<" rank="<<RmwQue[i]->rank<<" group="<<RmwQue[i]->group<<" bank="
                        <<RmwQue[i]->bankIndex<<" row="<<RmwQue[i]->row<<" mode="<<RMW_CMD_MODE<<" (rmw_cmd_cnt:"<<rmw_cmd_cnt<<")"<<endl);
                } 
                RmwCmdState[i]->rmwState = SEND_READY;   
            }
        }
    }
}



void Rmw::arb_node() {
    if (RmwQue.empty()) return;

    unsigned size = RmwQue.size();
    for (unsigned i = 0; i < size; i++) {
        if ((RmwQue[i]->transactionType == DATA_READ) && (RmwCmdState[i]->rmwState!=QUE_WAITING)){
            ERROR(setw(10)<<now()<<" -- Read Cmd State Wrong, task="<<RmwQue[i]->task<<" type="<<RmwQue[i]->transactionType<<" channel="<<RmwQue[i]->channel
                   <<" address="<<hex<<RmwQue[i]->address<<" cmd_state="<<RmwCmdState[i]->rmwState<<" mask_write="<<RmwQue[i]->mask_wcmd
                   <<" rmw_mode="<<RMW_CMD_MODE);
            assert(0);
        }

        if (now() < RmwQue[i]->arb_time) { 
            continue;
        }
        if (RmwConfCnt[i]->ad_conf_cnt != 0) {
            continue;
        }
        if ((RmwCmdState[i]-> rmwState == MERGE_READ)&&(RmwQue[i]->mask_wcmd==true)) {
            continue;
        }
        if ((RmwQue[i]->transactionType == DATA_WRITE)&&(RmwQue[i]->mask_wcmd==false)&&(RMW_CMD_MODE==1)&&(RmwCmdState[i]-> rmwState!=SEND_READY)) {
            continue;
        }
        if ((RmwQue[i]->transactionType == DATA_WRITE)&&(RmwQue[i]->mask_wcmd==false)&&(RmwQue[i]->ecc_flag==true)&&(RmwCmdState[i]-> rmwState!=SEND_READY)) {
            continue;
        }

        uint8_t ch = RmwQue[i]->channel;
        
        // RMW_CMD_MODE: 0 -> fast command mode; 1 -> non fast command mode
        if ((RmwQue[i]->transactionType == DATA_WRITE)&&(RmwQue[i]->mask_wcmd==true)&&(RmwCmdState[i]->rmwState==QUE_WAITING)) {  
            Transaction *trans = new Transaction(RmwQue[i]);
            trans->transactionType = DATA_READ; 
//            if (top->channels[ch]->addTransaction(trans)) {
            if (top->addTransaction(trans)) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- RMW SCH :: [MERGE_READ_CMD] task="<<RmwQue[i]->task<<" type="<<RmwQue[i]->transactionType<<" mask_write="<<RmwQue[i]->mask_wcmd<<" ecc_flag="<<RmwQue[i]->ecc_flag
                            <<" qos="<<RmwQue[i]->qos<<" burst_length="<<RmwQue[i]->burst_length<<" channel="<<RmwQue[i]->channel<<" data_ready_cnt="<<RmwQue[i]->data_ready_cnt<<" address="<<hex<<RmwQue[i]->address
                            <<dec<<" rank="<<RmwQue[i]->rank<<" bank="<<RmwQue[i]->bankIndex<<" row="<<RmwQue[i]->row<<" col="<<trans->col<<" addr_col="<<trans->addr_col<<" rmw_mode="<<RMW_CMD_MODE
                            <<" rmw_cmd_cnt="<<rmw_cmd_cnt<<endl);
                }
                RmwCmdState[i]->rmwState = MERGE_READ;
                break;
            } else {
                delete trans;
            }
        } else if ((RmwQue[i]->transactionType == DATA_READ)   // read cmd
                    || ((RmwQue[i]->transactionType == DATA_WRITE)&&(RmwQue[i]->mask_wcmd==false)&&(RMW_CMD_MODE==1)&&(RmwCmdState[i]->rmwState==SEND_READY))  // full write cmd under non fast command mode
                    || ((RmwQue[i]->transactionType == DATA_WRITE)&&(RmwQue[i]->mask_wcmd==false)&&(RmwQue[i]->ecc_flag==true)&&(RmwCmdState[i]->rmwState==SEND_READY))  // full write cmd with ecc flag
                    || ((RmwQue[i]->transactionType == DATA_WRITE)&&(RmwQue[i]->mask_wcmd==true)&&(RmwCmdState[i]->rmwState==SEND_READY))) {     // mask write cmd
//            if (top->channels[ch]->addTransaction(RmwQue[i])) {
            if (top->addTransaction(RmwQue[i])) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- RMW SCH :: task="<<RmwQue[i]->task<<" type="<<RmwQue[i]->transactionType<<" mask_write="<<RmwQue[i]->mask_wcmd<<" ecc_flag="<<RmwQue[i]->ecc_flag 
                            <<" qos="<<RmwQue[i]->qos<<" burst_length="<<RmwQue[i]->burst_length<<" channel="<<RmwQue[i]->channel<<" data_ready_cnt="<<RmwQue[i]->data_ready_cnt<<" address="<<hex<<RmwQue[i]->address
                            <<dec<<" rank="<<RmwQue[i]->rank<<" bank="<<RmwQue[i]->bankIndex<<" row="<<RmwQue[i]->row<<" col="<<RmwQue[i]->col<<" addr_col="<<RmwQue[i]->addr_col<<" rmw_mode="<<RMW_CMD_MODE
                            <<" rmw_cmd_cnt="<<rmw_cmd_cnt<<endl);
                }

                // collect all wdata for sended write cmd
                if ((RmwQue[i]->transactionType==DATA_WRITE) && ((RmwQue[i]->mask_wcmd==true || (RmwQue[i]->mask_wcmd==false && RMW_CMD_MODE==1) || (!RmwQue[i]->mask_wcmd && RmwQue[i]->ecc_flag)))) {
                    unsigned send_beats = RmwQue[i]->burst_length + 1;
                    map<uint64_t, unsigned>::iterator send_it = MaskWriteSendBeats.find(RmwQue[i]->task);
                    if (send_it != MaskWriteSendBeats.end()) send_beats = send_it->second;
                    for (size_t j = 0; j < send_beats; j++) {
                        WdataToSend.push_back(RmwQue[i]->task);
                        WdataChannel.push_back(ch);
                    }
                }

                //update statistic info
                if (RmwQue[i]->transactionType == DATA_READ) {
                    rcmd_cnt --;
                } else {
                    wcmd_cnt --;
                }
                rmw_cmd_cnt --;

                //delete all states related to sended cmd this round
                cmd_release_conflict(RmwQue[i]);
                MaskWriteDataBeats.erase(RmwQue[i]->task);
                MaskWriteSendBeats.erase(RmwQue[i]->task);
                prefetched_write_data_beats.erase(RmwQue[i]->task);
                delete RmwConfCnt[i];
                RmwConfCnt.erase(RmwConfCnt.begin() + i);
                delete RmwCmdState[i];
                RmwCmdState.erase(RmwCmdState.begin() + i);
                RmwQue.erase(RmwQue.begin() + i);
                break;
            }
        } else if ((RmwQue[i]->transactionType == DATA_WRITE)&&(RmwQue[i]->mask_wcmd==false)&&(RMW_CMD_MODE==0)) {     // full write cmd under fast cmd mode
//            if (top->channels[ch]->addTransaction(RmwQue[i])) {
            if (top->addTransaction(RmwQue[i])) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- RMW SCH :: task="<<RmwQue[i]->task<<" type="<<RmwQue[i]->transactionType<<" mask_write="<<RmwQue[i]->mask_wcmd<<" ecc_flag="<<RmwQue[i]->ecc_flag 
                            <<" qos="<<RmwQue[i]->qos<<" burst_length="<<RmwQue[i]->burst_length<<" channel="<<RmwQue[i]->channel<<" data_ready_cnt="<<RmwQue[i]->data_ready_cnt<<" address="<<hex<<RmwQue[i]->address
                            <<dec<<" rank="<<RmwQue[i]->rank<<" bank="<<RmwQue[i]->bankIndex<<" row="<<RmwQue[i]->row<<" col="<<RmwQue[i]->col<<" addr_col="<<RmwQue[i]->addr_col<<" rmw_mode="<<RMW_CMD_MODE
                            <<" rmw_cmd_cnt="<<rmw_cmd_cnt<<endl);
                }

                //update statistic info
                if (RmwQue[i]->transactionType == DATA_READ) {
                    rcmd_cnt --;
                } else {
                    wcmd_cnt --;
                }
                rmw_cmd_cnt --;

                //delete all states related to sended cmd this round
                cmd_release_conflict(RmwQue[i]);
                MaskWriteDataBeats.erase(RmwQue[i]->task);
                MaskWriteSendBeats.erase(RmwQue[i]->task);
                delete RmwConfCnt[i];
                RmwConfCnt.erase(RmwConfCnt.begin() + i);
                delete RmwCmdState[i];
                RmwCmdState.erase(RmwCmdState.begin() + i);
                RmwQue.erase(RmwQue.begin() + i);
                break;
            }
        } else {
            ERROR(setw(10)<<now()<<" -- Such Cmd Not Expected, task="<<RmwQue[i]->task<<" type="<<RmwQue[i]->transactionType
                   <<" address="<<hex<<RmwQue[i]->address<<" cmd_state="<<RmwCmdState[i]->rmwState<<" mask_write="<<RmwQue[i]->mask_wcmd
                   <<" rmw_mode="<<RMW_CMD_MODE);
            assert(0);
            
        }
    }

}


void Rmw::send_wdata() {
    if (!WdataToSend.empty() && !WdataChannel.empty()) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- RMW_SEND_WDATA :: task="<<WdataToSend[0]<<" channel="<<WdataChannel[0]<<endl);
        }
        top->receiveFromCPU(0, WdataToSend[0]);
//        bool ret = top->channels[WdataChannel[0]]->addData(0, WdataToSend[0]);
//        if (ret) {
        WdataToSend.erase(WdataToSend.begin());
        WdataChannel.erase(WdataChannel.begin());
//        }
    }
}


unsigned Rmw:: GetRmwQsize() {
    return (rmw_cmd_cnt);
}

void Rmw::check_cnt() {
    uint32_t size = GetRmwQsize();
    if (size >= rmw_que_cnt.size()) {
        rmw_que_cnt.resize(size + 1, 0);
    }
    rmw_que_cnt.at(size)++;
}

//void Rmw::register_write(uint64_t address, uint32_t data) {
//    uint32_t offset = ((address != 0) ? 4 : 0);
//    switch (offset) {
//        case 0x0:{
//            start_cycle = top->now();
//            break;
//        }
//        case 0x4:{
//            end_cycle = top->now();
//            if (start_cycle != end_cycle && STATE_LOG == true) {
//                statistics();
//            }
//            break;
//        }
//        default: break;
//    }
//}


//void Rmw::statistics() {
//    unsigned size = 0;
//    STATE_PRINTN(setiosflags(ios::left));
//    STATE_PRINTN("======================================== START ========================================\n");
//    STATE_PRINTN("-------------------- Base Message -----------------------------------------------------\n");
//    STATE_PRINTN(DDR_TYPE<<" "<<DMC_RATE<<"Mbps, x"<<JEDEC_DATA_BUS_BITS<<", DMC Data Width: "
//            <<DMC_DATA_BUS_BITS<<", CKR: "<<setprecision(1)<<WCK2DFI_RATIO<<endl);
//    STATE_PRINTN("Current time: "<<fixed<<now()<<", tDFI: "<<setprecision(4)<<tDFI<<endl);
//
//    unsigned reads = totalReads;
//    unsigned bypass_reads = totalBypassReads;
//    unsigned writes = totalWrites;
//    unsigned bypass_writes = totalBypassWrites;
//    unsigned full_writes = totalFullWrites;
//    unsigned mask_writes = totalMaskWrites;
//    unsigned totals = totalTransactions;
////    unsigned address_conf_cnt = addrconf_cnt;
////    unsigned read_cnt = read_cnt;
////    unsigned write_cnt = write_cnt;
////    unsigned mwrite_cnt = mwrite_cnt;
//
//    STATE_PRINTN("-------------------- Task Statistics (DMC Command Number) -----------------------------\n");
//    
//    STATE_PRINTN("Read            : "<<setw(8)<<reads-pre_reads);
//    STATE_PRINTN(" | Total reads       : "<<setw(8)<<reads);
//    STATE_PRINTN(" | Bypass Read            : "<<setw(8)<<bypass_reads-pre_bypass_reads);
//    STATE_PRINTN(" | Total bypass reads       : "<<setw(8)<<bypass_reads<<" | "<<endl);
//    STATE_PRINTN("Write           : "<<setw(8)<<writes-pre_writes);
//    STATE_PRINTN(" | Total writes      : "<<setw(8)<<writes);
//    STATE_PRINTN(" | Full Write             : "<<setw(8)<<full_writes-pre_full_writes);
//    STATE_PRINTN(" | Total full writes        : "<<setw(8)<<full_writes<<" | "<<endl);
//    STATE_PRINTN("Mask Write      : "<<setw(8)<<mask_writes-pre_mask_writes);
//    STATE_PRINTN(" | Total mask writes : "<<setw(8)<<mask_writes);
//    STATE_PRINTN(" | Bypass Write           : "<<setw(8)<<bypass_writes-pre_bypass_writes);
//    STATE_PRINTN(" | Total bypass writes      : "<<setw(8)<<bypass_writes<<" | "<<endl);
//    STATE_PRINTN("Total           : "<<setw(8)<<totals-pre_totals);
//    STATE_PRINTN(" | Total commands    : "<<setw(8)<<totals<<" | "<<endl);
//
//
//
////    STATE_PRINTN("-------------------- Confilct Statistics (DDR Command Number) -------------------------\n");
////    STATE_PRINTN(setw(36)<<"Address conflict"<<" : "<<setw(12)<<address_conf_cnt - pre_address_conf_cnt);
////    STATE_PRINTN(" | "<<setw(36)<<"Total address conf cnt"<<" : "<<address_conf_cnt<<endl);
//
//
////    STATE_PRINTN("-------------------- RMW Pressure Statistics (Percentage/Cycle) -----------------------\n");
////    float ratio = float(task_cnt) * 100 / STATE_TIME;
////    STATE_PRINTN(setw(15)<<"Cmd valid"<<" : "<<setw(10)<<task_cnt<<" | ");
////    STATE_PRINTN(setw(15)<<"Ratio"<<" : "<<setw(10)<<ratio<<" | ");
////    ratio = float(total_task_cnt) * 100 / now();
////    STATE_PRINTN(setw(15)<<"Total cmd valid"<<" : "<<setw(10)<<total_task_cnt<<" | ");
////    STATE_PRINTN(setw(15)<<"Ratio"<<" : "<<setw(10)<<ratio<<" | "<<endl);
////    ratio = float(bp_cnt) * 100 / (bp_cnt + access_cnt);
////    STATE_PRINTN(setw(15)<<"DMC access"<<" : "<<setw(10)<<access_cnt<<" | ");
////    STATE_PRINTN(setw(15)<<"Command bp"<<" : "<<setw(10)<<bp_cnt<<" | ");
////    STATE_PRINTN(setw(15)<<"Bp ratio"<<" : "<<setw(10)<<ratio<<" | "<<endl);
////    ratio = float(total_bp_cnt) * 100 / (total_bp_cnt + total_access_cnt);
////    STATE_PRINTN(setw(15)<<"Total access"<<" : "<<setw(10)<<total_access_cnt<<" | ");
////    STATE_PRINTN(setw(15)<<"Total bp"<<" : "<<setw(10)<<total_bp_cnt<<" | ");
////    STATE_PRINTN(setw(15)<<"Total bp ratio"<<" : "<<setw(10)<<ratio<<" | "<<endl);
//
//    STATE_PRINTN("-------------------- RMW: Queue Statistics (Percentage/Cycle) --------------------------\n");
//    uint32_t total = 0;
//    size = rmw_que_cnt.size();
//
//    for (uint32_t index = 0; index <= size; index ++) {
//        STATE_PRINTN("--------");
//    }
//    STATE_PRINTN(endl);
//    STATE_PRINTN(setw(7)<<"Qnum"<<"|");
//    for (uint32_t index = 0; index < size; index++) {
//        total += rmw_que_cnt.at(index);
//        STATE_PRINTN(setw(7)<<index<<"|");
//    }
//    STATE_PRINTN(endl);
//    STATE_PRINTN(setw(7)<<"Per"<<"|");
//    for (uint32_t index = 0; index < size; index ++) {
//        float cnt_dist_ratio = (float(rmw_que_cnt.at(index)) * 100) / total;
//        STATE_PRINTN(setw(7)<<fixed<<setprecision(3)<<cnt_dist_ratio<<"|");
//    }
//    STATE_PRINTN(endl);
//    STATE_PRINTN(setw(7)<<"Cycle"<<"|");
//    for (uint32_t index = 0; index < size; index ++) {
//        STATE_PRINTN(setw(7)<<fixed<<setprecision(3)<<rmw_que_cnt.at(index)<<"|");
//    }
//    STATE_PRINTN(endl);
//    for (uint32_t index = 0; index <= size; index ++) {
//        STATE_PRINTN("--------");
//    }
//    STATE_PRINTN(endl);
//
//    for (uint32_t index = 0; index < size; index++) {
//        rmw_que_cnt.at(index) = 0;
//    }
//
////    task_cnt = 0;
////    access_cnt = 0;
////    bp_cnt = 0;
//
//    //save the value of last time stataes
//    pre_reads = reads;
//    pre_bypass_reads = bypass_reads;
//    pre_writes = writes;
//    pre_bypass_writes = bypass_writes;
//    pre_full_writes = full_writes;
//    pre_mask_writes = mask_writes;
//    pre_totals = totals;
////    pre_address_conf_cnt = address_conf_cnt;
////    pre_read_cnt = read_cnt;
////    pre_write_cnt = write_cnt;
////    pre_mwrite_cnt = mwrite_cnt;
//
//
//    //clear statistics
//    STATE_PRINTN("========================================= END =========================================\n");
//    STATE_PRINTN("\n");
//}

}
