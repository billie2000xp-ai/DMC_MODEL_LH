#include <iomanip>
#include <math.h>
#include "MemorySystem.h"
#include "IniReader.h"
#include "AddressMapping.h"
#include <assert.h>

using namespace std;

namespace LPDDRSim {

ofstream cmd_verify_out; //used in Rank.cpp and MemoryController.cpp if VERIFICATION_OUTPUT is set

//==============================================================================
MemorySystem::MemorySystem(unsigned dmcId,unsigned hhaId, ostream &DDRSim_log_,string LogPath) :
        ReturnReadData(NULL),
        WriteResp(NULL),
        ReadResp(NULL),
        CmdResp(NULL),
        dmc_id(dmcId),
        systemID(hhaId),
        log_path(LogPath) {
    channel = systemID * NUM_CHANS + dmc_id;
    channel_ohot = 1ull << channel;
    InitOutputFiles();
    write_map.clear();

    memoryController = new MemoryController(this, DDRSim_log, trace_log, cmdnum_log);
    ranks = new vector<Rank *>();

    for (size_t i=0; i<NUM_RANKS; i++) {
        Rank *r = new Rank(DDRSim_log, dram_log[i]);
        r->setId(channel * NUM_RANKS + i);
        r->attachMemoryController(memoryController);
        ranks->push_back(r);
    }

    memoryController->attachRanks(ranks);
#ifndef OLD_STATES_MODE
    start_cycle = 0;
    end_cycle = 0;
    flow_statis_start_cycle = 0;
    flow_statis_end_cycle = 0;

    curFlowPressureLevel = 0;

    task_cnt = 0;
    total_task_cnt = 0;
    enable_statistics = false;

    for (uint32_t index = 0; index < TRANS_QUEUE_DEPTH+1; index++) {
        que_cnt.push_back(0);
        que_rd_cnt.push_back(0);
        que_wr_cnt.push_back(0);
    }

    access_cnt = 0;
    bp_cnt = 0;
    total_access_cnt = 0;
    total_bp_cnt = 0;
    
    trans_fifo_data_cnt = 0;
    trans_fifo_full = false;

    occ_1_cnt = 0;
    occ_2_cnt = 0;
    occ_3_cnt = 0;
    occ_4_cnt = 0;

    pre_sch_level_cnt.clear(); pre_sch_level_cnt.resize(7);
    for (size_t i = 0; i < 7; i ++) {
        pre_sch_level_cnt.push_back(0);
    }

    pre_rmw_reads = 0; 
    pre_rmw_bypass_reads = 0; 
    pre_rmw_bypass_writes = 0; 
    pre_rmw_writes = 0; 
    pre_rmw_full_writes = 0; 
    pre_rmw_mask_writes = 0; 
    pre_rmw_totals = 0;
    
    pre_merge_read_cnt = 0;
    pre_ecc_read_cnt = 0;
    pre_ecc_write_cnt = 0;
    pre_reads = 0;
    pre_writes = 0;
    pre_totals = 0;
    pre_gbuf_reads = 0;
    pre_gbuf_writes = 0;
    pre_address_conf_cnt  = 0;
    pre_id_conf_cnt = 0;
    pre_ba_conf_cnt = 0;
    pre_total_conf = 0;
    pre_act_cnt = 0;
    pre_act_dst_cnt = 0;
    pre_pre_sb_cnt = 0;
    pre_pre_pb_cnt = 0;
    pre_pre_pb_dst_cnt = 0;
    pre_pre_ab_cnt = 0;
    pre_read_cnt = 0;
    pre_write_cnt = 0;
    pre_read_p_cnt = 0;
    pre_write_p_cnt = 0;
    pre_mwrite_cnt = 0;
    pre_mwrite_p_cnt = 0;
    pre_timeout_cnt = 0;
    pre_rt_timeout_cnt = 0;
    pre_row_hit_cnt = 0;
    pre_row_miss_cnt = 0;
    pre_rw_switch_cnt = 0;
    pre_rank_switch_cnt = 0;
    pre_sid_switch_cnt = 0;
    pre_refresh_pb_cnt = 0;
    pre_refresh_ab_cnt = 0;
    pre_r2w_switch_cnt = 0;
    pre_w2r_switch_cnt = 0;
    pre_phy_notlp_cnt = 0;
    pre_phy_lp_cnt = 0;
    pre_power = 0;
    pre_pde_cnt = 0;
    pre_asrefe_cnt = 0;
    pre_srpde_cnt = 0;
    pre_pdx_cnt = 0;
    pre_asrefx_cnt = 0;
    pre_srpdx_cnt = 0;
    PreRdCntBl.clear();   PreWrCntBl.clear();
    PreRdCntBl[BL8] = 0;  PreWrCntBl[BL8] = 0;
    PreRdCntBl[BL16] = 0; PreWrCntBl[BL16] = 0;
    PreRdCntBl[BL24] = 0; PreWrCntBl[BL24] = 0;
    PreRdCntBl[BL32] = 0; PreWrCntBl[BL32] = 0;
    PreRdCntBl[BL48] = 0; PreWrCntBl[BL48] = 0;
    PreRdCntBl[BL64] = 0; PreWrCntBl[BL64] = 0;

    Total_func_pre_cnt = 0;

    rd_one = true;
    rd_two = false;
    rd_three = false;

    wr_one = true;
    wr_two = false;
    wr_three = false;

    pre_total_latency = 0;
    pre_com_read_cnt = 0;
#endif
    pre_acc_rank_cnt.clear();
    pre_acc_rank_cnt.reserve(NUM_RANKS);
    pre_acc_bank_cnt.clear();
    pre_acc_bank_cnt.reserve(NUM_BANKS * NUM_RANKS);
    pre_racc_rank_cnt.clear();
    pre_racc_rank_cnt.reserve(NUM_RANKS);
    pre_racc_bank_cnt.clear();
    pre_racc_bank_cnt.reserve(NUM_BANKS * NUM_RANKS);
    pre_wacc_rank_cnt.clear();
    pre_wacc_rank_cnt.reserve(NUM_RANKS);
    pre_wacc_bank_cnt.clear();
    pre_wacc_bank_cnt.reserve(NUM_BANKS * NUM_RANKS);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        prePdTime.push_back(0);
        preAsrefTime.push_back(0);
        preSrpdTime.push_back(0);
        preWakeUpTime.push_back(0);
        pre_acc_rank_cnt.push_back(0);
        pre_racc_rank_cnt.push_back(0);
        pre_wacc_rank_cnt.push_back(0);
        for (size_t j = 0; j < NUM_BANKS; j ++) {
            pre_acc_bank_cnt.push_back(0);
            pre_racc_bank_cnt.push_back(0);
            pre_wacc_bank_cnt.push_back(0);
        }
    }

    PreBankRowActCnt.clear();
    PreBankRowActCnt.reserve(NUM_BANKS * NUM_RANKS);
    for (size_t i = 0; i < NUM_BANKS * NUM_RANKS; i ++) {
        PreBankRowActCnt.push_back(0);
    }
    PreDmcPipeQueue.reserve(tPIPE_PRE_DMC);

    pre_abr_cnt.clear();
//    for (size_t i = 0; i < NUM_RANKS; i ++) {
//        pre_abr_cnt.push_back(0);
//    }
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        pre_abr_cnt.push_back(vector<unsigned>());
        for (size_t j = 0; j < memoryController->sc_num; j++){
            pre_abr_cnt[i].push_back(0);
        }
    }
    

    pre_pbr_cnt.clear();     //todo: revise for e-mode
    if (ENH_PBR_EN) {
        for (size_t i = 0; i < NUM_RANKS * NUM_BANKS; i ++) {
            pre_pbr_cnt.push_back(0);
        }
    } else {
        for (size_t i = 0; i < NUM_RANKS * memoryController->pbr_bank_num * memoryController->sc_num; i ++) {
            pre_pbr_cnt.push_back(0);
        }
    }
}

//==============================================================================
void MemorySystem::InitOutputFiles() {
    if ((DEBUG_BUS || DEBUG_STATE || DEBUG_GBUF_STATE || DEBUG_RMW_STATE) && (channel_ohot == (channel_ohot & PRINT_CH_OHOT))) {
        dmc_log = log_path + "/lpddr_sim" + std::to_string(channel) + ".log";
        DDRSim_log.open(dmc_log.c_str(),ios_base::out | ios_base::trunc);
        if (!DDRSim_log) {
            ERROR("Cannot open "<<dmc_log);
            assert(0);
        }
    }

    if (STATE_LOG) {
        string st_log = log_path + "/lpddr_state" + std::to_string(channel) + ".log";
        state_log.open(st_log.c_str(),ios_base::out | ios_base::trunc);
        if (!state_log) {
             ERROR("Cannot open "<<st_log);
             assert(0);
        }
    }

    if ((PRINT_TRACE || PRINT_UT_TRACE || PRINT_READ || PRINT_RDATA || PRINT_BW) &&
            (channel_ohot == (channel_ohot & PRINT_CH_OHOT))) {
        string t_log = log_path + "/lpddr_trace_" + std::to_string(channel) + ".log";
        trace_log.open(t_log.c_str(),ios_base::out | ios_base::trunc);
        if (!trace_log) {
             ERROR("Cannot open "<<t_log);
             assert(0);
        }
        if (PRINT_UT_TRACE) {
            TRACE_PRINT("type,address,trans_size,burst_len,mid,delay(ns),ATIME,ch_num,qos,gid"<<endl);
        } else {
            TRACE_PRINT("Message: DMC_RATE="<<DMC_RATE<<", tDFI="<<tDFI<<", DMC_DATA_BUS_BITS="
                    <<DMC_DATA_BUS_BITS<<", JEDEC_DATA_BUS_BITS="<<JEDEC_DATA_BUS_BITS<<endl);
        }
    }

    if (PRINT_CMD_NUM && (channel_ohot == (channel_ohot & PRINT_CH_OHOT))) {
        string c_log = log_path + "/lpddr_cmdnum_" + std::to_string(channel) + ".log";
        cmdnum_log.open(c_log.c_str(),ios_base::out | ios_base::trunc);
        if (!cmdnum_log) {
             ERROR("Cannot open "<<c_log);
             assert(0);
        }
    }

    dram_log.resize(NUM_RANKS);
    if (PRINT_DRAM_TRACE && (channel_ohot == (channel_ohot & PRINT_CH_OHOT))) {
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            string d_log = log_path + "/lpddr_dram_" + std::to_string(channel) +
                "_" + std::to_string(rank) + ".log";
            dram_log[rank].open(d_log.c_str(),ios_base::out | ios_base::trunc);
            if (!dram_log[rank]) {
                 ERROR("Cannot open "<<d_log);
                 assert(0);
            }
        }
    }
}
//==============================================================================
void MemorySystem::update_print() {
    static int log_num = 0;
    unsigned max_file_size = 100000000LL;
    //file.close();
    string cur_dmc_log = dmc_log;
    std::ifstream in(cur_dmc_log.c_str() , ios::binary|ios::ate);
    unsigned file_size = in.tellg();
    in.close();
    if (file_size > max_file_size) {
        DDRSim_log.close();
        log_num = (log_num + 1) % 2;
        cur_dmc_log = dmc_log + std::to_string(log_num);
        DDRSim_log.open(cur_dmc_log.c_str() ,ios::out);
    }
}
//==============================================================================
MemorySystem::~MemorySystem() {
    if (DRAM_POWER_EN) {
        for (size_t i = 0; i < NUM_RANKS; i ++) {
            (*ranks).at(i)->CalcDramPower();
        }
    }
    register_write(4,0);
    register_write(0,0);

    delete(memoryController);

    for (size_t i = 0; i < NUM_RANKS; i ++) {
        delete (*ranks).at(i);
    }
    ranks->clear();
    delete(ranks);
}
//==============================================================================
bool MemorySystem::WillAcceptTransaction() {
    return memoryController->WillAcceptTransaction();
}
//==============================================================================
void MemorySystem::noc_read_inform(bool fast_wakeup_rank0, bool fast_wakeup_rank1, bool bus_rempty) {
    bus_state BusSt;
    BusSt.fast_wakeup_rank0 = fast_wakeup_rank0;
    BusSt.fast_wakeup_rank1 = fast_wakeup_rank1;
    BusSt.bus_rempty = bus_rempty;
    BusSt.valid_time = now() + 2;
    BusStateAsync.push_back(BusSt);
    if (DEBUG_BUS) {
        if (fast_wakeup_rank0 || fast_wakeup_rank1) {
            PRINTN(setw(10)<<now()<<" -- Fast Wakeup, rank0="<<fast_wakeup_rank0<<", rank1="<<fast_wakeup_rank1<<endl);
        }
    }
}
//==============================================================================
bool MemorySystem::addTransaction(Transaction *trans) {
    if (memoryController->pre_req_time == memoryController->now() 
            || (WRITE_BUFFER_ENABLE && memoryController->wb->pre_req_time == memoryController->wb->now()) 
            || (RMW_ENABLE && memoryController->rmw->pre_req_time == memoryController->rmw->now())) {
        return false;
    }
    bool ret = false;
//    Transaction *trans = new Transaction(command);
    trans_init(trans, now());

    write_msg msg;
    msg.pt = DMC_PATH;
    msg.num_256bit = trans->burst_length + 1;

//    addressMapping(*trans);
//    trans_check(trans);

    if (tPIPE_PRE_DMC == 0) {
        ret = memoryController->push_req(trans);
    } else {
        if (PreDmcPipeQueue.size() >= tPIPE_PRE_DMC) {
            ret = false;
        } else {
            trans->async_delay_time = now() + tPIPE_PRE_DMC;
            PreDmcPipeQueue.push_back(trans);
            ret = true;
        }
    }
    if (ret) {
        access_cnt ++;
        total_access_cnt ++;
        if (trans->transactionType != DATA_READ) {
            write_map[trans->task] = msg;
        }
    } else {
        //delete trans;
        bp_cnt ++;
        total_bp_cnt ++;
    }
    task_cnt ++;
    total_task_cnt ++;

    if (ret && !trans->pre_act) {
        if (trans->transactionType == DATA_READ) {
            if (PRINT_READ || PRINT_TRACE) {
                TRACE_PRINT(setw(10)<<now()<<" -- ADD :: [R]B["<<trans->burst_length<<"]QOS["<<trans->qos<<"]MID["
                        <<trans->mid<<"] addr=0x"<<hex<<trans->address<<" task="<<dec<<trans->task<<" rank="<<trans->rank
                        <<" sid="<<trans->sid<<" group="<<trans->group<<" bank="<<trans->bankIndex<<" row="<<trans->row
                        <<" col="<<trans->col<<" data_size="<<trans->data_size<<" timeAdded="<<trans->timeAdded
                        <<" timeout_th="<<trans->timeout_th<<endl);
            }
            if (PRINT_IDLE_LAT) {
                DEBUG(setw(10)<<now()<<" -- ADD :: [R]B["<<trans->burst_length<<"]QOS["<<trans->qos<<"] addr=0x"
                        <<hex<<trans->address<<" task="<<dec<<trans->task<<" rank="<<trans->rank<<" sid="<<trans->sid
                        <<" group="<<trans->group<<" bank="<<trans->bankIndex<<" row="<<trans->row<<" col="<<trans->col
                        <<" data_size="<<trans->data_size<<" timeAdded="<<trans->timeAdded);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- ADD :: [R]B["<<trans->burst_length<<"]QOS["<<trans->qos<<"]MID["
                        <<trans->mid<<"] addr=0x"<<hex<<trans->address<<" task="<<dec<<trans->task<<" rank="<<trans->rank
                        <<" sid="<<trans->sid<<" group="<<trans->group<<" bank="<<trans->bankIndex<<" row="<<trans->row
                        <<" col="<<trans->col<<" data_size="<<trans->data_size<<" timeAdded="<<trans->timeAdded
                        <<" timeout_th="<<trans->timeout_th<<endl);
            }
        } else {
            if (PRINT_TRACE) {
                TRACE_PRINT(setw(10)<<now()<<" -- ADD :: [W]B["<<trans->burst_length<<"]QOS["<<trans->qos<<"]MID["
                        <<trans->mid<<"] addr=0x"<<hex<<trans->address<<" task="<<dec<<trans->task<<" rank="<<trans->rank
                        <<" sid="<<trans->sid<<" group="<<trans->group<<" bank="<<trans->bankIndex<<" row="<<trans->row
                        <<" col="<<trans->col<<" data_size="<<trans->data_size<<" timeAdded="<<trans->timeAdded
                        <<" timeout_th="<<trans->timeout_th<<endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- ADD :: [W]B["<<trans->burst_length<<"]QOS["<<trans->qos<<"]MID["
                        <<trans->mid<<"] addr=0x"<<hex<<trans->address<<" task="<<dec<<trans->task<<" rank="<<trans->rank
                        <<" sid="<<trans->sid<<" group="<<trans->group<<" bank="<<trans->bankIndex<<" row="<<trans->row
                        <<" col="<<trans->col<<" data_size="<<trans->data_size<<" timeAdded="<<trans->timeAdded
                        <<" timeout_th="<<trans->timeout_th<<endl);
            }
        }
        if (PRINT_UT_TRACE) {
            string ut_type;
            unsigned ut_mid = 0, ut_gid = 0;
            static uint64_t ut_pretime = 0;
            double ut_interval = double(now() - ut_pretime) * tDFI;
            double ut_time = double(now()) * tDFI;
            unsigned ut_qos = 7 - trans->qos;
            if (trans->transactionType == DATA_READ) ut_type = "nr";
            else ut_type = "nw";
            TRACE_PRINT(fixed<<ut_type<<" "<<hex<<trans->address<<dec<<" "<<trans->data_size<<" "
                    <<trans->burst_length<<" "<<hex<<ut_mid<<dec<<setprecision(3)<<" "<<ut_interval
                    <<" "<<ut_time<<" "<<channel<<" "<<ut_qos<<" "<<ut_gid<<endl);
            ut_pretime = now();
        }
    } else {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- DROP :: addr=0x"<<hex<<trans->address<<" task="<<dec<<trans->task
                    <<" QR="<<memoryController->Read_Cnt()<<" QW="<<memoryController->Write_Cnt()<<endl);
        }
    }
    if (PERFECT_DMC_EN) delete trans;
    return ret;
}
//==============================================================================
uint32_t MemorySystem::GetTransactionLen() {
    return (memoryController->GetDmcQsize());
}
//==============================================================================
//    below interface for dmc write data
//==============================================================================
bool MemorySystem::addData(uint32_t *data ,uint64_t taskId, bool ecc_flag) {
    
//    // one cycle one data
//    if (memoryController->pre_req_data_time == memoryController->now()) {
//        return false;
//    }
    if (ecc_flag) {
        // one cycle one data check
        if (memoryController->pre_req_data_time == memoryController->now() ) {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- ECC_WDATA :: BP ECC_Wdata, one data one cycle :: task="<<taskId
                            <<" last_wdata_time="<<memoryController->pre_req_data_time<<endl);
            }
            return false;
        }
    } else {    
        if (!trans_fifo_full && TRANS_FIFO_DEPTH!=0) {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- TRANS FIFO is NOT FULL :: task="<<taskId<<" data_cnt="
                        <<trans_fifo_data_cnt<<" last_wdata_time="<<memoryController->pre_req_data_time<<endl);
            }
            
            // one cycle one data check
            if (memoryController->pre_req_data_time == memoryController->now() || (RMW_ENABLE && memoryController->rmw->pre_req_data_time == memoryController->rmw->now()) ) {
                return false;
            }
            if (RMW_ENABLE) {
                unsigned size = memoryController->rmw->RmwQue.size(); 
                for (unsigned i =0; i < size; i++) {
                    if ((taskId==memoryController->rmw->RmwQue[i]->task)&&(memoryController->rmw->RmwQue[i]->transactionType==DATA_WRITE)
                         &&(!memoryController->rmw->RmwQue[i]->mask_wcmd)&&(RMW_CMD_MODE==0)&&(!memoryController->rmw->RmwQue[i]->ecc_flag)) {
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- WDATA :: RMW BP Wdata, FULL Wcmd must be send first to DMC under fast cmd mode, task="<<taskId<<endl);
                        }
                        return false; 
                    }
                }
            } 
            if ((memoryController->pre_req_data_time + 1) == memoryController->now()) { // sequential wdata
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- SEQ WDATA TO TRANS UNDER NOT FULL :: task="<<taskId<<" last_wdata_time="<<memoryController->pre_req_data_time<<endl);
                }
                trans_fifo_data_cnt ++;
                if (trans_fifo_data_cnt == TRANS_FIFO_DEPTH) {
                    trans_fifo_full = true;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- TRANS FIFO CHANGE TO FULL :: task="<<taskId<<" data_cnt="
                                <<trans_fifo_data_cnt<<" last_wdata_time="<<memoryController->pre_req_data_time<<endl);
                    }
                }
            }
        } else {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- TRANS FIFO is FULL :: task="<<taskId<<" data_cnt="
                        <<trans_fifo_data_cnt<<" last_wdata_time="<<memoryController->pre_req_data_time<<endl);
            }

            // two cycle one data when trans fifo full
            if (memoryController->pre_req_data_time == memoryController->now() || ((memoryController->pre_req_data_time + 1) == memoryController->now())) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- WDATA :: BP Wdata, one data two cyle under full trans fifo :: task="<<taskId
                                <<" last_wdata_time="<<memoryController->pre_req_data_time<<endl);
                }
                return false;
            }
        }
    }
    
    if (DROP_WRITE_CMD || PERFECT_DMC_EN) return true;


    auto it = write_map.find(taskId);

    if (it == write_map.end()) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- WDATA :: BP Wdata, Wcmd must be send first, task="<<taskId<<endl);
        }
        return false;
    } else {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- WDATA :: Add Wdata, task="<<taskId<<endl);
        }
    }

    if (WRITE_BUFFER_ENABLE) {
        memoryController->wb->addData(data ,taskId);
    } else if (RMW_ENABLE) {
        memoryController->rmw->addData(data ,taskId);
    } else {
        memoryController->receiveFromCPU(data ,taskId);
    }
    if (it->second.num_256bit == 0) {
        ERROR(setw(10)<<now()<<" -- ERROR, burst number mismatch, ID="
                <<taskId<<", chnl: "<<channel);
    } else {
        it->second.num_256bit --;
    }
    if (it->second.num_256bit == 0) {
        write_map.erase(taskId);
    }

    return true;
}
//==============================================================================
//update the memory systems state
//==============================================================================
void MemorySystem::update() {
    //updates the state of each of the objects
    // NOTE - do not change order

    //trans_fifo_data_cnt update
    if (memoryController->pre_req_data_time != now()) {
        if (trans_fifo_data_cnt > 0) {
            trans_fifo_data_cnt --;
            if (trans_fifo_data_cnt < TRANS_FIFO_DEPTH) {
                trans_fifo_full = false;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- TRANS FIFO CHANGE TO NOT FULL :: data_cnt="
                            <<trans_fifo_data_cnt<<" last_wdata_time="<<memoryController->pre_req_data_time<<endl);
                }
            }
        }
    }

    uint8_t bus_ptr = 0;
    for (auto bus : BusStateAsync) {
        if (now() >= bus.valid_time) {
            memoryController->noc_read_inform(bus.fast_wakeup_rank0,
                    bus.fast_wakeup_rank1, bus.bus_rempty);
            bus_ptr ++;
        } else {
            break;
        }
    }
    if (bus_ptr > 0) {
        BusStateAsync.erase(BusStateAsync.begin(), BusStateAsync.begin() + bus_ptr);
    }

    for (size_t i = 0; i < NUM_RANKS; i ++) {
        (*ranks).at(i)->update();
    }

    if (IECC_ENABLE) {
        memoryController->iecc->update();
    }
    if (WRITE_BUFFER_ENABLE) {
        memoryController->wb->update();
    }
    if (RMW_ENABLE) {
        memoryController->rmw->update();
    }
    memoryController->update();

    if (IECC_ENABLE) {
        memoryController->iecc->step();
    }
    if (WRITE_BUFFER_ENABLE) {
        memoryController->wb->step();
    }
    if (RMW_ENABLE) {
        memoryController->rmw->step();
    }
    memoryController->step();

    for (size_t i=0;i<NUM_RANKS;i++) {
        (*ranks).at(i)->step();
    }

    this->step();

    if (STATE_TIME != 0) {
        if ((now() % STATE_TIME) == 0) {
            register_write(4,0);
            register_write(0,0);
        }
    }
    memoryController->rmw->check_cnt();
    check_cnt();

    if (FLOW_STAT_TIME != 0) {
        if ((now() % FLOW_STAT_TIME) == 0) {
            float total_bw = flowStatistic();
            curFlowPressureLevel = updateFlowState(total_bw);
        }
    }

    if (!PreDmcPipeQueue.empty()) {
        auto trans = PreDmcPipeQueue[0];
        if (now() >= trans->async_delay_time && memoryController->push_req(trans)) {
            PreDmcPipeQueue.erase(PreDmcPipeQueue.begin());
        }
    }
}
//==============================================================================
uint32_t MemorySystem::getFlowPressureLevel() {
    return curFlowPressureLevel;
}
//==============================================================================
uint32_t MemorySystem::updateFlowState(float total_bw) {
    if (DMC_THEORY_BW <= 0.0) return 0;
    if (total_bw < DMC_THEORY_BW*0.375) {
        return 0;
    } else if (total_bw < DMC_THEORY_BW*0.625) {
        return 1;
    } else if (total_bw < DMC_THEORY_BW*0.75) {
        return 2;
    } else {
        return 3;
    }

}
//==============================================================================
void MemorySystem::communicate() {
    memoryController->communicate();
}
//==============================================================================
void MemorySystem::RegisterCallbacks(Callback_t* readData, Callback_t* writeCB , Callback_t *readCB, Callback_t *cmdCB) {
    ReturnReadData = readData;
    WriteResp = writeCB;
    ReadResp = readCB;
    CmdResp = cmdCB;
}
//==============================================================================
void MemorySystem ::dfs_backpress(bool backpress) {
    memoryController->dfs_backpress(backpress);
}

//==============================================================================
uint8_t MemorySystem::get_occ() {
    return memoryController->occ;
}
//==============================================================================
float MemorySystem::flowStatistic() {
    flow_statis_end_cycle = now();
    if (flow_statis_end_cycle == flow_statis_start_cycle) return 0.0;

    uint64_t bytes = memoryController->flowStatisTotalBytes;
    float total_bw = (float(bytes)) / ((flow_statis_end_cycle - flow_statis_start_cycle)* tDFI);

    flow_statis_start_cycle = now();
    memoryController->flowStatisTotalBytes = 0;
    return total_bw;
}
//==============================================================================
void MemorySystem::statistics() {
    unsigned size = 0;
    STATE_PRINTN(setiosflags(ios::left));
    STATE_PRINTN("======================================== START ========================================\n");
    STATE_PRINTN("-------------------- Base Message -----------------------------------------------------\n");
    STATE_PRINTN(DDR_TYPE<<" "<<DMC_RATE<<"Mbps, x"<<JEDEC_DATA_BUS_BITS<<", DMC Data Width: "
            <<DMC_DATA_BUS_BITS<<", CKR: "<<setprecision(1)<<WCK2DFI_RATIO<<endl);
    uint64_t bytes = memoryController->TotalBytes - memoryController->ecc_total_bytes - memoryController->rmw_total_bytes;
    STATE_PRINTN("Current time: "<<fixed<<now()<<", tDFI: "<<setprecision(4)<<tDFI<<" ns, DMC Total bytes: ");
    STATE_PRINTN(memoryController->DmcTotalBytes<<", DDR Total bytes: "<<bytes<<endl);
    STATE_PRINTN("-------------------- DFI Performance Statistics (GB/s) --------------------------------\n");
    if (end_cycle == start_cycle) return;
    bytes = memoryController->TotalReadBytes - memoryController->ecc_total_reads - memoryController->rmw_total_reads;
    float data_read_bw = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);
    bytes = memoryController->TotalWriteBytes - memoryController->ecc_total_writes;
    float data_write_bw = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);
    bytes = memoryController->TotalBytes - memoryController->ecc_total_bytes - memoryController->rmw_total_bytes;
    float data_total_bw = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);

    bytes = memoryController->ecc_total_bytes;
    float ecc_total_bw = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);
    bytes = memoryController->ecc_total_writes;
    float ecc_write_bw = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);
    bytes = memoryController->ecc_total_reads;
    float ecc_read_bw = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);

    bytes = memoryController->rmw_total_bytes;
    float rmw_total_bw = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);
    bytes = memoryController->rmw_total_reads;
    float rmw_read_bw = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);

    bytes = memoryController->TotalReadBytes;
    float read_bw = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);
    bytes = memoryController->TotalWriteBytes;
    float write_bw = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);
    bytes = memoryController->TotalBytes;
    float total_bw = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);

    STATE_PRINTN("Data Read bandwidth   : "<<setprecision(2)<<setw(6)<<data_read_bw);
    STATE_PRINTN(" | Data Write bandwidth   : "<<setprecision(2)<<setw(6)<<data_write_bw);
    STATE_PRINTN(" | Data Total bandwidth  : "<<setprecision(2)<<data_total_bw<<endl);

    STATE_PRINTN("ECC Read bandwidth    : "<<setprecision(2)<<setw(6)<<ecc_read_bw);
    STATE_PRINTN(" | ECC Write bandwidth    : "<<setprecision(2)<<setw(6)<<ecc_write_bw);
    STATE_PRINTN(" | ECC Total bandwidth   : "<<setprecision(2)<<ecc_total_bw<<endl);

    STATE_PRINTN("RMW Read bandwidth    : "<<setprecision(2)<<setw(6)<<rmw_read_bw);
    STATE_PRINTN(" | RMW Total bandwidth    : "<<setprecision(2)<<rmw_total_bw<<endl);

    STATE_PRINTN("Total Read bandwidth  : "<<setprecision(2)<<setw(6)<<read_bw);
    STATE_PRINTN(" | Total Write bandwidth  : "<<setprecision(2)<<setw(6)<<write_bw);
    STATE_PRINTN(" | Total bandwidth       : "<<setprecision(2)<<total_bw<<endl);

    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        bytes = memoryController->TotalBytesRank[rank];
        float total_bw_rank = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);
        STATE_PRINTN("Total Rank"<<rank<<" bandwidth : "<<fixed<<setw(6)<<total_bw_rank<<" | ");
        float rank_avail = 0;
        if (IS_G3D){
            rank_avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                    (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO / float(NUM_GROUPS);
        } else {
            rank_avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                    (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO;
        }
        if (IS_LP6) rank_avail = rank_avail * 9 / 8;
        STATE_PRINTN("Total Rank"<<rank<<" efficiency : "<<setprecision(2)<<rank_avail<<"%"<<endl);
    }

    bytes = memoryController->TotalBytes - memoryController->ecc_total_bytes - memoryController->rmw_total_bytes;
    float data_avail = 0;
    if (IS_G3D){
        data_avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO / float(NUM_GROUPS);
    } else {
        data_avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO;
    }
    bytes = memoryController->ecc_total_bytes;
    float ecc_avail = 0;
    if (IS_G3D){
        ecc_avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO / float(NUM_GROUPS);
    } else {
        ecc_avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO;
    }
    bytes = memoryController->rmw_total_bytes;
    float rmw_avail = 0;
    if (IS_G3D){
        rmw_avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO / float(NUM_GROUPS);
    } else {
        rmw_avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO;
    }
    bytes = memoryController->TotalBytes;
    float avail = 0;
    if (IS_G3D){
        avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO / float(NUM_GROUPS);
    } else {
        avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO;
    }
    if (IS_LP6) data_avail = data_avail * 9 / 8;
    if (IS_LP6) ecc_avail = ecc_avail * 9 / 8;
    if (IS_LP6) rmw_avail = rmw_avail * 9 / 8;
    if (IS_LP6) avail = avail * 9 / 8;
    STATE_PRINTN("Data efficiency       : "<<setprecision(2)<<setw(5)<<data_avail<<"%");
    STATE_PRINTN(" | ECC efficiency         : "<<setprecision(2)<<setw(5)<<ecc_avail<<"%");
    STATE_PRINTN(" | RMW efficiency         : "<<setprecision(2)<<setw(5)<<rmw_avail<<"%"<<endl);
    STATE_PRINTN("DFI Total efficiency  : "<<setprecision(2)<<setw(5)<<avail<<"%"<<endl);

    STATE_PRINTN("-------------------- DMC Performance Statistics (GB/s) --------------------------------\n");
    bytes = memoryController->DmcTotalReadBytes;
    avail = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);
    STATE_PRINTN("Dmc Read bandwidth    : "<<setprecision(2)<<setw(6)<<avail);
    bytes = memoryController->DmcTotalWriteBytes;
    avail = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);
    STATE_PRINTN(" | Dmc Write bandwidth    : "<<setprecision(2)<<setw(6)<<avail);
    bytes = memoryController->DmcTotalBytes;
    avail = (float(bytes)) / ((end_cycle - start_cycle) * tDFI);
    STATE_PRINTN(" | Dmc Total bandwidth   : "<<setprecision(2)<<setw(6)<<avail<<endl);
    bytes = memoryController->DmcTotalReadBytes;
    if (IS_G3D){
        avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO / float(NUM_GROUPS);
    } else {
        avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO;
    }
    if (IS_LP6) avail = avail * 9 / 8;
    STATE_PRINTN("Dmc Read efficiency   : "<<setprecision(2)<<setw(5)<<avail<<"%");
    bytes = memoryController->DmcTotalWriteBytes;
    if (IS_G3D){
        avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO / float(NUM_GROUPS);
    } else {
        avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO;
    }
    if (IS_LP6) avail = avail * 9 / 8;
    STATE_PRINTN(" | Dmc Write efficiency   : "<<setprecision(2)<<setw(5)<<avail<<"%");
    bytes = memoryController->DmcTotalBytes;
    if (IS_G3D){
        avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO / float(NUM_GROUPS);
    } else {
        avail = (float (bytes * 8 / (2 * JEDEC_DATA_BUS_BITS))) * 100 /
                (end_cycle - start_cycle) / WCK2DFI_RATIO / PAM_RATIO;
    }
    if (IS_LP6) avail = avail * 9 / 8;
    STATE_PRINTN(" | Dmc Total efficiency  : "<<setprecision(2)<<setw(5)<<avail<<"%"<<endl);

    STATE_PRINTN("-------------------- Rank LP Time Statistics (DFI Clock) ------------------------------\n");
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN("Rank"<<i<<" Current PdTime      : "<<setw(12)
                <<(memoryController->PdTime[i] - prePdTime[i])<<" | ");
    }
    STATE_PRINTN(endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN("Rank"<<i<<" Current AsrefTime   : "<<setw(12)
                <<(memoryController->AsrefTime[i] - preAsrefTime[i])<<" | ");
    }
    STATE_PRINTN(endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN("Rank"<<i<<" Current SrpdTime    : "<<setw(12)
                <<(memoryController->SrpdTime[i] - preSrpdTime[i])<<" | ");
    }
    STATE_PRINTN(endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN("Rank"<<i<<" Current WakeUpTime  : "<<setw(12)
                <<(memoryController->WakeUpTime[i] - preWakeUpTime[i])<<" | ");
    }
    STATE_PRINTN(endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN("Rank"<<i<<" Total PdTime        : "<<setw(12)
                <<memoryController->PdTime[i]<<" | ");
    }
    STATE_PRINTN(endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN("Rank"<<i<<" Total AsrefTime     : "<<setw(12)
                <<memoryController->AsrefTime[i]<<" | ");
    }
    STATE_PRINTN(endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN("Rank"<<i<<" Total SrpdTime      : "<<setw(12)
                <<memoryController->SrpdTime[i]<<" | ");
    }
    STATE_PRINTN(endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN("Rank"<<i<<" Total WakeUpTime    : "<<setw(12)
                <<memoryController->WakeUpTime[i]<<" | ");
    }
    STATE_PRINTN(endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN("Rank"<<i<<" Total PdEnterCnt    : "<<setw(12)
                <<memoryController->PdEnterCnt[i]<<" | ");
    }
    STATE_PRINTN(endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN("Rank"<<i<<" Total PdExitCnt     : "<<setw(12)
                <<memoryController->PdExitCnt[i]<<" | ");
    }
    STATE_PRINTN(endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN("Rank"<<i<<" Total AsrefEnterCnt : "<<setw(12)
                <<memoryController->AsrefEnterCnt[i]<<" | ");
    }
    STATE_PRINTN(endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN("Rank"<<i<<" Total AsrefExitCnt  : "<<setw(12)
                <<memoryController->AsrefExitCnt[i]<<" | ");
    }
    STATE_PRINTN(endl);
    STATE_PRINTN("Phy LP Cnt(Close Clock)   : "<<setw(12)<<memoryController->phy_lp_cnt<<" |"<<endl);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        prePdTime[i] = memoryController->PdTime[i];
        preAsrefTime[i] = memoryController->AsrefTime[i];
        preSrpdTime[i] = memoryController->SrpdTime[i];
        preWakeUpTime[i] = memoryController->WakeUpTime[i];
    }
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        uint64_t page_idle_cnt = memoryController->bank_idle_cnt[rank];
        uint64_t page_act_cnt = memoryController->bank_act_cnt[rank];
        STATE_PRINTN("Rank"<<rank<<" Total Idle ratio    : "<<setw(12)
                <<((long double)page_idle_cnt * 100 / now() / NUM_BANKS)<<" | ");
        STATE_PRINTN("Rank"<<rank<<" Total act ratio     : "<<setw(12)
                <<((long double)page_act_cnt * 100 / now() / NUM_BANKS)<<" | "<<endl);
    }
    memoryController->TotalBytes = 0;
    memoryController->TotalReadBytes = 0;
    memoryController->TotalWriteBytes = 0;
    memoryController->ecc_total_bytes = 0;
    memoryController->ecc_total_reads = 0;
    memoryController->ecc_total_writes = 0;
    memoryController->DmcTotalBytes = 0;
    memoryController->DmcTotalReadBytes = 0;
    memoryController->DmcTotalWriteBytes = 0;
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        memoryController->TotalBytesRank[rank] = 0;
    }

    unsigned reads = memoryController->totalReads;
    unsigned writes = memoryController->totalWrites;
    unsigned totals = memoryController->totalTransactions;
    unsigned address_conf_cnt = memoryController->addrconf_cnt;
    unsigned id_conf_cnt = memoryController->idconf_cnt;
    unsigned ba_conf_cnt = memoryController->baconf_cnt;
    unsigned total_conf = memoryController->totalconf_cnt;
    unsigned act_cnt = memoryController->active_cnt;
    unsigned act_dst_cnt = memoryController->active_dst_cnt;
    unsigned pre_sb_cnt = memoryController->precharge_sb_cnt;
    unsigned pre_pb_cnt = memoryController->precharge_pb_cnt;
    unsigned pre_pb_dst_cnt = memoryController->precharge_pb_dst_cnt;
    unsigned pre_ab_cnt = memoryController->precharge_ab_cnt;
    unsigned read_cnt = memoryController->read_cnt;
    unsigned write_cnt = memoryController->write_cnt;
    unsigned read_p_cnt = memoryController->read_p_cnt;
    unsigned write_p_cnt = memoryController->write_p_cnt;
    unsigned mwrite_cnt = memoryController->mwrite_cnt;
    unsigned mwrite_p_cnt = memoryController->mwrite_p_cnt;
    unsigned timeout_cnt = memoryController->dmc_timeout_cnt;
    unsigned rt_timeout_cnt = memoryController->RtCmdCnt;
    unsigned row_hit_cnt = 0;
    unsigned row_miss_cnt = 0;
    unsigned rw_switch_cnt = memoryController->rw_switch_cnt;
    unsigned rank_switch_cnt = memoryController->rank_switch_cnt;
    unsigned sid_switch_cnt = memoryController->sid_switch_cnt;
    unsigned refresh_pb_cnt = memoryController->refresh_pb_cnt;
    unsigned refresh_ab_cnt = memoryController->refresh_ab_cnt;
    unsigned r2w_switch_cnt = memoryController->r2w_switch_cnt;
    unsigned w2r_switch_cnt = memoryController->w2r_switch_cnt;
    unsigned phy_notlp_cnt = memoryController->phy_notlp_cnt;
    unsigned phy_lp_cnt = memoryController->phy_lp_cnt;
    unsigned ecc_read_cnt = memoryController->ecc_read_cnt;
    unsigned ecc_write_cnt = memoryController->ecc_write_cnt;
    unsigned merge_read_cnt = memoryController->merge_read_cnt;
    unsigned pde_cnt = 0, asrefe_cnt = 0, srpde_cnt = 0;
    unsigned pdx_cnt = 0, asrefx_cnt = 0, srpdx_cnt = 0;

    unsigned rmw_reads = memoryController->rmw->totalReads;
    unsigned rmw_bypass_reads = memoryController->rmw->totalBypassReads;
    unsigned rmw_writes = memoryController->rmw->totalWrites;
    unsigned rmw_bypass_writes = memoryController->rmw->totalBypassWrites;
    unsigned rmw_full_writes = memoryController->rmw->totalFullWrites;
    unsigned rmw_mask_writes = memoryController->rmw->totalMaskWrites;
    unsigned rmw_totals = memoryController->rmw->totalTransactions;

    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        pde_cnt += memoryController->PdEnterCnt[rank];
        asrefe_cnt += memoryController->AsrefEnterCnt[rank];
        srpde_cnt += memoryController->SrpdEnterCnt[rank];
        pdx_cnt += memoryController->PdExitCnt[rank];
        asrefx_cnt += memoryController->AsrefExitCnt[rank];
        srpdx_cnt += memoryController->SrpdExitCnt[rank];
    }

    if (POWER_EN) {
        STATE_PRINTN("-------------------- Power Event And Power Consumption --------------------------------\n");
        for (size_t que = 1; que <= TRANS_QUEUE_DEPTH; que ++) {
            STATE_PRINTN("PMU Que"<<setw(5)<<que<<":"<<setw(8)<<que_cnt[que]<<" | ");
            if (que % 4 == 0) STATE_PRINTN(endl);
        }
        STATE_PRINTN("PMU RdInc   :"<<setw(8)<<memoryController->rd_inc_cnt<<" | ");
        STATE_PRINTN("PMU RdWrap  :"<<setw(8)<<memoryController->rd_wrap_cnt<<" | ");
        STATE_PRINTN("PMU WrInc   :"<<setw(8)<<memoryController->wr_inc_cnt<<" | ");
        STATE_PRINTN("PMU WrWrap  :"<<setw(8)<<memoryController->wr_wrap_cnt<<" | ");
        STATE_PRINTN(endl);
        STATE_PRINTN("PMU Rdata   :"<<setw(8)<<memoryController->rdata_cnt<<" | ");
        STATE_PRINTN("PMU Wdata   :"<<setw(8)<<memoryController->wdata_cnt<<" | ");
        STATE_PRINTN("PMU Act     :"<<setw(8)<<(act_cnt - pre_act_cnt)<<" | ");
        STATE_PRINTN("PMU Prep    :"<<setw(8)<<(pre_pb_cnt - pre_pre_pb_cnt)<<" | ");
        STATE_PRINTN(endl);
        STATE_PRINTN("PMU Pres    :"<<setw(8)<<(pre_sb_cnt - pre_pre_sb_cnt)<<" | ");
        STATE_PRINTN("PMU Prea    :"<<setw(8)<<(pre_ab_cnt - pre_pre_ab_cnt)<<" | ");
        STATE_PRINTN("PMU RdBl8   :"<<setw(8)<<(memoryController->RdCntBl[BL8] - PreRdCntBl[BL8])<<" | ");
        STATE_PRINTN("PMU RdBl16  :"<<setw(8)<<(memoryController->RdCntBl[BL16] - PreRdCntBl[BL16])<<" | ");
        STATE_PRINTN(endl);
        STATE_PRINTN("PMU RdBl24  :"<<setw(8)<<(memoryController->RdCntBl[BL24] - PreRdCntBl[BL24])<<" | ");
        STATE_PRINTN("PMU RdBl32  :"<<setw(8)<<(memoryController->RdCntBl[BL32] - PreRdCntBl[BL32])<<" | ");
        STATE_PRINTN("PMU RdBl48  :"<<setw(8)<<(memoryController->RdCntBl[BL48] - PreRdCntBl[BL48])<<" | ");
        STATE_PRINTN("PMU RdBl64  :"<<setw(8)<<(memoryController->RdCntBl[BL64] - PreRdCntBl[BL64])<<" | ");
        STATE_PRINTN(endl);
        STATE_PRINTN("PMU WrBl8   :"<<setw(8)<<(memoryController->WrCntBl[BL8] - PreWrCntBl[BL8])<<" | ");
        STATE_PRINTN("PMU WrBl16  :"<<setw(8)<<(memoryController->WrCntBl[BL16] - PreWrCntBl[BL16])<<" | ");
        STATE_PRINTN("PMU WrBl24  :"<<setw(8)<<(memoryController->WrCntBl[BL24] - PreWrCntBl[BL24])<<" | ");
        STATE_PRINTN("PMU WrBl32  :"<<setw(8)<<(memoryController->WrCntBl[BL32] - PreWrCntBl[BL32])<<" | ");
        STATE_PRINTN(endl);
        STATE_PRINTN("PMU WrBl48  :"<<setw(8)<<(memoryController->WrCntBl[BL48] - PreWrCntBl[BL48])<<" | ");
        STATE_PRINTN("PMU WrBl64  :"<<setw(8)<<(memoryController->WrCntBl[BL64] - PreWrCntBl[BL64])<<" | ");
        STATE_PRINTN("PMU Pbr     :"<<setw(8)<<(refresh_pb_cnt - pre_refresh_pb_cnt)<<" | ");
        STATE_PRINTN("PMU Abr     :"<<setw(8)<<(refresh_ab_cnt - pre_refresh_ab_cnt)<<" | ");
        STATE_PRINTN(endl);
        STATE_PRINTN("PMU R2w     :"<<setw(8)<<(r2w_switch_cnt - pre_r2w_switch_cnt)<<" | ");
        STATE_PRINTN("PMU W2r     :"<<setw(8)<<(w2r_switch_cnt - pre_w2r_switch_cnt)<<" | ");
        STATE_PRINTN("PMU Rnksw   :"<<setw(8)<<(rank_switch_cnt - pre_rank_switch_cnt)<<" | ");
        STATE_PRINTN("PMU Pde     :"<<setw(8)<<(pde_cnt - pre_pde_cnt)<<" | ");
        STATE_PRINTN(endl);
        STATE_PRINTN("PMU Asrefe  :"<<setw(8)<<(asrefe_cnt - pre_asrefe_cnt)<<" | ");
        STATE_PRINTN("PMU Srpde   :"<<setw(8)<<(srpde_cnt - pre_srpde_cnt)<<" | ");
        STATE_PRINTN("PMU Pdx     :"<<setw(8)<<(pdx_cnt - pre_pdx_cnt)<<" | ");
        STATE_PRINTN("PMU Asrefx  :"<<setw(8)<<(asrefx_cnt - pre_asrefx_cnt)<<" | ");
        STATE_PRINTN(endl);
        STATE_PRINTN("PMU Srpdx   :"<<setw(8)<<(srpdx_cnt - pre_srpdx_cnt)<<" | ");
        STATE_PRINTN("PMU Idle    :"<<setw(8)<<(phy_notlp_cnt - pre_phy_notlp_cnt)<<" | ");
        STATE_PRINTN("PMU Pdcc    :"<<setw(8)<<(phy_lp_cnt - pre_phy_lp_cnt)<<" | ");
        STATE_PRINTN(endl);
        STATE_PRINTN("Power Consumption :"<<fixed<<setw(26)<<(memoryController->calc_power()-pre_power)<<" | ");
        STATE_PRINTN("Power Consumption Total :"<<fixed<<setw(20)<<memoryController->calc_power()<<endl);
        pre_power = memoryController->calc_power();
    }

    if (DRAM_POWER_EN) {
        STATE_PRINTN("-------------------- Dram Power Consumption -------------------------------------------\n");
        double energy; string unit;
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            auto power = (*ranks)[rank]->DramPower[0];
            UnitConvert(&energy, &unit, power.act_energy);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"ActEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.pre_energy);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"PreEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.rd_energy);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"RdEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(endl);
            UnitConvert(&energy, &unit, power.wr_energy);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"WrEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.act_standby_energy);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"ActIdleEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.pre_standby_energy);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"PreIdleEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(endl);
            UnitConvert(&energy, &unit, power.refpb_energy);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"RefpbEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.refab_energy);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"RefabEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.asref_refab_energy);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"SrefRefEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(endl);
            UnitConvert(&energy, &unit, power.asref_pre_energy);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"SrefIdleEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.asref_energy);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"SrefEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.act_pd_energy);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"ActPdEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(endl);
            UnitConvert(&energy, &unit, power.idle_pd_energy);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"IdlePdEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.srpd_energy);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"SrPdEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.IddEnergy["IDD0"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"Idd0Energy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(endl);
            UnitConvert(&energy, &unit, power.IddEnergy["IDD2N"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"Idd2nEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.IddEnergy["IDD2P"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"Idd2pEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.IddEnergy["IDD3N"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"Idd3nEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(endl);
            UnitConvert(&energy, &unit, power.IddEnergy["IDD3P"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"Idd3pEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.IddEnergy["IDD4W"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"Idd4wEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.IddEnergy["IDD4R"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"Idd4rEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(endl);
            UnitConvert(&energy, &unit, power.IddEnergy["IDD5"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"Idd5Energy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.IddEnergy["IDD6"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"Idd6Energy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.VddEnergy["VDD1"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"Vdd1Energy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(endl);
            UnitConvert(&energy, &unit, power.VddEnergy["VDD2H"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"Vdd2hEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.VddEnergy["VDD2L"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"Vdd2lEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.VddEnergy["VDDQH"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"VddqhEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(endl);
            UnitConvert(&energy, &unit, power.VddEnergy["VDDQL"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"VddqlEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.AvgCurrent["IDD0"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"Idd0Average("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.AvgCurrent["IDD2N"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"Idd2nAverage("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(endl);
            UnitConvert(&energy, &unit, power.AvgCurrent["IDD2P"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"Idd2pAverage("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.AvgCurrent["IDD3N"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"Idd3nAverage("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.AvgCurrent["IDD3P"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"Idd3pAverage("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(endl);
            UnitConvert(&energy, &unit, power.AvgCurrent["IDD4W"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"Idd4wAverage("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.AvgCurrent["IDD4R"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"Idd4rAverage("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            UnitConvert(&energy, &unit, power.AvgCurrent["IDD5"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"Idd5Average("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN(endl);
            UnitConvert(&energy, &unit, power.AvgCurrent["IDD6"]);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"Idd6Average("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"EngEffi(pJ/bit)");
            STATE_PRINTN(" : "<<setw(8)<<power.energy_efficiency<<" | ");
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"AvgPower(mW)");
            STATE_PRINTN(" : "<<setw(8)<<power.average_power<<" | ");
            STATE_PRINTN(endl);
        }
        double total_energy = 0, total_power = 0;
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            auto power = (*ranks)[rank]->DramPower[0];
            UnitConvert(&energy, &unit, power.total_energy);
            STATE_PRINTN("Rank"<<setw(2)<<rank<<setw(18)<<"AllEnergy("+unit+")");
            STATE_PRINTN(" : "<<setw(8)<<energy<<setw(3)<<" | ");
            total_energy += power.total_energy;
        }
        UnitConvert(&energy, &unit, total_energy);
        STATE_PRINTN(setw(24)<<"TotalEnergy("+unit+")"<<" : "<<setw(8)<<energy<<setw(3)<<" | "<<endl);
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            total_power += (*ranks)[rank]->DramPower[0].average_power;
        }
        STATE_PRINTN(setw(24)<<"TotalAvgPower(mW)"<<" : "<<setw(8)<<total_power<<setw(3)<<" | "<<endl);
    }
    STATE_PRINTN("-------------------- Task Statistics (DMC Command Number) -----------------------------\n");
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        STATE_PRINTN("Rank"<<rank<<" read      : "<<setw(8)<<
                (memoryController->racc_rank_cnt[rank]-pre_racc_rank_cnt[rank])<<" | ");
        STATE_PRINTN("Rank"<<rank<<" total read  : "<<setw(8)<<memoryController->racc_rank_cnt[rank]<<" | ");
        if (rank == NUM_RANKS - 1) STATE_PRINTN(endl);
    }
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        STATE_PRINTN("Rank"<<rank<<" write     : "<<setw(8)<<
                (memoryController->wacc_rank_cnt[rank]-pre_wacc_rank_cnt[rank])<<" | ");
        STATE_PRINTN("Rank"<<rank<<" total write : "<<setw(8)<<memoryController->wacc_rank_cnt[rank]<<" | ");
        if (rank == NUM_RANKS - 1) STATE_PRINTN(endl);
    }
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        STATE_PRINTN("Rank"<<rank<<" r+w       : "<<setw(8)<<
                (memoryController->acc_rank_cnt[rank]-pre_acc_rank_cnt[rank])<<" | ");
        STATE_PRINTN("Rank"<<rank<<" total r+w   : "<<setw(8)<<memoryController->acc_rank_cnt[rank]<<" | ");
        if (rank == NUM_RANKS - 1) STATE_PRINTN(endl);
    }
    STATE_PRINTN("Read            : "<<setw(8)<<reads-pre_reads);
    STATE_PRINTN(" | Total reads       : "<<setw(8)<<reads);
    STATE_PRINTN(" | Write           : "<<setw(8)<<writes-pre_writes);
    STATE_PRINTN(" | Total writes      : "<<setw(8)<<writes<<" |"<<endl);
    STATE_PRINTN("Total           : "<<setw(8)<<totals-pre_totals);
    STATE_PRINTN(" | Total commands    : "<<setw(8)<<totals);
    STATE_PRINTN(" | DMC             : "<<setw(8)<<access_cnt);
    STATE_PRINTN(" | Total pre_act     : "<<setw(8)<<memoryController->total_pre_act_cnt<<" |"<<endl);
    STATE_PRINTN("Total 32B Read  : "<<setw(8)<<memoryController->TotalDmcRd32B);
    STATE_PRINTN(" | Total 32B Write   : "<<setw(8)<<memoryController->TotalDmcWr32B);
    STATE_PRINTN(" | Total 64B Read  : "<<setw(8)<<memoryController->TotalDmcRd64B);
    STATE_PRINTN(" | Total 64B Write   : "<<setw(8)<<memoryController->TotalDmcWr64B<<" |"<<endl);
    STATE_PRINTN("Total 128B Read : "<<setw(8)<<memoryController->TotalDmcRd128B);
    STATE_PRINTN(" | Total 128B Write  : "<<setw(8)<<memoryController->TotalDmcWr128B);
    STATE_PRINTN(" | Total 256B Read : "<<setw(8)<<memoryController->TotalDmcRd256B);
    STATE_PRINTN(" | Total 256B Write  : "<<setw(8)<<memoryController->TotalDmcWr256B<<" |"<<endl);
    if (RMW_ENABLE && memoryController->rmw != NULL) {
        STATE_PRINTN("Write Merge Input : "<<setw(8)<<memoryController->rmw->totalWriteMergeInput);
        STATE_PRINTN(" | Pair Merge       : "<<setw(8)<<memoryController->rmw->totalWriteMergePair);
        STATE_PRINTN(" | Unpaired To RMW : "<<setw(8)<<memoryController->rmw->totalWriteMergeUnpairedToRmw);
        STATE_PRINTN(" | Unpaired Direct : "<<setw(8)<<memoryController->rmw->totalWriteMergeUnpairedDirect);
        STATE_PRINTN(" | Buffer Full : "<<setw(8)<<memoryController->rmw->totalWriteMergeBufferFull<<" |"<<endl);
    }
    STATE_PRINTN("IECC            : "<<setw(8)<<memoryController->total_iecc_cnt);
    STATE_PRINTN(" | No IECC           : "<<setw(8)<<memoryController->total_noiecc_cnt);
    STATE_PRINTN(" | Total pre_act_success : "<<setw(8)<<memoryController->total_pre_act_success_cnt<<" |"<<endl);
    for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            unsigned b = rank * NUM_BANKS + bank;
            STATE_PRINTN("Bank"<<setw(2)<<b<<" ");
            STATE_PRINTN("read : "<<setw(6)<<memoryController->racc_bank_cnt[b]<<" | ");
            STATE_PRINTN("write : "<<setw(6)<<memoryController->wacc_bank_cnt[b]<<" | ");
            STATE_PRINTN("total : "<<setw(6)<<memoryController->acc_bank_cnt[b]<<" | ");
        }
        STATE_PRINTN(endl);
    }
    STATE_PRINTN("-------------------- Row Conflict Precharge Count (Counter) ---------------------------\n");
    for (size_t bank = 0; bank < NUM_RANKS * NUM_BANKS; bank ++) {
        STATE_PRINTN("Bank"<<setw(2)<<bank<<" "<<setw(13)<<"RowConfPre");
        STATE_PRINTN(" : "<<setw(6)<<memoryController->rowconf_pre_cnt[bank]<<" | ");
        if (bank % 4 == 3) STATE_PRINTN(endl);
    }
    STATE_PRINTN("-------------------- Page Timeout Count (Counter) -------------------------------------\n");
    for (size_t bank = 0; bank < NUM_RANKS * NUM_BANKS; bank ++) {
        STATE_PRINTN("Bank"<<setw(2)<<bank<<" "<<setw(13)<<"PageTimeout");
        STATE_PRINTN(" : "<<setw(6)<<memoryController->pageto_pre_cnt[bank]<<" | ");
        if (bank % 4 == 3) STATE_PRINTN(endl);
    }
    STATE_PRINTN("-------------------- Enhance Page Adapt Level Count (Counter) -------------------------\n");
    size = MAP_CONFIG["ENH_PAGE_ADPT_TIME"].size();
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        STATE_PRINTN("Rank"<<i<<" "<<setw(13)<<"ENH_PAGE_ADAPT"<<" : ");
        for (size_t j = 0; j < size; j ++) {
            STATE_PRINTN("Level"<<j<<" : "<<setw(13)<<memoryController->ehs_page_adapt_cnt[i][j]<<" | ");
        }
        STATE_PRINTN(endl);
    }
    STATE_PRINTN("-------------------- Func Precharge Count (Counter) -----------------------------------\n");
    for (size_t bank = 0; bank < NUM_RANKS * NUM_BANKS; bank ++) {
        STATE_PRINTN("Bank"<<setw(2)<<bank<<" "<<setw(13)<<"FuncPrecharge");
        STATE_PRINTN(" : "<<setw(6)<<memoryController->func_pre_cnt[bank]<<" | ");
        Total_func_pre_cnt += memoryController->func_pre_cnt[bank]; 
        if (bank % 4 == 3) STATE_PRINTN(endl);
    }
    STATE_PRINTN("Total"<<setw(2)<<" "<<setw(13)<<"FuncPrecharge");
    STATE_PRINTN(" : "<<setw(6)<<Total_func_pre_cnt<<" | ");
    STATE_PRINTN(endl);
    STATE_PRINTN("-------------------- Func ABR Count (Counter) -----------------------------------------\n");     //todo: revise for e-mode
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        for (size_t sub_channel = 0; sub_channel < memoryController->sc_num; sub_channel ++) {
            STATE_PRINTN("Rank"<<setw(2)<<rank<<" & Sc"<<setw(2)<<sub_channel<<setw(13)<<"FuncAbr"<<" : "<<setw(6));
            STATE_PRINTN((memoryController->rank_refresh_cnt[rank][sub_channel] - pre_abr_cnt[rank][sub_channel])<<" | ");
        }
        if (rank % 4 == 3 || rank == NUM_RANKS - 1) STATE_PRINTN(endl);
    }
    STATE_PRINTN("-------------------- Func PBR Count (Counter) -----------------------------------------\n");
    if (ENH_PBR_EN) {
        for (size_t bank = 0; bank < NUM_RANKS * NUM_BANKS; bank ++) {
            STATE_PRINTN("Bank"<<setw(2)<<bank<<" "<<setw(13)<<"FuncPbr"<<" : "<<setw(6));
            STATE_PRINTN((memoryController->perbank_refresh_cnt[bank] - pre_pbr_cnt[bank])<<" | ");
            if (bank % 4 == 3) STATE_PRINTN(endl);
        }
    
    } else {
        for (size_t bank = 0; bank < NUM_RANKS * memoryController->pbr_bank_num * memoryController->sc_num; bank ++) {
            STATE_PRINTN("Bank"<<setw(2)<<bank<<" "<<setw(13)<<"FuncPbr"<<" : "<<setw(6));
            STATE_PRINTN((memoryController->perbank_refresh_cnt[bank] - pre_pbr_cnt[bank])<<" | ");
            if (bank % 4 == 3) STATE_PRINTN(endl);
        }
    }

    STATE_PRINTN("-------------------- Request Statistics (DDR Command Number) --------------------------\n");
    row_hit_cnt = ((read_cnt + read_p_cnt + write_cnt + write_p_cnt + mwrite_cnt + mwrite_p_cnt) > act_cnt) ?
            (read_cnt + read_p_cnt + write_cnt + write_p_cnt + mwrite_cnt + mwrite_p_cnt - act_cnt) : 0;
    row_miss_cnt = act_cnt;

    STATE_PRINTN(setw(36)<<"Active cnt"<<" : "<<setw(12)<<act_cnt - pre_act_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total act cnt"<<" : "<<act_cnt<<endl);
    STATE_PRINTN(setw(36)<<"Disturb active cnt"<<" : "<<setw(12)<<act_dst_cnt - pre_act_dst_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total disturb active cnt"<<" : "<<act_dst_cnt<<endl);
    STATE_PRINTN(setw(36)<<"Precharge_sb cnt"<<" : "<<setw(12)<<pre_sb_cnt - pre_pre_sb_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total precharge_sb cnt"<<" : "<<pre_sb_cnt<<endl);
    STATE_PRINTN(setw(36)<<"Precharge_pb cnt"<<" : "<<setw(12)<<pre_pb_cnt - pre_pre_pb_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total precharge_pb cnt"<<" : "<<pre_pb_cnt<<endl);
    STATE_PRINTN(setw(36)<<"Disturb precharge_pb cnt"<<" : "<<setw(12)<<pre_pb_dst_cnt - pre_pre_pb_dst_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total disturb precharge_pb cnt"<<" : "<<pre_pb_dst_cnt<<endl);
    STATE_PRINTN(setw(36)<<"Precharge_ab cnt"<<" : "<<setw(12)<<pre_ab_cnt - pre_pre_ab_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total precharge_ab cnt"<<" : "<<pre_ab_cnt<<endl);
    STATE_PRINTN(setw(36)<<"Read cnt"<<" : "<<setw(12)<<read_cnt - pre_read_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total read cnt"<<" : "<<read_cnt<<endl);
    STATE_PRINTN(setw(36)<<"Write"<<" : "<<setw(12)<<write_cnt - pre_write_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total write"<<" : "<<write_cnt<<endl);
    STATE_PRINTN(setw(36)<<"Read with AP"<<" : "<<setw(12)<<read_p_cnt - pre_read_p_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total read with AP"<<" : "<<read_p_cnt<<endl);
    STATE_PRINTN(setw(36)<<"Write with AP"<<" : "<<setw(12)<<write_p_cnt - pre_write_p_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total write with AP"<<" : "<<write_p_cnt<<endl);
    STATE_PRINTN(setw(36)<<"Mask write"<<" : "<<setw(12)<<mwrite_cnt - pre_mwrite_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total mask write"<<" : "<<mwrite_cnt<<endl);
    STATE_PRINTN(setw(36)<<"Mask write with AP"<<" : "<<setw(12)<<mwrite_p_cnt - pre_mwrite_p_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total mask write with AP"<<" : "<<mwrite_p_cnt<<endl);
    STATE_PRINTN(setw(36)<<"Row hit"<<" : "<<setw(12)<<row_hit_cnt - pre_row_hit_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total row hit"<<" : "<<row_hit_cnt<<endl);
    STATE_PRINTN(setw(36)<<"Row miss cnt"<<" : "<<setw(12)<<row_miss_cnt - pre_row_miss_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total row miss cnt"<<" : "<<row_miss_cnt<<endl);
    STATE_PRINTN(setw(36)<<"ecc read cnt"<<" : "<<setw(12)<<ecc_read_cnt - pre_ecc_read_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total ecc read cnt"<<" : "<<ecc_read_cnt<<endl);
    STATE_PRINTN(setw(36)<<"ecc write cnt"<<" : "<<setw(12)<<ecc_write_cnt - pre_ecc_write_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total ecc write cnt"<<" : "<<ecc_write_cnt<<endl);
    STATE_PRINTN(setw(36)<<"merge read cnt"<<" : "<<setw(12)<<merge_read_cnt - pre_merge_read_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total merge read cnt"<<" : "<<merge_read_cnt<<endl);

    unsigned total_cnt = row_hit_cnt + row_miss_cnt;
    float total_row_hit_ratio = (total_cnt == 0) ? 0 : ((float)row_hit_cnt / total_cnt);
    float total_row_miss_ratio = (total_cnt == 0) ? 0 : ((float)row_miss_cnt / total_cnt);

    unsigned cur_total_cnt = row_hit_cnt - pre_row_hit_cnt + row_miss_cnt - pre_row_miss_cnt;
    float cur_total_row_hit_ratio = (cur_total_cnt == 0) ? 0 :
            ((float)(row_hit_cnt>pre_row_hit_cnt?(row_hit_cnt-pre_row_hit_cnt):0)/cur_total_cnt);
    float cur_total_row_miss_ratio = (cur_total_cnt == 0) ? 0 :
            ((float)(row_miss_cnt>pre_row_miss_cnt?(row_miss_cnt-pre_row_miss_cnt):0)/cur_total_cnt);

    STATE_PRINTN(setw(36)<<"Row hit ratio"<<" : "<<setw(12)<<cur_total_row_hit_ratio);
    STATE_PRINTN(" | "<<setw(36)<<"Total row hit ratio"<<" : "<<total_row_hit_ratio<<endl);
    STATE_PRINTN(setw(36)<<"Row miss ratio"<<" : "<<setw(12)<<cur_total_row_miss_ratio);
    STATE_PRINTN(" | "<<setw(36)<<"Total row miss ratio"<<" : "<<total_row_miss_ratio<<endl);
    STATE_PRINTN(setw(36)<<"Timeout cnt"<<" : "<<setw(12)<<timeout_cnt - pre_timeout_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total timeout cnt"<<" : "<<timeout_cnt<<endl);
    STATE_PRINTN(setw(36)<<"Real Time Timeout cnt"<<" : "<<setw(12)<<rt_timeout_cnt - pre_rt_timeout_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total RT timeout cnt"<<" : "<<rt_timeout_cnt<<endl);

    STATE_PRINTN("-------------------- Confilct Statistics (DDR Command Number) -------------------------\n");
    STATE_PRINTN(setw(36)<<"Address conflict"<<" : "<<setw(12)<<address_conf_cnt - pre_address_conf_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total address conf cnt"<<" : "<<address_conf_cnt<<endl);
    STATE_PRINTN(setw(36)<<"Id conflict"<<" : "<<setw(12)<<id_conf_cnt - pre_id_conf_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total id conf cnt"<<" : "<<id_conf_cnt<<endl);
    STATE_PRINTN(setw(36)<<"Bank conflict"<<" : "<<setw(12)<<ba_conf_cnt - pre_ba_conf_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total bank conf cnt"<<" : "<<ba_conf_cnt<<endl);
    STATE_PRINTN(setw(36)<<"All conflict"<<" : "<<setw(12)<<total_conf - pre_total_conf);
    STATE_PRINTN(" | "<<setw(36)<<"Total all conf cnt"<<" : "<<total_conf<<endl);
    STATE_PRINTN(setw(36)<<"RW switch cnt"<<" : "<<setw(12)<<rw_switch_cnt - pre_rw_switch_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total RW switch cnt"<<" : "<<rw_switch_cnt<<endl);
    STATE_PRINTN(setw(36)<<"Rank switch cnt"<<" : "<<setw(12)<<rank_switch_cnt - pre_rank_switch_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total rank switch cnt"<<" : "<<rank_switch_cnt<<endl);
    STATE_PRINTN(setw(36)<<"Sid switch cnt"<<" : "<<setw(12)<<sid_switch_cnt - pre_sid_switch_cnt);
    STATE_PRINTN(" | "<<setw(36)<<"Total sid switch cnt"<<" : "<<sid_switch_cnt<<endl);
    if (rank_switch_cnt == 0) {
        STATE_PRINTN(setw(36)<<"Average rank cnt"<<" : "<<setw(12)<<0);
    } else {
        STATE_PRINTN(setw(36)<<"Average rank cnt"<<" : "<<setw(12)<<totals / rank_switch_cnt);
    }
    if (sid_switch_cnt == 0) {
        STATE_PRINTN(" | "<<setw(36)<<"Average sid cnt"<<" : 0"<<endl);
    } else {
        STATE_PRINTN(" | "<<setw(36)<<"Average sid cnt"<<" : "<<totals / sid_switch_cnt<<endl);
    }

    STATE_PRINTN("-------------------- Latency Statistics (ns less than) --------------------------------\n");
    size = memoryController->lat_dly_cnt.size();
    for (size_t mid = 0; mid <= size; mid ++) {
        STATE_PRINTN("--------");
    }
    STATE_PRINTN(endl);
    STATE_PRINTN(setw(6)<<"LatTh"<<" |");
    for (size_t i = 0; i < size; i ++) {
        STATE_PRINTN(setw(6)<<memoryController->lat_dly_step[i]<<" |");
    }
    STATE_PRINTN(endl);
    STATE_PRINTN(setw(6)<<"LatNe"<<" |");
    for (size_t i = 0; i < size; i ++) {
        STATE_PRINTN(setw(6)<<memoryController->lat_dly_cnt[i]<<" |");
    }
    STATE_PRINTN(endl);
    for (size_t mid = 0; mid <= size; mid ++) {
        STATE_PRINTN("--------");
    }
    STATE_PRINTN(endl);

    STATE_PRINTN("-------------------- Latency Statistics (ns) ------------------------------------------\n");
    STATE_PRINTN("max latency : "<<float(memoryController->max_delay) * tDFI
            <<", task="<<memoryController->max_delay_id<<", ");
    STATE_PRINTN("min latency : "<<float(memoryController->min_delay) * tDFI
            <<", task="<<memoryController->min_delay_id<<endl);
    STATE_PRINTN("high qos max latency : "<<float(memoryController->highqos_max_delay) * tDFI
            <<", task="<<memoryController->highqos_max_delay_id<<endl);
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        STATE_PRINTN("total ddrc average latency rank"<<rank<<" : "
                <<memoryController->ddrc_av_lat_rank[rank] * tDFI<<endl);
    }
    if ((memoryController->com_read_cnt - pre_com_read_cnt) > 0) {
        float avg_lat = float(memoryController->total_latency - pre_total_latency) /
                (memoryController->com_read_cnt - pre_com_read_cnt);
        STATE_PRINTN("ddrc average latency : "<<avg_lat * tDFI<<endl);
    } else {
        STATE_PRINTN("ddrc average latency : 0"<<endl);
    }
    STATE_PRINTN("total ddrc average latency : "<<memoryController->ddrc_av_lat * tDFI<<endl);
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        STATE_PRINTN("total ddrc average high qos latency rank"<<rank<<" : "
                <<memoryController->rank_ddrc_av_highqos_lat[rank] * tDFI<<endl);
    }
    STATE_PRINTN("total ddrc average high qos latency : "<<memoryController->ddrc_av_highqos_lat * tDFI<<endl);
    STATE_PRINTN("high qos trigger read group swiching cnt : "<<memoryController->highqos_trig_grpsw_cnt<<endl);
    float cmd2dfi_ave_lat = (memoryController->cmd_in2dfi_cnt == 0) ? 0 :
            (float(memoryController->cmd_in2dfi_lat) / memoryController->cmd_in2dfi_cnt);
    STATE_PRINTN("total cmd2dfi average latency : "<<cmd2dfi_ave_lat * tDFI<<endl);
    float cmd_rdmet_cnt = (memoryController->cmd_in2dfi_cnt == 0) ? 0 :
            (float(memoryController->cmd_rdmet_cnt) / memoryController->cmd_in2dfi_cnt);
    STATE_PRINTN("total read timing met average cnt : "<<cmd_rdmet_cnt<<endl);
    unsigned inhere_lat = tD_D+tCMD_PHY+tDAT_PHY+tCMD_RASC+tDAT_RASC+tCMD_ADAPT+RL+(BLEN/2/2/WCK2DFI_RATIO);
    STATE_PRINTN("inhere latency(first rdata) : "<<float(inhere_lat) * tDFI<<endl);
    pre_total_latency = memoryController->total_latency;
    pre_com_read_cnt = memoryController->com_read_cnt;

    STATE_PRINTN("Pd Block Read cnt : "<<memoryController->rd_met_pd_cnt<<", Average cnt : "
            <<(float(memoryController->rd_met_pd_cnt)/memoryController->totalReads)<<endl);
    STATE_PRINTN("Asref Block Read cnt : "<<memoryController->rd_met_asref_cnt<<", Average cnt : "
            <<(float(memoryController->rd_met_asref_cnt)/memoryController->totalReads)<<endl);
    STATE_PRINTN("Cmd Met Pd cnt : "<<memoryController->cmd_met_pd_cnt<<", Average cnt : "
            <<(float(memoryController->cmd_met_pd_cnt)/memoryController->totalTransactions)<<endl);
    STATE_PRINTN("Cmd Met Asref cnt : "<<memoryController->cmd_met_asref_cnt<<", Average cnt : "
            <<(float(memoryController->cmd_met_asref_cnt)/memoryController->totalTransactions)<<endl);
    STATE_PRINTN("PBR Block Read cnt : "<<memoryController->rd_met_pbr_cnt<<", Average cnt : "
            <<(float(memoryController->rd_met_pbr_cnt)/memoryController->totalReads)<<endl);
    STATE_PRINTN("ABR Block Read cnt : "<<memoryController->rd_met_abr_cnt<<", Average cnt : "
            <<(float(memoryController->rd_met_abr_cnt)/memoryController->totalReads)<<endl);
    STATE_PRINTN("PBR Block Cmd cycle : "<<memoryController->pbr_block_allcmd_cycle<<", PBR cycle : "<<memoryController->pbr_cycle<<", ref_block_ratio : "
            <<(float(memoryController->pbr_block_allcmd_cycle)/memoryController->pbr_cycle)<<endl);
    float samerow_ratio = 100 * float(memoryController->samerow_mask_rdcnt) / totals;
    STATE_PRINTN("Same row mask read cnt : "<<memoryController->samerow_mask_rdcnt);
    STATE_PRINTN(", ratio : "<<samerow_ratio<<"%"<<endl);
    samerow_ratio = 100 * float(memoryController->samerow_mask_wrcnt) / totals;
    STATE_PRINTN("Same row mask write cnt : "<<memoryController->samerow_mask_wrcnt);
    STATE_PRINTN(", ratio : "<<samerow_ratio<<"%"<<endl);
    float page_exceed_ratio = (memoryController->precharge_pb_cnt == 0) ? 0 :
            100 * memoryController->page_exceed_cnt / memoryController->precharge_pb_cnt;
    STATE_PRINTN("Precharge_pb cnt : "<<memoryController->precharge_pb_cnt);
    STATE_PRINTN(", Page open time exceed cnt : "<<memoryController->page_exceed_cnt);
    STATE_PRINTN(", ratio : "<<page_exceed_ratio<<"%"<<endl);
    if (TRFC_CC_EN) {
        float cc_ratio = 100 * double(memoryController->abr_cc_cnt) / double(now());
        STATE_PRINTN("Abr close clock : "<<memoryController->abr_cc_cnt);
        STATE_PRINTN(", ratio : "<<cc_ratio<<"%"<<endl);
        cc_ratio = 100 * double(memoryController->pbr_cc_cnt) / double(now());
        STATE_PRINTN("Pbr close clock : "<<memoryController->pbr_cc_cnt);
        STATE_PRINTN(", ratio : "<<cc_ratio<<"%"<<endl);
    }
#if 0
    //STATE_PRINTN("    ");
    //for (size_t i = 0; i < 32; i ++) {
    //    STATE_PRINTN(" "<<setw(5)<<i<<" |");
    //}
    //STATE_PRINTN(endl);
    //STATE_PRINTN("RD :");
    //for (size_t i = 0; i < 32; i ++) {
    //    STATE_PRINTN(" "<<setw(5)<<memoryController->samerow_bit_rdcnt[i]<<" |");
    //}
    //STATE_PRINTN(endl);
    //STATE_PRINTN("WR :");
    //for (size_t i = 0; i < 32; i ++) {
    //    STATE_PRINTN(" "<<setw(5)<<memoryController->samerow_bit_wrcnt[i]<<" |");
    //}
    //STATE_PRINTN(endl);
    for (size_t i = 0; i < 32; i ++) {
        STATE_PRINTN(setw(10)<<(memoryController->samerow_bit_rdcnt[i]+memoryController->samerow_bit_wrcnt[i])<<endl);
    }
#endif
    uint32_t total_qos_cnt = 0;
    for (size_t i = 0; i < 8; i ++) {
        total_qos_cnt += memoryController->qos_cnt[i];
    }
    STATE_PRINTN("-------------------- QOS Statistics (Total DMC Command Number/ns) ---------------------\n");
    STATE_PRINTN("qos "<<"|Count   "<<"|Ratio "<<"|Tout cnt "<<"|Av lat "<<"|0~50      "
            <<"|50~100    "<<"|100~150   "<<"|150~200   "<<"|200~300   "<<"|300~500   "<<"|500~600   "
            <<"|600~750   "<<"|750~1000  "<<"|1000~1500 "<<"|1500~2000 "<<"|2000~max  |"<<endl);
    for (size_t qos = 0; qos < 8; qos ++) {
        STATE_PRINTN(setw(4)<<qos<<"|"<<setw(8)<<memoryController->qos_cnt[qos]<<"|");
        STATE_PRINTN(setw(6)<<fixed<<setprecision(2)<<((total_qos_cnt == 0) ? 0 :
                    (float(memoryController->qos_cnt[qos]) * 100 / total_qos_cnt))<<"|");
        STATE_PRINTN(setw(9)<<memoryController->qos_timeout_cnt[qos]<<"|");
        STATE_PRINTN(setw(7)<<fixed<<setprecision(2)<<((memoryController->qos_cnt[qos] == 0) ? 0 :
                    (float(memoryController->qos_delay_cnt[qos]) * tDFI / memoryController->qos_cnt[qos]))<<"|");
        for (size_t level = 0; level < 12; level ++) {
            STATE_PRINTN(setw(10)<<memoryController->qos_level_cnt[qos][level]<<"|");
        }
        STATE_PRINTN(endl);
    }

    STATE_PRINTN("-------------------- MID Statistics (Command Number:Cmd2DQ Latency/ns) ----------------\n");
    for (size_t mid = 0; mid <= MidMax; mid ++) {
        STATE_PRINTN("--------");
    }
    STATE_PRINTN(endl);
    STATE_PRINTN("Mid    |");
    for (size_t mid = 0; mid < MidMax; mid ++) {
        STATE_PRINTN(setw(7)<<mid<<"|");
    }
    STATE_PRINTN(endl);
    STATE_PRINTN("MidCnt |");
    for (size_t mid = 0; mid < MidMax; mid ++) {
        STATE_PRINTN(setw(7)<<memoryController->mid_cnt[mid]<<"|");
    }
    STATE_PRINTN(endl);
    STATE_PRINTN("MidLat |");
    for (size_t mid = 0; mid < MidMax; mid ++) {
        float mid_delay = float(memoryController->mid_delay_cnt[mid]);
        float mid_cnt = float(memoryController->mid_cnt[mid]);
        float mid_latency = (mid_cnt == 0) ? 0 : mid_delay * tDFI / mid_cnt;
        STATE_PRINTN(setw(7)<<mid_latency<<"|");
    }
    STATE_PRINTN(endl);
    for (size_t mid = 0; mid <= MidMax; mid ++) {
        STATE_PRINTN("--------");
    }
    STATE_PRINTN(endl);

    STATE_PRINTN("-------------------- OCC Statistics (Counter Number) ----------------------------------\n");
    STATE_PRINTN("occ 0 cnt : "<<setw(10)<<memoryController->occ_1_cnt<<" | ");
    STATE_PRINTN("occ 1 cnt : "<<setw(10)<<memoryController->occ_2_cnt<<" | ");
    STATE_PRINTN("occ 2 cnt : "<<setw(10)<<memoryController->occ_3_cnt<<" | ");
    STATE_PRINTN("occ 3 cnt : "<<setw(10)<<memoryController->occ_4_cnt<<endl);

#ifdef SYSARCH_PLATFORM
    STATE_PRINTN("-------------------- SQRT Statistics (Counter Number) ---------------------------------\n");
    STATE_PRINTN("DMC availability : "<<(memoryController->avai_sqrt)<<endl);
#endif

    STATE_PRINTN("-------------------- GBUF Statistics (Counter Number) ---------------------------------\n");
    STATE_PRINTN(setw(15)<<"Gbuf read"<<" : "<<setw(10)<<(memoryController->wb->read_cnt-pre_gbuf_reads)<<" | ");
    STATE_PRINTN(setw(15)<<"Total Gbuf read"<<" : "<<setw(10)<<memoryController->wb->read_cnt<<" | ");
    STATE_PRINTN(setw(15)<<"Gbuf write"<<" : "<<setw(10)<<(memoryController->wb->write_cnt-pre_gbuf_writes)<<" | ");
    STATE_PRINTN(setw(15)<<"Total Gbuf write"<<" : "<<setw(10)<<memoryController->wb->write_cnt<<" | "<<endl);
    STATE_PRINTN(setw(15)<<"Merge"<<" : "<<setw(10)<<memoryController->wb->merge_cnt<<" | ");
    STATE_PRINTN(setw(15)<<"Forward"<<" : "<<setw(10)<<memoryController->wb->forward_cnt<<" | ");
    STATE_PRINTN(setw(15)<<"AddrPush"<<" : "<<setw(10)<<memoryController->wb->push_cnt<<" | "<<endl);
    pre_gbuf_reads = memoryController->wb->read_cnt;
    pre_gbuf_writes = memoryController->wb->write_cnt;
    for (uint8_t i = 0; i < 6; i ++) {
        string level = "Gbuf lvl" + to_string(i);
        STATE_PRINTN(setw(15)<<level<<" : "<<setw(10)<<memoryController->wb->sch_level_cnt[i]-pre_sch_level_cnt[i]<<" | ");
        STATE_PRINTN(setw(15)<<"Total "+level<<" : "<<setw(10)<<memoryController->wb->sch_level_cnt[i]<<" | "<<endl);
    }

    STATE_PRINTN("-------------------- RMW Statistics (Counter Number) -----------------------------\n");
    STATE_PRINTN("Read            : "<<setw(8)<<rmw_reads-pre_rmw_reads);
    STATE_PRINTN(" | Total reads       : "<<setw(8)<<rmw_reads);
    STATE_PRINTN(" | Bypass Read            : "<<setw(8)<<rmw_bypass_reads-pre_rmw_bypass_reads);
    STATE_PRINTN(" | Total bypass reads       : "<<setw(8)<<rmw_bypass_reads<<" | "<<endl);
    STATE_PRINTN("Write           : "<<setw(8)<<rmw_writes-pre_rmw_writes);
    STATE_PRINTN(" | Total writes      : "<<setw(8)<<rmw_writes);
    STATE_PRINTN(" | Full Write             : "<<setw(8)<<rmw_full_writes-pre_rmw_full_writes);
    STATE_PRINTN(" | Total full writes        : "<<setw(8)<<rmw_full_writes<<" | "<<endl);
    STATE_PRINTN("Mask Write      : "<<setw(8)<<rmw_mask_writes-pre_rmw_mask_writes);
    STATE_PRINTN(" | Total mask writes : "<<setw(8)<<rmw_mask_writes);
    STATE_PRINTN(" | Bypass Write           : "<<setw(8)<<rmw_bypass_writes-pre_rmw_bypass_writes);
    STATE_PRINTN(" | Total bypass writes      : "<<setw(8)<<rmw_bypass_writes<<" | "<<endl);
    STATE_PRINTN("Total           : "<<setw(8)<<rmw_totals-pre_rmw_totals);
    STATE_PRINTN(" | Total commands    : "<<setw(8)<<rmw_totals<<" | "<<endl);


    STATE_PRINTN("-------------------- DMC Pressure Statistics (Percentage/Cycle) -----------------------\n");
    float ratio = float(task_cnt) * 100 / STATE_TIME;
    STATE_PRINTN(setw(15)<<"Cmd valid"<<" : "<<setw(10)<<task_cnt<<" | ");
    STATE_PRINTN(setw(15)<<"Ratio"<<" : "<<setw(10)<<ratio<<" | ");
    ratio = float(total_task_cnt) * 100 / now();
    STATE_PRINTN(setw(15)<<"Total cmd valid"<<" : "<<setw(10)<<total_task_cnt<<" | ");
    STATE_PRINTN(setw(15)<<"Ratio"<<" : "<<setw(10)<<ratio<<" | "<<endl);
    ratio = float(bp_cnt) * 100 / (bp_cnt + access_cnt);
    STATE_PRINTN(setw(15)<<"DMC access"<<" : "<<setw(10)<<access_cnt<<" | ");
    STATE_PRINTN(setw(15)<<"Command bp"<<" : "<<setw(10)<<bp_cnt<<" | ");
    STATE_PRINTN(setw(15)<<"Bp ratio"<<" : "<<setw(10)<<ratio<<" | "<<endl);
    ratio = float(total_bp_cnt) * 100 / (total_bp_cnt + total_access_cnt);
    STATE_PRINTN(setw(15)<<"Total access"<<" : "<<setw(10)<<total_access_cnt<<" | ");
    STATE_PRINTN(setw(15)<<"Total bp"<<" : "<<setw(10)<<total_bp_cnt<<" | ");
    STATE_PRINTN(setw(15)<<"Total bp ratio"<<" : "<<setw(10)<<ratio<<" | "<<endl);

    STATE_PRINTN("-------------------- DMC Queue Statistics (Percentage/Cycle) --------------------------\n");
    uint32_t total = 0;
    size = que_cnt.size();

    for (uint32_t index = 0; index <= size; index ++) {
        STATE_PRINTN("--------");
    }
    STATE_PRINTN(endl);
    STATE_PRINTN(setw(7)<<"Qnum"<<"|");
    for (uint32_t index = 0; index < size; index++) {
        total += que_cnt.at(index);
        STATE_PRINTN(setw(7)<<index<<"|");
    }
    STATE_PRINTN(endl);
    STATE_PRINTN(setw(7)<<"Per"<<"|");
    for (uint32_t index = 0; index < size; index ++) {
        float cnt_dist_ratio = (float(que_cnt.at(index)) * 100) / total;
        STATE_PRINTN(setw(7)<<fixed<<setprecision(3)<<cnt_dist_ratio<<"|");
    }
    STATE_PRINTN(endl);
    STATE_PRINTN(setw(7)<<"Cycle"<<"|");
    for (uint32_t index = 0; index < size; index ++) {
        STATE_PRINTN(setw(7)<<fixed<<setprecision(3)<<que_cnt.at(index)<<"|");
    }
    STATE_PRINTN(endl);
    for (uint32_t index = 0; index <= size; index ++) {
        STATE_PRINTN("--------");
    }
    STATE_PRINTN(endl);

    for (uint32_t index = 0; index < size; index++) {
        que_cnt.at(index) = 0;
    }

    STATE_PRINTN("-------------------- RMW: Queue Statistics (Percentage/Cycle) --------------------------\n");
    uint32_t rmw_total = 0;
    size = memoryController->rmw->rmw_que_cnt.size();

    for (uint32_t index = 0; index <= size; index ++) {
        STATE_PRINTN("--------");
    }
    STATE_PRINTN(endl);
    STATE_PRINTN(setw(7)<<"Qnum"<<"|");
    for (uint32_t index = 0; index < size; index++) {
        rmw_total += memoryController->rmw->rmw_que_cnt.at(index);
        STATE_PRINTN(setw(7)<<index<<"|");
    }
    STATE_PRINTN(endl);
    STATE_PRINTN(setw(7)<<"Per"<<"|");
    for (uint32_t index = 0; index < size; index ++) {
        float rmw_cnt_dist_ratio = (float(memoryController->rmw->rmw_que_cnt.at(index)) * 100) / rmw_total;
        STATE_PRINTN(setw(7)<<fixed<<setprecision(3)<<rmw_cnt_dist_ratio<<"|");
    }
    STATE_PRINTN(endl);
    STATE_PRINTN(setw(7)<<"Cycle"<<"|");
    for (uint32_t index = 0; index < size; index ++) {
        STATE_PRINTN(setw(7)<<fixed<<setprecision(3)<<memoryController->rmw->rmw_que_cnt.at(index)<<"|");
    }
    STATE_PRINTN(endl);
    for (uint32_t index = 0; index <= size; index ++) {
        STATE_PRINTN("--------");
    }
    STATE_PRINTN(endl);

    for (uint32_t index = 0; index < size; index++) {
        memoryController->rmw->rmw_que_cnt.at(index) = 0;
    }

    if(DEBUG_PDU){
        STATE_PRINTN("------------------------- IECC HIT RATE Statistics  -----------------------------\n");
        uint32_t try_cnt=memoryController->iecc->try_count;
        uint32_t hit_cnt=memoryController->iecc->rhit;
        STATE_PRINTN("IECC Try Hit count : "<< try_cnt<<endl);
        STATE_PRINTN("IECC Read Hit count : "<< hit_cnt);
        STATE_PRINTN(", ratio : "<<100*(float)hit_cnt/try_cnt<<"%"<<endl);
        hit_cnt=memoryController->iecc->whit;
        STATE_PRINTN("IECC Write Hit count : "<< hit_cnt);
        STATE_PRINTN(", ratio : "<<100*(float)hit_cnt/try_cnt<<"%"<<endl);
        STATE_PRINTN("IECC FULL Write count : "<< memoryController->iecc->wdata_full_counter<<endl);
        STATE_PRINTN("IECC MERGE Write count : "<< memoryController->iecc->merge_cnt<<endl);
        for (auto &bufs : memoryController->iecc->wr_ecc_buf) {
            int iecc_n_cnt=0;
            uint32_t iecc_n_shift=bufs.wr_ecc_pos;
            while(iecc_n_shift){
                iecc_n_cnt++;
                iecc_n_shift=iecc_n_shift&(iecc_n_shift-1);
            }
            memoryController->iecc->wpos[iecc_n_cnt]++;
        }
        STATE_PRINTN("wr_ecc_pos with n ones: "<< endl);
        for(int iecc_i=0;iecc_i<17;iecc_i++){
            STATE_PRINTN("    "<<iecc_i<<": "<< memoryController->iecc->wpos[iecc_i]);
        }
    }
    task_cnt = 0;
    access_cnt = 0;
    bp_cnt = 0;

    //save the value of last time stataes
    pre_reads = reads;
    pre_writes = writes;
    pre_totals = totals;
    pre_address_conf_cnt = address_conf_cnt;
    pre_id_conf_cnt = id_conf_cnt;
    pre_ba_conf_cnt = ba_conf_cnt;
    pre_total_conf = total_conf;
    pre_act_cnt = act_cnt;
    pre_act_dst_cnt = act_dst_cnt;
    pre_pre_sb_cnt = pre_sb_cnt;
    pre_pre_pb_cnt = pre_pb_cnt;
    pre_pre_pb_dst_cnt = pre_pb_dst_cnt;
    pre_pre_ab_cnt = pre_ab_cnt;
    pre_read_cnt = read_cnt;
    pre_write_cnt = write_cnt;
    pre_read_p_cnt = read_p_cnt;
    pre_write_p_cnt = write_p_cnt;
    pre_mwrite_cnt = mwrite_cnt;
    pre_mwrite_p_cnt = mwrite_p_cnt;
    pre_timeout_cnt = timeout_cnt;
    pre_rt_timeout_cnt = rt_timeout_cnt;
    pre_row_hit_cnt = row_hit_cnt;
    pre_row_miss_cnt = row_miss_cnt;
    pre_ecc_read_cnt = ecc_read_cnt;
    pre_ecc_write_cnt = ecc_write_cnt;
    pre_merge_read_cnt = merge_read_cnt;
    pre_rw_switch_cnt = rw_switch_cnt;
    pre_rank_switch_cnt = rank_switch_cnt;
    pre_sid_switch_cnt = sid_switch_cnt;
    pre_refresh_pb_cnt = refresh_pb_cnt;
    pre_refresh_ab_cnt = refresh_ab_cnt;
    pre_r2w_switch_cnt = r2w_switch_cnt;
    pre_w2r_switch_cnt = w2r_switch_cnt;
    pre_phy_notlp_cnt = phy_notlp_cnt;
    pre_phy_lp_cnt = phy_lp_cnt;
    pre_acc_rank_cnt = memoryController->acc_rank_cnt;
    pre_acc_bank_cnt = memoryController->acc_bank_cnt;
    pre_racc_rank_cnt = memoryController->racc_rank_cnt;
    pre_racc_bank_cnt = memoryController->racc_bank_cnt;
    pre_wacc_rank_cnt = memoryController->wacc_rank_cnt;
    pre_wacc_bank_cnt = memoryController->wacc_bank_cnt;
    pre_pde_cnt = pde_cnt;
    pre_asrefe_cnt = asrefe_cnt;
    pre_srpde_cnt = srpde_cnt;
    pre_pdx_cnt = pdx_cnt;
    pre_asrefx_cnt = asrefx_cnt;
    pre_srpdx_cnt = srpdx_cnt;
    pre_abr_cnt = memoryController->rank_refresh_cnt;
    pre_pbr_cnt = memoryController->perbank_refresh_cnt;

    pre_rmw_reads = rmw_reads;
    pre_rmw_bypass_reads = rmw_bypass_reads;
    pre_rmw_writes = rmw_writes;
    pre_rmw_bypass_writes = rmw_bypass_writes;
    pre_rmw_full_writes = rmw_full_writes;
    pre_rmw_mask_writes = rmw_mask_writes;
    pre_rmw_totals = rmw_totals;

    for (uint8_t i = 0; i < NUM_RANKS * NUM_BANKS; i ++) {
        PreBankRowActCnt[i] = memoryController->BankRowActCnt[i];
    }
    for (uint8_t i = 0; i < 7; i ++) {
        pre_sch_level_cnt[i] = memoryController->wb->sch_level_cnt[i];
    }

    //clear statistics
    STATE_PRINTN("========================================= END =========================================\n");
    STATE_PRINTN("\n");
}
//==============================================================================
void MemorySystem::register_read(uint64_t address, uint32_t &data) {
    uint32_t offset = address & 0xff;
    switch (offset) {
        case 0x0: break;
        case 0x4: break;
        default:  break;
    }
}
//==============================================================================
void MemorySystem::register_write(uint64_t address, uint32_t data) {
    uint32_t offset = ((address != 0) ? 4 : 0);
    switch (offset) {
        case 0x0:{
            start_cycle = this->now();
            break;
        }
        case 0x4:{
            end_cycle = this->now();
            if (start_cycle != end_cycle && STATE_LOG == true) {
                statistics();
            }
            break;
        }
        default: break;
    }
}
//==============================================================================
void MemorySystem::check_cnt() {
    uint32_t queRdNum = 0;
    uint32_t queWrNum = 0;

    uint32_t size = GetTransactionLen();
    que_cnt.at(size)++;

    for (uint32_t index = 0; index < size; index++) {
        if (memoryController->transactionQueue.at(index)->transactionType == DATA_READ) {
            queRdNum++;
        } else {
            queWrNum++;
        }
    }
    que_rd_cnt.at(queRdNum)++;
    que_wr_cnt.at(queWrNum)++;
}
//==============================================================================
uint32_t MemorySystem::getTransQueSize(bool isRd) {
    uint32_t queRdNum = 0;
    uint32_t queWrNum = 0;

    uint32_t size = GetTransactionLen();

    for (uint32_t index = 0; index < size; index++) {
        if (memoryController->transactionQueue.at(index)->transactionType == DATA_READ) {
            queRdNum++;
        } else {
            queWrNum++;
        }
    }
    if (isRd) {
        return queRdNum;
    } else {
        return queWrNum;
    }
}
//==============================================================================
void MemorySystem::GetQueueCmdNum(unsigned *dmc_rd_num, unsigned *dmc_wr_num,
        unsigned *gbuf_rd_num, unsigned *gbuf_wr_num) {
    *dmc_rd_num = memoryController->que_read_cnt;
    *dmc_wr_num = memoryController->que_write_cnt;
    *gbuf_rd_num = memoryController->wb->rcmd_num();
    *gbuf_wr_num = memoryController->wb->wcmd_num();
}
//==============================================================================
void MemorySystem::UnitConvert(double *oenergy, string *ouint, double ienergy) {
    unsigned convert_cnt = 0;
    double energy = ienergy;
    while (energy >= 10000) {
        energy /= 1000;
        convert_cnt ++;
    }
    *oenergy = energy;
    if (convert_cnt == 0) *ouint = "pJ";
    else if (convert_cnt == 1) *ouint = "nJ";
    else if (convert_cnt == 2) *ouint = "uJ";
    else if (convert_cnt == 3) *ouint = "mJ";
    else if (convert_cnt == 4) *ouint = "J";
    else if (convert_cnt == 5) *ouint = "kJ";
    else {ERROR("No such unit, energy:"<<ienergy); assert(0);}
}
//==============================================================================
void MemorySystem::GetDmcBusyStatus(bool *dmc_busy) {
    unsigned cmd_cnt = 0;
    cmd_cnt = memoryController->que_read_cnt + memoryController->wb->rcmd_num();
    if (BUSYSTATE_INC_WCMD) {
        cmd_cnt += memoryController->que_write_cnt;
    }
    if (cmd_cnt >= BUSYSTATE_TH) {
        *dmc_busy = true;
    } else {
        *dmc_busy = false;
    }
}
//==============================================================================
void MemorySystem::trans_init(Transaction *trans, uint64_t inject_time) {
    trans->data_size = (trans->burst_length + 1) * DMC_DATA_BUS_BITS / 8;
    trans->inject_time = inject_time;
    if (!LAT_INC_BP) trans->reqEnterDmcBufTime = now() * tDFI;
}

//void MemorySystem::trans_check(Transaction *t) {
//    if (IS_HBM2E || IS_HBM3) {
//        if (NUM_SIDS == 3 && t->sid == 3) {
//            ERROR(setw(10)<<now()<<" Error Sid! task="<<t->task<<" address="<<hex<<t->address<<dec<<" sid="<<t->sid);
//        }
//    }
//}

//==============================================================================
// This function can be used by autoconf AC_CHECK_LIB since
// apparently it can't detect C++ functions.
// Basically just an entry in the symbol table
//==============================================================================
extern "C" {
    void libDDRSim_is_present(void) {
    }
}
}