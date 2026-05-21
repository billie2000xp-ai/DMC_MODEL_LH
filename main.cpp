#include <stdio.h>
#include "test.h"
#include <list>
#include <time.h>
#include <bitset>
#include <random>

using namespace LPDDRSim;

LPMemorySystemTop *mem;
vector <vector <hha_command>> CommandQueue;
vector <hha_command> CommandQueueTest;
typedef std::list<int> LISTINT;
std::vector <wdata> write_task;
std::map <uint64_t, uint32_t> OutstandingQueue;
std::map <uint64_t, uint32_t> OutstandingQueueTest;
std::vector <vector <fastwakeup>> FastWakeupQueue;

uint32_t data_cnt = 0;
uint64_t task_cnt = 0;
uint64_t r_task = 0;
uint64_t cnt = 0;
uint64_t total_bytes = 0;
uint64_t trace_send_cnt = 0;
uint32_t pre_channel = 0;


ifstream file;

float calc_effi() {
    if (IS_G3D) {
        return (float(100 * total_bytes * 8) / cnt / 2 / JEDEC_DATA_BUS_BITS
                / WCK2DFI_RATIO / PAM_RATIO / float(NUM_GROUPS));
    } else if (IS_LP6) {
        return (float(100 * total_bytes * 8) / cnt / 2 / JEDEC_DATA_BUS_BITS
                / WCK2DFI_RATIO / PAM_RATIO * 9 / 8);
    } else {
        return (float(100 * total_bytes * 8) / cnt / 2 / JEDEC_DATA_BUS_BITS
                / WCK2DFI_RATIO / PAM_RATIO);
    }
}

/* callback functors */
bool some_object::read_data(unsigned channel, uint64_t task, double readDataEnterDmcTime,
        double reqAddToDmcTime, double reqEnterDmcBufTime) {
    total_bytes += DMC_DATA_BUS_BITS / 8;
    if (LATENCY_MODE) {
        float latency = 0;
        float efficiency = 0;
        static uint32_t loadlatency_cnt = 0;
        static unsigned loadlatency_cycle = 0;
        static float total_latency = 0;

        if (task >= 0xF000000000000000) {
            auto it = OutstandingQueueTest.find(task);
            if (it != OutstandingQueueTest.end()) {
                loadlatency_cnt ++;
                total_latency += ceil((readDataEnterDmcTime - reqEnterDmcBufTime) / tDFI);
                if (loadlatency_cnt == T_NUM) {
                    loadlatency_cycle = total_latency * tDFI / 0.3195;
                    DEBUG("Core 0 exit after "<<loadlatency_cycle<<" cycles, "
                        <<loadlatency_cnt<<" instructions, 16921 uops, 0.0011 ipc");
                    latency = float(total_latency) / loadlatency_cnt;
                    efficiency = calc_effi();
                    DEBUG("Done, time: "<<cnt<<", efficiency: "<<fixed<<setprecision(2)<<efficiency
                            <<"%, average latency: "<<latency<<", tDFI: "<<tDFI<<"ns");
                    DEBUG("Power Consumption: "<<fixed<<mem->channels[0]->memoryController->calc_power());
                    mem->~LPMemorySystemTop();
                    exit(0);
                }
                OutstandingQueueTest.erase(task);
            }
        }
    }
    if (task < 0xF000000000000000) {
        OutstandingQueue[task] --;
        if (OutstandingQueue[task] == 0) {
//            if (task==98) {
//                DEBUG(" task="<<task<<" address="<< &OutstandingQueue[task]); 
//            }
//            for (auto &t : OutstandingQueue){
//                if (t.first == 98){
//                    DEBUG(" task="<<task<<" first address="<< &(t.first)<<" second address"<<&(t.second)); 
//                }
//            }
            OutstandingQueue.erase(task);
        }
    }
    return true;
}

bool some_object::write_response(unsigned channel, uint64_t task, double readDataEnterDmcTime_,
        double reqAddToDmcTime_, double reqEnterDmcBufTime_) {
    for (auto w : write_task) {
        if (task == w.task) {
            ERROR(setw(10)<<cnt<<" -- task="<<task<<", Wresp receive before all wdata send out!");
            assert(0);
        }
    }
    return true;
}

bool some_object::read_response(unsigned channel, uint64_t task, double readDataEnterDmcTime,
        double reqAddToDmcTime, double reqEnterDmcBufTime) {
    return true;
}

bool some_object::cmd_response(unsigned channel, uint64_t task, double readDataEnterDmcTime,
        double reqAddToDmcTime, double reqEnterDmcBufTime) {
    return true;
}

void get_line() {
    string line;
    uint64_t f_address = 0;
    double f_time = 0;
    string f_type;
    uint32_t f_data_length, f_qos, f_mid, f_ch_num;
    size_t space_pos_start, space_pos_end;
    static double start_time = 0;
    uint32_t line_number = 0;

    while (!file.eof()) {
        if (TRACE_CMD_NUM != 0 && line_number > TRACE_CMD_NUM) break;
        getline(file, line);
        if (line_number != 0 && line.size() != 0) {
            space_pos_start = 0;
            space_pos_end = 0;

            //type
            space_pos_end = line.find(" ", space_pos_start + 1);
            istringstream iss_type(line.substr(space_pos_start, space_pos_end));
            iss_type >> dec >> f_type;

            //address
            space_pos_start = space_pos_end + 1;
            space_pos_end = line.find(" ", space_pos_start + 1);
            istringstream iss_address(line.substr(space_pos_start, space_pos_end));
            iss_address >> hex >> f_address;

            //data_len
            space_pos_start = space_pos_end + 1;
            space_pos_end = line.find(" ", space_pos_start + 1);
            istringstream iss_data_length(line.substr(space_pos_start, space_pos_end));
            iss_data_length >> dec >> f_data_length;

            //burst_size
            space_pos_start = space_pos_end + 1;
            space_pos_end = line.find(" ", space_pos_start + 1);
            istringstream iss_burst_length(line.substr(space_pos_start, space_pos_end));

            //mid
            space_pos_start = space_pos_end + 1;
            space_pos_end = line.find(" ", space_pos_start + 1);
            istringstream iss_mid(line.substr(space_pos_start, space_pos_end));
            iss_mid >> dec >> f_mid;

            //delay
            space_pos_start = space_pos_end + 1;
            space_pos_end = line.find(" ", space_pos_start + 1);

            //ATIME
            space_pos_start = space_pos_end + 1;
            space_pos_end = line.find(" ", space_pos_start + 1);
            istringstream iss_time(line.substr(space_pos_start, space_pos_end));
            iss_time >> dec >> f_time;
            if (line_number == 1) start_time = f_time;
            f_time = f_time - start_time;

            //ch_num/
            space_pos_start = space_pos_end + 1;
            space_pos_end = line.find(" ", space_pos_start + 1);
            istringstream iss_ch_num(line.substr(space_pos_start, space_pos_end));
            iss_ch_num >> dec >> f_ch_num;

            //qos
            space_pos_start = space_pos_end + 1;
            space_pos_end = line.find(" ", space_pos_start + 1);
            istringstream iss_qos(line.substr(space_pos_start, space_pos_end));
            iss_qos >> dec >> f_qos;

            // lp5 adjust adress to 32B aligned
            f_address = f_address & (~(static_cast<uint64_t>(DMC_DATA_BUS_BITS)/8 - 1));
            bool valid_command = false;
            if (TRACE_EN) {
                if (f_ch_num <= VLD_CH_NUM) valid_command = true;
            } else {
                unsigned intlv_ch_num = 0;
                if (CH_XOR_EN) {
                    if (CHINTLV_BIT == 2) { // addr7/8
                        intlv_ch_num = bitset<64>(f_address).test(7) ^ bitset<64>(f_address).test(9)
                                ^ bitset<64>(f_address).test(11) ^ bitset<64>(f_address).test(13);
                        intlv_ch_num |= (bitset<64>(f_address).test(8) ^ bitset<64>(f_address).test(10)
                                ^ bitset<64>(f_address).test(12) ^ bitset<64>(f_address).test(14)) << 1;
                    } else if (CHINTLV_BIT == 3) { // addr7/8/9
                        intlv_ch_num = bitset<64>(f_address).test(7) ^ bitset<64>(f_address).test(10)
                                ^ bitset<64>(f_address).test(13);
                        intlv_ch_num |= (bitset<64>(f_address).test(8) ^ bitset<64>(f_address).test(11)
                                ^ bitset<64>(f_address).test(14)) << 1;
                        intlv_ch_num |= (bitset<64>(f_address).test(9) ^ bitset<64>(f_address).test(12)) << 2;
                    } else {
                        ERROR("CHINTLV_BIT must be 2 or 3, now is "<<CHINTLV_BIT);
                        assert(0);
                    }
                } else {
                    for (size_t i = CHINTLV_START; i < (CHINTLV_START + CHINTLV_BIT); i ++) {
                        intlv_ch_num = (intlv_ch_num << 1) | bitset<64>(f_address).test(i);
                    }
                }
                if (intlv_ch_num <= VLD_CH_NUM) valid_command = true;
            }

            if (valid_command) {
                if (task_cnt % TRACE_Q_MAX_CNT == 0) {
                    CommandQueue.resize(task_cnt / TRACE_Q_MAX_CNT + 1);
                }
                if (r_task % TRACE_Q_MAX_CNT == 0) {
                    FastWakeupQueue.resize(r_task / TRACE_Q_MAX_CNT + 1);
                }
                hha_command transaction;
                if (TRACE_EN) {
                    transaction.address = f_address;
                } else {
                    unsigned start_bit = 0;
                    if (CH_XOR_EN) start_bit = 7;
                    else start_bit = CHINTLV_START;
                    unsigned addr_mask_low = 0;
                    for (size_t i = 0; i < start_bit; i ++) {
                        addr_mask_low = (addr_mask_low << 1) | 1;
                    }
                    transaction.address = (f_address & addr_mask_low) |
                            ((f_address>>(start_bit + CHINTLV_BIT))<<start_bit);
                }
                transaction.burst_length = ceil(float(f_data_length) * 8 / DMC_DATA_BUS_BITS) - 1;
                transaction.task = task_cnt  ;
                transaction.id = task_cnt % 1000000;
                if (f_type == "nw" || f_type == "naw") {
                    transaction.type = 1;
                } else if (f_type == "nr" || f_type == "wr") {
                    transaction.type = 0;
                    transaction.wrap_cmd = (f_type == "wr") ? true : false;
                } else {
                    ERROR("Error command type: "<<f_type);
                    assert(0);
                }
                transaction.qos = f_qos;
                transaction.mid = f_mid;
                transaction.channel = f_ch_num;
                //transaction.channel = rand() % 2;
                //rmw mask write test
                if(RMW_ENABLE && transaction.type==1) {
//                    transaction.mask_wcmd = (unsigned(rand()) % 100 >= 50);
                    transaction.mask_wcmd = false;
                }
                transaction.pri = 0x0;
                transaction.cmd_rt_type = false;
                transaction.reqEnterDmcBufTime = double(ceil(f_time * GEAR_RATIO / tDFI));
                transaction.pf_type = rand() % 4;
                transaction.sub_pftype = rand() % 13;
                transaction.sub_src = rand() % 4;
                CommandQueue[task_cnt / TRACE_Q_MAX_CNT].push_back(transaction);
                OutstandingQueue[transaction.task] = (transaction.burst_length + 1);
                task_cnt ++;
                if (FASTWAKEUP_CYCLE > 0 && transaction.type == 0) {
                    fastwakeup fw;
                    fw.rank = bitset<64>(MATRIX_RA0 & transaction.address).count() & 1;
                    if ((unsigned)transaction.reqEnterDmcBufTime <= FASTWAKEUP_CYCLE) {
                        fw.send_time = 0;
                    } else {
                        fw.send_time = ((unsigned)transaction.reqEnterDmcBufTime-FASTWAKEUP_CYCLE);
                    }
                    FastWakeupQueue[r_task / TRACE_Q_MAX_CNT].push_back(fw);
                    r_task ++;
                }
            }
        }
        line_number ++;
    }
}

void rand_command(LPMemorySystemTop *ddrc, bool is_test_cmd) {
    hha_command transaction;
    unsigned data_size = (SEQ_DATA_SIZE == 0) ? DATA_SIZE : SEQ_DATA_SIZE;
    unsigned align = log2(data_size * SEQ_NUM);
    static uint64_t r_address = 0x0;
//    static uint64_t w_address = 0x80000000;
    static uint64_t w_address = 0x00000400;
    static unsigned seq_cnt = 0;
    static bool is_read = true;
    bool is_rank0 = (unsigned(rand()) % 100 >= RK_SW_RATIO);


    if (SEQ_NUM == 0 && !is_test_cmd) {
        is_read = (unsigned(rand() % 100) >= WR_RATIO);
        if (is_read) transaction.address = r_address + data_size;
        else transaction.address = w_address + data_size;
    } else {
        if (seq_cnt == 0) {
            is_read = (unsigned(rand() % 100) >= WR_RATIO);
            uint64_t addr_rand = ((uint64_t(rand()) | uint64_t(rand()) << 31) & (0xFFFFFFFFFFFFFFFF << align))
                    % uint64_t(uint64_t(DRAM_CAPACITY) * 1024 * 1024 * 1024 / 8);
            if (RK_SW_RATIO > 0) {
                if (is_rank0) {
                    transaction.address = addr_rand & ~MATRIX_RA0;
                } else { 
                    transaction.address = addr_rand | MATRIX_RA0;
                    if (IS_LP6 && EM_ENABLE && EM_MODE==2) {      // system addres constraint
                        transaction.address = transaction.address & ~MATRIX_ROW15;
                    }
                }
            } else {
                transaction.address = addr_rand & ~MATRIX_RA0;
            }
        } else {
            if (is_read) transaction.address = r_address + DATA_SIZE;
            else transaction.address = w_address + DATA_SIZE;
        }
    }
    if (SAME_ADDR_MODE) transaction.address = 0;
    transaction.burst_length = DATA_SIZE * 8 / DMC_DATA_BUS_BITS - 1;
    transaction.id = task_cnt % 1000000;
    if(NUM_CHANS>1){  // lp6 n-mode/dynamic e-mode/combo e-mode
        transaction.channel = rand() % 2;
//        transaction.channel = 0;
    } else {
        transaction.channel = 0; // lp6 static e-mode/lp5 
    }
//    if (transaction.type == DATA_WRITE) {
//        transaction.mask_wcmd = (unsigned(rand()) % 100 >= MASK_WR_RATIO);
//    }
//    transaction.mask_wcmd = true;
    transaction.wrap_cmd = false;
    transaction.pri = 0x0;
    transaction.cmd_rt_type = false;
    // 0: Demand Read, 1: PreFetch L1, 2: PreFetch L2, 3: PreFetch L3
    transaction.pf_type = rand() % 4;
    transaction.sub_pftype = rand() % 13;
    transaction.sub_src = rand() % 4;
    if (is_test_cmd) {
        transaction.task = task_cnt | 0xF000000000000000;
        transaction.type = DATA_READ;
        transaction.qos = 3;
//        transaction.qos = 6;
//        transaction.qos =(unsigned(rand()) % 100 >= 50) ? 5 : 6;
//    transaction.mask_wcmd = (unsigned(rand()) % 100 >= 50);
//        transaction.mask_wcmd = false;
        transaction.reqEnterDmcBufTime = double(cnt + BKD_DLY);
        if (OutstandingQueueTest.size() < T_OSTD) {
            // guarantee balance between two sub channels of a die for lp6
//            if (transaction.channel == pre_channel && VLD_CH_NUM > 0) {
//                transaction.channel = transaction.channel ^ 0x00000001;
//            }
            CommandQueueTest.push_back(transaction);
//            pre_channel = transaction.channel;
            OutstandingQueueTest[transaction.task] += (transaction.burst_length + 1);
            task_cnt ++;
        }
    } else {
        transaction.task = task_cnt;
        transaction.type = is_read ? DATA_READ : DATA_WRITE;
        transaction.qos = is_read ? 3 : 0;
//        transaction.qos = is_read ? ((rand()%10) ? 3 : 0): 1;
        if (!is_read) {
            transaction.mask_wcmd = (unsigned(rand()) % 100 < MASK_WR_RATIO);
        }
        transaction.reqEnterDmcBufTime = double(cnt + T_DLY);
        if (OutstandingQueue.size() < BKD_OSTD) {
            // guarantee balance between two sub channels of a die for lp6
            if (transaction.channel == pre_channel && VLD_CH_NUM > 0) {
                transaction.channel = transaction.channel ^ 0x00000001;      // reverse
//                transaction.channel = transaction.channel ^ 0x00000000;        // unchanged 
                
            }
            CommandQueue[0].push_back(transaction);
            pre_channel = transaction.channel;
            OutstandingQueue[transaction.task] = (transaction.burst_length + 1);
            task_cnt ++;
            seq_cnt ++;
            if (seq_cnt == SEQ_NUM) seq_cnt = 0;
            if (is_read) {
                r_address = transaction.address;
                r_task ++;
            } else {
                w_address = transaction.address;
            }
        }
    }
}

void send_command(LPMemorySystemTop *ddrc) {
    hha_command transaction;
    bool test_cmd_time_met = true;

    if (!CommandQueueTest.empty()) { // Test core
        transaction = CommandQueueTest.at(0);
        if (cnt >= uint64_t(transaction.reqEnterDmcBufTime)) {
            test_cmd_time_met = false;
            bool ret = ddrc->addTransaction(transaction);
            if (ret) {
                if (transaction.type) {
                    for (size_t i = 0; i < transaction.burst_length + 1; i ++) {
                        wdata w_data;
                        w_data.task = transaction.task;
                        w_data.delay = cnt + WDATA_DLY;
                        w_data.ch = transaction.channel;
                        write_task.push_back(w_data);
                        data_cnt ++;
                    }
                }
                CommandQueueTest.erase(CommandQueueTest.begin());
            }
        }
    }

    unsigned size = CommandQueue.size();
    for (size_t i = 0; i < size; i ++) {
        if (CommandQueue[i].size() == 0) continue;
        if (test_cmd_time_met && !CommandQueue[i].empty()) {
            if (WRITE_BUFFER_ENABLE) {
                bool bus_rempty = true;
                for (auto cmd : CommandQueue[i]) {
                    if ((cnt + 40) < uint64_t(cmd.reqEnterDmcBufTime)) break;
                    if (cmd.type == DATA_WRITE) continue;
                    bus_rempty = false;
                    break;
                }
                if (FASTWAKEUP_CYCLE <= 0) mem->noc_read_inform(0, false, false, bus_rempty);
            }
            transaction = CommandQueue[i].at(0);

            if (cnt >= uint64_t(transaction.reqEnterDmcBufTime)) {
                bool ret = ddrc->addTransaction(transaction);
                if (ret) {
                    trace_send_cnt ++;
                    if (transaction.type) {
                        for (size_t i = 0; i < transaction.burst_length + 1; i ++) {
                            wdata w_data;
                            w_data.task = transaction.task;
                            w_data.delay = cnt + WDATA_DLY;
                            //uint64_t address_and = MATRIX_CH & transaction.address;
                            //w_data.ch = (bitset<64>(address_and).count() & 1);
                            w_data.ch = transaction.channel;
                            write_task.push_back(w_data);
                            data_cnt ++;
                        }
                        OutstandingQueue.erase(transaction.task);
                    }
                    CommandQueue[i].erase(CommandQueue[i].begin());
                }
            }
            break;
        }
    }
}

void send_wdata(LPMemorySystemTop *ddrc) {
    if (data_cnt != 0) {
        uint64_t task = write_task[0].task;
        unsigned ch = write_task[0].ch;
        if (cnt >= write_task[0].delay) {
            bool ret = ddrc->addData(NULL,ch,task);
            if (ret) {
                write_task.erase(write_task.begin());
                data_cnt --;
                total_bytes += DMC_DATA_BUS_BITS / 8;
            }
        }
    }
}

void parameter_check() {
    unsigned mode_cnt = 0;
    if (LATENCY_MODE) mode_cnt ++;
    if (MATCH_MODE) mode_cnt ++;
    if (TRACE_EN) mode_cnt ++;
    if (DOU_TRACE_EN) mode_cnt ++;
    if (mode_cnt > 1) {
        ERROR("Mode config true count great than 1!");
        ERROR("LATENCY_MODE: "<<LATENCY_MODE);
        ERROR("MATCH_MODE: "<<MATCH_MODE);
        ERROR("TRACE_EN: "<<TRACE_EN);
        ERROR("DOU_TRACE_EN: "<<DOU_TRACE_EN);
        assert(0);
    }
    if (SAME_ADDR_MODE && SAME_BA_RAND_ROW_MODE) {
        ERROR("Both SAME_ADDR_MODE & SAME_BA_RAND_ROW_MODE are true is not allowed!");
        assert(0);
    }
    if (FASTWAKEUP_CYCLE == 0) FASTWAKEUP_EN = false;
    else FASTWAKEUP_EN = true;
}

void print_message() {
    DEBUG("---------------------- Main Config Message ----------------------");
    DEBUG("LATENCY_MODE                     : "<<boolalpha<<LATENCY_MODE);
    DEBUG("MATCH_MODE                       : "<<boolalpha<<MATCH_MODE);
    DEBUG("BKD_OSTD                         : "<<boolalpha<<BKD_OSTD);
    DEBUG("TRACE_EN                         : "<<boolalpha<<TRACE_EN);
    DEBUG("DOU_TRACE_EN                     : "<<boolalpha<<DOU_TRACE_EN);
    DEBUG("TRACE                            : "<<boolalpha<<TRACE);
    DEBUG("GEAR_RATIO                       : "<<boolalpha<<GEAR_RATIO);
    DEBUG("SEQ_NUM                          : "<<boolalpha<<SEQ_NUM);
    DEBUG("DATA_SIZE                        : "<<boolalpha<<DATA_SIZE);
    DEBUG("RK_SW_RATIO                      : "<<boolalpha<<RK_SW_RATIO);
    DEBUG("WR_RATIO                         : "<<boolalpha<<WR_RATIO);
    DEBUG("------------------------ Now Build DDRC ------------------------");
}

void print_pass() {
    DEBUG("-----------------------------------------------------------------");
    DEBUG("Simulation Status: pass, Keyword and Place:\"PASSED\"");
    DEBUG("-----------------------------------------------------------------");
}

int main(int argc, char *argv[]) {
    get_param_path(argc, argv);
    build_cfg();
    update_cfg(argc, argv);
    print_message();

    some_object obj;
    TransactionCompleteCB * rdata_cb;
    TransactionCompleteCB * write_cb;
    TransactionCompleteCB * read_cb;
    TransactionCompleteCB * cmd_cb;
    rdata_cb = new LPDDRSim::Callback<some_object, bool, unsigned, uint64_t,
            double, double, double>(&obj,&some_object::read_data);
    write_cb = new LPDDRSim::Callback<some_object, bool, unsigned, uint64_t,
            double, double, double>(&obj,&some_object::write_response);
    read_cb = new LPDDRSim::Callback<some_object, bool, unsigned, uint64_t,
            double, double, double>(&obj,&some_object::read_response);
    cmd_cb = new LPDDRSim::Callback<some_object, bool, unsigned, uint64_t,
            double, double, double>(&obj,&some_object::cmd_response);

//    mem = new LPMemorySystemTop(0, PARAM_PATH, LOG_PATH, argc, argv);
    mem = new LPMemorySystemTop(0, PARAM_PATH, LOG_PATH, argc, argv);
    mem->RegisterCallbacks(rdata_cb, write_cb, read_cb, cmd_cb);
    parameter_check();
    CommandQueue.resize(1);

    if (TRACE_EN || DOU_TRACE_EN) {
        DEBUG("Read from trace file...");
        string filename = TRACE;
        file.open(filename.c_str());
        if (!file.is_open()) {
            ERROR("Error open log file! path="<<filename);
            assert(0);
        }
        get_line();
        DEBUG("Read trace file over!");
    } else {
        printf("DDRSim_test main()\n");
        DEBUG("Write ratio is ["<<WR_RATIO<<"%]. Now start the simulation...");
    }

    if (SAME_BA_RAND_ROW_MODE) {
        MATRIX_CH = 0x0;
        MATRIX_RA0 = 0x0;
        MATRIX_BA4 = 0x0;
        MATRIX_BA3 = 0x0;
        MATRIX_BA2 = 0x0;
        MATRIX_BA1 = 0x0;
        MATRIX_BA0 = 0x0;
        MATRIX_BG4 = 0x0;
        MATRIX_BG3 = 0x0;
        MATRIX_BG2 = 0x0;
        MATRIX_BG1 = 0x0;
        MATRIX_BG0 = 0x0;
        if (IS_GD2) {
            MATRIX_ROW1 = 0x0;
            MATRIX_ROW0 = 0x0;
        }
    }

    while (1) {
        if (STOP_WITH_STATETIME && cnt >= STOP_WINDOW * STATE_TIME) {
            if (STATE_TIME == 0) {
                ERROR("STATE_TIME=0 is not allowed with STOP mode.");
                assert(0);
            }
            if (STOP_WINDOW == 0) {
                ERROR("STOP_WINDOW=0 is not allowed with STOP mode.");
                assert(0);
            }
            float efficiency = calc_effi();
            DEBUG("Done, time: "<<cnt<<", efficiency: "<<fixed<<setprecision(2)<<efficiency<<"%");
            print_pass();
            delete mem;
            exit(0);
        }
        if (TRACE_EN || DOU_TRACE_EN) {
            unsigned fw_que_size = FastWakeupQueue.size();
            for (size_t i = 0; i < fw_que_size; i ++) {
                if (FastWakeupQueue[i].size() == 0) continue;
                while (!FastWakeupQueue[i].empty() && cnt >= FastWakeupQueue[i][0].send_time) {
                    bool rank0 = FastWakeupQueue[i][0].rank == 0 ? true : false;
                    bool rank1 = !rank0;
                    mem->noc_read_inform(0, rank0, rank1, false);
                    FastWakeupQueue[i].erase(FastWakeupQueue[i].begin());
                }
                break;
            }
            if (trace_send_cnt >= task_cnt) {
                float efficiency = 0;
                efficiency = calc_effi();
                DEBUG("Done, time: "<<cnt<<", total command cnt: "<<task_cnt<<", efficiency: "<<fixed<<setprecision(2)<<efficiency<<"%");
                DEBUG("Power Consumption: "<<fixed<<mem->channels[0]->memoryController->calc_power());
                delete mem;
                print_pass();
                exit(0);
            }
        } else {
            if (LATENCY_MODE) rand_command(mem, true);
            if (PRINT_IDLE_LAT && cnt >= 1000) exit(0);
            if (!PRINT_IDLE_LAT || (cnt % 1000 == 0)) rand_command(mem, false);
        }
        send_command(mem);
        send_wdata(mem);
        if (MATCH_MODE) {
            if (task_cnt == 20000) {
            //if (task_cnt == 100000) {
                float efficiency = 0;
                efficiency = calc_effi();
                DEBUG("Done, time: "<<cnt<<", efficiency: "<<fixed<<setprecision(2)<<efficiency<<"%");
                DEBUG("Power Consumption: "<<fixed<<mem->channels[0]->memoryController->calc_power());
                delete mem;
                print_pass();
                exit(0);
            }
        }
        mem->update();
        cnt ++;
    }
    return 0;
}