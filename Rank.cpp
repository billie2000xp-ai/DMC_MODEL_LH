#include "Rank.h"
#include "MemoryController.h"
#include <iomanip>
#include <cmath>
#include <assert.h>

using namespace std;
using namespace LPDDRSim;

Rank::Rank(ostream &DDRSim_log_, ostream &dram_log_) :
    id(-1),
    DDRSim_log(DDRSim_log_), dram_log(dram_log_) {
    memoryController = NULL;
    outgoingDataPacket = 0;
    channel = 0xFFFFFFFF;
    channel = 0x0;
    RankStat.resize(1);
    DramPower.resize(1);
    for (size_t i = 0; i < NUM_BANKS; i ++) {
        dramBankState state;
        DramBankState.push_back(state);
    }
    pbr_bg_num = 0;
    pbr_bank_num = 0;
    sc_num = 0;
    sc_bank_num = 0;
}

// mutators
void Rank::setId(int id) {
    this->id = id;
    channel = id / NUM_RANKS;
    channel_ohot = 1ull << channel;
}

// attachMemoryController() must be called before any other Rank functions
// are called
void Rank::attachMemoryController(MemoryController *memoryController) {
    this->memoryController = memoryController;
    pbr_bg_num = memoryController->pbr_bg_num;
    pbr_bank_num = memoryController->pbr_bank_num;
    sc_num = memoryController->sc_num;
    sc_bank_num = memoryController->sc_bank_num;
//    DEBUG(now()<<" rank_attach, sc_num="<<sc_num); 

}

Rank::~Rank() {
}

void Rank::receiveFromBus(const BusPacket &packet) {
    unsigned bankIndex = packet.bankIndex % NUM_BANKS;
    unsigned sub_channel = (packet.bankIndex % NUM_BANKS) / sc_bank_num;
    unsigned bank_start = sub_channel * NUM_BANKS / sc_num;
//    unsigned bank_pair_start = sub_channel * pbr_bank_num;
    switch (packet.type) {
        case READ_CMD :
        case READ_P_CMD : {
            unsigned BasicBL = 0;
            if (IS_LP5 || IS_GD2) {
                BasicBL = 8; // means BL16 / 2
            } else if (IS_LP6) {
                BasicBL = 12; // means BL24 / 2
            }
            float cycle_min = ceil(float(DMC_DATA_BUS_BITS)/JEDEC_DATA_BUS_BITS/WCK2DFI_RATIO/PAM_RATIO)/2;
            unsigned data_ratio = ceil(packet.trans_size * 8 * PAM_RATIO / DMC_DATA_BUS_BITS);
            DataPacket dp;
            dp.task = packet.task;
            dp.mask_wcmd = packet.mask_wcmd;
            for (size_t i = 1; i <= data_ratio; i ++) {
                if ((IS_LP5 || IS_LP6 || IS_GD2) &&
                        NUM_GROUPS != 1 && unsigned((cycle_min * i) * WCK2DFI_RATIO) > BasicBL) {
                    dp.delay = RL + ceil(cycle_min * i) + BasicBL / unsigned(WCK2DFI_RATIO);
                } else {
                    dp.delay = RL + ceil(cycle_min * i);
                }
                readReturnPacket.push_back(dp);
            }
            if (packet.bl == 16) RankStat[0].rd16_num ++;
            else if (packet.bl == 32) RankStat[0].rd32_num ++;
            RankStat[0].total_byte += packet.bl * JEDEC_DATA_BUS_BITS / 8;
            if (packet.type == READ_P_CMD) {
                RankStat[0].prepb_num ++;
                DramBankState[bankIndex].state = BANK_PRE;
                if ((now() + PCFG_TRTP) >= DramBankState[bankIndex].state_change_time) {
                    DramBankState[bankIndex].state_change_time = now() + PCFG_TRTP + tRPpb;
                } else {
                    DramBankState[bankIndex].state_change_time += tRPpb;
                }
            }
            DramBankState[bankIndex].last_cmd = packet.type;
            break;
        }

        case WRITE_CMD :
        case WRITE_P_CMD :
        case WRITE_MASK_CMD :
        case WRITE_MASK_P_CMD : {
            if (packet.bl == 16) RankStat[0].wr16_num ++;
            else if (packet.bl == 32) RankStat[0].wr32_num ++;
            RankStat[0].total_byte += packet.bl * JEDEC_DATA_BUS_BITS / 8;
            if (packet.type == WRITE_P_CMD || packet.type == WRITE_MASK_P_CMD) {
                RankStat[0].prepb_num ++;
                DramBankState[bankIndex].state = BANK_PRE;
                if ((now() + PCFG_TRTP) >= DramBankState[bankIndex].state_change_time) {
                    DramBankState[bankIndex].state_change_time = now() + PCFG_TRTP + tRPpb;
                } else {
                    DramBankState[bankIndex].state_change_time += tRPpb;
                }
            }
            DramBankState[bankIndex].last_cmd = packet.type;
            break;
        }
        case ACTIVATE2_CMD : {
            RankStat[0].act_num ++;
            DramBankState[bankIndex].state = BANK_ACT;
            DramBankState[bankIndex].last_cmd = packet.type;
            DramBankState[bankIndex].state_change_time = now() + tRAS;
            break;
        }
        case PRECHARGE_PB_CMD : {
            RankStat[0].prepb_num ++;
            DramBankState[bankIndex].state = BANK_PRE;
            DramBankState[bankIndex].state_change_time = now() + tRPpb;
            DramBankState[bankIndex].last_cmd = packet.type;
            break;
        }
        case PRECHARGE_AB_CMD : {     //todo: revise for e-mode
            for (size_t bank = 0; bank < NUM_BANKS/sc_num; bank ++) {
                if (DramBankState[bank + bank_start].state == BANK_ACT) {
                    RankStat[0].preab_num ++;
                }
                DramBankState[bank + bank_start].state = BANK_PRE;
                DramBankState[bank + bank_start].state_change_time = now() + tRPab;
                DramBankState[bank + bank_start].last_cmd = packet.type;
            }
            break;
        }
        case REFRESH_CMD : {       //todo: revise for e-mode
            RankStat[0].refab_num ++;
            for (size_t bank = 0; bank < NUM_BANKS/sc_num; bank ++) {
                DramBankState[bank + bank_start].state = BANK_REF;
                DramBankState[bank + bank_start].state_change_time = now() + tRFCab;
                DramBankState[bank + bank_start].last_cmd = packet.type;
            }
            break;
        }
        case PER_BANK_REFRESH_CMD : {
            RankStat[0].refpb_num ++;
            if (!ENH_PBR_EN) {
                for (size_t pbr_bg = 0; pbr_bg < pbr_bg_num; pbr_bg ++) {
                    unsigned pbr_ba = bankIndex + pbr_bg * pbr_bank_num;
                    DramBankState[pbr_ba].state = BANK_REF;
                    DramBankState[pbr_ba].state_change_time = now() + tRFCpb;
                    DramBankState[pbr_ba].last_cmd = packet.type;
                }
            } else {
                unsigned pbr_ba_fst = packet.fst_bankIndex % NUM_BANKS;
                unsigned pbr_ba_lst = packet.lst_bankIndex % NUM_BANKS;
                DramBankState[pbr_ba_fst].state = BANK_REF;
                DramBankState[pbr_ba_fst].state_change_time = now() + tRFCpb;
                DramBankState[pbr_ba_fst].last_cmd = packet.type;
                DramBankState[pbr_ba_lst].state = BANK_REF;
                DramBankState[pbr_ba_lst].state_change_time = now() + tRFCpb;
                DramBankState[pbr_ba_lst].last_cmd = packet.type;
            }
            break;
        }
        case ACTIVATE2_DST_CMD : {
            RankStat[0].refpb_num ++;
            DramBankState[bankIndex].state = BANK_REF;
            DramBankState[bankIndex].state_change_time = now() + tRAS + tRPpb;
            DramBankState[bankIndex].last_cmd = packet.type;
            break;
        }
        case PRECHARGE_SB_CMD : {
            break;
        }
        case PRECHARGE_PB_DST_CMD : {
            DramBankState[bankIndex].state = BANK_PRE;
            DramBankState[bankIndex].state_change_time = now() + tRPpb;
            DramBankState[bankIndex].last_cmd = packet.type;
            break;
        }
        case PD_CMD : {
            RankStat[0].pde_num ++;
            RankStat[0].state = RANK_PD;
            break;
        }
        case SRPDX_CMD : {
            RankStat[0].pdx_num ++;
            RankStat[0].state = RANK_ASREF;
            break;
        }
        case PDX_CMD : {
            RankStat[0].pdx_num ++;
            RankStat[0].state = RANK_IDLE;
            break;
        }
        case ASREF_CMD : {
            RankStat[0].sre_num ++;
            RankStat[0].state = RANK_ASREF;
            break;
        }
        case ASREFX_CMD : {
            RankStat[0].srx_num ++;
            RankStat[0].state = RANK_IDLE;
            break;
        }
        case SRPD_CMD : {
            RankStat[0].srpd_num ++;
            RankStat[0].state = RANK_SRPD;
            break;
        }
        case ACTIVATE1_CMD :
        case PDE_CMD :
        case SRPDE_CMD :
        case ASREFE_CMD :
        case ACTIVATE1_DST_CMD : {
            break;
        }
        default : {
            ERROR(setw(10)<<now()<<" -- Error, Unknown BusPacketType, task="<<packet.task
                    <<", cmd="<<packet.type<<", chnl="<<id);
            assert(0);
            break;
        }
    }
    // ACT, RD, WR, REFpb, REFab, PREpb, PREab, RDAP, WRAP, SRE, SRX, PDE, PDX, SREPD
    if (PRINT_DRAM_TRACE && channel_ohot == (channel_ohot & PRINT_CH_OHOT)) {
        unsigned bank = packet.bankIndex % NUM_BANKS;
        unsigned rank = packet.rank;
        uint64_t cur_time = 0;
        if (IS_LP5 || IS_LP6 || IS_GD2) {
            cur_time = now();
        } else {
            cur_time = now() * uint64_t(WCK2DFI_RATIO);
        }
        switch (packet.type) {
            case READ_CMD : {
                DRAM_PRINT(cur_time<<",RD"<<packet.bl<<","<<bank);
                break;
            }
            case READ_P_CMD : {
                DRAM_PRINT(cur_time<<",RD"<<packet.bl<<"AP,"<<bank);
                break;
            }
            case WRITE_CMD :
            case WRITE_MASK_CMD : {
                DRAM_PRINT(cur_time<<",WR"<<packet.bl<<","<<bank);
                break;
            }
            case WRITE_P_CMD :
            case WRITE_MASK_P_CMD : {
                DRAM_PRINT(cur_time<<",WR"<<packet.bl<<"AP,"<<bank);
                break;
            }
            case ACTIVATE1_CMD :
            case ACTIVATE1_DST_CMD : {
                break;
            }
            case ACTIVATE2_CMD : {
                DRAM_PRINT(cur_time<<",ACT,"<<bank);
                break;
            }
            case ACTIVATE2_DST_CMD : {
                DRAM_PRINT(cur_time<<",ACT_dst,"<<bank);
                break;
            }
            case PRECHARGE_SB_CMD : {
                DRAM_PRINT(cur_time<<",PREsb,"<<bank);
                break;
            }
            case PRECHARGE_PB_CMD : {
                if (packet.fg_ref) {DRAM_PRINT(cur_time<<",PREpbfg,"<<bank);}
                else {DRAM_PRINT(cur_time<<",PREpb,"<<bank);}
                break;
            }
            case PRECHARGE_AB_CMD : {
                DRAM_PRINT(cur_time<<",PREab,"<<rank);
                break;
            }
            case REFRESH_CMD : {
                DRAM_PRINT(cur_time<<",REFab,"<<rank);
                break;
            }
            case PER_BANK_REFRESH_CMD : {
                DRAM_PRINT(cur_time<<",REFpb,"<<bank);
                break;
            }
            case PD_CMD : {
                DRAM_PRINT(cur_time<<",PDE,"<<rank);
                break;
            }
            case PDX_CMD : {
                DRAM_PRINT(cur_time<<",PDX,"<<rank);
                break;
            }
            case ASREF_CMD : {
                DRAM_PRINT(cur_time<<",SRE,"<<rank);
                break;
            }
            case ASREFX_CMD : {
                DRAM_PRINT(cur_time<<",SRX,"<<rank);
                break;
            }
            case SRPD_CMD : {
                DRAM_PRINT(cur_time<<",SREPD,"<<rank);
                break;
            }
            case SRPDX_CMD : {
                DRAM_PRINT(cur_time<<",PDX,"<<rank);
                break;
            }
            case PDE_CMD :
            case ASREFE_CMD :
            case SRPDE_CMD :
            case PRECHARGE_PB_DST_CMD : {
                break;
            }
            default : {
                ERROR(setw(10)<<now()<<" -- Error, Unknown BusPacketType, task="<<packet.task
                        <<", cmd="<<packet.type<<", chnl="<<id);
                assert(0);
                break;
            }
        }
    }
}

int Rank::getId() const {
    return this->id;
}

void Rank::update() {
    update_bank_state();

    if (!readReturnPacket.empty()) {
        unsigned size = readReturnPacket.size();
        for (size_t i = 0; i < size; i ++) {
            if (readReturnPacket[i].delay > 0) {
                readReturnPacket[i].delay --;
            }
        }
        for (size_t i = 0; i < size; i ++) {
            if (readReturnPacket[i].delay == 0) {
                outgoingDataPacket = readReturnPacket[i].task;
                mask_wcmd_flag     = readReturnPacket[i].mask_wcmd;
                readReturnPacket.erase(readReturnPacket.begin() + i);
                memoryController->receiveFromBus(outgoingDataPacket, mask_wcmd_flag);
                break;
            }
        }
    }
}

void Rank::update_bank_state() {      //todo: revise for e-mode
    switch (RankStat[0].state) {
        case RANK_IDLE : {
            bool is_refab = true;
            for (size_t i = 0; i < NUM_BANKS; i ++) {
                if (DramBankState[i].state == BANK_REF) continue;
                is_refab = false;
                break;
            }
            if (is_refab) break;

            bool all_bank_pre = true;
            for (size_t i = 0; i < NUM_BANKS; i ++) {
                if (DramBankState[i].state == BANK_IDLE) continue;
                all_bank_pre = false;
                RankStat[0].act_cycle_bank ++;
            }
            if (all_bank_pre) RankStat[0].pre_standby_cycle ++;
            else RankStat[0].act_standby_cycle ++;
            break;
        }
        case RANK_PD : {
            bool has_bank_act = false;
            for (size_t i = 0; i < NUM_BANKS; i ++) {
                if (DramBankState[i].state == BANK_ACT || DramBankState[i].state == BANK_REF) {
                    has_bank_act = true;
                    break;
                }
            }
            if (has_bank_act) {
                RankStat[0].act_pd_cycle ++;
            } else {
                RankStat[0].idle_pd_cycle ++;
            }
            break;
        }
        case RANK_ASREF : {
            RankStat[0].asref_cycle ++;
            break;
        }
        case RANK_SRPD : {
            RankStat[0].srpd_cycle ++;
            break;
        }
        default : break;
    }

    for (size_t i = 0; i < NUM_BANKS; i ++) {
        if (now() < DramBankState[i].state_change_time) continue;
        if (DramBankState[i].state == BANK_IDLE) continue;
        switch (DramBankState[i].last_cmd) {
            case PRECHARGE_PB_CMD :
            case PRECHARGE_AB_CMD :
            case REFRESH_CMD :
            case PER_BANK_REFRESH_CMD :
            case ACTIVATE2_DST_CMD :
            case PRECHARGE_PB_DST_CMD : {
                DramBankState[i].state = BANK_IDLE;
                break;
            }
            case READ_P_CMD :
            case WRITE_P_CMD :
            case WRITE_MASK_P_CMD : {
                DramBankState[i].state = BANK_IDLE;
                break;
            }
            default : break;
        }
    }
}

double Rank::CalcEnergy(string vdd, uint64_t cycles, double current, double voltage) {
 current = current < 0 ? 0 : current;
    double energy = static_cast<double>(cycles) * tDFI * current * voltage;
    if (DramPower[0].VddEnergy.find(vdd) == DramPower[0].VddEnergy.end()) {
        ERROR(setw(10)<<now()<<" -- Error vdd: "<<vdd);
        assert(0);
    } else {
        DramPower[0].VddEnergy[vdd] += energy;
    }
 return energy;
}

void Rank::CalcDramPower() {
    float ratio = 0;
    if (IS_LP5 || IS_LP6 || IS_GD2) {
        ratio = float(1) / OFREQ_RATIO;
    } else {
        ratio = WCK2DFI_RATIO;
    }

    // Active opertion energy
    uint64_t act_op_cycle = RankStat[0].act_num * tRAS;
    act_op_cycle = unsigned(float(act_op_cycle) * ratio);
    DramPower[0].act_energy = CalcEnergy("VDD1", act_op_cycle, IDD01 - IDD3N1, VDD1);
    DramPower[0].act_energy += CalcEnergy("VDD2H", act_op_cycle, IDD02H - IDD3N2H, VDD2H);
    DramPower[0].act_energy += CalcEnergy("VDDQH", act_op_cycle, IDD0Q - IDD3NQ, VDDQH);
    DramPower[0].act_energy += CalcEnergy("VDD2L", act_op_cycle, IDD02L - IDD3N2L, VDD2L);

    // Precharge opertion energy
    uint64_t pre_op_cycle = RankStat[0].prepb_num * tRPpb + RankStat[0].preab_num * tRPab;
    pre_op_cycle = unsigned(float(pre_op_cycle) * ratio);
    DramPower[0].pre_energy = CalcEnergy("VDD1", pre_op_cycle, IDD01 - IDD2N1, VDD1);
    DramPower[0].pre_energy += CalcEnergy("VDD2H", pre_op_cycle, IDD02H - IDD2N2H, VDD2H);
    DramPower[0].pre_energy += CalcEnergy("VDDQH", pre_op_cycle, IDD0Q - IDD2NQ, VDDQH);
    DramPower[0].pre_energy += CalcEnergy("VDD2L", pre_op_cycle, IDD02L - IDD2N2L, VDD2L);

    // Read data energy
    uint64_t rd_cycle = RankStat[0].rd16_num * BL16 / 2 / WCK2DFI_RATIO;
    rd_cycle += RankStat[0].rd32_num * BL32 / 2 / WCK2DFI_RATIO;
    rd_cycle = unsigned(float(rd_cycle) * ratio);
    if (NUM_GROUPS > 1) { // BG mode
        DramPower[0].rd_energy = CalcEnergy("VDD1", rd_cycle, IDD4R1_BG - IDD3N1, VDD1);
        DramPower[0].rd_energy += CalcEnergy("VDD2H", rd_cycle, IDD4R2H_BG - IDD3N2H, VDD2H);
        DramPower[0].rd_energy += CalcEnergy("VDDQH", rd_cycle, IDD4RQ_BG - IDD3NQ, VDDQH);
        DramPower[0].rd_energy += CalcEnergy("VDD2L", rd_cycle, IDD4R2L_BG - IDD3N2L, VDD2L);
    } else { // Bank mode
        DramPower[0].rd_energy = CalcEnergy("VDD1", rd_cycle, IDD4R1_BK - IDD3N1, VDD1);
        DramPower[0].rd_energy += CalcEnergy("VDD2H", rd_cycle, IDD4R2H_BK - IDD3N2H, VDD2H);
        DramPower[0].rd_energy += CalcEnergy("VDDQH", rd_cycle, IDD4RQ_BK - IDD3NQ, VDDQH);
        DramPower[0].rd_energy += CalcEnergy("VDD2L", rd_cycle, IDD4R2L_BK - IDD3N2L, VDD2L);
    }

    // Write data energy
    uint64_t wr_cycle = RankStat[0].wr16_num * BL16 / 2 / WCK2DFI_RATIO;
    wr_cycle += RankStat[0].wr32_num * BL32 / 2 / WCK2DFI_RATIO;
    wr_cycle = unsigned(float(wr_cycle) * ratio);
    if (NUM_GROUPS > 1) { // BG mode
        DramPower[0].wr_energy = CalcEnergy("VDD1", wr_cycle, IDD4W1_BG - IDD3N1, VDD1);
        DramPower[0].wr_energy += CalcEnergy("VDD2H", wr_cycle, IDD4W2H_BG - IDD3N2H, VDD2H);
        DramPower[0].wr_energy += CalcEnergy("VDDQH", wr_cycle, IDD4WQ_BG - IDD3NQ, VDDQH);
        DramPower[0].wr_energy += CalcEnergy("VDD2L", wr_cycle, IDD4W2L_BG - IDD3N2L, VDD2L);
    } else { // Bank mode
        DramPower[0].wr_energy = CalcEnergy("VDD1", wr_cycle, IDD4W1_BK - IDD3N1, VDD1);
        DramPower[0].wr_energy += CalcEnergy("VDD2H", wr_cycle, IDD4W2H_BK - IDD3N2H, VDD2H);
        DramPower[0].wr_energy += CalcEnergy("VDDQH", wr_cycle, IDD4WQ_BK - IDD3NQ, VDDQH);
        DramPower[0].wr_energy += CalcEnergy("VDD2L", wr_cycle, IDD4W2L_BK - IDD3N2L, VDD2L);
    }

    // Active standby energy
    uint64_t act_standby_cycle = RankStat[0].act_standby_cycle;
    double average_act_bank = (act_standby_cycle == 0) ? 0 : (double)RankStat[0].act_cycle_bank / act_standby_cycle;
    double idd1_ext_act = (IDD3N1 - IDD2N1) * (average_act_bank - 1);
    double idd2h_ext_act = (IDD3N2H - IDD2N2H) * (average_act_bank - 1);
    double iddq_ext_act = (IDD3NQ - IDD2NQ) * (average_act_bank - 1);
    double idd2l_ext_act = (IDD3N2L - IDD2N2L) * (average_act_bank - 1);
    DramPower[0].act_standby_energy = CalcEnergy("VDD1", act_standby_cycle, IDD3N1 + idd1_ext_act, VDD1);
    DramPower[0].act_standby_energy += CalcEnergy("VDD2H", act_standby_cycle, IDD3N2H + idd2h_ext_act, VDD2H);
    DramPower[0].act_standby_energy += CalcEnergy("VDDQH", act_standby_cycle, IDD3NQ + iddq_ext_act, VDDQH);
    DramPower[0].act_standby_energy += CalcEnergy("VDD2L", act_standby_cycle, IDD3N2L + idd2l_ext_act, VDD2L);

    // Precharge standby energy
    uint64_t pre_standby_cycle = RankStat[0].pre_standby_cycle;
    DramPower[0].pre_standby_energy = CalcEnergy("VDD1", pre_standby_cycle, IDD2N1, VDD1);
    DramPower[0].pre_standby_energy += CalcEnergy("VDD2H", pre_standby_cycle, IDD2N2H, VDD2H);
    DramPower[0].pre_standby_energy += CalcEnergy("VDDQH", pre_standby_cycle, IDD2NQ, VDDQH);
    DramPower[0].pre_standby_energy += CalcEnergy("VDD2L", pre_standby_cycle, IDD2N2L, VDD2L);

    // REFpb energy
    unsigned trfcpb = (IS_GD2) ? (tRAS + tRPpb) : tRFCpb;
    uint64_t refpb_cycle = RankStat[0].refpb_num * trfcpb;
    refpb_cycle = unsigned(float(refpb_cycle) * ratio);
    unsigned trefipb = ceil(float(tREFI) / pbr_bank_num);
    double idd_pb1 = 0, idd_pb2 = 0, idd_pbq = 0, idd_pb2pl = 0;
    if (!IS_GD2) {
        idd_pb1 = (IDD5PB1 * (double)trefipb - IDD2N1 * (double)(trefipb - tRFCpb)) / (double)tRFCpb - IDD3N1;
        idd_pb2 = (IDD5PB2H * (double)trefipb - IDD2N2H * (double)(trefipb - tRFCpb)) / (double)tRFCpb - IDD3N2H;
        idd_pbq = (IDD5PBQ * (double)trefipb - IDD2NQ * (double)(trefipb - tRFCpb)) / (double)tRFCpb - IDD3NQ;
        idd_pb2pl = 0.01 - IDD3N2L; // define to compatable with GD2
    } else {
        idd_pb1 = IDD01 - IDD3N1;
        idd_pb2 = IDD02H - IDD3N2H;
        idd_pb2pl = IDD02L - IDD3N2L;
        idd_pbq = IDD0Q - IDD3NQ;
    }
    DramPower[0].refpb_energy = CalcEnergy("VDD1", refpb_cycle, idd_pb1, VDD1);
    DramPower[0].refpb_energy += CalcEnergy("VDD2H", refpb_cycle, idd_pb2, VDD2H);
    DramPower[0].refpb_energy += CalcEnergy("VDDQH", refpb_cycle, idd_pbq, VDDQH);
    DramPower[0].refpb_energy += CalcEnergy("VDD2L", refpb_cycle, idd_pb2pl, VDD2L);

    // REFab energy
    uint64_t refab_cycle = RankStat[0].refab_num * tRFCab;
    refab_cycle = unsigned(float(refab_cycle) * ratio);
    DramPower[0].refab_energy = CalcEnergy("VDD1", refab_cycle, IDD51, VDD1);
    DramPower[0].refab_energy += CalcEnergy("VDD2H", refab_cycle, IDD52H, VDD2H);
    DramPower[0].refab_energy += CalcEnergy("VDDQH", refab_cycle, IDD5Q, VDDQH);

//    // Refab in asref energy
//    uint64_t asref_cycle = RankStat[0].asref_cycle;
//    asref_cycle = unsigned(float(asref_cycle) * ratio);
//    uint64_t asref_refab_cycle = (asref_cycle / tREFI) * tRFCab;
//    DramPower[0].asref_refab_energy = CalcEnergy("VDD1", asref_refab_cycle, IDD51, VDD1);
//    DramPower[0].asref_refab_energy += CalcEnergy("VDD2H", asref_refab_cycle, IDD52H, VDD2H);
//    DramPower[0].asref_refab_energy += CalcEnergy("VDDQH", asref_refab_cycle, IDD5Q, VDDQH);
//    // Idle in asref energy
//    uint64_t asref_pre_cycle = asref_cycle - asref_refab_cycle;
//    DramPower[0].asref_pre_energy = CalcEnergy("VDD1", asref_pre_cycle, IDD2N1, VDD1);
//    DramPower[0].asref_pre_energy += CalcEnergy("VDD2H", asref_pre_cycle, IDD2N2H, VDD2H);
//    DramPower[0].asref_pre_energy += CalcEnergy("VDDQH", asref_pre_cycle, IDD2NQ, VDDQH);
//    DramPower[0].asref_energy = DramPower[0].asref_refab_energy + DramPower[0].asref_pre_energy;

    // Asref energy, IDD6 + (IDD2N - IDD2P), as Idle non-pd current - Idle pd current
    uint64_t asref_cycle = RankStat[0].asref_cycle;
    asref_cycle = unsigned(float(asref_cycle) * ratio);
    DramPower[0].asref_energy = CalcEnergy("VDD1", asref_cycle, IDD61 + IDD2N1 - IDD2P1, VDD1);
    DramPower[0].asref_energy += CalcEnergy("VDD2H", asref_cycle, IDD62H + IDD2N2H - IDD2P2H, VDD2H);
    DramPower[0].asref_energy += CalcEnergy("VDDQH", asref_cycle, IDD6Q + IDD2NQ - IDD2PQ, VDDQH);

    // Active pd energy
    uint64_t act_pd_cycle = RankStat[0].act_pd_cycle;
    act_pd_cycle = unsigned(float(act_pd_cycle) * ratio);
    DramPower[0].act_pd_energy = CalcEnergy("VDD1", act_pd_cycle, IDD3P1, VDD1);
    DramPower[0].act_pd_energy += CalcEnergy("VDD2H", act_pd_cycle, IDD3P2H, VDD2H);
    DramPower[0].act_pd_energy += CalcEnergy("VDDQH", act_pd_cycle, IDD3PQ, VDDQH);
    DramPower[0].act_pd_energy += CalcEnergy("VDD2L", act_pd_cycle, IDD3P2L, VDD2L);

    // Idle pd energy
    uint64_t idle_pd_cycle = RankStat[0].idle_pd_cycle;
    idle_pd_cycle = unsigned(float(idle_pd_cycle) * ratio);
    DramPower[0].idle_pd_energy = CalcEnergy("VDD1", idle_pd_cycle, IDD2P1, VDD1);
    DramPower[0].idle_pd_energy += CalcEnergy("VDD2H", idle_pd_cycle, IDD2P2H, VDD2H);
    DramPower[0].idle_pd_energy += CalcEnergy("VDDQH", idle_pd_cycle, IDD2PQ, VDDQH);
    DramPower[0].idle_pd_energy += CalcEnergy("VDD2L", idle_pd_cycle, IDD2P2L, VDD2L);

    // Sr pd energy
    uint64_t srpd_cycle = RankStat[0].srpd_cycle;
    srpd_cycle = unsigned(float(srpd_cycle) * ratio);
    DramPower[0].srpd_energy = CalcEnergy("VDD1", srpd_cycle, IDD61, VDD1);
    DramPower[0].srpd_energy += CalcEnergy("VDD2H", srpd_cycle, IDD62H, VDD2H);
    DramPower[0].srpd_energy += CalcEnergy("VDDQH", srpd_cycle, IDD6Q, VDDQH);

    // IDD energy
    DramPower[0].IddEnergy["IDD0"] = DramPower[0].act_energy + DramPower[0].pre_energy;
    DramPower[0].IddEnergy["IDD2N"] = DramPower[0].asref_pre_energy + DramPower[0].pre_standby_energy;
    DramPower[0].IddEnergy["IDD2P"] = DramPower[0].idle_pd_energy;
    DramPower[0].IddEnergy["IDD3N"] = DramPower[0].act_standby_energy;
    DramPower[0].IddEnergy["IDD3P"] = DramPower[0].act_pd_energy;
    DramPower[0].IddEnergy["IDD4W"] = DramPower[0].wr_energy;
    DramPower[0].IddEnergy["IDD4R"] = DramPower[0].rd_energy;
    DramPower[0].IddEnergy["IDD5"] = DramPower[0].refpb_energy + DramPower[0].refab_energy
            + DramPower[0].asref_refab_energy;
    DramPower[0].IddEnergy["IDD6"] = DramPower[0].srpd_energy;
    double IddTotalEnergy = 0;
    for (auto idd : DramPower[0].IddEnergy) IddTotalEnergy += idd.second;

    // VDD averge current
    DramPower[0].AvgCurrent["VDD1"] = DramPower[0].VddEnergy["VDD1"] / (double)now() / tDFI / VDD1;
    DramPower[0].AvgCurrent["VDD2H"] = DramPower[0].VddEnergy["VDD2H"] / (double)now() / tDFI / VDD2H;
    DramPower[0].AvgCurrent["VDD2L"] = DramPower[0].VddEnergy["VDD2L"] / (double)now() / tDFI / VDD2H;
    DramPower[0].AvgCurrent["VDDQH"] = DramPower[0].VddEnergy["VDDQH"] / (double)now() / tDFI / VDDQH;
    DramPower[0].AvgCurrent["VDDQL"] = DramPower[0].VddEnergy["VDDQL"] / (double)now() / tDFI / VDDQL;

    // Total energy
    DramPower[0].total_energy = DramPower[0].VddEnergy["VDD1"];
    DramPower[0].total_energy += DramPower[0].VddEnergy["VDD2H"];
    DramPower[0].total_energy += DramPower[0].VddEnergy["VDD2L"];
    DramPower[0].total_energy += DramPower[0].VddEnergy["VDDQH"];
    DramPower[0].total_energy += DramPower[0].VddEnergy["VDDQL"];

    // Energy efficiency
    DramPower[0].energy_efficiency = (RankStat[0].total_byte == 0) ? 0 :
            DramPower[0].total_energy / 8 / (double)RankStat[0].total_byte;

    // Average power
    DramPower[0].average_power = DramPower[0].total_energy / (double)now() / tDFI;

    if (DramPower[0].IddEnergy.size() != 9) {
        ERROR(setw(10)<<now()<<" -- IddEnergy size is "<<DramPower[0].IddEnergy.size());
        assert(0);
    }
    if (DramPower[0].VddEnergy.size() != 5) {
        ERROR(setw(10)<<now()<<" -- VddEnergy size is "<<DramPower[0].VddEnergy.size());
        assert(0);
    }
    if (DramPower[0].AvgCurrent.size() != 5) {
        ERROR(setw(10)<<now()<<" -- AvgCurrent size is "<<DramPower[0].AvgCurrent.size());
        assert(0);
    }

    if (DEBUG_BUS) {
        PRINTN("-------- Rank"<<id%NUM_RANKS<<" Print Power PMU Event And Power Consumption(pJ)"<<" --------"<<endl);
        PRINTN("Active opertion -- Number: "<<RankStat[0].act_num<<", Cycle: "<<act_op_cycle<<", Energy: "
                <<DramPower[0].act_energy<<endl);
        PRINTN("Precharge opertion -- Number: "<<(RankStat[0].preab_num+RankStat[0].prepb_num)<<", Cycle: "
                <<pre_op_cycle<<", Energy: "<<DramPower[0].pre_energy<<endl);
        PRINTN("Read opertion -- Number: "<<(RankStat[0].rd16_num+RankStat[0].rd32_num)<<", Cycle: "<<rd_cycle
                <<", Energy: "<<DramPower[0].rd_energy<<endl);
        PRINTN("Write opertion -- Number: "<<(RankStat[0].wr16_num+RankStat[0].wr32_num)<<", Cycle: "<<wr_cycle
                <<", Energy: "<<DramPower[0].wr_energy<<endl);
        PRINTN("Active Standby -- Cycle: "<<act_standby_cycle<<", Energy: "<<DramPower[0].act_standby_energy
                <<", Average_act_bank: "<<average_act_bank<<endl);
        PRINTN("Precharge Standby -- Cycle: "<<pre_standby_cycle<<", Energy: "<<DramPower[0].pre_standby_energy<<endl);
        PRINTN("Perbank Refresh -- Number: "<<RankStat[0].refpb_num<<", Cycle: "<<refpb_cycle<<", Energy: "
                <<DramPower[0].refpb_energy<<endl);
        PRINTN("Allbank Refresh -- Number: "<<RankStat[0].refab_num<<", Cycle: "<<refab_cycle<<", Energy: "
                <<DramPower[0].refab_energy<<endl);
//        PRINTN("Auto Self Refresh -- Number: "<<RankStat[0].sre_num<<", Ref_cycle: "<<asref_refab_cycle
//                <<", Idle_cycle: "<<asref_pre_cycle<<", Asref_ycle: "<<asref_cycle<<", Ref_energy: "
//                <<DramPower[0].asref_refab_energy<<", Idle_energy: "<<DramPower[0].asref_pre_energy
//                <<", Asref_energy: "<<DramPower[0].asref_energy<<endl);
        PRINTN("Auto Self Refresh -- Number: "<<RankStat[0].sre_num<<", Ref_cycle: "<<", Asref_ycle: "<<asref_cycle
                <<", Ref_energy: "<<DramPower[0].asref_refab_energy<<", Idle_energy: "<<DramPower[0].asref_pre_energy
                <<", Asref_energy: "<<DramPower[0].asref_energy<<endl);
        PRINTN("Power Down -- Act_pd_cycle: "<<act_pd_cycle<<", Idle_pd_cycle: "<<idle_pd_cycle<<", Srpd_cycle: "
                <<srpd_cycle<<", Act_pd_energy: "<<DramPower[0].act_pd_energy<<", Idle_pd_energy: "
                <<DramPower[0].idle_pd_energy<<", Srpd_energy: "<<DramPower[0].srpd_energy<<endl);
        PRINTN("VDD Energy -- VDD1: "<<DramPower[0].VddEnergy["VDD1"]<<", VDD2H: "<<DramPower[0].VddEnergy["VDD2H"]
                <<", VDD2L: "<<DramPower[0].VddEnergy["VDD2L"]<<", VDDQH: "<<DramPower[0].VddEnergy["VDDQH"]
                <<", VDDQL: "<<DramPower[0].VddEnergy["VDDQL"]<<endl);
        PRINTN("IDD Energy -- IDD0: "<<DramPower[0].IddEnergy["IDD0"]<<", IDD2N: "<<DramPower[0].IddEnergy["IDD2N"]
                <<", IDD2P: "<<DramPower[0].IddEnergy["IDD2P"]<<", IDD3N: "<<DramPower[0].IddEnergy["IDD3N"]
                <<" IDD3P: "<<DramPower[0].IddEnergy["IDD3P"]<<endl);
        PRINTN("IDD Energy -- IDD4W: "<<DramPower[0].IddEnergy["IDD4W"]<<", IDD4R: "<<DramPower[0].IddEnergy["IDD4R"]
                <<", IDD5: "<<DramPower[0].IddEnergy["IDD5"]<<", IDD6: "<<DramPower[0].IddEnergy["IDD6"]
                <<", IddTotalEnergy: "<<IddTotalEnergy<<endl);
        PRINTN("Averge Current -- VDD1: "<<DramPower[0].AvgCurrent["VDD1"]<<", VDD2H: "
                <<DramPower[0].AvgCurrent["VDD2H"]<<", VDD2L: "<<DramPower[0].AvgCurrent["VDD2L"]
                <<", VDDQH: "<<DramPower[0].AvgCurrent["VDDQH"]<<", VDDQL: "<<DramPower[0].AvgCurrent["VDDQL"]<<endl);
        PRINTN("Total Energy -- Energy: "<<DramPower[0].total_energy<<"(pJ), Effi: "<<DramPower[0].energy_efficiency
                <<"(pJ/bit), Avg_pwr: "<<DramPower[0].average_power<<"(mW)"<<endl);
    }
}