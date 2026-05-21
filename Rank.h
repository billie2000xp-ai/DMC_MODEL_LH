#ifndef _LPDDR_RANK_H
#define _LPDDR_RANK_H

#include "SimulatorObject.h"
#include "SystemConfiguration.h"
#include "BankState.h"
#include "Transaction.h"
using namespace std;

namespace LPDDRSim {

class MemoryController; //forward declaration

class Rank : public SimulatorObject {
    private:
    int id;
    ostream &DDRSim_log;
    ostream &dram_log;
    unsigned pbr_bg_num;
    unsigned pbr_bank_num;
    unsigned sc_num;
    unsigned sc_bank_num;

    public:
    //functions
    Rank(ostream &DDRSim_log_, ostream &dram_log_);
    virtual ~Rank();
    void receiveFromBus(const BusPacket &packet);
    void attachMemoryController(MemoryController *mc);
    int getId() const;
    void setId(int id);
    void update();
    //fields
    MemoryController *memoryController;
    unsigned long long outgoingDataPacket;
    bool mask_wcmd_flag;
    //these are vectors so that each element is per-bank
    vector<DataPacket> readReturnPacket;
    uint8_t refresh_cnt;
    bool freshwating_set;//for postponed
    unsigned channel;
    uint64_t channel_ohot;

    enum DdrBankState {
        BANK_IDLE,
        BANK_ACT,
        BANK_REF,
        BANK_PRE
    };
    enum DdrRankState {
        RANK_IDLE,
        RANK_PD,
        RANK_ASREF,
        RANK_SRPD
    };
    struct SeparateRankStat {
        DdrRankState state;
        unsigned act_num;
        unsigned prepb_num;
        unsigned preab_num;
        unsigned rd16_num;
        unsigned rd32_num;
        unsigned wr16_num;
        unsigned wr32_num;
        unsigned refpb_num;
        unsigned refab_num;
        unsigned sre_num;
        unsigned srx_num;
        unsigned pde_num;
        unsigned pdx_num;
        unsigned srpd_num;
        uint64_t asref_cycle;
        uint64_t act_pd_cycle;
        uint64_t idle_pd_cycle;
        uint64_t srpd_cycle;
        uint64_t act_standby_cycle;
        uint64_t pre_standby_cycle;
        uint64_t act_cycle_bank;
        uint64_t total_byte;
        SeparateRankStat() {
            state = RANK_IDLE;
            act_num = 0;
            prepb_num = 0;
            preab_num = 0;
            rd16_num = 0;
            rd32_num = 0;
            wr16_num = 0;
            wr32_num = 0;
            refpb_num = 0;
            refab_num = 0;
            sre_num = 0;
            srx_num = 0;
            pde_num = 0;
            pdx_num = 0;
            srpd_num = 0;
            asref_cycle = 0;
            act_pd_cycle = 0;
            idle_pd_cycle = 0;
            srpd_cycle = 0;
            act_standby_cycle = 0;
            pre_standby_cycle = 0;
            act_cycle_bank = 0;
            total_byte = 0;
        }
    };
    struct dramBankState {
        DdrBankState state;
        uint64_t state_change_time;
        TransactionCmd last_cmd;
        dramBankState() {
            state = BANK_IDLE;
            state_change_time = 0;
            last_cmd = INVALID;
        }
    };

    vector <SeparateRankStat> RankStat;
    vector <dramBankState> DramBankState;
    void update_bank_state();
    void CalcDramPower();
    double CalcEnergy(string vdd, uint64_t cycles, double current, double voltage);
    struct dramPower {
        double act_energy;
        double pre_energy;
        double rd_energy;
        double wr_energy;
        double act_standby_energy;
        double pre_standby_energy;
        double refpb_energy;
        double refab_energy;
        double asref_refab_energy;
        double asref_pre_energy;
        double asref_energy;
        double act_pd_energy;
        double idle_pd_energy;
        double srpd_energy;
        map <string, double> IddEnergy;
        map <string, double> VddEnergy;
        map <string, double> AvgCurrent;
        double total_energy;
        double energy_efficiency;
        double average_power;
        dramPower() {
            act_energy = 0;
            pre_energy = 0;
            rd_energy = 0;
            wr_energy = 0;
            act_standby_energy = 0;
            pre_standby_energy = 0;
            refpb_energy = 0;
            refab_energy = 0;
            asref_refab_energy = 0;
            asref_pre_energy = 0;
            asref_energy = 0;
            act_pd_energy = 0;
            idle_pd_energy = 0;
            srpd_energy = 0;

            IddEnergy["IDD0"] = 0;
            IddEnergy["IDD2N"] = 0;
            IddEnergy["IDD2P"] = 0;
            IddEnergy["IDD3N"] = 0;
            IddEnergy["IDD3P"] = 0;
            IddEnergy["IDD4W"] = 0;
            IddEnergy["IDD4R"] = 0;
            IddEnergy["IDD5"] = 0;
            IddEnergy["IDD6"] = 0;

            VddEnergy["VDD1"] = 0;
            VddEnergy["VDD2H"] = 0;
            VddEnergy["VDD2L"] = 0;
            VddEnergy["VDDQH"] = 0;
            VddEnergy["VDDQL"] = 0;

            AvgCurrent["VDD1"] = 0;
            AvgCurrent["VDD2H"] = 0;
            AvgCurrent["VDD2L"] = 0;
            AvgCurrent["VDDQH"] = 0;
            AvgCurrent["VDDQL"] = 0;

            total_energy = 0;
            energy_efficiency = 0;
            average_power = 0;
        }
    };
    vector <dramPower> DramPower;
};
}
#endif