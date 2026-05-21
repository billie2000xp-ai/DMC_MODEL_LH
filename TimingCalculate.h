#include <map>

namespace LPDDRSim {
    void TimingInit();
    void TimingCalc();
    void TimingPrint();
    unsigned Max(float ns, unsigned nck, float tck, unsigned ratio, float ofreq);
    static std::map <unsigned, std::vector<unsigned>> RL_SPEC;
    static std::map <unsigned, std::vector<unsigned>> WL_SPEC;
    static std::map <unsigned, std::vector<unsigned>> NWR_SPEC;
    static std::map <unsigned, std::vector<unsigned>> NRTP_SPEC;
    static std::map <unsigned, std::vector<unsigned>> BLN_SPEC;
}