#ifndef __MACROS_H_
#define __MACROS_H_

#include <iostream>

#define ERROR(str) (std::cerr<<"[ERROR ("<<__FILE__<<":"<<__LINE__<<")]: "<<str<<std::endl);
#define WARNING(str) (std::cerr<<"[Warning ("<<__FILE__<<":"<<__LINE__<<")]: "<<str<<std::endl);

using std::ostream;

#define DEBUG(str)  (std::cout<<std::fixed<< str <<std::endl);
#define DEBUG_TIME(str)  if (now() >= DEBUG_START_TIME && now() <= DEBUG_END_TIME) \
        std::cout<<std::fixed<< str <<std::endl;
#define DEBUGN(str) std::cout<<std::fixed<< str;

#define PRINT(str) {DDRSim_log <<std::fixed<<str<<std::endl; DDRSim_log.flush();}
#define PRINTN(str) if (channel_ohot == (channel_ohot & PRINT_CH_OHOT) && now() >= DEBUG_START_TIME \
        && now() <= DEBUG_END_TIME) DDRSim_log <<std::fixed<<str; DDRSim_log.flush();
#define STATE_PRINTN(str) {state_log <<std::fixed<< str; state_log.flush();}
#define TRACE_PRINT(str) if (channel_ohot == (channel_ohot & PRINT_CH_OHOT)) \
        {trace_log<<std::fixed<<str; trace_log.flush();}
#define CMDNUM_PRINT(str) if (channel_ohot == (channel_ohot & PRINT_CH_OHOT)) \
        {cmdnum_log<<std::fixed<<str; cmdnum_log.flush();}
#define DRAM_PRINT(str) if (channel_ohot == (channel_ohot & PRINT_CH_OHOT)) \
        {dram_log<<std::fixed<<str<<endl; dram_log.flush();}
#define PRINTR(str)  {DDRSim_log <<std::fixed<< str;}

#define ALIGNED_SIZE (0x7)
#define ALIGNED_DATA_64B (0xFFFFFF80)
#define ALIGNED_NUMB_64B (0x40)
#define INVALID_POSITION (0xffffffff)

// ---------------------------------------------------------------------
// general useful macro function definitions
// ---------------------------------------------------------------------

// BitValue : recieve a specific bit value from a data
#ifndef BitValue
#define BitValue(data,bit)  (((data)>>(bit))&((uint32_t)(0x1)))
#endif

// BitValueInBool : receive a specific bit value from a data, and then
// convert this value into bool format
#ifndef BitValueInBool
#define BitValueInBool(data,bit)    (bool)BitValue(data,bit)
#endif

// SetBitValue : set a specific bit value to '1' in a data
#ifndef SetBitValue
#define SetBitValue(data,bit)       ((data)=(data)|(((uint32_t)(0x1))<<(bit)))
#endif

// ClrBitValue : set a specific bit value to '0' in a data
#ifndef ClrBitValue
#define ClrBitValue(data,bit)       ((data)=(data)&(~(((uint32_t)(0x1))<<(bit))))
#endif

#define EMB_GET_PARAM(para1, para2, type) para1 = cfg->type(para2);
#define GET_PARAM(para1, para2, type) if (cfg.has(para2)) {para1 = cfg.type(para2);}
#define dmc_random(value, x, y) srand((int)time(0)); value = (rand() % (y - x + 1)) + x;
#define DmcLog2(x, y) pow(2, floor(log2(x * y)))
#define MidMax 256
#define BIT_GET(value, x, y) ((unsigned(value) >> unsigned(x)) & unsigned(pow(2, unsigned(y)) - 1))

//#define SYSARCH_PLATFORM

#endif /*PRINT_MACROS_H*/