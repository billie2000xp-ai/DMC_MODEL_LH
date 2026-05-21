#include "SystemConfiguration.h"
#include "AddressMapping.h"
#include <assert.h>
#include <bitset>

namespace LPDDRSim {

void addressMapping(Transaction &trans) {
    if (trans.mpam_id >= 16) {
        ERROR("Wrong mpam_id, task="<<trans.task<<", mpam_id="<<trans.mpam_id);
        assert(0);
    }
    if (trans.qos >= 8) {
        ERROR("Wrong qos, task="<<trans.task<<", qos="<<trans.qos);
        assert(0);
    }

    if (QOS_INV) trans.qos = (~trans.qos) & 0x7;
    else trans.qos = trans.qos & 0x7;
    if (GREEN_PATH_DIS) {
        if (trans.qos >= 6) trans.qos = 7;
        else trans.qos ++;
    }

    trans.pri = trans.qos;

    uint8_t sid0,sid1,sid2,bg0,bg1,bg2,bg3,bg4,ba0,ba1,ba2,ba3,ba4,ba5,ba6,ra0,ra1,ra2;
    uint8_t row23,row22,row21,row20,row19,row18,row17,row16,row15,row14,row13,row12;
    uint8_t row11,row10,row9,row8,row7,row6,row5,row4,row3,row2,row1,row0;
    uint8_t col10,col9,col8,col7,col6,col5,col4,col3,col2,col1,col0;
    uint8_t ch;

    ra0 = bit_xor(MATRIX_RA0, trans.address);
    ra1 = bit_xor(MATRIX_RA1, trans.address);
    ra2 = bit_xor(MATRIX_RA2, trans.address);
    trans.rank = ra0 | (ra1<<1) | (ra2<<2);
    
    ch = bit_xor(MATRIX_CH, trans.address);
    trans.sc = ch;

    row0 = bit_xor(MATRIX_ROW0, trans.address);
    row1 = bit_xor(MATRIX_ROW1, trans.address);
    row2 = bit_xor(MATRIX_ROW2, trans.address);
    row3 = bit_xor(MATRIX_ROW3, trans.address);
    row4 = bit_xor(MATRIX_ROW4, trans.address);
    row5 = bit_xor(MATRIX_ROW5, trans.address);
    row6 = bit_xor(MATRIX_ROW6, trans.address);
    row7 = bit_xor(MATRIX_ROW7, trans.address);
    row8 = bit_xor(MATRIX_ROW8, trans.address);
    row9 = bit_xor(MATRIX_ROW9, trans.address);
    row10 = bit_xor(MATRIX_ROW10, trans.address);
    row11 = bit_xor(MATRIX_ROW11, trans.address);
    row12 = bit_xor(MATRIX_ROW12, trans.address);
    row13 = bit_xor(MATRIX_ROW13, trans.address);
    row14 = bit_xor(MATRIX_ROW14, trans.address);
    row15 = bit_xor(MATRIX_ROW15, trans.address);
    row16 = bit_xor(MATRIX_ROW16, trans.address);
    row17 = bit_xor(MATRIX_ROW17, trans.address);
    row18 = bit_xor(MATRIX_ROW18, trans.address);
    row19 = bit_xor(MATRIX_ROW19, trans.address);
    row20 = bit_xor(MATRIX_ROW20, trans.address);
    row21 = bit_xor(MATRIX_ROW21, trans.address);
    row22 = bit_xor(MATRIX_ROW22, trans.address);
    row23 = bit_xor(MATRIX_ROW23, trans.address);
    trans.row = row0 | (row1<<1) | (row2<<2) | (row3<<3) | (row4<<4) | (row5<<5) | (row6<<6) | (row7<<7) |
            (row8<<8) | (row9<<9) | (row10<<10) | (row11<<11) | (row12<<12) | (row13<<13) | (row14<<14) |
            (row15<<15) | (row16<<16) | (row17<<17) | (row18<<18) | (row19<<19) | (row20<<20) |
            (row21<<21) | (row22<<22) | (row23<<23);

    bitset<32> row_addr(trans.row);
    // swap row_swl bit of row address with sc bit under combo e-mode
    if (EM_ENABLE && (EM_MODE==2)) {  
        if (trans.rank == 1) {
            if (row_addr[ROW_SEL] == 1) {
                ERROR("forbidden address, task="<<trans.task<<", address="<<hex<<trans.address<<dec<<", row="<<trans.row);
                assert(0);
            }
            trans.sc = (trans.row >> ROW_SEL) / 2;
            row_addr[ROW_SEL] = ch;
            trans.row = row_addr.to_ulong();
        }    
    }

    sid2 = bit_xor(MATRIX_SID2, trans.address);
    sid1 = bit_xor(MATRIX_SID1, trans.address);
    sid0 = bit_xor(MATRIX_SID0, trans.address);
    trans.sid = sid0 | (sid1<<1) | (sid2<<2);

    bg0 = bit_xor(MATRIX_BG0, trans.address);
    bg1 = bit_xor(MATRIX_BG1, trans.address);
    bg2 = (EM_ENABLE && DMC_RATE>6400) ? trans.sc : bit_xor(MATRIX_BG2, trans.address);   // bg2 under e-mode
    bg3 = bit_xor(MATRIX_BG3, trans.address);
    bg4 = bit_xor(MATRIX_BG4, trans.address);
    trans.group = bg0 | (bg1<<1) | (bg2<<2) | (bg3<<3) | (bg4<<4);

    ba0 = bit_xor(MATRIX_BA0, trans.address);
    ba1 = bit_xor(MATRIX_BA1, trans.address);
    ba2 = bit_xor(MATRIX_BA2, trans.address);
    ba3 = bit_xor(MATRIX_BA3, trans.address);
    ba4 = (EM_ENABLE && DMC_RATE<=6400) ? trans.sc : bit_xor(MATRIX_BA4, trans.address);   // ba4 under e-mode
    ba5 = bit_xor(MATRIX_BA5, trans.address);
    ba6 = bit_xor(MATRIX_BA6, trans.address);
    trans.bank = ba0 | (ba1<<1) | (ba2<<2) | (ba3<<3) | (ba4<<4) | (ba5<<5) | (ba6<<6);

    col0 = bit_xor(MATRIX_COL0, trans.address);
    col1 = bit_xor(MATRIX_COL1, trans.address);
    col2 = bit_xor(MATRIX_COL2, trans.address);
    col3 = bit_xor(MATRIX_COL3, trans.address);
    col4 = bit_xor(MATRIX_COL4, trans.address);
    col5 = bit_xor(MATRIX_COL5, trans.address);
    col6 = bit_xor(MATRIX_COL6, trans.address);
    col7 = bit_xor(MATRIX_COL7, trans.address);
    col8 = bit_xor(MATRIX_COL8, trans.address);
    col9 = bit_xor(MATRIX_COL9, trans.address);
    col10 = bit_xor(MATRIX_COL10, trans.address);
    trans.col = col0 | (col1<<1) | (col2<<2) | (col3<<3) | (col4<<4) | (col5<<5) |
            (col6<<6) | (col7<<7) | (col8<<8) | (col9<<9) | (col10<<10);
    if (IS_LP6) {
        trans.addr_col = trans.col * 16 / 8;
    } else {
        trans.addr_col = trans.col * JEDEC_DATA_BUS_BITS / 8;
    }

    trans.bankIndex = trans.bank + trans.group * (NUM_BANKS / NUM_SIDS / NUM_GROUPS) +
            trans.rank * NUM_BANKS + trans.sid * (NUM_BANKS / NUM_SIDS);
}

uint8_t bit_xor(uint64_t matrix, uint64_t address) {
    uint64_t address_and = matrix & address;
    return (bitset<64>(address_and).count() & 1);
}

void CalcMatrixNum() {
    uint8_t num = 0;
    if (MATRIX_RA2 != 0x0) num ++;
    if (MATRIX_RA1 != 0x0) num ++;
    if (MATRIX_RA0 != 0x0) num ++;
    NUM_RANKS = pow(2, num);

    num = 0;
    if (MATRIX_COL10 != 0x0) num ++;
    if (MATRIX_COL9 != 0x0) num ++;
    if (MATRIX_COL8 != 0x0) num ++;
    if (MATRIX_COL7 != 0x0) num ++;
    if (MATRIX_COL6 != 0x0) num ++;
    if (MATRIX_COL5 != 0x0) num ++;
    if (MATRIX_COL4 != 0x0) num ++;
    if (MATRIX_COL3 != 0x0) num ++;
    if (MATRIX_COL2 != 0x0) num ++;
    if (MATRIX_COL1 != 0x0) num ++;
    if (MATRIX_COL0 != 0x0) num ++;
    NUM_COLS = pow(2, num);

    num = 0;
    if (MATRIX_SID2 != 0x0 ) num ++;
    if (MATRIX_SID1 != 0x0 ) num ++;
    if (MATRIX_SID0 != 0x0 ) num ++;
    NUM_SIDS = pow(2, num);
    if (NUM_SIDS == 4) NUM_SIDS = 3;

    num = 0;
    if (MATRIX_BG4 != 0x0 ) num ++;
    if (MATRIX_BG3 != 0x0 ) num ++;
    if (MATRIX_BG2 != 0x0 ) num ++;
    if (EM_ENABLE && DMC_RATE>6400) num ++;   // BG2 under efficiency mode
    if (MATRIX_BG1 != 0x0 ) num ++;
    if (MATRIX_BG0 != 0x0 ) num ++;
    NUM_GROUPS = pow(2, num);

    num = 0;
    if (MATRIX_BA6 != 0x0 ) num ++;
    if (MATRIX_BA5 != 0x0 ) num ++;
    if (MATRIX_BA4 != 0x0 ) num ++;
    if (EM_ENABLE && DMC_RATE<=6400) num ++;   // BA4 under efficiency mode
    if (MATRIX_BA3 != 0x0 ) num ++;
    if (MATRIX_BA2 != 0x0 ) num ++;
    if (MATRIX_BA1 != 0x0 ) num ++;
    if (MATRIX_BA0 != 0x0 ) num ++;
    NUM_BANKS = pow(2, num);
    NUM_BANKS = NUM_BANKS * NUM_GROUPS * NUM_SIDS;

    num = 0;
    if (MATRIX_ROW23 != 0x0) num ++;
    if (MATRIX_ROW22 != 0x0) num ++;
    if (MATRIX_ROW21 != 0x0) num ++;
    if (MATRIX_ROW20 != 0x0) num ++;
    if (MATRIX_ROW19 != 0x0) num ++;
    if (MATRIX_ROW18 != 0x0) num ++;
    if (MATRIX_ROW17 != 0x0) num ++;
    if (MATRIX_ROW16 != 0x0) num ++;
    if (MATRIX_ROW15 != 0x0) num ++;
    if (MATRIX_ROW14 != 0x0) num ++;
    if (MATRIX_ROW13 != 0x0) num ++;
    if (MATRIX_ROW12 != 0x0) num ++;
    if (MATRIX_ROW11 != 0x0) num ++;
    if (MATRIX_ROW10 != 0x0) num ++;
    if (MATRIX_ROW9 != 0x0) num ++;
    if (MATRIX_ROW8 != 0x0) num ++;
    if (MATRIX_ROW7 != 0x0) num ++;
    if (MATRIX_ROW6 != 0x0) num ++;
    if (MATRIX_ROW5 != 0x0) num ++;
    if (MATRIX_ROW4 != 0x0) num ++;
    if (MATRIX_ROW3 != 0x0) num ++;
    if (MATRIX_ROW2 != 0x0) num ++;
    if (MATRIX_ROW1 != 0x0) num ++;
    if (MATRIX_ROW0 != 0x0) num ++;
    NUM_ROWS = pow(2, num);
    
    if (IS_LP6) {
        DRAM_CAPACITY = (uint64_t)NUM_COLS * NUM_BANKS * NUM_ROWS * 16 / 1024 / 1024 / 1024;
    } else {
        DRAM_CAPACITY = (uint64_t)NUM_COLS * NUM_BANKS * NUM_ROWS * JEDEC_DATA_BUS_BITS / 1024 / 1024 / 1024;
    }
}
};