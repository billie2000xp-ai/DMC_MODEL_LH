#include "MemoryController.h"
#include "MemorySystem.h"

using namespace LPDDRSim;
//==============================================================================
MemoryController::MemoryController(MemorySystem *parent, ostream &DDRSim_log_,
        ostream &trace_log_, ostream &cmdnum_log_): DDRSim_log(DDRSim_log_),
        trace_log(trace_log_), cmdnum_log(cmdnum_log_) {
    //get handle on parent
    parentMemorySystem = parent;
    log_path = parentMemorySystem->log_path;
    channel = parentMemorySystem->systemID * NUM_CHANS + parentMemorySystem->dmc_id;
    sub_cha = parentMemorySystem->dmc_id; 
    channel_ohot = 1ull << channel;
    PostCalcTiming();
    //bus related fields
    cmdCyclesLeft = 0;
    //set here to avoid compile errors
    //reserve memory for vectors
    transactionQueue.reserve(TRANS_QUEUE_DEPTH);
    slt_valid.clear();
    //slt_valid.resize(TRANS_QUEUE_DEPTH);
    //DEBUG("size="<<slt_valid.size());
    for (size_t i = 0; i < TRANS_QUEUE_DEPTH; i ++) {
        slt_valid.push_back(false);
        //slt_valid[i]=false;
    //DEBUG("i="<<i<<" size="<<slt_valid.size());
    }
    CmdQueue.reserve(TRANS_QUEUE_DEPTH);
    pending_TransactionQue.clear();
    totalTransactions = 0;
    RtCmdCnt = 0;
    totalReads = 0;
    totalWrites = 0;
    addrconf_cnt = 0;
    idconf_cnt = 0;
    baconf_cnt = 0;
    totalconf_cnt = 0;
    active_cnt = 0;
    active_dst_cnt = 0;
    precharge_sb_cnt = 0;
    precharge_pb_cnt = 0;
    precharge_ab_cnt = 0;
    precharge_pb_dst_cnt = 0;
    read_p_cnt = 0;
    write_p_cnt = 0;
    read_cnt = 0;
    write_cnt = 0;
    mwrite_cnt = 0;
    mwrite_p_cnt = 0;
    refresh_ab_cnt = 0;
    refresh_pb_cnt = 0;
    dmc_timeout_cnt = 0;
    com_read_cnt = 0;
    rw_switch_cnt = 0;
    r2w_switch_cnt = 0;
    w2r_switch_cnt = 0;
    rank_switch_cnt = 0;
    sid_switch_cnt = 0;
    pbr_overall_cnt = 0;
    page_exceed_cnt = 0;
    dresp_cnt = 0;
//    rank_refresh_cnt.clear();
//    refresh_cnt_pb.clear();
//    refresh_cnt_pb.reserve(NUM_RANKS);
    fast_wakeup.reserve(NUM_RANKS);
    fast_wakeup_cnt.reserve(NUM_RANKS);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
//        refresh_pbr_has_finish.push_back(false);
//        force_pbr_refresh.push_back(false);
//        forceRankBankIndex.push_back(0);
//        refresh_cnt_pb.push_back(0);
        RankStatus lp = RankStatus(i);
        lp.pd_cnt = tXP;
        RankState.push_back(lp);
        PdTime.push_back(0);
        AsrefTime.push_back(0);
        SrpdTime.push_back(0);
        WakeUpTime.push_back(0);
        PdEnterCnt.push_back(0);
        PdExitCnt.push_back(0);
        AsrefEnterCnt.push_back(0);
        AsrefExitCnt.push_back(0);
        SrpdEnterCnt.push_back(0);
        SrpdExitCnt.push_back(0);
        fast_wakeup.push_back(false);
        fast_wakeup_cnt.push_back(0);
    }
    WdataPipe.clear();
    phy_lp_cnt = 0;
    rd_inc_cnt = 0;
    rd_wrap_cnt = 0;
    wr_inc_cnt = 0;
    wr_wrap_cnt = 0;
    rdata_cnt = 0;
    wdata_cnt = 0;
    phy_notlp_cnt = 0;
    if (PD_ENABLE) {
        for (size_t i = 0; i < NUM_RANKS; i ++) RankState[i].lp_state = IDLE;
    }
    if (ASREF_ENABLE) {
        for (size_t i = 0; i < NUM_RANKS; i ++) RankState[i].asref_cnt = ASREF_PRD;
    }
    total_latency = 0;
    dly_ex2000_cnt = 0;
    writeDataToSend.reserve(TRANS_QUEUE_DEPTH);
    
    grt_fifo_wcmd_cnt = 0;
    grt_fifo_bp = false;

    //initialization for priority of 4 abr gourps: 0->abr_group0; 1->abr_group1; 2->abr_group2; 3->abr_group3;
    for (size_t i = 0; i < 4; i ++) {
        arb_group_pri.push_back(i);
    }

    for (size_t i = 0; i < 4; i ++) {
        arb_group_cnt.push_back(0);
    }

    pre_cmd_time = 0xFFFFFFFFFFFFFFFF;

    packet.clear();
    pre_req_time = 0xFFFFFFFFFFFFFFFF;
    pre_req_data_time = 0xFFFFFFFFFFFFFFFF;
    pre_rresp_time = 0xFFFFFFFFFFFFFFFF;
    pre_cresp_time = 0xFFFFFFFFFFFFFFFF;
    rd_met_abr_cnt = 0;
    rd_met_pbr_cnt = 0;
    pbr_block_allcmd_cycle = 0;
    pbr_cycle = 0;
    pre_dat_time = 0xFFFFFFFFFFFFFFFF;
    acc_rank_cnt.clear();
    acc_rank_cnt.reserve(NUM_RANKS);
    acc_bank_cnt.clear();
    acc_bank_cnt.reserve(NUM_BANKS * NUM_RANKS);
    racc_rank_cnt.clear();
    racc_rank_cnt.reserve(NUM_RANKS);
    racc_bank_cnt.clear();
    racc_bank_cnt.reserve(NUM_BANKS * NUM_RANKS);
    wacc_rank_cnt.clear();
    wacc_rank_cnt.reserve(NUM_RANKS);
    wacc_bank_cnt.clear();
    wacc_bank_cnt.reserve(NUM_BANKS * NUM_RANKS);
    r_bank_cnt.clear();
    r_bank_cnt.reserve(NUM_BANKS * NUM_RANKS);
    w_bank_cnt.clear();
    w_bank_cnt.reserve(NUM_BANKS * NUM_RANKS);
    rank_cnt.clear();
    rank_cnt.reserve(NUM_RANKS);
    rank_pre_act_cnt.clear();
    rank_pre_act_cnt.reserve(NUM_RANKS);
    bank_cnt.clear();
    bank_cnt.reserve(NUM_BANKS * NUM_RANKS);
    deqCmdWakeupLp.clear();
    deqCmdWakeupLp.resize(NUM_RANKS);
    r_rank_cnt.clear();
    r_rank_cnt.reserve(NUM_RANKS);
    w_rank_cnt.clear();
    w_rank_cnt.reserve(NUM_RANKS);
    r_qos_cnt.clear();
    r_qos_cnt.reserve(8);
    w_qos_cnt.clear();
    w_qos_cnt.reserve(8);
    active_cmd_cnt.clear();
    active_cmd_cnt.reserve(NUM_BANKS * NUM_RANKS);
    BankRowActCnt.clear();
    BankRowActCnt.reserve(NUM_BANKS * NUM_RANKS);
    min_delay = 0;
    max_delay = 0;
    min_delay_id = 0;
    max_delay_id = 0;
    r_rank_bst.clear();
    r_rank_bst.reserve(NUM_RANKS);
    r_rank_mux.clear();
    r_rank_mux.reserve(NUM_RANKS);
    bank_cnt_ehs.clear();
    bank_cnt_ehs.reserve(NUM_BANKS * NUM_RANKS);
    for (size_t i = 0; i < 8; i ++) {
        qos_delay_cnt.push_back(0);
        qos_cnt.push_back(0);
        qos_timeout_cnt.push_back(0);
        r_qos_cnt.push_back(0);
        w_qos_cnt.push_back(0);
    }
    for (size_t i = 0; i < MidMax; i ++) {
        mid_delay_cnt.push_back(0);
        mid_cnt.push_back(0);
    }
    for (size_t i = 0; i < 4; i ++) {
        pf_delay_cnt.push_back(0);
        pf_cnt.push_back(0);
    }
    qos_level_cnt.resize(12);
    for (auto &level_cnt : qos_level_cnt) {
        level_cnt.resize(13);
    }
    lat_dly_cnt.resize(40);
    lat_dly_step.resize(40);
    lat_dly_step[0] = 10; lat_dly_step[1] = 20; lat_dly_step[2] = 30;
    lat_dly_step[3] = 40; lat_dly_step[4] = 50; lat_dly_step[5] = 60;
    lat_dly_step[6] = 70; lat_dly_step[7] = 80; lat_dly_step[8] = 90;
    lat_dly_step[9] = 100; lat_dly_step[10] = 110; lat_dly_step[11] = 120;
    lat_dly_step[12] = 130; lat_dly_step[13] = 140; lat_dly_step[14] = 150;
    lat_dly_step[15] = 160; lat_dly_step[16] = 170; lat_dly_step[17] = 180;
    lat_dly_step[18] = 190; lat_dly_step[19] = 200; lat_dly_step[20] = 210;
    lat_dly_step[21] = 220; lat_dly_step[22] = 230; lat_dly_step[23] = 240;
    lat_dly_step[24] = 250; lat_dly_step[25] = 260; lat_dly_step[26] = 270;
    lat_dly_step[27] = 280; lat_dly_step[28] = 290; lat_dly_step[29] = 300;
    lat_dly_step[30] = 400; lat_dly_step[31] = 500; lat_dly_step[32] = 600;
    lat_dly_step[33] = 700; lat_dly_step[34] = 800; lat_dly_step[35] = 900;
    lat_dly_step[36] = 1000; lat_dly_step[37] = 1500; lat_dly_step[38] = 2000;
    lat_dly_step[39] = 10000;

    ddrc_av_lat = 0;

    for (size_t i = 0; i < NUM_RANKS; i ++) {
        rank_cnt.push_back(0);
        rank_pre_act_cnt.push_back(0);
        r_rank_cnt.push_back(0);
        w_rank_cnt.push_back(0);
//        refreshALL.push_back(i);
//        rank_refresh_cnt.push_back(0);
        ddrc_av_lat_rank.push_back(0);
        total_latency_rank.push_back(0);
        com_read_cnt_rank.push_back(0);
        r_rank_bst.push_back(0);
        w_rank_bst.push_back(0);
        r_rank_mux.push_back(NULL);
        w_rank_mux.push_back(NULL);
        for (size_t j = 0; j < tCMD_WAKEUP; j ++) {
            deqCmdWakeupLp[i].push_back(0);
        }
    }
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        if (GRP_RANK_MODE == 0) {
            r_rank_mux[i] = &r_rank_cnt[i];
            w_rank_mux[i] = &w_rank_cnt[i];
        } else if (GRP_RANK_MODE == 1) {
            r_rank_mux[i] = &r_rank_bst[i];
            w_rank_mux[i] = &w_rank_bst[i];
        }
    }

    issue_state.clear();
    issue_state.reserve(NUM_BANKS * NUM_RANKS);
    for (size_t i = 0; i< NUM_RANKS; i ++) {
        for (size_t j = 0; j < NUM_BANKS; j ++){
            issue_state.push_back(false);
        }
    }


    if (IS_LP5 || IS_LP6) {
        pbr_bg_num = 2;
        sc_num = EM_ENABLE ? 2:1;
        pbr_bank_num = NUM_BANKS / pbr_bg_num / sc_num;
        sc_bank_num = NUM_BANKS/ sc_num;
        activate_cmd = ACTIVATE1_CMD;
        pbr_sb_group_num = 4;
        pbr_sb_num = NUM_BANKS / pbr_sb_group_num;
    } else if (IS_LP4) {
        pbr_bg_num = 1;
        sc_num = EM_ENABLE ? 2:1;
        pbr_bank_num = NUM_BANKS / pbr_bg_num / sc_num;
        sc_bank_num = NUM_BANKS/ sc_num;
        activate_cmd = ACTIVATE1_CMD;
        pbr_sb_group_num = 4;
        pbr_sb_num = NUM_BANKS / pbr_sb_group_num;
    } else if (IS_GD2 || IS_HBM2E || IS_HBM3) {
        pbr_bg_num = 1;
        sc_num = EM_ENABLE ? 2:1;
        pbr_bank_num = NUM_BANKS / pbr_bg_num / sc_num;
        sc_bank_num = NUM_BANKS/ sc_num;
        activate_cmd = ACTIVATE1_CMD;
        pbr_sb_group_num = 4;
        pbr_sb_num = NUM_BANKS / pbr_sb_group_num;
    } else if (IS_GD1 || IS_G3D) {
        pbr_bg_num = 1;
        sc_num = EM_ENABLE ? 2:1;
        pbr_bank_num = NUM_BANKS / pbr_bg_num / sc_num;
        sc_bank_num = NUM_BANKS/ sc_num;
        activate_cmd = ACTIVATE2_CMD;
        pbr_sb_group_num = 4;
        pbr_sb_num = NUM_BANKS / pbr_sb_group_num;
    } else if (IS_DDR5 || IS_DDR4) {
        pbr_bg_num = NUM_GROUPS; //pbr bg number of bank
        sc_num = EM_ENABLE ? 2:1;
        pbr_bank_num = NUM_BANKS / pbr_bg_num / sc_num;
        sc_bank_num = NUM_BANKS/ sc_num;
        activate_cmd = ACTIVATE2_CMD;
        pbr_sb_group_num = 4;
        pbr_sb_num = NUM_BANKS / pbr_sb_group_num;
    }

    //added for SBR_WEIGHT_ENH_MODE (new stratege)   
    pre_enh_pbr_bagroup.clear();
    pre_enh_pbr_bagroup.reserve(NUM_RANKS);
    for (size_t rank = 0; rank < NUM_RANKS; rank ++){
        pre_enh_pbr_bagroup.push_back(0xFFFFFFFF);
    }

    pre_sch_bankIndex.clear();
    pre_sch_bankIndex.reserve(NUM_RANKS);
    for (size_t rank = 0; rank < NUM_RANKS; rank ++){
        pre_sch_bankIndex.push_back(0xFFFFFFFF);
    }

    refreshALL.clear();
    refreshALL.reserve(NUM_RANKS);
    rank_refresh_cnt.clear();
    rank_refresh_cnt.reserve(NUM_RANKS);
    refresh_pbr_has_finish.clear();
    refresh_pbr_has_finish.reserve(NUM_RANKS);
    force_pbr_refresh.clear();
    force_pbr_refresh.reserve(NUM_RANKS);
    forceRankBankIndex.clear();
    forceRankBankIndex.reserve(NUM_RANKS);
    refresh_cnt_pb.clear();
    refresh_cnt_pb.reserve(NUM_RANKS);
    sc_cnt.clear();
    sc_cnt.reserve(NUM_RANKS);
    rank_cnt_sbridle.clear();
    rank_cnt_sbridle.reserve(NUM_RANKS);
    rank_send_pbr.clear();
    rank_send_pbr.reserve(NUM_RANKS);
    bank_pair_cmd_cnt.clear();
    bank_pair_cmd_cnt.reserve(NUM_RANKS);

    for (size_t i = 0; i < NUM_RANKS; i ++) {
        bank_pair_cmd_cnt.push_back(vector<unsigned>());
        if (ENH_PBR_EN) {
            for (size_t bank = 0; bank < pbr_sb_group_num; bank ++){
                for (size_t sb_bank = 0; sb_bank < pbr_sb_num; sb_bank ++){
                    for (size_t sb_bank_tmp = (sb_bank+1); sb_bank_tmp < pbr_sb_num; sb_bank_tmp ++){
                        bank_pair_cmd_cnt[i].push_back(0);
//                        sb_cnt ++;
//                        DEBUG(now()<<" no.bank pair="<<sb_cnt<<" fst_bank_idx="<<sb_bank_idx<<" lst_bank_idx="<<sb_bank_tmp_idx);
                    }
                }
            }
//            DEBUG(now()<<" ini, SbWeight_size="<<SbWeight[rank].size()<<" rank="<<rank);
        } else {
            for (size_t j = 0; j < sc_num * pbr_bank_num ; j ++) {
                bank_pair_cmd_cnt[i].push_back(0);
            }
        }
    }

    for (size_t rank = 0; rank < NUM_RANKS; rank++) {
        refreshALL.push_back(vector<FORALLREFRESH>());
        for (size_t sub_ch = 0; sub_ch < sc_num; sub_ch ++) {
            FORALLREFRESH far = FORALLREFRESH(rank);
            refreshALL[rank].push_back(far);
        }
    }

    for (size_t i = 0; i < NUM_RANKS; i ++) {
//        refreshALL.push_back(vector<unsigned>());
        rank_refresh_cnt.push_back(vector<unsigned>());
        refresh_pbr_has_finish.push_back(vector<bool>());
        force_pbr_refresh.push_back(vector<bool>());
        forceRankBankIndex.push_back(vector<unsigned>());
        refresh_cnt_pb.push_back(vector<unsigned>());
        sc_cnt.push_back(vector<uint32_t>());
        rank_cnt_sbridle.push_back(vector<uint32_t>());
        rank_send_pbr.push_back(vector<bool>());
        for (size_t j = 0; j < sc_num ; j ++) {
//            refreshALL[i].push_back(i);        // todo, revise initialization
            rank_refresh_cnt[i].push_back(0);
            refresh_pbr_has_finish[i].push_back(false);
            force_pbr_refresh[i].push_back(false);
            forceRankBankIndex[i].push_back(0);
            refresh_cnt_pb[i].push_back(0);
            sc_cnt[i].push_back(0);
            rank_cnt_sbridle[i].push_back(0);
            rank_send_pbr[i].push_back(SBR_IDLE_ADAPT_EN);
        }
    }

    if (IS_GD2) NUM_MATGRPS = 4;
    else NUM_MATGRPS = 1;

    refreshPerBank.clear();
    refreshPerBank.reserve(NUM_BANKS * NUM_RANKS);
    for (size_t i = 0; i < NUM_BANKS * NUM_RANKS; i ++) {
        refreshPerBank.push_back(i);
        bank_cnt.push_back(0);
    }

    perbank_refresh_cnt.clear();
    if (ENH_PBR_EN) {
        for (size_t i = 0; i < NUM_RANKS * NUM_BANKS; i ++) {
            perbank_refresh_cnt.push_back(0);
        }
    } else {
        for (size_t i = 0; i < NUM_RANKS * pbr_bank_num * sc_num; i ++) {   // todo: revise for e-mode
            perbank_refresh_cnt.push_back(0);
        }
    }

    DistRefState.reserve(NUM_BANKS * NUM_RANKS);
    for (size_t i = 0; i < NUM_BANKS * NUM_RANKS; i ++) {
        GD2_DIST_STATE dst = GD2_DIST_STATE(i);
        DistRefState.push_back(dst);
    }

    PbrWeight.resize(NUM_RANKS);
    SbGroupWeight.resize(NUM_RANKS);
    SbWeight.resize(NUM_RANKS);
    
    if(ENH_PBR_EN){
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
                pbr_weight pbr = pbr_weight(bank, bank, bank);
                PbrWeight[rank].push_back(pbr);
            }
        }
    } else {
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            for (size_t bank = 0; bank < pbr_bank_num * sc_num * NUM_MATGRPS; bank ++) {
                pbr_weight pbr = pbr_weight(bank, bank, bank);
                PbrWeight[rank].push_back(pbr);
            }
        }
    }

    // sgw : samebank group weight
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        for (size_t bank = 0; bank < pbr_sb_group_num; bank ++) {
            pbr_weight sgw = pbr_weight(bank, bank, bank);
            SbGroupWeight[rank].push_back(sgw);
        }
    }
    
    // initialization for weight of 24 bank bank pairs: 
    // (0,4),(0,8),(0,12),(4,8),(4,12),(8,12),(1,5),(1,9),(1,13),(5,9),(5,13),(9,13)
    // (2,6),(2,10),(2,14),(6,10),(6,14),(10,14),(3,7),(3,11),(3,15),(7,11),(7,15),(11,15)
    // sw: samebank weight
//    unsigned sb_cnt = 0;
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        for (size_t bank = 0; bank < pbr_sb_group_num; bank ++){
            for (size_t sb_bank = 0; sb_bank < pbr_sb_num; sb_bank ++){
                unsigned sb_bank_idx = rank * NUM_BANKS + bank + sb_bank*pbr_sb_num; 
                for (size_t sb_bank_tmp = (sb_bank+1); sb_bank_tmp < pbr_sb_num; sb_bank_tmp ++){
                    unsigned sb_bank_tmp_idx = rank * NUM_BANKS + bank + sb_bank_tmp*pbr_sb_num; 
                    pbr_weight sw = pbr_weight(sb_bank_idx, sb_bank_idx, sb_bank_tmp_idx);
                    SbWeight[rank].push_back(sw);
//                    sb_cnt ++;
//                    DEBUG(now()<<" no.bank pair="<<sb_cnt<<" fst_bank_idx="<<sb_bank_idx<<" lst_bank_idx="<<sb_bank_tmp_idx);
                }
            }
        }
//        DEBUG(now()<<" ini, SbWeight_size="<<SbWeight[rank].size()<<" rank="<<rank);
    }


    for (size_t i = 0; i < NUM_RANKS; i ++) {
        r_bg_cnt.push_back(vector<unsigned>());
        w_bg_cnt.push_back(vector<unsigned>());
        bg_cnt.push_back(vector<unsigned>());
        r_sid_cnt.push_back(vector<unsigned>());
        w_sid_cnt.push_back(vector<unsigned>());
        sid_cnt.push_back(vector<unsigned>());
        for (size_t j = 0; j < NUM_GROUPS; j ++) {
            r_bg_cnt[i].push_back(0);
            w_bg_cnt[i].push_back(0);
            bg_cnt[i].push_back(0);
        }
        for (size_t j = 0; j < NUM_SIDS; j ++) {
            r_sid_cnt[i].push_back(0);
            w_sid_cnt[i].push_back(0);
            sid_cnt[i].push_back(0);
        }
    }
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        acc_rank_cnt.push_back(0);
        racc_rank_cnt.push_back(0);
        wacc_rank_cnt.push_back(0);
        for (size_t j = 0; j < NUM_BANKS; j ++) {
            r_bank_cnt.push_back(0);
            w_bank_cnt.push_back(0);
            acc_bank_cnt.push_back(0);
            racc_bank_cnt.push_back(0);
            wacc_bank_cnt.push_back(0);
            active_cmd_cnt.push_back(0);
            BankRowActCnt.push_back(0);
            bank_cnt_ehs.push_back(0);
        }
    }
    ehs_page_adapt_cnt.resize(NUM_RANKS);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        ehs_page_adapt_cnt[i].resize(MAP_CONFIG["ENH_PAGE_ADPT_TIME"].size());
    }
    access_bank_delay.clear();
    access_bank_delay.reserve(NUM_BANKS * NUM_RANKS);
    bank_cas_delay.clear();
    bank_cas_delay.reserve(NUM_BANKS * NUM_RANKS);
    page_timeout_rd.clear();
    page_timeout_rd.reserve(NUM_BANKS * NUM_RANKS);
    page_timeout_wr.clear();
    page_timeout_wr.reserve(NUM_BANKS * NUM_RANKS);
    page_cmd_cnt.clear();
    page_cmd_cnt.reserve(NUM_BANKS * NUM_RANKS);
    COUNTER cnt;
    cnt.enable = false;
    cnt.cnt = 0;
    que_read_cnt = 0;
    que_write_cnt = 0;
    dmc_cmd_cnt = 0;
    arb_enable = true;
    total_iecc_cnt = 0;
    total_noiecc_cnt = 0;
//    act_executing = false;
    even_cycle = false;
    odd_cycle = false;

    lqos_bp = false;
    lqos_rd_bp = false;
    lqos_wr_bp = false;

    rowconf_pre_cnt.clear();
    pageto_pre_cnt.clear();
    func_pre_cnt.clear();
    act_executing.clear();
    rowconf_pre_cnt.reserve(NUM_BANKS * NUM_RANKS);
    pageto_pre_cnt.reserve(NUM_BANKS * NUM_RANKS);
    func_pre_cnt.reserve(NUM_BANKS * NUM_RANKS);
    act_executing.reserve(NUM_BANKS * NUM_RANKS);
    for (size_t i = 0; i < NUM_RANKS * NUM_BANKS; i ++) {
        access_bank_delay.push_back(cnt);
        bank_cas_delay.push_back(0);
        page_timeout_rd.push_back(OPENPAGE_TIME_RD);
        page_timeout_wr.push_back(OPENPAGE_TIME_WR);
        page_cmd_cnt.push_back(0);
        rowconf_pre_cnt.push_back(0);
        pageto_pre_cnt.push_back(0);
        func_pre_cnt.push_back(0);
        act_executing.push_back(false);
    }
    adpt_openpage_time = OPENPAGE_TIME_RD;

    page_timeout_window[0] = 1; page_timeout_window[1] = 5; page_timeout_window[2] = 10;
    page_timeout_window[3] = 20; page_timeout_window[4] = 30; page_timeout_window[5] = 40;
    page_timeout_window[6] = 50; page_timeout_window[7] = 60; page_timeout_window[8] = 70;
    page_timeout_window[9] = 80; page_timeout_window[10] = 90; page_timeout_window[11] = 100;
    page_timeout_window[12] = 110; page_timeout_window[13] = 120; page_timeout_window[14] = 130;
    page_timeout_window[15] = 140; page_timeout_window[16] = 150; page_timeout_window[17] = 160;
    page_timeout_window[18] = 170; page_timeout_window[19] = 180; page_timeout_window[20] = 190;
    page_timeout_window[21] = 200; page_timeout_window[22] = 210; page_timeout_window[23] = 220;
    page_timeout_window[24] = 230; page_timeout_window[25] = 240; page_timeout_window[26] = 250;
    page_timeout_window[27] = 260; page_timeout_window[28] = 270; page_timeout_window[29] = 280;
    page_timeout_window[30] = 290; page_timeout_window[31] = 300; page_timeout_window[32] = 400;
    page_timeout_window[33] = 500;
    page_row_hit.resize(NUM_BANKS*NUM_RANKS);
    for (auto &ele : page_row_hit)
        ele.resize(sizeof(page_timeout_window)/sizeof(page_timeout_window[0]));

    page_row_miss.resize(NUM_BANKS * NUM_RANKS);
    for (auto &ele : page_row_miss)
        ele.resize(sizeof(page_timeout_window)/sizeof(page_timeout_window[0]));

    page_row_conflict.resize(NUM_BANKS * NUM_RANKS);
    for (auto &ele : page_row_conflict)
        ele.resize(sizeof(page_timeout_window)/sizeof(page_timeout_window[0]));

    cmd_in2dfi_lat = 0;
    cmd_in2dfi_cnt = 0;
    cmd_rdmet_cnt = 0;
    forward_64B_cnt = 0;
    forward_128B_cnt = 0;
    //staggers when each rank is due for a refresh
    TotalBytes = 0;
    TotalReadBytes = 0;
    TotalWriteBytes = 0;
    DmcTotalBytes = 0;
    DmcTotalReadBytes = 0;
    DmcTotalWriteBytes = 0;
    flowStatisTotalBytes = 0;

//    //tFAW check for each rank, each subchann 
//    tFAWCountdown.reserve(NUM_RANKS);
//    for (size_t i = 0; i < NUM_RANKS; i ++) {
//        tFAWCountdown.push_back(vector<vector<unsigned>()>);
//        for (size_t j = 0; j < sc_num; j ++) {
//            tFAWCountdown[i].push_back(vector<unsigned>());
//        }
//    }

    tFPWCountdown.reserve(NUM_RANKS);
    tFAWCountdown.reserve(NUM_RANKS);
    tFAWCountdown_sc1.reserve(NUM_RANKS);
    has_wakeup.reserve(NUM_RANKS);
    rank_has_cmd.reserve(NUM_RANKS);
    rank_has_pre_act_cmd.reserve(NUM_RANKS);
    bank_idle_cnt.reserve(NUM_RANKS);
    bank_act_cnt.reserve(NUM_RANKS);
    rank_cnt_asref.reserve(NUM_RANKS);
//    rank_cnt_sbridle.reserve(NUM_RANKS);
//    rank_send_pbr.reserve(NUM_RANKS);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        //init the empty vectors here so we don't seg fault later
        tFAWCountdown.push_back(vector<unsigned>());
        tFAWCountdown_sc1.push_back(vector<unsigned>());
        tFPWCountdown.push_back(vector<unsigned>());
        FuncState state;
        funcState.push_back(state);
        TotalBytesRank.push_back(0);
        has_wakeup.push_back(false);
        rank_has_cmd.push_back(false);
        rank_has_pre_act_cmd.push_back(false);
        bank_idle_cnt.push_back(0);
        bank_act_cnt.push_back(0);
        rank_cnt_asref.push_back(0);
//        rank_cnt_sbridle.push_back(0);
//        rank_send_pbr.push_back(SBR_IDLE_ADAPT_EN);
    }
    exec_valid = false;

    for (size_t r = 0; r < NUM_RANKS; r++) {
        for (size_t s = 0; s < NUM_SIDS; s++) {
            for (size_t g = 0; g < NUM_GROUPS; g++) {
                for (size_t b = 0; b < NUM_BANKS / NUM_SIDS / NUM_GROUPS; b++) {
                    BankTableState state = BankTableState(DDRSim_log,r,s,g,b);
                    bankStates.push_back(state);
                }
            }
        }
    }

    tasks_info.clear();
    wdata_info.clear();
    rmw_rd_finish.clear();

    occ = 0;
    availability = 0;
    sum_avai = 0;
    sum_pwr_avai = 0;
    row_hit_ratio = 0;
    occ_1_cnt = 0;
    occ_2_cnt = 0;
    occ_3_cnt = 0;
    occ_4_cnt = 0;
    bw_totalcmds = 0;
    bw_totalwrites = 0;
    bw_totalreads = 0;
    ecc_total_bytes = 0;
    ecc_total_reads = 0;
    ecc_total_writes = 0;
    rmw_total_bytes = 0;
    rmw_total_reads = 0;
    TotalDmcBytes = 0;
    TotalDmcRdBytes = 0;
    TotalDmcWrBytes = 0;
    TotalDmcRd32B = 0;
    TotalDmcRd64B = 0;
    TotalDmcRd128B = 0;
    TotalDmcRd256B = 0;
    TotalDmcWr32B = 0;
    TotalDmcWr64B = 0;
    TotalDmcWr128B = 0;
    TotalDmcWr256B = 0;
    wb = new WriteBuff(this,channel,DDRSim_log);
    rmw = new Rmw(this,channel,DDRSim_log);
    iecc = new Inline_ECC(this,channel,DDRSim_log);
    avai_sqrt = 0;
    rw_group_state.reserve(8);
    for (size_t i = 0; i < 8; i ++) rw_group_state.push_back(NO_GROUP);
    in_write_group = false; // true is write group, false is read group
    rk_grp_state = NO_RGRP;
    real_rk_grp_state = NO_RGRP;
    serial_cmd_cnt = 0x0;
    rwgrp_ch_cmd_cnt = 0x0;
    rankgrp_ch_cmd_cnt = 0x0;
    no_sch_cmd_en = false;
    no_sch_cmd_cnt = 0x0;

    if (IS_LP4) {
        cmd_cycle = ceil(2 / WCK2DFI_RATIO);
        pre_cycle = ceil(2 / WCK2DFI_RATIO);
        rw_cycle = ceil(4 / WCK2DFI_RATIO);
    } else if (IS_LP6) {
        cmd_cycle = ceil(2 * OFREQ_RATIO);
        pre_cycle = ceil(2 * OFREQ_RATIO);
        rw_cycle = ceil(2 * OFREQ_RATIO);
    } else if (IS_GD2) {
        cmd_cycle = ceil(OFREQ_RATIO);
        pre_cycle = ceil(OFREQ_RATIO) * 2;
        rw_cycle = ceil(OFREQ_RATIO);
    } else if (IS_LP5 || IS_GD1 || IS_G3D || IS_DDR5 || IS_DDR4) {
        cmd_cycle = ceil(OFREQ_RATIO);
        pre_cycle = ceil(OFREQ_RATIO);
        rw_cycle = ceil(OFREQ_RATIO);
    } else if (IS_HBM2E || IS_HBM3) {
        cmd_cycle = ceil(4 / WCK2DFI_RATIO);
        pre_cycle = ceil(1 / WCK2DFI_RATIO);
        rw_cycle = ceil(1 / WCK2DFI_RATIO);
    }

    BLEN = MAP_CONFIG["BL"][0];
    bl_data_size.clear();
    uint8_t bl_size = MAP_CONFIG["BL"].size();
    for (size_t i = 0; i < bl_size; i ++) {
        unsigned bl = MAP_CONFIG["BL"][i];
        if (IS_LP6){
            bl_data_size[bl] = bl * JEDEC_DATA_BUS_BITS * PAM_RATIO / 9;
        } else {
            bl_data_size[bl] = bl * JEDEC_DATA_BUS_BITS * PAM_RATIO / 8;
        }
    }
//    for (auto it = bl_data_size.begin(); it != bl_data_size.end(); it ++) {
//        DEBUG(now()<<" bl="<<it->first<<" bl_data_size="<<it->second);
//    }

    if (IS_LP6) {
        max_bl_data_size = BLEN * JEDEC_DATA_BUS_BITS * PAM_RATIO / 9;
        min_bl_data_size = MAP_CONFIG["BL"][bl_size - 1] * JEDEC_DATA_BUS_BITS * PAM_RATIO / 9;
    } else {
        max_bl_data_size = BLEN * JEDEC_DATA_BUS_BITS * PAM_RATIO / 8;
        min_bl_data_size = MAP_CONFIG["BL"][bl_size - 1] * JEDEC_DATA_BUS_BITS * PAM_RATIO / 8;
    }

    bp_cycle.clear();
    bp_step.clear();
    if (NUM_GROUPS > 1) {
        if (IS_LP5 || IS_GD2) {
            if (WCK2DFI_RATIO == 4) {
                for (size_t i = 3; i <= 5; i ++) bp_step.push_back(i);
            } else if (WCK2DFI_RATIO == 2) {
                for (size_t i = 5; i <= 11; i ++) bp_step.push_back(i);
            } else {
                ERROR(setw(10)<<now()<<" -- Error WCK2DFI_RATIO: "<<WCK2DFI_RATIO);
                assert(0);
            }
        } else if (IS_LP6) {
            if (WCK2DFI_RATIO == 4) {
                for (size_t i = 4; i <= 8; i ++) bp_step.push_back(i);
            } else if (WCK2DFI_RATIO == 2) {
                for (size_t i = 7; i <= 17; i ++) bp_step.push_back(i);
            } else {
                ERROR(setw(10)<<now()<<" -- Error WCK2DFI_RATIO: "<<WCK2DFI_RATIO);
                assert(0);
            }
        }
    }

    RdCntBl.clear();   WrCntBl.clear();
    RdCntBl[BL8] = 0;  WrCntBl[BL8] = 0;
    RdCntBl[BL16] = 0; WrCntBl[BL16] = 0;
    RdCntBl[BL24] = 0; WrCntBl[BL24] = 0;
    RdCntBl[BL32] = 0; WrCntBl[BL32] = 0;
    RdCntBl[BL48] = 0; WrCntBl[BL48] = 0;
    RdCntBl[BL64] = 0; WrCntBl[BL64] = 0;

    dfs_backpress_en = false;
    total_dfs_bp_cnt = 0;

    if (NUM_GROUPS > 1) {
        if (IS_LP5 || IS_GD2) {
            BL_n_min[BL16] = unsigned(ceil(float(BL16) / WCK2DFI_RATIO / 2));
            BL_n_max[BL16] = unsigned(ceil(float(BL16) / WCK2DFI_RATIO));
            BL_n_min[BL32] = unsigned(ceil(1.5 * float(BL32) / WCK2DFI_RATIO / 2));
            BL_n_max[BL32] = unsigned(ceil(float(BL32) / WCK2DFI_RATIO));
        } else if (IS_LP6) {
            BL_n_min[BL24] = unsigned(ceil(float(BL24) / WCK2DFI_RATIO / 2));
            BL_n_max[BL24] = unsigned(ceil(float(BL24) / WCK2DFI_RATIO));
            BL_n_min[BL48] = unsigned(ceil(1.5 * float(BL48) / WCK2DFI_RATIO / 2));
            BL_n_max[BL48] = unsigned(ceil(float(BL48) / WCK2DFI_RATIO));
        }
    } else {
        for (auto &it : MAP_CONFIG["BL"]) {
            BL_n_min[it] = ceil(float(it) / WCK2DFI_RATIO / 2);
            BL_n_max[it] = ceil(float(it) / WCK2DFI_RATIO / 2);
        }
    }

    if (PRINT_CMD_NUM) {
        CMDNUM_PRINT("DFI Cycle | ");
        CMDNUM_PRINT("            DMC Read           ||");
        CMDNUM_PRINT("            DMC Write          ||");
        CMDNUM_PRINT("           Gbuf Read           ||");
        CMDNUM_PRINT("           Gbuf Write          ||");
        CMDNUM_PRINT(endl);
        CMDNUM_PRINT("Qos       | ");
        for (size_t i = 0; i < 4; i ++) {
            for (size_t j = 0; j < 8; j ++) {
                CMDNUM_PRINT(setw(3)<<j<<"|");
            }
            CMDNUM_PRINT("|");
        }
        CMDNUM_PRINT(endl);
    }

    for (size_t i = 0; i <= TRANS_QUEUE_DEPTH; i ++) {
        que_cmd_time.push_back(0);
    }
    bg_intlv_cnt = 0;
    pre_rdata_time = 0xFFFFFFFFFFFFFFFF;
    pre_wresp_time = 0xFFFFFFFFFFFFFFFF;
    core_concurr_en = false;
    rw_cmd_num = 0;
    act_cmd_num = 0;
    tout_high_pri = 0xFFFF;
    rd_met_pd_cnt = 0;
    rd_met_asref_cnt = 0;
    cmd_met_pd_cnt = 0;
    cmd_met_asref_cnt = 0;
    page_act_cnt = 0;
    page_rw_cnt = 0;
    samerow_mask_rdcnt = 0;
    samerow_mask_wrcnt = 0;
    for (size_t i = 0; i < 32; i ++) {
        samerow_bit_rdcnt.push_back(0);
        samerow_bit_wrcnt.push_back(0);
    }
    pbr_cc_cnt = 0;
    abr_cc_cnt = 0;
    has_bypact_exec = false;
    sch_tout_cmd = false;
    sch_tout_type = DATA_READ;
    sch_tout_rank = 0;
    opc_cnt = 0;
    ppc_cnt = 0;
    rw_exec_cnt = 0;
    table_use_cnt = 0;
    page_adpt_win_cnt = 0;
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        send_wckfs.push_back(false);
        pbr_hold_pre.push_back(false);
        pbr_hold_pre_time.push_back(0);
        max_rcmd_bank.push_back(0);
        max_wcmd_bank.push_back(0);
    }

    com_highqos_read_cnt = 0;
    total_highqos_latency = 0;
    highqos_max_delay = 0;
    highqos_max_delay_id = 0;
    ddrc_av_highqos_lat = 0;
    highqos_trig_grpsw_cnt = 0;
    que_read_highqos_cnt.reserve(NUM_RANKS);
    rank_cmd_high_qos.reserve(NUM_RANKS);
    rank_rhit_num.reserve(NUM_RANKS);
    rank_ddrc_av_highqos_lat.reserve(NUM_RANKS);
    rank_total_highqos_latency.reserve(NUM_RANKS);
    rank_com_highqos_read_cnt.reserve(NUM_RANKS);
    highqos_r_bank_cnt.reserve(NUM_BANKS * NUM_RANKS);
    sbr_gap_cnt.reserve(NUM_RANKS);
    ref_offset.reserve(NUM_RANKS);
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        que_read_highqos_cnt.push_back(0);
        rank_cmd_high_qos.push_back(false);
        rank_rhit_num.push_back(0);
        rank_ddrc_av_highqos_lat.push_back(0);
        rank_total_highqos_latency.push_back(0);
        rank_com_highqos_read_cnt.push_back(0);
        sbr_gap_cnt.push_back(0);
        if (AREF_OFFSET_EN) ref_offset.push_back((tREFI / NUM_RANKS) * i);
        else ref_offset.push_back(0);
        for (size_t j = 0; j < NUM_BANKS; j ++) {
            highqos_r_bank_cnt.push_back(0);
        }
    }
    grp_sid_level = MAP_CONFIG["GRP_SID_LEVEL"];

//    DEBUG(now()<<" after ini, sc_num="<<sc_num);
}

void MemoryController::PostCalcTiming() {
    // -- (4{pipe} + 5{bp_pipe} + 1{pipe} + 3{Match UT}) * func_clk_ratio
    if (sub_cha == 0) {
        PD_PRD = ceil(float(PD_PRD) / tDFI);
        ASREF_PRD = ceil(float(ASREF_PRD) / tDFI);
        ASREF_ADAPT_WIN = ceil(float(ASREF_ADAPT_WIN) / tDFI);
        SBR_IDLE_ADAPT_WIN = ceil(float(SBR_IDLE_ADAPT_WIN) / tDFI);
        tREFI = ceil((tREFI / DERATING_RATIO));
        
        PRINT_BW_WIN = ceil(float(PRINT_BW_WIN) / tDFI);
    }

    if (channel == 0) {    // MAP_CONFIG obly read once
        for (size_t i = 0; i < MAP_CONFIG["ASREF_ADAPT_PRD"].size(); i ++) {
            MAP_CONFIG["ASREF_ADAPT_PRD"][i] = ceil(float(MAP_CONFIG["ASREF_ADAPT_PRD"][i]) / tDFI);
        }
    }

    if (DMC_RATE > 3200) {
        trfcpb = tRFCpb + (OFREQ_EN ? 6 : 13); // add state machine pipe cycle for pbr
        trfcab = tRFCab + (OFREQ_EN ? 6 : 13); // add state machine pipe cycle for abr
    } else {
        trfcpb = tRFCpb + 6; // add state machine pipe cycle for pbr
        trfcab = tRFCab + 6; // add state machine pipe cycle for abr
    }

    if (DERATING_EN && sub_cha == 0) {
        tRCD    = tRCD + ceil(1.875 / tDFI);
        tRCD_WR = tRCD_WR + ceil(1.875 / tDFI);
        tRAS    = tRAS + ceil(1.875 / tDFI);
        tRPpb   = tRPpb + ceil(1.875 / tDFI);
        tRPab   = tRPab + ceil(1.875 / tDFI);
    }

    if (OFREQ_EN && sub_cha == 0) {
        OPENPAGE_TIME_RD = OPENPAGE_TIME_RD >> 1;
        OPENPAGE_TIME_WR = OPENPAGE_TIME_WR >> 1;
        ENH_PAGE_ADPT_WIN = ENH_PAGE_ADPT_WIN >> 1;
        tREFI = tREFI & 0xFFFFFFFE;
    }

    if (OFREQ_EN && channel == 0) {      //MAP_CONFIG only read once
        unsigned size = MAP_CONFIG["TIMEOUT_PRI_RD"].size();
        for (size_t i = 0; i < size; i ++) {
            MAP_CONFIG["TIMEOUT_PRI_RD"][i] = MAP_CONFIG["TIMEOUT_PRI_RD"][i] >> 1;
        }
        size = MAP_CONFIG["TIMEOUT_PRI_WR"].size();
        for (size_t i = 0; i < size; i ++) {
            MAP_CONFIG["TIMEOUT_PRI_WR"][i] = MAP_CONFIG["TIMEOUT_PRI_WR"][i] >> 1;
        }
    }


    if (DMC_RATE < 3000) {
        PCFG_RANKTRTR_CASFS = 15;
        PCFG_RANKTRTW_CASFS = 17;
        PCFG_RANKTWTR_CASFS = 9;
        PCFG_RANKTWTW_CASFS = 13;
    } else if (DMC_RATE == 3063) {
        PCFG_RANKTRTR_CASFS = 15;
        PCFG_RANKTRTW_CASFS = 19;
        PCFG_RANKTWTR_CASFS = 8;
        PCFG_RANKTWTW_CASFS = 14;
    } else if (DMC_RATE == 4266) {
        PCFG_RANKTRTR_CASFS = 9;
        PCFG_RANKTRTW_CASFS = 15;
        PCFG_RANKTWTR_CASFS = 7;
        PCFG_RANKTWTW_CASFS = 15;
    } else if (DMC_RATE == 5500) {
        PCFG_RANKTRTR_CASFS = 9;
        PCFG_RANKTRTW_CASFS = 17;
        PCFG_RANKTWTR_CASFS = 7;
        PCFG_RANKTWTW_CASFS = 17;
    } else {
        PCFG_RANKTRTR_CASFS = PCFG_RANKTRTR;
        PCFG_RANKTRTW_CASFS = PCFG_RANKTRTW;
        PCFG_RANKTWTR_CASFS = PCFG_RANKTWTR;
        PCFG_RANKTWTW_CASFS = PCFG_RANKTWTW;
    }
}

//get a bus packet from either data or cmd bus
void MemoryController::receiveFromBus(unsigned long long task, bool mask_wcmd) {
    //add to return read data queue
    if (mask_wcmd) {
        data_delay = tD_D + tDAT_PHY + tDAT_RASC + 2;
    } else {
        data_delay = tD_D + tDAT_PHY + tDAT_RASC;
    }
    gen_rdata(task, 1, data_delay, mask_wcmd);
    if (DEBUG_BUS) {
        PRINTN(setw(10)<<now()<<" -- R DDR :: Receiving From Data Bus, task="<<task<<endl);
    }
}

void MemoryController::Cmd2Dfi_statistics(uint64_t task, uint64_t timeAdded, unsigned qos,
        unsigned mid, unsigned pf_type, unsigned rank) {
    uint32_t delay = (now() + 1 - timeAdded);

    if      (float(delay) * tDFI < 50  ) {qos_level_cnt[qos][0] ++;}
    else if (float(delay) * tDFI < 100 ) {qos_level_cnt[qos][1] ++;}
    else if (float(delay) * tDFI < 150 ) {qos_level_cnt[qos][2] ++;}
    else if (float(delay) * tDFI < 200 ) {qos_level_cnt[qos][3] ++;}
    else if (float(delay) * tDFI < 300 ) {qos_level_cnt[qos][4] ++;}
    else if (float(delay) * tDFI < 500 ) {qos_level_cnt[qos][5] ++;}
    else if (float(delay) * tDFI < 600 ) {qos_level_cnt[qos][6] ++;}
    else if (float(delay) * tDFI < 750 ) {qos_level_cnt[qos][7] ++;}
    else if (float(delay) * tDFI < 1000) {qos_level_cnt[qos][8] ++;}
    else if (float(delay) * tDFI < 1500) {qos_level_cnt[qos][9] ++;}
    else if (float(delay) * tDFI < 2000) {qos_level_cnt[qos][10] ++;}
    else                                 {qos_level_cnt[qos][11] ++;}

    float delay_ns = tDFI * delay;
    uint8_t size = lat_dly_cnt.size();
    for (size_t i = 0; i < size; i ++) {
        if (delay_ns < float(lat_dly_step[i])) {
            lat_dly_cnt[i] ++;
            break;
        }
    }

    qos_cnt[qos] ++;
    qos_delay_cnt[qos] += delay;
    mid_cnt[mid] ++;
    mid_delay_cnt[mid] += delay;
    pf_cnt[pf_type] ++;
    pf_delay_cnt[pf_type] += delay;

    if (mid >= CPU_MID_START && mid <= CPU_MID_END && qos <= SWITCH_HQOS_LEVEL) {
        com_highqos_read_cnt ++;
        total_highqos_latency += delay;
        ddrc_av_highqos_lat = float(total_highqos_latency) / com_highqos_read_cnt;
        rank_com_highqos_read_cnt[rank] ++;
        rank_total_highqos_latency[rank] += delay;
        rank_ddrc_av_highqos_lat[rank] = float(rank_total_highqos_latency[rank]) / rank_com_highqos_read_cnt[rank];
    }
    if (mid >= CPU_MID_START && mid <= CPU_MID_END && qos <= SWITCH_HQOS_LEVEL) {
        if (delay > highqos_max_delay) {
            highqos_max_delay = delay;
            highqos_max_delay_id = task;
        }
    }
}

void MemoryController::ReturnData_statistics(uint64_t task, uint64_t timeAdded, unsigned qos,
        unsigned mid, unsigned pf_type, unsigned rank) {
    uint32_t delay = (now() - timeAdded);
    if (DEBUG_BUS) {
        PRINTN(setw(10)<<now()<<" -- OVER :: Read DELAY ("<<delay<<") task="<<task<<endl);
    }

    if (PRINT_RDATA) {
        TRACE_PRINT(setw(10)<<now()<<" -- Rdata task="<<task<<", latency="<<delay<<endl);
    }

    if (delay > 2500) {
        dly_ex2000_cnt ++;
        PRINTN(setw(16)<<now()<<"Read DMC["<<delay<<"], num ["<<dly_ex2000_cnt<<"]"<<endl);
    }
    com_read_cnt ++;
    total_latency += delay;
    ddrc_av_lat = float(total_latency) / com_read_cnt;

    total_latency_rank[rank] += delay;
    com_read_cnt_rank[rank] ++;
    ddrc_av_lat_rank[rank] = float(total_latency_rank[rank]) / com_read_cnt_rank[rank];

    if (delay > max_delay) {
        max_delay = delay;
        max_delay_id = task;
    }
    if (delay < min_delay || min_delay == 0) {
        min_delay = delay;
        min_delay_id = task;
    }
}
//sends read data back to the CPU
bool MemoryController::returnReadData(unsigned int channel_num,unsigned long long task,
        double readDataEnterDmcTime, double reqAddToDmcTime, double reqEnterDmcBufTime) {
    if (parentMemorySystem->ReturnReadData!=NULL) {
        return (*parentMemorySystem->ReturnReadData)(channel_num, task,
                readDataEnterDmcTime, reqAddToDmcTime, reqEnterDmcBufTime);
    } else {
        return false;
    }
}

//receive the write data from CPU
void MemoryController::receiveFromCPU(unsigned int *data, uint64_t task) {
    unsigned second_time = 0;
    unsigned third_time = 0;
    if (!IECC_ENABLE || (!tasks_info[task].wr_ecc && !tasks_info[task].rd_ecc)) {
        if (pre_dat_time == now()) {
           ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] DATA received in the same cycle");
           assert(0);
        }
        if (third_time == 1) {
            third_time = 0;
            pre_dat_time = now();
        } else if (second_time == 1) {
            second_time = 0;
            third_time = 1;
        } else {
            second_time = 1;
        }
    }
    wdata_cnt ++;
    wdata_pipe wdata;
    wdata.task = task;
    wdata.delay = now() + tWDATA_DMC;
    WdataPipe.push_back(wdata);
    pre_req_data_time= now();
    if (DEBUG_BUS) {
        PRINTN(setw(10)<<now()<<" -- R_CPU :: Get Data from CPU, task="<<task<<endl);
    }
}

//gives the memory controller a handle on the rank objects
void MemoryController::attachRanks(vector<Rank *> *rank) {
    this->ranks = rank;
}

unsigned MemoryController::CalcCasTiming(unsigned bl, unsigned sync, unsigned wck_pst) {
    unsigned ret = 0;
    if (NUM_GROUPS > 1) { // BG mode
        ret = ceil(float(BL32) / WCK2DFI_RATIO) + sync + wck_pst;
    } else { // Bank mode
        ret = ceil(float(BL32) / 2 / WCK2DFI_RATIO) + sync + wck_pst;
    }
    return ret;
}

unsigned MemoryController::CalcTiming(bool is_trtp, unsigned cmd_bl, unsigned timing) {
    if (IS_LP5 || IS_GD2) {
        if (cmd_bl == BL32) {
            return timing;
        } else if (cmd_bl == BL16) {
            if (timing < ceil(float(BL32) / 2 / WCK2DFI_RATIO)) {
                return 0;
            } else {
                return timing - ceil(float(BL32) / 2 / WCK2DFI_RATIO);
            }
        } else {
            ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] No such BLEN: "<<cmd_bl);
            assert(0);
        }
    } else if (IS_LP6) {
        if (cmd_bl == BL48) {
            return timing;
        } else if (cmd_bl == BL24) {
            if (timing < ceil(float(BL48) / 2 / WCK2DFI_RATIO)) {
                return 0;
            } else {
                return timing - ceil(float(BL48) / 2 / WCK2DFI_RATIO);
            }
        } else {
            ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] No such BLEN: "<<cmd_bl);
            assert(0);
        }
    } else if (IS_LP4 || IS_DDR5 || IS_DDR4 || IS_GD1 || IS_HBM2E || IS_HBM3) {
        if (is_trtp) {
            if (IS_LP4 && cmd_bl == BL32) {
                return timing + ceil(float(BL16) / 2 / WCK2DFI_RATIO);
            } else {
                if (cmd_bl == BL32) return timing;
                else return timing - (8 / unsigned(WCK2DFI_RATIO));
            }
        } else {
            if (timing < ceil(float(BLEN - cmd_bl) / 2 / WCK2DFI_RATIO)) {
                return 0;
            } else {
                return timing - ceil(float(BLEN - cmd_bl) / 2 / WCK2DFI_RATIO);
            }
        }
    } else { // IS_G3D
        if (timing < ceil(float(BLEN - cmd_bl) / 2 / WCK2DFI_RATIO)) {
            return 0;
        } else {
            return timing - ceil(float(BLEN - cmd_bl) / 2 / WCK2DFI_RATIO);
        }
    }
}

unsigned MemoryController::CalcTccd(bool is_samebg, unsigned cmd_bl, unsigned tccd) {
    if (IS_LP5 || IS_GD2) {
        if (cmd_bl == BL32) {
            return tccd;
        } else if (cmd_bl == BL16) {
            if (is_samebg) {
                if (NUM_GROUPS > 1) return (BL16 / unsigned(WCK2DFI_RATIO));
                else return (BL16 / 2 / unsigned(WCK2DFI_RATIO));
            } else {
                return tccd;
            }
        } else {
            ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] No such BLEN: "<<cmd_bl);
            assert(0);
        }
//    } else if (IS_LP6) {
//        if (cmd_bl == BL48) {
//            return tccd;
//        } else if (cmd_bl == BL24) {
//            if (is_samebg) {
//                if (NUM_GROUPS > 1) return (BL24 / unsigned(WCK2DFI_RATIO));
//                else return (BL24 / 2 / unsigned(WCK2DFI_RATIO));
//            } else {
//                return tccd;
//            }
//        } else {
//            ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] No such BLEN: "<<cmd_bl);
//            assert(0);
//        }
    } else if (IS_LP6) {
        if (cmd_bl == BL48) {
            if (is_samebg) {
                if (NUM_GROUPS > 1) return tCCD_L48;
                else return tccd;
            } else {
                return tccd;
            }
        } else if (cmd_bl == BL24) {
            if (is_samebg) {
                if (NUM_GROUPS > 1) return tCCD_L24;
                else return (BL24 / 2 / unsigned(WCK2DFI_RATIO));
            } else {
                return tccd;
            }
        } else {
            ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] No such BLEN: "<<cmd_bl);
            assert(0);
        }
    } else if (IS_DDR5) {
        if (is_samebg) {
            //return max(ceil(float(cmd_bl) / 2 / WCK2DFI_RATIO), ceil(float(5) / tDFI));
            return max(ceil(float(cmd_bl) / 2 / WCK2DFI_RATIO), float(tCCD_L));
        } else {
            return ceil(float(cmd_bl) / 2 / WCK2DFI_RATIO);
        }
    } else if (IS_HBM2E || IS_HBM3) {
        return tccd;
    } else if (IS_DDR4) {
        return tccd;
    } else if (IS_LP4) {
        return ceil(float(cmd_bl) / 2 / WCK2DFI_RATIO);
    } else if (IS_GD1) {
        if (is_samebg) {
            return tccd;
        } else {
            return ceil(float(cmd_bl) / 2 / WCK2DFI_RATIO);
        }
    } else { // IS_G3D
        if (!is_samebg) {
            if (cmd_bl == BLEN) {
                return tccd;
            } else {
                return tccd / 2;
            }
        } else {
            return ceil(float(cmd_bl) / 2 / WCK2DFI_RATIO);
        }
    }
}

int MemoryController::CalcCmdCycle(uint8_t pre_cmd, uint8_t next_cmd) {
    return (pre_cmd - next_cmd);
}

unsigned MemoryController::CalcWrite2Mwrite(bool is_samebg, bool is_sameba, unsigned cmd_bl) {
    if (IS_LP4) {
        if (is_sameba) {
            if (cmd_bl == BL16) return tCCDMW;
            else return tCCDMW + unsigned(float(BL16) / 2 / WCK2DFI_RATIO);
        } else {
            return unsigned(float(cmd_bl) / 2 / WCK2DFI_RATIO);
        }
    } else if (IS_LP5 || IS_LP6 || IS_GD2) {
        if (NUM_GROUPS > 1) { // BG mode
            if (is_sameba) {
                if (cmd_bl == BL16 || cmd_bl == BL24) { 
                    if (DMC_RATE <= 8533){
                        return 4 * BL_n_max[cmd_bl];
                    } else { // 9600/10667
                        return 5 * BL_n_max[cmd_bl];
                    }
                } else {
                    if (DMC_RATE <= 8533) {
                        return unsigned(2.5 * float(BL_n_max[cmd_bl]));
                    } else {
                        return unsigned(3 * float(BL_n_max[cmd_bl]));
                    }
                }
            } else if (is_samebg) {
                return BL_n_max[cmd_bl];
            } else {
                if (cmd_bl == BL16 || cmd_bl == BL24) return unsigned(float(cmd_bl) / 2 / WCK2DFI_RATIO);
                else return unsigned(float(cmd_bl) / 4 / WCK2DFI_RATIO);
            }
        } else { // Bank mode
            if (is_sameba) {
                if (cmd_bl == BL16 || cmd_bl == BL24) return unsigned(4 * float(cmd_bl) / 2 / WCK2DFI_RATIO);
                else return unsigned(2.5 * float(cmd_bl) / 2 / WCK2DFI_RATIO);
            } else {
                return unsigned(float(cmd_bl) / 2 / WCK2DFI_RATIO);
            }
        }
    } else {
        return tCCDMW;
    }
}

unsigned MemoryController::CalcMwrite2Write(bool is_samebg, bool is_sameba, unsigned cmd_bl) {
    if (IS_LP4) {
        return unsigned(float(cmd_bl) / 2 / WCK2DFI_RATIO);
    } else if (IS_LP5 || IS_LP6 || IS_GD2) {
        if (NUM_GROUPS > 1) { // BG mode
            if (is_samebg) return unsigned(2 * float(cmd_bl) / 2 / WCK2DFI_RATIO);
            else return unsigned(float(cmd_bl) / 2 / WCK2DFI_RATIO);
        } else { // Bank mode
            return unsigned(float(cmd_bl) / 2 / WCK2DFI_RATIO);
        }
    } else {
        return tCCDMW;
    }
}

unsigned MemoryController::CalcMwrite2Mwrite(bool is_samebg, bool is_sameba, unsigned cmd_bl) {
    if (IS_LP4) {
        if (is_sameba) return tCCDMW;
        else return unsigned(float(cmd_bl) / 2 / WCK2DFI_RATIO);
    } else if (IS_LP5 || IS_LP6 || IS_GD2) {
        if (NUM_GROUPS > 1) { // BG mode
            if (is_sameba) { 
                if (DMC_RATE <= 8533) {
                    return 4 * BL_n_max[cmd_bl];
                } else { // 9600/10667
                    return 5 * BL_n_max[cmd_bl];
                }
                
            }else if (is_samebg) return BL_n_max[cmd_bl];
            else return unsigned(float(cmd_bl) / 2 / WCK2DFI_RATIO);
        } else { // Bank mode
            if (is_sameba) return unsigned(4 * float(cmd_bl) / 2 / WCK2DFI_RATIO);
            else return unsigned(float(cmd_bl) / 2 / WCK2DFI_RATIO);
        }
    } else {
        return tCCDMW;
    }
}

/***************************************************************************************************
descriptor: This function is to refresh the timing, once there is a new command to send, it need to
update all the timing,
****************************************************************************************************/
void MemoryController::fresh_timing(const BusPacket &bus_packet,bool hit) {
    //update each bank's state based on the command that was just popped out of the command queue
    //for readability's sake
    unsigned rank = bus_packet.rank;
    unsigned bank = bus_packet.bank;
    unsigned group = bus_packet.group;
    unsigned sub_channel = (bus_packet.bankIndex % NUM_BANKS) / sc_bank_num;
    unsigned sid = bus_packet.sid;
//    DEBUG(now()<<" sc="<<sub_channel<<" sc_bank_num="<<sc_bank_num<<" NUM_BANKS="<<NUM_BANKS<<" sc_num="<<sc_num);
    unsigned bank_start = sub_channel * NUM_BANKS / sc_num;
//    unsigned bank_pair_start = sub_channel * pbr_bank_num;
    unsigned rw_intlv_cnt = 0;
    if (bus_packet.type >= WRITE_CMD && bus_packet.type <= READ_P_CMD) {
        bankStates[bus_packet.bankIndex].state->rwIntlvCountdown = BL_n_min[bus_packet.bl];
    }
    for (auto &state : bankStates) {
        if (state.state->rwIntlvCountdown > 0) rw_intlv_cnt ++;
    }
    unsigned trp_pb = bus_packet.fg_ref ? tRPfg : tRPpb;
    switch (bus_packet.type) {
        case READ_CMD :
        case READ_P_CMD :{     //todo: revise for e-mode
            for (auto &state : bankStates) {
                unsigned state_channel = (state.bankIndex % NUM_BANKS) / sc_bank_num;
                if (state.rank == rank) { // same rank
                    if (state_channel == sub_channel) { // same rank, same subchannel
                        if (state.sid == sid) { // same rank, same sid
                            if (state.group == group) { // same rank, same sid, same bg
                                if (state.bank == bank) { // same rank, same sid, same bg, same bank
                                    if (bus_packet.type == READ_P_CMD) { // same rank, same sid, same bg, same bank, read ap
                                        //fix :in order to prenvent rot-hit command to send a read or write request
                                        state.state->currentBankState = Precharging;
                                        if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2) {
                                            state.state->nextActivate1 = max(now() + CalcTiming(true, bus_packet.bl, PCFG_TRTP)
                                                    + trp_pb - unsigned(ceil(OFREQ_RATIO * 2))
                                                    + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextActivate1);
                                            state.state->nextActivate2 = max(now() + CalcTiming(true, bus_packet.bl, PCFG_TRTP)
                                                    + trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextActivate2);
                                        } else if (IS_LP4 || IS_LP5 || IS_GD2 || IS_HBM2E || IS_HBM3) {
                                            state.state->nextActivate1 = max(now() + CalcTiming(true, bus_packet.bl, PCFG_TRTP)
                                                    + trp_pb - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(rw_cycle, cmd_cycle),
                                                    state.state->nextActivate1);
                                        } else if (IS_DDR5 || IS_DDR4 || IS_GD1 || IS_G3D) {
                                            state.state->nextActivate2 = max(now() + CalcTiming(true, bus_packet.bl, PCFG_TRTP)
                                                    + trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextActivate2);
                                        }
                                        state.state->nextPerBankRefresh = max(now() + CalcTiming(true, bus_packet.bl, PCFG_TRTP)
                                                + trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextPerBankRefresh);
                                        state.state->nextAllBankRefresh = max(now() + CalcTiming(true, bus_packet.bl, PCFG_TRTP)
                                                + trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextAllBankRefresh);
                                        state.state->stateChangeEn = true;
                                        state.state->stateChangeCountdown = CalcTiming(true, bus_packet.bl, PCFG_TRTP) +
                                            CalcCmdCycle(rw_cycle, 1);
                                        if (IS_GD1 || IS_GD2 || IS_G3D) {
                                            for (size_t ba = 0; ba < NUM_BANKS; ba ++) {
                                                uint32_t bank_tmp = rank * NUM_BANKS + ba;
                                                unsigned bg = ba * NUM_GROUPS / NUM_BANKS;
                                                unsigned tppd = (bg == group) ? tPPD_L : tPPD;
                                                bankStates[bank_tmp].state->nextReadAp = max(CalcCmdCycle(cmd_cycle, cmd_cycle)
                                                        + now() + tppd, bankStates[bank_tmp].state->nextReadAp);
                                            }
                                        }
                                    } else { // same rank, same sid, same bg, same bank, read
                                        state.state->nextPrecharge = max(now() + CalcTiming(true, bus_packet.bl, PCFG_TRTP)
                                                + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextPrecharge);
                                    }
                                    state.last_activerow = bus_packet.row;
                                    if (IS_DDR5) {
                                        state.state->nextRead = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) +
                                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                        state.state->nextReadAp = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) +
                                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                                    }
                                } else { // same rank, same sid, same bg, diff bank
                                    if (IS_DDR5) {
                                        state.state->nextRead = max(now() + tCCD_M + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                        state.state->nextReadAp = max(now() + tCCD_M + CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                                    }
                                }
                                if (!IS_DDR5) {
                                    state.state->nextRead = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                    state.state->nextReadAp = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                                }
                                state.state->nextWrite = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW_L) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                                state.state->nextWriteAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW_L) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                                state.state->nextWriteRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW_L) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                                state.state->nextWriteApRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW_L) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                                state.state->nextWriteMask = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW_L) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                                state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW_L) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                            } else { // same rank, same sid, diff bg
                                state.state->nextRead = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                state.state->nextReadAp = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                                if (IS_G3D) {
                                    state.state->nextWrite = max(now() + PCFG_TRTW + CalcCmdCycle(rw_cycle, rw_cycle),
                                            state.state->nextWrite);
                                    state.state->nextWriteAp = max(now() + PCFG_TRTW + CalcCmdCycle(rw_cycle, rw_cycle),
                                            state.state->nextWriteAp);
                                    state.state->nextWriteRmw = max(now() + PCFG_TRTW + CalcCmdCycle(rw_cycle, rw_cycle),
                                            state.state->nextWriteRmw);
                                    state.state->nextWriteApRmw = max(now() + PCFG_TRTW + CalcCmdCycle(rw_cycle, rw_cycle),
                                            state.state->nextWriteApRmw);
                                    state.state->nextWriteMask = max(now() + PCFG_TRTW + CalcCmdCycle(rw_cycle, rw_cycle),
                                            state.state->nextWriteMask);
                                    state.state->nextWriteMaskAp = max(now() + PCFG_TRTW + CalcCmdCycle(rw_cycle, rw_cycle),
                                            state.state->nextWriteMaskAp);
                                } else {
                                    state.state->nextWrite = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                                    state.state->nextWriteAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                                    state.state->nextWriteRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                                    state.state->nextWriteApRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                                    state.state->nextWriteMask = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                                    state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                                }
                            }
                            if (bus_packet.type == READ_P_CMD) {
                                funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tRDAPPD) +
                                        CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                            } else {
                                funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tRDPD) +
                                        CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                            }
                        } else { // same rank, diff sid
                            state.state->nextRead = max(now() + CalcTccd(false, bus_packet.bl, tCCD_R) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                            state.state->nextWrite = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                            state.state->nextWriteMask = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        }
                    } else {  // same rank, diff subchannel for lp6
                        if (state.group == group) {  // same group
                            state.state->nextWrite = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW_L) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                            state.state->nextWriteAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW_L) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                            state.state->nextRead = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                            state.state->nextReadAp = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        } else {   // diff group
                            state.state->nextWrite = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                            state.state->nextWriteAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TRTW) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                            state.state->nextRead = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                            state.state->nextReadAp = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        }
                        if (bus_packet.type == READ_P_CMD) {
                            funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tRDAPPD) +
                                    CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                        } else {
                            funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tRDPD) +
                                    CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                        }
                    }
                } else { // diff rank
                    if (WCK_ALWAYS_ON || send_wckfs[state.rank]) {
                        state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTR_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                        state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTR_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        state.state->nextWrite = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        state.state->nextWriteRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                        state.state->nextWriteApRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                        state.state->nextWriteMask = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                    } else {
                        state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTR) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                        state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTR) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        state.state->nextWrite = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        state.state->nextWriteRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                        state.state->nextWriteApRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                        state.state->nextWriteMask = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTRTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                    }
                }
            }
            RankState[rank].wck_off_time = now() + CalcCasTiming(bus_packet.bl, RL, 0);
            RankState[rank].wck_on = true;
            send_wckfs[rank] = false;
            break;
        }
        case WRITE_CMD :
        case WRITE_P_CMD :{      //todo: revise for e-mode
            for (auto &state : bankStates) {
                unsigned state_channel = (state.bankIndex % NUM_BANKS) / sc_bank_num;
                if (state.rank == rank) { // same rank
                    if (state_channel == sub_channel) { // same rank, same subchannel
                        if (state.sid == sid) { // same rank, same sid
                            if (state.group == group) { // same rank, same sid, same bg
                                if (state.bank == bank) { // same rank, same sid, same bg, same bank
                                    if (bus_packet.type == WRITE_P_CMD) { // same rank, same sid, same bg, same bank, write ap
                                        state.state->currentBankState = Precharging;
                                        if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2) {
                                            state.state->nextActivate1 = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)
                                                    + trp_pb - unsigned(ceil(OFREQ_RATIO * 2)) +
                                                    CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextActivate1);
                                            state.state->nextActivate2 = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)
                                                    + trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextActivate2);
                                        } else if (IS_LP4 || IS_LP5 || IS_GD2 || IS_HBM2E || IS_HBM3) {
                                            state.state->nextActivate1 = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)
                                                    + trp_pb - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(rw_cycle, cmd_cycle),
                                                    state.state->nextActivate1);
                                        } else if (IS_DDR5 || IS_DDR4 || IS_GD1 || IS_G3D) {
                                            state.state->nextActivate2 = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)
                                                    + trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextActivate2);
                                        }
                                        state.state->stateChangeEn = true;
                                        state.state->stateChangeCountdown = CalcTiming(false, bus_packet.bl, PCFG_TWR) +
                                            CalcCmdCycle(rw_cycle, 1);
                                        state.state->nextPerBankRefresh = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)
                                                + trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextPerBankRefresh);
                                        state.state->nextAllBankRefresh = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)
                                                + trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextAllBankRefresh);
                                        if (IS_GD1 || IS_GD2 || IS_G3D) {
                                            for (size_t ba = 0; ba < NUM_BANKS; ba ++) {
                                                uint32_t bank_tmp = rank * NUM_BANKS + ba;
                                                unsigned bg = ba * NUM_GROUPS / NUM_BANKS;
                                                unsigned tppd = (bg == group) ? tPPD_L : tPPD;
                                                bankStates[bank_tmp].state->nextWriteAp = max(CalcCmdCycle(cmd_cycle, cmd_cycle)
                                                        + now() + tppd, bankStates[bank_tmp].state->nextWriteAp);
                                            }
                                        }
                                    }
                                    state.state->nextPrecharge = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR) +
                                            CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextPrecharge);
                                    if (IS_G3D) {
                                        state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_SB) +
                                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                        state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_SB) +
                                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                                    } else {
                                        state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_L) +
                                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                        state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_L) +
                                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                                    }
                                    state.state->nextWriteMask = max(now() + CalcWrite2Mwrite(true, true, bus_packet.bl) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                                    state.state->nextWriteMaskAp = max(now() + CalcWrite2Mwrite(true, true, bus_packet.bl) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                                    if (IS_DDR5) {
                                        unsigned tccd_l_wr = (bus_packet.bl == BL16) ? tCCD_L_WR : (tCCD_L_WR+unsigned(8/WCK2DFI_RATIO));
                                        unsigned tccd_l_wr2 = (bus_packet.bl == BL16) ? tCCD_L_WR2 : (tCCD_L_WR2+unsigned(8/WCK2DFI_RATIO));
                                        state.state->nextWrite = max(now() + tccd_l_wr2 + CalcCmdCycle(rw_cycle, rw_cycle),
                                                state.state->nextWrite);
                                        state.state->nextWriteAp = max(now() + tccd_l_wr2 + CalcCmdCycle(rw_cycle, rw_cycle),
                                                state.state->nextWriteAp);
                                        state.state->nextWriteRmw = max(now() + tccd_l_wr + CalcCmdCycle(rw_cycle, rw_cycle),
                                                state.state->nextWriteRmw);
                                        state.state->nextWriteApRmw = max(now() + tccd_l_wr + CalcCmdCycle(rw_cycle, rw_cycle),
                                                state.state->nextWriteApRmw);
                                    }
                                } else { // same rank, same sid, same bg, diff bank
                                    if (IS_GD1) {
                                        state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) +
                                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                        state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) +
                                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                                    } else if (IS_DDR5) {
                                        unsigned tccd_l_wr2 = (bus_packet.bl==BL16)?tCCD_L_WR2:(tCCD_L_WR2+unsigned(8/WCK2DFI_RATIO));
                                        state.state->nextWrite = max(now() + tccd_l_wr2 + CalcCmdCycle(rw_cycle, rw_cycle),
                                                state.state->nextWrite);
                                        state.state->nextWriteAp = max(now() + tccd_l_wr2 + CalcCmdCycle(rw_cycle, rw_cycle),
                                                state.state->nextWriteAp);
                                        state.state->nextWriteRmw = max(now() + tCCD_M_WR + CalcCmdCycle(rw_cycle, rw_cycle),
                                                state.state->nextWriteRmw);
                                        state.state->nextWriteApRmw = max(now() + tCCD_M_WR + CalcCmdCycle(rw_cycle, rw_cycle),
                                                state.state->nextWriteApRmw);
                                    } else {
                                        state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_L) +
                                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                        state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_L) +
                                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                                    }
                                    state.state->nextWriteMask = max(now() + CalcWrite2Mwrite(true, false, bus_packet.bl) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                                    state.state->nextWriteMaskAp = max(now() + CalcWrite2Mwrite(true, false, bus_packet.bl) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                                }
                                state.last_activerow = bus_packet.row;       //todo: revise for e-mode
                                if (!IS_DDR5) {
                                    state.state->nextWrite = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                                    state.state->nextWriteAp = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                                    state.state->nextWriteRmw = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                                    state.state->nextWriteApRmw = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                                }
                            } else { // same rank, same sid, diff bg
                                state.state->nextWrite = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                                state.state->nextWriteAp = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                                state.state->nextWriteRmw = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                                state.state->nextWriteApRmw = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                                state.state->nextWriteMask = max(now() + CalcWrite2Mwrite(false, false, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                                state.state->nextWriteMaskAp = max(now() + CalcWrite2Mwrite(false, false, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                                if (IS_G3D) {
                                    state.state->nextRead = max(now() + PCFG_TWTR + CalcCmdCycle(rw_cycle, rw_cycle),
                                            state.state->nextRead);
                                    state.state->nextReadAp = max(now() + PCFG_TWTR + CalcCmdCycle(rw_cycle, rw_cycle),
                                            state.state->nextReadAp);
                                } else {
                                    state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                    state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) +
                                            CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                                }
                            }
                            if (bus_packet.type == WRITE_CMD) {
                                funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tWRPD) +
                                        CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                            } else {
                                funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tWRAPPD) +
                                        CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                            }
                        } else { // same rank, diff sid
                            state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                            state.state->nextWrite = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                            state.state->nextWriteMask = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        }
                    } else {  // same rank, diff subchannel
                        if (state.group == group) {  // same group
                            state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_L) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                            state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_L) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                            state.state->nextWrite = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                            state.state->nextWriteAp = max(now() + CalcTccd(true, bus_packet.bl, tCCD_L) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        } else {  // diff group
                            state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                            state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                            state.state->nextWrite = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                            state.state->nextWriteAp = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        }
                        if (bus_packet.type == WRITE_CMD) {
                            funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tWRPD) +
                                    CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                        } else {
                            funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tWRAPPD) +
                                    CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                        }
                    }
                } else { // diff rank
                    if (WCK_ALWAYS_ON || send_wckfs[state.rank]) {
                        state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                        state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        state.state->nextWrite = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        state.state->nextWriteRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                        state.state->nextWriteApRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                        state.state->nextWriteMask = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                    } else {
                        state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                        state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        state.state->nextWrite = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        state.state->nextWriteRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                        state.state->nextWriteApRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                        state.state->nextWriteMask = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                    }
                }
            }
            RankState[rank].wck_off_time = now() + CalcCasTiming(bus_packet.bl, WL, 0);
            RankState[rank].wck_on = true;
            send_wckfs[rank] = false;
            break;
        }
        case WRITE_MASK_CMD :// mask write is always BL16, mask write not used for lpddr6   
        case WRITE_MASK_P_CMD :{
            for (auto &state : bankStates) {
                if (state.rank == rank) { // same rank
                    if (state.sid == sid) { // same rank, same sid
                        if (state.group == group) { // same rank, same sid, same bg
                            if (state.bank == bank) { // same rank, same sid, same bg, same bank
                                if (bus_packet.type == WRITE_MASK_P_CMD) { // same rank, same sid, same bg, same bank, mask write ap
                                    state.state->currentBankState = Precharging;
                                    if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2) {
                                        state.state->nextActivate1 = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)
                                                + trp_pb - unsigned(ceil(OFREQ_RATIO * 2)) +
                                                CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextActivate1);
                                        state.state->nextActivate2 = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)
                                                + trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextActivate2);
                                    } else if (IS_LP4 || IS_LP5 || IS_GD2 || IS_HBM2E || IS_HBM3) {
                                        state.state->nextActivate1 = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)
                                                + trp_pb - unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(rw_cycle, cmd_cycle),
                                                state.state->nextActivate1);
                                    } else if (IS_DDR5 || IS_DDR4 || IS_GD1 || IS_G3D) {
                                        state.state->nextActivate2 = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR)
                                                + trp_pb + CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextActivate2);
                                    }
                                    state.state->stateChangeEn = true;
                                    state.state->stateChangeCountdown = CalcTiming(false, bus_packet.bl, PCFG_TWR) +
                                        CalcCmdCycle(rw_cycle, 1);
                                    if (IS_GD1 || IS_GD2 || IS_G3D) {
                                        for (size_t ba = 0; ba < NUM_BANKS; ba ++) {
                                            uint32_t bank_tmp = rank * NUM_BANKS + ba;
                                            unsigned bg = ba * NUM_GROUPS / NUM_BANKS;
                                            unsigned tppd = (bg == group) ? tPPD_L : tPPD;
                                            bankStates[bank_tmp].state->nextWriteMaskAp = max(CalcCmdCycle(cmd_cycle, cmd_cycle)
                                                    + now() + tppd , bankStates[bank_tmp].state->nextWriteMaskAp);
                                        }
                                    }
                                }
                                state.state->nextPrecharge = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWR) +
                                        CalcCmdCycle(rw_cycle, cmd_cycle), state.state->nextPrecharge);
                                state.state->nextWrite = max(now() + CalcMwrite2Write(true, true, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                                state.state->nextWriteAp = max(now() + CalcMwrite2Write(true, true, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                                state.state->nextWriteRmw = max(now() + CalcMwrite2Write(true, true, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                                state.state->nextWriteApRmw = max(now() + CalcMwrite2Write(true, true, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                                state.state->nextWriteMask = max(now() + CalcMwrite2Mwrite(true, true, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                                state.state->nextWriteMaskAp = max(now() + CalcMwrite2Mwrite(true, true, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                                state.last_activerow = bus_packet.row;
                            } else { // same rank, same sid, same bg, diff bank
                                state.state->nextWrite = max(now() + CalcMwrite2Write(true, false, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                                state.state->nextWriteAp = max(now() + CalcMwrite2Write(true, false, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                                state.state->nextWriteRmw = max(now() + CalcMwrite2Write(true, false, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                                state.state->nextWriteApRmw = max(now() + CalcMwrite2Write(true, false, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                                state.state->nextWriteMask = max(now() + CalcMwrite2Mwrite(true, false, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                                state.state->nextWriteMaskAp = max(now() + CalcMwrite2Mwrite(true, false, bus_packet.bl) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                            }
                            state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_L) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                            state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR_L) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        } else { // same rank, same sid, diff bg
                            state.state->nextWrite = max(now() + CalcMwrite2Write(false, false, bus_packet.bl) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                            state.state->nextWriteAp = max(now() + CalcMwrite2Write(false, false, bus_packet.bl) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                            state.state->nextWriteRmw = max(now() + CalcMwrite2Write(false, false, bus_packet.bl) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                            state.state->nextWriteApRmw = max(now() + CalcMwrite2Write(false, false, bus_packet.bl) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                            state.state->nextWriteMask = max(now() + CalcMwrite2Mwrite(false, false, bus_packet.bl) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                            state.state->nextWriteMaskAp = max(now() + CalcMwrite2Mwrite(false, false, bus_packet.bl) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                            if (IS_G3D) {
                                state.state->nextRead = max(now() + PCFG_TWTR + CalcCmdCycle(rw_cycle, rw_cycle),
                                        state.state->nextRead);
                                state.state->nextReadAp = max(now() + PCFG_TWTR + CalcCmdCycle(rw_cycle, rw_cycle),
                                        state.state->nextReadAp);
                            } else {
                                state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                                state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) +
                                        CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                            }
                        }
                        if (bus_packet.type == WRITE_MASK_CMD) {
                            funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tWRPD) +
                                    CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                        } else {
                            funcState[rank].nextPde = max(now() + CalcTiming(false, bus_packet.bl, tWRAPPD) +
                                    CalcCmdCycle(rw_cycle, 1), funcState[rank].nextPde);
                        }
                    } else { // same rank, diff sid
                        state.state->nextWrite = max(now() + CalcTccd(false, bus_packet.bl, tCCD_S) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteMask = max(now() + CalcTiming(false, bus_packet.bl, tCCDMW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_TWTR) +
                                    CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                    }
                } else { // diff rank
                    if (WCK_ALWAYS_ON || send_wckfs[state.rank]) {
                        state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                        state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        state.state->nextWrite = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        state.state->nextWriteRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                        state.state->nextWriteApRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                        state.state->nextWriteMask = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW_CASFS) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                    } else {
                        state.state->nextRead = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextRead);
                        state.state->nextReadAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTR) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextReadAp);
                        state.state->nextWrite = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWrite);
                        state.state->nextWriteAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteAp);
                        state.state->nextWriteRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteRmw);
                        state.state->nextWriteApRmw = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteApRmw);
                        state.state->nextWriteMask = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMask);
                        state.state->nextWriteMaskAp = max(now() + CalcTiming(false, bus_packet.bl, PCFG_RANKTWTW) +
                                CalcCmdCycle(rw_cycle, rw_cycle), state.state->nextWriteMaskAp);
                    }
                }
            }
            RankState[rank].wck_off_time = now() + CalcCasTiming(bus_packet.bl, WL, 0);
            RankState[rank].wck_on = true;
            send_wckfs[rank] = false;
            break;
        }
        case ACTIVATE1_CMD:{         //todo: revise for e-mode
            if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2) {
                for (auto &state : bankStates) {
                    unsigned state_channel = (state.bankIndex % NUM_BANKS) / sc_bank_num;
                    if (rank == state.rank) {   // same rank
                        if (sub_channel == state_channel) {   // same rank, same subchannel
                            if (state.group == group && state.bank == bank) {  // same rank, same bg, same bank
                                state.state->nextActivate1 = max(now() + unsigned(ceil(OFREQ_RATIO * 2)) +      
                                        CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                            } else {
                                state.state->nextActivate1 = max(now() + unsigned(ceil(OFREQ_RATIO * 2)) +
                                        CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                                state.state->nextActivate2 = max(now() + unsigned(ceil(OFREQ_RATIO * 2)) +
                                        CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                            }
                        } 
                    }
                }
            } else {
                bankStates[bus_packet.bankIndex].state->nextActivate2 = max(now() + unsigned(ceil(OFREQ_RATIO)) +
                        CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bus_packet.bankIndex].state->nextActivate2);
            }
            break;
        }
        case ACTIVATE2_CMD:{         //todo: revisr for e-mode
            for (auto &state : bankStates) {
                unsigned state_channel = (state.bankIndex % NUM_BANKS) / sc_bank_num;
                if (rank == state.rank) { // same rank
                    if (sub_channel == state_channel) {   //same rank. same subchannel
                        if (sid == state.sid) { // same rank, same sid
                            if (state.group == group) { // same rank, same sid, same bg
                                if (state.bank == bank) { // same rank, same sid, same bg, same bank
                                    state.state->currentBankState = RowActive;
                                    state.state->openRowAddress = bus_packet.row;
                                    if (IS_GD2) {
                                        state.state->nextPrecharge = max(now() + tRAS + 4 +
                                                CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPrecharge);
                                    } else {
                                        state.state->nextPrecharge = max(now() + tRAS +
                                                CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPrecharge);
                                    }
                                    if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2) {
                                        state.state->nextActivate1 = max(now() + tRAS + trp_pb - unsigned(ceil(OFREQ_RATIO * 2))
                                                + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                                    } else {
                                        state.state->nextActivate1 = max(now() + tRAS + trp_pb - unsigned(ceil(OFREQ_RATIO)) +
                                                CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                                    }
                                    if (!DERATING_EN) {
                                    state.state->nextActivate2 = max(now() + tRAS + trp_pb +
                                            CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                                    } else {
                                    state.state->nextActivate2 = max(now() + tRAS + trp_pb + unsigned(ceil(3.75 / tDFI)) +
                                            CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                                    }
                                    state.state->nextRead = max(now() + tRCD +
                                            CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextRead);
                                    state.state->nextReadAp = max(now() + tRCD +
                                            CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextReadAp);
                                    state.state->nextWrite = max(now() + tRCD_WR +
                                            CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextWrite);
                                    state.state->nextWriteAp = max(now() + tRCD_WR +
                                            CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextWriteAp);
                                    state.state->nextWriteRmw = max(now() + tRCD_WR +
                                            CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextWriteRmw);
                                    state.state->nextWriteApRmw = max(now() + tRCD_WR +
                                            CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextWriteApRmw);
                                    state.state->nextWriteMask = max(now() + tRCD +
                                            CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextWriteMask);
                                    state.state->nextWriteMaskAp = max(now() + tRCD +
                                            CalcCmdCycle(cmd_cycle, rw_cycle), state.state->nextWriteMaskAp);
                                } else { // same rank, same sid, same bg, diff bank
                                    if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2) {
                                        state.state->nextActivate1 = max(now() + tRRD_L - unsigned(ceil(OFREQ_RATIO * 2)) +
                                                CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                                    } else {
                                        state.state->nextActivate1 = max(now() + tRRD_L - unsigned(ceil(OFREQ_RATIO)) +
                                                CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                                    }
                                    state.state->nextActivate2 = max(now() + tRRD_L +
                                            CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                                    state.state->nextPerBankRefresh = max(now() + tRRD_L +
                                            CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPerBankRefresh);
                                    state.state->nextAllBankRefresh = max(now() + tRRD_L +
                                            CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextAllBankRefresh);
                                }
                            } else { // same rank, same sid, diff bg
                                if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2) {
                                    state.state->nextActivate1 = max(now() + tRRD_S - unsigned(ceil(OFREQ_RATIO * 2)) +
                                            CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                                } else {
                                    state.state->nextActivate1 = max(now() + tRRD_S - unsigned(ceil(OFREQ_RATIO)) +
                                            CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                                }
                                state.state->nextActivate2 = max(now() + tRRD_S +
                                        CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                                state.state->nextPerBankRefresh = max(now() + tRRD_S +
                                        CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPerBankRefresh);
                                state.state->nextAllBankRefresh = max(now() + tRRD_S +
                                        CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextAllBankRefresh);
                            }
                        } else { // same rank, diff sid
                            state.state->nextActivate1 = max(now() + tRRD_L - unsigned(ceil(OFREQ_RATIO)) +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                            state.state->nextActivate2 = max(now() + tRRD_L +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                            state.state->nextPerBankRefresh = max(now() + tRRD_L +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPerBankRefresh);
                            state.state->nextAllBankRefresh = max(now() + tRRD_L +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextAllBankRefresh);
                        }
                    } 
                }
                funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
            }
            break;
        }
        case PRECHARGE_SB_CMD:{
            for (size_t ba = 0; ba < NUM_BANKS; ba ++) {
                uint32_t bank_tmp = rank * NUM_BANKS + ba;
                unsigned bg = ba * NUM_GROUPS / NUM_BANKS;
                unsigned tppd = (bg == group) ? tPPD_L : tPPD;
                bankStates[bank_tmp].state->nextPrecharge = max(now() + tppd + CalcCmdCycle(cmd_cycle, cmd_cycle),
                        bankStates[bank_tmp].state->nextPrecharge);
            }
            if (IS_DDR5) {
                for (size_t i = 0; i < pbr_bg_num; i ++) {
                    uint32_t bankIndex = i * pbr_bank_num + bus_packet.bankIndex;
                    bankStates[bankIndex].state->currentBankState = Precharging;
                    bankStates[bankIndex].state->stateChangeEn = true;
                    bankStates[bankIndex].state->stateChangeCountdown = tRPab + CalcCmdCycle(cmd_cycle, 1);
                    bankStates[bankIndex].state->nextPerBankRefresh = max(now() + tRPab +
                            CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                    bankStates[bankIndex].state->nextAllBankRefresh = max(now() + tRPab +
                            CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextAllBankRefresh);
                    bankStates[bankIndex].state->nextActivate2 = max(now() + tRPab +
                            CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                }
            }
            if (DMC_V596)
                funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
            else // rtl not use tCMDPD
                funcState[rank].nextPde = max(now() + trp_pb + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
            break;
        }
        case PRECHARGE_PB_CMD:{        //todo: revise for e-mode
            for (size_t ba = 0; ba < NUM_BANKS; ba ++) {
                uint32_t bank_tmp = rank * NUM_BANKS + ba;
                unsigned bg = ba * NUM_GROUPS / NUM_BANKS;
                unsigned tppd = (bg == group) ? tPPD_L : tPPD;
                unsigned state_channel = (bank_tmp % NUM_BANKS) / sc_bank_num;
                if (sub_channel == state_channel) {    // same rank, same subchannel
                    bankStates[bank_tmp].state->nextPrecharge = max(now() + tppd + CalcCmdCycle(cmd_cycle, cmd_cycle),
                            bankStates[bank_tmp].state->nextPrecharge);
                }
            }
            bankStates[bus_packet.bankIndex].state->currentBankState = Precharging;
            if (bus_packet.cmd_source == 1 || bus_packet.cmd_source == 2) {
//                trp_pb += 15; // For pagetimeout and func precharge command
                if (DMC_RATE <= 3200) {
                    trp_pb += 7;   // For pagetimeout and func precharge command, low frequency
                } else {
                    trp_pb += 15;    // For pagetimeout and func precharge command 
                }
            }
            bankStates[bus_packet.bankIndex].state->stateChangeEn = true;
            bankStates[bus_packet.bankIndex].state->stateChangeCountdown = trp_pb + CalcCmdCycle(cmd_cycle, 1);
            bankStates[bus_packet.bankIndex].state->nextPerBankRefresh = max(now() + trp_pb +
                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bus_packet.bankIndex].state->nextPerBankRefresh);
            bankStates[bus_packet.bankIndex].state->nextAllBankRefresh = max(now() + trp_pb +
                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bus_packet.bankIndex].state->nextAllBankRefresh);
            if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2) {
                bankStates[bus_packet.bankIndex].state->nextActivate1 = max(now() + trp_pb -
                        unsigned(ceil(OFREQ_RATIO * 2)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                        bankStates[bus_packet.bankIndex].state->nextActivate1);
            } else {
                bankStates[bus_packet.bankIndex].state->nextActivate1 = max(now() + trp_pb -
                        unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                        bankStates[bus_packet.bankIndex].state->nextActivate1);
            }
            bankStates[bus_packet.bankIndex].state->nextActivate2 = max(now() + trp_pb +
                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bus_packet.bankIndex].state->nextActivate2);
            if (DMC_V596) {
                if (IS_LP6) {    // tCMDPD + tnACU
                    funcState[rank].nextPde = max(now() + tCMDPD + tnACU + CalcCmdCycle(cmd_cycle, cmd_cycle), funcState[rank].nextPde);
                } else {
                    funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, cmd_cycle), funcState[rank].nextPde);
                }
            } else { // rtl not use tCMDPD, trp_pb of lp6 include nACU
                funcState[rank].nextPde = max(now() + trp_pb + CalcCmdCycle(cmd_cycle, cmd_cycle), funcState[rank].nextPde);
            }
            break;
        }
        case PRECHARGE_AB_CMD:{       //todo: revise for e-mode
            for (size_t i = 0; i < NUM_BANKS; i ++) {
                uint32_t bankIndex = rank * NUM_BANKS + i;
                unsigned state_channel = (bankIndex % NUM_BANKS) / sc_bank_num;
                if (sub_channel == state_channel) {       //same rank, same subchannel
                    bankStates[bankIndex].state->currentBankState = Precharging;
                    bankStates[bankIndex].state->stateChangeEn = true;
                    bankStates[bankIndex].state->stateChangeCountdown = tRPab + CalcCmdCycle(cmd_cycle, 1);
                    bankStates[bankIndex].state->nextPerBankRefresh = max(now() + tRPab +
                            CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                    bankStates[bankIndex].state->nextAllBankRefresh = max(now() + tRPab +
                            CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextAllBankRefresh);
                    if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2) {
                        bankStates[bankIndex].state->nextActivate1 = max(now() + tRPab - unsigned(ceil(OFREQ_RATIO * 2)) +
                                CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                    } else {
                        bankStates[bankIndex].state->nextActivate1 = max(now() + tRPab - unsigned(ceil(OFREQ_RATIO)) +
                                CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                    }
                    bankStates[bankIndex].state->nextActivate2 = max(now() + tRPab +
                            CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                }
            }
            if (DMC_V596) {
                if (IS_LP6) {     // tCMDPD + tnACU
                    funcState[rank].nextPde = max(now() + tCMDPD + tnACU + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
                } else {
                    funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
                }
            } else { // rtl not use tCMDPD, tRPab of lp6 include nACU
                funcState[rank].nextPde = max(now() + tRPab + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
            }
            break;
        }
        case REFRESH_CMD:{       //todo: revise for e-mode
            for (size_t i = 0; i < NUM_BANKS; i ++) {
                uint32_t bankIndex = rank * NUM_BANKS + i;
                unsigned state_channel = (bankIndex % NUM_BANKS) / sc_bank_num;
                if (sub_channel == state_channel) {        // same rank, same channel
                    if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2) {
                        bankStates[bankIndex].state->nextActivate1 = max(now() + trfcab - unsigned(ceil(OFREQ_RATIO * 2)) +
                                CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                    } else {
                        bankStates[bankIndex].state->nextActivate1 = max(now() + trfcab - unsigned(ceil(OFREQ_RATIO)) +
                                CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate1);
                    }
                    bankStates[bankIndex].state->nextActivate2 = max(now() + trfcab +
                            CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                    bankStates[bankIndex].state->nextPerBankRefresh = max(now() + trfcab +
                            CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                    bankStates[bankIndex].state->nextAllBankRefresh = max(now() + trfcab +
                            CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextAllBankRefresh);
                    bankStates[bankIndex].state->currentBankState = Refreshing;
                    bankStates[bankIndex].state->stateChangeEn = true;
                    bankStates[bankIndex].state->stateChangeCountdown = trfcab + CalcCmdCycle(cmd_cycle, 1);
                }
            }
            funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
            break;
        }
        case PER_BANK_REFRESH_CMD:{          //todo:revise for e-mode
            if (ENH_PBR_EN) {                          
                for (size_t i = 0; i < NUM_BANKS; i ++) {
                    uint32_t bankIndex = rank * NUM_BANKS + i;
                    if (bankIndex == (bus_packet.fst_bankIndex) || bankIndex == (bus_packet.lst_bankIndex)) {    // same rank, same bank group, same bank
//                        DEBUG(now()<<" fresh timing, bank="<<bankIndex<<" fst_bankIndex="<<bus_packet.fst_bankIndex<<" lst_bankIndex="<<bus_packet.lst_bankIndex);
                        // bankIndex check
                        if (bus_packet.fst_bankIndex == bus_packet.lst_bankIndex) {
                            ERROR(setw(10)<<now()<<" Not allowed same bank in a bank pair, task="<<bus_packet.task<<" bank="<<bankIndex
                                    <<" fst_bankIndex"<<bus_packet.fst_bankIndex<<" lst_bankIndex"<<bus_packet.lst_bankIndex);
                            assert(0);
                        } else if (bus_packet.fst_bankIndex > bus_packet.lst_bankIndex) {
                            if (((bus_packet.fst_bankIndex-bus_packet.lst_bankIndex)%pbr_sb_num) != 0) {
                                ERROR(setw(10)<<now()<<" Non-4x Diff between banks in a bank pair, task="<<bus_packet.task<<" bank="<<bankIndex
                                        <<" fst_bankIndex"<<bus_packet.fst_bankIndex<<" lst_bankIndex"<<bus_packet.lst_bankIndex);
                                assert(0);
                            }
                        } else if (bus_packet.lst_bankIndex > bus_packet.fst_bankIndex) {
                            if (((bus_packet.lst_bankIndex-bus_packet.fst_bankIndex)%pbr_sb_num) != 0) {
                                ERROR(setw(10)<<now()<<" Non-4x Diff between banks in a bank pair, task="<<bus_packet.task<<" bank="<<bankIndex
                                        <<" fst_bankIndex"<<bus_packet.fst_bankIndex<<" lst_bankIndex"<<bus_packet.lst_bankIndex);
                                assert(0);
                            }
                        }

                        bankStates[bankIndex].state->currentBankState = Refreshing;
                        bankStates[bankIndex].state->stateChangeEn = true;
                        bankStates[bankIndex].state->stateChangeCountdown = trfcpb + CalcCmdCycle(cmd_cycle, 1);
                        if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2) {
                            bankStates[bankIndex].state->nextActivate1 = max(now() + trfcpb -
                                    unsigned(ceil(OFREQ_RATIO * 2)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                                    bankStates[bankIndex].state->nextActivate1);
                        } else {
                            bankStates[bankIndex].state->nextActivate1 = max(now() + trfcpb -
                                    unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                                    bankStates[bankIndex].state->nextActivate1);
                        }
                        bankStates[bankIndex].state->nextActivate2 = max(now() + trfcpb +
                                CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                        bankStates[bankIndex].state->nextPerBankRefresh = max(now() + trfcpb +
                                CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                        bankStates[bankIndex].state->nextAllBankRefresh = max(now() + trfcpb +
                                CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextAllBankRefresh);
                    } else {
                        if (IS_LP6 && (refresh_cnt_pb[bus_packet.rank][sub_channel] == (NUM_BANKS/sc_num -2))) {           // enhanced dbr: refresh_cnt_pb = 2x PER_BANK_REFRESH_CMD
                            bankStates[bankIndex].state->nextPerBankRefresh = max(now() + tPBR2PBR_L +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                            //if(DEBUG_BUS){
                            //    PRINT(setw(10)<<now()<<" -- LP6 use tpbrtpbr_L here, refreshed bank pairs="<<refresh_cnt_pb[bus_packet.rank]<<endl);
                            //}

                        } else {
                            bankStates[bankIndex].state->nextPerBankRefresh = max(now() + tPBR2PBR +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                        }
                        //bankStates[bankIndex].state->nextPerBankRefresh = max(now() + trfcpb +
                        //        CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);        
                        bankStates[bankIndex].state->nextAllBankRefresh = max(now() + tPBR2PBR +
                                CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextAllBankRefresh);
                        if (IS_LP5 || IS_LP6 || IS_GD2) {
                            if (DMC_V580 || IS_LP6) {
                                bankStates[bankIndex].state->nextActivate1 = max(now() + tPBR2ACT -
                                        unsigned(ceil(OFREQ_RATIO * 2)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                                        bankStates[bankIndex].state->nextActivate1);
                                bankStates[bankIndex].state->nextActivate2 = max(now() + tPBR2ACT +
                                        CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                            } else {
                                bankStates[bankIndex].state->nextActivate1 = max(now() + tPBR2ACT -
                                        unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                                        bankStates[bankIndex].state->nextActivate1);
                            }
                        } else if (IS_LP4) {
                            bankStates[bankIndex].state->nextActivate1 = max(now() + tRRD_S -
                                    unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                                    bankStates[bankIndex].state->nextActivate1);
                            bankStates[bankIndex].state->nextActivate2 = max(now() + tRRD_S +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                        } else if (IS_LP4 || IS_HBM2E || IS_HBM3) {
                            bankStates[bankIndex].state->nextActivate1 = max(now() + tPBR2ACT -
                                    unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                                    bankStates[bankIndex].state->nextActivate1);
                            bankStates[bankIndex].state->nextActivate2 = max(now() + tPBR2ACT +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                        } else if (IS_DDR5) {
                            bankStates[bankIndex].state->nextActivate2 = max(now() + tPBR2ACT +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                        } else if (IS_GD1 || IS_G3D) {
                            bankStates[bankIndex].state->nextActivate2 = max(now() + tRRD_S +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                        }
                    }
                }
                if (DMC_V596)
                    funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
                else // rtl not use tCMDPD
                    funcState[rank].nextPde = max(now() + trfcpb + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
                break;
            } else {         
                for (size_t i = 0; i < pbr_bank_num; i ++) {
                    for (size_t j = 0; j < pbr_bg_num; j++) {
                        uint32_t bankIndex = rank * NUM_BANKS + i + j * pbr_bank_num + bank_start;
                        if (bankIndex == (bus_packet.bankIndex + j * pbr_bank_num)) {     // same bank with one of the bank pair in same subchannel
                            bankStates[bankIndex].state->currentBankState = Refreshing;
                            bankStates[bankIndex].state->stateChangeEn = true;
                            bankStates[bankIndex].state->stateChangeCountdown = trfcpb + CalcCmdCycle(cmd_cycle, 1);
                            if ((DMC_V580 && IS_LP5) || IS_LP6 || IS_GD2) {
                                bankStates[bankIndex].state->nextActivate1 = max(now() + trfcpb -
                                        unsigned(ceil(OFREQ_RATIO * 2)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                                        bankStates[bankIndex].state->nextActivate1);
                            } else {
                                bankStates[bankIndex].state->nextActivate1 = max(now() + trfcpb -
                                        unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                                        bankStates[bankIndex].state->nextActivate1);
                            }
                            bankStates[bankIndex].state->nextActivate2 = max(now() + trfcpb +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                            bankStates[bankIndex].state->nextPerBankRefresh = max(now() + trfcpb +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                            bankStates[bankIndex].state->nextAllBankRefresh = max(now() + trfcpb +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextAllBankRefresh);
                        } else {     //diff bank from one of bank pair in same subchannel
                            if (IS_LP6 && (refresh_cnt_pb[bus_packet.rank][sub_channel] == (pbr_bank_num -1))) {
                                bankStates[bankIndex].state->nextPerBankRefresh = max(now() + tPBR2PBR_L +
                                        CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                                //if(DEBUG_BUS){
                                //    PRINT(setw(10)<<now()<<" -- LP6 use tpbrtpbr_L here, refreshed bank pairs="<<refresh_cnt_pb[bus_packet.rank]<<endl);
                                //}

                            } else {
                                bankStates[bankIndex].state->nextPerBankRefresh = max(now() + tPBR2PBR +
                                        CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);
                            }
                            //bankStates[bankIndex].state->nextPerBankRefresh = max(now() + trfcpb +
                            //        CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextPerBankRefresh);        
                            bankStates[bankIndex].state->nextAllBankRefresh = max(now() + tPBR2PBR +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextAllBankRefresh);
                            if (IS_LP5 || IS_LP6 || IS_GD2) {
                                if (DMC_V580 || IS_LP6) {
                                    bankStates[bankIndex].state->nextActivate1 = max(now() + tPBR2ACT -
                                            unsigned(ceil(OFREQ_RATIO * 2)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                                            bankStates[bankIndex].state->nextActivate1);
                                    bankStates[bankIndex].state->nextActivate2 = max(now() + tPBR2ACT +
                                            CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                                } else {
                                    bankStates[bankIndex].state->nextActivate1 = max(now() + tPBR2ACT -
                                            unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                                            bankStates[bankIndex].state->nextActivate1);
                                }
                            } else if (IS_LP4) {
                                bankStates[bankIndex].state->nextActivate1 = max(now() + tRRD_S -
                                        unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                                        bankStates[bankIndex].state->nextActivate1);
                                bankStates[bankIndex].state->nextActivate2 = max(now() + tRRD_S +
                                        CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                            } else if (IS_LP4 || IS_HBM2E || IS_HBM3) {
                                bankStates[bankIndex].state->nextActivate1 = max(now() + tPBR2ACT -
                                        unsigned(ceil(OFREQ_RATIO)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                                        bankStates[bankIndex].state->nextActivate1);
                                bankStates[bankIndex].state->nextActivate2 = max(now() + tPBR2ACT +
                                        CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                            } else if (IS_DDR5) {
                                bankStates[bankIndex].state->nextActivate2 = max(now() + tPBR2ACT +
                                        CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                            } else if (IS_GD1 || IS_G3D) {
                                bankStates[bankIndex].state->nextActivate2 = max(now() + tRRD_S +
                                        CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bankIndex].state->nextActivate2);
                            }
                        }
                    }
                }
                if (DMC_V596)
                    funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
                else // rtl not use tCMDPD
                    funcState[rank].nextPde = max(now() + trfcpb + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
                break;
            }
        }
        case ACTIVATE1_DST_CMD:{
            bankStates[bus_packet.bankIndex].state->currentBankState = Refreshing;
            for (auto &state : bankStates) {
                if (rank != state.rank) continue;
                state.state->nextActivate2 = max(now() + unsigned(ceil(OFREQ_RATIO * 2)) +
                        CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
            }
            break;
        }
        case ACTIVATE2_DST_CMD:{
            for (auto &state : bankStates) {
                if (rank == state.rank) {
                    if (state.group == group) {
                        if (state.bank == bank) {
                            state.state->nextPrecharge = max(now() + tRAS +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPrecharge);
                            state.state->nextActivate1 = max(now() + tRAS + trp_pb - unsigned(ceil(OFREQ_RATIO * 2))
                                    + CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                            state.state->nextActivate2 = max(now() + tRAS + trp_pb +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                        } else {
                            state.state->nextActivate1 = max(now() + tRRD_L - unsigned(ceil(OFREQ_RATIO * 2)) +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                            state.state->nextActivate2 = max(now() + tRRD_L +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                            state.state->nextPerBankRefresh = max(now() + tRRD_L +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPerBankRefresh);
                            state.state->nextAllBankRefresh = max(now() + tRRD_L +
                                    CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextAllBankRefresh);
                        }
                    } else {
                        state.state->nextActivate1 = max(now() + tRRD_S - unsigned(ceil(OFREQ_RATIO * 2)) +
                                CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate1);
                        state.state->nextActivate2 = max(now() + tRRD_S +
                                CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextActivate2);
                        state.state->nextPerBankRefresh = max(now() + tRRD_S +
                                CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextPerBankRefresh);
                        state.state->nextAllBankRefresh = max(now() + tRRD_S +
                                CalcCmdCycle(cmd_cycle, cmd_cycle), state.state->nextAllBankRefresh);
                    }
                }
                if (DMC_V596)
                    funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, 1), funcState[rank].nextPde);
            }
            break;
        }
        case PRECHARGE_PB_DST_CMD:{
            for (size_t ba = 0; ba < NUM_BANKS; ba ++) {
                uint32_t bank_tmp = rank * NUM_BANKS + ba;
                unsigned bg = ba * NUM_GROUPS / NUM_BANKS;
                unsigned tppd = (bg == group) ? tPPD_L : tPPD;
                bankStates[bank_tmp].state->nextPrecharge = max(now() + tppd + CalcCmdCycle(cmd_cycle, cmd_cycle),
                        bankStates[bank_tmp].state->nextPrecharge);
            }
            bankStates[bus_packet.bankIndex].state->currentBankState = Precharging;
            bankStates[bus_packet.bankIndex].state->stateChangeEn = true;
            bankStates[bus_packet.bankIndex].state->stateChangeCountdown = trp_pb + CalcCmdCycle(cmd_cycle, 1);
            bankStates[bus_packet.bankIndex].state->nextPerBankRefresh = max(now() + trp_pb +
                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bus_packet.bankIndex].state->nextPerBankRefresh);
            bankStates[bus_packet.bankIndex].state->nextAllBankRefresh = max(now() + trp_pb +
                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bus_packet.bankIndex].state->nextAllBankRefresh);
            bankStates[bus_packet.bankIndex].state->nextActivate1 = max(now() + trp_pb -
                    unsigned(ceil(OFREQ_RATIO * 2)) + CalcCmdCycle(cmd_cycle, cmd_cycle),
                    bankStates[bus_packet.bankIndex].state->nextActivate1);
            bankStates[bus_packet.bankIndex].state->nextActivate2 = max(now() + trp_pb +
                    CalcCmdCycle(cmd_cycle, cmd_cycle), bankStates[bus_packet.bankIndex].state->nextActivate2);
            if (DMC_V596)
                funcState[rank].nextPde = max(now() + tCMDPD + CalcCmdCycle(cmd_cycle, cmd_cycle), funcState[rank].nextPde);
            break;
        }
        default : break;
    }
}

/***************************************************************************************************
descriptor: power event statistics
****************************************************************************************************/
void MemoryController::power_event_stat() {
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        if (RankState[rank].lp_state == PD) continue;
        if (RankState[rank].lp_state == PDLP) continue;
        if (RankState[rank].lp_state == ASREF) continue;
        if (RankState[rank].lp_state == SRPD) continue;
        if (RankState[rank].lp_state == SRPDLP) continue;
        que_cmd_time[GetDmcQsize()] ++;
        break;
    }
}

/***************************************************************************************************
descriptor: update wdata pipeline
****************************************************************************************************/
void MemoryController::update_wdata() {
    if (WdataPipe.empty()) return;

    for (size_t i = 0; i < WdataPipe.size(); i++) {
        uint64_t task = WdataPipe[i].task;
        uint64_t delay = WdataPipe[i].delay;
        if (now() < delay) {
            continue;
        }

        for (auto &t : transactionQueue) {
            if (t->transactionType == DATA_READ) continue;
            if (t->data_ready_cnt > t->burst_length) continue;
            if (task != t->task) {
                continue;}
            t->data_ready_cnt ++;
            WdataPipe.erase(WdataPipe.begin() + i);
            if (DEBUG_BUS) {
                 PRINTN(setw(10)<<now()<<" -- MATCH :: data ready cnt="<<t->data_ready_cnt<<", burst_length="
                         <<t->burst_length<<", data_size="<<t->data_size<<", task="<<t->task<<endl);
            }
            return;
        }
    }
}

/***************************************************************************************************
descriptor: grant fifo backpress check
****************************************************************************************************/
void MemoryController::update_grt_fifo() {
    if (!IS_LP6 || (IS_LP6 && EM_ENABLE)) return;

    //grt fifo bp
    grt_fifo_wcmd_cnt = 0;
    grt_fifo_bp =false;
    for (auto &t : transactionQueue) {
        if (t->data_ready_cnt > t->burst_length) continue;
        if (t->transactionType == DATA_WRITE) {
            unsigned wdata_pipe_cnt = 0;
            for (auto &wpipe : WdataPipe) {
                if (wpipe.task == t->task) {
                    wdata_pipe_cnt ++;
                }
            }
            if ((wdata_pipe_cnt + t->data_ready_cnt) <= t->burst_length) { 
                grt_fifo_wcmd_cnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- UNFULL WDATA CMD IN QUE :: [W]B["<<t->burst_length<<"]QOS["<<t->qos
                            <<"]MID["<<t->mid<<"] addr=0x"<<hex<<t->address<<dec<<" task="<<t->task
                            <<" data_ready_cnt="<<t->data_ready_cnt<<" rank="<<t->rank<<" group="<<t->group<<" bank="<<t->bankIndex<<" row="
                            <<t->row<<" col="<<t->col<<" data_size="<<t->data_size<<" Q="<<GetDmcQsize()
                            <<" QR="<<que_read_cnt<<" QW="<<que_write_cnt<<" timeAdded="<<t->timeAdded
                            <<" timeout_th="<<t->timeout_th<<" mask_wcmd="<<t->mask_wcmd<<endl);
                }
            }
        }
    }
    if ((grt_fifo_wcmd_cnt >= GRT_FIFO_DEPTH) && (sub_cha==1)) {  // valid for sub channel 1 of lp6
//    if ((grt_fifo_wcmd_cnt >= GRT_FIFO_DEPTH)) {  // valid for sub channel 1 of lp6
        grt_fifo_bp = true;
        if (DEBUG_BUS) {    
            PRINTN(setw(10)<<now()<<" -- GRT FIFO BP :: wcmd_nt in GRT_FIFO="<<grt_fifo_wcmd_cnt<<endl);
        }
    }
}
/***************************************************************************************************
descriptor: update pd status, include pd enter and pd exit
****************************************************************************************************/
void MemoryController::update_lp_state() {
    bool has_rank_blocking = false;
    unsigned blocking_rank = 0;
#if 0
    DEBUGN(now()<<" -- phylp_state="<<PhyLpState.phylp_state<<", phylp_cnt="<<PhyLpState.lp_cnt);
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        DEBUGN(" | rank="<<rank<<", rank_cnt="<<+rank_cnt[rank]<<", state="<<RankState[rank].lp_state
            <<", pd_cnt="<<RankState[rank].pd_cnt<<", asref_cnt="<<RankState[rank].asref_cnt
            <<", state_cnt="<<RankState[rank].state_cnt<<", wakeup="<<funcState[rank].wakeup
            <<", nextPde="<<funcState[rank].nextPde);
        if (rank == (NUM_RANKS - 1)) {
            DEBUGN(endl);
        }
    }
#endif
    // PHY LP logic
    bool phylp_timing_met = false;
    if (PhyLpState.lp_cnt > 0) {
        PhyLpState.lp_cnt --;
        if (PhyLpState.lp_cnt == 0) phylp_timing_met = true;
    }

    if (GRP_RANK_EN && rk_grp_state != NO_RGRP) {
        unsigned rank_grp = (rk_grp_state >> 1) % NUM_RANKS;
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            bool rank_has_timeout = false;
            for (size_t i = 0; i < NUM_BANKS; i ++) {
                if (bankStates[rank * NUM_BANKS + i].has_timeout) {
                    rank_has_timeout = true;
                    break;
                }
            }
            if (rank == rank_grp || rank_has_timeout) rank_has_cmd[rank] = true;
            else rank_has_cmd[rank] = false;
        }
    } else {
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            if (deqCmdWakeupLp[rank].front() > 0) rank_has_cmd[rank] = true;
            else rank_has_cmd[rank] = false;
        }
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            if (rank_pre_act_cnt[rank] > 0) rank_has_pre_act_cmd[rank] = true;
            else rank_has_pre_act_cmd[rank] = false;
        }
    }

    //for (size_t i = 0; i < NUM_RANKS; i ++) {
    //        DEBUG(now()<<" rank_has_pre_act_cmd="<<rank_has_pre_act_cmd[i]<<" rank="<<i);
    //        DEBUG(now()<<" rank_pre_act_cnt="<<rank_pre_act_cnt[i]<<" rank="<<i);
    //}

    bool is_allrank_phylp = true;
    bool phylp_wakeup = false;
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        if (RankState[rank].lp_state != PDLP && RankState[rank].lp_state != SRPDLP) {
            is_allrank_phylp = false;
            break;
        }
    }
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        if (rank_has_cmd[rank] || rank_has_pre_act_cmd[rank] || fast_wakeup[rank] || funcState[rank].wakeup) {
            phylp_wakeup = true;
            break;
        }
    }
    switch (PhyLpState.phylp_state) {
        case PHYLP_IDLE: {
            if (is_allrank_phylp) {
                PhyLpState.lp_cnt = tPHYLPE;
                PhyLpState.phylp_state = PHYLPE;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PHY_LP :: cur_state=PHYLP_IDLE, next_state=PHYLPE"<<endl);
                }
            }
            phy_notlp_cnt ++;
            break;
        }
        case PHYLPE: {
            if (phylp_wakeup) {
                PhyLpState.lp_cnt = tPHYLPX;
                PhyLpState.phylp_state = PHYLPX;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PHY_LP :: cur_state=PHYLPE, next_state=PHYLPX"<<endl);
                }
            } else if (phylp_timing_met) {
                PhyLpState.phylp_state = PHYLP;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PHY_LP :: cur_state=PHYLPE, next_state=PHYLP"<<endl);
                }
            }
            phy_notlp_cnt ++;
            break;
        }
        case PHYLP: {
            if (phylp_wakeup) {
                PhyLpState.lp_cnt = tPHYLPX;
                PhyLpState.phylp_state = PHYLPX;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PHY_LP :: cur_state=PHYLP, next_state=PHYLPX"<<endl);
                }
            }
            phy_lp_cnt ++;
            break;
        }
        case PHYLPX: {
            if (phylp_timing_met) {
                PhyLpState.phylp_state = PHYLP_IDLE;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PHY_LP :: cur_state=PHYLPX, next_state=PHYLP_IDLE"<<endl);
                }
            }
            phy_notlp_cnt ++;
            break;
        }
        default: break;
    }

    // Func PD & Asref logic
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        if (IS_LP5 || IS_LP6 || IS_GD2) {
            if (RankState[rank].lp_state == PDE || RankState[rank].lp_state == PDX ||
                    RankState[rank].lp_state == ASREFE || RankState[rank].lp_state == ASREFX ||
                    RankState[rank].lp_state == SRPDE || RankState[rank].lp_state == SRPDX) {
                has_rank_blocking = true;
                blocking_rank = rank;
                break;
            }
        }
    }
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        if (has_rank_blocking && blocking_rank != rank) continue;
        if (RankState[rank].pd_cnt > 0) RankState[rank].pd_cnt --;
        if (RankState[rank].asref_cnt > 0) RankState[rank].asref_cnt --;
        if (RankState[rank].state_cnt > 0) RankState[rank].state_cnt --;
    }

    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        bool combo_emode_unable = EM_ENABLE && EM_MODE==2 && rank==1;
        bool pd_timing_met = false;
        bool asref_timing_met = false;
        bool state_timing_met = false;
        bool has_fastwakeup = fast_wakeup_cnt[rank] > 0;
//        if (RankState[rank].pd_cnt == 0 && now() >= funcState[rank].nextPde) pd_timing_met = PD_ENABLE;
        if (RankState[rank].pd_cnt == 0 && now() >= funcState[rank].nextPde) pd_timing_met = PD_ENABLE && even_cycle;  // every other command, even;
        if (RankState[rank].asref_cnt == 0) {
            bool all_bank_idle = true;
            for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
                unsigned sub_channel = bank / sc_bank_num;
                if (combo_emode_unable && sub_channel==1) continue;        //rank1, sc1 forbidden under combo e-mode
                if (bankStates[rank*NUM_BANKS+bank].state->currentBankState == Idle) continue;
                all_bank_idle = false;
                break;
            }
//            if (all_bank_idle) asref_timing_met = ASREF_ENABLE;
            if (all_bank_idle) asref_timing_met = ASREF_ENABLE && even_cycle;   //every other command, even;
        }
        if (RankState[rank].state_cnt == 0) {
//            state_timing_met = PD_ENABLE;
            state_timing_met = PD_ENABLE && even_cycle;   //every other command, even;
        }
        if (DMC_V590 && ASREF_ADAPT_EN) {
            if (now() % ASREF_ADAPT_WIN == 0 && now() != 0) {
                if (rank_cnt_asref[rank] >= MAP_CONFIG["ASREF_ADAPT_LEVEL"][2]) {
                    ASREF_PRD = MAP_CONFIG["ASREF_ADAPT_PRD"][3];
                } else if (rank_cnt_asref[rank] >= MAP_CONFIG["ASREF_ADAPT_LEVEL"][1]) {
                    ASREF_PRD = MAP_CONFIG["ASREF_ADAPT_PRD"][2];
                } else if (rank_cnt_asref[rank] >= MAP_CONFIG["ASREF_ADAPT_LEVEL"][0]) {
                    ASREF_PRD = MAP_CONFIG["ASREF_ADAPT_PRD"][1];
                } else {
                    ASREF_PRD = MAP_CONFIG["ASREF_ADAPT_PRD"][0];
                }
                rank_cnt_asref[rank] = 0;
            }
        }
        // added for e-mode
        unsigned refresh_all = true;
        for (size_t i = 0; i < sc_num; i++) {
            if (combo_emode_unable && i==1) continue;    //rank1, sc1 forbidden under combo e-mode
            if (refreshALL[rank][i].refresh_cnt !=0){
                refresh_all = false;
                break;
            } 
        }
        switch (RankState[rank].lp_state) {
            case IDLE: {
                if (rank_has_cmd[rank] || fast_wakeup[rank] || has_fastwakeup) {
                    if (PD_ENABLE) RankState[rank].pd_cnt = PD_PRD;
                    if (ASREF_ENABLE) RankState[rank].asref_cnt = ASREF_PRD;
                } else if (!DMC_V580 && funcState[rank].wakeup) {
                    if (PD_ENABLE) RankState[rank].pd_cnt = PD_PRD;
                } else if (asref_timing_met && ASREF_ENABLE && !funcState[rank].wakeup) {
                    RankState[rank].asref_cnt = tASREFE;
                    RankState[rank].lp_state = pd_timing_met ? SRPDE : ASREFE;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=IDLE, next_state=ASREFE"<<endl);
                    }
//                } else if (pd_timing_met && PD_ENABLE && !funcState[rank].wakeup && (refreshALL[rank].refresh_cnt==0)) {
                } else if (pd_timing_met && PD_ENABLE && !funcState[rank].wakeup && refresh_all) {
                    RankState[rank].pd_cnt = tPDE;
                    RankState[rank].lp_state = PDE;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=IDLE, next_state=PDE"<<endl);
                    }
                }
                WakeUpTime[rank] ++;
                break;
            }
            case PDE: {
                if (pd_timing_met) {
                    RankState[rank].pd_cnt = tPDLP;
                    RankState[rank].lp_state = PD;
                    RankState[rank].state_cnt = tCSPD;
                    phy p;
                    p.command.type = PD_CMD;
                    p.command.rank = rank;
                    p.command.task = 0xFFFFFFFFFFFFFF6;
                    p.delay = tCMD_PHY;
                    packet.push_back(p);
                    PdEnterCnt[rank] ++;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=PDE, next_state=PD"<<endl);
                    }
                }
                WakeUpTime[rank] ++;
                break;
            }
            case PD: {
                if ((rank_has_cmd[rank] || rank_has_pre_act_cmd[rank] || fast_wakeup[rank] || funcState[rank].wakeup) &&
                        state_timing_met) {
                    RankState[rank].pd_cnt = tXP;
                    RankState[rank].lp_state = PDX;
                    phy p;
                    p.command.type = PDX_CMD;
                    p.command.rank = rank;
                    p.command.task = 0xFFFFFFFFFFFFFF5;
                    p.delay = tCMD_PHY;
                    packet.push_back(p);
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=PD, next_state=PDX"<<endl);
                    }
                } else if (pd_timing_met) {
                    RankState[rank].lp_state = PDLP;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=PD, next_state=PDLP"<<endl);
                    }
                }
                PdTime[rank] ++;
                break;
            }
            case PDLP: {
                if ((rank_has_cmd[rank] || rank_has_pre_act_cmd[rank] || fast_wakeup[rank] || asref_timing_met ||
                        funcState[rank].wakeup) && state_timing_met) {
                    RankState[rank].pd_cnt = tXP;
                    RankState[rank].lp_state = PDX;
                    phy p;
                    p.command.type = PDX_CMD;
                    p.command.rank = rank;
                    p.command.task = 0xFFFFFFFFFFFFFF5;
                    p.delay = tCMD_PHY;
                    packet.push_back(p);
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=PDLP, next_state=PDX"<<endl);
                    }
                }
                PdTime[rank] ++;
                break;
            }
            case PDX: {
                if (pd_timing_met) {
                    if (!asref_timing_met && !funcState[rank].wakeup) {
                        RankState[rank].pd_cnt = PD_PRD;
                        if (ASREF_ENABLE) RankState[rank].asref_cnt = ASREF_PRD;
                    }
                    RankState[rank].lp_state = IDLE;
                    PdExitCnt[rank] ++;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=PDX, next_state=IDLE"<<endl);
                    }
                }
                WakeUpTime[rank] ++;
                break;
            }
            case ASREFE: {
                if (asref_timing_met) {
                    RankState[rank].lp_state = ASREF;
                    phy p;
                    p.command.type = ASREF_CMD;
                    p.command.rank = rank;
                    p.command.task = 0xFFFFFFFFFFFFFF3;
                    p.delay = tCMD_PHY;
                    packet.push_back(p);
                    AsrefEnterCnt[rank] ++;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=ASREFE, next_state=ASREF"<<endl);
                    }
                }
                WakeUpTime[rank] ++;
                break;
            }
            case ASREF: {
                if (rank_has_cmd[rank] || rank_has_pre_act_cmd[rank] || fast_wakeup[rank] || funcState[rank].wakeup) {
                    RankState[rank].asref_cnt = tXSR;
                    RankState[rank].lp_state = ASREFX;
                    phy p;
                    p.command.type = ASREFX_CMD;
                    p.command.rank = rank;
                    p.command.task = 0xFFFFFFFFFFFFFF2;
                    p.delay = tCMD_PHY;
                    packet.push_back(p);
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=ASREF, next_state=ASREFX"<<endl);
                    }
                } else if (pd_timing_met) {
                    if (DMC_V590) {
                        RankState[rank].pd_cnt = tPDLP;
                        RankState[rank].lp_state = SRPD;
                        RankState[rank].state_cnt = tCSPD;
                        phy p;
                        p.command.type = SRPD_CMD;
                        p.command.rank = rank;
                        p.command.task = 0xFFFFFFFFFFFFFF0;
                        p.delay = tCMD_PHY;
                        packet.push_back(p);
                        PdEnterCnt[rank] ++;
                        SrpdEnterCnt[rank] ++;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=SRPDE, next_state=SRPD"<<endl);
                        }
                    } else {
                        RankState[rank].pd_cnt = tPDE;
                        RankState[rank].lp_state = SRPDE;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=ASREF, next_state=SRPDE"<<endl);
                        }
                    }
                }
                AsrefTime[rank] ++;
                break;
            }
            case ASREFX: {
                if (asref_timing_met) {
                    if (!funcState[rank].wakeup) {
                        RankState[rank].pd_cnt = PD_PRD;
                        RankState[rank].asref_cnt = ASREF_PRD;
                    }
                    RankState[rank].lp_state = IDLE;
                    AsrefExitCnt[rank] ++;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=ASREFX, next_state=IDLE"<<endl);
                    }
                }
                WakeUpTime[rank] ++;
                break;
            }
            case SRPDE: {
                if (pd_timing_met) {
                    RankState[rank].pd_cnt = tPDLP;
                    RankState[rank].lp_state = SRPD;
                    RankState[rank].state_cnt = tCSPD;
                    phy p;
                    p.command.type = SRPD_CMD;
                    p.command.rank = rank;
                    p.command.task = 0xFFFFFFFFFFFFFF0;
                    p.delay = tCMD_PHY;
                    packet.push_back(p);
                    PdEnterCnt[rank] ++;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=SRPDE, next_state=SRPD"<<endl);
                    }
                }
                AsrefTime[rank] ++;
                break;
            }
            case SRPD: {
                if ((rank_has_cmd[rank] || rank_has_pre_act_cmd[rank] || fast_wakeup[rank] || funcState[rank].wakeup) &&
                        state_timing_met) {
                    RankState[rank].pd_cnt = tXP;
                    RankState[rank].lp_state = SRPDX;
                    phy p;
                    p.command.type = SRPDX_CMD;
                    p.command.rank = rank;
                    p.command.task = 0xFFFFFFFFFFFFFED;
                    p.delay = tCMD_PHY;
                    packet.push_back(p);
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=SRPD, next_state=SRPDX"<<endl);
                    }
                } else if (pd_timing_met) {
                    RankState[rank].lp_state = SRPDLP;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=SRPD, next_state=SRPDLP"<<endl);
                    }
                }
                SrpdTime[rank] ++;
                break;
            }
            case SRPDLP: {
                if ((rank_has_cmd[rank] || rank_has_pre_act_cmd[rank] || fast_wakeup[rank] || funcState[rank].wakeup) && state_timing_met) {
                    RankState[rank].pd_cnt = tXP;
                    RankState[rank].lp_state = SRPDX;
                    phy p;
                    p.command.type = SRPDX_CMD;
                    p.command.rank = rank;
                    p.command.task = 0xFFFFFFFFFFFFFED;
                    p.delay = tCMD_PHY;
                    packet.push_back(p);
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=SRPDLP, next_state=SRPDX"<<endl);
                    }
                }
                SrpdTime[rank] ++;
                break;
            }
            case SRPDX: {
                if (pd_timing_met) {
                    RankState[rank].asref_cnt = tXSR;
                    RankState[rank].lp_state = ASREFX;
                    PdExitCnt[rank] ++;
                    SrpdExitCnt[rank] ++;
                    phy p;
                    p.command.type = ASREFX_CMD;
                    p.command.rank = rank;
                    p.command.task = 0xFFFFFFFFFFFFFF2;
                    p.delay = tCMD_PHY;
                    packet.push_back(p);
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LP_STATE :: rank="<<rank<<", cur_state=SRPDX, next_state=ASREFX"<<endl);
                    }
                }
                AsrefTime[rank] ++;
                break;
            }
            default: break;
        }
    }
    // Fast wakeup clear
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) fast_wakeup[rank] = false;
    // Func wakeup clear
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        if (now() >= funcState[rank].nextPde && funcState[rank].wakeup) {
            funcState[rank].wakeup = false;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- FUNC :: Pde timing met, rank="<<rank<<endl);
            }
        }
    }
}

/***************************************************************************************************
descriptor: execute unit, this function interacts with DDR grain
            this function will be DFI bus
****************************************************************************************************/
void MemoryController::exec() {
    if (PRINT_CMD_NUM) {
        CMDNUM_PRINT(setw(10)<<now()<<": ");
        for (size_t i = 0; i < 8; i ++) {
            CMDNUM_PRINT(setw(3)<<+r_qos_cnt[i]<<"|");
        }
        CMDNUM_PRINT("|");
        for (size_t i = 0; i < 8; i ++) {
            CMDNUM_PRINT(setw(3)<<+w_qos_cnt[i]<<"|");
        }
        CMDNUM_PRINT("|");
        for (size_t i = 0; i < 8; i ++) {
            CMDNUM_PRINT(setw(3)<<+wb->rb_qos_cnt[i]<<"|");
        }
        CMDNUM_PRINT("|");
        for (size_t i = 0; i < 8; i ++) {
            CMDNUM_PRINT(setw(3)<<+wb->wb_qos_cnt[i]<<"|");
        }
        CMDNUM_PRINT("|"<<endl);
    }

    if (!packet.empty()) {
        uint8_t size = packet.size();
        uint8_t erase_cnt = 0;
        for (size_t i = 0; i < size; i ++) {
            if (packet[i - erase_cnt].delay == 0) {
                (*ranks)[packet[0].command.rank]->receiveFromBus(packet[i - erase_cnt].command);
                if (PRINT_IDLE_LAT) {
                    if (command.type == READ_CMD) {
                        DEBUG(setw(10)<<now()<<" -- EXEC(DRAM) :: send [READ_CMD] to DFI task="
                                <<command.task<<" bank="<<command.bankIndex<<" row="<<command.row<<" wcnt="
                                <<que_write_cnt<<" rcnt="<<que_read_cnt<<" bl="<<command.bl);
                    } else if (command.type == READ_P_CMD) {
                        DEBUG(setw(10)<<now()<<" -- EXEC(DRAM) :: send [READ_P_CMD] to DFI task="
                                <<command.task<<" bank="<<command.bankIndex<<" row="<<command.row<<" wcnt="
                                <<que_write_cnt<<" rcnt="<<que_read_cnt<<" bl="<<command.bl);
                    } else if (command.type == ACTIVATE1_CMD) {
                        DEBUG(setw(10)<<now()<<" -- EXEC(DRAM) :: send [ACTIVATE1_CMD] to DFI task="
                                <<command.task<<" bank="<<command.bankIndex<<" row="<<command.row
                                <<" bl="<<command.bl);
                    } else if (command.type == ACTIVATE2_CMD) {
                        DEBUG(setw(10)<<now()<<" -- EXEC(DRAM) :: send [ACTIVATE2_CMD] to DFI task="
                                <<command.task<<" bank="<<command.bankIndex<<" row="<<command.row
                                <<" bl="<<command.bl);
                    }
                }
                packet.erase(packet.begin());
                erase_cnt ++;
            }
        }
    }

    if (!WriteResp.empty()) {
        if (pre_wresp_time != now()) {
            if (write_response(WriteResp[0],0)) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- Wresp Received :: task="<<WriteResp[0]<<endl);
                }
                pre_wresp_time = now();
                WriteResp.erase(WriteResp.begin());
                dresp_cnt --;
            } else {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- Wresp Back Pressure :: task="<<WriteResp[0]<<endl);
                }
            }
        }
    }

    if (!ReadResp.empty()) {
        if (pre_rresp_time != now()) {
            if (read_response(ReadResp[0],0)) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- Rresp Received :: task="<<ReadResp[0]<<endl);
                }
                pre_rresp_time = now();
                ReadResp.erase(ReadResp.begin());
                dresp_cnt --;
            } else {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- Rresp Back Pressure :: task="<<ReadResp[0]<<endl);
                }
            }
        }
    }

    //check for outgoing command packets and handle countdowns
    arb_enable = true;
    if (exec_valid) {
        if (command_pend != 0 && core_concurr_en) {
            command_pend --;
            if (command_pend > 1) arb_enable = false;
            if (DEBUG_BUS && command_pend > 0) {
                PRINTN(setw(10)<<now()<<" -- CMD_PEND :: task="<<command.task<<endl);
            }
        }
        if (command_pend == 0) {
            phy p;
            p.command = command;
            p.delay = tCMD_PHY;
            packet.push_back(p);
            exec_valid = false;

            if (!odd_cycle){
                ERROR(setw(10)<<now()<<" Every other command at even cycle violated"<<" task="<<command.task<<" cmd_type="
                        <<command.type<<" rank="<<command.rank<<" group="<<command.group<<" bank="<<command.bankIndex
                        <<" row"<<command.row);
                assert(0);
            }

            if (PRINT_EXEC) {
                static uint64_t pre_time_exec = 0;
                if (command.type == READ_CMD) {
                    DEBUGN(setw(10)<<now()<<" -- EXEC :: send [READ_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex
                            <<" row="<<command.row<<" addr_col="<<command.addr_col<<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt
                            <<" bl="<<command.bl<<" interval="<<(now()-pre_time_exec));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    DEBUGN(" chnl="<<channel<<endl);
                    pre_time_exec = now();
                } else if (command.type == READ_P_CMD) {
                    DEBUGN(setw(10)<<now()<<" -- EXEC :: send [READ_P_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex
                            <<" row="<<command.row<<" addr_col="<<command.addr_col<<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt
                            <<" bl="<<command.bl<<" interval="<<(now()-pre_time_exec));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    DEBUGN(" chnl="<<channel<<endl);
                    pre_time_exec = now();
                } else if (command.type == WRITE_CMD) {
                    DEBUGN(setw(10)<<now()<<" -- EXEC :: send [WRITE_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex
                            <<" row="<<command.row<<" addr_col"<<command.addr_col<<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt
                            <<" bl="<<command.bl<<" interval="<<(now()-pre_time_exec));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    DEBUGN(" chnl="<<channel<<endl);
                    pre_time_exec = now();
                } else if (command.type == WRITE_P_CMD) {
                    DEBUGN(setw(10)<<now()<<" -- EXEC :: send [WRITE_P_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex
                            <<" row="<<command.row<<" addr_col="<<command.addr_col<<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt
                            <<" bl="<<command.bl<<" interval="<<(now()-pre_time_exec));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    DEBUGN(" chnl="<<channel<<endl);
                    pre_time_exec = now();
                } else if (command.type == WRITE_MASK_CMD) {
                    DEBUGN(setw(10)<<now()<<" -- EXEC :: send [WRITE_MASK_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex
                            <<" row="<<command.row<<" addr_col="<<command.addr_col<<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt
                            <<" bl="<<command.bl<<" interval="<<(now()-pre_time_exec));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    DEBUGN(" chnl="<<channel<<endl);
                    pre_time_exec = now();
                } else if (command.type == WRITE_MASK_P_CMD) {
                    DEBUGN(setw(10)<<now()<<" -- EXEC :: send [WRITE_MASK_P_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex
                            <<" row="<<command.row<<" addr_col="<<command.addr_col<<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt
                            <<" bl="<<command.bl<<" interval="<<(now()-pre_time_exec));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    DEBUGN(" chnl="<<channel<<endl);
                    pre_time_exec = now();
                }
            }
            if (PRINT_IDLE_LAT) {
                if (command.type == READ_CMD) {
                    DEBUG(setw(10)<<now()<<" -- EXEC(DFI) :: send [READ_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" bank="<<command.bankIndex<<" row="<<command.row
                            <<" addr_col="<<command.addr_col<<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<" bl="<<command.bl);
                } else if (command.type == READ_P_CMD) {
                    DEBUG(setw(10)<<now()<<" -- EXEC(DFI) :: send [READ_P_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" bank="<<command.bankIndex<<" row="<<command.row
                            <<" addr_col="<<command.addr_col<<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<" bl="<<command.bl);
                } else if (command.type == ACTIVATE1_CMD) {
                    DEBUG(setw(10)<<now()<<" -- EXEC(DFI) :: send [ACTIVATE1_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" bank="<<command.bankIndex<<" row="<<command.row
                            <<" addr_col="<<command.addr_col<<" bl="<<command.bl);
                } else if (command.type == ACTIVATE2_CMD) {
                    DEBUG(setw(10)<<now()<<" -- EXEC(DFI) :: send [ACTIVATE2_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" bank="<<command.bankIndex<<" row="<<command.row
                            <<" addr_col="<<command.addr_col<<" bl="<<command.bl);
                }
            }
            if (DEBUG_BUS) {
                static uint64_t pre_time = 0;
                if (command.type == READ_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [READ_CMD] to DFI task="<<command.task<<" rank="<<command.rank
                            <<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex<<" row="<<command.row
                            <<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<" bl="<<command.bl<<" interval="<<(now()-pre_time));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    PRINTN(endl);
                    pre_time = now();
                } else if (command.type == READ_P_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [READ_P_CMD] to DFI task="<<command.task<<" rank="<<command.rank
                            <<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex<<" row="<<command.row
                            <<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<" bl="<<command.bl<<" interval="<<(now()-pre_time));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    PRINTN(endl);
                    pre_time = now();
                } else if (command.type == ACTIVATE1_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [ACTIVATE1_CMD] to DFI task="<<command.task<<" rank="<<command.rank
                            <<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex<<" row="<<command.row<<endl);
                } else if (command.type == ACTIVATE2_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [ACTIVATE2_CMD] to DFI task="<<command.task<<" rank="<<command.rank
                            <<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex<<" row="<<command.row<<endl);
                } else if (command.type == WRITE_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [WRITE_CMD] to DFI task="<<command.task<<" rank="<<command.rank
                            <<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex<<" row="<<command.row
                            <<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<" bl="<<command.bl<<" interval="<<(now()-pre_time));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    PRINTN(endl);
                    pre_time = now();
                } else if (command.type == WRITE_P_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [WRITE_P_CMD] to DFI task="<<command.task<<" rank="<<command.rank
                            <<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex<<" row="<<command.row
                            <<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<" bl="<<command.bl<<" interval="<<(now()-pre_time));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    PRINTN(endl);
                    pre_time = now();
                } else if (command.type == WRITE_MASK_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [WRITE_MASK_CMD] to DFI task="<<command.task<<" rank="<<command.rank
                            <<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex<<" row="<<command.row
                            <<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<" bl="<<command.bl<<" interval="<<(now()-pre_time));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    PRINTN(endl);
                    pre_time = now();
                } else if (command.type == WRITE_MASK_P_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [WRITE_MASK_P_CMD] to DFI task="<<command.task<<" rank="<<command.rank
                            <<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex<<" row="<<command.row
                            <<" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<" bl="<<command.bl<<" interval="<<(now()-pre_time));
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    PRINTN(endl);
                    pre_time = now();
                } else if (command.type == PRECHARGE_SB_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [PRECHARGE_SB_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" bank="<<command.bankIndex<<" row="<<command.row<<endl);
                } else if (command.type == PRECHARGE_PB_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [PRECHARGE_PB_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" bank="<<command.bankIndex<<" row="<<command.row<<endl);
                } else if (command.type == PRECHARGE_AB_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [PRECHARGE_AB_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<endl);
                } else if (command.type == REFRESH_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [REFRESH_CMD] to DFI task="<<command.task<<", rank="
                            <<command.rank<<endl);
                } else if (command.type == PER_BANK_REFRESH_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [PER_BANK_REFRESH_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" sid="<<command.sid<<" group="<<command.group<<" bank="<<command.bankIndex<<endl);
                } else if (command.type == ACTIVATE1_DST_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [ACTIVATE1_DST_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" group="<<command.group<<" bank="<<command.bankIndex<<endl);
                } else if (command.type == ACTIVATE2_DST_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [ACTIVATE2_DST_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<" group="<<command.group<<" bank="<<command.bankIndex<<endl);
                } else if (command.type == PRECHARGE_PB_DST_CMD) {
                    PRINTN(setw(10)<<now()<<" -- EXEC :: send [PRECHARGE_PB_DST_CMD] to DFI task="<<command.task<<" rank="
                            <<command.rank<<endl);
                }
            }
        }
    }
    if (core_concurr_en) {
        for (auto &dfi : packet) if (dfi.delay > 0) dfi.delay --;
        for (auto &state : bankStates) {
            if (state.state->currentBankState == RowActive) BankRowActCnt[state.bankIndex] ++;
        }
    }
}

bool MemoryController::write_response(uint64_t task,uint64_t address) {
     return (*parentMemorySystem->WriteResp)(channel,task,0,0,0);
}

bool MemoryController::read_response(uint64_t task,uint64_t address) {
     return (*parentMemorySystem->ReadResp)(channel,task,0,0,0);
}

bool MemoryController::cmd_response(uint64_t task,uint64_t address) {
     return (*parentMemorySystem->CmdResp)(channel,task,0,0,0);
}

/***************************************************************************************************
descriptor: state machine
****************************************************************************************************/
void MemoryController::state_fresh() {
    //update bank states
    for (auto &state : bankStates) {
        unsigned state_channel = (state.bankIndex % NUM_BANKS) / sc_bank_num;
        if (EM_ENABLE && EM_MODE==2 && state.rank==1 && state_channel==1) continue;   // rank1, sc1 forbidden under combo e-mode                 
        if (state.state->rwIntlvCountdown > 0 && core_concurr_en) state.state->rwIntlvCountdown --;
        if (state.state->stateChangeEn) {
            //decrement counters
            if (core_concurr_en && state.state->stateChangeCountdown > 0){
                state.state->stateChangeCountdown --;
            }
            //if counter has reached 0, change state
            if (state.state->stateChangeCountdown == 0) {
                switch (state.state->lastCommand) { //only these commands have an implicit state change
                    case WRITE_P_CMD:
                    case READ_P_CMD:
                    case WRITE_MASK_P_CMD:{
                        // fix ,if the state of transaction is precharging ,it must be hold until idle
                        state.state->currentBankState = Precharging;
                        state.state->lastCommand = PRECHARGE_PB_CMD;
                        state.state->stateChangeCountdown = state.state->fg_ref ? tRPfg : tRPpb;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- CHANGE :: READ/WRITE/WRITE_MASK_P_CMD, rank="
                                    <<state.rank<<", bank="<<state.bankIndex<<endl);
                        }
                        break;
                    }
                    case REFRESH_CMD :{
                        unsigned sub_channel = (state.bankIndex % NUM_BANKS) / sc_bank_num;
                        state.state->currentBankState = Idle;
                        refreshALL[state.rank][sub_channel].refreshWaiting = false;       //todo: revise for e-mode
                        refreshALL[state.rank][sub_channel].refreshing = false;           //todo: revise for e-mode 
                        refreshPerBank[state.bankIndex].refreshWaiting = false;
                        refreshPerBank[state.bankIndex].refreshing = false;
                        refreshPerBank[state.bankIndex].refreshWaitingPre = false;
                        state.hold_refresh_pb = false;
                        state.state->stateChangeEn = false;
                        if (DEBUG_BUS && (state.bankIndex % NUM_BANKS == 0)) {
                            PRINTN(setw(10)<<now()<<" -- CHANGE :: REFRESH_CMD, rank="
                                    <<state.rank<<", bank="<<state.bankIndex<<endl);
                        }
                        break;
                    }
                    case PRECHARGE_SB_CMD :
                    case PRECHARGE_PB_CMD :{
                        state.state->currentBankState = Idle;
                        state.state->stateChangeEn = false;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- CHANGE :: PRECHARGE_P/SB_CMD, rank="
                                    <<state.rank<<", bank="<<state.bankIndex<<endl);
                        }
                        break;
                    }
                    case PRECHARGE_AB_CMD :{
                        state.state->currentBankState = Idle;
                        state.state->stateChangeEn = false;
                        if (DEBUG_BUS && (state.bankIndex % NUM_BANKS == 0)) {
                            PRINTN(setw(10)<<now()<<" -- CHANGE :: PRECHARGE_AB_CMD, rank="
                                    <<state.rank<<", bank="<<state.bankIndex<<endl);
                        }
                        break;
                    }
                    case PER_BANK_REFRESH_CMD :{
                        state.state->currentBankState = Idle;
                        refreshPerBank[state.bankIndex].refreshWaiting = false;
                        refreshPerBank[state.bankIndex].refreshing = false;
                        refreshPerBank[state.bankIndex].refreshWaitingPre = false;
                        state.hold_refresh_pb = false;
                        //state.finish_refresh_pb = true;
                        state.state->stateChangeEn = false;
                        sbr_gap_cnt[state.rank] = now() + SBR_GAP_CNT;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- CHANGE :: PER_BANK_REFRESH_CMD, rank="
                                    <<state.rank<<", bank="<<state.bankIndex<<endl);
                        }
                        break;
                    }
                    case ACTIVATE2_DST_CMD :
                    case ACTIVATE2_CMD :{
                        state.state->stateChangeEn = false;
                        refreshPerBank[state.bankIndex].refreshWaiting = false;
                        refreshPerBank[state.bankIndex].refreshing = false;
                        refreshPerBank[state.bankIndex].refreshWaitingPre = false;
                        state.hold_refresh_pb = false;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- CHANGE :: ACTIVATE2/_DST_CMD, rank="
                                    <<state.rank<<", bank="<<state.bankIndex<<endl);
                        }
                        break;
                    }
                    case PRECHARGE_PB_DST_CMD :{
                        state.state->currentBankState = Idle;
                        refreshPerBank[state.bankIndex].refreshWaiting = false;
                        refreshPerBank[state.bankIndex].refreshing = false;
                        refreshPerBank[state.bankIndex].refreshWaitingPre = false;
                        state.state->stateChangeEn = false;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- CHANGE :: PRECHARGE_PB_DST_CMD, rank="
                                    <<state.rank<<", bank="<<state.bankIndex<<endl);
                        }
                        break;
                    }
                    default: break;
                }
            }
        }
    }

    if (!WCK_ALWAYS_ON && (IS_LP5 || IS_LP6 || IS_GD2)) {
        for (size_t i = 0; i < NUM_RANKS; i ++) {
            if (!RankState[i].wck_on) continue;
            if (now() < RankState[i].wck_off_time) continue;
            RankState[i].wck_on = false;
        }
    }
}

/***************************************************************************************************
descriptor: refreshing process
****************************************************************************************************/
bool MemoryController::check_rankStates(uint8_t rank) {
     for (size_t i = 0; i < NUM_BANKS; i ++) {
         uint32_t bank = rank*NUM_BANKS + i;
         if (bankStates[bank].state->currentBankState == RowActive) return false;
     }
     return true;
}

void MemoryController::refresh(unsigned sc) {
    //if its time for a refresh issue a refresh
    // else pop from command queue if it's not empty
    bool combo_emode_unable = EM_ENABLE && (EM_MODE==2) && (sc==1); 
    if (IS_GD2) {
        if (PBR_EN && arb_enable) gd2_dist_refresh();
        return;
    } else if (AREF_EN || PBR_EN) { // other spec
        if (DMC_V590 && SBR_IDLE_ADAPT_EN) {
            if (now() % SBR_IDLE_ADAPT_WIN == 0 && now() != 0) {
                for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
                    if (combo_emode_unable && rank==1) continue;                                                  //rank1, sc1 forbidden under combo e-mode  
                    rank_send_pbr[rank][sc] = rank_cnt_sbridle[rank][sc] >= SBR_IDLE_ADAPT_LEVEL;      //todo: revise for e-mode
                    rank_cnt_sbridle[rank][sc] = 0;                                                    //todo: revise for e-mode
                }
            }
        }
        

        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            if (refreshALL[rank][sc].refresh_cnt > 8) {
                ERROR(setw(10)<<now()<<" Postponed exceed 8, rank="<<rank<<", sc="<<sc<<", cnt="<<refreshALL[rank][sc].refresh_cnt);
                assert(0);
            }
        }
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            if (combo_emode_unable && rank==1) continue;                                                  //rank1, sc1 forbidden under combo e-mode  
            if ((RankState[rank].lp_state == IDLE || RankState[rank].lp_state == PDE ||
                    RankState[rank].lp_state == PD || RankState[rank].lp_state == PDLP ||
                    RankState[rank].lp_state == PDX) &&
                    ((now() + ref_offset[rank] - AsrefTime[rank] - SrpdTime[rank]) % tREFI) == 0) {    //todo: revise for e-mode
                refreshALL[rank][sc].refresh_cnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PSTPND_ADD :: POSTPND="<<refreshALL[rank][sc].refresh_cnt<<", rank="
                            <<rank<<", sc="<<sc<<", AsrefTime="<<AsrefTime[rank]<<", SrpdTime="<<SrpdTime[rank]<<endl);
                }
            }
            if (refresh_pbr_has_finish[rank][sc]) {
                refreshALL[rank][sc].refresh_cnt --;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PSTPND_SUB :: POSTPND="<<refreshALL[rank][sc].refresh_cnt<<", rank="
                            <<rank<<", sc="<<sc<<", AsrefTime="<<AsrefTime[rank]<<", SrpdTime="<<SrpdTime[rank]<<endl);
                }
            }
        }
    }


    if (AREF_EN) {
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            if (RankState[rank].lp_state == ASREFE) continue;
            if (RankState[rank].lp_state == ASREF) continue;
            if (RankState[rank].lp_state == ASREFX) continue;
            if (RankState[rank].lp_state == SRPDE) continue;
            if (RankState[rank].lp_state == SRPD) continue;
            if (RankState[rank].lp_state == SRPDLP) continue;
            if (RankState[rank].lp_state == SRPDX) continue;
            if (combo_emode_unable && rank==1) continue;                                                  //rank1, sc1 forbidden under combo e-mode  
            all_bank_refresh(rank, sc);
        }
    }

    if (PBR_EN) {
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            if (combo_emode_unable && rank==1) continue;                                                  //rank1, sc1 forbidden under combo e-mode  
            if (refresh_pbr_has_finish[rank][sc]) {
                refresh_pbr_has_finish[rank][sc] = false;
            }
            if ((!IS_HBM2E && !IS_HBM3) && !PBR_PARA_EN) {
                if (ENH_PBR_EN){     //todo: revise for e-mode
                    unsigned pbr_bank_cnt = 0;
                    unsigned pre_bank_tmp = 0;
                    for (size_t i = 0; i < pbr_sb_group_num; i ++) {
                        for (size_t j = 0; j < pbr_sb_num; j ++) {
                            unsigned bank_tmp = rank * NUM_BANKS + i * pbr_sb_group_num + j;
                            if (refreshPerBank[bank_tmp].refreshWaiting || refreshPerBank[bank_tmp].refreshing) {
                                if ((bank_tmp % pbr_sb_group_num)!=(pre_bank_tmp % pbr_sb_group_num) && (((pbr_bank_cnt==1) && (pre_bank_tmp==0)) || pre_bank_tmp!=0)) {
                                    ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] Enhanced Refresh Wrong Bank pairs, rank="<<rank<<" bank="<<bank_tmp<<" pre bank="<<pre_bank_tmp);
                                    assert(0);
                                }
                                pre_bank_tmp = bank_tmp;
                                pbr_bank_cnt ++;
                                if (pbr_bank_cnt > 2) {
                                    ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] Enhanced Refresh bank exceed 2, rank="<<rank);
                                    assert(0);
                                }
                            }
                        }
                    }
                } else {
                    for (size_t i = 0; i < pbr_bank_num; i ++) {
                        unsigned bank_tmp = rank * NUM_BANKS + i + (sc*sc_bank_num);
                        if (refreshPerBank[bank_tmp].refreshWaiting || refreshPerBank[bank_tmp].refreshing) {
                            for (size_t j = i + 1; j < pbr_bank_num; j ++) {
                                unsigned bank_loop = rank * NUM_BANKS + j + (sc*sc_bank_num);
                                if (refreshPerBank[bank_loop].refreshWaiting || refreshPerBank[bank_loop].refreshing) {
                                    ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] Refresh bank exceed 2, rank="<<rank<<", sc="<<sc);
                                    assert(0);
                                }
                            }
                            break;
                        }
                    }
                }
            }
            if (RankState[rank].lp_state == ASREFE) continue;
            if (RankState[rank].lp_state == ASREF) continue;
            if (RankState[rank].lp_state == ASREFX) continue;
            if (RankState[rank].lp_state == SRPDE) continue;
            if (RankState[rank].lp_state == SRPD) continue;
            if (RankState[rank].lp_state == SRPDLP) continue;
            if (RankState[rank].lp_state == SRPDX) continue;
            if ((RankState[rank].lp_state == PD || RankState[rank].lp_state == PDLP) && !PD_PBR_EN) continue;
            if (ENH_PBR_EN) {
                enh_per_bank_refresh(rank, sc);
            } else {
                per_bank_refresh(rank, sc);
            }
        }
    }
}

void MemoryController::gd2_dist_refresh() {
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        bool has_bank_distref = false;
        unsigned distref_bank = 0;
        if (SEND_DSTREF_SERIAL) {
            for (size_t l = 0; l < NUM_BANKS; l ++) {
                if (refreshPerBank[i * NUM_BANKS + l].refreshing) {
                    has_bank_distref = true;
                    distref_bank = i * NUM_BANKS + l;
                    break;
                }
            }
        }

        for (size_t j = 0; j < NUM_BANKS; j ++) {
            unsigned bank = i * NUM_BANKS + j;
            for (size_t k = 0; k < NUM_MATGRPS; k ++) {
                if (DistRefState[bank].pre_cmd_cnt[k] == PRE_NUM_SEND_PBR) {
                    DistRefState[bank].pre_cmd_cnt[k] = 0;
                    DistRefState[bank].dist_pstpnd_num[k] ++;
                }
                if (DistRefState[bank].dist_pstpnd_num[k] > 8) {
                    ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] DistRef Postponed exceed, bank="
                            <<bank<<", matgrp="<<k<<", cnt="<<DistRefState[bank].dist_pstpnd_num[k]);
                    assert(0);
                }
            }

            for (size_t k = 0; k < NUM_MATGRPS; k ++) {
                DistRefState[bank].force_dist_refresh = false;
                if (DistRefState[bank].dist_pstpnd_num[k] < PBR_PSTPND_LEVEL) continue;
                DistRefState[bank].force_dist_refresh = true;
                break;
            }

            bool has_matgrp_distref = false;
            unsigned matgrp_distref = 0, max_dist_pstpnd_num = 0;
            for (size_t k = 0; k < NUM_MATGRPS; k ++) {
                if (DistRefState[bank].dist_pstpnd_num[k] == 0) continue;
                has_matgrp_distref = true;
                if (DistRefState[bank].dist_pstpnd_num[k] > max_dist_pstpnd_num) {
                    matgrp_distref = k;
                    max_dist_pstpnd_num = DistRefState[bank].dist_pstpnd_num[k];
                }
            }

            if (bankStates[bank].state->currentBankState != Idle &&
                    bankStates[bank].state->currentBankState != Refreshing) continue;
            if (!DistRefState[bank].force_dist_refresh && bank_cnt[bank] > 0 &&
                    bankStates[bank].state->currentBankState != Refreshing) continue;
            if (bankStates[bank].state->act_executing) continue;
            if (SEND_DSTREF_SERIAL && has_bank_distref && distref_bank != bank) continue;
            if (!has_matgrp_distref) continue;
            unsigned rank = bank / NUM_BANKS;
            funcState[rank].wakeup = true;
            if (RankState[rank].lp_state != IDLE) continue;

            switch (DistRefState[bank].state) {
                case DIST_IDLE : {
                    DistRefState[bank].state = SEND_ACT1;
                    break;
                }
                case SEND_ACT1 : {
                    if ((now() + 1) >= bankStates[bank].state->nextActivate1) {
                        Cmd *c = new Cmd;
                        c->state = working;
                        c->cmd_type = ACTIVATE1_DST_CMD;
                        c->bank = bankStates[bank].bank;
                        c->rank = bankStates[bank].rank;
                        c->group = bankStates[bank].group;
                        c->bankIndex = bankStates[bank].bankIndex;
                        c->row = matgrp_distref;
                        c->cmd_source = 2;
                        c->task = 0xFFFFFFFFFFFFFFE;
                        CmdQueue.push_back(c);
                    }
                    break;
                }
                case SEND_ACT2 : {
                    if ((now() + 1) >= bankStates[bank].state->nextActivate2 && tFAWCountdown[rank].size() < 4) {
                        Cmd *c = new Cmd;
                        c->state = working;
                        c->cmd_type = ACTIVATE2_DST_CMD;
                        c->bank = bankStates[bank].bank;
                        c->rank = bankStates[bank].rank;
                        c->group = bankStates[bank].group;
                        c->bankIndex = bankStates[bank].bankIndex;
                        c->row = matgrp_distref;
                        c->cmd_source = 2;
                        c->task = 0xFFFFFFFFFFFFFFE;
                        CmdQueue.push_back(c);
                    }
                    break;
                }
                case SEND_PRE : {
                    if ((now() + 1) >= bankStates[bank].state->nextPrecharge && tFPWCountdown[rank].size() < 4) {
                        Cmd *c = new Cmd;
                        c->state = working;
                        c->cmd_type = PRECHARGE_PB_DST_CMD;
                        c->bank = bankStates[bank].bank;
                        c->rank = bankStates[bank].rank;
                        c->group = bankStates[bank].group;
                        c->bankIndex = bankStates[bank].bankIndex;
                        c->row = matgrp_distref;
                        c->cmd_source = 2;
                        c->task = 0xFFFFFFFFFFFFFFE;
                        CmdQueue.push_back(c);
                    }
                    break;
                }
            }
        }
    }
}

void MemoryController::all_bank_refresh(unsigned rank, unsigned sc) {
    if (RankState[rank].lp_state == ASREFE || RankState[rank].lp_state == ASREF ||
            RankState[rank].lp_state == ASREFX || RankState[rank].lp_state == SRPDE ||
            RankState[rank].lp_state == SRPD || RankState[rank].lp_state == SRPDLP ||
            RankState[rank].lp_state == SRPDX || refreshALL[rank][sc].refresh_cnt == 0) {
        return;
    }

    //guarantee rd/wr cmd with issue_size!=0 not broken by refresh
    bool has_issue_size = false;
    for (auto &trans : transactionQueue) {
        unsigned trans_sc = (trans->bankIndex % NUM_BANKS) / sc_bank_num;
        if (trans->issue_size != 0 && trans->rank == rank && trans_sc == sc) {
            has_issue_size =true;
        }
    }

    if (has_issue_size) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- Rd/Wr cmd (issue!=0) must be send before all bank refresh"<<" rank="<<rank<<", sc="<<sc<<endl);
        }
        return;
    }


    // guarantee activate2 followed with activate1 within 8 cycles
    bool act2_left = false;
    for (size_t bank = 0; bank < NUM_BANKS * NUM_RANKS; bank ++) {
        if (act_executing[bank]){
            act2_left = true;
        }        
    }

    if (act2_left){     //todo: revise for e-mode
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- Act2 must be send before refresh"<<" rank="<<rank<<", sc="<<sc<<endl);
        }
        return;
    }
    
    unsigned bank_start = sc * (NUM_BANKS/sc_num);
//    unsigned bank_pair_start = sc * pbr_bank_num; 

    if (refreshALL[rank][sc].refreshWaiting && !refreshALL[rank][sc].refreshing) {
        bool idle = true;
        bool precharge_en = true;
        for (size_t bank = 0; bank < NUM_BANKS/sc_num; bank ++) {
            unsigned bank_tmp = rank * NUM_BANKS + bank + bank_start;
            if (bankStates[bank_tmp].state->currentBankState == RowActive) {
                idle = false;
                break;
            }
        }
        for (size_t bank = 0; bank < NUM_BANKS/sc_num; bank ++) {
            unsigned bank_tmp = rank * NUM_BANKS + bank + bank_start;
            if ((now() + 1) < bankStates[bank_tmp].state->nextPrecharge || bankStates[bank_tmp].state->act_executing) {
                precharge_en = false;
                break;
            }
        }
        if (arb_enable && !idle && precharge_en) {
            funcState[rank].wakeup = true;
//            if (RankState[rank].lp_state == IDLE) {
            if (RankState[rank].lp_state == IDLE && even_cycle) {    //every other command, even;
                while (!CmdQueue.empty()) {
                    CmdQueue.erase(CmdQueue.begin());
                }
                Cmd *c = new Cmd;
                c->rank = rank;
                c->group = 0;
                c->bank = 0;
                c->row = 0;
                c->cmd_type = PRECHARGE_AB_CMD;
//                c->channel = sc;           // sub channel for lpddr6
//                c->bankIndex = rank * NUM_BANKS;
                c->bankIndex = rank * NUM_BANKS + sc * sc_bank_num;
                c->cmd_source = 2;
                c->task = 0xFFFFFFFFFFFFFF9;
                CmdQueue.push_back(c);
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- REQ :: refresh, precharge bank"<<c->bank<<", task="<<c->task<<", rank="<<rank<<", sc="<<sc<<endl);
                }
            } else {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- REQ_LP :: hold PRECHARGE in lp state, rank="<<rank<<", sc="<<sc<<endl);
                }
            }
        }

        //if true , it means this rank is in Idle states
        if (idle) {
            bool abr_timing_met = true;
            for (size_t bank = 0; bank < NUM_BANKS/sc_num; bank ++) {
                if ((now() + 1) < bankStates[rank * NUM_BANKS + bank + bank_start].state->nextAllBankRefresh) {
                    abr_timing_met = false;
                    break;
                }
            }
//            if (arb_enable && abr_timing_met) {
            if (arb_enable && abr_timing_met) { 
                funcState[rank].wakeup = true;
//                if (RankState[rank].lp_state == IDLE) {
                if (RankState[rank].lp_state == IDLE && even_cycle) {   //every other command, even;
                    //refresh this rank
                    while (!CmdQueue.empty()) {
                        CmdQueue.erase(CmdQueue.begin());
                    }
                    //step 2 : construct a refresh command
                    Cmd *c = new Cmd;
                    c->rank = rank;
//                    c->channel = sc;
//                    c->bankIndex = rank * NUM_BANKS;
                    c->bankIndex = rank * NUM_BANKS + sc * sc_bank_num;
                    c->cmd_type = REFRESH_CMD;
                    c->cmd_source = 2;
                    c->task = 0xFFFFFFFFFFFFFFA;
                    CmdQueue.push_back(c);
                    force_pbr_refresh[rank][sc] = false;

                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- REQ :: REFRESH rank"<<c->rank<<", task="<<c->task<<" rank="<<rank<<", sc="<<sc<<endl);
                    }
                } else {
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- REQ_LP :: hold REFRESH in lp state, rank="<<rank<<", sc="<<sc<<endl);
                    }
                }
            }
        }
    }

    if (refreshALL[rank][sc].refresh_cnt >= ABR_PSTPND_LEVEL) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- POS :: refresh cnt="<<refreshALL[rank][sc].refresh_cnt<<", force refresh"<<", rank="<<rank<<", sc="<<sc<<endl);
        }
        refreshALL[rank][sc].refreshWaiting = true;
    } else if (refreshALL[rank][sc].refresh_cnt > 0) {
        if (sc_cnt[rank][sc] == 0) {      //todo: revise for e-mode
            if (SBR_IDLE_EN) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- POS :: refresh_cnt="<<refreshALL[rank][sc].refresh_cnt
                            <<", rank="<<rank<<", sc="<<sc<<" is Idle, will refresh by pbr"<<endl);
                }
            } else if (!rank_send_pbr[rank][sc]) {
                bool no_bank_pbr = true;
                for (size_t bank = 0; bank < NUM_BANKS/sc_num; bank ++) {
                    unsigned bank_tmp = rank * NUM_BANKS + bank + bank_start;
                    if (refreshPerBank[bank_tmp].refreshWaiting || refreshPerBank[bank_tmp].refreshing) {
                        no_bank_pbr = false;
                        break;
                    }
                }
                bool idle_abr_send = true;
#if 0
                if (DMC_V590) {
                    if (refreshALL[rank].refresh_cnt > ABR_PSTPND_LEVEL) {
                        idle_abr_send = true;
                    } else {
                        idle_abr_send = false;
                    }
                }
#endif
                if (no_bank_pbr && idle_abr_send) {
                    refreshALL[rank][sc].refreshWaiting = true;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- POS :: refresh_cnt="<<refreshALL[rank][sc].refresh_cnt
                                <<", rank="<<rank<<", sc="<<sc<<" is Idle, will refresh by abr"<<endl);
                    }
                }
            }
        }
    }
}

void MemoryController::per_bank_refresh(unsigned rank, unsigned sc) {
    if (refreshALL[rank][sc].refresh_cnt == 0 || refreshALL[rank][sc].refreshWaiting ||
            RankState[rank].lp_state == ASREFE || RankState[rank].lp_state == ASREF ||
            RankState[rank].lp_state == ASREFX || RankState[rank].lp_state == SRPDE ||
            RankState[rank].lp_state == SRPD || RankState[rank].lp_state == SRPDLP ||
            RankState[rank].lp_state == SRPDX) return;
    if (now() < sbr_gap_cnt[rank]) return;       //todo: revise for e-mode

#if 0
    for (size_t bank = 0; bank < pbr_bank_num * NUM_MATGRPS; bank ++) {
        PbrWeight[rank][bank].block_pbr = false;
    }
    if (DMC_V590 && refreshALL[rank].refresh_cnt < PBR_PSTPND_LEVEL) {
        for (size_t bank = 0; bank < pbr_bank_num * NUM_MATGRPS; bank ++) {
            unsigned cmd_cnt = 0;
            for (size_t bg = 0; bg < pbr_bg_num; bg ++) {
                unsigned bank_tmp = rank * NUM_BANKS + bg * pbr_bank_num + bank;
                if (r_bank_cnt[bank_tmp] != 0) cmd_cnt ++;
            }
            if (cmd_cnt != 0) {
                for (size_t bg = 0; bg < pbr_bg_num; bg ++) {
                    unsigned bank_tmp = rank * NUM_BANKS + bg * pbr_bank_num + bank;
                    refreshPerBank[bank_tmp].refreshWaiting = false;
                }
                PbrWeight[rank][bank].block_pbr = true;
            }
        }
    }
#endif

    
    // guarantee activate2 followed with activate1 within 8 cycles
    bool act2_left = false;
    for (size_t bank = 0; bank < NUM_BANKS * NUM_RANKS; bank ++) {
        if (act_executing[bank]){
            act2_left = true;
        }        
    }

    if (act2_left){     //todo: revise for e-mode
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- Act2 must be send before refresh"<<", sc="<<sc<<endl);
        }
        return;
    }

    unsigned bank_start = sc * (NUM_BANKS/sc_num);
    unsigned bank_pair_start = sc * pbr_bank_num; 
    bool have_bank_refresh = false; // send pbr one by one if set true
    for (size_t bank = 0; bank < pbr_bank_num; bank ++) {
        unsigned bank_tmp = rank * NUM_BANKS + bank + bank_start;
        //if (IS_HBM2E || IS_HBM3 ) {
        if (IS_HBM2E || IS_HBM3 || PBR_PARA_EN) {
            if (!refreshPerBank[bank_tmp].refreshWaiting) continue;
        } else {
            if (!refreshPerBank[bank_tmp].refreshWaiting && !refreshPerBank[bank_tmp].refreshing) continue;
        }
        have_bank_refresh = true;
        break;
    }

    if (!have_bank_refresh) {
//        for (size_t i = 0; i < pbr_bank_num; i++) {
//            PbrWeight[rank][i].bank_pair_cnt = 0;
//            PbrWeight[rank][i].send_pbr = false;
//        }
        for (size_t i = 0 ; i < pbr_bank_num; i++) {
            PbrWeight[rank][i + bank_pair_start].bank_pair_cnt = 0;
            PbrWeight[rank][i + bank_pair_start].send_pbr = false;
        }
        
        for (size_t bank = 0; bank < pbr_bank_num; bank ++) {
            unsigned has_bank_open = false;
            unsigned has_cmd_vld = false;
            unsigned bankIndex = rank * NUM_BANKS + bank + bank_start;
            unsigned another_bankIndex = rank * NUM_BANKS + bank + pbr_bank_num + bank_start;
            for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                unsigned bank_tmp = rank * NUM_BANKS + sbr_bank * pbr_bank_num + bank + bank_start;
                if (bankStates[bank_tmp].state->currentBankState == RowActive) {
                    has_bank_open = 1;
                    break;
                }
            }
            for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                unsigned bank_tmp = rank * NUM_BANKS + sbr_bank * pbr_bank_num + bank + bank_start;
                if (bank_cnt[bank_tmp] > 0) {
                    has_cmd_vld = 1;
                    break;
                }
            }

            if (SBR_WEIGHT_MODE == 0) PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt = has_bank_open;
            else if (SBR_WEIGHT_MODE == 1) PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt = has_cmd_vld;
            else if (SBR_WEIGHT_MODE == 2) PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt = has_bank_open + has_cmd_vld;
            else if (SBR_WEIGHT_MODE == 3) PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt = has_bank_open + has_cmd_vld * 2;
            else PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt = bank_cnt[bankIndex] + bank_cnt[another_bankIndex];    // added for SBR_WEIGHT_MODE=4/5, select bank pair with least/most cmd

            //added for SBR_WEIGHT_ENH_MODE=3/4 as bonus, select bank pair with least/most cmd
            //based on SBR_WEIGHT_MODE = 0/1/2/3, 4/5 forbidden
//            DEBUG(now()<<" bank_pair="<<(bank+bank_pair_start)<<" bankIndex="<<bankIndex<<" another bankIndex="<<another_bankIndex);
            if (SBR_WEIGHT_ENH_MODE == 3 || SBR_WEIGHT_ENH_MODE == 4) {
                bank_pair_cmd_cnt[rank][bank + bank_pair_start] = bank_cnt[bankIndex] + bank_cnt[another_bankIndex];
            }

        }


#if 1
        if (refreshALL[rank][sc].refresh_cnt < PBR_PSTPND_LEVEL) {    //todo: revise for e-mode
            for (size_t i = 0; i < NUM_BANKS/sc_num; i++) {
                unsigned bank = rank * NUM_BANKS + i + bank_start;
                if (bank_cnt[bank] > 0) PbrWeight[rank][i % pbr_bank_num + bank_pair_start].bank_pair_cnt = 0xFFFFFFFF;
            }
        }
#endif

        if (WRITE_BUFFER_ENABLE) {   // todo: revise for e-mode
            if (GBUF_RCMD_BLOCK_PBR && refreshALL[rank][sc].refresh_cnt < PBR_PSTPND_LEVEL) {
                for (size_t bank = 0; bank < NUM_BANKS/sc_num; bank ++) {
                    unsigned bank_tmp = rank * NUM_BANKS + bank + bank_start;
                    if (!wb->rcmd_bank_state[bank_tmp]) continue;
                    PbrWeight[rank][bank_tmp % pbr_bank_num + bank_pair_start].bank_pair_cnt = 0xFFFFFFFF;
                }
            }
        }

#if 0
        if (DMC_V590) {
            for (size_t bank = 0; bank < pbr_bank_num * NUM_MATGRPS; bank ++) {
                if (!PbrWeight[rank][bank].block_pbr) continue;
                PbrWeight[rank][bank].bank_pair_cnt = 0xFFFFFFFF;
            }
        }
#endif

        unsigned bank_pair_cnt_min = 0xFFFFFFFF;
        unsigned bank_pair_cnt_min_num = 0xFFFFFFFF;
        bool     pre_sch_bankIndex_met = false;
        unsigned pre_sch_bank_pair_cnt = 0xFFFFFFFF;
        unsigned pre_sch_bank_pair = 0xFFFFFFFF;
        unsigned pbr_bankidx = 0xFFFFFFFF;
        unsigned pbr_bankidx_another = 0xFFFFFFFF;
        for (size_t bank = 0; bank < pbr_bank_num; bank++) {
            if (bank_pair_cnt_min > PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt &&
                    !bankStates[rank * NUM_BANKS + bank + bank_start].finish_refresh_pb &&
                    (!DMC_V580 || PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt != 0xFFFFFFFF) &&
                    (SBR_WEIGHT_MODE!=5)) {   //todo: revise for e-mode
                bank_pair_cnt_min = PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt;
                bank_pair_cnt_min_num = bank + bank_pair_start;
            }

            //added for SBR_WEIGHT_MODE=5, select bank pair with most cmd
            if ((bank_pair_cnt_min < PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt || bank_pair_cnt_min == 0xFFFFFFFF) &&
                    !bankStates[rank * NUM_BANKS + bank + bank_start].finish_refresh_pb &&
                    (!DMC_V580 || PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt != 0xFFFFFFFF) &&
                    (SBR_WEIGHT_MODE==5)) {   //todo: revise for e-mode
                bank_pair_cnt_min = PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt;
                bank_pair_cnt_min_num = bank + bank_pair_start;
            }

            pbr_bankidx_another = rank * NUM_BANKS + bank + pbr_bank_num + bank_start;
            pbr_bankidx = rank * NUM_BANKS + bank + bank_start;
            // added for SBR_WEIGHT_ENH_MODE=2
            if ((pbr_bankidx==pre_sch_bankIndex[rank] || (pbr_bankidx_another==pre_sch_bankIndex[rank]))&& (SBR_WEIGHT_ENH_MODE==2) && 
                    (!bankStates[pbr_bankidx].finish_refresh_pb && !bankStates[pbr_bankidx_another].finish_refresh_pb) &&
                    (!DMC_V580 || PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt != 0xFFFFFFFF)) {
                pre_sch_bankIndex_met = true;
                pre_sch_bank_pair_cnt = PbrWeight[rank][bank + bank_pair_start].bank_pair_cnt; 
                pre_sch_bank_pair     = bank + bank_pair_start; 
            }

        }
        
        //added for SBR_WEIGHT_ENH_MODE=2, most recently scheduled cmd priority
        if (SBR_WEIGHT_ENH_MODE==2 && pre_sch_bankIndex_met &&
                pre_sch_bank_pair_cnt!=0xFFFFFFFF && pre_sch_bank_pair!=0xFFFFFFFF){
            if (bank_pair_cnt_min >= pre_sch_bank_pair_cnt) {
                bank_pair_cnt_min = PbrWeight[rank][pre_sch_bank_pair].bank_pair_cnt;
                bank_pair_cnt_min_num = pre_sch_bank_pair;
            }       
        }

        //added for SBR_WEIGHT_ENH_MODE=3/4, least/most cmd
        for (size_t bank = 0; bank < pbr_bank_num; bank ++) {
            unsigned bank_pair = bank + bank_pair_start;
            unsigned bankIndex = rank * NUM_BANKS + bank + bank_start;
            if (bank_pair_cnt_min_num != 0xFFFFFFFF) {
                if (PbrWeight[rank][bank_pair].bank_pair_cnt == bank_pair_cnt_min &&
                        ((bank_pair_cmd_cnt[rank][bank_pair] < bank_pair_cmd_cnt[rank][bank_pair_cnt_min_num] && SBR_WEIGHT_ENH_MODE==3) ||
                        (bank_pair_cmd_cnt[rank][bank_pair] > bank_pair_cmd_cnt[rank][bank_pair_cnt_min_num] && SBR_WEIGHT_ENH_MODE==4)) &&
                        bank_pair_cnt_min_num != bank_pair && bank_pair_cnt_min_num!=0xFFFFFFFF && !bankStates[bankIndex].finish_refresh_pb){ 
                    bank_pair_cnt_min = PbrWeight[rank][bank_pair].bank_pair_cnt;
                    bank_pair_cnt_min_num = bank_pair;
                }
            }
        }

        if (bank_pair_cnt_min_num != 0xFFFFFFFF) {
            PbrWeight[rank][bank_pair_cnt_min_num].send_pbr = true;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- PBR_WEIGHT :: rank="<<rank<<" bank="
                        <<(bank_pair_cnt_min_num/NUM_MATGRPS)<<" matgrp="
                        <<+bank_pair_cnt_min_num<<" sc="<<sc<<" win the arbitration!"<<endl);
            }
        }
    }

    refresh_cnt_pb[rank][sc] = 0;
    for (size_t bank = 0; bank < pbr_bank_num; bank ++) {       //todo: revise for e-mode
        unsigned bank_tmp = rank * NUM_BANKS + bank + bank_start;
        if (bankStates[bank_tmp].finish_refresh_pb) refresh_cnt_pb[rank][sc] ++;
        else forceRankBankIndex[rank][sc] = bank + bank_start;

        if (SBR_REQ_MODE == 0) {
            if (bank == pbr_bank_num - 1) {
                if (refresh_cnt_pb[rank][sc] == pbr_bank_num) {
                    refresh_pbr_has_finish[rank][sc] = true;
                    force_pbr_refresh[rank][sc] = false;
                    for (size_t bank_sbr = 0; bank_sbr < pbr_bank_num; bank_sbr ++) {
                        for (size_t group_sbr = 0; group_sbr < pbr_bg_num; group_sbr ++) {
                            unsigned bank_idx = rank * NUM_BANKS + bank_sbr + group_sbr * pbr_bank_num + bank_start;
                            bankStates[bank_idx].finish_refresh_pb = false;
                        }
                    }
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- PBR :: RANK "<<rank<<", SC "<<sc<<" has been refreshed by pbr"<<endl);
                    }
                } else {
                    force_pbr_refresh[rank][sc] = true;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- PBR :: Force pbr rank "<<rank<<", SC "<<sc<<" has no idle bank not refreshed"<<endl);
                    }
                }
            }
        } else {
            if (bank == pbr_bank_num - 1) {
                if (refresh_cnt_pb[rank][sc] == pbr_bank_num) {
                    refresh_pbr_has_finish[rank][sc] = true;
                    force_pbr_refresh[rank][sc] = false;
                    for (size_t bank_sbr = 0; bank_sbr < pbr_bank_num; bank_sbr ++) {
                        for (size_t group_sbr = 0; group_sbr < pbr_bg_num; group_sbr ++) {
                            unsigned bank_idx = rank * NUM_BANKS + bank_sbr + group_sbr * pbr_bank_num + bank_start;
                            bankStates[bank_idx].finish_refresh_pb = false;
                        }
                    }
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- PBR :: RANK "<<rank<<", SC "<<sc<<" has been refreshed by pbr"<<endl);
                    }
                } else if (refresh_cnt_pb[rank][sc] >= SBR_FRCST_NUM) {
                    force_pbr_refresh[rank][sc] = true;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- PBR :: Force pbr for the left banks in RANK "<<rank<<", SC "<<sc<<endl);
                    }
                }
            }
        }
    }

    for (size_t bank = 0; bank < pbr_bank_num; bank ++) {
        unsigned bank_tmp = rank * NUM_BANKS + bank + bank_start;
        if (!bankStates[bank_tmp].finish_refresh_pb) {
            bankStates[bank_tmp].en_refresh_pb = true;
            bankStates[bank_tmp].hold_refresh_pb = true;
        }
    }

    send_pb_precharge(rank, sc);
    send_pb_refresh(rank, sc);
}
void MemoryController::send_pb_precharge(unsigned rank, unsigned sc) {
    // unrefreshed bank is 2 left, force pbr
    bool have_bank_refresh = false; // send pbr one by one if set true
    unsigned bank_start = sc * (NUM_BANKS/sc_num);
    unsigned bank_pair_start = sc * pbr_bank_num; 
    //if (!IS_HBM2E && !IS_HBM3 ) {
    if ((!IS_HBM2E && !IS_HBM3) && !PBR_PARA_EN ) {
        for (size_t bank = 0; bank < pbr_bank_num; bank ++) {
            unsigned bank_tmp = rank * NUM_BANKS + bank + bank_start;
            if (refreshPerBank[bank_tmp].refreshing){
                have_bank_refresh = true;
                break;
            }
        }
    }
    for (size_t bank = 0; bank < pbr_bank_num; bank ++) {
        unsigned bank_tmp = rank * NUM_BANKS + bank + bank_start;
        if (refresh_pbr_has_finish[rank][sc]) continue;
        if (force_pbr_refresh[rank][sc] && !bankStates[bank_tmp].finish_refresh_pb
                && !have_bank_refresh && PbrWeight[rank][bank + bank_pair_start].send_pbr) {
            for (size_t matgrp = 0; matgrp < NUM_MATGRPS; matgrp ++) {
                if (arb_enable) {
                    for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                        unsigned bank_idx = bank_tmp + sbr_bank * pbr_bank_num;
                        if (!refreshPerBank[bank_idx].refreshWaiting) {
                            refreshPerBank[bank_idx].refreshWaiting = true;
                            refreshPerBank[bank_idx].refreshWaitingPre = true;
                            refreshPerBank[bank_idx].refreshing = false;
                            if (bankStates[bank_idx].state->currentBankState == RowActive) {
                                refreshPerBank[bank_idx].refreshWaitingPre = false;
                            }
                            pbr_hold_pre[rank] = true;             //todo, revise for e-mode
                            pbr_hold_pre_time[rank] = now() + 6;   //todo, revise for e-mode
                        }
                    }

                    bool ready_send_pbr = true;
                    for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                        unsigned bank_idx = bank_tmp + sbr_bank * pbr_bank_num;
                        if (bankStates[bank_idx].state->currentBankState != Idle ||
                                (now() + 1) < bankStates[bank_idx].state->nextPerBankRefresh || issue_state[bank_idx]) {
                            ready_send_pbr = false;
                            break;
                        }
                    }
                    if (ready_send_pbr && ((tFAWCountdown[rank].size() < 4 && sc==0)      //todo: revise for e-mode
                            || (tFAWCountdown_sc1[rank].size() < 4 && sc==1))
                            && (!ASREF_ENABLE || (ASREF_ENABLE && RankState[rank].asref_cnt != 0))) {
                        funcState[rank].wakeup = true;      
//                        if (RankState[rank].lp_state == IDLE) {
                        if (RankState[rank].lp_state == IDLE && even_cycle) {    // every other command, even;
                            Cmd *c = new Cmd;
                            c->state = working;
                            c->cmd_type = PER_BANK_REFRESH_CMD;
                            c->force_pbr = true;
                            c->bank = bankStates[bank_tmp].bank;
                            c->rank = bankStates[bank_tmp].rank;
                            c->group = bankStates[bank_tmp].group;
//                            c->channel = sc;
                            c->cmd_source = 2;
                            c->task = 0xFFFFFFFFFFFFFFB;
                            c->bankIndex = bankStates[bank_tmp].bankIndex;
                            for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                                unsigned bank_idx = bank_tmp + sbr_bank * pbr_bank_num;
                                bankStates[bank_idx].hold_refresh_pb = false;
                            }
                            CmdQueue.push_back(c);
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- REQ :: force pbr, rank="<<rank<<", bank="<<bank_tmp<<", sc="<<sc<<endl);
                            }
                            return;
                        } else {
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- REQ_LP :: hold FPBR in lp, rank="<<rank<<", bank="<<bank_tmp<<", sc="<<sc<<endl);
                            }
                        }
                    } else if (tFPWCountdown[rank].size() < 4) {     // no need revise for lpddr
                        bool sb_ready_send_pre = true;
                        for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                            unsigned bank_idx = bank_tmp + sbr_bank * pbr_bank_num;
                            if (bankStates[bank_idx].state->currentBankState == RowActive) {
                                for (size_t sbr_bank_w = 0; sbr_bank_w < pbr_bg_num; sbr_bank_w ++) {
                                    unsigned bank_idx_w = bank_tmp + sbr_bank_w * pbr_bank_num;
                                    refreshPerBank[bank_idx_w].refreshWaiting = true;
                                    refreshPerBank[bank_idx_w].refreshing = false;
                                    pbr_hold_pre[rank] = true;       //todo: revise for e-mode
                                    if (bankStates[bank_idx_w].state->currentBankState == RowActive) {
                                        refreshPerBank[bank_idx_w].refreshWaitingPre = false;
                                    }
                                }
                                
                                if (IS_LP4 || IS_LP5 || IS_LP6){
                                    if ((now() + 1) < bankStates[bank_idx].state->nextPrecharge || issue_state[bank_idx]) {
                                        sb_ready_send_pre = false;
                                    }
                                } else {
                                    for (size_t sbr_bank_w = 0; sbr_bank_w < pbr_bg_num; sbr_bank_w ++) {
                                        unsigned bank_idx_w = bank_tmp + sbr_bank_w * pbr_bank_num;
                                        if ((now() + 1) < bankStates[bank_idx_w].state->nextPrecharge) {
                                            sb_ready_send_pre = false;
                                            break;
                                        }
                                    }
                                }

                                if (sb_ready_send_pre) {
                                    funcState[rank].wakeup = true;
//                                    if (RankState[rank].lp_state == IDLE) {
                                    if (RankState[rank].lp_state == IDLE && even_cycle) {     //every other command, even;
                                        Cmd *c = new Cmd;
                                        c->state = working;
                                        if (IS_LP4 || IS_LP5 || IS_LP6 || IS_HBM2E || IS_HBM3) {
                                            c->cmd_type = PRECHARGE_PB_CMD;
                                            c->bank = bankStates[bank_idx].bank;
                                            c->rank = bankStates[bank_idx].rank;
                                            c->group = bankStates[bank_idx].group;
//                                            c->channel = sc;
                                            c->bankIndex = bankStates[bank_idx].bankIndex;
                                            bankStates[bank_idx].hold_refresh_pb = true;
                                            bankStates[bank_idx].finish_refresh_pb = false;
                                            pbr_hold_pre_time[rank] = now() + 4;           //todo: revise for e-mode
                                        } else if (IS_DDR5) {
                                            c->cmd_type = PRECHARGE_SB_CMD;
                                            c->bank = bankStates[bank_tmp].bank;
                                            c->rank = bankStates[bank_tmp].rank;
                                            c->group = bankStates[bank_tmp].group;
//                                            c->channel = sc;
                                            c->bankIndex = bankStates[bank_tmp].bankIndex;
                                            for (size_t sbr_bank_i = 0; sbr_bank_i < pbr_bg_num; sbr_bank_i ++) {
                                                unsigned bank_idx_i = bank_tmp + sbr_bank_i * pbr_bank_num;
                                                bankStates[bank_idx_i].hold_refresh_pb = true;
                                                bankStates[bank_idx_i].finish_refresh_pb = false;
                                            }
                                        }
                                        c->force_pbr = true;
                                        c->cmd_source = 2;
                                        c->task = 0xFFFFFFFFFFFFFFD;
                                        CmdQueue.push_back(c);
                                        if (DEBUG_BUS) {
                                            PRINTN(setw(10)<<now()<<" -- REQ :: force precharge, rank="<<rank<<", bank="<<bank_tmp<<", sc="<<sc<<endl);
                                        }
                                        return;
                                    } else {
                                        if (DEBUG_BUS) {
                                            PRINTN(setw(10)<<now()<<" -- REQ_LP :: hold PRECHARGE PB/SB in lp"
                                                    <<" state, rank"<<rank<<", sc="<<sc<<", bank="<<bank_tmp<<endl);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void MemoryController::send_pb_refresh(unsigned rank, unsigned sc) {
    
    unsigned bank_start = sc * (NUM_BANKS/sc_num);
    unsigned bank_pair_start = sc * pbr_bank_num; 
    // find Idle bank send pb refresh
    for (size_t bank = 0; bank < pbr_bank_num; bank ++) {    //todo: revise for e-mode
        unsigned bank_tmp = rank * NUM_BANKS + bank + bank_start;
        if (force_pbr_refresh[rank][sc]) continue;
        if (refresh_pbr_has_finish[rank][sc]) continue;
        if (bankStates[bank_tmp].en_refresh_pb && bankStates[bank_tmp].hold_refresh_pb && PbrWeight[rank][bank + bank_pair_start].send_pbr) {
            for (size_t matgrp = 0; matgrp < NUM_MATGRPS; matgrp ++) {
                bool diff_bg_idle = true;
                for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                    unsigned bank_idx = bank_tmp + sbr_bank * pbr_bank_num;
                    if (bankStates[bank_idx].state->currentBankState != Idle || issue_state[bank_idx]) {
                        diff_bg_idle = false;
                    }
                }
                if (arb_enable && ((now() + 1) >= bankStates[bank_tmp].state->nextPerBankRefresh)
                        && ((tFAWCountdown[rank].size() < 4 && sc==0) 
                        ||(tFAWCountdown_sc1[rank].size() < 4 && sc==1)) && diff_bg_idle) {    //todo: revise for e-mode
                    funcState[rank].wakeup = true;
//                    if (RankState[rank].lp_state == IDLE) {
                    if (RankState[rank].lp_state == IDLE && even_cycle) {    //every other command, even;
                        Cmd *c = new Cmd;
                        c->state = working;
                        c->cmd_type = PER_BANK_REFRESH_CMD;
                        c->force_pbr = true;
                        c->bank = bankStates[bank_tmp].bank;
                        c->rank = bankStates[bank_tmp].rank;
                        c->group = bankStates[bank_tmp].group;
//                        c->channel = sc;
                        c->cmd_source = 2;
                        c->task = 0xFFFFFFFFFFFFFFE;
                        c->bankIndex = bankStates[bank_tmp].bankIndex;
                        CmdQueue.push_back(c);
                        for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                            unsigned bank_idx = bank_tmp + sbr_bank * pbr_bank_num;
                            bankStates[bank_idx].hold_refresh_pb = false;
                        }
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- REQ :: PBR IDLE, rank="<<rank<<", bank="<<bank_tmp<<", sc="<<sc<<endl);
                        }
                        return;
                    } else {
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- REQ_LP :: hold FPBR in lp, rank="<<rank<<", sc="<<sc<<", bank="<<bank_tmp<<endl);
                        }
                    }
                } else {
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- PBR :: rank"<<rank<<", bank="<<bank_tmp<<", arb_enable="<<arb_enable<<", sc="<<sc<<endl);
                    }
                    return;
                }
            }
        }
    }
}


void MemoryController::enh_per_bank_refresh(unsigned rank, unsigned sc) {
    if (refreshALL[rank][sc].refresh_cnt == 0 || refreshALL[rank][sc].refreshWaiting ||
            RankState[rank].lp_state == ASREFE || RankState[rank].lp_state == ASREF ||
            RankState[rank].lp_state == ASREFX || RankState[rank].lp_state == SRPDE ||
            RankState[rank].lp_state == SRPD || RankState[rank].lp_state == SRPDLP ||
            RankState[rank].lp_state == SRPDX) return;
    if (now() < sbr_gap_cnt[rank]) return;       //todo: revise for e-mode


    unsigned bank_start = sc * (NUM_BANKS/sc_num);
//    unsigned bank_pair_start = sc * pbr_bank_num; 
    bool have_bank_refresh = false; // send pbr one by one if set true
    for (size_t bank = 0; bank < NUM_BANKS/sc_num; bank ++) {
        unsigned bank_tmp = rank * NUM_BANKS + bank + bank_start;
        if (IS_HBM2E || IS_HBM3 || PBR_PARA_EN) {
            if (!refreshPerBank[bank_tmp].refreshWaiting) continue;
        } else {
            if (!refreshPerBank[bank_tmp].refreshWaiting && !refreshPerBank[bank_tmp].refreshing) continue;
        }
        have_bank_refresh = true;
        break;
    }

    if (!have_bank_refresh) {

        for (size_t i = 0; i < pbr_sb_group_num; i++) {   //todo: revise for e-mode
            SbGroupWeight[rank][i].bank_pair_cnt = 0;
// todo: revise            SbGroupWeight[rank][i].send_pbr = false;
        }

        for (size_t i = 0; i < NUM_BANKS; i++) {   //todo: revise for e-mode 
            PbrWeight[rank][i].bank_pair_cnt = 0;
            PbrWeight[rank][i].send_pbr = false;
        }

        
        vector<pbr_weight> SbWeightOrder;
        // calculate weight for diffrent groups, in each of which banks have same ba1ba0;
        for (size_t bank = 0; bank < pbr_sb_group_num; bank ++) {
            unsigned has_bank_open = 0;
            unsigned has_cmd_vld = 0;
            unsigned bank_idle = pbr_sb_num;
            unsigned bank_cmd_cnt = 0;
            for (size_t sb_bank = 0; sb_bank < pbr_sb_num; sb_bank ++) {
                unsigned bank_tmp = rank * NUM_BANKS + bank  + sb_bank * pbr_sb_num;    //todo: revise for e-mode 
                if (bankStates[bank_tmp].state->currentBankState == RowActive) {
                    bank_idle --;
                    if (bank_idle < 2) {
                        has_bank_open = 1;
                        break;
                    }
                }
            }
            for (size_t sbr_bank = 0; sbr_bank < pbr_sb_num; sbr_bank ++) {
                unsigned bank_tmp = rank * NUM_BANKS + bank + sbr_bank * pbr_sb_num;    //todo: revise for e-mode
                if (bank_cnt[bank_tmp] > 0) {
                    bank_cmd_cnt ++;
                    if (bank_cmd_cnt >2) {
                        has_cmd_vld = 1;
                        break;
                    }
                }
            }

            if (SBR_WEIGHT_MODE == 0) SbGroupWeight[rank][bank].bank_pair_cnt = has_bank_open;
            else if (SBR_WEIGHT_MODE == 1) SbGroupWeight[rank][bank].bank_pair_cnt = has_cmd_vld;
            else if (SBR_WEIGHT_MODE == 2) SbGroupWeight[rank][bank].bank_pair_cnt = has_bank_open + has_cmd_vld;
            else if (SBR_WEIGHT_MODE == 3) SbGroupWeight[rank][bank].bank_pair_cnt = has_bank_open + has_cmd_vld * 2;
            else { // added for SBR_WEIGHT_MODE=4/5, select pbr ba group with least/most cmd
                for (size_t sb_bank = 0; sb_bank < pbr_sb_num; sb_bank ++) {
                    unsigned bankIndex = rank * NUM_BANKS + bank + sb_bank * pbr_sb_num; 
                    SbGroupWeight[rank][bank].bank_pair_cnt += bank_cnt[bankIndex];
                }
            };

            SbGroupWeight[rank][bank].bagroup = bank;

            //reorder the priority of 4 groups: ba1ba0: (00,01,10,11)
            unsigned pre_bank_pair_cnt = 0;
            
            // ba1ba0:00
            if (bank == 0) {
                SbWeightOrder.push_back(SbGroupWeight[rank][bank]);
            }

            //reorder from small to large
            for (size_t bank_pre = 0; bank_pre < bank; bank_pre ++) {
                if ((SbGroupWeight[rank][bank].bank_pair_cnt < SbWeightOrder[bank_pre].bank_pair_cnt) && 
                    (SbGroupWeight[rank][bank].bank_pair_cnt) >= pre_bank_pair_cnt) {
                    SbWeightOrder.insert(SbWeightOrder.begin()+bank_pre, SbGroupWeight[rank][bank]);
                    break;
                }
                if (bank == (bank_pre + 1)) {
                    SbWeightOrder.push_back(SbGroupWeight[rank][bank]);
                }
                pre_bank_pair_cnt = SbWeightOrder[bank_pre].bank_pair_cnt; 
            }

            if (SBR_WEIGHT_MODE == 5) {     // reorder from large to small
                reverse(SbWeightOrder.begin(), SbWeightOrder.end());
            }

        }

        // calculate weight for bank pairs with same ba1ba0
        // 4 different groups: [0,4,8,12], [1,5,9,13], [2,6,10,14], [3,7,11,15]
        // order of SbWeight : (0,4),(0,8),(0,12),(4,8),(4,12),(8,12),(1,5),(1,9)......
        unsigned sbweight_idx = 0;
        for (size_t bank = 0; bank < pbr_sb_group_num; bank ++) {
            for (size_t sb_bank = 0 ; sb_bank < pbr_sb_num; sb_bank ++) {
                for (size_t sb_bank_tmp = (sb_bank + 1) ; sb_bank_tmp < pbr_sb_num; sb_bank_tmp ++) {
                    unsigned has_bank_open_sb = 0;
                    unsigned has_cmd_vld_sb = 0;
                    unsigned fst_bank_tmp = rank * NUM_BANKS + bank + sb_bank * pbr_sb_num;          // one bank of a bank pair 
                    unsigned lst_bank_tmp = rank * NUM_BANKS + bank + sb_bank_tmp * pbr_sb_num;      // another bank of a bank pair
                    if (bankStates[fst_bank_tmp].state->currentBankState == RowActive || bankStates[lst_bank_tmp].state->currentBankState == RowActive) {
                        has_bank_open_sb = 1;
                    }
                    if (bank_cnt[fst_bank_tmp] > 0 || bank_cnt[lst_bank_tmp] > 0 ) {
                        has_cmd_vld_sb = 1;
                    }
                    if (SBR_WEIGHT_MODE == 0) SbWeight[rank][sbweight_idx].bank_pair_cnt = has_bank_open_sb;
                    else if (SBR_WEIGHT_MODE == 1) SbWeight[rank][sbweight_idx].bank_pair_cnt = has_cmd_vld_sb;
                    else if (SBR_WEIGHT_MODE == 2) SbWeight[rank][sbweight_idx].bank_pair_cnt = has_bank_open_sb + has_cmd_vld_sb;
                    else if (SBR_WEIGHT_MODE == 3) SbWeight[rank][sbweight_idx].bank_pair_cnt = has_bank_open_sb + has_cmd_vld_sb * 2;
                    else {  // added for SBR_WEIGHT_MODE=4/5, select bank pair with least/most cmd
                        SbWeight[rank][sbweight_idx].bank_pair_cnt = bank_cnt[fst_bank_tmp] + bank_cnt[lst_bank_tmp];  
                    }
                    
                    SbWeight[rank][sbweight_idx].fst_bankIndex = fst_bank_tmp; 
                    SbWeight[rank][sbweight_idx].lst_bankIndex = lst_bank_tmp; 
                    SbWeight[rank][sbweight_idx].bagroup = bank;

                    // added for SBR_WEIGHT_ENH_MODE=3/4, select bank pair with least/most cmd
                    // based on SBR_WEIGHT_MODE
                    if (SBR_WEIGHT_ENH_MODE == 3 || SBR_WEIGHT_ENH_MODE == 4) {
                        bank_pair_cmd_cnt[rank][sbweight_idx] = bank_cnt[fst_bank_tmp] + bank_cnt[lst_bank_tmp];
                    }

                    sbweight_idx ++;

//                    DEBUG(now()<<" sbweight, bank pair="<<sbweight_idx<<" fst_bankIndex="<<fst_bank_tmp<<" lst_bankIndex="<<lst_bank_tmp);
                }
            }
        }
//        DEBUG(now()<<" arb, SbWeight_size="<<SbWeight[rank].size()<<" rank="<<rank);

#if 1
        if (refreshALL[rank][sc].refresh_cnt < PBR_PSTPND_LEVEL) {
            for (size_t i = 0; i < NUM_BANKS; i++) {
                unsigned bank = rank * NUM_BANKS + i;
                if (bank_cnt[bank] > 0) {
                    for (size_t j=0; j < SbWeight[rank].size(); j++) {
                        if ((i==SbWeight[rank][j].fst_bankIndex)||(i==SbWeight[rank][j].lst_bankIndex))
                        SbWeight[rank][j].bank_pair_cnt = 0xFFFFFFFF;
                    }
                }
            }
        }
#endif
        // todo: keep unchanged in enhanced DBR? 
        if (WRITE_BUFFER_ENABLE) {
            if (GBUF_RCMD_BLOCK_PBR && refreshALL[rank][sc].refresh_cnt < PBR_PSTPND_LEVEL) {
                for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
                    unsigned bank_tmp = rank * NUM_BANKS + bank;
                    if (!wb->rcmd_bank_state[bank_tmp]) continue;
                    PbrWeight[rank][bank_tmp % pbr_bank_num].bank_pair_cnt = 0xFFFFFFFF;
                }
            }
        }

#if 0
        if (DMC_V590) {
            for (size_t bank = 0; bank < pbr_bank_num * NUM_MATGRPS; bank ++) {
                if (!PbrWeight[rank][bank].block_pbr) continue;
                PbrWeight[rank][bank].bank_pair_cnt = 0xFFFFFFFF;
            }
        }
#endif

        bool pbr_bank_left = true;
        unsigned pbr_bagroup_cnt = 0;
        if (SBR_WEIGHT_ENH_MODE==1){
            for (size_t group = 0; group < pbr_sb_group_num; group++){
                if (group == pre_enh_pbr_bagroup[rank]) {
                    for (size_t i = 0; i < pbr_sb_num; i ++){
                        unsigned bank = NUM_BANKS * rank + group + i * pbr_sb_num; 
                        if (bankStates[bank].finish_refresh_pb){
                            pbr_bagroup_cnt ++;
                            if (pbr_bagroup_cnt >= (pbr_sb_num-1)) {
                                pbr_bank_left = false;
                                break;
                            }
                        }
                    }    
                } 
            }  
        }
//        if (now()>=5260 && now()<=5558 && rank==0) {
//            DEBUG(now()<<" rank="<<rank<<" previous ref bagroup="<<pre_enh_pbr_bagroup[rank]<<" pbr_bank_left="<<pbr_bank_left<<" sub_channel="<<sub_cha);
//        }

        unsigned bank_pair_cnt_min = 0xFFFFFFFF;
        unsigned bank_pair_cnt_min_num_fst = 0xFFFFFFFF;
        unsigned bank_pair_cnt_min_num_lst = 0xFFFFFFFF;
        unsigned bagroup_cnt_min = 0xFFFFFFFF;
        unsigned bank_pair_cmd_cnt_min_num = 0xFFFFFFFF;
        bool     pre_sch_bankIndex_met = false;
        unsigned pre_sch_bank_pair_cnt = 0xFFFFFFFF;
        unsigned pre_sch_bank_pair = 0xFFFFFFFF;
        if (ENH_PBR_CEIL_EN) {  // 1 in 24, the best for enhanced pbr 
            for (size_t group = 0;  group < pbr_sb_group_num; group++) {
                for (size_t bank = 0; bank < SbWeight[rank].size(); bank++) {
                    unsigned fst_bank = SbWeight[rank][bank].fst_bankIndex;
                    unsigned lst_bank = SbWeight[rank][bank].lst_bankIndex;
                    unsigned bagroup_cnt = SbWeight[rank][bank].bagroup;
                    if (group == bagroup_cnt) {
                        if (bank_pair_cnt_min > SbWeight[rank][bank].bank_pair_cnt &&
                             (!bankStates[fst_bank].finish_refresh_pb && !bankStates[lst_bank].finish_refresh_pb) &&
                             (!DMC_V580 || SbWeight[rank][bank].bank_pair_cnt != 0xFFFFFFFF)) {
                            bank_pair_cnt_min = SbWeight[rank][bank].bank_pair_cnt;
                            bank_pair_cnt_min_num_fst = fst_bank;
                            bank_pair_cnt_min_num_lst = lst_bank;
                            bagroup_cnt_min = bagroup_cnt;
                        }
                        // added for SBR_WEIGHT_ENH_MODE=2
                        if ((fst_bank==pre_sch_bankIndex[rank]||lst_bank==pre_sch_bankIndex[rank])&&(SBR_WEIGHT_ENH_MODE==2) && 
                                (!bankStates[fst_bank].finish_refresh_pb && !bankStates[lst_bank].finish_refresh_pb) &&
                                (!DMC_V580 || SbWeight[rank][bank].bank_pair_cnt != 0xFFFFFFFF)) {
                            pre_sch_bankIndex_met = true;
                            pre_sch_bank_pair_cnt = SbWeight[rank][bank].bank_pair_cnt; 
                            pre_sch_bank_pair     = bank; 
                        }
                    }
                }
            }
        } else {      //real for RTL: 4 in 1 && 6 in 1
            for (size_t group = 0;  group < pbr_sb_group_num; group++) {     // 4 in 1
                unsigned bank_pair_group = SbWeightOrder[group].bagroup;
//                if (now() < 6000) {
//                    DEBUG(now()<<" sub_channel="<<sub_cha<<" rank="<<rank<<" bank_pair_group="<<bank_pair_group<<", bank_pair_cnt="<<SbWeightOrder[group].bank_pair_cnt);
//                }
                if (bagroup_cnt_min != 0xFFFFFFFF && bagroup_cnt_min != bank_pair_group) break;   // standard 4in1 && 6in1
                if (pbr_bank_left && bank_pair_group != pre_enh_pbr_bagroup[rank] 
                        && pre_enh_pbr_bagroup[rank] != 0xFFFFFFFF && SBR_WEIGHT_ENH_MODE==1) continue;     // same bank pair group priority, added for SBR_WEIGHT_ENH_MODE=1
                for (size_t bank = 0; bank < SbWeight[rank].size(); bank++) {
                    unsigned fst_bank = SbWeight[rank][bank].fst_bankIndex;
                    unsigned lst_bank = SbWeight[rank][bank].lst_bankIndex;
                    unsigned bagroup_cnt = SbWeight[rank][bank].bagroup;
                    if (bank_pair_group == bagroup_cnt) {    // 6 in 1
                        if (bank_pair_cnt_min > SbWeight[rank][bank].bank_pair_cnt &&
                             (!bankStates[fst_bank].finish_refresh_pb && !bankStates[lst_bank].finish_refresh_pb) &&
                             (!DMC_V580 || SbWeight[rank][bank].bank_pair_cnt != 0xFFFFFFFF) &&
                             (SBR_WEIGHT_MODE!=5)) {
                            bank_pair_cnt_min = SbWeight[rank][bank].bank_pair_cnt;
                            bank_pair_cnt_min_num_fst = fst_bank;
                            bank_pair_cnt_min_num_lst = lst_bank;
                            bagroup_cnt_min = bagroup_cnt;
                            bank_pair_cmd_cnt_min_num = bank;
                        }

                        //added for SBR_WEIGHT_MODE=5, select bank pair with most cmd
                        if ((bank_pair_cnt_min < SbWeight[rank][bank].bank_pair_cnt || bank_pair_cnt_min == 0xFFFFFFFF) &&
                             (!bankStates[fst_bank].finish_refresh_pb && !bankStates[lst_bank].finish_refresh_pb) &&
                             (!DMC_V580 || SbWeight[rank][bank].bank_pair_cnt != 0xFFFFFFFF) &&
                             (SBR_WEIGHT_MODE==5)) {
                            bank_pair_cnt_min = SbWeight[rank][bank].bank_pair_cnt;
                            bank_pair_cnt_min_num_fst = fst_bank;
                            bank_pair_cnt_min_num_lst = lst_bank;
                            bagroup_cnt_min = bagroup_cnt;
                            bank_pair_cmd_cnt_min_num = bank;
                        }

                        // added for SBR_WEIGHT_ENH_MODE=2
                        if ((fst_bank==pre_sch_bankIndex[rank]||lst_bank==pre_sch_bankIndex[rank])&&(SBR_WEIGHT_ENH_MODE==2) && 
                                (!bankStates[fst_bank].finish_refresh_pb && !bankStates[lst_bank].finish_refresh_pb) &&
                                (!DMC_V580 || SbWeight[rank][bank].bank_pair_cnt != 0xFFFFFFFF)) {
                            pre_sch_bankIndex_met = true;
                            pre_sch_bank_pair_cnt = SbWeight[rank][bank].bank_pair_cnt; 
                            pre_sch_bank_pair     = bank; 
                        }
                    }
                }
            }
        }
        
        //added for SBR_WEIGHT_ENH_MODE=2, most recently scheduled cmd priority
        if (SBR_WEIGHT_ENH_MODE==2 && pre_sch_bankIndex_met &&
                pre_sch_bank_pair_cnt!=0xFFFFFFFF && pre_sch_bank_pair!=0xFFFFFFFF){
            if (bank_pair_cnt_min >= pre_sch_bank_pair_cnt) {
                bank_pair_cnt_min = SbWeight[rank][pre_sch_bank_pair].bank_pair_cnt;
                bank_pair_cnt_min_num_fst = SbWeight[rank][pre_sch_bank_pair].fst_bankIndex;
                bank_pair_cnt_min_num_lst = SbWeight[rank][pre_sch_bank_pair].lst_bankIndex;
                bagroup_cnt_min = SbWeight[rank][pre_sch_bank_pair].bagroup ;
//                DEBUG(now()<<" scheduled priority affects"<<" rank="<<rank<<" bankIndex="<<pre_sch_bankIndex[rank]<<" sub channel="<<sub_cha);
            }       
        }

        //added for SBR_WEIGHT_ENH_MODE=3/4, select bank pair with least/most cmd
        for (size_t bank = 0; bank < SbWeight[rank].size(); bank++) {
            unsigned fst_bank = SbWeight[rank][bank].fst_bankIndex;
            unsigned lst_bank = SbWeight[rank][bank].lst_bankIndex;
            unsigned bagroup_cnt = SbWeight[rank][bank].bagroup;
            if (bank_pair_cmd_cnt_min_num != 0xFFFFFFFF) {
                if (bagroup_cnt_min == bagroup_cnt && bagroup_cnt_min!=0xFFFFFFFF && bank_pair_cnt_min == SbWeight[rank][bank].bank_pair_cnt && 
                        (!bankStates[fst_bank].finish_refresh_pb && !bankStates[lst_bank].finish_refresh_pb)){
                    if ((bank_pair_cmd_cnt[rank][bank] < bank_pair_cmd_cnt[rank][bank_pair_cmd_cnt_min_num] && SBR_WEIGHT_ENH_MODE==3) || 
                            (bank_pair_cmd_cnt[rank][bank] > bank_pair_cmd_cnt[rank][bank_pair_cmd_cnt_min_num] && SBR_WEIGHT_ENH_MODE==4)) {
                        bank_pair_cnt_min = SbWeight[rank][bank].bank_pair_cnt;
                        bank_pair_cnt_min_num_fst = fst_bank;
                        bank_pair_cnt_min_num_lst = lst_bank;
                        bagroup_cnt_min = bagroup_cnt;
                        bank_pair_cmd_cnt_min_num = bank;
                    }

                }
            }
        }

        if ((bank_pair_cnt_min_num_fst != 0xFFFFFFFF) && (bank_pair_cnt_min_num_lst != 0xFFFFFFFF) && (bagroup_cnt_min != 0xFFFFFFFF) ) {
//            DEBUG(now()<<" min_fst_bankindex="<<bank_pair_cnt_min_num_fst<<" min_lst_bankindex="<<bank_pair_cnt_min_num_lst<<" min_bagroup="<<bagroup_cnt_min);
            unsigned min_fst_bank = bank_pair_cnt_min_num_fst % NUM_BANKS;
            unsigned min_lst_bank = bank_pair_cnt_min_num_lst % NUM_BANKS;
            PbrWeight[rank][min_fst_bank].send_pbr = true;
            PbrWeight[rank][min_lst_bank].send_pbr = true;
            PbrWeight[rank][min_fst_bank].bagroup = bagroup_cnt_min;
            PbrWeight[rank][min_lst_bank].bagroup = bagroup_cnt_min;
            PbrWeight[rank][min_fst_bank].fst_bankIndex = bank_pair_cnt_min_num_fst;
            PbrWeight[rank][min_fst_bank].lst_bankIndex = bank_pair_cnt_min_num_lst;
            PbrWeight[rank][min_lst_bank].fst_bankIndex = bank_pair_cnt_min_num_lst;
            PbrWeight[rank][min_lst_bank].lst_bankIndex = bank_pair_cnt_min_num_fst;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- ENHANCED PBR_WEIGHT :: rank="<<rank<<" fst_bank="
                        <<bank_pair_cnt_min_num_fst<<" lst_bank="
                        <<bank_pair_cnt_min_num_lst<<" ba group="<<bagroup_cnt_min<<" win the arbitration!"<<endl);
            }
            // bankIndex check
            if (bank_pair_cnt_min_num_fst == bank_pair_cnt_min_num_lst) {
                ERROR(setw(10)<<now()<<" Not allowed same bank in a bank pair, bagroup="<<bagroup_cnt_min
                        <<" fst_bankIndex"<<bank_pair_cnt_min_num_fst<<" lst_bankIndex"<<bank_pair_cnt_min_num_lst);
                assert(0);
            } else if (bank_pair_cnt_min_num_fst > bank_pair_cnt_min_num_lst) {
                if (((bank_pair_cnt_min_num_fst-bank_pair_cnt_min_num_lst)%pbr_sb_num) != 0) {
                    ERROR(setw(10)<<now()<<" Non-4x Diff between banks in a bank pair, bagroup="<<bagroup_cnt_min
                            <<" fst_bankIndex"<<bank_pair_cnt_min_num_fst<<" lst_bankIndex"<<bank_pair_cnt_min_num_lst);
                    assert(0);
                }
            } else if (bank_pair_cnt_min_num_lst > bank_pair_cnt_min_num_fst) {
                if (((bank_pair_cnt_min_num_lst-bank_pair_cnt_min_num_fst)%pbr_sb_num) != 0) {
                    ERROR(setw(10)<<now()<<" Non-4x Diff between banks in a bank pair, bgroup="<<bagroup_cnt_min
                            <<" fst_bankIndex"<<bank_pair_cnt_min_num_fst<<" lst_bankIndex"<<bank_pair_cnt_min_num_lst);
                    assert(0);
                }
            }
            //bagroup and bankIndex combined check
            if (((bank_pair_cnt_min_num_lst % pbr_sb_num)!=bagroup_cnt_min) ||  ((bank_pair_cnt_min_num_fst % pbr_sb_num!=bagroup_cnt_min))) {
                ERROR(setw(10)<<now()<<" Wrong bagroup, bgroup="<<bagroup_cnt_min
                        <<" fst_bankIndex"<<bank_pair_cnt_min_num_fst<<" lst_bankIndex"<<bank_pair_cnt_min_num_lst);
                assert(0);
            }
        }

    }

    refresh_cnt_pb[rank][sc] = 0;
    for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
        unsigned bank_tmp = rank * NUM_BANKS + bank;
        if (bankStates[bank_tmp].finish_refresh_pb) refresh_cnt_pb[rank][sc] ++;    //2X refresh commands
        else forceRankBankIndex[rank][sc] = bank;

        if (SBR_REQ_MODE == 0) {
            if (bank == NUM_BANKS - 1) {
                if (refresh_cnt_pb[rank][sc] == NUM_BANKS) {
                    refresh_pbr_has_finish[rank][sc] = true;
                    force_pbr_refresh[rank][sc] = false;
                    pre_enh_pbr_bagroup[rank] = 0xFFFFFFFF;
                    for (size_t bank_sbr = 0; bank_sbr < NUM_BANKS; bank_sbr ++) {
                        unsigned bank_idx = rank * NUM_BANKS + bank_sbr;
                        bankStates[bank_idx].finish_refresh_pb = false;
                    }
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- ENHANCED PBR :: RANK "<<rank<<" has been refreshed by pbr"<<endl);
                    }
                } else {
                    force_pbr_refresh[rank][sc] = true;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- ENHANCED PBR :: Force pbr rank "<<rank<<" has no idle bank not refreshed"<<endl);
                    }
                }
            }
        } else {
            if (bank == NUM_BANKS - 1) {
                if (refresh_cnt_pb[rank][sc] == NUM_BANKS) {
                    refresh_pbr_has_finish[rank][sc] = true;
                    force_pbr_refresh[rank][sc] = false;
                    pre_enh_pbr_bagroup[rank] = 0xFFFFFFFF;
                    for (size_t bank_sbr = 0; bank_sbr < NUM_BANKS; bank_sbr ++) {
                        unsigned bank_idx = rank * NUM_BANKS + bank_sbr;
                        bankStates[bank_idx].finish_refresh_pb = false;
                    }
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- ENHANCED PBR :: RANK "<<rank<<" has been refreshed by pbr"<<endl);
                    }
                } else if ((refresh_cnt_pb[rank][sc]) >= (2*SBR_FRCST_NUM)) {
                    force_pbr_refresh[rank][sc] = true;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- ENHANCED PBR :: Force pbr for the left banks in RANK "<<rank<<endl);
                    }
                }
            }
        }
    }

    if ((refresh_cnt_pb[rank][sc] % 2)!=0 || (refresh_cnt_pb[rank][sc] > 16)){
        ERROR(setw(10)<<now()<<" Pbr Cnt Not Even or Exceed 16, refresh cnt(x2)="<<refresh_cnt_pb[rank][sc]);
        assert(0);
        
    }

    for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
        unsigned bank_tmp = rank * NUM_BANKS + bank;
        if (!bankStates[bank_tmp].finish_refresh_pb) {
            bankStates[bank_tmp].en_refresh_pb = true;
            bankStates[bank_tmp].hold_refresh_pb = true;
        }
    }

    enh_send_pb_precharge(rank, sc);
    enh_send_pb_refresh(rank, sc);
}

void MemoryController::enh_send_pb_precharge(unsigned rank, unsigned sc) {
    // unrefreshed bank is 2 left, force pbr
    bool have_bank_refresh = false; // send pbr one by one if set true
    //if (!IS_HBM2E && !IS_HBM3 ) {
    if (!IS_HBM2E && !IS_HBM3 && !PBR_PARA_EN) {
        for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
            unsigned bank_tmp = rank * NUM_BANKS + bank;
            if (refreshPerBank[bank_tmp].refreshing){
                have_bank_refresh = true;
                break;
            }
        }
    }
    
    vector<unsigned> bp_idx;
    for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
        unsigned bank_tmp = rank * NUM_BANKS + bank;
        unsigned fst_bank_idx = PbrWeight[rank][bank].fst_bankIndex; 
        unsigned lst_bank_idx = PbrWeight[rank][bank].lst_bankIndex;
//        unsigned bagroup = PbrWeight[rank][bank].bagroup; 

        if (refresh_pbr_has_finish[rank][sc]) continue;
        
        // bp_idx size check
        if (bp_idx.size()!=0) {
            ERROR(setw(10)<<now()<<" after clear, Vector bp_idx size check failed, size="<<bp_idx.size()<<" bank="<<bank_tmp
                    <<" fst_bankIndex="<<fst_bank_idx<<" lst_bankIndex="<<lst_bank_idx);
            assert(0);
        }
        
        bp_idx.push_back(fst_bank_idx);
        bp_idx.push_back(lst_bank_idx);
        
        // bp_idx size check
        if (bp_idx.size()!=2) {
            for(size_t i = 0; i<bp_idx.size();i++){
                DEBUG(now()<<" bankIndex="<<bp_idx[i]);
            }
            ERROR(setw(10)<<now()<<" Vector bp_idx size check failed, size="<<bp_idx.size()<<" rank="<<rank
                    <<" bank="<<bank_tmp<<" fst_bankIndex="<<fst_bank_idx<<" lst_bankIndex="<<lst_bank_idx);
            assert(0);
        }

//        if (refresh_pbr_has_finish[rank]) continue;
        if (force_pbr_refresh[rank][sc] && !bankStates[bank_tmp].finish_refresh_pb
                && !have_bank_refresh && PbrWeight[rank][bank].send_pbr) {

            // bankIndex check
            if (fst_bank_idx == lst_bank_idx) {
                ERROR(setw(10)<<now()<<" Not allowed same bank in a bank pair, bank="<<bank_tmp
                        <<" fst_bankIndex"<<fst_bank_idx<<" lst_bankIndex"<<lst_bank_idx);
                assert(0);
            } else if (fst_bank_idx > lst_bank_idx) {
                if (((fst_bank_idx-lst_bank_idx)%pbr_sb_num) != 0) {
                    ERROR(setw(10)<<now()<<" Non-4x Diff between banks in a bank pair, bank="<<bank_tmp
                            <<" fst_bankIndex"<<fst_bank_idx<<" lst_bankIndex"<<lst_bank_idx);
                    assert(0);
                }
            } else if (lst_bank_idx > fst_bank_idx) {
                if (((lst_bank_idx-fst_bank_idx)%pbr_sb_num) != 0) {
                    ERROR(setw(10)<<now()<<" Non-4x Diff between banks in a bank pair, bank="<<bank_tmp
                            <<" fst_bankIndex"<<fst_bank_idx<<" lst_bankIndex"<<lst_bank_idx);
                    assert(0);
                }
            }

            for (size_t matgrp = 0; matgrp < NUM_MATGRPS; matgrp ++) {
                if (arb_enable) {
                    for(size_t i = 0; i<bp_idx.size(); i++) {
                        unsigned bank_idx = bp_idx[i];
                        if (!refreshPerBank[bank_idx].refreshWaiting) {
                            refreshPerBank[bank_idx].refreshWaiting = true;
                            refreshPerBank[bank_idx].refreshing = false;
                            refreshPerBank[bank_idx].refreshWaitingPre = false;
                            pbr_hold_pre[rank] = true;
                            pbr_hold_pre_time[rank] = now() + 6;
                        }
                    }

                    bool ready_send_pbr = true;
                    for(size_t i = 0; i<bp_idx.size(); i++) {
                        unsigned bank_idx = bp_idx[i];
                        if ((bankStates[bank_idx].state->currentBankState != Idle || (now() + 1) < bankStates[bank_idx].state->nextPerBankRefresh) || issue_state[bank_idx]) { 
                            ready_send_pbr = false;
                            break;
                        }
                    }
                    
                    if (ready_send_pbr && ((tFAWCountdown[rank].size() < 4 && sc==0) 
                            ||(tFAWCountdown_sc1[rank].size() < 4 && sc==1)) 
                            && (!ASREF_ENABLE || (ASREF_ENABLE && RankState[rank].asref_cnt != 0))) {
                        funcState[rank].wakeup = true;
//                        if (RankState[rank].lp_state == IDLE) {
                        if (RankState[rank].lp_state == IDLE && even_cycle) {    // every other command, even;
                            
                            //todo: info of fst_bank/lst_bank may be transfered to Cmd -> changed already
                            Cmd *c = new Cmd;
                            c->state = working;
                            c->cmd_type = PER_BANK_REFRESH_CMD;
                            c->force_pbr = true;
                            c->bank = bankStates[bank_tmp].bank;
                            c->rank = bankStates[bank_tmp].rank;
                            c->group = bankStates[bank_tmp].group;
                            c->cmd_source = 2;
                            c->task = 0xFFFFFFFFFFFFFFB;
                            c->bankIndex = bankStates[bank_tmp].bankIndex;
                            c->fst_bankIndex = fst_bank_idx;
                            c->lst_bankIndex = lst_bank_idx;


                            bankStates[fst_bank_idx].hold_refresh_pb = false;
                            bankStates[lst_bank_idx].hold_refresh_pb = false;
                            
                            CmdQueue.push_back(c);
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- REQ :: force enhanced pbr, rank="<<rank<<", fst_bank="<<fst_bank_idx<<", lst_bank="<<lst_bank_idx<<endl);
                            }
                            return;
                        } else {
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- REQ_LP :: hold enhanced FPBR in lp, rank="<<rank<<", fst_bank="<<fst_bank_idx<<", lst_bank="<<lst_bank_idx<<endl);
                            }
                        }
                    } else if (tFPWCountdown[rank].size() < 4) {    //stop here 0717
                        bool sb_ready_send_pre = true;
                        for (size_t j = 0; j < bp_idx.size(); j ++) {
                            unsigned bank_index = bp_idx[j]; 
                            if (bankStates[bank_index].state->currentBankState == RowActive) { 
                                refreshPerBank[bank_index].refreshWaiting = true;
                                refreshPerBank[bank_index].refreshing = false;
                                refreshPerBank[bank_index].refreshWaitingPre = false;
                                pbr_hold_pre[rank] = true;
                            }
                        }

                        if (((now() + 1) < bankStates[bank_tmp].state->nextPrecharge || issue_state[bank_tmp]) && bankStates[bank_tmp].state->currentBankState == RowActive) { 
                            sb_ready_send_pre = false;
                    
                        }

                        if (sb_ready_send_pre && (bankStates[bank_tmp].state->currentBankState == RowActive)) {
                            funcState[rank].wakeup = true;
//                                    if (RankState[rank].lp_state == IDLE) {
                            if (RankState[rank].lp_state == IDLE && even_cycle) {     //every other command, evan;
                                Cmd *c = new Cmd;
                                c->state = working;
                                if (IS_LP6) {
                                    //todo: fst_bank/lst_bank info may be transfered? -> changed already
                                    c->cmd_type = PRECHARGE_PB_CMD;
                                    c->bank = bankStates[bank_tmp].bank;
                                    c->rank = bankStates[bank_tmp].rank;
                                    c->group = bankStates[bank_tmp].group;
                                    c->bankIndex = bankStates[bank_tmp].bankIndex;
                                    c->fst_bankIndex = fst_bank_idx;
                                    c->lst_bankIndex = lst_bank_idx;
                                    bankStates[bank_tmp].hold_refresh_pb = true;
                                    bankStates[bank_tmp].finish_refresh_pb = false;
                                    pbr_hold_pre_time[rank] = now() + 4;
                                } else {
                                    ERROR(setw(10)<<now()<<" Ehanced DBR Only Applies to LP6 ");
                                    assert(0);
                                }
                                c->force_pbr = true;
                                c->cmd_source = 2;
                                c->task = 0xFFFFFFFFFFFFFFD;
                                CmdQueue.push_back(c);
                                if (DEBUG_BUS) {
                                    PRINTN(setw(10)<<now()<<" -- REQ ::  force ENHANCED PRECHARGE, rank="<<rank<<", bank="<<bank_tmp<<endl);
                                }
                                return;
                            } else {
                                if (DEBUG_BUS) {
                                    PRINTN(setw(10)<<now()<<" -- REQ_LP :: hold ENAHNCED PRECHARGE PB/SB in lp"<<" state, rank"<<rank<<", bank="<<bank_tmp<<endl);
                                }
                            }
                        }
                    }
                }
            }
        }
        bp_idx.clear();
        
    }
}

void MemoryController::enh_send_pb_refresh(unsigned rank, unsigned sc) {
    // find Idle bank send pb refresh
    for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
        unsigned bank_tmp = rank * NUM_BANKS + bank;
        unsigned fst_bank_idx = PbrWeight[rank][bank].fst_bankIndex; 
        unsigned lst_bank_idx = PbrWeight[rank][bank].lst_bankIndex;
        
        if (force_pbr_refresh[rank][sc]) continue;
        if (refresh_pbr_has_finish[rank][sc]) continue;
        if (bankStates[bank_tmp].en_refresh_pb && bankStates[bank_tmp].hold_refresh_pb && PbrWeight[rank][bank].send_pbr) {
            for (size_t matgrp = 0; matgrp < NUM_MATGRPS; matgrp ++) {
                bool diff_bg_idle = true;
                if (bankStates[fst_bank_idx].state->currentBankState != Idle || 
                        bankStates[lst_bank_idx].state->currentBankState != Idle || issue_state[fst_bank_idx] || issue_state[lst_bank_idx]) {
                    diff_bg_idle = false;
                }
                
                if (arb_enable && ((now() + 1) >= bankStates[fst_bank_idx].state->nextPerBankRefresh)
                        && ((now() + 1) >= bankStates[lst_bank_idx].state->nextPerBankRefresh) 
                        && ((tFAWCountdown[rank].size() < 4 && sc==0) 
                        ||(tFAWCountdown_sc1[rank].size() < 4 && sc==1)) && diff_bg_idle) {
                    funcState[rank].wakeup = true;
//                    if (RankState[rank].lp_state == IDLE) {
                    if (RankState[rank].lp_state == IDLE && even_cycle) {    //every other command, even;
                        Cmd *c = new Cmd;
                        c->state = working;
                        c->cmd_type = PER_BANK_REFRESH_CMD;
                        c->force_pbr = true;
                        c->bank = bankStates[bank_tmp].bank;
                        c->rank = bankStates[bank_tmp].rank;
                        c->group = bankStates[bank_tmp].group;
                        c->cmd_source = 2;
                        c->task = 0xFFFFFFFFFFFFFFE;
                        c->bankIndex = bankStates[bank_tmp].bankIndex;
                        c->fst_bankIndex = fst_bank_idx;
                        c->lst_bankIndex = lst_bank_idx;
                        CmdQueue.push_back(c);
                        
                        bankStates[fst_bank_idx].hold_refresh_pb = false;
                        bankStates[lst_bank_idx].hold_refresh_pb = false;
                        
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- REQ :: ENHANCED PBR IDLE, rank="<<rank<<", fst_bank="<<fst_bank_idx<<", lst_bank"<<lst_bank_idx<<endl);
                        }
                        return;
                    } else {
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- REQ_LP :: hold ENHANCED FPBR in lp, rank="<<rank<<", fst_bank="<<fst_bank_idx<<", lst_bank"<<lst_bank_idx<<endl);
                        }
                    }
                } else {
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- ENHANCED PBR :: rank"<<rank<<", fst_bank="<<fst_bank_idx<<", lst_bank="<<lst_bank_idx<<", arb_enable="<<arb_enable<<endl);
                    }
                    return;
                }
            }
        }
    }
}

unsigned MemoryController::priority(Cmd *cmd) {
    unsigned level_ret = 0;
    switch (cmd->cmd_type) {
        case LPDDRSim::INVALID :
        case LPDDRSim::REFRESH_CMD :
        case LPDDRSim::PRECHARGE_SB_CMD :
        case LPDDRSim::PRECHARGE_PB_CMD :
        case LPDDRSim::PRECHARGE_AB_CMD : {
            if (cmd->force_pbr) level_ret = 400;
            else level_ret = 0;
            break;
        }
        case LPDDRSim::ACTIVATE1_CMD : {
            level_ret = 300;
            break;
        }
        case LPDDRSim::ACTIVATE1_DST_CMD :
        case LPDDRSim::PRECHARGE_PB_DST_CMD : {
            if (cmd->force_pbr) level_ret = 400;
            else level_ret = 200;
            break;
        }
        case LPDDRSim::ACTIVATE2_DST_CMD : {
            level_ret = 500;
            break;
        }
        case LPDDRSim::PER_BANK_REFRESH_CMD : {
            level_ret = 400;
//           level_ret = 800;
            break;
        }
        case LPDDRSim::ACTIVATE2_CMD : {
            if (IS_LP5 || IS_LP6 || IS_GD2) {
//                level_ret = 500;
                level_ret = 9999;
            } else if (IS_LP4) {
                level_ret = 9999;
            } else if (IS_DDR5 || IS_DDR4 || IS_GD1 || IS_G3D) {
                level_ret = 300;
            } else if (IS_HBM2E || IS_HBM3) {
                if(cmd->sid == PreCmd.sid) {
                    if(cmd->group == PreCmd.group) level_ret = 310;
                    else level_ret = (tCCD_R > tCCD_L) ? 320 : 300;
                } else {
                    level_ret = (tCCD_R > tCCD_L) ? 300 : 320;
                }
            }
            break;
        }
        case LPDDRSim::WRITE_CMD :
        case LPDDRSim::WRITE_P_CMD :
        case LPDDRSim::WRITE_MASK_CMD :
        case LPDDRSim::WRITE_MASK_P_CMD : {
            if (cmd->issue_size == 0) level_ret = 600;
            else level_ret = 700;
            break;
        }
        case LPDDRSim::READ_CMD :
        case LPDDRSim::READ_P_CMD : {
            if (cmd->issue_size == 0) level_ret = 600;
            else level_ret = 700;
            break;
        }
        default:
            break;
    }
    if (BG_ROTATE_EN && cmd->cmd_type == activate_cmd) level_ret -= cmd->bg_rotate_pri;
    return level_ret;
}

unsigned MemoryController::priority_pri(Cmd *cmd) {
    unsigned level_ret = cmd->pri;
    if (RCMD_BANK_ARB_EN && cmd->cmd_type >= READ_CMD && cmd->cmd_type <= READ_P_CMD) {
        if (cmd->bankIndex % NUM_BANKS == max_rcmd_bank[cmd->rank] && rw_group_state[0] != NO_GROUP) {
            if (cmd->pri > RCMD_BANK_ARB_PRI) level_ret = RCMD_BANK_ARB_PRI;
        }
    } else if (WCMD_BANK_ARB_EN && cmd->cmd_type >= WRITE_CMD && cmd->cmd_type <= WRITE_MASK_P_CMD) {
        if (cmd->bankIndex % NUM_BANKS == max_wcmd_bank[cmd->rank] && rw_group_state[0] != NO_GROUP) {
            if (cmd->pri > WCMD_BANK_ARB_PRI) level_ret = WCMD_BANK_ARB_PRI;
        }
    }
    return level_ret;
}

/***************************************************************************************************
descriptor: main scheduler,The purpose of this function is selecting the best task to perform from the queue
****************************************************************************************************/
void MemoryController::scheduler() {
    if (CmdQueue.empty()) return;

    if (RCMD_BANK_ARB_EN && rw_group_state[0] != NO_GROUP) {
        for (unsigned i = 0; i < NUM_RANKS; i ++) {
            unsigned max_cnt = 0;
            for (unsigned j = 0; j < NUM_BANKS; j ++) {
                unsigned sub_channel = j / sc_bank_num;
                if (EM_ENABLE && EM_MODE==2 && i==1 && sub_channel==1) continue;
                if (r_bank_cnt[i * NUM_BANKS + j] > max_cnt) {
                    max_cnt = r_bank_cnt[i * NUM_BANKS + j];
                    max_rcmd_bank[i] = j;
                }
            }
        }
    }

    if (WCMD_BANK_ARB_EN && rw_group_state[0] != NO_GROUP) {
        for (unsigned i = 0; i < NUM_RANKS; i ++) {
            unsigned max_cnt = 0;
            for (unsigned j = 0; j < NUM_BANKS; j ++) {
                unsigned sub_channel = j / sc_bank_num;
                if (EM_ENABLE && EM_MODE==2 && i==1 && sub_channel==1) continue;
                if (w_bank_cnt[i * NUM_BANKS + j] > max_cnt) {
                    max_cnt = w_bank_cnt[i * NUM_BANKS + j];
                    max_wcmd_bank[i] = j;
                }
            }
        }
    }

    Cmd *c = NULL;
    if (CORE_CONCURR == 1) {
        for (auto &cmd : CmdQueue) {
            if (cmd->issue_size != 0) {c = cmd; break;}
            if (c == NULL) {c = cmd; continue;}
            if (priority(cmd) > priority(c)) {
                c = cmd;
            } else if (priority(cmd) == priority(c)) {
//                if (priority_pri(cmd) < priority_pri(c)) c = cmd;
                if (priority_pri(cmd) < priority_pri(c)) {
                    c =cmd;
                } else if (priority_pri(cmd) == priority_pri(c)) {
                    if (!SLOT_FIFO) {
                        if ((cmd->cmd_type == activate_cmd) && (c->cmd_type == activate_cmd) && 
                                (cmd->bank == c->bank) && (cmd->timeAdded < c->timeAdded)) {
                            c = cmd;
                        }
                    }
                }
            }
        }
    } else {
        vector <bool> bank_has_rw;
        vector <bool> rank_has_rw;
        bank_has_rw.clear();
        bank_has_rw.reserve(NUM_BANKS * NUM_RANKS);
        rank_has_rw.clear();
        rank_has_rw.reserve(NUM_RANKS);
        for (size_t i = 0; i < NUM_BANKS * NUM_RANKS; i ++) {
            bank_has_rw.push_back(false);
        }
        for (size_t i = 0; i < NUM_RANKS; i ++) {
            rank_has_rw.push_back(false);
        }

        for (auto &cmd : CmdQueue) {
            if (cmd->cmd_type >= WRITE_CMD && cmd->cmd_type <= READ_P_CMD) {
                bank_has_rw[cmd->bankIndex] = true;
                rank_has_rw[cmd->rank] = true;
            }
        }

        uint8_t erase_cnt = 0;
        uint8_t que_size = CmdQueue.size();
        for (size_t i = 0; i < que_size; i ++) {
            uint8_t ecnt = i - erase_cnt;
            if ((CmdQueue[ecnt]->cmd_type == PRECHARGE_PB_CMD && bank_has_rw[CmdQueue[ecnt]->bankIndex]) ||
                    (CmdQueue[ecnt]->cmd_type == PRECHARGE_AB_CMD && rank_has_rw[CmdQueue[ecnt]->rank])) {
                CmdQueue.erase(CmdQueue.begin() + ecnt);
                erase_cnt ++;
            }
        }

        if (now() % CORE_CONCURR_PRD != 0) {
            return;
        } else if (!core_concurr_en) { //send read/write command
            for (auto &cmd : CmdQueue) {
                if (cmd->cmd_type >= WRITE_CMD && cmd->cmd_type <= READ_P_CMD) {
                    if (cmd->issue_size != 0) {c = cmd; break;}
                    if (c == NULL) {c = cmd; continue;}
                    if (priority(cmd) > priority(c)) {
                        c = cmd;
                    } else if (priority(cmd) == priority(c)) {
                        if (priority_pri(cmd) < priority_pri(c)) c = cmd;
                    }
                }
            }
            if (c == NULL) {
                CmdQueue.clear();
                return;
            }
        } else { //send act/pre/ref/pbr and so on
            for (auto &cmd : CmdQueue) {
                if (cmd->cmd_type >= WRITE_CMD && cmd->cmd_type <= READ_P_CMD) continue;
                if (c == NULL) {c = cmd; continue;}
                if (priority(cmd) > priority(c)) {
                    c = cmd;
                } else if (priority(cmd) == priority(c)) {
                    if (priority_pri(cmd) < priority_pri(c)) c = cmd;
                }
            }
            if (c == NULL) return;
        }
    }

    arb_enable = false;
    if (c->cmd_type >= WRITE_CMD && c->cmd_type <= READ_P_CMD) {
        no_sch_cmd_en = true;
        no_sch_cmd_cnt = 0x0;
        page_rw_cnt ++;
        if (RWGRP_TRANS_BY_TOUT && c->timeout) {
            sch_tout_cmd = true;
            sch_tout_type = c->type;
            sch_tout_rank = c->rank;
        }
        if (PreCmd.trans_type != c->type) {
            rw_switch_cnt ++;
            if (PreCmd.trans_type != DATA_READ && c->type == DATA_READ) w2r_switch_cnt ++;
            else r2w_switch_cnt ++;
            if (PRINT_EXEC) {
                DEBUGN(setw(10)<<now()<<" -- EXEC :: [RW_SWITCH] type="<<c->type<<" ser_rw_cnt="<<ser_rw_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- EXEC :: [RW_SWITCH] type="<<c->type<<" ser_rw_cnt="<<ser_rw_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<endl);
            }
            ser_rw_cnt = 0;
        }
        if (PreCmd.rank != c->rank) {
            rank_switch_cnt ++;
            if (PRINT_EXEC) {
                DEBUGN(setw(10)<<now()<<" -- EXEC :: [RANK_SWITCH] rank="<<c->rank<<" ser_rank_cnt="<<ser_rank_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- EXEC :: [RANK_SWITCH] rank="<<c->rank<<" ser_rank_cnt="<<ser_rank_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<endl);
            }
            ser_rank_cnt = 0;
        }
        if (PreCmd.sid != c->sid) {
            sid_switch_cnt ++;
            if (PRINT_EXEC) {
                DEBUGN(setw(10)<<now()<<" -- EXEC :: [SID_SWITCH] ser_sid_cnt="<<ser_sid_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- EXEC :: [SID_SWITCH] ser_sid_cnt="<<ser_sid_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(" wcnt="<<que_write_cnt<<" rcnt="<<que_read_cnt<<endl);
            }
            ser_sid_cnt = 0;
        }
        ser_rw_cnt ++;
        ser_rank_cnt ++;
        ser_sid_cnt ++;
        PreCmd.type = c->cmd_type;
        PreCmd.trans_type = c->type;
        PreCmd.rank = c->rank;
        PreCmd.sid = c->sid;
        PreCmd.group = c->group;
    }

    if (BG_ROTATE_EN && c->cmd_type == activate_cmd) {
        for (auto &trans : transactionQueue) {
            if (trans->rank != c->rank) continue;
            if (c->group == trans->group) {
                trans->bg_rotate_pri = NUM_GROUPS - 1;
            } else {
                if (trans->bg_rotate_pri > 0) trans->bg_rotate_pri --;
            }
        }
    }

    if (CAS_FS_EN && NUM_RANKS > 1 && DMC_RATE < 4266 && (IS_LP5 || IS_LP6 || IS_GD2)) {
        bool rw_cmd = c->cmd_type >= WRITE_CMD && c->cmd_type <= READ_P_CMD;
        bool has_wckfs = false;
        for (size_t i = 0; i < NUM_RANKS; i ++) {
            if (send_wckfs[i]) {
                has_wckfs = true;
                break;
            }
        }
        if (!has_wckfs && rw_cmd) {
            bool wck_on = false;
            for (auto &state : RankState) {
                if (!state.wck_on) continue;
                wck_on = true;
                break;
            }

            if (!wck_on) {
                for (size_t i = 0; i < NUM_RANKS; i ++) { // next rank
                    if (c->rank == i) continue;
                    if (rank_cnt[c->rank] <= CAS_FS_TH && rank_cnt[i] > 0) {
                        for (size_t j = 0; j < NUM_RANKS; j ++) send_wckfs[j] = true;
                        casfs_cnt ++;
                        break;
                    }
                }
            }
        }
    }

    if (GRP_RANK_EN && rk_grp_state != NO_RGRP) {
        uint8_t t_state = (c->rank << 1) | uint8_t(c->type);
        bool exec_cmd = c->cmd_type >= WRITE_CMD && c->cmd_type <= READ_P_CMD;
        if (rk_grp_state != real_rk_grp_state && !c->timeout && t_state == rk_grp_state
                && exec_cmd) {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- RGRP_REAL :: "<<+real_rk_grp_state<<" to "
                        <<+rk_grp_state<<", cmd_state="<<+t_state<<", task="<<c->task
                        <<", rank="<<c->rank<<", type="<<c->type<<endl);
            }
            real_rk_grp_state = rk_grp_state;
        }
    }

    if (GRP_RW_EN) {
        if (rw_group_state[0] == READ_GROUP && in_write_group && !c->timeout &&
                (c->cmd_type == READ_CMD || c->cmd_type == READ_P_CMD)) {
            in_write_group = false;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- GRP_REAL :: Real Change to READ_GROUP"<<endl);
            }
        } else if (rw_group_state[0] == WRITE_GROUP && !in_write_group && !c->timeout
                && (c->cmd_type >= WRITE_CMD && c->cmd_type <= WRITE_MASK_P_CMD)) {
            in_write_group = true;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- GRP_REAL :: Real Change to WRITE_GROUP"<<endl);
            }
        }
    }
    //added for SBR_WEIGHT_ENH_MODE=2, most recently scheduled bankIndex
    if (c->cmd_type >= WRITE_CMD && c->cmd_type <= READ_P_CMD) {
        pre_sch_bankIndex[c->rank] = c->bankIndex;
//        DEBUG(now()<<" most recently scheduled cmd, bankIndex="<<pre_sch_bankIndex[c->rank]<<", rank="<<c->rank<<", sub channel="<<sub_cha);
    }

    unsigned matgrp = 0;
    unsigned sub_channel = (c->bankIndex % NUM_BANKS) / sc_bank_num;
//    unsigned bank_start = sub_channel * NUM_BANKS / sc_num; 
    unsigned bank_pair_start = sub_channel * pbr_bank_num; 
    if (IS_GD2) matgrp = c->row % NUM_MATGRPS;
    else matgrp = 0;
    switch (c->cmd_type) {
        case READ_CMD : {
            if (RDATA_TYPE == 0) {
                for (auto &trans : transactionQueue) {
                    if (c->task != trans->task) continue;
                    if (trans->issue_size != 0) continue;
                    if (trans->mask_wcmd==true) continue;
                    if (!IECC_ENABLE || !tasks_info[c->task].rd_ecc) {
                        gen_rresp(c->task);
                    }
                    break;
                }
            }
            if (IECC_ENABLE && tasks_info[c->task].rd_ecc) {
                ecc_read_cnt++;
            }
            if (RMW_ENABLE && c->mask_wcmd) {
                merge_read_cnt++;
            }
            read_cnt++;
            access_bank_delay[c->bankIndex].enable = true;
            access_bank_delay[c->bankIndex].cnt = 0;
            bank_cas_delay[c->bankIndex] = 0;
            bankStates[c->bankIndex].state->lastCommand = READ_CMD;
            bankStates[c->bankIndex].state->lastCmdSource = c->cmd_source;
            has_wakeup[c->rank] = false;
            RdCntBl[c->bl] ++;
            if (((IS_LP5 || IS_GD2) && c->bl == BL32) || (IS_LP6 && c->bl == BL48)) {
                for (auto &bp : bp_step) bp_cycle.push_back(now() + bp);
            } else if (IS_DDR5) {
                for (size_t i = 1; i < tCCD_NSR; i ++) bp_cycle.push_back(now() + i + c->bl/2/unsigned(WCK2DFI_RATIO));
            }
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: READ_CMD task="<<c->task<<" Pri="<<c->pri<<" Qos="<<c->qos<<" rank="<<c->rank
                        <<" sid="<<c->sid<<" group="<<c->group<<" bank="<<c->bankIndex<<" row="<<c->row<<" trans_size="
                        <<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: READ_CMD task="<<c->task<<" Pri="<<c->pri<<" Qos="<<c->qos<<" rank="<<c->rank
                        <<" sid="<<c->sid<<" group="<<c->group<<" bank="<<c->bankIndex<<" row="<<c->row<<" trans_size="
                        <<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(endl);
            }
            break;
        }
        case READ_P_CMD : {
            if (RDATA_TYPE == 0) {
                for (auto &trans : transactionQueue) {
                    if (c->task != trans->task) continue;
                    if (trans->issue_size != 0) continue;
                    if (trans->mask_wcmd==true) continue;
                    if (!IECC_ENABLE || !tasks_info[c->task].rd_ecc) {
                        gen_rresp(c->task);
                    }
                    break;
                }
            }
            if (IECC_ENABLE && tasks_info[c->task].rd_ecc) {
                ecc_read_cnt++;
            }
            if (RMW_ENABLE && c->mask_wcmd) {
                merge_read_cnt++;
            }
            read_p_cnt++;
            access_bank_delay[c->bankIndex].enable = false;
            access_bank_delay[c->bankIndex].cnt = 0;
            bank_cas_delay[c->bankIndex] = 0;
            DistRefState[c->bankIndex].pre_cmd_cnt[matgrp] ++;
            DistRefState[c->bankIndex].pre_cmd_cnt_fg[matgrp] ++;
            CheckFgRef(c, c->bankIndex, matgrp);
            bankStates[c->bankIndex].state->lastCommand = READ_P_CMD;
            bankStates[c->bankIndex].state->lastCmdSource = c->cmd_source;
            bankStates[c->bankIndex].state->lastPrechargeSource = 1;
            bankStates[c->bankIndex].ser_rhit_cnt = 0;
            has_wakeup[c->rank] = false;
            RdCntBl[c->bl] ++;
            if (((IS_LP5 || IS_GD2) && c->bl == BL32) || (IS_LP6 && c->bl == BL48)) {
                for (auto &bp : bp_step) bp_cycle.push_back(now() + bp);
            } else if (IS_DDR5) {
                for (size_t i = 1; i < tCCD_NSR; i ++) bp_cycle.push_back(now() + i + c->bl/2/unsigned(WCK2DFI_RATIO));
            }
            if (now() - bankStates[c->bankIndex].state->pageOpenTime >= tREFI) {
                page_exceed_cnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PAGE_EXCEED :: bank="<<c->bankIndex<<" pageOpenTime="
                            <<bankStates[c->bankIndex].state->pageOpenTime
                            <<" open_time="<<(now() - bankStates[c->bankIndex].state->pageOpenTime)
                            <<" tREFI="<<tREFI<<endl);
                }
            }
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: READ_P_CMD task="<<c->task<<" Pri="<<c->pri<<" Qos="<<c->qos<<" rank="<<c->rank
                        <<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="<<c->row<<" trans_size="
                        <<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl<<" pre_cmd_cnt="
                        <<+DistRefState[c->bankIndex].pre_cmd_cnt[matgrp]<<" pre_cmd_cnt_fg="
                        <<DistRefState[c->bankIndex].pre_cmd_cnt_fg[matgrp]<<" fg_ref="<<c->fg_ref);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: READ_P_CMD task="<<c->task<<" Pri="<<c->pri<<" Qos="<<c->qos<<" rank="<<c->rank
                        <<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="<<c->row<<" trans_size="
                        <<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl<<" pre_cmd_cnt="
                        <<+DistRefState[c->bankIndex].pre_cmd_cnt[matgrp]<<" pre_cmd_cnt_fg="
                        <<DistRefState[c->bankIndex].pre_cmd_cnt_fg[matgrp]<<" fg_ref="<<c->fg_ref);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(endl);
            }
            break;
        }
        case WRITE_CMD : {
            if (IECC_ENABLE && tasks_info[c->task].wr_ecc) {
                ecc_write_cnt++;
            }
            write_cnt++;
            access_bank_delay[c->bankIndex].enable = true;
            access_bank_delay[c->bankIndex].cnt = 0;
            bank_cas_delay[c->bankIndex] = 0;
            bankStates[c->bankIndex].state->lastCommand = WRITE_CMD;
            bankStates[c->bankIndex].state->lastCmdSource = c->cmd_source;
            has_wakeup[c->rank] = false;
            WrCntBl[c->bl] ++;
            if (((IS_LP5 || IS_GD2) && c->bl == BL32) || (IS_LP6 && c->bl == BL48)) {
                for (auto &bp : bp_step) bp_cycle.push_back(now() + bp);
            } else if (IS_DDR5) {
                for (size_t i = 1; i < tCCD_NSW; i ++) bp_cycle.push_back(now() + i + c->bl/2/unsigned(WCK2DFI_RATIO));
            }
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: WRITE_CMD task="<<c->task<<" Pri="<<c->pri<<" Qos="<<c->qos<<" rank="<<c->rank
                        <<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="<<c->row<<" trans_size="
                        <<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: WRITE_CMD task="<<c->task<<" Pri="<<c->pri<<" Qos="<<c->qos<<" rank="<<c->rank
                        <<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="<<c->row<<" trans_size="
                        <<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(endl);
            }
            break;
        }
        case WRITE_P_CMD : {
            if (IECC_ENABLE && tasks_info[c->task].wr_ecc) {
                ecc_write_cnt++;
            }
            write_p_cnt++;
            access_bank_delay[c->bankIndex].enable = false;
            access_bank_delay[c->bankIndex].cnt = 0;
            bank_cas_delay[c->bankIndex] = 0;
            DistRefState[c->bankIndex].pre_cmd_cnt[matgrp] ++;
            DistRefState[c->bankIndex].pre_cmd_cnt_fg[matgrp] ++;
            CheckFgRef(c, c->bankIndex, matgrp);
            bankStates[c->bankIndex].state->lastCommand = WRITE_P_CMD;
            bankStates[c->bankIndex].state->lastCmdSource = c->cmd_source;
            bankStates[c->bankIndex].state->lastPrechargeSource = 1;
            bankStates[c->bankIndex].ser_rhit_cnt = 0;
            has_wakeup[c->rank] = false;
            WrCntBl[c->bl] ++;
            if (((IS_LP5 || IS_GD2) && c->bl == BL32) || (IS_LP6 && c->bl == BL48)) {
                for (auto &bp : bp_step) bp_cycle.push_back(now() + bp);
            } else if (IS_DDR5) {
                for (size_t i = 1; i < tCCD_NSW; i ++) bp_cycle.push_back(now() + i + c->bl/2/unsigned(WCK2DFI_RATIO));
            }
            if (now() - bankStates[c->bankIndex].state->pageOpenTime >= tREFI) {
                page_exceed_cnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PAGE_EXCEED :: bank="<<c->bankIndex
                            <<" pageOpenTime="<<bankStates[c->bankIndex].state->pageOpenTime
                            <<" open_time="<<(now() - bankStates[c->bankIndex].state->pageOpenTime)
                            <<" tREFI="<<tREFI<<endl);
                }
            }
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: WRITE_P_CMD task="<<c->task<<" Pri="<<c->pri<<" Qos="<<c->qos<<" rank="<<c->rank
                        <<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="<<c->row<<" trans_size="
                        <<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl<<" pre_cmd_cnt="
                        <<+DistRefState[c->bankIndex].pre_cmd_cnt[matgrp]<<" pre_cmd_cnt_fg="
                        <<DistRefState[c->bankIndex].pre_cmd_cnt_fg[matgrp]<<" fg_ref="<<c->fg_ref);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: WRITE_P_CMD task="<<c->task<<" Pri="<<c->pri<<" Qos="<<c->qos<<" rank="<<c->rank
                        <<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="<<c->row<<" trans_size="
                        <<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl<<" pre_cmd_cnt="
                        <<+DistRefState[c->bankIndex].pre_cmd_cnt[matgrp]<<" pre_cmd_cnt_fg="
                        <<DistRefState[c->bankIndex].pre_cmd_cnt_fg[matgrp]<<" fg_ref="<<c->fg_ref);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(endl);
            }
            break;
        }
        case WRITE_MASK_CMD : {
            if (IECC_ENABLE && tasks_info[c->task].wr_ecc) {
                ecc_write_cnt++;
            }
            mwrite_cnt ++;
            access_bank_delay[c->bankIndex].enable = true;
            access_bank_delay[c->bankIndex].cnt = 0;
            bank_cas_delay[c->bankIndex] = 0;
            bankStates[c->bankIndex].state->lastCommand = WRITE_MASK_CMD;
            bankStates[c->bankIndex].state->lastCmdSource = c->cmd_source;
            has_wakeup[c->rank] = false;
            WrCntBl[c->bl] ++;
            if (((IS_LP5 || IS_GD2) && c->bl == BL32) || (IS_LP6 && c->bl == BL48)) {
                for (auto &bp : bp_step) bp_cycle.push_back(now() + bp);
            } else if (IS_DDR5) {
                for (size_t i = 1; i < tCCD_NSW; i ++) bp_cycle.push_back(now() + i + c->bl/2/unsigned(WCK2DFI_RATIO));
            }
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: WRITE_MASK_CMD task="<<c->task<<" QoS="<<c->pri<<" rank="<<c->rank
                        <<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="<<c->row<<" trans_size="
                        <<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: WRITE_MASK_CMD task="<<c->task<<" QoS="<<c->pri<<" rank="<<c->rank
                        <<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="<<c->row<<" trans_size="
                        <<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(endl);
            }
            break;
        }
        case WRITE_MASK_P_CMD : {
            if (IECC_ENABLE && tasks_info[c->task].wr_ecc) {
                ecc_write_cnt++;
            }
            mwrite_p_cnt ++;
            access_bank_delay[c->bankIndex].enable = false;
            access_bank_delay[c->bankIndex].cnt = 0;
            bank_cas_delay[c->bankIndex] = 0;
            DistRefState[c->bankIndex].pre_cmd_cnt[matgrp] ++;
            DistRefState[c->bankIndex].pre_cmd_cnt_fg[matgrp] ++;
            CheckFgRef(c, c->bankIndex, matgrp);
            bankStates[c->bankIndex].state->lastCommand = WRITE_MASK_P_CMD;
            bankStates[c->bankIndex].state->lastCmdSource = c->cmd_source;
            bankStates[c->bankIndex].state->lastPrechargeSource = 1;
            bankStates[c->bankIndex].ser_rhit_cnt = 0;
            has_wakeup[c->rank] = false;
            WrCntBl[c->bl] ++;
            if (((IS_LP5 || IS_GD2) && c->bl == BL32) || (IS_LP6 && c->bl == BL48)) {
                for (auto &bp : bp_step) bp_cycle.push_back(now() + bp);
            } else if (IS_DDR5) {
                for (size_t i = 1; i < tCCD_NSW; i ++) bp_cycle.push_back(now() + i + c->bl/2/unsigned(WCK2DFI_RATIO));
            }
            if (now() - bankStates[c->bankIndex].state->pageOpenTime >= tREFI) {
                page_exceed_cnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PAGE_EXCEED :: bank="<<c->bankIndex
                            <<" pageOpenTime="<<bankStates[c->bankIndex].state->pageOpenTime
                            <<" open_time="<<(now() - bankStates[c->bankIndex].state->pageOpenTime)
                            <<" tREFI="<<tREFI<<endl);
                }
            }
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: WRITE_MASK_P_CMD task="<<c->task<<" QoS="<<c->pri<<" rank="
                        <<c->rank<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="<<c->row
                        <<" trans_size="<<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl<<" pre_cmd_cnt="
                        <<+DistRefState[c->bankIndex].pre_cmd_cnt[matgrp]<<" pre_cmd_cnt_fg="
                        <<DistRefState[c->bankIndex].pre_cmd_cnt_fg[matgrp]<<" fg_ref="<<c->fg_ref);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: WRITE_MASK_P_CMD task="<<c->task<<" QoS="<<c->pri<<" rank="
                        <<c->rank<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="<<c->row
                        <<" trans_size="<<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl<<" pre_cmd_cnt="
                        <<+DistRefState[c->bankIndex].pre_cmd_cnt[matgrp]<<" pre_cmd_cnt_fg="
                        <<DistRefState[c->bankIndex].pre_cmd_cnt_fg[matgrp]<<" fg_ref="<<c->fg_ref);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(endl);
            }
            break;
        }
        case ACTIVATE1_CMD : {
            bankStates[c->bankIndex].state->lastCmdSource = c->cmd_source;
            bankStates[c->bankIndex].state->lastCmdType = c->type;
            bankStates[c->bankIndex].state->lastCmdPri = c->pri;
            bankStates[c->bankIndex].state->act_executing = true;
            act_executing[c->bankIndex] = true;
            for (auto &trans : transactionQueue) {
                if (c->task == trans->task) {
                    trans->act_executing = true;
                    break;
                }
            }
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: ACTIVATE1_CMD task="<<c->task<<" Pri="<<c->pri<<" Qos="<<c->qos 
                        <<" rank="<<c->rank<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex
                        <<" row="<<c->row<<" matgrp="<<matgrp<<endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: ACTIVATE1_CMD task="<<c->task<<" Pri="<<c->pri<<" Qos="<<c->qos  
                        <<" rank="<<c->rank<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex
                        <<" row="<<c->row<<" matgrp="<<matgrp<<endl);
            }
            break;
        }
        case ACTIVATE2_CMD : {
            if ((bankStates[c->bankIndex].state->lastRow & 0x80000000) != 0x80000000) {
#if 0
                unsigned samerow_ohot = bankStates[c->bankIndex].state->lastRow ^ c->row;
                if (c->type == DATA_READ) {
                    for (size_t i = 0; i < 32; i ++) {
                        if ((samerow_ohot & 1) == 1) samerow_bit_rdcnt[i] ++;
                        samerow_ohot = samerow_ohot >> 1;
                    }
                } else {
                    for (size_t i = 0; i < 32; i ++) {
                        if ((samerow_ohot & 1) == 1) samerow_bit_wrcnt[i] ++;
                        samerow_ohot = samerow_ohot >> 1;
                    }
                }
#endif
                if ((bankStates[c->bankIndex].state->lastRow & 0xFFFFFF00) == (c->row & 0xFFFFFF00)) {
                    if (c->type == DATA_READ) samerow_mask_rdcnt ++;
                    else samerow_mask_wrcnt ++;
                }
            }
            if (bankStates[c->bankIndex].state->lastPrechargeSource == 1 && bankStates[c->bankIndex].state->lastRow == c->row) {
                bank_cnt_ehs[c->bankIndex] ++;
            }

            active_cnt++;
            active_cmd_cnt[c->bankIndex] ++;
            access_bank_delay[c->bankIndex].enable = false;
            access_bank_delay[c->bankIndex].cnt = 0;
            bankStates[c->bankIndex].state->lastCommand = ACTIVATE2_CMD;
            bankStates[c->bankIndex].state->lastCmdSource = c->cmd_source;
            bankStates[c->bankIndex].state->lastCmdType = c->type;
            bankStates[c->bankIndex].state->lastCmdPri = c->pri;
            bankStates[c->bankIndex].state->lastRow = c->row;
            bankStates[c->bankIndex].state->pageOpenTime = now();
            act_executing[c->bankIndex] = false;
            act_cmd_num ++;
            for (auto &trans : transactionQueue) {
                if (trans->task != c->task) continue;
                if (trans->transactionType != DATA_READ) trans->trcd_met = now() + tRCD_WR;
                else trans->trcd_met = now() + tRCD;
                break;
            }
            page_act_cnt ++;
            bankStates[c->bankIndex].state->act_executing = false;
            for (auto &trans : transactionQueue) {
                if (c->task == trans->task) {
                    trans->act_executing = false;
                    break;
                }
            }
            if (has_bypact_exec) {
                has_bypact_exec = false;
                for (auto &trans : transactionQueue) trans->arb_time ++;
            }
            for (auto &trans : transactionQueue) {
                if (c->task == trans->task) {
                    trans->has_send_act = true;
                    break;
                }
            }

            bankStates[c->bankIndex].row_hit_cnt = 0;
            bankStates[c->bankIndex].row_miss_cnt = 0;
            for (auto &t : transactionQueue) {
                if (c->bankIndex != t->bankIndex) continue;
                if (t->row == c->row) bankStates[c->bankIndex].row_hit_cnt ++;
                else bankStates[c->bankIndex].row_miss_cnt ++;
            }

            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: ACTIVATE2_CMD task="<<c->task<<" Pri="<<c->pri<<" Qos="<<c->qos 
                        <<" rank="<<c->rank<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex
                        <<" row="<<c->row<<" matgrp="<<matgrp);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: ACTIVATE2_CMD task="<<c->task<<" Pri="<<c->pri<<" Qos="<<c->qos 
                        <<" rank="<<c->rank<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex
                        <<" row="<<c->row<<" matgrp="<<matgrp);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(endl);
            }
            break;
        }
        case PRECHARGE_SB_CMD : {
            if ((c->bankIndex % NUM_BANKS) >= pbr_bank_num) {
                ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] PRECHAGE_SB_CMD, rank="<<c->rank
                        <<" sid="<<c->sid<<", bank="<<c->bankIndex);
                assert(0);
            }
            precharge_sb_cnt++;
            for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                unsigned bank_tmp = c->bankIndex + sbr_bank * pbr_bank_num;
                access_bank_delay[bank_tmp].enable = false;
                access_bank_delay[bank_tmp].cnt = 0;
                bankStates[bank_tmp].state->lastCommand = PRECHARGE_SB_CMD;
                bankStates[bank_tmp].state->lastCmdSource = c->cmd_source;
                bankStates[bank_tmp].ser_rhit_cnt = 0;
            }
            for (auto &trans : transactionQueue) {
                for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                    unsigned bank_tmp = c->bankIndex + sbr_bank * pbr_bank_num;
                    if (bank_tmp == trans->bankIndex) trans->act_executing = false;
                }
            }
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: PRECHARGE_SB_CMD task="<<c->task<<" Pri="<<c->pri<<" Qos="<<c->qos 
                        <<" rank="<<c->rank<<" sc="<<sub_channel<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex
                        <<" POSTPND="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: PRECHARGE_SB_CMD task="<<c->task<<" Pri="<<c->pri<<" Qos="<<c->qos 
                        <<" rank="<<c->rank<<" sc="<<sub_channel<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex
                        <<" POSTPND="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(endl);
            }
            break;
        }
        case PRECHARGE_PB_CMD : {   //todo: revise for enh_send_precharge
            precharge_pb_cnt++;
            access_bank_delay[c->bankIndex].enable = false;
            access_bank_delay[c->bankIndex].cnt = 0;
            bankStates[c->bankIndex].state->lastCommand = PRECHARGE_PB_CMD;
            bankStates[c->bankIndex].state->lastCmdSource = c->cmd_source;
            bankStates[c->bankIndex].state->lastPrechargeSource = c->cmd_source;
            bankStates[c->bankIndex].ser_rhit_cnt = 0;
            for (auto &trans : transactionQueue) {
                if (c->bankIndex == trans->bankIndex) trans->act_executing = false;
            }
            if (c->cmd_source == 0) rowconf_pre_cnt[c->bankIndex] ++;
            else if (c->cmd_source == 1) pageto_pre_cnt[c->bankIndex] ++;
            else func_pre_cnt[c->bankIndex] ++;
            DistRefState[c->bankIndex].pre_cmd_cnt[matgrp] ++;
            DistRefState[c->bankIndex].pre_cmd_cnt_fg[matgrp] ++;
            CheckFgRef(c, c->bankIndex, matgrp);
            if (c->cmd_source == 2) {
                refreshPerBank[c->bankIndex].refreshWaitingPre = true;
            } else {
                refreshPerBank[c->bankIndex].refreshWaitingPre = false;
            }
            if (now() - bankStates[c->bankIndex].state->pageOpenTime >= tREFI) {
                page_exceed_cnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- PAGE_EXCEED :: bank="<<c->bankIndex<<" pageOpenTime="
                            <<bankStates[c->bankIndex].state->pageOpenTime<<" open_time="
                            <<(now()-bankStates[c->bankIndex].state->pageOpenTime)<<" tREFI="<<tREFI<<endl);
                }
            }
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: PRECHARGE_PB_CMD task="<<c->task<<" Pri="<<c->pri<<" Qos="<<c->qos  
                        <<" rank="<<c->rank<<" sc="<<sub_channel<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="
                        <<c->row<<" matgrp="<<matgrp<<" POSTPND_PB="<<DistRefState[c->bankIndex].dist_pstpnd_num[matgrp]
                        <<" pre_cmd_cnt="<<DistRefState[c->bankIndex].pre_cmd_cnt[matgrp]<<" fg_ref="<<c->fg_ref
                        <<" pre_cmd_cnt_fg="<<DistRefState[c->bankIndex].pre_cmd_cnt_fg[matgrp]
                        <<" POSTPND_AB="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: PRECHARGE_PB_CMD task="<<c->task<<" Pri="<<c->pri<<" Qos="<<c->qos 
                        <<" rank="<<c->rank<<" sc="<<sub_channel<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex<<" row="
                        <<c->row<<" matgrp="<<matgrp<<" POSTPND_PB="<<DistRefState[c->bankIndex].dist_pstpnd_num[matgrp]
                        <<" pre_cmd_cnt="<<DistRefState[c->bankIndex].pre_cmd_cnt[matgrp]<<" fg_ref="<<c->fg_ref
                        <<" pre_cmd_cnt_fg="<<DistRefState[c->bankIndex].pre_cmd_cnt_fg[matgrp]
                        <<" POSTPND_AB="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(endl);
            }
            break;
        }
        case PRECHARGE_AB_CMD : {
//            if ((c->bankIndex % NUM_BANKS) != 0) {
            if ((c->bankIndex % sc_bank_num) != 0) {
                ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] PRECHAGE_AB_CMD, rank="<<c->rank
                        <<" sid="<<c->sid<<", bank="<<c->bankIndex);
                assert(0);
            }
            precharge_ab_cnt++;
            for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
                unsigned bank_tmp = c->rank * NUM_BANKS + bank;
                unsigned sub_channel = bank / sc_bank_num;
                if (EM_ENABLE && EM_MODE==2 && c->rank==1 && sub_channel==1) continue;      //rank1, sc1 forbidden under combo e-mode
                auto &state = bankStates[bank_tmp].state;
                access_bank_delay[bank_tmp].enable = false;
                access_bank_delay[bank_tmp].cnt = 0;
                bankStates[bank_tmp].state->lastCommand = PRECHARGE_AB_CMD;
                bankStates[bank_tmp].state->lastCmdSource = c->cmd_source;
                bankStates[bank_tmp].ser_rhit_cnt = 0;
                if (state->currentBankState == RowActive) {
                    DistRefState[bank_tmp].pre_cmd_cnt[state->lastRow % NUM_MATGRPS] ++;
                    DistRefState[bank_tmp].pre_cmd_cnt_fg[state->lastRow % NUM_MATGRPS] ++;
                    CheckFgRef(c, bank_tmp, state->lastRow % NUM_MATGRPS);
                }
            }
            for (auto &trans : transactionQueue) {
                if (c->rank == trans->rank) trans->act_executing = false;
            }
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: PRECHARGE_AB_CMD task="<<c->task<<" QoS="<<c->pri
                        <<" rank="<<c->rank<<" sc="<<sub_channel<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex
                        <<" pre_cmd_cnt="<<DistRefState[c->bankIndex].pre_cmd_cnt[matgrp]<<" fg_ref="<<c->fg_ref
                        <<" pre_cmd_cnt_fg="<<DistRefState[c->bankIndex].pre_cmd_cnt_fg[matgrp]
                        <<" POSTPND="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: PRECHARGE_AB_CMD task="<<c->task<<" QoS="<<c->pri
                        <<" rank="<<c->rank<<" sc="<<sub_channel<<" sid="<<c->sid<<" group="<<c->group<< " bank="<<c->bankIndex
                        <<" pre_cmd_cnt="<<DistRefState[c->bankIndex].pre_cmd_cnt[matgrp]<<" fg_ref="<<c->fg_ref
                        <<" pre_cmd_cnt_fg="<<DistRefState[c->bankIndex].pre_cmd_cnt_fg[matgrp]
                        <<" POSTPND="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(endl);
            }
            break;
        }
        case REFRESH_CMD : {
//            if ((c->bankIndex % NUM_BANKS) != 0) {
            if ((c->bankIndex % sc_bank_num) != 0) {
                ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] REFRESH_CMD, rank="<<c->rank
                        <<" sid="<<c->sid<<", bank="<<c->bankIndex);
                assert(0);
            }
            refresh_ab_cnt ++; 
            rank_refresh_cnt[c->rank][sub_channel] ++;
            refreshALL[c->rank][sub_channel].refreshing = true;
            for (size_t i = 0; i < NUM_BANKS; i ++) {
                unsigned bank_tmp = c->rank * NUM_BANKS + i;
                unsigned sub_channel = i / sc_bank_num;
                if (EM_ENABLE && EM_MODE==2 && c->rank==1 && sub_channel==1) continue;      //rank1, sc1 forbidden under combo e-mode
                bankStates[bank_tmp].finish_refresh_pb = false;
                access_bank_delay[bank_tmp].enable = false;
                access_bank_delay[bank_tmp].cnt = 0;
                bankStates[bank_tmp].state->lastCommand = REFRESH_CMD;
                bankStates[bank_tmp].state->lastCmdSource = c->cmd_source;
            }
            refreshALL[c->rank][sub_channel].refresh_cnt --;
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: REFRESH_CMD task="<<c->task<<" QoS="<<c->pri
                        << " rank="<<c->rank<<" sc="<<sub_channel<<" POSTPND="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
                    DEBUGN(" rank"<<rank<<"r="<<+r_rank_cnt[rank]<<" rank"<<rank<<"w="<<+w_rank_cnt[rank]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: REFRESH_CMD task="<<c->task<<" QoS="<<c->pri
                        << " rank="<<c->rank<<" sc="<<sub_channel<<" POSTPND="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
                    PRINTN(" rank"<<rank<<"r="<<+r_rank_cnt[rank]<<" rank"<<rank<<"w="<<+w_rank_cnt[rank]);
                }
                PRINTN(endl);
            }
            break;
        }
        case PER_BANK_REFRESH_CMD : {
            if (ENH_PBR_EN) {
                if ((c->fst_bankIndex % NUM_BANKS) >= NUM_BANKS || (c->lst_bankIndex % NUM_BANKS) >= NUM_BANKS) {
                    ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] ENHANCED PER_BANK_REFRESH_CMD, rank="<<c->rank
                            <<" sid="<<c->sid<<", bank="<<c->bankIndex<<", fst_bank="<<c->fst_bankIndex<<", lst_bank="<<c->lst_bankIndex);
                    assert(0);
                }
                refresh_pb_cnt ++;
                perbank_refresh_cnt[c->fst_bankIndex] ++;        //todo: need change for statistics
                perbank_refresh_cnt[c->lst_bankIndex] ++;        //todo: need change for statistics
            
                access_bank_delay[c->fst_bankIndex].enable = false;
                access_bank_delay[c->fst_bankIndex].cnt = 0;
                bankStates[c->fst_bankIndex].state->lastCommand = PER_BANK_REFRESH_CMD;
                bankStates[c->fst_bankIndex].state->lastCmdSource = c->cmd_source;
                refreshPerBank[c->fst_bankIndex].refreshWaiting = false;
                refreshPerBank[c->fst_bankIndex].refreshing = true;
                refreshPerBank[c->fst_bankIndex].refreshWaitingPre = false;
                access_bank_delay[c->lst_bankIndex].enable = false;
                access_bank_delay[c->lst_bankIndex].cnt = 0;
                bankStates[c->lst_bankIndex].state->lastCommand = PER_BANK_REFRESH_CMD;
                bankStates[c->lst_bankIndex].state->lastCmdSource = c->cmd_source;
                refreshPerBank[c->lst_bankIndex].refreshWaiting = false;
                refreshPerBank[c->lst_bankIndex].refreshing = true;
                refreshPerBank[c->lst_bankIndex].refreshWaitingPre = false;
                
                //added for SBR_WEIGHT_ENH_MODE==1
//                DEBUG(now()<<" before SCH :: rank="<<c->rank<<" previous ref bagroup="<<pre_enh_pbr_bagroup[c->rank]<<" sub_channel="<<sub_cha);
                pre_enh_pbr_bagroup[c->rank] = (c->fst_bankIndex % NUM_BANKS) % pbr_sb_num;
//                DEBUG(now()<<" SCH :: rank="<<c->rank<<" previous ref bagroup="<<pre_enh_pbr_bagroup[c->rank]<<" sub_channel="<<sub_cha);

                if (PRINT_SCH) {
                    DEBUGN(setw(10)<<now()<<" -- SCH :: ENHANCED PER_BANK_REFRESH_CMD task="<<c->task<<" rank="<<c->rank
                            <<" sc="<<sub_channel<<" sid="<<c->sid<<" bank="<<c->bankIndex<<" fst_bank="<<c->fst_bankIndex<<" lst_bank="<<
                            c->lst_bankIndex<<" POSTPND="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    DEBUGN(endl);
                }
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- SCH :: ENHANCED PER_BANK_REFRESH_CMD task="<<c->task<<" rank="<<c->rank
                            <<" sc="<<sub_channel<<" sid="<<c->sid<<" bank="<<c->bankIndex<<" fst_bank="<<c->fst_bankIndex<<" lst_bank="<<
                            c->lst_bankIndex<<" POSTPND="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    PRINTN(endl);
                }
            
            } else {
                if ((c->bankIndex % (NUM_BANKS/sc_num)) >= pbr_bank_num) {
                    ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] PER_BANK_REFRESH_CMD, rank="<<c->rank
                            <<" sid="<<c->sid<<", bank="<<c->bankIndex);
                    assert(0);
                }
                refresh_pb_cnt ++;
                perbank_refresh_cnt[c->rank * pbr_bank_num * sc_num + c->bankIndex % pbr_bank_num + sub_channel * bank_pair_start] ++;      //todo: revise for e-mode
                for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                    unsigned bank_tmp = c->bankIndex + sbr_bank * pbr_bank_num;
                    access_bank_delay[bank_tmp].enable = false;
                    access_bank_delay[bank_tmp].cnt = 0;
                    bankStates[bank_tmp].state->lastCommand = PER_BANK_REFRESH_CMD;
                    bankStates[bank_tmp].state->lastCmdSource = c->cmd_source;
                    refreshPerBank[bank_tmp].refreshWaiting = false;
                    refreshPerBank[bank_tmp].refreshing = true;
                    refreshPerBank[bank_tmp].refreshWaitingPre = false;
                }
                if (PRINT_SCH) {
                    DEBUGN(setw(10)<<now()<<" -- SCH :: PER_BANK_REFRESH_CMD task="<<c->task<<" rank="<<c->rank
                            <<" sc="<<sub_channel<<" sid="<<c->sid<<" bank="<<c->bankIndex<<" POSTPND="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    DEBUGN(endl);
                }
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- SCH :: PER_BANK_REFRESH_CMD task="<<c->task<<" rank="<<c->rank
                            <<" sc="<<sub_channel<<" sid="<<c->sid<<" bank="<<c->bankIndex<<" POSTPND="<<refreshALL[c->rank][sub_channel].refresh_cnt);
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    PRINTN(endl);
                }
            }
            break;
        }
        case ACTIVATE1_DST_CMD : {
            bankStates[c->bankIndex].state->lastCommand = ACTIVATE1_DST_CMD;
            bankStates[c->bankIndex].state->lastCmdSource = c->cmd_source;
            refreshPerBank[c->bankIndex].refreshWaiting = false;
            refreshPerBank[c->bankIndex].refreshing = true;
            bankStates[c->bankIndex].state->lastMatgrp = c->row % NUM_MATGRPS;
            DistRefState[c->bankIndex].state = SEND_ACT2;
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: ACTIVATE1_DST_CMD task="<<c->task<<" QoS="<<c->pri
                        <<" rank="<<c->rank<<" group="<<c->group<< " bank="<<c->bankIndex<<" matgrp="<<matgrp
                        <<" pre_cmd_cnt="<<DistRefState[c->bankIndex].pre_cmd_cnt[matgrp]<<" POSTPND="
                        <<DistRefState[c->bankIndex].dist_pstpnd_num[matgrp]<<endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: ACTIVATE1_DST_CMD task="<<c->task<<" QoS="<<c->pri
                        <<" rank="<<c->rank<<" group="<<c->group<< " bank="<<c->bankIndex<<" matgrp="<<matgrp
                        <<" pre_cmd_cnt="<<DistRefState[c->bankIndex].pre_cmd_cnt[matgrp]<<" POSTPND="
                        <<DistRefState[c->bankIndex].dist_pstpnd_num[matgrp]<<endl);
            }
            break;
        }
        case ACTIVATE2_DST_CMD : {
            active_dst_cnt ++;
            access_bank_delay[c->bankIndex].enable = false;
            access_bank_delay[c->bankIndex].cnt = 0;
            bankStates[c->bankIndex].state->lastCommand = ACTIVATE2_DST_CMD;
            bankStates[c->bankIndex].state->lastCmdSource = c->cmd_source;
            DistRefState[c->bankIndex].state = SEND_PRE;
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: ACTIVATE2_DST_CMD task="<<c->task<<" QoS="<<c->pri
                        <<" rank="<<c->rank<<" group="<<c->group<< " bank="<<c->bankIndex<<" matgrp="<<matgrp
                        <<" pre_cmd_cnt="<<DistRefState[c->bankIndex].pre_cmd_cnt[matgrp]<<" POSTPND="
                        <<DistRefState[c->bankIndex].dist_pstpnd_num[matgrp]);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: ACTIVATE2_DST_CMD task="<<c->task<<" QoS="<<c->pri
                        <<" rank="<<c->rank<<" group="<<c->group<< " bank="<<c->bankIndex<<" matgrp="<<matgrp
                        <<" pre_cmd_cnt="<<DistRefState[c->bankIndex].pre_cmd_cnt[matgrp]<<" POSTPND="
                        <<DistRefState[c->bankIndex].dist_pstpnd_num[matgrp]);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(endl);
            }
            break;
        }
        case PRECHARGE_PB_DST_CMD : {
            precharge_pb_dst_cnt ++;
            access_bank_delay[c->bankIndex].enable = false;
            access_bank_delay[c->bankIndex].cnt = 0;
            bankStates[c->bankIndex].state->lastCommand = PRECHARGE_PB_DST_CMD;
            bankStates[c->bankIndex].state->lastCmdSource = c->cmd_source;
            refreshPerBank[c->bankIndex].refreshWaiting = false;
            refreshPerBank[c->bankIndex].refreshing = false;
            perbank_refresh_cnt[c->bankIndex] ++;
            DistRefState[c->bankIndex].dist_pstpnd_num[matgrp] --;
            DistRefState[c->bankIndex].pre_cmd_cnt[matgrp] ++;
            DistRefState[c->bankIndex].pre_cmd_cnt_fg[matgrp] ++;
            DistRefState[c->bankIndex].state = DIST_IDLE;
            CheckFgRef(c, c->bankIndex, matgrp);
            if (PRINT_SCH) {
                DEBUGN(setw(10)<<now()<<" -- SCH :: PRECHARGE_PB_DST_CMD task="<<c->task<<" bank="
                        <<c->bankIndex<<" matgrp="<<matgrp<<" pre_cmd_cnt="<<DistRefState[c->bankIndex].pre_cmd_cnt[matgrp]
                        <<" pre_cmd_cnt_fg="<<DistRefState[c->bankIndex].pre_cmd_cnt_fg[matgrp]<<" fg_ref="<<c->fg_ref
                        <<" POSTPND="<<DistRefState[c->bankIndex].dist_pstpnd_num[matgrp]);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    DEBUGN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                DEBUGN(endl);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- SCH :: PRECHARGE_PB_DST_CMD task="<<c->task<<" bank="
                        <<c->bankIndex<<" matgrp="<<matgrp<<" pre_cmd_cnt="<<DistRefState[c->bankIndex].pre_cmd_cnt[matgrp]
                        <<" pre_cmd_cnt_fg="<<DistRefState[c->bankIndex].pre_cmd_cnt_fg[matgrp]<<" fg_ref="<<c->fg_ref
                        <<" POSTPND="<<DistRefState[c->bankIndex].dist_pstpnd_num[matgrp]);
                for (size_t i = 0; i < NUM_RANKS; i ++) {
                    PRINTN(" R"<<i<<"R="<<+r_rank_cnt[i]<<" R"<<i<<"W="<<+w_rank_cnt[i]);
                }
                PRINTN(endl);
            }
            break;
        }
        default : break;
    }


    // reorder arb group proority after one Cmd scheduled
    if (c != NULL && !SLOT_FIFO && (((c->cmd_type >= WRITE_CMD && c->cmd_type <= READ_P_CMD)) || (c->cmd_type==activate_cmd))) {
        unsigned cmd_index = c->arb_group;
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- SCH CMD :: Arb Group="<<c->arb_group<<" task="<<c->task
                    <<" cmd_type="<<c->cmd_type<<" QoS="<<c->pri<<" rank="<<c->rank<<" sid="<<c->sid
                    <<" group="<<c->group<< " bank="<<c->bankIndex<<" row="<<c->row<<" trans_size="
                    <<c->trans_size<<" addr_col="<<c->addr_col<<" bl="<<c->bl<<endl);
        }
        
        
        // update arb group priority, 3 is the highest
        for (size_t i=0; i<4; i++) {
            if(cmd_index==i) {
                arb_group_pri[i] = 0;
            }
        }

        //ROUND ROBIN
        for (size_t j=0; j<4; j++) {
            if (cmd_index == j) continue;
            if (j > cmd_index) {
                arb_group_pri[j] = 4 - (j - cmd_index);
            } else {
                arb_group_pri[j] = cmd_index - j;
            }
            if (arb_group_pri[j] > 3) {
                ERROR(setw(10)<<now()<<" Wrong Arb Group Pri, arb_group="<<j<<", cnt="<<arb_group_cnt[j]);
                assert(0);
            }
        }
        
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- Updated Arb Group0 Priority="<<arb_group_pri[0]<<" -- Arb Group1 Priority="<<arb_group_pri[1]
                    <<" -- Arb Group2 Priority="<<arb_group_pri[2]<<" -- Arb Group3 Priority="<<arb_group_pri[3]<<endl);
        }
    }

    if ((c!=NULL) && !SLOT_FIFO && c->cmd_type >= WRITE_CMD && c->cmd_type <= READ_P_CMD && c->issue_size + c->trans_size >= c->data_size) {
        arb_group_cnt[c->arb_group]--;
        if (arb_group_cnt[c->arb_group] > 16) {
            ERROR(setw(10)<<now()<<" Wrong Arb Group Cnt, arb_group="<<c->arb_group<<", cnt="<<arb_group_cnt[c->arb_group]);
            assert(0);
        }
    }

    if (c != NULL) generate_packet(c);
    if (CORE_CONCURR == 1 || (CORE_CONCURR != 1 && !core_concurr_en)) {
        for (auto &cmd : CmdQueue) delete cmd;
        CmdQueue.clear();
    }
}

void MemoryController::CheckFgRef(Cmd *c, unsigned bank, unsigned matgrp) {
    if (!IS_GD2 || FG_REFRESH_TH == 0) return;
    if (DistRefState[bank].pre_cmd_cnt_fg[matgrp] >= FG_REFRESH_TH) {
        DistRefState[bank].pre_cmd_cnt_fg[matgrp] = 0;
        c->fg_ref = true;
    }
}

void MemoryController::faw_update() {
    for (size_t i = 0; i < NUM_RANKS; i++) {
        if (EM_ENABLE) {
            //sub channel0
            //tfaw decrement all the counters we have going: sub channel0
            for (size_t j = 0; j < tFAWCountdown[i].size(); j++) tFAWCountdown[i][j] --;
            //tfaw the head will always be the smallest counter, so check if it has reached 0
            if (tFAWCountdown[i].size() > 0 && tFAWCountdown[i][0] == 0) tFAWCountdown[i].erase(tFAWCountdown[i].begin());
            
            //sub channel1
            //tfaw decrement all the counters we have going
            for (size_t j = 0; j < tFAWCountdown_sc1[i].size(); j++) tFAWCountdown_sc1[i][j] --;
            //tfaw the head will always be the smallest counter, so check if it has reached 0
            if (tFAWCountdown_sc1[i].size() > 0 && tFAWCountdown_sc1[i][0] == 0) tFAWCountdown_sc1[i].erase(tFAWCountdown_sc1[i].begin());
        } else {
            //tfaw decrement all the counters we have going
            for (size_t j = 0; j < tFAWCountdown[i].size(); j++) tFAWCountdown[i][j] --;
            //tfaw the head will always be the smallest counter, so check if it has reached 0
            if (tFAWCountdown[i].size() > 0 && tFAWCountdown[i][0] == 0) tFAWCountdown[i].erase(tFAWCountdown[i].begin());
        }
    }

    for (size_t i = 0; i < NUM_RANKS; i++) {
        //tfpw decrement all the counters we have going
        for (size_t j = 0; j < tFPWCountdown[i].size(); j++) tFPWCountdown[i][j] --;
        //tfpw the head will always be the smallest counter, so check if it has reached 0
        if (tFPWCountdown[i].size() > 0 && tFPWCountdown[i][0] == 0) tFPWCountdown[i].erase(tFPWCountdown[i].begin());
    }
}

void MemoryController::calc_occ() {
    if (availability < MAP_CONFIG["DMC_BW_LEVEL"][0]) {
        occ = 0;
        occ_1_cnt ++;
    } else if (availability < MAP_CONFIG["DMC_BW_LEVEL"][1]) {
        occ = 1;
        occ_2_cnt ++;
    } else if (availability < MAP_CONFIG["DMC_BW_LEVEL"][2]) {
        occ = 2;
        occ_3_cnt ++;
    } else {
        occ = 3;
        occ_4_cnt ++;
    }
}

double MemoryController::calc_sqrt(double sum, double sum_pwr2, unsigned cnt) {
    if (cnt == 0) return 0;
    double avg = sum / cnt;
    double dev = (sum_pwr2 / cnt) - (avg * avg);
    return sqrt(dev);
}

float MemoryController::CalcBwByByte(uint64_t byte, unsigned cycle) {
    if (IS_LP6) {
        return (100 * float(byte) * 8 * 9 / 8 / 2 / JEDEC_DATA_BUS_BITS / cycle / WCK2DFI_RATIO / PAM_RATIO);
    } else {
        return (100 * float(byte) * 8 / 2 / JEDEC_DATA_BUS_BITS / cycle / WCK2DFI_RATIO / PAM_RATIO);
    }
}

void MemoryController::update_statistics() {
    if (DMC_BW_WIN != 0 && now() % DMC_BW_WIN == 0) {
        availability = unsigned(CalcBwByByte(bw_totalcmds, DMC_BW_WIN));
        bw_totalcmds   = 0;
        bw_totalwrites = 0;
        bw_totalreads  = 0;

        static unsigned rw_cnt = 0;
        static unsigned act_cnt = 0;
        unsigned row_hit_cnt = 0;
        rw_cnt = read_cnt + read_p_cnt + write_cnt + write_p_cnt + mwrite_cnt + mwrite_p_cnt - rw_cnt;
        act_cnt = active_cnt - act_cnt;
        row_hit_cnt = (rw_cnt > act_cnt) ? (rw_cnt - act_cnt) : 0;
        row_hit_ratio = (rw_cnt == 0) ? 0 : ((float)row_hit_cnt * 100 / rw_cnt);
        rw_cnt = read_cnt + read_p_cnt + write_cnt + write_p_cnt + mwrite_cnt + mwrite_p_cnt;
        act_cnt = active_cnt;

        calc_occ();

        sum_avai += availability;
        sum_pwr_avai += availability * availability;
        avai_sqrt = calc_sqrt(double(sum_avai), double(sum_pwr_avai), now() / DMC_BW_WIN + 1);
    }

    if (PRINT_BW && PRINT_BW_WIN != 0 && now() != 0 && now() % PRINT_BW_WIN == 0) {
        float dmc_availability = 0;
        dmc_availability = CalcBwByByte(TotalDmcBytes, PRINT_BW_WIN);
        TRACE_PRINT(setw(10)<<now()<<" -- Total availability: "<<setw(5)<<fixed<<setprecision(1)<<dmc_availability);
        dmc_availability = CalcBwByByte(TotalDmcRdBytes, PRINT_BW_WIN);
        TRACE_PRINT(", DMC Rd availability: "<<setw(5)<<fixed<<setprecision(1)<<dmc_availability);
        dmc_availability = CalcBwByByte(TotalDmcWrBytes, PRINT_BW_WIN);
        TRACE_PRINT(", DMC Wr availability: "<<setw(5)<<fixed<<setprecision(1)<<dmc_availability);
        TotalDmcBytes = 0;
        TotalDmcRdBytes = 0;
        TotalDmcWrBytes = 0;
        TRACE_PRINT(", Q: "<<setw(3)<<transactionQueue.size());
        TRACE_PRINT(", QR: "<<setw(3)<<+que_read_cnt);
        TRACE_PRINT(", QW: "<<setw(3)<<+que_write_cnt);
        unsigned bw_bank_cnt = 0;
        for (size_t bank = 0; bank < NUM_RANKS * NUM_BANKS; bank ++) {
            if ((r_bank_cnt[bank] + w_bank_cnt[bank]) > 0) {
                bw_bank_cnt ++;
            }
        }
        TRACE_PRINT(", Bank: "<<setw(2)<<bw_bank_cnt<<endl);
    }
}

void MemoryController::update_state() {
    if (!DEBUG_STATE) return;

    PRINTN("--------------------------------------------------------------------------------------------------"<<endl)
    PRINTN("Total Status: R:"<<que_read_cnt<<" W:"<<que_write_cnt<<" rw_group_state="<<+rw_group_state[0]<<" | in_write_group="
            <<in_write_group<<" | availability="<<availability<<" | row_hit_ratio="<<row_hit_ratio<<" | PreCmd.rank="<<PreCmd.rank
            <<" | PreCmd.type="<<PreCmd.type<<" | rk_grp_state="<<+rk_grp_state<<" | real_rk_grp_state="<<+real_rk_grp_state<<endl)
    for (auto &state : bankStates) {
        unsigned sub_channel = (state.bankIndex % NUM_BANKS) / sc_bank_num;
        if (EM_ENABLE) {
            PRINTN("ST time: "<<now()<<" | bank="<<state.bankIndex<<" | sc="<<sub_channel<<" | Rcnt="<<+r_bank_cnt[state.bankIndex]<<" | Wcnt="
                    <<+w_bank_cnt[state.bankIndex]<<" | state="<<bank_state_opcode(state.state->currentBankState)<<" | OpenRowAddr="
                    <<state.state->openRowAddress<<" | nAct1="<<state.state->nextActivate1<<" | nAct2="<<state.state->nextActivate2
                    <<" | nR="<<state.state->nextRead<<" | nW="<<state.state->nextWrite<<" | nWRmw="<<state.state->nextWriteRmw<<" | nMW="
                    <<state.state->nextWriteMask<<" | nRAp="<<state.state->nextReadAp<<" | nWAp="<<state.state->nextWriteAp<<" | nWApRmw="
                    <<state.state->nextWriteApRmw<<" | nMWAp="<<state.state->nextWriteMaskAp<<" | nPre="<<state.state->nextPrecharge
                    <<" | hPre="<<state.hold_precharge<<" | lastCmd="<<state.state->lastCommand<<" | lastCmdType="<<state.state->lastCmdType
                    <<" | lastCmdPri="<<state.state->lastCmdPri<<" | lastCmdSource="<<state.state->lastCmdSource<<" | HasCmdRhit="
                    <<state.has_cmd_rowhit<<" | rhit_brk="<<state.has_rhit_break<<" | ser_rhit_cnt="<<state.ser_rhit_cnt<<" | row_hit_cnt="
                    <<state.row_hit_cnt<<" | row_miss_cnt="<<state.row_miss_cnt<<" | pbrW="<<refreshPerBank[state.bankIndex].refreshWaiting
                    <<" | pbr="<<refreshPerBank[state.bankIndex].refreshing<<" | pbr_pre="<<refreshPerBank[state.bankIndex].refreshWaitingPre<<" | abrW="
                    <<refreshALL[state.rank][0].refreshWaiting<<" | abr="<<refreshALL[state.rank][0].refreshing
                    <<" | abrW_sc1="<<refreshALL[state.rank][1].refreshWaiting<<" | abr_sc1="<<refreshALL[state.rank][1].refreshing 
                    <<" | stc_cnt="<<state.state->stateChangeCountdown<<" | stc_en="<<state.state->stateChangeEn<<" | POSTPND="
                    <<refreshALL[state.rank][0].refresh_cnt<<" | POSTPND_SC1="<<refreshALL[state.rank][1].refresh_cnt 
                    <<" | Fpbr="<<state.finish_refresh_pb<<" | lp_state="<<RankState[state.rank].lp_state
                    <<" | rank_has_cmd="<<rank_has_cmd[state.rank]<<" | acc_en="<<access_bank_delay[state.bankIndex].enable<<" | acc_cnt="
                    <<access_bank_delay[state.bankIndex].cnt<<" | adpt_openpage_time="
                    <<adpt_openpage_time<<" | opc_cnt="<<opc_cnt<<" | ppc_cnt="<<ppc_cnt<<" | act_executing="<<state.state->act_executing);
        } else {
            PRINTN("ST time: "<<now()<<" | bank="<<state.bankIndex<<" | sc="<<sub_channel<<" | Rcnt="<<+r_bank_cnt[state.bankIndex]<<" | Wcnt="
                    <<+w_bank_cnt[state.bankIndex]<<" | state="<<bank_state_opcode(state.state->currentBankState)<<" | OpenRowAddr="
                    <<state.state->openRowAddress<<" | nAct1="<<state.state->nextActivate1<<" | nAct2="<<state.state->nextActivate2
                    <<" | nR="<<state.state->nextRead<<" | nW="<<state.state->nextWrite<<" | nWRmw="<<state.state->nextWriteRmw<<" | nMW="
                    <<state.state->nextWriteMask<<" | nRAp="<<state.state->nextReadAp<<" | nWAp="<<state.state->nextWriteAp<<" | nWApRmw="
                    <<state.state->nextWriteApRmw<<" | nMWAp="<<state.state->nextWriteMaskAp<<" | nPre="<<state.state->nextPrecharge
                    <<" | hPre="<<state.hold_precharge<<" | lastCmd="<<state.state->lastCommand<<" | lastCmdType="<<state.state->lastCmdType
                    <<" | lastCmdPri="<<state.state->lastCmdPri<<" | lastCmdSource="<<state.state->lastCmdSource<<" | HasCmdRhit="
                    <<state.has_cmd_rowhit<<" | rhit_brk="<<state.has_rhit_break<<" | ser_rhit_cnt="<<state.ser_rhit_cnt<<" | row_hit_cnt="
                    <<state.row_hit_cnt<<" | row_miss_cnt="<<state.row_miss_cnt<<" | pbrW="<<refreshPerBank[state.bankIndex].refreshWaiting
                    <<" | pbr="<<refreshPerBank[state.bankIndex].refreshing<<" | pbr_pre="<<refreshPerBank[state.bankIndex].refreshWaitingPre<<" | abrW="
                    <<refreshALL[state.rank][0].refreshWaiting<<" | abr="<<refreshALL[state.rank][0].refreshing<<" | stc_cnt="
                    <<state.state->stateChangeCountdown<<" | stc_en="<<state.state->stateChangeEn<<" | POSTPND="
                    <<refreshALL[state.rank][0].refresh_cnt<<" | Fpbr="<<state.finish_refresh_pb<<" | lp_state="<<RankState[state.rank].lp_state
                    <<" | rank_has_cmd="<<rank_has_cmd[state.rank]<<" | acc_en="<<access_bank_delay[state.bankIndex].enable<<" | acc_cnt="
                    <<access_bank_delay[state.bankIndex].cnt<<" | adpt_openpage_time="
                    <<adpt_openpage_time<<" | opc_cnt="<<opc_cnt<<" | ppc_cnt="<<ppc_cnt<<" | act_executing="<<state.state->act_executing);
        }
        for (size_t i = 0; i < NUM_MATGRPS; i ++) {
            PRINTN(" | pre_cmd_cnt"<<i<<"="<<DistRefState[state.bankIndex].pre_cmd_cnt[i]);
            PRINTN(" | pre_cmd_cnt_fg"<<i<<"="<<DistRefState[state.bankIndex].pre_cmd_cnt_fg[i]);
        }
        for (size_t i = 0; i < NUM_MATGRPS; i ++) {
            PRINTN(" | dist_pstpnd_num"<<i<<"="<<DistRefState[state.bankIndex].dist_pstpnd_num[i]);
        }
        PRINTN(endl);
    }
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl)
    PRINTN("Command Queue Status: Qsize="<<transactionQueue.size()<<" | QRcnt:"<<que_read_cnt<<" | QWcnt:"
            <<que_write_cnt<<endl)
    for (auto &trans : transactionQueue) {
        PRINTN("Que time: "<<now()<<" | task="<<trans->task<<" | bank="<<trans->bankIndex<<" | rank="<<trans->rank<<" | row="<<trans->row
                <<" | addr_col="<<trans->addr_col<<" | arb_time="<<trans->arb_time<<" | type="<<trans_type_opcode(trans->transactionType)
                <<" | nextCmd="<<trans_cmd_opcode(trans->nextCmd)<<" | arb_grp="<<trans->arb_group<<" | arb_grp_pri="
                <<arb_group_pri[trans->arb_group]<<" | arb_grp_cnt="<<arb_group_cnt[trans->arb_group]
                <<" | address="<<hex<<trans->address<<dec<<" | length="<<trans->burst_length<<" | data_size="<<trans->data_size<<" | issue_size="
                <<trans->issue_size<<" | data_ready_cnt="<<trans->data_ready_cnt<<" | timeout="<<trans->timeout<<" | bp_by_tout="
                <<trans->bp_by_tout<<" | qos="<<trans->qos<<" | pri="<<trans->pri<<" | addrconf="<<trans->addrconf<<" | timeout_th="
                <<trans->timeout_th<<" | timeAdded="<<trans->timeAdded<<" | pri_adapt_th="<<trans->pri_adapt_th<<" | act_executing="
                <<trans->act_executing<<" | pre_act="<<trans->pre_act<<" mask_wcmd="<<trans->mask_wcmd<<endl)
    }
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl)
    if (ENH_PAGE_ADPT_EN && now() % ENH_PAGE_ADPT_WIN == 0 && now() != 0) {
        PRINTN("EHS_PAGE_R: "<<now());
        for (size_t i = 0; i < NUM_RANKS * NUM_BANKS; i ++)
            PRINTN(" | R"<<i<<"= "<<setw(3)<<page_timeout_rd[i]);
        PRINTN(endl);
        PRINTN("EHS_PAGE_W: "<<now());
        for (size_t i = 0; i < NUM_RANKS * NUM_BANKS; i ++)
            PRINTN(" | R"<<i<<"= "<<setw(3)<<page_timeout_wr[i]);
        PRINTN(endl);
    }
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl)
}

std::string MemoryController::bank_state_opcode(CurrentBankState state) {
    switch (state) {
        case Idle        : {return "Idle"; break;}
        case RowActive   : {return "RowActive"; break;}
        case Precharging : {return "Precharging"; break;}
        case Refreshing  : {return "Refreshing"; break;}
        default          : break;
    }
    return "Unkown opcode";
}

std::string MemoryController::trans_cmd_opcode(TransactionCmd state) {
    switch (state) {
        case PRECHARGE_SB_CMD     : {return "PRECHARGE_SB"; break;}
        case PRECHARGE_PB_CMD     : {return "PRECHARGE_PB"; break;}
        case PRECHARGE_AB_CMD     : {return "PRECHARGE_AB"; break;}
        case ACTIVATE1_CMD        : {return "ACTIVATE1"; break;}
        case ACTIVATE2_CMD        : {return "ACTIVATE2"; break;}
        case WRITE_CMD            : {return "WRITE"; break;}
        case WRITE_P_CMD          : {return "WRITE_P"; break;}
        case WRITE_MASK_CMD       : {return "WRITE_MASK"; break;}
        case WRITE_MASK_P_CMD     : {return "WRITE_MASK_P"; break;}
        case READ_CMD             : {return "READ"; break;}
        case READ_P_CMD           : {return "READ_P"; break;}
        case REFRESH_CMD          : {return "REFRESH"; break;}
        case INVALID              : {return "INVALID"; break;}
        case PER_BANK_REFRESH_CMD : {return "PER_BANK_REFRESH"; break;}
        case PDE_CMD              : {return "PDE"; break;}
        case PD_CMD               : {return "PD"; break;}
        case PDX_CMD              : {return "PDX"; break;}
        case ASREF_CMD            : {return "ASREF"; break;}
        case ASREFX_CMD           : {return "ASREFX"; break;}
        case SRPDE_CMD            : {return "SRPDE"; break;}
        case SRPD_CMD             : {return "SRPD"; break;}
        case SRPDX_CMD            : {return "SRPDX"; break;}
        case ACTIVATE1_DST_CMD    : {return "ACTIVATE1_DST"; break;}
        case ACTIVATE2_DST_CMD    : {return "ACTIVATE2_DST"; break;}
        case PRECHARGE_PB_DST_CMD : {return "PRECHARGE_PB_DST"; break;}
        default                   : break;
    }
    return "Unkown opcode";
}

std::string MemoryController::trans_type_opcode(TransactionType state) {
    switch (state) {
        case DATA_READ       : {return "READ"; break;}
        case DATA_WRITE      : {return "WRITE"; break;}
        default              : break;
    }
    return "Unkown opcode";
}

/***************************************************************************************************
descriptor: main update ,every cycle will execute it
****************************************************************************************************/
void MemoryController::update() {
//    DEBUG(now()<<" update0, sc_num="<<sc_num); 
    
    update_even_cycle();

    update_cresp();

    update_statistics();

    update_state_pre();

    update_state();

    power_event_stat();

    update_lp_state();

    update_wdata();

    update_grt_fifo();

    for (size_t i = 0; i < CORE_CONCURR; i ++) {
        if (now() % CORE_CONCURR_PRD != 0 && i == 1) break;
        core_concurr_en = (i == 0);

        exec();

        state_fresh();

        scheduler();
    }

    faw_update();

    update_que();

    que_pipeline();

    page_timeout_policy();

    update_group_state();

    update_rwgroup_state();

    for (size_t i=0 ; i<sc_num; i++) {
        refresh(i);
    }

    data_fresh();

    update_state_post();

    ehs_page_adapt_policy();

    update_deque_fifo();
}

#if 0
void MemoryController::update_group_state() {
    if (!GRP_RANK_EN || NUM_RANKS <= 1) {
        return;
    }

    vector <bool> has_cmd; // rank0 read, rank0 write, rank1 read, rank1 write
    for (size_t i = 0; i < NUM_RANKS * 2; i ++) {
        has_cmd.push_back(false);
    }
    for (auto &trans : transactionQueue) {
        auto &st = bankStates[trans->bankIndex].state;
        if (trans->addrconf) continue;
        if (trans->pre_act) continue;
        if (now() < trans->enter_que_time) continue;
        if (trans->bp_by_tout) continue;
        if (refreshALL[trans->rank].refreshing) continue;
        if (refreshPerBank[trans->bankIndex].refreshing) continue;
        if (st->currentBankState == RowActive && trans->row != st->openRowAddress && GetDmcQsize() <= 8)
            continue;
        if (trans->transactionType == DATA_READ) {
            has_cmd[trans->rank * NUM_RANKS] = true;
        } else if (trans->data_ready_cnt == (trans->burst_length + 1)) {
            has_cmd[trans->rank * NUM_RANKS + 1] = true;
        }
    }

    if (no_sch_cmd_en) no_sch_cmd_cnt ++;
    if (GetDmcQsize() <= 4) {
        rk_grp_state = NO_RGRP;
        real_rk_grp_state = NO_RGRP;
        serial_cmd_cnt = 0;
        rwgrp_ch_cmd_cnt = 0;
    } else {
        if (rk_grp_state == NO_RGRP) {
            rank_group_weight();
            rk_grp_state = rg_weight;
            real_rk_grp_state = rg_weight ^ 1;
        } else if (rk_grp_state == real_rk_grp_state) {
            rank_group_weight();
            uint8_t *nxt_rank_cnt = NULL, *cur_rank_cnt = NULL;
            uint8_t c_idx = 0, n_idx = 0, c_wr = 0, n_wr = 0;
            c_idx = (rk_grp_state >> 1) & 1;
            n_idx = (rg_weight >> 1) & 1;
            c_wr = rk_grp_state & 1;
            n_wr = rg_weight & 1;

            if (c_wr == 1) cur_rank_cnt = &(w_rank_cnt[c_idx]);
            else cur_rank_cnt = &(r_rank_cnt[c_idx]);
            if (n_wr == 1) nxt_rank_cnt = &(w_rank_cnt[n_idx]);
            else nxt_rank_cnt = &(r_rank_cnt[n_idx]);

            uint8_t st_chg_flag = 0;
            if (has_cmd[rg_weight]) {
                if (no_sch_cmd_cnt >= 16 && *cur_rank_cnt <= 4) {
                    st_chg_flag = 1;
                    no_sch_cmd_en = false;
                    no_sch_cmd_cnt = 0x0;
                } else if (serial_cmd_cnt >= 10 && *nxt_rank_cnt >= 6 && *cur_rank_cnt <= 2) {
                    st_chg_flag = 2;
                } else if (serial_cmd_cnt >= 20 && *nxt_rank_cnt >= 12 && *cur_rank_cnt <= 10) {
                    st_chg_flag = 3;
                } else if (serial_cmd_cnt >= 64 && *cur_rank_cnt <= 16) {
                    st_chg_flag = 4;
                } else if (!has_cmd[rk_grp_state]) {
                    st_chg_flag = 5;
                }
            }
            if (st_chg_flag != 0) {
                if (1) {
                    DEBUGN(setw(10)<<now()<<" -- RGRP_CHG st_chg_flag="<<+st_chg_flag<<", cur_state="
                            <<+rk_grp_state<<", next_state="<<+rg_weight<<", ser="<<+serial_cmd_cnt);
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        DEBUGN(", R"<<i<<"R="<<+r_rank_cnt[i]<<", R"<<i<<"W="<<+w_rank_cnt[i]);
                    }
                    DEBUGN(endl);
                }
                serial_cmd_cnt = 0;
                rwgrp_ch_cmd_cnt = 0;
                rk_grp_state = rg_weight;
            }
        }
    }
    // Multiplex rw group for other logic, such as hold_precharge...
    rw_group_state[0] = (rk_grp_state == NO_RGRP) ? NO_GROUP : (rk_grp_state & 1);
    in_write_group = real_rk_grp_state & 1;
}
#else
void MemoryController::update_group_state() {
    if (!GRP_RANK_EN) return;

    // Send Precharge to others rank's open bank
    unsigned curr_cnt = rank_cnt[(((unsigned)rk_grp_state & (NUM_RANKS - 1)) >> 1ul)];
    if (GRP_RANK_PREPB && rk_grp_state != NO_RGRP && curr_cnt >= GRP_RANK_LEVEL) {
        vector <bool> has_timeout;
        for (size_t i = 0; i < NUM_RANKS * NUM_BANKS; i ++) has_timeout.push_back(false);

        for (auto &t : transactionQueue) {
            if (t->timeout) has_timeout[t->bankIndex] = true;
        }

        for (size_t i = 0; i < NUM_RANKS; i ++) {
            if (i == ((unsigned)rk_grp_state >> 1ul)) continue;
            if (tFPWCountdown[i].size() >= 4) continue;
            for (size_t j = 0; j < NUM_BANKS; j ++) {
                auto &state = bankStates[i * NUM_BANKS + j];
//                unsigned sub_channel = j / sc_bank_num;
                if (state.state->currentBankState != RowActive) continue;
                if ((now() + 1) < state.state->nextPrecharge) continue;
                if (has_timeout[i * NUM_BANKS + j]) continue;
                funcState[i].wakeup = true;
                if (RankState[i].lp_state != IDLE) continue;

                Cmd *c = new Cmd;
                c->rank = state.rank;
//                c->channel = sub_channel;
                c->sid = state.sid;
                c->group = state.group;
                c->bank = state.bank;
                c->row = state.state->openRowAddress;
                c->cmd_type = PRECHARGE_PB_CMD;
                c->bankIndex = state.bankIndex;
                c->cmd_source = 1;
                c->task = 0xFFFFFFFFFFFFFFF;
                CmdQueue.push_back(c);
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- REQ :: Page Timeout, precharge bank="
                            <<c->bankIndex<<", task="<<c->task<<endl);
                }
            }
        }
    } else if (GRP_RANK_PREAB && rk_grp_state != NO_RGRP && curr_cnt >= GRP_RANK_LEVEL) {
        vector <bool> has_timeout;
        for (size_t i = 0; i < NUM_RANKS; i ++) has_timeout.push_back(false);

        for (auto &t : transactionQueue) {
            if (has_timeout[t->rank]) continue;
            if (t->timeout) has_timeout[t->rank] = true;
        }

        for (size_t i = 0; i < NUM_RANKS; i ++) {
            if (i == ((unsigned)rk_grp_state >> 1ul)) continue;
            if (tFPWCountdown[i].size() >= 4) continue;
            if (has_timeout[i]) continue;

            bool idle = true, precharge_en = true;
            for (size_t j = 0; j < NUM_BANKS; j ++) {
                if (bankStates[i * NUM_BANKS + j].state->currentBankState != RowActive) continue;
                idle = false;
                break;
            }
            for (size_t j = 0; j < NUM_BANKS; j ++) {
                if ((now() + 1) < bankStates[i * NUM_BANKS + j].state->nextPrecharge ||
                        bankStates[i * NUM_BANKS + j].state->act_executing ||
                        bankStates[i * NUM_BANKS + j].state->currentBankState == Refreshing) {
                    precharge_en = false;
                    break;
                }
            }

            if (!idle && precharge_en) {
                funcState[i].wakeup = true;
                if (RankState[i].lp_state == IDLE) {
                    Cmd *c = new Cmd;
                    c->rank = i;
                    c->cmd_type = PRECHARGE_AB_CMD;
                    c->bankIndex = i * NUM_BANKS;     // todo: revise for e-mode
                    c->cmd_source = 1;
                    c->task = 0xFFFFFFFFFFFFFFF;
                    CmdQueue.push_back(c);
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- REQ :: Page Timeout, precharge bank="
                                <<c->bankIndex<<", task="<<c->task<<endl);
                    }
                }
            }
        }
    }

    vector <bool> has_rd_cmd;
    vector <bool> has_wr_cmd;
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        has_rd_cmd.push_back(false);
        has_wr_cmd.push_back(false);
    }
    for (auto &trans : transactionQueue) {
        auto &st = bankStates[trans->bankIndex].state;
        unsigned sub_channel = (trans->bankIndex % NUM_BANKS) / sc_bank_num;
        if (trans->addrconf) continue;
        if (trans->pre_act) continue;
        if (now() < trans->enter_que_time) continue;
        if (trans->bp_by_tout) continue;
        if (refreshALL[trans->rank][sub_channel].refreshing) continue;
        if (refreshPerBank[trans->bankIndex].refreshing) continue;
        if (st->currentBankState == RowActive && trans->row != st->openRowAddress && GetDmcQsize() <= 8)
            continue;
        if (trans->transactionType == DATA_READ) {
            has_rd_cmd[trans->rank] = true;
        } else if (trans->data_ready_cnt == (trans->burst_length + 1)) {
            has_wr_cmd[trans->rank] = true;
        }
    }

    switch (rk_grp_state) {
        case NO_RGRP: {
            if (GetDmcQsize() >= 4/*ENGRP_LEVEL*/) {
                unsigned rank = 0, type = 0;
                if (sch_tout_cmd) {
                    rank = sch_tout_rank;
                    type = sch_tout_type;
                } else {
                    rank_group_weight(&rank, &type);
                }
                rk_grp_state = rank << 1 | type;
                real_rk_grp_state = rank << 1 | type;
                serial_cmd_cnt = 0;
                rwgrp_ch_cmd_cnt = 0;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- RGRP_CHG :: From NG to "<<+rk_grp_state<<", Qsize="<<GetDmcQsize());
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        PRINTN(", R"<<i<<"R="<<*r_rank_mux[i]<<", R"<<i<<"W="<<*w_rank_mux[i]);
                    }
                    PRINTN(endl);
                }
            }
            break;
        }
        case R0RD_GROUP:
        case R0WR_GROUP:
        case R1RD_GROUP:
        case R1WR_GROUP: {
            if (GetDmcQsize() < 2/*EXGRP_LEVEL*/) {
                rk_grp_state = NO_RGRP;
                real_rk_grp_state = NO_RGRP;
                serial_cmd_cnt = 0;
                rwgrp_ch_cmd_cnt = 0;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- RGRP_CHG :: From "<<+rk_grp_state<<" to NG"<<", Qsize="<<GetDmcQsize());
                    for (size_t i = 0; i < NUM_RANKS; i ++) {
                        PRINTN(", R"<<i<<"R="<<*r_rank_mux[i]<<", R"<<i<<"W="<<*w_rank_mux[i]);
                    }
                    PRINTN(endl);
                }
            } else {
                if (sch_tout_cmd) {
                    uint8_t r_state = (sch_tout_rank << 1) | (sch_tout_type & 1);
                    rk_grp_state = r_state;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- RGRP_CHG :: Tout From "<<+real_rk_grp_state<<" to "
                                <<+rk_grp_state<<", serial_cmd_cnt="<<+serial_cmd_cnt);
                        for (size_t i = 0; i < NUM_RANKS; i ++) {
                            PRINTN(", R"<<i<<"R="<<*r_rank_mux[i]<<", R"<<i<<"W="<<*w_rank_mux[i]);
                        }
                        PRINTN(endl);
                    }
                    real_rk_grp_state = rk_grp_state;
                    serial_cmd_cnt = 0;
                    rwgrp_ch_cmd_cnt = 0;
                } else if (rk_grp_state == real_rk_grp_state) {
                    unsigned crank = (rk_grp_state >> 1) & 3, ctype = rk_grp_state & 1;
                    unsigned nrank = 0, ntype = 0;
                    rank_group_weight(&nrank, &ntype);
                    if (ctype == ntype && ctype == 0 && has_rd_cmd[nrank]) { // for diff rank r2r
                        if ((serial_cmd_cnt >= 8 && *r_rank_mux[nrank] >= 4 && *r_rank_mux[crank] <= 4)
                                || (serial_cmd_cnt >= 96 && *r_rank_mux[crank] <= 6) ||
                                (*r_rank_mux[crank] <= 4 && GetDmcQsize() >= 32) || !has_rd_cmd[crank]) {
                            rk_grp_state = nrank << 1 | ntype;
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- RGRP_CHG :: From "<<+real_rk_grp_state<<" to "
                                        <<+rk_grp_state<<", serial_cmd_cnt="<<+serial_cmd_cnt<<", cur_has="
                                        <<has_rd_cmd[crank]<<", nxt_has="<<has_rd_cmd[nrank]);
                                for (size_t i = 0; i < NUM_RANKS; i ++) {
                                    PRINTN(", R"<<i<<"R="<<*r_rank_mux[i]<<", R"<<i<<"W="<<*w_rank_mux[i]);
                                }
                                PRINTN(endl);
                            }
                            serial_cmd_cnt = 0x0;
                            rwgrp_ch_cmd_cnt = 0x0;
                            real_rk_grp_state = crank << 1 | ctype;
                        }
                    } else if (ctype == ntype && ctype == 1 && has_wr_cmd[nrank]) { // for diff rank w2w
                        if ((serial_cmd_cnt >= 4 && *w_rank_mux[nrank] >= 32 && *w_rank_mux[crank] <= 12)
                                || (serial_cmd_cnt >= 48 && *w_rank_mux[crank] <= 4) ||
                                (*w_rank_mux[crank] <= 4 && GetDmcQsize() >= 32) || !has_wr_cmd[crank]) {
                            rk_grp_state = nrank << 1 | ntype;
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- RGRP_CHG :: From "<<+real_rk_grp_state<<" to "
                                        <<+rk_grp_state<<", serial_cmd_cnt="<<+serial_cmd_cnt<<", cur_has="
                                        <<has_wr_cmd[crank]<<", nxt_has="<<has_wr_cmd[nrank]);
                                for (size_t i = 0; i < NUM_RANKS; i ++) {
                                    PRINTN(", R"<<i<<"R="<<*r_rank_mux[i]<<", R"<<i<<"W="<<*w_rank_mux[i]);
                                }
                                PRINTN(endl);
                            }
                            serial_cmd_cnt = 0x0;
                            rwgrp_ch_cmd_cnt = 0x0;
                            real_rk_grp_state = crank << 1 | ctype;
                        }
                    } else if (ctype != ntype && ctype == 0 && has_wr_cmd[nrank]) { // for same/diff rank r2w
                        if ((serial_cmd_cnt >= 16 && *w_rank_mux[nrank] >= 32 && *r_rank_mux[crank] <= 20) ||
                                (*r_rank_mux[crank] <= 4 && GetDmcQsize() >= 32) || serial_cmd_cnt >= 64 ||
                                !has_rd_cmd[crank]) {
                            rk_grp_state = nrank << 1 | ntype;
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- RGRP_CHG :: From "<<+real_rk_grp_state<<" to "
                                        <<+rk_grp_state<<", serial_cmd_cnt="<<+serial_cmd_cnt<<", cur_has="
                                        <<has_rd_cmd[crank]<<", nxt_has="<<has_wr_cmd[nrank]);
                                for (size_t i = 0; i < NUM_RANKS; i ++) {
                                    PRINTN(", R"<<i<<"R="<<*r_rank_mux[i]<<", R"<<i<<"W="<<*w_rank_mux[i]);
                                }
                                PRINTN(endl);
                            }
                            serial_cmd_cnt = 0x0;
                            rwgrp_ch_cmd_cnt = 0x0;
                            real_rk_grp_state = crank << 1 | ctype;
                        }
                    } else if (ctype != ntype && ctype == 1 && has_rd_cmd[nrank]) { // for same/diff rank w2r
                        if ((serial_cmd_cnt >= 4 && *w_rank_mux[crank] <= 16) || serial_cmd_cnt >= 32
                                || (*w_rank_mux[crank] <= 4 && GetDmcQsize() >= 32) || !has_wr_cmd[crank]) {
                            rk_grp_state = nrank << 1 | ntype;
                            if (DEBUG_BUS) {
                                PRINTN(setw(10)<<now()<<" -- RGRP_CHG :: From "<<+real_rk_grp_state<<" to "
                                        <<+rk_grp_state<<", serial_cmd_cnt="<<+serial_cmd_cnt<<", cur_has="
                                        <<has_wr_cmd[crank]<<", nxt_has="<<has_rd_cmd[nrank]);
                                for (size_t i = 0; i < NUM_RANKS; i ++) {
                                    PRINTN(", R"<<i<<"R="<<*r_rank_mux[i]<<", R"<<i<<"W="<<*w_rank_mux[i]);
                                }
                                PRINTN(endl);
                            }
                            serial_cmd_cnt = 0x0;
                            rwgrp_ch_cmd_cnt = 0x0;
                            real_rk_grp_state = crank << 1 | ctype;
                        }
                    }
                }
            }
            break;
        }
        default: {
            ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] no such rw group type: "<<+rk_grp_state);
            assert(0);
        }
    }

    sch_tout_cmd = false;
}
#endif

void MemoryController::rank_group_weight(unsigned * rank, unsigned * type) {
    vector <unsigned> weight;
    weight.resize(NUM_RANKS * 2);
    // 0->R0R, 1->R0W, 2->R1R, 3->R1W
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        // Read command weight high
        weight[i * NUM_RANKS] = (*r_rank_mux[i] == 0) ? 0 : *r_rank_mux[i] + 24;
        weight[i * NUM_RANKS + 1] = *w_rank_mux[i];
        // Same rank weight high, will reduce rank switch to reduce power consumption 
        if (real_rk_grp_state != NO_RGRP && unsigned(real_rk_grp_state >> 1) == i) {
            if (*r_rank_mux[i] != 0) weight[i * NUM_RANKS] += 4;
            if (*w_rank_mux[i] != 0) weight[i * NUM_RANKS + 1] += 4;
        }
    }

    unsigned bigger = 0;
    for (size_t i = 0; i < 2; i ++) {
        for (size_t j = 0; j < NUM_RANKS; j ++) {
            size_t index = j * 2 + i;
            if (index == real_rk_grp_state) continue;
            if (weight[index] > bigger) {
                bigger = weight[index];
                *type = i;
                *rank = j;
            }
        }
    }
}

void MemoryController::update_rwgroup_state() {
    if (GRP_RANK_EN || !GRP_RW_EN) return;

    bool has_read_cmd = false;
    bool has_write_cmd = false;
    for (auto &trans : transactionQueue) {
        unsigned sub_channel = (trans->bankIndex % NUM_BANKS) / sc_bank_num;
        if (trans->addrconf) continue;
        if (trans->pre_act) continue;
        if (now() < trans->enter_que_time) continue;
        if (refreshALL[trans->rank][sub_channel].refreshing) continue;
        //if (refreshPerBank[trans->bankIndex].refreshing) continue;
        if (trans->bp_by_tout) continue;
        if (trans->transactionType == DATA_READ) has_read_cmd = true;
        else if (trans->data_ready_cnt == (trans->burst_length + 1)) has_write_cmd = true;
    }

    bool nonflat_r2w_pre, nonflat_w2r_pre;
    bool stat_quc_rhit_high = HARDWARE_RHIT_EN && act_cmd_num <= RHIT_ACT_CMD_NUM;
    bool high_qos_trig = RCMD_HQOS_W2R_SWITCH_EN &&
            ((unsigned)accumulate(que_read_highqos_cnt.begin(), que_read_highqos_cnt.end(), 0) >= RCMD_HQOS_W2R_RLEVELH);
    if (RCMD_HQOS_RANK_SWITCH_EN) {
        for (size_t i = 0; i < NUM_RANKS; i ++) {
            if (rank_cmd_high_qos[i]) rank_cmd_high_qos[i] = que_read_highqos_cnt[i] >= RCMD_HQOS_RANK_SWITCH_LEVELL;
            else rank_cmd_high_qos[i] = que_read_highqos_cnt[i] >= RCMD_HQOS_RANK_SWITCH_LEVELH;
        }
    }

    switch (rw_group_state.back()) {
        case NO_GROUP: {
            if (GetDmcQsize() >= ENGRP_LEVEL) {
                if (que_write_cnt >= CMD_WLEVELH || !has_read_cmd) {
                    rw_group_state.push_back(WRITE_GROUP);
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- GRP_CHG :: NG to WG, Qsize="<<GetDmcQsize()
                                <<", =que_write_cnt"<<+que_write_cnt<<", has_read_cmd="<<has_read_cmd<<endl);
                    }
                    in_write_group = false;
                } else {
                    rw_group_state.push_back(READ_GROUP);
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- GRP_CHG :: NG to RG, Qsize="<<GetDmcQsize()
                                <<", que_write_cnt="<<+que_write_cnt<<", has_read_cmd="<<has_read_cmd<<endl);
                    }
                    in_write_group = true;
                }
            }
            break;
        }
        case READ_GROUP: {
            if (DMC_V580 || DMC_V590) {
                nonflat_r2w_pre = stat_quc_rhit_high ? (que_read_cnt < CMD_RLEVELL) :
                        (que_write_cnt >= CMD_WLEVELH && que_read_cnt <= RLEVEL_R2W);
            } else { // DMC V570
                nonflat_r2w_pre = que_write_cnt >= CMD_WLEVELH;
            }
            if (GetDmcQsize() < EXGRP_LEVEL && availability <= RWGRP_AUTO_BW) {
                rw_group_state.push_back(NO_GROUP);
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- GRP_CHG :: RG to NG, Qsize="<<GetDmcQsize()<<endl);
                }
                serial_cmd_cnt = 0x0;
                rwgrp_ch_cmd_cnt = 0x0;
            } else if (sch_tout_cmd && sch_tout_type != DATA_READ) { // Not used now
                rw_group_state.push_back(WRITE_GROUP);
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- GRP_CHG :: RG to WG by timeout command,"
                            <<" serial_cmd_cnt="<<+serial_cmd_cnt<<", que_read_cnt="<<+que_read_cnt
                            <<", que_write_cnt="<<+que_write_cnt<<endl);
                }
                serial_cmd_cnt = 0x0;
                rwgrp_ch_cmd_cnt = 0x0;
                in_write_group = true;
            } else if (((serial_cmd_cnt >= SERIAL_RLEVELL && nonflat_r2w_pre) ||
                    serial_cmd_cnt >= SERIAL_RLEVELH || !has_read_cmd) && has_write_cmd && !high_qos_trig) {
                rw_group_state.push_back(WRITE_GROUP);
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- GRP_CHG :: RG to WG, serial_cmd_cnt="
                            <<+serial_cmd_cnt<<", que_read_cnt="<<+que_read_cnt<<", que_write_cnt="
                            <<+que_write_cnt<<", has_read_cmd="<<has_read_cmd<<", has_write_cmd="
                            <<has_write_cmd<<endl);
                }
                serial_cmd_cnt = 0x0;
                rwgrp_ch_cmd_cnt = 0x0;
                in_write_group = false;
            } else if (in_write_group && rwgrp_ch_cmd_cnt >= RW_GRPCHG_W2R_TH && !high_qos_trig) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- GRP_CHG :: RG rwgrp_ch_cmd_cnt="<<+rwgrp_ch_cmd_cnt<<endl);
                }
                in_write_group = false;
                rwgrp_ch_cmd_cnt = 0;
            }
            break;
        }
        case WRITE_GROUP: {
            if (DMC_V580 || DMC_V590) {
                nonflat_w2r_pre = stat_quc_rhit_high ? (que_read_cnt > CMD_RLEVELH) : ((que_write_cnt <= CMD_WLEVELL)
                        || ((que_read_cnt + que_write_cnt) <= ALEVEL_W2R && (IS_HBM2E || IS_HBM3)));
            } else { // DMC V570
                nonflat_w2r_pre = que_write_cnt <= CMD_WLEVELL;
            }
            if (GetDmcQsize() < EXGRP_LEVEL && availability <= RWGRP_AUTO_BW) {
                rw_group_state.push_back(NO_GROUP);
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- GRP_CHG :: WG to NG, Qsize="<<GetDmcQsize()<<endl);
                }
                serial_cmd_cnt = 0x0;
                rwgrp_ch_cmd_cnt = 0x0;
            } else if (sch_tout_cmd && sch_tout_type == DATA_READ) { // Not used now
                rw_group_state.push_back(READ_GROUP);
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- GRP_CHG :: WG to RG by timeout command,"
                            <<" serial_cmd_cnt="<<+serial_cmd_cnt<<", que_read_cnt="<<+que_read_cnt
                            <<", que_write_cnt="<<+que_write_cnt<<endl);
                }
                serial_cmd_cnt = 0x0;
                rwgrp_ch_cmd_cnt = 0x0;
                in_write_group = false;
            } else if (high_qos_trig && has_read_cmd) {
                rw_group_state.push_back(READ_GROUP);
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- GRP_CHG :: WG to RG, high qos trigger, high_qos_trig="<<high_qos_trig<<endl);
                }
                serial_cmd_cnt = 0x0;
                rwgrp_ch_cmd_cnt = 0x0;
                in_write_group = true;
                if (high_qos_trig) highqos_trig_grpsw_cnt ++;
            } else if (((serial_cmd_cnt >= SERIAL_WLEVELL && nonflat_w2r_pre) ||
                    serial_cmd_cnt >= SERIAL_WLEVELH || !has_write_cmd) && has_read_cmd) {
                rw_group_state.push_back(READ_GROUP);
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- GRP_CHG :: WG to RG, serial_cmd_cnt="
                            <<+serial_cmd_cnt<<", que_read_cnt="<<+que_read_cnt<<", que_write_cnt="
                            <<+que_write_cnt<<", has_read_cmd="<<has_read_cmd<<", has_write_cmd="
                            <<has_write_cmd<<endl);
                }
                serial_cmd_cnt = 0x0;
                rwgrp_ch_cmd_cnt = 0x0;
                in_write_group = true;
            } else if (!in_write_group && rwgrp_ch_cmd_cnt >= RW_GRPCHG_R2W_TH) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- GRP_CHG :: WG rwgrp_ch_cmd_cnt="<<+rwgrp_ch_cmd_cnt<<endl);
                }
                in_write_group = true;
                rwgrp_ch_cmd_cnt = 0;
            }
            break;
        }
        default: {
            ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] no such rw group type: "<<rw_group_state.back());
            assert(0);
        }
    }
    if (rw_cmd_num >= RHIT_RW_CMD_NUM) {
        rw_cmd_num = 0;
        act_cmd_num = 0;
    }
    if (rw_group_state.size() > 8) {
        rw_group_state.erase(rw_group_state.begin());
    } else {
        rw_group_state.push_back(rw_group_state.back());
        rw_group_state.erase(rw_group_state.begin());
    }
    sch_tout_cmd = false;
}

void MemoryController::communicate() {
}

void MemoryController::update_que() {
    size_t len = transactionQueue.size();
    uint8_t delete_cnt = 0;
    for (size_t i = 0; i < len; i++) {
        size_t real_i = i - delete_cnt;
        Transaction *t = transactionQueue.at(real_i);
        if (t->issue_size < t->data_size) continue;
        if (t->pre_act) {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- DELETE PRE_ACT:: task="<<t->task<<", bank="
                        <<t->bankIndex<<", row"<<t->row<<endl);
            }
            t->reset();
            delete t;
            pre_act_cnt --;
            delete_cnt ++;
            rank_pre_act_cnt[t->rank]--;
            transactionQueue.erase(transactionQueue.begin() + real_i);
        }
    }

    len = transactionQueue.size();
    for (size_t i = 0; i < len; i++) {
        Transaction *t = transactionQueue[i];
        unsigned sub_channel = (t->bankIndex % NUM_BANKS) / sc_bank_num;
//        unsigned bank_start = sub_channel * NUM_BANKS /sc_num;
//        unsigned bank_pair_start = sub_channel * pbr_bank_num;
        if (t->issue_size < t->data_size) continue;
        if (t->issue_size > t->data_size) {
            ERROR(setw(10)<<now()<<" -- Error issue_size, task="<<t->task<<" data_size="
                    <<t->data_size<<" issue_size="<<t->issue_size);
            assert(0);
        }

        if (t->transactionType == DATA_READ) {
            r_rank_cnt[t->rank] --;
            r_rank_bst[t->rank] -= ceil(float(t->data_size) / max_bl_data_size);
            que_read_cnt --;
            r_bank_cnt[t->bankIndex] --;
            r_bg_cnt[t->rank][t->group] --;
            r_sid_cnt[t->rank][t->sid] --;
            r_qos_cnt[t->qos] --;
            if (RDATA_TYPE == 1 && t->mask_wcmd==false) {
                if (!IECC_ENABLE || !tasks_info[t->task].rd_ecc) {
                    gen_rresp(t->task);
                }
            }
            if (!GRP_RANK_EN && GRP_RW_EN && !t->timeout) {
                if (rw_group_state[0] == READ_GROUP) serial_cmd_cnt ++;
                else if (rw_group_state[0] == WRITE_GROUP) rwgrp_ch_cmd_cnt ++;
            }
            //remove conflict
            for (size_t j = i + 1; j < len; j++) {
                Transaction *tmp = transactionQueue[j];
//                if (tmp->task == t->task) continue;
                if (tmp->addrconf && ((tmp->address & ~ALIGNED_SIZE) == (t->address & ~ALIGNED_SIZE)) &&
                        tmp->transactionType != DATA_READ && (tmp->addr_block_source_id == t->task)) {
                    tmp->addrconf = false;
                    if (DEBUG_BUS) {
                         PRINTN(setw(10)<<now()<<" -- RL_CONF :: release ,task="<<tmp->task<<endl);
                    }
                }
            }
            if (t->qos <= SWITCH_HQOS_LEVEL) {
                que_read_highqos_cnt[t->rank] --;
                highqos_r_bank_cnt[t->bankIndex] --;
            }
        } else {
            w_rank_cnt[t->rank] --;
            w_rank_bst[t->rank] -= ceil(float(t->data_size) / max_bl_data_size);
            if (!IECC_ENABLE || !tasks_info[t->task].wr_ecc) {
                gen_wresp(t->task);
            }
            que_write_cnt --;
            w_qos_cnt[t->qos] --;
            if (!GRP_RANK_EN && GRP_RW_EN && !t->timeout) {
                if (rw_group_state[0] == READ_GROUP) rwgrp_ch_cmd_cnt ++;
                else if (rw_group_state[0] == WRITE_GROUP) serial_cmd_cnt ++;
            }
            w_bank_cnt[t->bankIndex] --;
            w_bg_cnt[t->rank][t->group] --;
            w_sid_cnt[t->rank][t->sid] --;
            //remove conflict
            for (size_t j = i + 1; j < len; j++) {
                Transaction *tmp = transactionQueue[j];
//                if (tmp->task == t->task) continue;
                if (tmp->addrconf && ((tmp->address & ~ALIGNED_SIZE) == (t->address & ~ALIGNED_SIZE))
                        && (tmp->addr_block_source_id == t->task)) {
                    tmp->addrconf = false;
                    if (DEBUG_BUS) {
                         PRINTN(setw(10)<<now()<<" -- RL_CONF :: release ,task="<<tmp->task<<endl);
                    }
                }
            }
        }

        if (RHIT_BREAK_EN && bankStates[t->bankIndex].row_miss_cnt != 0) bankStates[t->bankIndex].ser_rhit_cnt ++;
        if (WRITE_BUFFER_ENABLE) wb->dmc_release_conflict(t);
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- DELETE :: task="<<t->task<<", bank="
                    <<t->bankIndex<<", rowhit="<<!t->act_executing<<endl);
        }

        rank_cnt[t->rank] --;
        sc_cnt[t->rank][sub_channel] --;
        bank_cnt[t->bankIndex] --;
        bg_cnt[t->rank][t->group] --;
        sid_cnt[t->rank][t->sid] --;
        bankStates[t->bankIndex].row_hit_cnt --;
        if (t->timeout) {
            dmc_timeout_cnt ++;
            qos_timeout_cnt[t->qos] ++;
            if (t->cmd_rt_type) RtCmdCnt ++;
        }

        if (ENH_PAGE_ADPT_EN && !t->has_send_act) bank_cnt_ehs[t->bankIndex] ++;

        t->reset();
        delete t;

        if (!SLOT_FIFO) {
            unsigned slt_valid_cnt = 0;
            for (size_t j = 0; j < TRANS_QUEUE_DEPTH; j ++) {
                if (!slt_valid[j]) continue;
                if (slt_valid_cnt == i) {
                    slt_valid[j] = false;
                    break;
                } else {
                    slt_valid_cnt ++;
                }
            }
        }

        transactionQueue.erase(transactionQueue.begin() + i);
        dmc_cmd_cnt ++;
        rw_cmd_num ++;
        if (GRP_RANK_EN && !t->timeout && rk_grp_state != NO_RGRP) {
            if (rk_grp_state == real_rk_grp_state) {
                serial_cmd_cnt ++;
            } else {
                rwgrp_ch_cmd_cnt ++;
            }
        }
        break;
    }
}

void MemoryController::lqos_label() {
    // backpress lqos cmd (>=LOQS_BP_LEVEL)
    lqos_bp = false;
    lqos_rd_bp = false;
    lqos_wr_bp = false;
    bool lqos_not_all_rd = false;
    bool lqos_not_all_wr = false;
    for (auto &trans : transactionQueue) {
        trans->lqos_bp = false;
    }
    for (auto &trans : transactionQueue) {
        if (trans->qos < LQOS_BP_LEVEL) {
            lqos_bp = true;
            break;
        }
    }
    if (lqos_bp) {
        for (auto &trans : transactionQueue) {
            if (trans->qos >= LQOS_BP_LEVEL) {
                trans->lqos_bp = true;
            } else {
                trans->lqos_bp = false;
            }
        }
    }

    for (auto &trans : transactionQueue) {
        if (trans->qos < LQOS_BP_LEVEL && trans->transactionType == DATA_READ) {
            lqos_not_all_rd = true;
        } else if (trans->qos < LQOS_BP_LEVEL && trans->transactionType == DATA_WRITE) {
            lqos_not_all_wr = true;
        }
    }

    if (lqos_not_all_rd) {
        lqos_rd_bp = true;
    } else if (!lqos_not_all_rd && !lqos_not_all_wr) {
        lqos_rd_bp = true;
    }
    if (lqos_not_all_wr) {
        lqos_wr_bp = true;
    } else if (!lqos_not_all_rd && !lqos_not_all_wr) {
        lqos_wr_bp = true;
    }
}

void MemoryController::que_pipeline() {
    for (size_t i = 0; i < NUM_RANKS * NUM_BANKS; i ++) {
        bankStates[i].has_cmd_rowhit = false;
        bankStates[i].hold_precharge = false;
        bankStates[i].has_timeout = false;
        bankStates[i].has_rhit_break = false;
        bankStates[i].has_highqos_cmd_rowhit = false;
    }
    rw_exec_cnt = 0;
    for (auto &trans : transactionQueue) {
        if (trans->issue_size != 0) rw_exec_cnt ++;
    }
    
    //label lqos cmd ( pri lower than LQOS_BP_LEVEL)
    if (LQOS_BP_EN) {    
        lqos_label();
    }

    check_timeout_and_aging();
    if (dfs_backpress_en) total_dfs_bp_cnt ++;

    for (auto &t : transactionQueue) {
        unsigned t_state = (t->rank << 1) | t->transactionType;
        unsigned sub_channel = (t->bankIndex % NUM_BANKS) / sc_bank_num;
        if (t->addrconf) continue;
        if (t->pre_act) continue;
        if (now() < t->arb_time) continue;
        if (t->bp_by_tout) continue;
        if (refreshALL[t->rank][sub_channel].refreshing) continue;    //todo: revise for e-mode
        //if (refreshPerBank[t->bankIndex].refreshing) continue;
        if (bankStates[t->bankIndex].state->currentBankState == RowActive &&
                t->row == bankStates[t->bankIndex].state->openRowAddress) {
            if ((rw_group_state[0] == READ_GROUP && t->transactionType == DATA_READ) || (rk_grp_state == t_state)
                    || (rw_group_state[0] == WRITE_GROUP && t->transactionType == DATA_WRITE)) {
                bankStates[t->bankIndex].has_cmd_rowhit = true;
            }
            if (rw_group_state[0] == READ_GROUP && t->qos <= SWITCH_HQOS_LEVEL && t->transactionType == DATA_READ) {
                bankStates[t->bankIndex].has_highqos_cmd_rowhit = RHIT_HQOS_BREAK_EN;
            }
        }
    }

    for (auto &trans : transactionQueue) {
        if (TIME_ASSERT_EN) {
            if ((now() - trans->timeAdded) > 1000000) {
                ERROR(setw(10)<<now()<<" -- task="<<trans->task<<" address="<<hex<<trans->address<<dec
                        <<" rank="<<trans->rank<<" bank="<<trans->bankIndex<<" row="<<trans->row<<" matgrp="
                        <<(trans->row&(NUM_MATGRPS-1)));
                ERROR(setw(10)<<now()<<" -- error, qos="<<trans->qos<<", pri="<<trans->pri);
                ERROR(setw(10)<<now()<<" -- FATAL ERROR == big latency"<<", chnl:"<<channel);
                assert(0);
            }

            if (trans->transactionType != DATA_READ) {
                if (now() - trans->timeAdded > 100000 && trans->data_ready_cnt <= trans->burst_length) {
                    ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] task="<<trans->task<<" Wdata number miss match, EXP="
                            <<trans->burst_length<<", ACT="<<trans->data_ready_cnt);
                    assert(0);
                }
            }
        }

        if ((RHIT_BREAK_EN && bankStates[trans->bankIndex].ser_rhit_cnt >= RHIT_BREAK_LEVEL) ||
                (RHIT_HQOS_BREAK_EN && rw_group_state[0] == READ_GROUP &&
                 highqos_r_bank_cnt[trans->bankIndex] >= RHIT_HQOS_BREAK_OTH_RCMD_LEVEL)) {
            if (bankStates[trans->bankIndex].state->currentBankState == RowActive &&
                    trans->row == bankStates[trans->bankIndex].state->openRowAddress &&
                    !bankStates[trans->bankIndex].has_highqos_cmd_rowhit) {
                bankStates[trans->bankIndex].has_rhit_break = true;
            }
        }
    }

    for (size_t bank = 0; bank < NUM_RANKS * NUM_BANKS; bank ++) {
        if (bankStates[bank].state->currentBankState != RowActive) continue;
        if (rank_cnt[bank / NUM_BANKS] == 0) continue;
        if (bank_cnt[bank] == 0) continue;
        for (auto &trans : transactionQueue) {
            if (trans->addrconf) continue;
            if (trans->pre_act) continue;
            if (now() < trans->arb_time) continue;
            if (trans->bp_by_tout) continue;
            if (bank != trans->bankIndex) continue;
            if (trans->row != bankStates[bank].state->openRowAddress) continue;
            if (bankStates[bank].has_rhit_break) continue;
            if ((rw_group_state[0] == NO_GROUP && !GRP_RANK_EN) || (rk_grp_state == NO_RGRP && !GRP_RW_EN)) {
                bankStates[bank].hold_precharge = true;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- HPRE :: NO_GROUP, task="
                            <<trans->task<<", bank="<<trans->bankIndex<<endl);
                }
                break;
            } else if (rw_group_state[0] == READ_GROUP && trans->transactionType == DATA_READ &&
                    bankStates[bank].state->lastCmdType == DATA_READ) {
                bankStates[bank].hold_precharge = true;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- HPRE :: READ_GROUP, task="
                            <<trans->task<<", bank="<<trans->bankIndex<<endl);
                }
                break;
            } else if (rw_group_state[0] == WRITE_GROUP && trans->transactionType != DATA_READ &&
                    bankStates[bank].state->lastCmdType != DATA_READ) {
                bankStates[bank].hold_precharge = true;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- HPRE :: WRITE_GROUP, task="
                            <<trans->task<<", bank="<<trans->bankIndex<<endl);
                }
                break;
            } else if (rk_grp_state != NO_RGRP && unsigned(rk_grp_state>>1) == trans->rank &&
                    (rk_grp_state&1) == trans->transactionType &&
                    trans->transactionType == bankStates[bank].state->lastCmdType) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- HPRE :: RGROUP, task="
                            <<trans->task<<", bank="<<trans->bankIndex<<", rk_grp_state="
                            <<+rk_grp_state<<endl);
                }
                break;
            }
        }
    }

    if (DMC_V596) {
        for (auto &trans : transactionQueue) {
            if (trans->addrconf) continue;
            if (trans->pre_act) continue;
            if (now() < trans->arb_time) continue;
            if (!bankStates[trans->bankIndex].hold_precharge) continue;
            if (bankStates[trans->bankIndex].state->currentBankState != RowActive) continue;
            if (trans->row == bankStates[trans->bankIndex].state->openRowAddress) continue;
            if (tout_high_pri <= trans->pri) {
                bankStates[trans->bankIndex].hold_precharge = false;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- HPRE :: Timeout set hold_precharge false, task="
                            <<trans->task<<", bank="<<trans->bankIndex<<endl);
                }
            }
        }
        if (DMC_V580 && GRP_RW_EN && rw_group_state[0] == NO_GROUP &&
                (PreCmd.type == READ_CMD || PreCmd.type == READ_P_CMD)) {
            for (auto &trans : transactionQueue) {
                if (trans->addrconf) continue;
                if (trans->pre_act) continue;
                if (!bankStates[trans->bankIndex].hold_precharge) continue;
                if (now() < trans->arb_time) continue;
                if (trans->transactionType == DATA_READ &&
                        bankStates[trans->bankIndex].state->lastCmdType != DATA_READ) {
                    bankStates[trans->bankIndex].hold_precharge = false;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- HPRE :: ReadNoGroup set hold_precharge false,"
                                <<" task="<<trans->task<<", bank="<<trans->bankIndex<<endl);
                    }
                }
            }
        }
    }

    if (RCMD_HQOS_RANK_SWITCH_EN) {
        for (size_t i = 0; i < NUM_RANKS; i ++) rank_rhit_num[i] = 0;

        for (auto &trans : transactionQueue) {
            if (trans->transactionType == DATA_READ && bankStates[trans->bankIndex].state->openRowAddress == trans->row
                    && bankStates[trans->bankIndex].state->currentBankState == RowActive) {
                rank_rhit_num[trans->rank] ++;
            }
        }
        for (size_t i = 0; i < NUM_RANKS; i ++) {
            rank_cmd_high_qos[i] = rank_cmd_high_qos[i] && (rank_rhit_num[i] >= RCMD_HQOS_RANK_SWITCH_ROWHIT_LEVEL);
        }
    }

    table_use_cnt = 0;
    for (auto &state : bankStates) {
        if (state.state->currentBankState == RowActive || state.state->currentBankState == Precharging)
            table_use_cnt ++;
    }

    for (size_t bank_idx = 0; bank_idx < NUM_RANKS * NUM_BANKS; bank_idx ++) {
        issue_state[bank_idx] = false;
    }
    
    uint64_t cmd_met_pbr_cnt = 0;
    if (!SLOT_FIFO) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- Current Arb Group0 Priority="<<arb_group_pri[0]<<" -- Arb Group1 Priority="<<arb_group_pri[1]
                    <<" -- Arb Group2 Priority="<<arb_group_pri[2]<<" -- Arb Group3 Priority="<<arb_group_pri[3]<<endl);
        }
        // select tran from high rr pri to low rr pri
        for (size_t i=0; i<4; i++) {
            for (size_t j=0; j<4; j++){
                if (arb_group_pri[j] == (3-i)) {
                    for (auto &trans: transactionQueue) {
                        if (trans->arb_group == j) {
                            if (dfs_backpress_en) {
                                trans->timeAdded ++;
                                continue;
                            }

                            if (arb_enable && now() >= trans->arb_time && even_cycle) lc(trans);
                            
                            if ((refreshPerBank[trans->bankIndex].refreshWaiting || refreshPerBank[trans->bankIndex].refreshing)
                                    && trans->issue_size == 0 && !trans->act_executing) {
                                cmd_met_pbr_cnt ++;
                            }

                            if (trans->issue_size != 0) {
                                issue_state[trans->bankIndex] = true;
                            }
                        }
                    } 
                }
            }
        }
    } else {    
        for (auto &trans : transactionQueue) {
            if (dfs_backpress_en) {
                trans->timeAdded ++;
                continue;
            }

            if (arb_enable && now() >= trans->arb_time && even_cycle) lc(trans);
            
            if ((refreshPerBank[trans->bankIndex].refreshWaiting || refreshPerBank[trans->bankIndex].refreshing)
                    && trans->issue_size == 0 && !trans->act_executing) {
                cmd_met_pbr_cnt ++;
            }

            if (trans->issue_size != 0) {
                issue_state[trans->bankIndex] = true;
            }
        }
    }

//    uint64_t cmd_met_pbr_cnt = 0;
//    for (auto &trans : transactionQueue) {
//        if (dfs_backpress_en) {
//            trans->timeAdded ++;
//            continue;
//        }
//
//        if (arb_enable && now() >= trans->arb_time && even_cycle) lc(trans);      // every other command, even
//
//        if ((refreshPerBank[trans->bankIndex].refreshWaiting || refreshPerBank[trans->bankIndex].refreshing)
//                && trans->issue_size == 0 && !trans->act_executing) {
//            cmd_met_pbr_cnt ++;
//        }
//
//        if (trans->issue_size != 0) {
//            issue_state[trans->bankIndex] = true;
//        }
//
//    }

    // cycle when refreshing or refreshWaiting
    bool have_bank_ref = false;
//    DEBUG(now()<<" after lc, the number of Cmd in transactionQueue="<<transactionQueue.size());
    for (auto &trans : transactionQueue) {
        if ((refreshPerBank[trans->bankIndex].refreshWaiting || refreshPerBank[trans->bankIndex].refreshing)) {
            have_bank_ref = true;
        }
    }
    if (have_bank_ref) {
        pbr_cycle ++ ;
    }

    // cycle when all commands blocked by pbr
    if (cmd_met_pbr_cnt == transactionQueue.size() && cmd_met_pbr_cnt!=0){
        pbr_block_allcmd_cycle ++;
    }
}

void MemoryController::page_timeout_policy() {
    if (!PAGE_TIMEOUT_EN) return;

    for (auto &state : bankStates) {
        unsigned sub_channel = (state.bankIndex % NUM_BANKS) / sc_bank_num;
        if (EM_ENABLE && EM_MODE==2 && state.rank==1 && sub_channel==1) continue;      //rank1, sc1 forbidden under combo e-mode
        bank_cas_delay[state.bankIndex] ++;
        if ((w_bank_cnt[state.bankIndex] + r_bank_cnt[state.bankIndex]) > 0) continue;
        if (state.state->currentBankState != RowActive) continue;
        if ((access_bank_delay[state.bankIndex].cnt >= page_timeout_rd[state.bankIndex] &&
                state.state->lastCmdType == DATA_READ) || (state.state->lastCmdType == DATA_WRITE &&
                access_bank_delay[state.bankIndex].cnt >= page_timeout_wr[state.bankIndex])) {
//            if ((now() + 1) >= state.state->nextPrecharge && tFPWCountdown[state.rank].size() < 4) {
            if ((now() + 1) >= state.state->nextPrecharge && tFPWCountdown[state.rank].size() < 4 && even_cycle) {   //every other command, even;
                if (arb_enable) {
                    funcState[state.rank].wakeup = true;
                    if (RankState[state.rank].lp_state == IDLE) {
                        Cmd *c = new Cmd;
                        c->rank = state.rank;
//                        c->channel = sub_channel;
                        c->group = state.group;
                        c->bank = state.bank;
                        c->row = state.state->openRowAddress;
                        c->cmd_type = PRECHARGE_PB_CMD;
                        c->bankIndex = state.bankIndex;
                        c->cmd_source = 1;
                        c->task = 0xFFFFFFFFFFFFFFF;
                        CmdQueue.push_back(c);
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- REQ :: Page Timeout, precharge bank="
                                    <<c->bankIndex<<", task="<<c->task<<endl);
                        }
                    } else {
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- REQ_LP :: hold PRECHARGE in"
                                    <<" lp state, rank"<<state.rank<<", bank="<<state.bankIndex<<endl);
                        }
                    }
                }
            }
        } else if (access_bank_delay[state.bankIndex].enable) {
            access_bank_delay[state.bankIndex].cnt ++;
        }
    }
}

/***************************************************************************************************
descriptor: The purpose of this function is receive the data packet and transmit packet
****************************************************************************************************/
void MemoryController::data_fresh() {
    for (auto &cmd : CmdQueue) {
        if (RankState[cmd->rank].lp_state != IDLE) {
            ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] Send command in lp state, cmd_type="
                    <<cmd->cmd_type<<", task="<<cmd->task<<", rank="<<cmd->rank
                    <<", bank="<<cmd->bankIndex);
            assert(0);
        }
    }

    unsigned size = writeDataToSend.size();
    if (size > 0) {
        for (size_t i = 0; i < size; i ++) {
            if (writeDataToSend[i].delay > 0)
                writeDataToSend[i].delay --;
        }

        if (writeDataToSend[0].delay == 0) {
            //send to bus and print debug stuff
            auto it = wdata_info.find(writeDataToSend[0].task);
            if (it != wdata_info.end()) {
                wdata_info[writeDataToSend[0].task]--;
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- WDATA CHECK :: , task="<<writeDataToSend[0].task<<", task_cnt="<<wdata_info[writeDataToSend[0].task]<<endl);
            }
            if (wdata_info[writeDataToSend[0].task] == 0) {
                wdata_info.erase(writeDataToSend[0].task);
                if (IECC_ENABLE) tasks_info[writeDataToSend[0].task].wr_finish = true;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- WDATA CHECK :: WR FINISH"<<endl);
                }
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- T_DDR :: write data on Data Bus, task="
                        <<writeDataToSend[0].task<<endl);
            }
            writeDataToSend.erase(writeDataToSend.begin());
        }
    }

    //check for outstanding data to return to the CPU
    size = read_data_buffer.size();
    for (size_t i = 0; i < size; i ++) {
        if (0 == read_data_buffer[i].delay) {
            unsigned long long task = read_data_buffer[i].task;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- T_CPU :: Issuing to CPU bus, task="<<task<<endl);
            }
            auto it = pending_TransactionQue.find(task);
            if (it == pending_TransactionQue.end()) {
                ERROR(setw(10)<<now()<<" -- [DMC"<<channel<<"]"<<" mismatch data, task="<<task);
                assert(0);
//                read_data_buffer.erase(read_data_buffer.begin() + i);
//                break;
            }

            TRANS_MSG msg = it->second;
            msg.burst_cnt ++;
            read_data_buffer[i].channel = msg.channel;
#ifdef SYSARCH_PLATFORM
            unsigned rdata_type = 0;
            if (msg.burst_cnt == (msg.burst_length + 1)) rdata_type |= 1; // bit[0] is rdata_end
            if (msg.burst_cnt == 0) rdata_type |= (1 << 1); // bit[1] is rdata_start
            rdata_type |= (msg.qos << 2); // bit[5:2] is qos
            rdata_type |= (msg.pf_type << 6); // bit[7:6] is pf_type
            rdata_type |= (msg.sub_pftype << 8); // bit[11:8] is pf_type
            rdata_type |= (msg.sub_src << 12); // bit[13:12] is pf_type
            msg.reqAddToDmcTime = double(rdata_type);
#endif
            if ((!IECC_ENABLE || !tasks_info[task].rd_ecc) && (!RMW_ENABLE || (!read_data_buffer[i].mask_wcmd && RMW_ENABLE))) {
                read_data_buffer[i].readDataEnterDmcTime = now() * tDFI;
                if (!returnReadData(read_data_buffer[i].channel, task,
                        read_data_buffer[i].readDataEnterDmcTime,
                        msg.reqAddToDmcTime, msg.reqEnterDmcBufTime)) {
                    if (PRINT_READ) {
                        TRACE_PRINT(setw(10)<<now()<<" -- Rdata Back Pressure :: task="<<task<<" ch="<<channel<<endl);
                    }
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- Rdata Back Pressure :: task="<<task<<" ch="<<channel<<endl);
                    }
                    return;
                } else {
                    pre_rdata_time = now();
                    rdata_cnt ++;
                    if (PRINT_READ) {
                        TRACE_PRINT(setw(10)<<now()<<" -- Rdata Received :: task="<<task<<", latency="
                                <<ceil(((read_data_buffer[i].readDataEnterDmcTime
                                - msg.reqEnterDmcBufTime) / tDFI))<<" ch="<<channel<<endl);
                    }
                    if (PRINT_IDLE_LAT) {
                        DEBUG(setw(10)<<now()<<" -- Rdata Received :: task="<<task<<", latency="
                                <<ceil(((read_data_buffer[i].readDataEnterDmcTime
                                - msg.reqEnterDmcBufTime) / tDFI)));
                    }
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- Rdata Received :: task="<<task<<", latency="
                                <<ceil(((read_data_buffer[i].readDataEnterDmcTime
                                - msg.reqEnterDmcBufTime) / tDFI))<<endl);
                    }
                }
            }
            if (msg.burst_cnt == (msg.burst_length + 1)) {
                ReturnData_statistics(task, msg.time, msg.qos, msg.mid, msg.pf_type, msg.rank);
            }
            //return latency
            if (msg.burst_cnt == (msg.burst_length + 1)) {
                pending_TransactionQue.erase(task);
                if (IECC_ENABLE) tasks_info[task].rd_finish = true;
                if (RMW_ENABLE && read_data_buffer[i].mask_wcmd==true)  {
                    auto iter = rmw_rd_finish.find(task);
                    if (iter == rmw_rd_finish.end()){
                        ERROR(setw(10)<<now()<<" -- Merge Read Data Mismatch, task="<<task);
                        assert(0);
                    }
                    rmw_rd_finish[task] = true;
                }
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- Finish :: Issuing to CPU bus, task="<<task<<endl);
                }
            } else {
                it->second.burst_cnt = msg.burst_cnt;
            }
            read_data_buffer.erase(read_data_buffer.begin() + i);
            break;
        } else {
            if (IS_G3D) break;
        }
    }
    size = read_data_buffer.size();
    for (size_t i = 0; i < size; i++) {
        if (read_data_buffer[i].delay > 0) read_data_buffer[i].delay--;
    }
}

/***************************************************************************************************
descriptor: update state var pre
****************************************************************************************************/
void MemoryController::update_state_pre() {
    for (auto &state : bankStates) {
        unsigned state_channel = (state.bankIndex % NUM_BANKS) / sc_bank_num;
        if (EM_ENABLE && EM_MODE==2 && state_channel==1 && state.rank==1) continue;     //rank1, sc1 forbidden under combo e-mode 
        if (state.state->currentBankState == Idle) bank_idle_cnt[state.rank] ++;
        if (state.state->currentBankState == RowActive) bank_act_cnt[state.rank] ++;
    }

    for (auto &trans : transactionQueue) {
        if (now() < trans->arb_time) continue;
        if (trans->transactionType != DATA_READ) continue;
        if (RankState[trans->rank].lp_state == IDLE) continue;
        auto &st = RankState[trans->rank].lp_state;
        if (st >= PDE && st <= PDX) rd_met_pd_cnt ++;
        else if (st >= ASREFE && st <= SRPDX) rd_met_asref_cnt ++;
    }

    if (TRFC_CC_EN) {     // tdo: revise for e-mode
        bool has_pbr = false, has_abr = false;
        unsigned cc = 0, lp_cnt = 0;
        for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
            for (size_t i = 0; i < sc_num; i++){
                if (refreshALL[rank][i].refreshing) has_abr = true;
                break;
            }
//            if (refreshALL[rank].refreshing) has_abr = true;
            if (RankState[rank].lp_state != IDLE) lp_cnt ++;
            for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
                unsigned bank_tmp = rank * NUM_BANKS + bank;
                if (refreshPerBank[bank_tmp].refreshing) has_pbr = true;
                if (bankStates[bank_tmp].state->currentBankState == RowActive) break;
                if (bankStates[bank_tmp].state->currentBankState == Precharging) break;
                cc ++;
            }
        }
        if ((cc == NUM_RANKS * NUM_BANKS) && (lp_cnt != NUM_RANKS)) {
            if (has_abr) abr_cc_cnt ++;
            else if (has_pbr) pbr_cc_cnt ++;
        }
    }

    bool has_casfs = false;
    for (size_t i = 0; i < NUM_RANKS; i ++) if (send_wckfs[i]) has_casfs = true;
    if (has_casfs) casfs_time ++;

    // useless?
    for (size_t rank = 0; rank < NUM_RANKS; rank++) {
        if (!pbr_hold_pre[rank]) continue;
        bool pbr_bank_open = false;
        for (size_t bank = 0; bank < NUM_BANKS; bank++) {
            unsigned sub_channel = bank / sc_bank_num;
            if (EM_ENABLE && EM_MODE==2 && rank==1 && sub_channel==1) continue;      //rank1, sc1 forbidden under combo e-mode 
            if (!refreshPerBank[rank * NUM_BANKS + bank].refreshWaiting) continue;
            if (bankStates[rank * NUM_BANKS + bank].state->currentBankState != RowActive) continue;
            pbr_bank_open = true;
            break;
        }
        if (pbr_bank_open) continue;
        if (now() >= pbr_hold_pre_time[rank]) pbr_hold_pre[rank] = false;
    }
}

/***************************************************************************************************
descriptor: update state var
****************************************************************************************************/
void MemoryController::update_state_post() {
    // Command bp erase
    uint8_t bp_size = bp_cycle.size();
    uint8_t erase_cnt = 0;
    for (size_t i = 0; i < bp_size; i ++) {
        if (now() >= bp_cycle[i - erase_cnt]) {
            bp_cycle.erase(bp_cycle.begin() + i - erase_cnt);
            erase_cnt ++;
        }
    }
}

/***************************************************************************************************
descriptor: Load tFPW
****************************************************************************************************/
void MemoryController::LoadTfpw(uint8_t rank, unsigned tfpw) {
    if (IS_GD2 && tfpw != 0) tFPWCountdown[rank].push_back(tfpw);
}

/***************************************************************************************************
descriptor: Load tFAW
****************************************************************************************************/
void MemoryController::LoadTfaw(uint8_t rank, unsigned tfaw, unsigned sc) {
    if (!IS_GD1 && !IS_G3D) {
        if (sc == 0) {   // sub channel 0
            tFAWCountdown[rank].push_back(tfaw);
        } else if (sc == 1 && EM_ENABLE) {   // sub channel 1 under e-mode
            tFAWCountdown_sc1[rank].push_back(tfaw);
        } else {
            ERROR(setw(10)<<now()<<" -- FAW Mode Config Wrong, sc="<<sc<<", e-mode="<<EM_ENABLE);
            assert(0);
        }
    }
}

/***************************************************************************************************
descriptor: generate for DDR packet.
****************************************************************************************************/
void MemoryController::generate_packet(Cmd *c) {
    //now that we know there is room in the command queue, we can remove from the transaction queue
    bool hit = false;
    unsigned sub_channel = (c->bankIndex % NUM_BANKS) / sc_bank_num;
//    unsigned bank_start = sub_channel * NUM_BANKS / sc_num; 
//    unsigned bank_pair_start = sub_channel * pbr_bank_num; 
    command.reset();
    switch (c->cmd_type) {
        case PRECHARGE_PB_CMD :
        case PRECHARGE_PB_DST_CMD : {
            if (c->fg_ref) command_pend = pre_cycle;
            else  command_pend = rw_cycle;
            LoadTfpw(c->rank, tFPW);
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- RMHOLD :: PRECHARGE_PB Remove bank="
                        <<c->bankIndex<<" Hold Precharge, task="<<c->task<<endl);
            }
            break;
        }
        case PRECHARGE_SB_CMD : {
            command_pend = pre_cycle;
            LoadTfpw(c->rank, tFPW);
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- RMHOLD :: PRECHARGE_SB Remove bank="
                        <<c->bankIndex<<" Hold Precharge, task="<<c->task<<endl);
            }
            break;
        }
        case PRECHARGE_AB_CMD : {
            command_pend = pre_cycle;
            LoadTfpw(c->rank, tFPW);
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- RMHOLD :: PRECHARGE_AB Remove rank="
                        <<c->rank<<" Hold Precharge, task="<<c->task<<endl);
            }
            break;
        }
        case READ_CMD :
        case READ_P_CMD : {
            if (c->cmd_type == READ_P_CMD && c->fg_ref) command_pend = pre_cycle;
            else if (!RankState[c->rank].wck_on) command_pend = rw_cycle * 2;
            else command_pend = rw_cycle;
            TotalBytes += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            flowStatisTotalBytes += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            bw_totalcmds += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            TotalReadBytes += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            if (IECC_ENABLE && tasks_info[c->task].rd_ecc) {
                ecc_total_bytes += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
                ecc_total_reads += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            }
            if (RMW_ENABLE && c->mask_wcmd) {
                rmw_total_bytes += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
                rmw_total_reads += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            }
            TotalBytesRank[c->rank] += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            if (c->cmd_type == READ_P_CMD) LoadTfpw(c->rank, tFPW);
            
            if (RMW_ENABLE && IECC_ENABLE && tasks_info[c->task].rd_ecc && c->mask_wcmd) {
                ERROR(setw(10)<<now()<<" -- Merge read can not coexisit with ECC read, task="<<c->task);
                assert(0);
            }
              
            break;
        }
        case WRITE_CMD :
        case WRITE_P_CMD :
        case WRITE_MASK_CMD :
        case WRITE_MASK_P_CMD : {
            if ((c->cmd_type == WRITE_P_CMD || c->cmd_type == WRITE_MASK_P_CMD) && c->fg_ref) command_pend = pre_cycle;
            else if (!RankState[c->rank].wck_on) command_pend = rw_cycle * 2;
            else command_pend = rw_cycle;
            //create read or write command and enqueue it
            DataPacket dp;
            //dp.delay = WL;
            dp.task = c->task;
            dp.bl = c->bl;
            writeDataToSend.push_back(dp);
            this->wdata_info[c->task]++;
            if (c->cmd_type == WRITE_P_CMD || c->cmd_type == WRITE_MASK_P_CMD
                    || (c->issue_size + max_bl_data_size) >= c->data_size) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- RMHOLD :: WRITE Remove bank="
                            <<c->bankIndex<<" Hold Precharge,task="<<c->task<<endl);
                }
            }
            TotalBytes += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            flowStatisTotalBytes += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            bw_totalcmds += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            bw_totalwrites += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            TotalWriteBytes += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            if (IECC_ENABLE && tasks_info[c->task].wr_ecc) {
                ecc_total_bytes += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
                ecc_total_writes += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            }
            TotalBytesRank[c->rank] += unsigned(PAM_RATIO * DmcLog2(c->bl, JEDEC_DATA_BUS_BITS)) / 8;
            if (c->cmd_type == WRITE_P_CMD) LoadTfpw(c->rank, tFPW);
            if (c->cmd_type == WRITE_MASK_P_CMD) LoadTfpw(c->rank, tFPW);
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- RMHOLD :: Remove bank="
                        <<c->bankIndex<<" Hold Precharge,task="<<c->task<<endl);
            }
            break;
        }
        case ACTIVATE1_CMD :
        case ACTIVATE1_DST_CMD : {
            command_pend = cmd_cycle;
            break;
        }
        case ACTIVATE2_CMD: {
            command_pend = cmd_cycle;
            if (DEBUG_BUS) {
                if (c->pre_act) {
                    PRINTN(setw(10)<<now()<<" -- HOLD :: bank="<<c->bankIndex
                            <<" PreAct Not Hold Precharge, task="<<c->task<<endl);
                } else {
                    PRINTN(setw(10)<<now()<<" -- HOLD :: bank="<<c->bankIndex
                            <<" Hold Precharge, task="<<c->task<<endl);
                }
            }

            if (c->type == DATA_READ) bankStates[c->bankIndex].write_hold = false;
            else bankStates[c->bankIndex].write_hold = true;
            //if its an activate, add a t_faw counter
            LoadTfaw(c->rank, tFAW, sub_channel);
            break;
        }
        case REFRESH_CMD : {
            command_pend = cmd_cycle;
            break;
        }
        case PER_BANK_REFRESH_CMD : {
            command_pend = cmd_cycle;
            LoadTfaw(c->rank, tFAW, sub_channel);
            if (ENH_PBR_EN) {
                bankStates[c->fst_bankIndex].finish_refresh_pb = true;
                bankStates[c->lst_bankIndex].finish_refresh_pb = true;
            } else {
                for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++) {
                    bankStates[c->bankIndex + sbr_bank * pbr_bank_num].finish_refresh_pb = true;
                }
            }
            break;
        }
        case ACTIVATE2_DST_CMD : {
            command_pend = cmd_cycle;
            LoadTfaw(c->rank, tFAW, sub_channel);
            break;
        }
        default : break;
    }

    if (c->cmd_type == READ_CMD || c->cmd_type == READ_P_CMD || c->cmd_type == WRITE_CMD || c->cmd_type == WRITE_P_CMD) {
        for (auto &t : transactionQueue) {
            if (t->task != c->task) continue;
            //first bl excute
            //if (t->issue_size == 0)
            //last bl excete
            if ( (t->issue_size+t->trans_size) == t->data_size) {
                uint64_t rd_lat = (now() + 1 - t->inject_time);
                uint64_t large_flag = 0;
                if(rd_lat>100)
                    large_flag = 1;
                cmd_in2dfi_lat += rd_lat;
                cmd_in2dfi_cnt ++;
                cmd_rdmet_cnt --;
                Cmd2Dfi_statistics(t->task, t->timeAdded, t->qos, t->mid, t->pf_type, t->rank);
                if (PRINT_LATENCY) {
                    DEBUG(setw(10)<<now()<<" -- LAT :: cnt="<<cmd_in2dfi_cnt<<", lat="<<rd_lat<<", lat_all="
                            <<cmd_in2dfi_lat<<", ave_lat="<<float(cmd_in2dfi_lat)/float(cmd_in2dfi_cnt)<<", task="
                            <<t->task<<", inject_time="<<t->inject_time<<", qos="<<t->qos<<",large_flag="<<large_flag);
                }
            }
            break;
        }
    }

    if (command_pend == 1) arb_enable = true;
    command.creat(c);
    exec_valid = true;
    for (auto &trans : transactionQueue) {
        if (trans->task != c->task) continue;
        if (trans->nextCmd >= WRITE_CMD && trans->nextCmd <= READ_P_CMD) {
            trans->issue_size += trans->trans_size;
            trans->addr_col += trans->trans_size;
        } else if (trans->nextCmd == ACTIVATE2_CMD) {
            trans->has_active = true;
        }
        if (trans->pre_act && trans->nextCmd == ACTIVATE2_CMD) {
            trans->issue_size = trans->data_size;
            if(now()< (trans->timeAdded+tRCD)){
                total_pre_act_success_cnt++;
                if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- PRE_ACT_SUCCESS :: task="<<trans->task<<endl);
                }
            }

        }
    }

    fresh_timing(command,hit);
}

/***************************************************************************************************
descriptor: check timeout and aging.
****************************************************************************************************/
void MemoryController::check_timeout_and_aging() {
    //timeout check
    if (!TIMEOUT_ENABLE && !MPAM_MAPPING_TIMEOUT) return;

    // calculate the highest pri for all commands in DMC Queue
    unsigned highest_pri = 0xfff;
    if (QOS_POLICY == 2) {
        for (auto &trans : transactionQueue) {
            if (now() < trans->arb_time) continue;
            if (trans->pri < highest_pri) highest_pri = trans->pri;
        }
    }

    // priority adapt
    if (PRI_ADPT_ENABLE || MPAM_MAPPING_TIMEOUT) {
        for (auto &trans : transactionQueue) {
            if (now() < trans->arb_time) continue;
            if (trans->addrconf) continue;
            if (trans->pri_adapt_th == 0 || trans->qos == 0) continue;
            if (now() - trans->timeAdded >= ((trans->improve_cnt + 1) * trans->pri_adapt_th)) {
                trans->improve_cnt ++;
                if (trans->pri > 0) {
                    trans->pri --;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- PRI_APAPT :: task="<<trans->task<<" pri="<<trans->pri<<endl);
                    }
                }
            }
        }
    }
    

    bool has_tout_cmd = false;
    bool has_rt_tout_cmd = false;
    bool has_hqos_tout_cmd = false;
//    bool has_real_tout_cmd = false;
    // generate original timeout flag
    for (auto &trans : transactionQueue) {
        if (now() < trans->arb_time) continue;
        if (trans->transactionType == DATA_WRITE && trans->data_ready_cnt <= trans->burst_length) continue;
        if (trans->addrconf) continue;
        if (!trans->timeout && ((now() - trans->timeAdded >= trans->timeout_th &&
                trans->timeout_th != 0) || trans->qos == 0)) {
            trans->timeout = true;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- TIMEOUT_DEBUG :: task="<<trans->task<<" qos="<<trans->qos<<" pri="<<trans->pri
                        <<" cmd_rt_type="<<trans->cmd_rt_type<<" cmd_hqos_type="<<trans->cmd_hqos_type<<" transactiontype="<<trans_type_opcode(trans->transactionType)<<endl);
            }
        } else if (trans->timeout && trans->pri != 0 && ((QOS_POLICY == 1) || (QOS_POLICY == 2 && trans->pri <= highest_pri))) {
//        } else if (trans->timeout && trans->pri != 0 && ((QOS_POLICY == 1) || (QOS_POLICY == 2 && trans->pri <= highest_pri)) && !has_real_tout_cmd) {
            if(TIMEOUT_MODE == 0){
                trans->pri = 0;
            }else if(TIMEOUT_MODE == 1){
                trans->pri = trans->pri;
            }else if(TIMEOUT_MODE == 2 && !trans->timeout_dec_once){
                trans->pri = trans->pri - 1;
                trans->timeout_dec_once = true;
            }
            
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- TIMEOUT_PRI :: task="<<trans->task<<" qos="<<trans->qos
                        <<" pri="<<trans->pri<<" transactiontype="<<trans_type_opcode(trans->transactionType)<<endl);
            }
        }
        if (trans->timeout && !has_tout_cmd) has_tout_cmd = true;
        if (trans->timeout && trans->cmd_rt_type && !has_rt_tout_cmd) has_rt_tout_cmd = true;
        if (trans->timeout && trans->cmd_hqos_type && !has_hqos_tout_cmd) has_hqos_tout_cmd = true;
//        if (trans->timeout && trans->pri == 0 && !has_real_tout_cmd) has_real_tout_cmd = true;
    }

    tout_high_pri = 0xFFFF;
    // generate real timeout flag
    for (auto &trans : transactionQueue) {
        if (!trans->timeout) continue;
        if (trans->pri < tout_high_pri) tout_high_pri = trans->pri;
    }

    // set bankStates has_timeout flag
    for (auto &trans : transactionQueue) {
        if (trans->timeout && trans->pri <= tout_high_pri) {
            bankStates[trans->bankIndex].has_timeout = true;
        }
    }
    
    // check qos of timeout
    bool real_tout_all_lqos = true;
    if (LQOS_BP_EN) {
        for (auto &trans : transactionQueue) {
            if (!trans->timeout) continue;
            if ((trans->qos < LQOS_BP_LEVEL) && (trans->pri == 0)) {
                real_tout_all_lqos = false;
                break;
            }
        }
    }

    // generate DMC Queue bp flag
    for (auto &trans : transactionQueue) {
        trans->bp_by_tout = false;
        if ((trans->qos < LQOS_BP_LEVEL) && real_tout_all_lqos && LQOS_BP_EN) continue;
        if (has_rt_tout_cmd && !trans->timeout) trans->bp_by_tout = true;
        else if (has_hqos_tout_cmd && !trans->timeout) trans->bp_by_tout = true;
        else if (QOS_POLICY == 1 && has_tout_cmd && !trans->timeout) trans->bp_by_tout = true;
        else if (QOS_POLICY == 2 && (trans->pri > tout_high_pri || (!trans->timeout && trans->pri == tout_high_pri))) {
            trans->bp_by_tout = true;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- Backpress by tout :: task="<<trans->task<<" qos="<<trans->qos
                        <<" pri="<<trans->pri<<" tout_high_pri="<<tout_high_pri<<endl);
            }
        }
    }
}
/***************************************************************************************************
descriptor: The purpose of this function is to Calculate burst length of DDR command
****************************************************************************************************/
void MemoryController::CalcBl(Transaction *t) {
    // Calc DDR command BL
    if (t->transactionType == DATA_READ) {
        t->trans_size = max_bl_data_size - t->addr_col % max_bl_data_size;
        if (t->data_size - t->issue_size < t->trans_size) t->trans_size = t->data_size - t->issue_size;
    } else {
        // address aligned check for lpddr
        if ((t->addr_col % min_bl_data_size != 0) && (IS_LP5 || IS_LP6 || IS_LP4)){
            ERROR(setw(10)<<now()<<" -- Address not 32B aligned, task="<<t->task);
            assert(0);
        }
        // Dif between data_size and issue_size check for lpddr
        if ((t->data_size - t->issue_size < min_bl_data_size) && (IS_LP5 || IS_LP6 || IS_LP4)){
            ERROR(setw(10)<<now()<<" -- Issue size wrong, task="<<t->task);
            assert(0);
        }

        if (t->mask_wcmd && !IS_LP6) {                                     //mask write condition, addr_col 32B aligned, up to mask_wcmd
            if (t->issue_size == 0) {                                      //first must be BL16 mask write
                t->trans_size = min_bl_data_size;
            } else {  
                for (auto it = bl_data_size.end(); it != bl_data_size.begin();) { // From max BL to min BL
                    it --;
                    if ((t->data_size - t->issue_size) >= it->second && t->addr_col % it->second == 0) {
                        t->trans_size = it->second;
                        break;
                    }
                }
            } 
        } else {                                                           //mask write original condition, addr_col not 32B aligned 
            if (t->data_size - t->issue_size < min_bl_data_size) {
                t->trans_size = t->data_size - t->issue_size;
            } else if (t->addr_col % min_bl_data_size == 0) {  
                for (auto it = bl_data_size.end(); it != bl_data_size.begin();) { // From max BL to min BL
                    it --;
                    if ((t->data_size - t->issue_size) >= it->second && t->addr_col % it->second == 0) {
                        t->trans_size = it->second;
                        break;
                    }
                }
            } else {
                t->trans_size = min_bl_data_size - t->addr_col % min_bl_data_size;
            }
        }
    }
    
    if (t->mask_wcmd && !IS_LP6) {  //todo: 0802  
        if (t->issue_size == 0) {   
            uint8_t bl_map_size = MAP_CONFIG["BL"].size();
            unsigned bl_min = MAP_CONFIG["BL"][bl_map_size - 1];
            t->bl = bl_min;
        } else {
            for (auto it = bl_data_size.begin(); it != bl_data_size.end(); it ++) { // From min BL to max BL
                if (t->addr_col % it->second + t->trans_size <= it->second) {
                    t->bl = it->first;
                    break;
                }
            }
        } 
    } else {
        for (auto it = bl_data_size.begin(); it != bl_data_size.end(); it ++) { // From min BL to max BL
            if (t->addr_col % it->second + t->trans_size <= it->second) {
                t->bl = it->first;
                break;
            }
        }
    }
}

/***************************************************************************************************
descriptor: The purpose of this function is to determine what kind of command to send next.
****************************************************************************************************/
void MemoryController::need_issue(Transaction *trans) {
    if (bankStates[trans->bankIndex].state->currentBankState == RowActive) {
        uint32_t &openRow = bankStates[trans->bankIndex].state->openRowAddress;
        if (trans->row == openRow) {
            if (rw_exec_cnt < EXEC_NUMBER || trans->issue_size != 0) {
                CalcBl(trans);
                bool row_hit = false;
                if ((trans->issue_size + trans->trans_size) < trans->data_size) {
                    row_hit = true;
                } else {
                    for (auto &t : transactionQueue) {
                        if (t->task == trans->task) continue;
                        if (t->bankIndex != trans->bankIndex) continue;
                        if (trans->row != t->row) continue;
                        row_hit = true;
                        break;
                    }
                }

                if (trans->transactionType == DATA_READ) {
                    if (RD_APRE_EN || trans->ap_cmd || page_timeout_rd[trans->bankIndex] == 0) {
                        if (row_hit) trans->nextCmd = READ_CMD;
                        else trans->nextCmd = READ_P_CMD;
                    } else if (ENHAN_RD_AP_EN && !row_hit) {
                        if (bank_cnt[trans->bankIndex] <= 1) trans->nextCmd = READ_CMD;
                        else trans->nextCmd = READ_P_CMD;
                    } else {
                        trans->nextCmd = READ_CMD;
                    }
                } else if (trans->transactionType == DATA_WRITE) {
                    bool is_mask = false;

                    // write address in DMC must be aligned, WRITE_MASK_CMD not allowed
                    if (IS_LP6){
//                        is_mask = (trans->trans_size < (trans->bl * JEDEC_DATA_BUS_BITS / 9)) || trans->mask_wcmd;
                        is_mask = false;
                    } else {
                        if (trans->mask_wcmd) {
                            is_mask = (trans->issue_size==0);          //first wcmd with mask flag must be BL16 mask write
                        } else {
                            is_mask = (trans->trans_size < (trans->bl * JEDEC_DATA_BUS_BITS / 8));
                        }
                    }
                    if (WR_APRE_EN || trans->ap_cmd || page_timeout_wr[trans->bankIndex] == 0) {
                        if (row_hit) trans->nextCmd = is_mask ? WRITE_MASK_CMD : WRITE_CMD;
                        else trans->nextCmd = is_mask ? WRITE_MASK_P_CMD : WRITE_P_CMD;
                    } else if (ENHAN_WR_AP_EN && !row_hit) {
                        if (bank_cnt[trans->bankIndex] <= 1) trans->nextCmd = WRITE_CMD;
                        else trans->nextCmd = WRITE_P_CMD;
                    } else {
                        trans->nextCmd = is_mask ? WRITE_MASK_CMD : WRITE_CMD;
                    }
                }
            } else {
                trans->nextCmd = INVALID;
            }
        } else {
            if (!bankStates[trans->bankIndex].hold_precharge) {
                trans->nextCmd = PRECHARGE_PB_CMD;
            }
            else trans->nextCmd = INVALID;
        }
    } else {
        if (IS_LP4 || IS_LP5 || IS_LP6 || IS_GD2) {
            if (trans->act_executing) trans->nextCmd = ACTIVATE2_CMD;
            else if (bankStates[trans->bankIndex].state->act_executing) trans->nextCmd = INVALID;
            else trans->nextCmd = ACTIVATE1_CMD;
        } else {
            trans->nextCmd = ACTIVATE2_CMD;
        }
    }

    if (trans->nextCmd == READ_P_CMD || trans->nextCmd == WRITE_P_CMD || trans->nextCmd == WRITE_MASK_P_CMD) {
        if (trans->issue_size + trans->trans_size < trans->data_size) {
            ERROR(setw(10)<<now()<<" -- Error Send AP cmd! task="<<trans->task<<", issue_size="<<trans->issue_size
                    <<", trans_size="<<trans->trans_size<<", data_size="<<trans->data_size);
            assert(0);
        }
    }
}

/***************************************************************************************************
descriptor: timing check
****************************************************************************************************/
void MemoryController::lc(Transaction *t) {
    unsigned sub_channel = (t->bankIndex % NUM_BANKS) / sc_bank_num;
    unsigned bank_start = sub_channel * sc_bank_num;
    if (t->pre_act && bankStates[t->bankIndex].state->currentBankState == RowActive) {
        t->issue_size = t->data_size;
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LC :: PRE-ACT. pre active meet row active"<<", task="<<t->task<<endl);
        }
        return;
    }

    if (RankState[t->rank].lp_state != IDLE) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LP_BP_CMD :: task="<<t->task<<" is BP by lowpower mode. rank="
                    <<t->rank<<", lp_state="<<RankState[t->rank].lp_state<<endl);
        }
        return;
    }

    if (!RCMD_HQOS_RANK_SWITCH_EN && DMC_V590 && NUM_RANKS > 1 && SIMPLE_GRP_RANK_EN) {
        if (t->transactionType == DATA_READ && t->rank == PreCmd.rank && t->issue_size == 0 &&
                bankStates[t->bankIndex].state->currentBankState != RowActive && !t->timeout &&
                !t->act_executing) {
            for (size_t next_rank = 0; next_rank < NUM_RANKS; next_rank ++) {
                if (next_rank == PreCmd.rank) continue;
                if ((r_rank_cnt[next_rank] >= READ_RANK_GRP_LEVEL0_H && r_rank_cnt[PreCmd.rank]
                        <= READ_RANK_GRP_LEVEL0_L) || (r_rank_cnt[next_rank] >= READ_RANK_GRP_LEVEL1_H
                        && r_rank_cnt[PreCmd.rank] <= READ_RANK_GRP_LEVEL1_L)) {
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LC :: Rank GRP BP read command. task="<<t->task<<" rank="<<t->rank<<endl);
                    }
                    return;
                }
            }
        }
    }

    if (NUM_SIDS > 1 && SIMPLE_GRP_SID_EN) {
        if (t->transactionType == DATA_READ && t->sid == PreCmd.sid && t->issue_size == 0 && !t->act_executing
                && bankStates[t->bankIndex].state->currentBankState != RowActive && !t->timeout) {
            for (size_t next_sid = 0; next_sid < NUM_SIDS; next_sid ++) {
                if (next_sid == PreCmd.sid) continue;
                if (r_sid_cnt[t->rank][next_sid] >= grp_sid_level[0] &&
                        r_sid_cnt[t->rank][PreCmd.sid] <= grp_sid_level[1]) {
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LC :: SID GRP BP read command. task="<<t->task<<" sid="<<t->sid<<endl);
                    }
                    return;
                }
            }
        }
    }

    if (DistRefState[t->bankIndex].force_dist_refresh && t->issue_size == 0) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LC :: GD2 force disturb refresh bp, bank"<<t->bankIndex<<", task="<<t->task<<endl);
        }
        return;
    }
    // todo: change for enahnced DBR?
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        if (force_pbr_refresh[rank][sub_channel] && SBR_REQ_MODE == 1 && t->issue_size == 0) {
            for (size_t sbr_bank = 0; sbr_bank < pbr_bg_num; sbr_bank ++){
                if ((forceRankBankIndex[rank][sub_channel] + sbr_bank * pbr_bank_num + rank * NUM_BANKS) == t->bankIndex) {
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LC :: force the same bank request in Rank "<<rank<<", SC "<<sub_channel<<". task="<<t->task<<endl);
                    }
                    return;
                }
            }
        }
    }

    if ((refreshPerBank[t->bankIndex].refreshWaiting || refreshPerBank[t->bankIndex].refreshing)
            && t->issue_size == 0 && !t->act_executing) {
        if (t->transactionType == DATA_READ) rd_met_pbr_cnt ++;
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LC :: this bank is refreshing. bank="<<t->bankIndex<<" sc="<<sub_channel<<" task="
                    <<t->task<<" postpnd="<<refreshALL[t->rank][sub_channel].refresh_cnt<<" refreshWaiting="
                    <<refreshPerBank[t->bankIndex].refreshWaiting<<" refreshing="
                    <<refreshPerBank[t->bankIndex].refreshing<<endl);
        }
        return;
    }

    if ((refreshALL[t->rank][sub_channel].refreshWaiting || refreshALL[t->rank][sub_channel].refreshing)
            && t->issue_size == 0 && !t->act_executing) {
        if (t->transactionType == DATA_READ) rd_met_abr_cnt ++;
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LC :: this rank is refreshing. rank="<<t->rank<<", sc="<<sub_channel<<" task="
                    <<t->task<<" refreshWaiting="<<refreshALL[t->rank][sub_channel].refreshWaiting
                    <<" refreshing="<<refreshALL[t->rank][sub_channel].refreshing<<endl);
        }
        return;
    }

    //if the t has address conflict ,it cannot be send a task to SCH
    //not rhit break command can send precharge to DDR
    if (t->row != bankStates[t->bankIndex].state->openRowAddress && !t->timeout && t->issue_size == 0
            && !bankStates[t->bankIndex].has_rhit_break && bankStates[t->bankIndex].has_cmd_rowhit
            && !t->act_executing) {
       if (DEBUG_BUS) {
           PRINTN(setw(10)<<now()<<" -- LC :: row miss; bank="<<t->bankIndex<<" task="<<t->task<<endl);
       }
       return;
    }

    if (t->addrconf) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LC :: addr conflict. task="<<t->task<<endl);
        }
        return;
    }


    need_issue(t);

    if (dresp_cnt >= DRESP_BP_TH && t->nextCmd >= WRITE_CMD && t->nextCmd <= READ_P_CMD
            && t->issue_size == 0) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LC :: dresp backpress ing---. dresponse counter="<<dresp_cnt<<endl);
        }
        return;
    }
    if (RHIT_BREAK_EN && !t->timeout && bankStates[t->bankIndex].has_rhit_break && t->nextCmd != PRECHARGE_PB_CMD
            && t->issue_size == 0 && !t->act_executing) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LC :: rowhit break. bank="<<t->bankIndex<<" row="<<t->row<<" task="<<t->task<<endl);
        }
        return;
    }

    if (TABLE_DEPTH != 0 && table_use_cnt >= TABLE_DEPTH && (t->nextCmd == ACTIVATE1_CMD || t->nextCmd == ACTIVATE2_CMD)) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LC :: Table limited. task="<<t->task<<", bank="<<t->bankIndex<<endl);
        }
        return;
    }

    if (t->bp_by_tout && t->issue_size == 0 && !t->act_executing && 
            (bankStates[t->bankIndex].has_timeout || t->nextCmd != PRECHARGE_PB_CMD)) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LC :: Block by timeout cmd. task="<<t->task<<", bank="
                    <<t->bankIndex<<", pri="<<t->pri<<", tout_high_pri="<<tout_high_pri<<endl);
        }
        return;
    }

    if (t->transactionType != DATA_READ && t->nextCmd != ACTIVATE1_CMD &&
            t->nextCmd != ACTIVATE2_CMD && t->nextCmd != PRECHARGE_PB_CMD) {
        if (t->data_ready_cnt <= t->burst_length) {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- LC :: write data not ready. cnt="<<t->data_ready_cnt
                        <<" issue_size="<<t->issue_size<<" task="<<t->task<<endl);
            }
            return;
        }
    }

    if (rk_grp_state != NO_RGRP && !t->timeout && t->issue_size == 0 && t->nextCmd != PRECHARGE_PB_CMD
            && !t->act_executing) {
        uint8_t t_state = (t->rank << 1) | uint8_t(t->transactionType);
        if (real_rk_grp_state == t_state && rk_grp_state != real_rk_grp_state) {
            if (t->nextCmd == activate_cmd || rwgrp_ch_cmd_cnt >= RW_GRPCHG_W2R_TH) {
                if (DEBUG_BUS) { // currnet
                    PRINTN(setw(10)<<now()<<" -- LC :: current cmd bp. C="<<+real_rk_grp_state<<", N="
                            <<+rk_grp_state<<". task="<<t->task<<", rank="<<t->rank<<", type="
                            <<t->transactionType<<endl);
                }
                return;
            }
        } else if (t_state != rk_grp_state && t_state != real_rk_grp_state) {
            if (DEBUG_BUS) { // others
                PRINTN(setw(10)<<now()<<" -- LC :: others cmd bp. C="<<+real_rk_grp_state<<", N="
                        <<+rk_grp_state<<". task="<<t->task<<", rank="<<t->rank<<", bank="
                        <<t->bankIndex<<", type="<<t->transactionType<<endl);
            }
            return;
        }
    }
 

    if (rw_group_state[0] != NO_GROUP && !t->timeout && t->issue_size == 0 && t->nextCmd != PRECHARGE_PB_CMD
            && !t->act_executing) {
        if (rw_group_state[0] == READ_GROUP && t->transactionType != DATA_READ && (!LQOS_BP_EN || (lqos_rd_bp && LQOS_BP_EN))) {
            if (t->nextCmd == activate_cmd || rwgrp_ch_cmd_cnt >= RW_GRPCHG_W2R_TH || !in_write_group) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: Read group backpress a write command. task="
                            <<t->task<<", nextCmd="<<t->nextCmd<<endl);
                }
                return;
            }
        } else if (rw_group_state[0] == READ_GROUP && t->transactionType == DATA_READ && RCMD_HQOS_RANK_SWITCH_EN) {
            if ((t->nextCmd == READ_CMD || t->nextCmd == READ_P_CMD) && !rank_cmd_high_qos[t->rank] &&
                    ((unsigned)accumulate(rank_cmd_high_qos.begin(), rank_cmd_high_qos.end(), 0) >= 1)) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: Read group backpress a rank command by highqos."<<
                            " task="<<t->task<<", nextCmd="<<t->nextCmd<<endl);
                }
                return;
            }
        } else if (rw_group_state[0] == WRITE_GROUP && t->transactionType == DATA_READ && (!LQOS_BP_EN || (lqos_wr_bp && LQOS_BP_EN))) {
            if (t->nextCmd == activate_cmd || rwgrp_ch_cmd_cnt >= RW_GRPCHG_R2W_TH || in_write_group) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: Write group backpress a read command. task="
                            <<t->task<<", nextCmd="<<t->nextCmd<<endl);
                }
                return;
            }
        }
    }

    if (LQOS_BP_EN && t->issue_size == 0 && t->nextCmd != PRECHARGE_PB_CMD
            && !t->act_executing && t->qos>=LQOS_BP_LEVEL) {
        if (lqos_bp && !(bankStates[t->bankIndex].state->openRowAddress == t->row)) {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- LC :: Higher qos backpress a low qos command. task="
                        <<t->task<<", nextCmd="<<t->nextCmd<<", qos="<<t->qos<<endl);
            }
            return;
        }
    }

#if 0
    if (!IS_DDR5 && t->nextCmd == PRECHARGE_PB_CMD && pbr_hold_pre[t->rank]) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LC :: PBR hold PRE command. task="<<t->task<<endl);
        }
        return;
    }
#else
    // samerank precharge blocked before pbr precharge sent
    if (!IS_DDR5 && t->nextCmd == PRECHARGE_PB_CMD) {   //todo: revise for e-mode
        for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
            unsigned bank_tmp = t->rank * NUM_BANKS + bank;
//            refreshPerBank[bank_tmp].refreshWaitingPre =false;
            if (refreshPerBank[bank_tmp].refreshWaiting && !refreshPerBank[bank_tmp].refreshWaitingPre) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: PBR hold PRE command. task="<<t->task<<endl);
                }
                return;
            }
        }
    }

//    // samerank act1 blocked before pbr sent
//    if (!IS_DDR5 && t->nextCmd == ACTIVATE1_CMD && t->issue_size == 0) {   //todo: revise for e-mode
//        for (size_t bank = 0; bank < NUM_BANKS; bank ++) {
//            unsigned bank_tmp = t->rank * NUM_BANKS + bank;
//            if (refreshPerBank[bank_tmp].refreshWaiting && refreshPerBank[bank_tmp].refreshWaitingPre) {
//                if (DEBUG_BUS) {
//                    PRINTN(setw(10)<<now()<<" -- LC :: PBR hold ACT1 command. task="<<t->task<<endl);
//                }
//                return;
//            }
//        }
//    }
#endif
//    // guarantee pbr can be sent, block active1 of selected bank pair
//    if ((refreshPerBank[t->bankIndex].refreshWaiting || refreshPerBank[t->bankIndex].refreshing)
//            && t->nextCmd == ACTIVATE1_CMD) {
//        if (DEBUG_BUS) {
//            PRINTN(setw(10)<<now()<<" -- LC :: this bank is refreshing, act1 must be blocked. bank="<<t->bankIndex<<" sc="<<sub_channel<<" task="
//                    <<t->task<<" postpnd="<<refreshALL[t->rank][sub_channel].refresh_cnt<<" refreshWaiting="
//                    <<refreshPerBank[t->bankIndex].refreshWaiting<<" refreshing="
//                    <<refreshPerBank[t->bankIndex].refreshing<<endl);
//        }
//        return;
//    }

    if (t->nextCmd == INVALID) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- LC :: Invalid command. task="<<t->task<<endl);
        }
        return;
    }

    bool timing_met = false;
    switch (t->nextCmd) {
        case READ_CMD : {
            if (has_cmd_bp()) break;
            if ((now() + 1) >= bankStates[t->bankIndex].state->nextRead) {
                timing_met = true;
                if (t->issue_size == 0) cmd_rdmet_cnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: READ timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                }
            }
            break;
        }
        case READ_P_CMD : {
            if (has_cmd_bp()) break;
            if ((now() + 1) >= bankStates[t->bankIndex].state->nextReadAp) {
                timing_met = true;
                if (t->issue_size == 0) cmd_rdmet_cnt ++;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: READ_P timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                }
            }
            break;
        }
        case WRITE_CMD : {
            if (has_cmd_bp()) break;
            if (IS_DDR5) {
                if (DDR_MODE == "_x4" && t->bl == BL16) {
                    // next command is RMW
                    if ((now() + 1) >= bankStates[t->bankIndex].state->nextWriteRmw) {
                        timing_met = true;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- LC :: WRITE RMW timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                        }
                    }
                } else if ((DDR_MODE == "_x4" && t->bl == BL32) || ((DDR_MODE == "_x8" || DDR_MODE == "_x16") && t->bl == BL16)) {
                    // next command is JW
                    if ((now() + 1) >= bankStates[t->bankIndex].state->nextWrite) {
                        timing_met = true;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- LC :: WRITE JW timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                        }
                    }
                } else {
                    ERROR(setw(10)<<now()<<" -- Error DDR Mode: "<<DDR_MODE<<", BL="<<t->bl);
                    assert(0);
                }
            } else {
                if ((now() + 1) >= bankStates[t->bankIndex].state->nextWrite) {
                    timing_met = true;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LC :: WRITE timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                    }
                }
            }
            break;
        }
        case WRITE_P_CMD : {
            if (has_cmd_bp()) break;
            if (IS_DDR5) {
                if (DDR_MODE == "_x4" && t->bl == BL16) {
                    // next command is RMW
                    if ((now() + 1) >= bankStates[t->bankIndex].state->nextWriteApRmw) {
                        timing_met = true;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- LC :: WRITE_P RMW timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                        }
                    }
                } else if ((DDR_MODE == "_x4" && t->bl == BL32) || ((DDR_MODE == "_x8" || DDR_MODE == "_x16") && t->bl == BL16)) {
                    // next command is JW
                    if ((now() + 1) >= bankStates[t->bankIndex].state->nextWriteAp) {
                        timing_met = true;
                        if (DEBUG_BUS) {
                            PRINTN(setw(10)<<now()<<" -- LC :: WRITE_P JW timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                        }
                    }
                } else {
                    ERROR(setw(10)<<now()<<" -- Error DDR Mode: "<<DDR_MODE<<", BL="<<t->bl);
                    assert(0);
                }
            } else {
                if ((now() + 1) >= bankStates[t->bankIndex].state->nextWriteAp) {
                    timing_met = true;
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- LC :: WRITE_P timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                    }
                }
            }
            break;
        }
        case WRITE_MASK_CMD : {
            if (has_cmd_bp()) break;
            if ((now() + 1) >= bankStates[t->bankIndex].state->nextWriteMask) {
                timing_met = true;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: WRITE_MASK timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                }
            }
            break;
        }
        case WRITE_MASK_P_CMD : {
            if (has_cmd_bp()) break;
            if ((now() + 1) >= bankStates[t->bankIndex].state->nextWriteMaskAp) {
                timing_met = true;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: WRITE_MASK_P timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                }
            }
            break;
        }
        case PRECHARGE_PB_CMD : {
            if ((now() + 1) >= bankStates[t->bankIndex].state->nextPrecharge && tFPWCountdown[t->rank].size() < 4) {
                timing_met = true;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: PRECHARGE_PB timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                }
            }
            break;
        }
        case PRECHARGE_SB_CMD : {
            uint8_t met_cnt = 0;
            for (size_t i = 0; i < pbr_bg_num; i ++) {
                uint32_t bank = t->rank * NUM_BANKS + i * pbr_bank_num + t->bankIndex % pbr_bank_num;
                if ((now() + 1) >= bankStates[bank].state->nextPrecharge) met_cnt ++;
            }
            if (met_cnt == pbr_bg_num && tFPWCountdown[t->rank].size() < 4) {
                timing_met = true;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: PRECHARGE_SB timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                }
            }
            break;
        }
        case PRECHARGE_AB_CMD : {   //todo: revise for e-mode
            uint8_t met_cnt = 0;
            for (size_t i = 0; i < NUM_BANKS/sc_num; i ++) {
                uint32_t bank = i + t->rank * NUM_BANKS + bank_start;
                if ((now() + 1) >= bankStates[bank].state->nextPrecharge) met_cnt ++;
            }
            if ((met_cnt == NUM_BANKS/sc_num) && tFPWCountdown[t->rank].size() < 4) {
                timing_met = true;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: PRECHARGE_AB timing met, bank="<<t->bankIndex<<", sc="<<sub_channel<<", task="<<t->task<<endl);
                }
            }
            break;
        }
        case ACTIVATE1_CMD : {
            if ((now() + 1) >= bankStates[t->bankIndex].state->nextActivate1) {
                timing_met = true;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: ACTIVE1 timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                }
            }
            break;
        }
        case ACTIVATE2_CMD : {
            if ((now() + 1) >= bankStates[t->bankIndex].state->nextActivate2 && ((tFAWCountdown[t->rank].size() < 4 && sub_channel==0) 
                    ||(tFAWCountdown_sc1[t->rank].size() < 4 && sub_channel==1))) {
                timing_met = true;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- LC :: ACTIVE2 timing met, bank="<<t->bankIndex<<", task="<<t->task<<endl);
                }
            }
            break;
        }
        default:break;
    }

    if (timing_met) {
        Cmd *c = new Cmd;
        if (((t->issue_size + t->trans_size) >= t->data_size) &&
                ((t->nextCmd != PRECHARGE_PB_CMD && t->nextCmd != ACTIVATE2_CMD))) {
            *c = Cmd(*t, false);
        } else {
            *c = Cmd(*t, true);
        }
        CmdQueue.push_back(c);
    }
}

bool MemoryController::WillAcceptTransaction() {
    return GetDmcQsize() < TRANS_QUEUE_DEPTH;
}

/***************************************************************************************************
descriptor: check conflict,The purpose of this approach is to keep order
****************************************************************************************************/
void MemoryController::check_conflict(Transaction *trans) {
    for (auto &t : transactionQueue) {
        if (((trans->address & ~ALIGNED_SIZE) == (t->address & ~ALIGNED_SIZE))
                && (!t->pre_act) && ((t->transactionType != DATA_READ && trans->transactionType == DATA_READ)
                || (t->transactionType == DATA_READ && trans->transactionType != DATA_READ)
                || (t->transactionType != DATA_READ && trans->transactionType != DATA_READ))) {
            trans->addrconf = true;
            if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- ADDR_CONF ::  task="<<trans->task<<" dst task="<<t->task
                            <<"type="<<trans->transactionType<<" dst type="<<t->transactionType
                            <<" qos="<<trans->qos<<" pri="
                            <<trans->pri<<" Aligned_size="<<ALIGNED_SIZE<<endl);
                }
            addrconf_cnt++;
            if (trans->pri < t->pri && PRIORITY_PASS_ENABLE) {
                t->pri = trans->pri;
                t->qos = trans->qos;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- ADD_CONF_PUSH :: push add conflict pri, src task="
                            <<trans->task<<" dst task="<<t->task<<" qos="<<trans->qos<<" pri="
                            <<trans->pri<<endl);
                }
            }
            trans->addr_block_source_id = t->task;
        }
    }
}

void MemoryController::ehs_page_adapt_policy() {
    if (!ENH_PAGE_ADPT_EN) return;
    if (now() % ENH_PAGE_ADPT_WIN == 0 && now() != 0) {
        for (size_t i = 0; i < NUM_RANKS * NUM_BANKS; i ++) {
            if (bank_cnt_ehs[i] >= MAP_CONFIG["ENH_PAGE_ADPT_LVL"][2]) {
                page_timeout_rd[i] = MAP_CONFIG["ENH_PAGE_ADPT_TIME"][3];
                page_timeout_wr[i] = MAP_CONFIG["ENH_PAGE_ADPT_TIME"][3];
                ehs_page_adapt_cnt[i/NUM_BANKS][3] ++;
            } else if (bank_cnt_ehs[i] >= MAP_CONFIG["ENH_PAGE_ADPT_LVL"][1]) {
                page_timeout_rd[i] = MAP_CONFIG["ENH_PAGE_ADPT_TIME"][2];
                page_timeout_wr[i] = MAP_CONFIG["ENH_PAGE_ADPT_TIME"][2];
                ehs_page_adapt_cnt[i/NUM_BANKS][2] ++;
            } else if (bank_cnt_ehs[i] >= MAP_CONFIG["ENH_PAGE_ADPT_LVL"][0]) {
                page_timeout_rd[i] = MAP_CONFIG["ENH_PAGE_ADPT_TIME"][1];
                page_timeout_wr[i] = MAP_CONFIG["ENH_PAGE_ADPT_TIME"][1];
                ehs_page_adapt_cnt[i/NUM_BANKS][1] ++;
            } else {
                page_timeout_rd[i] = MAP_CONFIG["ENH_PAGE_ADPT_TIME"][0];
                page_timeout_wr[i] = MAP_CONFIG["ENH_PAGE_ADPT_TIME"][0];
                ehs_page_adapt_cnt[i/NUM_BANKS][0] ++;
            }
            bank_cnt_ehs[i] = 0;
        }
    }
}

void MemoryController::page_adapt_policy(Transaction *trans) {
    if (!PAGE_ADAPT_EN) return;

    if (now() % 800 == 0 && now() != 0 && 0) {
        unsigned rowhit_ratio = 0;
        unsigned page_tout_rd = 0;
        unsigned page_tout_wr = 0;
        if (page_rw_cnt != 0) rowhit_ratio = 1000 * page_act_cnt / page_rw_cnt;

        if (rowhit_ratio >= 750) {
            page_tout_rd = 5;
            page_tout_wr = 5;
        } else if (rowhit_ratio >= 500) {
            page_tout_rd = 50;
            page_tout_wr = 50;
        } else if (rowhit_ratio >= 250) {
            page_tout_rd = 100;
            page_tout_wr = 100;
        } else {
            page_tout_rd = 200;
            page_tout_wr = 200;
        }

        for (size_t bank = 0; bank < NUM_RANKS * NUM_BANKS; bank ++) {
            page_timeout_rd[bank] = page_tout_rd;
            page_timeout_wr[bank] = page_tout_wr;
        }
        page_act_cnt = 0;
        page_rw_cnt = 0;
    }

    unsigned bank = trans->bankIndex;
    unsigned row = trans->row;
    for (uint32_t i = 0; i < sizeof(page_timeout_window)/sizeof(page_timeout_window[0]); i ++) {
        if (row == bankStates[bank].last_activerow) {
            if (bank_cas_delay[bank] < page_timeout_window[i]) page_row_hit[bank][i] ++;
            else page_row_miss[bank][i] ++;
        } else {
            uint32_t gap;
            gap = (bank_cas_delay[bank] < tRPpb) ? 0 : bank_cas_delay[bank] - tRPpb;
            if (gap < page_timeout_window[i]) page_row_conflict[bank][i] ++;
            else page_row_miss[bank][i] ++;
        }
    }

    page_cmd_cnt[bank] ++;
    if ((trans->transactionType == DATA_READ && page_cmd_cnt[bank] >= OPENPAGE_TIME_RD) ||
            (trans->transactionType == DATA_WRITE && page_cmd_cnt[bank] >= OPENPAGE_TIME_WR)) {
        uint32_t index = 0;
        int32_t best_timeout = page_row_hit[bank][0] - page_row_conflict[bank][0];

        for (uint32_t i = 1; i < sizeof(page_timeout_window)/sizeof(page_timeout_window[0]); i ++) {
            if ((page_row_hit[bank][i] - page_row_conflict[bank][i]) > best_timeout) {
                best_timeout = page_row_hit[bank][i] - page_row_conflict[bank][i];
                index = i;
            }
        }

        if (trans->transactionType == DATA_READ) page_timeout_rd[bank] = page_timeout_window[index];
        else page_timeout_wr[bank] = page_timeout_window[index];

        for (uint32_t i = 0; i < sizeof(page_timeout_window)/sizeof(page_timeout_window[0]); i ++) {
            page_row_hit[bank][i] = 0;
            page_row_miss[bank][i] = 0;
            page_row_conflict[bank][i] = 0;
        }
        page_cmd_cnt[bank] = 0;
    }
}

void MemoryController::page_adpt_policy(Transaction *trans) {
    if (!PAGE_ADPT_EN) return;

    if (PAGE_WIN_MODE == 1) page_adpt_win_cnt ++;

    auto &state = bankStates[trans->bankIndex].state;
    // Has no same bank command, bank open, row miss, precharge timing met
    // Overdue page close, add opc
    if (bank_cnt[trans->bankIndex] == 0 && state->currentBankState == RowActive &&
            state->openRowAddress != trans->row && now() >= state->nextPrecharge) {
        opc_cnt ++;
        if (PAGE_WIN_MODE == 0) page_adpt_win_cnt ++;
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- OPC_ADD, opc_cnt="<<opc_cnt<<", ppc_cnt="
                    <<ppc_cnt<<", task="<<trans->task<<", bank="<<trans->bankIndex
                    <<", adpt_openpage_time="<<adpt_openpage_time<<endl);
        }
    }
    // Has no same bank command, bank not open, row hit with last row
    // Premature page close, add ppc
    if (bank_cnt[trans->bankIndex] == 0 && state->currentBankState != RowActive && state->lastRow == trans->row
            && (!DMC_V596 || state->lastCmdSource != 2)) {
        ppc_cnt ++;
        if (PAGE_WIN_MODE == 0) page_adpt_win_cnt ++;
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- PPC_ADD, opc_cnt="<<opc_cnt<<", ppc_cnt="
                    <<ppc_cnt<<", task="<<trans->task<<", bank="<<trans->bankIndex
                    <<", adpt_openpage_time="<<adpt_openpage_time<<endl);
        }
    }

    if (page_adpt_win_cnt == PAGE_WIN_SIZE) {
        if (ppc_cnt >= PAGE_PPC_TH && opc_cnt < PAGE_OPC_TH) {
            if (adpt_openpage_time >= PAGE_TIME_MAX) adpt_openpage_time = PAGE_TIME_MAX;
            else adpt_openpage_time += PAGE_ADPT_STEP;
        } else if (ppc_cnt < PAGE_PPC_TH && opc_cnt >= PAGE_OPC_TH) {
            if (adpt_openpage_time < PAGE_ADPT_STEP) adpt_openpage_time = 0;
            else adpt_openpage_time -= PAGE_ADPT_STEP;
        }

        for (size_t i = 0; i < NUM_RANKS * NUM_BANKS; i ++) {
            page_timeout_rd[i] = adpt_openpage_time;
            page_timeout_wr[i] = adpt_openpage_time;  
        }

        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- WIN_EXCEED, page_adpt_win_cnt="<<page_adpt_win_cnt
                    <<", opc_cnt="<<opc_cnt<<", ppc_cnt="<<ppc_cnt<<", adpt_openpage_time="
                    <<adpt_openpage_time<<endl);
        }
        opc_cnt = 0;
        ppc_cnt = 0;
        page_adpt_win_cnt = 0;
    }
}

void MemoryController::update_deque_fifo() {
    for (size_t i = 0; i < NUM_RANKS; i ++) {
        deqCmdWakeupLp[i].push_back(rank_cnt[i]);
        deqCmdWakeupLp[i].pop_front();
    }
    
    //for (size_t i = 0; i < NUM_RANKS; i ++) {
    //    for (size_t j = 0; j < deqCmdWakeupLp[i].size(); j++){
    //        DEBUG(now()<<" deqCmdWakeuplp="<<deqCmdWakeupLp[i][j]<<" rank="<<i);
    //    }
    //}
}

bool MemoryController::push_req(Transaction * trans) {
//    DEBUG(now()<<" push_req0, sc_num="<<sc_num); 
    if (PERFECT_DMC_EN) {
        if (trans->transactionType == DATA_READ) {
            for (size_t i = 0; i <= trans->burst_length; i ++) {
                unsigned cnt = i % 2 + 1;
                gen_rdata(trans->task, cnt, PERFECT_DMC_DELAY, trans->mask_wcmd);
            }
            push_pending_TransactionQue(trans);
        }
        return true;
    }
    bool pos = false;
    trans->inject_time = now();

    if (DROP_WRITE_CMD && trans->transactionType != DATA_READ) {
        pos = true;
    } else if (IECC_ENABLE && (!IECC_PARTIAL_BYPASS || trans->address < IECC_BYPASS_ADDRESS) && !trans->pre_act) {
        pos = iecc->addTransaction(trans);
    } else if (WRITE_BUFFER_ENABLE && !trans->pre_act) {
        pos = wb->addTransaction(trans);
    } else if (RMW_ENABLE && !trans->pre_act) {
        pos = rmw->addTransaction(trans);
    } else {
        pos = addTransaction(trans);
    }

    if (pos) {
        DmcTotalBytes += trans->data_size;
        if (trans->transactionType == DATA_READ) DmcTotalReadBytes += trans->data_size;
        else DmcTotalWriteBytes += trans->data_size;

        if (!trans->pre_act) {
            if (IECC_ENABLE && (!IECC_PARTIAL_BYPASS || trans->address < IECC_BYPASS_ADDRESS)) total_iecc_cnt ++;
            else total_noiecc_cnt ++;
        }
    }

    if (!(IECC_ENABLE && (!IECC_PARTIAL_BYPASS || trans->address < IECC_BYPASS_ADDRESS))
            && pos && trans->transactionType == DATA_READ && !trans->pre_act) {
        if (pending_TransactionQue.size() >= 1000) {
            ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] pending_TransactionQue size is too big");
            assert(0);
        }
        if (pending_TransactionQue.find(trans->task) != pending_TransactionQue.end()) {
            ERROR(setw(10)<<now()<<" -- [DMC"<<channel<<"] should be error, task="<<trans->task);
            assert(0);
        }
        push_pending_TransactionQue(trans);
    }
    return pos;
}

void MemoryController::pushQosForSameMpamTrans(Transaction *trans) {
    for (auto &trans_t : transactionQueue) {
        if (trans_t->mpam_id == trans->mpam_id) {
            if (trans->pri < trans_t->pri && trans_t->transactionType == DATA_READ) trans_t->pri = trans->pri;
        }
    }
}

void MemoryController::pushQosForSameMidTrans(Transaction *trans) {
    for (auto &trans_t : transactionQueue) {
        if (trans_t->mid == trans->mid) {
            if (trans->pri < trans_t->pri && trans_t->transactionType == DATA_READ) trans_t->pri = trans->pri;
        }
    }
}

void MemoryController::noc_read_inform(bool fast_wakeup_rank0, bool fast_wakeup_rank1, bool bus_rempty) {
    bool wakeup[2];
    wakeup[0] = fast_wakeup_rank0;
    wakeup[1] = fast_wakeup_rank1;
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        fast_wakeup[rank] = wakeup[rank];
        if (FASTWAKEUP_EN && !PREDICT_FASTWAKEUP) fast_wakeup_cnt[rank] += int(wakeup[rank]);
    }
    if (DEBUG_BUS) {
        if (fast_wakeup_rank0 || fast_wakeup_rank1) {
            PRINTN(setw(10)<<now()<<" -- Fast Wakeup, rank0="<<fast_wakeup_rank0<<", rank1="<<fast_wakeup_rank1<<endl);
        }
    }
}

//allows outside source to make request of memory system
bool MemoryController::addTransaction(Transaction *trans) {
    if (!full()) {
        auto &state = bankStates[trans->bankIndex];
        trans_state_init(trans);

        // no e-mode : only 0 ; e-mode: upto subchannel
        unsigned sub_channel = (trans->bankIndex % NUM_BANKS) / sc_bank_num;
//        unsigned bank_start = sub_channel * NUM_BANKS /sc_num;
//        unsigned bank_pair_start = sub_channel * pbr_bank_num;

        if (trans->pre_act) {
            //if (RankState[trans->rank].lp_state != IDLE) return true;//add function:preact can wake up pd
            pre_act_cnt ++;
            total_pre_act_cnt ++;
            if (trans->transactionType != DATA_READ) {
                trans->data_ready_cnt = trans->burst_length + 1;
            }
            if (RankState[trans->rank].lp_state == IDLE && (state.state->currentBankState == Idle) && GetDmcQsize() == 0){ 
                bool act_met = false;
                act_met = true;
                for (auto &b : bankStates) {
                    if (now() < b.state->nextActivate2) {
                        act_met = false;
                        break;
                    }
                }
                if(act_met){
                    trans->arb_time = now() + tCMD2SCH_BYPACT;
                }
            }
            transactionQueue.push_back(trans);
            rank_pre_act_cnt[trans->rank] ++;
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- PRE_ACT :: type"<<trans->transactionType<<" addr="<<hex
                        <<trans->address<<dec<<" task="<<trans->task<<" rank="<<trans->rank<<" bank="<<trans->bankIndex<<" row="
                        <<trans->row<<endl);
            }
            return true;
        }

        pre_req_time = now();
        totalTransactions++;
        check_conflict(trans);
        page_adapt_policy(trans);
        page_adpt_policy(trans);
        rank_cnt[trans->rank] ++;
        sc_cnt[trans->rank][sub_channel] ++;       //todo: revise for e-mode
        bank_cnt[trans->bankIndex] ++;
        bg_cnt[trans->rank][trans->group] ++;
        sid_cnt[trans->rank][trans->sid] ++;
        acc_rank_cnt[trans->rank] ++;
        acc_bank_cnt[trans->bankIndex] ++;

        if (MPAM_PUSH_EN) pushQosForSameMpamTrans(trans);
        if (MID_PUSH_EN) pushQosForSameMidTrans(trans);

        if (BYP_ACT_EN && trans->transactionType == DATA_READ && GetDmcQsize() == 0 &&
                RankState[trans->rank].lp_state == IDLE && state.state->currentBankState == Idle) {
            bool act_met = false;
            if (DMC_V596) {
                if (now() >= state.state->nextActivate2) act_met = true;
            } else {
                act_met = true;
                for (auto &b : bankStates) {
                    if (now() < b.state->nextActivate2) {
                        act_met = false;
                        break;
                    }
                }
            }

            if (act_met) {
                has_bypact_exec = true;
                trans->arb_time = now() + tCMD2SCH_BYPACT;
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- BYP_ACT :: addr=0x"<<hex<<trans->address<<dec
                            <<" task="<<trans->task<<" bank="<<trans->bankIndex<<endl);
                }
            }
        }
        
        if (DMC_V590 && !SBR_IDLE_EN && !rank_send_pbr[trans->rank][sub_channel] &&
                refreshALL[trans->rank][sub_channel].refresh_cnt < ABR_PSTPND_LEVEL) {
            if (refreshALL[trans->rank][sub_channel].refreshWaiting && !refreshALL[trans->rank][sub_channel].refreshing)
                refreshALL[trans->rank][sub_channel].refreshWaiting = false;
        }

        TotalDmcBytes += trans->data_size;
        if (trans->transactionType == DATA_READ) {
            r_rank_cnt[trans->rank] ++;
            r_rank_bst[trans->rank] += ceil(float(trans->data_size) / max_bl_data_size);
            que_read_cnt ++;
            totalReads++;
            r_bank_cnt[trans->bankIndex] ++;
            r_bg_cnt[trans->rank][trans->group] ++;
            r_sid_cnt[trans->rank][trans->sid] ++;
            r_qos_cnt[trans->qos] ++;
            racc_rank_cnt[trans->rank] ++;
            racc_bank_cnt[trans->bankIndex] ++;
            TotalDmcRdBytes += trans->data_size;
            if (trans->data_size == 32) TotalDmcRd32B ++;
            else if (trans->data_size == 64) TotalDmcRd64B ++;
            else if (trans->data_size == 128) TotalDmcRd128B ++;
            else if (trans->data_size == 256) TotalDmcRd256B ++;
            if ((trans->address % trans->data_size) == 0) rd_inc_cnt ++;
            else rd_wrap_cnt ++;
            if (trans->qos <= SWITCH_HQOS_LEVEL) {
                que_read_highqos_cnt[trans->rank] ++;
                highqos_r_bank_cnt[trans->bankIndex] ++;
            }
            if (RCMD_HQOS_EN && trans->qos <= RCMD_HQOS_LEVEL) trans->cmd_hqos_type = true;

            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- ADD_DMC :: [R]B["<<trans->burst_length<<"]QOS["<<trans->qos
                        <<"]MID["<<trans->mid<<"] addr=0x"<<hex<<trans->address<<dec<<" task="<<trans->task
                        <<" rank="<<trans->rank<<" group="<<trans->group<<" bank="<<trans->bank<<" bankIndex="
                        <<trans->bankIndex<<" row="<<trans->row<<" col="<<trans->col<<" addr_col="<<trans->addr_col
                        <<" data_size="<<trans->data_size<<" Q="<<GetDmcQsize()<<" QR="<<que_read_cnt<<" QW="
                        <<que_write_cnt<<" timeAdded="<<trans->timeAdded<<" timeout_th="<<trans->timeout_th
                        <<" mask_wcmd="<<trans->mask_wcmd<<endl);
            }
        } else {
            w_rank_cnt[trans->rank] ++;
            w_rank_bst[trans->rank] += ceil(float(trans->data_size) / max_bl_data_size);
            w_bank_cnt[trans->bankIndex] ++;
            w_bg_cnt[trans->rank][trans->group] ++;
            w_sid_cnt[trans->rank][trans->sid] ++;
            w_qos_cnt[trans->qos] ++;
            que_write_cnt ++;
            totalWrites++;
//            DEBUG(" total write cnt="<<totalWrites);
            wacc_rank_cnt[trans->rank] ++;
            wacc_bank_cnt[trans->bankIndex] ++;
            TotalDmcWrBytes += trans->data_size;
            if (trans->data_size == 32) TotalDmcWr32B ++;
            else if (trans->data_size == 64) TotalDmcWr64B ++;
            else if (trans->data_size == 128) TotalDmcWr128B ++;
            else if (trans->data_size == 256) TotalDmcWr256B ++;
            if ((trans->address % trans->data_size) == 0) wr_inc_cnt ++;
            else wr_wrap_cnt ++;

            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- ADD_DMC :: [W]B["<<trans->burst_length<<"]QOS["<<trans->qos
                        <<"]MID["<<trans->mid<<"] addr=0x"<<hex<<trans->address<<dec<<" task="<<trans->task
                        <<" rank="<<trans->rank<<" group="<<trans->group<<" bank="<<trans->bank<<" bankIndex="
                        <<trans->bankIndex<<" row="<<trans->row<<" col="<<trans->col<<" addr_col="<<trans->addr_col
                        <<" data_size="<<trans->data_size<<" Q="<<GetDmcQsize()<<" QR="<<que_read_cnt<<" QW="
                        <<que_write_cnt<<" timeAdded="<<trans->timeAdded<<" timeout_th="<<trans->timeout_th
                        <<" mask_wcmd="<<trans->mask_wcmd<<endl);
            }
        }

        if (BG_ROTATE_EN) {
            for (auto &t : transactionQueue) {
                if (trans->group == t->group) {
                    trans->bg_rotate_pri = t->bg_rotate_pri;
                    break;
                }
            }
        }

        for (auto &t : transactionQueue) {
            if (trans->task == t->task) {
                ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] task:"<<trans->task<<", task duplication!");
                assert(0);
            }
        }

        
        //label no of arb group for each tran in transactionQue
        if (!SLOT_FIFO) {
            for (size_t grp_index = 0; grp_index < 4; grp_index ++) {
                if(arb_group_cnt[grp_index] < 16) {
                    trans->arb_group = grp_index;
                    arb_group_cnt[grp_index]++;
                    break;
                }
            }

            if (trans->arb_group >= 4) {
                ERROR(setw(10)<<now()<<" Wrong Arb Group, task="<<trans->task<<", arb_group="<<trans->arb_group);
                assert(0);
            }
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- Label New Cmd :: addr=0x"<<hex<<trans->address<<dec
                        <<" task="<<trans->task<<" arb_group="<<trans->arb_group<<" index="<<arb_group_cnt[trans->arb_group]
                        <<" bank="<<trans->bankIndex<<endl);
            }
        }

        transactionQueue.push_back(trans);

//        if (SLOT_FIFO) {
//            transactionQueue.push_back(trans);
//        } else {
//            for (size_t slt = 0; slt < TRANS_QUEUE_DEPTH; slt ++) {
//                if (slt_valid[slt]) continue;
//                slt_valid[slt] = true;
//                transactionQueue.insert(transactionQueue.begin() + slt, trans);
//                break;
//            }
//        }
        
        //lp6: full write and merge read under fast command mode
        if (((trans->transactionType==DATA_WRITE && !trans->mask_wcmd) || (trans->transactionType==DATA_READ && trans->mask_wcmd)) && RMW_CMD_MODE==0 && IS_LP6) {
            if ((!IECC_ENABLE || !tasks_info[trans->task].wr_ecc) && !trans->ecc_flag) {
                gen_cresp(trans->task);
            }
        }
        
        //lp5: full write and mask write
        if ((trans->transactionType==DATA_WRITE) && IS_LP5) {
            if ((!IECC_ENABLE || !tasks_info[trans->task].wr_ecc) && !trans->ecc_flag) {
                gen_cresp(trans->task);
            }
            
        }

//        if (!CmdResp.empty()) {
//            if (pre_cresp_time != now()) {
//                if (cmd_response(CmdResp[0],0)) {
//                    if (DEBUG_BUS) {
//                        PRINTN(setw(10)<<now()<<" -- Cresp Received :: task="<<CmdResp[0]<<endl);
//                    }
//                    pre_cresp_time = now();
//                    CmdResp.erase(CmdResp.begin());
//                } else {
//                    if (DEBUG_BUS) {
//                        PRINTN(setw(10)<<now()<<" -- Cresp Back Pressure :: task="<<CmdResp[0]<<endl);
//                    }
//                }
//            }
//        }
            

        if (((IECC_ENABLE && (!IECC_PARTIAL_BYPASS || trans->address < IECC_BYPASS_ADDRESS))
                || (RMW_ENABLE && trans->mask_wcmd))   
                && trans->transactionType == DATA_READ && !trans->pre_act) {
            if (pending_TransactionQue.size() >= 1000) {
                ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] pending_TransactionQue size is too big");
                assert(0);
            }
            if (pending_TransactionQue.find(trans->task) != pending_TransactionQue.end()) {
                ERROR(setw(10)<<now()<<" -- [DMC"<<channel<<"] should be error, task="<<trans->task);
                assert(0);
            }
            push_pending_TransactionQue(trans);
        }
        if (FASTWAKEUP_EN && !PREDICT_FASTWAKEUP && trans->transactionType == DATA_READ) {
            if (fast_wakeup_cnt[trans->rank] == 0 && now() >= 100) {
                ERROR(setw(10)<<now()<<" -- DMC["<<channel<<"] rank:"<<trans->rank<<", Error fast wakeup count!");
                assert(0);
            }
            fast_wakeup_cnt[trans->rank] -= 1;
        }

        if (ENH_PAGE_ADPT_EN && state.state->currentBankState == RowActive && trans->row == state.state->openRowAddress)
            bank_cnt_ehs[trans->bankIndex] ++;

        if (state.state->currentBankState == RowActive && trans->row == state.state->openRowAddress) state.row_hit_cnt ++;
        else state.row_miss_cnt ++;

        auto &st = RankState[trans->rank].lp_state;
        if (st >= PDE && st <= PDX) cmd_met_pd_cnt ++;
        if (st >= ASREFE && st <= SRPDX) cmd_met_asref_cnt ++;
        rank_cnt_asref[trans->rank] ++;
        rank_cnt_sbridle[trans->rank][sub_channel] ++;       //todo: revise for e-mode
        return true;
    } else {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- CMD_BP :: addr=0x"<<hex<<trans->address<<dec<<" task="<<trans->task
                    <<" bank="<<trans->bankIndex<<" QR="<<que_read_cnt<<" QW="<<que_write_cnt<<endl);
        }
        return false;
    }
}

void MemoryController::update_cresp() {
    // return cresp to ha 
    if (!CmdResp.empty()) {
        if (pre_cresp_time != now()) {
            if (cmd_response(CmdResp[0],0)) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- Cresp Received :: task="<<CmdResp[0]<<endl);
                }
                pre_cresp_time = now();
                CmdResp.erase(CmdResp.begin());
            } else {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- Cresp Back Pressure :: task="<<CmdResp[0]<<endl);
                }
            }
        }
    }
}

void MemoryController::update_even_cycle() {
    even_cycle = ((now()%2==1 && IS_LP6 && DMC_RATE<=6400) || !IS_LP6 || (IS_LP6 && DMC_RATE>6400));
    odd_cycle  = ((now()%2==0 && IS_LP6 && DMC_RATE<=6400) || !IS_LP6 || (IS_LP6 && DMC_RATE>6400));
}

void MemoryController::trans_state_init(Transaction *trans) {
    if (FORCE_BAINTLV_EN) {
        trans->rank = 0;
        trans->group = trans->task % NUM_GROUPS;
        trans->bank = trans->task / NUM_GROUPS % (NUM_BANKS / NUM_SIDS / NUM_GROUPS);
        trans->sid = trans->task / (NUM_BANKS / NUM_SIDS) % NUM_SIDS;
        trans->bankIndex = trans->bank + trans->group * (NUM_BANKS / NUM_SIDS / NUM_GROUPS) +
                trans->rank * NUM_BANKS + trans->sid * (NUM_BANKS / NUM_SIDS);
    }

    trans->timeAdded = now();
    trans->pri = trans->qos;
    trans->reqAddToDmcTime = now() * tDFI;
    trans->arb_time = now() + tCMD2SCH;
    trans->enter_que_time = now() + tCMD_CONF;
    trans->data_ready_cnt = 0;

    string mpam_timeout, mpam_adapt;
    unsigned timeout = 0, pri_adapt = 0;
    if (MPAM_MAPPING_TIMEOUT && trans->mpam_id != 0) { // mpam_id mapping timeout
        if (trans->transactionType == DATA_READ) {
            mpam_timeout = "MPAM_TIMEOUT_RD";
            mpam_adapt = "MPAM_ADAPT_RD";
        } else {
            mpam_timeout = "MPAM_TIMEOUT_WR";
            mpam_adapt = "MPAM_ADAPT_WR";
        }
        if (TIMEOUT_ENABLE) timeout = MAP_CONFIG[mpam_timeout][trans->mpam_id];
        if (PRI_ADPT_ENABLE) pri_adapt = MAP_CONFIG[mpam_adapt][trans->mpam_id];
    } else if (trans->qos > 0) { // qos mapping timeout
        if (trans->transactionType == DATA_READ) {
            mpam_timeout = "TIMEOUT_PRI_RD";
            mpam_adapt = "ADAPT_PRI_RD";
        } else {
            mpam_timeout = "TIMEOUT_PRI_WR";
            mpam_adapt = "ADAPT_PRI_WR";
        }
        if (TIMEOUT_ENABLE) timeout = MAP_CONFIG[mpam_timeout][trans->qos - 1];
        if (PRI_ADPT_ENABLE) pri_adapt = MAP_CONFIG[mpam_adapt][trans->qos - 1];
    }
    if (timeout == 0) trans->timeout_th = 0;
    else trans->timeout_th = timeout + trans->qos * pri_adapt;
    trans->pri_adapt_th = pri_adapt;
    if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- trans-timeout_th="<<trans->timeout_th<<" timeout="<<timeout<<" pri_adapt="<<pri_adapt<<endl);
            }
}

bool MemoryController::check_samebank(unsigned int bank) {
    for (auto &trans : transactionQueue) {
        if (bank == trans->bankIndex) return true;
    }
    return false;
}

void MemoryController::dfs_backpress(bool backpress) {
    dfs_backpress_en = backpress;
    if (DEBUG_BUS) {
        PRINTN(setw(10)<<now()<<" -- DFS :: set dfs_backpress_en="<<dfs_backpress_en<<endl);
    }
}

void MemoryController::gen_cresp(uint64_t task) {
    CmdResp.push_back(task);
}

void MemoryController::gen_wresp(uint64_t task) {
    WriteResp.push_back(task);
    dresp_cnt++;
}

void MemoryController::gen_rresp(uint64_t task) {
    ReadResp.push_back(task);
    dresp_cnt++;
//    DEBUG(" rcmd response generated, task="<<task);
}

void MemoryController::gen_rdata(uint64_t task, unsigned cnt, unsigned delay, bool mask_wcmd) {
    data_packet pkt;
    //the goal of mins 1 is that data path can start to counter early
    pkt.task = task;
    pkt.cnt = cnt;
    pkt.delay = delay;
    pkt.mask_wcmd = mask_wcmd;
    read_data_buffer.push_back(pkt);
}

void MemoryController::push_pending_TransactionQue(Transaction *trans) {
    TRANS_MSG msg;
    if (LAT_INC_BP) msg.time = uint64_t(trans->reqEnterDmcBufTime);
    else msg.time = now();
    msg.reqEnterDmcBufTime = trans->reqEnterDmcBufTime;
    msg.reqAddToDmcTime = trans->reqAddToDmcTime;
    msg.burst_cnt = 0;
    msg.channel = trans->channel;
    msg.rank = trans->rank;
    msg.data_size = trans->data_size;
    msg.burst_length = trans->burst_length;
    msg.qos = trans->qos;
    msg.mid = trans->mid;
    msg.pf_type = trans->pf_type;
    pending_TransactionQue[trans->task] = msg;
}

bool MemoryController::has_cmd_bp() {
    for (auto &bpc : bp_cycle) {
        if ((now() + 1) == bpc) return true;
    }
    return false;
}

float MemoryController::calc_power() {
    float power_sum = 0;
    unsigned power_cnt = 0;
    power_sum += POWER_RDINC_K * rd_inc_cnt;
    power_sum += POWER_RDWRAP_K * rd_wrap_cnt;
    power_sum += POWER_WRINC_K * wr_inc_cnt;
    power_sum += POWER_WRWRAP_K * wr_wrap_cnt;
    power_sum += POWER_RDATA_K * rdata_cnt;
    power_sum += POWER_WDATA_K * wdata_cnt;
    power_sum += POWER_ACT_K * active_cnt;
    power_sum += POWER_PREP_K * (precharge_pb_cnt + precharge_pb_dst_cnt);
    power_sum += POWER_PRES_K * precharge_sb_cnt;
    power_sum += POWER_PREA_K * precharge_ab_cnt;
    power_sum += MAP_CONFIG["POWER_RD_K"][0] * RdCntBl[BL8];
    power_sum += MAP_CONFIG["POWER_RD_K"][1] * RdCntBl[BL16];
    power_sum += MAP_CONFIG["POWER_RD_K"][2] * RdCntBl[BL24];
    power_sum += MAP_CONFIG["POWER_RD_K"][3] * RdCntBl[BL32];
    power_sum += MAP_CONFIG["POWER_RD_K"][4] * RdCntBl[BL48];
    power_sum += MAP_CONFIG["POWER_RD_K"][5] * RdCntBl[BL64];
    power_sum += MAP_CONFIG["POWER_WR_K"][0] * WrCntBl[BL8];
    power_sum += MAP_CONFIG["POWER_WR_K"][1] * WrCntBl[BL16];
    power_sum += MAP_CONFIG["POWER_WR_K"][2] * WrCntBl[BL24];
    power_sum += MAP_CONFIG["POWER_WR_K"][3] * WrCntBl[BL32];
    power_sum += MAP_CONFIG["POWER_WR_K"][4] * WrCntBl[BL48];
    power_sum += MAP_CONFIG["POWER_WR_K"][5] * WrCntBl[BL64];
    power_sum += POWER_PBR_K * refresh_pb_cnt;
    power_sum += POWER_ABR_K * refresh_ab_cnt;
    power_sum += POWER_R2W_K * r2w_switch_cnt;
    power_sum += POWER_W2R_K * w2r_switch_cnt;
    power_sum += POWER_RNKSW_K * rank_switch_cnt;
    power_sum += POWER_IDLE_K * phy_notlp_cnt;
    power_sum += POWER_PDCC_K * phy_lp_cnt;
    for (size_t rank = 0; rank < NUM_RANKS; rank ++) {
        power_cnt += POWER_PDE_K * PdEnterCnt[rank];
        power_cnt += POWER_ASREFE_K * AsrefEnterCnt[rank];
        power_cnt += POWER_SRPDE_K * SrpdEnterCnt[rank];
        power_cnt += POWER_PDX_K * PdExitCnt[rank];
        power_cnt += POWER_ASREFX_K * AsrefExitCnt[rank];
        power_cnt += POWER_SRPDX_K * SrpdExitCnt[rank];
    }
//    for (size_t que = 1; que <= TRANS_QUEUE_DEPTH; que ++) {
    for (size_t que = 0; que < TRANS_QUEUE_DEPTH; que ++) {
        power_cnt += MAP_CONFIG["POWER_QUEUE_K"][que] * parentMemorySystem->que_cnt[que];
    }
    rd_inc_cnt = 0;
    rd_wrap_cnt = 0;
    wr_inc_cnt = 0;
    wr_wrap_cnt = 0;
    rdata_cnt = 0;
    wdata_cnt = 0;
    return power_sum;
}

MemoryController::~MemoryController() {
//    DEBUG("CAS_FS Stat: Total DMC Command: "<<totalTransactions<<", Rank Switch Count: "
//            <<rank_switch_cnt<<", CAS_FS Count: "<<casfs_cnt<<", CAS_FS Next Rank Time: "
//            <<casfs_time<<", AVG: "<<(double(casfs_time)/casfs_cnt));
}

/**********************************************************
            GOD BLESS YOU
 **********************************************************/