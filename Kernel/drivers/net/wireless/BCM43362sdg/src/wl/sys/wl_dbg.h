/*
 * Minimal debug/trace/assert driver definitions for
 * Broadcom 802.11 Networking Adapter.
 *
 * Copyright (C) 2011, Broadcom Corporation
 * All Rights Reserved.
 * 
 * THIS SOFTWARE IS OFFERED "AS IS", AND BROADCOM GRANTS NO WARRANTIES OF ANY
 * KIND, EXPRESS OR IMPLIED, BY STATUTE, COMMUNICATION OR OTHERWISE. BROADCOM
 * SPECIFICALLY DISCLAIMS ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A SPECIFIC PURPOSE OR NONINFRINGEMENT CONCERNING THIS SOFTWARE.
 *
 * $Id: wl_dbg.h,v 1.115.6.3 2010-12-15 21:42:23 $
 */



#ifndef _wl_dbg_h_
#define _wl_dbg_h_


extern uint32 wl_msg_level;
extern uint32 wl_msg_level2;

#define WL_PRINT(args)      printf args



#define WL_NONE(args)

#define WL_ERROR(args)
#define WL_TRACE(args)
#ifndef LINUX_POSTMOGRIFY_REMOVAL
#ifdef WLMSG_PRHDRS
#define WL_PRHDRS(i, p, f, t, r, l) wlc_print_hdrs(i, p, f, t, r, l)
#else
#define WL_PRHDRS(i, p, f, t, r, l)
#endif
#ifdef WLMSG_PRPKT
#define WL_PRPKT(m, b, n)   prhex(m, b, n)
#else
#define WL_PRPKT(m, b, n)
#endif
#ifdef WLMSG_INFORM
#define WL_INFORM(args)     WL_PRINT(args)
#else
#define WL_INFORM(args)
#endif
#define WL_TMP(args)
#ifdef WLMSG_OID
#define WL_OID(args)        WL_PRINT(args)
#else
#define WL_OID(args)
#endif
#define WL_RATE(args)
#ifdef WLMSG_ASSOC
#define WL_ASSOC(args)      WL_PRINT(args)
#else
#define WL_ASSOC(args)
#endif
#define WL_PRUSR(m, b, n)
#ifdef WLMSG_PS
#define WL_PS(args)     WL_PRINT(args)
#else
#define WL_PS(args)
#endif

#define WL_PORT(args)
#define WL_DUAL(args)
#ifdef WLMSG_WSEC
#define WL_WSEC(args)       WL_PRINT(args)
#define WL_WSEC_DUMP(args)  WL_PRINT(args)
#else
#define WL_WSEC(args)
#define WL_WSEC_DUMP(args)
#endif

#define WL_REGULATORY(args)

#ifdef WLMSG_MPC
#define WL_MPC(args)        WL_PRINT(args)
#else
#define WL_MPC(args)
#endif
#define WL_APSTA(args)
#define WL_APSTA_UPDN(args)
#define WL_APSTA_BCN(args)
#define WL_APSTA_TX(args)
#define WL_APSTA_RX(args)
#define WL_APSTA_TSF(args)
#define WL_APSTA_BSSID(args)
#define WL_BA(args)
#define WL_MBSS(args)
#define WL_PROTO(args)

#define WL_CAC(args)
#define WL_AMSDU(args)
#define WL_AMPDU(args)
#define WL_FFPLD(args)
#define WL_MCHAN(args)



#ifdef WLMSG_DFS
#define WL_DFS(args)        do {if (wl_msg_level & WL_DFS_VAL) WL_PRINT(args);} while (0)
#else 
#define WL_DFS(args)
#endif 
#define WL_WOWL(args)
#define WL_DPT(args)
#define WL_ASSOC_OR_DPT(args)
#ifdef WLMSG_SCAN
#define WL_SCAN(args)       WL_PRINT(args)
#else
#define WL_SCAN(args)
#endif
#define WL_COEX(args)
#define WL_RTDC(w, s, i, j)
#define WL_RTDC2(w, s, i, j)

#ifdef WLMSG_CHANINT
#define WL_CHANINT(args) WL_PRINT(args)
#else
#define WL_CHANINT(args)
#endif

#ifdef WLMSG_BTA
#define WL_BTA(args)        WL_PRINT(args)
#else
#define WL_BTA(args)
#endif
#define WL_P2P(args)
#define WL_TDLS(args)
#define WL_ERROR_ON()       0
#define WL_TRACE_ON()       0
#ifdef WLMSG_PRHDRS
#define WL_PRHDRS_ON()      1
#else
#define WL_PRHDRS_ON()      0
#endif
#ifdef WLMSG_PRPKT
#define WL_PRPKT_ON()       1
#else
#define WL_PRPKT_ON()       0
#endif
#ifdef WLMSG_INFORM
#define WL_INFORM_ON()      1
#else
#define WL_INFORM_ON()      0
#endif
#ifdef WLMSG_OID
#define WL_OID_ON()     1
#else
#define WL_OID_ON()     0
#endif
#define WL_TMP_ON()     0
#define WL_RATE_ON()        0
#ifdef WLMSG_ASSOC
#define WL_ASSOC_ON()       1
#else
#define WL_ASSOC_ON()       0
#endif
#define WL_PORT_ON()        0
#ifdef WLMSG_WSEC
#define WL_WSEC_ON()        1
#define WL_WSEC_DUMP_ON()   1
#else
#define WL_WSEC_ON()        0
#define WL_WSEC_DUMP_ON()   0
#endif
#ifdef WLMSG_MPC
#define WL_MPC_ON()     1
#else
#define WL_MPC_ON()     0
#endif
#define WL_REGULATORY_ON()  0

#define WL_APSTA_ON()       0
#define WL_BA_ON()      0
#define WL_MBSS_ON()        0
#ifdef WLMSG_DFS
#define WL_DFS_ON()     1
#else 
#define WL_DFS_ON()     0
#endif 
#define WL_DPT_ON()     0
#ifdef WLMSG_SCAN
#define WL_SCAN_ON()            1
#else
#define WL_SCAN_ON()            0
#endif
#ifdef WLMSG_BTA
#define WL_BTA_ON()     1
#else
#define WL_BTA_ON()     0
#endif
#define WL_P2P_ON()     0
#define WL_MCHAN_ON()       0
#define WL_TDLS_ON()		0

#define WL_AMPDU_UPDN(args)
#define WL_AMPDU_RX(args)
#define WL_AMPDU_ERR(args)
#define WL_AMPDU_TX(args)
#define WL_AMPDU_CTL(args)
#define WL_AMPDU_HW(args)
#define WL_AMPDU_HWTXS(args)
#define WL_AMPDU_HWDBG(args)
#define WL_AMPDU_ERR_ON()       0
#define WL_AMPDU_HW_ON()        0
#define WL_AMPDU_HWTXS_ON()     0

#endif 


extern uint32 wl_msg_level;
extern uint32 wl_msg_level2;
#endif 
