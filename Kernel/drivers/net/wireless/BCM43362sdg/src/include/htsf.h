/*
 * Time stamps for latency measurements 
 *
 * Copyright (C) 2011, Broadcom Corporation
 * All Rights Reserved.
 * 
 * THIS SOFTWARE IS OFFERED "AS IS", AND BROADCOM GRANTS NO WARRANTIES OF ANY
 * KIND, EXPRESS OR IMPLIED, BY STATUTE, COMMUNICATION OR OTHERWISE. BROADCOM
 * SPECIFICALLY DISCLAIMS ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A SPECIFIC PURPOSE OR NONINFRINGEMENT CONCERNING THIS SOFTWARE.
 *
 * $Id: htsf.h 277737 2011-08-16 17:54:59Z $
 */
#ifndef _HTSF_H_
#define _HTSF_H_

#define HTSFMAGIC       	0xCDCDABAB  /* in network order for tcpdump  */
#define HTSFENDMAGIC    	0xEFEFABAB  /* to distinguish from RT2 magic */
#define HTSF_HOSTOFFSET		102
#define HTSF_DNGLOFFSET		HTSF_HOSTOFFSET	- 4
#define HTSF_DNGLOFFSET2	HTSF_HOSTOFFSET	+ 106
#define HTSF_MIN_PKTLEN 	200
#define ETHER_TYPE_BRCM_PKTDLYSTATS     0x886d

typedef enum htsfts_type {
	T10,
	T20,
	T30,
	T40,
	T50,
	T60,
	T70,
	T80,
	T90,
	TA0,
	TE0
} htsf_timestamp_t;

typedef struct {
	uint32 magic;
	uint32 prio;
	uint32 seqnum;
	uint32 misc;
	uint32 c10;
	uint32 t10;
	uint32 c20;
	uint32 t20;
	uint32 t30;
	uint32 t40;
	uint32 t50;
	uint32 t60;
	uint32 t70;
	uint32 t80;
	uint32 t90;
	uint32 cA0;
	uint32 tA0;
	uint32 cE0;
	uint32 tE0;
	uint32 endmagic;
} htsfts_t;

#endif /* _HTSF_H_ */
