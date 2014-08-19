/*
 * 802.1Q VLAN protocol definitions
 *
 * Copyright (C) 2011, Broadcom Corporation
 * All Rights Reserved.
 * 
 * THIS SOFTWARE IS OFFERED "AS IS", AND BROADCOM GRANTS NO WARRANTIES OF ANY
 * KIND, EXPRESS OR IMPLIED, BY STATUTE, COMMUNICATION OR OTHERWISE. BROADCOM
 * SPECIFICALLY DISCLAIMS ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A SPECIFIC PURPOSE OR NONINFRINGEMENT CONCERNING THIS SOFTWARE.
 *
 * $Id: vlan.h 277737 2011-08-16 17:54:59Z $
 */


#ifndef _vlan_h_
#define _vlan_h_

#ifndef _TYPEDEFS_H_
#include <typedefs.h>
#endif


#include <packed_section_start.h>

#define VLAN_VID_MASK		0xfff	
#define	VLAN_CFI_SHIFT		12	
#define VLAN_PRI_SHIFT		13	

#define VLAN_PRI_MASK		7	

#define	VLAN_TAG_LEN		4
#define	VLAN_TAG_OFFSET		(2 * ETHER_ADDR_LEN)	

#define VLAN_TPID		0x8100	

struct ethervlan_header {
	uint8	ether_dhost[ETHER_ADDR_LEN];
	uint8	ether_shost[ETHER_ADDR_LEN];
	uint16	vlan_type;		
	uint16	vlan_tag;		
	uint16	ether_type;
};

#define	ETHERVLAN_HDR_LEN	(ETHER_HDR_LEN + VLAN_TAG_LEN)



#include <packed_section_end.h>

#define ETHERVLAN_MOVE_HDR(d, s) \
do { \
	struct ethervlan_header t; \
	t = *(struct ethervlan_header *)(s); \
	*(struct ethervlan_header *)(d) = t; \
} while (0)

#endif 
