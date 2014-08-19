/*
 * Dongle WL Header definitions
 *
 * Copyright (C) 2011, Broadcom Corporation
 * All Rights Reserved.
 * 
 * THIS SOFTWARE IS OFFERED "AS IS", AND BROADCOM GRANTS NO WARRANTIES OF ANY
 * KIND, EXPRESS OR IMPLIED, BY STATUTE, COMMUNICATION OR OTHERWISE. BROADCOM
 * SPECIFICALLY DISCLAIMS ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A SPECIFIC PURPOSE OR NONINFRINGEMENT CONCERNING THIS SOFTWARE.
 *
 * $Id: dngl_wlhdr.h,v 1.1 2009-01-08 01:21:12 $
 */

#ifndef _dngl_wlhdr_h_
#define _dngl_wlhdr_h_

typedef struct wl_header {
    uint8   type;           /* Header type */
    uint8   version;        /* Header version */
	int8	rssi;			/* RSSI */
	uint8	pad;			/* Unused */
} wl_header_t;

#define WL_HEADER_LEN   sizeof(wl_header_t)
#define WL_HEADER_TYPE  0
#define WL_HEADER_VER   1
#endif /* _dngl_wlhdr_h_ */
