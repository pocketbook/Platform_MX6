/*
 * TRX image file header format.
 *
 * Copyright (C) 2011, Broadcom Corporation
 * All Rights Reserved.
 * 
 * THIS SOFTWARE IS OFFERED "AS IS", AND BROADCOM GRANTS NO WARRANTIES OF ANY
 * KIND, EXPRESS OR IMPLIED, BY STATUTE, COMMUNICATION OR OTHERWISE. BROADCOM
 * SPECIFICALLY DISCLAIMS ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A SPECIFIC PURPOSE OR NONINFRINGEMENT CONCERNING THIS SOFTWARE.
 *
 * $Id: trxhdr.h 286295 2011-09-27 06:39:43Z $
 */

#ifndef	_TRX_HDR_H_
#define	_TRX_HDR_H_

#include <typedefs.h>

#define TRX_MAGIC	0x30524448	/* "HDR0" */
#define TRX_VERSION	1		/* Version 1 */
#define TRX_MAX_LEN	0x3B0000	/* Max length */
#define TRX_NO_HEADER	1		/* Do not write TRX header */
#define TRX_GZ_FILES	0x2     /* Contains up to TRX_MAX_OFFSET individual gzip files */
#define TRX_OVERLAYS	0x4     /* Contains an overlay header after the trx header */
#define TRX_MAX_OFFSET	3		/* Max number of individual files */
#define TRX_UNCOMP_IMAGE	0x20	/* Trx contains uncompressed rtecdc.bin image */
#define TRX_ROMSIM_IMAGE	0x10	/* Trx contains ROM simulation image */

struct trx_header {
	uint32 magic;		/* "HDR0" */
	uint32 len;		/* Length of file including header */
	uint32 crc32;		/* 32-bit CRC from flag_version to end of file */
	uint32 flag_version;	/* 0:15 flags, 16:31 version */
	uint32 offsets[TRX_MAX_OFFSET];	/* Offsets of partitions from start of header */
};

/* Compatibility */
typedef struct trx_header TRXHDR, *PTRXHDR;

#endif /* _TRX_HDR_H_ */
