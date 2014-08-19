/*
 * Copyright (C) 2009 Samsung Electronics
 * Copyright (C) 2012 Nvidia Cooperation
 * Minkyu Kang <mk7.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX17048_BATTERY_H_
#define __MAX17048_BATTERY_H_
//#include <linux/smb349-charger.h>

////drive name and I2C slave address
#define  MAX17048_BATTERY_DRIVE_NAME "max17048"
#define  MAX17048_BATTERY_SLAVE_ADD  0x36

#define MAX17048_VCELL          0x02
#define MAX17048_SOC            0x04
#define MAX17048_MODE        0x06
#define MAX17048_VER            0x08
#define MAX17048_HIBRT          0x0A
#define MAX17048_CONFIG         0x0C
#define MAX17048_OCV            0x0E
#define MAX17048_VLRT           0x14
#define MAX17048_VRESET         0x18
#define MAX17048_STATUS         0x1A
#define MAX17048_UNLOCK         0x3E
#define MAX17048_TABLE          0x40
#define MAX17048_RCOMPSEG1      0x80
#define MAX17048_RCOMPSEG2      0x90
#define MAX17048_CMD            0xFE/*0xFF*/


#define MAX17048_UNLOCK_VALUE   0x4a57
#define MAX17048_MODE_VALUE   0x4000
#define MAX17048_RESET_VALUE    0x5400
#define MAX17048_DELAY          1000
#define MAX17048_BATTERY_FULL   100
#define MAX17048_BATTERY_LOW    15
#define MAX17048_VERSION_NO     0x11

struct max17048_battery_model {
	uint8_t rcomp;
	uint8_t soccheck_A;
	uint8_t soccheck_B;
	uint8_t bits;
	uint8_t alert_threshold;
	uint8_t one_percent_alerts;
	uint8_t alert_on_reset;
	uint16_t rcomp_seg;
	uint16_t hibernate;
	uint16_t vreset;
	uint16_t valert;
	uint16_t ocvtest;
};






struct max17048_platform_data {
	int (*battery_online)(void);
	int (*charging_status)(void);
	int (*charger_online)(void);
};


#endif
