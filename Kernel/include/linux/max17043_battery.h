/*
 *  Copyright (C) 2009 Samsung Electronics
 *  Minkyu Kang <mk7.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX17043_BATTERY_H_
#define __MAX17043_BATTERY_H_

////drive name and I2C slave address
#define  MAX17043_BATTERY_DRIVE_NAME "max17043"
#define  MAX17043_BATTERY_SLAVE_ADD  0x36

#define MAX17043_VCELL		0x02
#define MAX17043_SOC		0x04
#define MAX17043_MODE		0x06
#define MAX17043_VERSION	0x08
#define MAX17043_CONFIG	0x0C
#define MAX17043_COMMAND	0xFE

#define MAX17043_DELAY		1000
#define MAX17043_BATTERY_FULL 100/*95*/
#define MAX17043_BATTERY_LOW 15

/* 5% alert setting */
#define MAX17043_5_ALERT	0x1B
#define COMP_FULL_MARGIN	300	/* 3% */


struct max17043_platform_data {
        u8 alert_flag;
        u8 charging_rcomp;
        u8 discharging_rcomp;
        int standard_temp;
        int comp_full;
        int comp_empty;
        int (*battery_online)(void);
        int (*charger_online)(void);
        int (*charger_enable)(void);
        int (*low_batt_cb)(void);
};

#endif
