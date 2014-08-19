/*
 * Gas_Gauge driver for CW2015/2013
 * Copyright (C) 2012, CellWise
 *
 * Authors: ChenGang <ben.chen@cellwise-semi.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.And this driver depends on 
 * I2C and uses IIC bus for communication with the host.
 *
 */

#ifndef CW2015_BATTERY_H
#define CW2015_BATTERY_H

#define SIZE_BATINFO    64 

////drive name and I2C slave address
#define  CW2015_BATTERY_DRIVE_NAME "cw201x"
#define  CW2015_BATTERY_SLAVE_ADD  0x62


struct cw_bat_platform_data {

        int is_dc_charge;
        int dc_det_pin;
        int dc_det_level;

        int is_usb_charge;
        int chg_mode_sel_pin;
        int chg_mode_sel_level;

        int bat_low_pin;
        int bat_low_level;
        int chg_ok_pin;
        int chg_ok_level;
        u8* cw_bat_config_info;
};

#endif
