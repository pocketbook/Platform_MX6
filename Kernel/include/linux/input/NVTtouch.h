/*
 * include/linux/NVTtouch_touch.h
 *
 * 
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#ifndef 	_LINUX_NVT_TOUCH_H
#define	_LINUX_NVT_TOUCH_H

#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>

//*************************TouchScreen Work Part*****************************

#define NVTTOUCH_I2C_NAME "NVT-ts"

//define I2C Address
#define FW_Address 0x20/*0x01*/
#define HW_Address 0x7F

//define default resolution of the touchscreen
#define TOUCH_MAX_HEIGHT 	1024			
#define TOUCH_MAX_WIDTH		758

//set i2c Bus number
#define NVT_BUS_NUM 0


//set trigger mode
#define INT_TRIGGER			0

#define POLL_TIME		10	/*actual query spacing interval:POLL_TIME+6 */

#define NVTTOUCH_MULTI_TOUCH
#ifdef NVTTOUCH_MULTI_TOUCH
	#define MAX_FINGER_NUM	10  
#else
	#define MAX_FINGER_NUM	1	
#endif

#define ModeB	0
#define NVT_TOUCH_CTRL_DRIVER 0
#define UPDATE_FIRMWARE 0
#define ReportRate	1
#define Qualcomm	1
#define NVT_PROXIMITY_FUNC_SUPPORT 0

#define IC 3    /*1:NT11002 3:NT11003 4:NT11004 */
#if (IC==2||IC==4)
#define BUFFER_LENGTH    16384
#else
#define BUFFER_LENGTH    26624
#endif
//struct  NVTtouch_i2c_platform_data {
//   int power_gpio;
//   int irq_gpio;
//   int reset_gpio;
//};


struct NVTtouch_ts_data {
	uint16_t addr;
	uint8_t bad_data;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_reset;/*use RESET flag*/
	int use_irq;/*use EINT flag*/
	//int pw_gpio;
	int read_mode;/*read moudle mode,20110221 by andrew*/
	struct hrtimer timer;
	struct work_struct  work;
	char phys[32];
	int retry;
	struct early_suspend early_suspend;
	int (*power)(struct NVTtouch_ts_data * ts, int on);
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t max_touch_num;
	uint8_t max_button_num;
	uint8_t int_trigger_type;
	uint8_t chip_ID;
	uint8_t green_wake_mode;
};

#if NVT_TOUCH_CTRL_DRIVER
struct nvt_flash_data {
	rwlock_t lock;
	unsigned char bufferIndex;
	unsigned int length;
	struct i2c_client *client;
};
#endif
	

//*****************************End of Part I *********************************

//*************************Touchkey Surpport Part*****************************
//#define HAVE_TOUCH_KEY
#ifdef HAVE_TOUCH_KEY
	#define READ_COOR_ADDR 0x00
	const uint16_t touch_key_array[]={
	KEY_BACK,/*MENU*/
	KEY_HOME,/*HOME*/
	KEY_MENU/*CALL*/
	}; 
	#define MAX_KEY_NUM	 (sizeof(touch_key_array)/sizeof(touch_key_array[0]))
#else
	#define READ_COOR_ADDR 0x00
#endif

//*************************End of Touchkey Surpport Part*****************************


#endif /* _LINUX_NVT_TOUCH_H */
