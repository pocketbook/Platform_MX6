/*
 *  zForce touchscreen driver
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
      
#include <linux/input.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/input/mt.h>
#include <linux/suspend.h>
#include <linux/proc_fs.h>
#include <linux/pm.h>
#include <mach/system.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>

#include "../../../arch/arm/mach-mx6/ntx_hwconfig.h"
extern volatile NTX_HWCONFIG *gptHWCFG;

static const char ZFORCE_TS_NAME[]	= "zForce-ir-touch";

#ifdef DEBUG
#define dbg(...) printk(__VA_ARGS__)
#else
#define dbg(...)
#endif

#define TS_POLL_PERIOD	msecs_to_jiffies(10) /* ms delay between samples */

#define tp_int_pin			183

#define DEFAULT_PANEL_W		800
#define DEFAULT_PANEL_H		600
//#define ZFORCE_TS_WIDTH			600
//#define ZFORCE_TS_HIGHT			800

//#define ZFORCE_TS_X_MAX 		ZFORCE_TS_WIDTH<<1
//#define ZFORCE_TS_Y_MAX 		ZFORCE_TS_HIGHT<<1
#define IDX_QUEUE_SIZE		20
#define IDX_PACKET_SIZE		129

static struct workqueue_struct *zForce_wq;
static volatile uint16_t g_touch_pressed, g_touch_triggered, g_ghost_occurred=0, g_touch_suspended;
static int g_zforce_initial_step;
static int giIRLED_Signal_AutoCalibration=0;
static unsigned short gwZforce_FW_verA[4];
static int custom_touchSizeLimit_set = 0;
static unsigned long lastINT = 0;

static unsigned long ZFORCE_TS_WIDTH=DEFAULT_PANEL_W;
static unsigned long ZFORCE_TS_HIGHT=DEFAULT_PANEL_H;
static unsigned long ZFORCE_TS_X_MAX=DEFAULT_PANEL_W; 
static unsigned long ZFORCE_TS_Y_MAX=DEFAULT_PANEL_H; 

static struct zForce_data {
	int intr_gpio;
	struct delayed_work	work;
	struct i2c_client *client;
	struct input_dev *input;
	wait_queue_head_t wait;
	unsigned char PD_Resp;
	unsigned char lower_amp_flag;
} zForce_ir_touch_data;

static uint8_t cmd_Resolution_v2[] = {0xEE, 0x05, 0x02, (DEFAULT_PANEL_H&0xFF), (DEFAULT_PANEL_H>>8), (DEFAULT_PANEL_W&0xFF), (DEFAULT_PANEL_W>>8)};
static const uint8_t cmd_TouchData_v2[] = {0xEE, 0x01, 0x04};
static uint8_t cmd_Frequency_v2[] = {0xEE, 0x07, 0x08, 50, 00, 100, 00, 100, 00};
static const uint8_t cmd_getFirmwareVer_v2[] = {0xEE, 0x01, 0x1E};
static const uint8_t cmd_Active_v2[] = {0xEE, 0x01, 0x01};
static const uint8_t cmd_Deactive_v2[] = {0xEE, 0x01, 0x00};
static const uint8_t cmd_Dual_touch_v2[] = {0xEE, 0x05,0x03,1,0,0,0};
static uint8_t cmd_SetTouchSizeLimits[] = {0xEE, 0x05, 0x09, 0, 0, 0, 0};
static const uint8_t cmd_LowSignalRequestX[] = {0xEE, 0x02, 0x0D, 0};
static const uint8_t cmd_LowSignalRequestY[] = {0xEE, 0x02, 0x0D, 1};
static const uint8_t cmd_FixedSinalStrengthX[] = {0xEE, 0x03, 0x0F, 0, 0x09};
static const uint8_t cmd_FixedSinalStrengthY[] = {0xEE, 0x03, 0x0F, 1, 0x09};
static uint8_t cmd_LowerPDAmpResp[] = {0xEE, 0x05, 0x22, 0x01, 0x00, 0x75, 0x37};

static uint8_t cmd_Resolution[] = {0x02, (DEFAULT_PANEL_H&0xFF), (DEFAULT_PANEL_H>>8), (DEFAULT_PANEL_W&0xFF), (DEFAULT_PANEL_W>>8)};
static const uint8_t cmd_TouchData[] = {0x04};
static const uint8_t cmd_Frequency[] = {0x08,10,100};
static const uint8_t cmd_getFirmwareVer[] = {0x0A};
static const uint8_t cmd_Active[] = {0x01};
static const uint8_t cmd_Deactive[] = {0x00};
static const uint8_t cmd_Dual_touch[] = {0x03,1,0,0,0};

static int last_x[16], last_y[16];
extern int gIsCustomerUi;

extern int in_keylock;

int gIsCustomerUi_PB = 1;

extern unsigned int msp430_read(unsigned int reg);
/*--------------------------------------------------------------*/

#define IR_TOUCH_RST		(4*32 + 9)	/* GPIO_5_9 */
static void _zForce_ir_touch_RESET(void)
{
	int iChk;
	printk(KERN_ERR "%s()\n",__FUNCTION__);
	gpio_direction_output(IR_TOUCH_RST, 0);
	msleep(20);
	gpio_direction_input(IR_TOUCH_RST);
	msleep(500);

	g_touch_pressed = 0; 
	g_touch_triggered = 0;

	iChk = i2c_master_send(zForce_ir_touch_data.client, cmd_Active_v2, sizeof(cmd_Active_v2));
	if(iChk<0) {
		printk(KERN_ERR "%s(),send active fail !\n",__FUNCTION__);
	}
}

static int zForce_ir_touch_detect_int_level(void)
{
	unsigned v;
	v = gpio_get_value(zForce_ir_touch_data.intr_gpio);
	return v;
}

static int __zForce_read_data (struct i2c_client *client, char* buffer)
{
	uint8_t buf_recv[2];
	int rc;
		
	while (zForce_ir_touch_detect_int_level())
		schedule_timeout(2);
	rc = i2c_master_recv(client, buf_recv, 2);
	if (0xEE != buf_recv[0]) {
		printk (KERN_ERR "[%s-%d] Error , frame start not found !!\n",__func__,__LINE__);
		return 0;
	}
	
	while (zForce_ir_touch_detect_int_level())
		schedule_timeout(2);
	return i2c_master_recv(client, buffer, buf_recv[1]);
}

/*	__zForce_ir_touch_init -- hand shaking with touch panel
 *
 *	1.recv hello packet
 */
static int __zForce_ir_touch_init(struct i2c_client *client)
{
	uint8_t buf_recv[10] = { 0 };
	uint8_t buf[10];

	if(8==gptHWCFG->m_val.bTouchCtrl || 11==gptHWCFG->m_val.bTouchCtrl) {
		printk ("[%s-%d] zForce controller with v2 command sets.\n",__func__,__LINE__);
		return i2c_master_send(client, cmd_getFirmwareVer_v2, sizeof(cmd_getFirmwareVer_v2));
	}else{
		printk ("[%s-%d] zForce controller with v1 command sets.\n",__func__,__LINE__);
		return i2c_master_send(client, cmd_getFirmwareVer, sizeof(cmd_getFirmwareVer));
	}	
}


static void _zForce_ir_touch_report_touch_up(char id)
{
	if (gIsCustomerUi_PB){
		dbg("[%s] id = %i\n",__func__,id);
		input_mt_slot(zForce_ir_touch_data.input, (int)id - 1);
		input_mt_report_slot_state(zForce_ir_touch_data.input,MT_TOOL_FINGER,0);
		//input_mt_report_pointer_emulation(zForce_ir_touch_data.input, true);
		input_sync(zForce_ir_touch_data.input);
	}
	else {
		input_report_abs(zForce_ir_touch_data.input, ABS_MT_TRACKING_ID, id);
		input_report_abs(zForce_ir_touch_data.input, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(zForce_ir_touch_data.input, ABS_MT_WIDTH_MAJOR, 0);
		input_report_abs(zForce_ir_touch_data.input, ABS_MT_POSITION_X, last_x[id]);
		input_report_abs(zForce_ir_touch_data.input, ABS_MT_POSITION_Y, last_y[id]);

		input_report_abs(zForce_ir_touch_data.input, ABS_Y, last_y[id]);
		input_report_abs(zForce_ir_touch_data.input, ABS_X, last_x[id]);
		input_report_abs(zForce_ir_touch_data.input, ABS_PRESSURE, 0);
		input_report_key(zForce_ir_touch_data.input, BTN_TOUCH, 0);
		input_sync(zForce_ir_touch_data.input);
	} 
	//dbg (KERN_ERR"[%s-%d] touch %d (%d, %d) up\n",__func__,__LINE__, id,last_x[id],last_y[id]);
	//dbg (KERN_ERR"[%s-%d] touch %d up\n",__func__,__LINE__, id);
	g_touch_pressed &= (~(1<<id));
}

static int zForce_ir_touch_recv_data(struct i2c_client *client, uint8_t *buf)
{
	uint8_t buf_recv[2]={0,};
	int result = 0;
	int rc = 0;
	char *pBuffer;
	static int sLowSignalXY, sLowSignalFlag;
	int i;

	if (buf == NULL)
		return -EINVAL;
	
	rc = i2c_master_recv(client, buf_recv, 2);
	if ( rc < 0 ){
		_zForce_ir_touch_RESET();
		return 0;
	}
	if (0xEE != buf_recv[0]) {
		if (0xEE != buf_recv[1]) {
			printk (KERN_ERR "[%s-%d] Error , frame start not found !!\n",__func__,__LINE__);
			return 0;
		}
		else
			i2c_master_recv(client, &buf_recv[1], 1);
	}
	if (buf_recv[1] > IDX_PACKET_SIZE) {
		printk ("[%s-%d] Packet size too large (%d).\n",__func__,__LINE__,buf_recv[1]);
		_zForce_ir_touch_RESET();
		return 0;
	}

	rc = i2c_master_recv(client, buf, buf_recv[1]);
	if ( rc < 0 ){
		_zForce_ir_touch_RESET();
		return 0;
	}
	
	switch (buf[0]) {
		case 0:
			dbg ("[%s-%d] command Deactivate ...\n",__func__,__LINE__);
			break;
		case 1:
			dbg ("[%s-%d] command Activate (%d) ...\n",__func__,__LINE__,buf[1]);
#if 0
			if(8==gptHWCFG->m_val.bTouchCtrl || 11==gptHWCFG->m_val.bTouchCtrl) {  // v2 & v3
				i2c_master_send(client, cmd_Resolution_v2, sizeof(cmd_Resolution_v2));
			}else{
				i2c_master_send(client, cmd_Resolution, sizeof(cmd_Resolution));
			}
			g_zforce_initial_step = 2;
#else
			if(giIRLED_Signal_AutoCalibration) {
//				dbg ("[%s-%d] send cmd_FixedSinalStrengthX\n",__func__,__LINE__);
				zForce_ir_touch_data.PD_Resp = 0x3F;
				zForce_ir_touch_data.lower_amp_flag = 0;
				i2c_master_send(client, cmd_FixedSinalStrengthX, sizeof(cmd_FixedSinalStrengthX));
				g_zforce_initial_step = 1;
			}
			else {
				if(8==gptHWCFG->m_val.bTouchCtrl || 11==gptHWCFG->m_val.bTouchCtrl) {  // v2 & v3
					i2c_master_send(client, cmd_Resolution_v2, sizeof(cmd_Resolution_v2));
				}else{
					i2c_master_send(client, cmd_Resolution, sizeof(cmd_Resolution));
				}
				g_zforce_initial_step = 2;
			}
#endif
			break;
		case 2:
			dbg ("[%s-%d] command Resolution (%d) ...\n",__func__,__LINE__,buf[1]);
			if(8==gptHWCFG->m_val.bTouchCtrl || 11==gptHWCFG->m_val.bTouchCtrl) {
				i2c_master_send(client, cmd_Frequency_v2, sizeof(cmd_Frequency_v2));
			}else{
				i2c_master_send(client, cmd_Frequency, sizeof(cmd_Frequency));
			}	
//			dbg ("[%s-%d] send cmd_Frequency\n",__func__,__LINE__);
			g_zforce_initial_step = 8;
			break;
		case 3:
			dbg ("[%s-%d] command Configuration ...\n",__func__,__LINE__);
			if(8==gptHWCFG->m_val.bTouchCtrl || 11==gptHWCFG->m_val.bTouchCtrl) {
				i2c_master_send(client, cmd_TouchData_v2, sizeof(cmd_TouchData_v2));
			}else{
				i2c_master_send(client, cmd_TouchData, sizeof(cmd_TouchData));
			}	
//			dbg ("[%s-%d] send cmd_StartTouchData\n",__func__,__LINE__);
			g_zforce_initial_step = 4;
			break;
		case 4:
//			dbg ("[%s-%d] command Touch Data (count %d)...\n",__func__,__LINE__,buf[1]);
//			dbg ("[%02X %02X %02X %02X %02X %02X %02X]\n",buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8]);
			result = 1;
			break;
		case 7:
			if (5 == buf_recv[1])
				dbg (KERN_ERR "[%s-%d] command BootComplete (%02X %02X %02X %02X)...\n",__func__,__LINE__,buf[1],buf[2],buf[3],buf[4]);
			else
				dbg (KERN_ERR "[%s-%d] command BootComplete (%02X)...\n",__func__,__LINE__,buf[1]);
			break;
		case 8:
			dbg ("[%s-%d] command Frequency (%d) ...\n",__func__,__LINE__,buf[1]);
			if(8==gptHWCFG->m_val.bTouchCtrl || 11==gptHWCFG->m_val.bTouchCtrl) {
				i2c_master_send(client, cmd_Dual_touch_v2, sizeof(cmd_Dual_touch_v2));
			}else{
				i2c_master_send(client, cmd_Dual_touch, sizeof(cmd_Dual_touch));
			}
//			dbg ("[%s-%d] send cmd_SetConfiguration\n",__func__,__LINE__);
			g_zforce_initial_step = 3;
			break;
		case 0x09:
			dbg ("[%s-%d] Set touch size limits Response\n",__func__,__LINE__);
			if( 0 == buf[1] ) {
				dbg("set touch size limits succeed\n");
			}
			else {
				dbg("set touch size limits failed\n");
			}
			break;
		case 0x0A:
			dbg ("[%s-%d] firmware version %02X%02X %02X%02X %02X%02X %02X%02X \n", __func__, __LINE__, \
				buf[2], buf[1], buf[4], buf[3], buf[6], buf[5], buf[8], buf[7]);
			break;
		case 0x0D:
			dbg ("[%s-%d] Low signal Response\n",__func__,__LINE__);
			int i,numberOfSignals;
			int lowLedMask=0;
			numberOfSignals = buf[2];

//			dbg ("number of signals = %d\n",numberOfSignals);
			for ( i=0; i< (numberOfSignals+7)/8 ; i++ ) {
				lowLedMask = lowLedMask | buf[3+i] << 8*i;
			}
//			dbg("low signal leds = %x\n",lowLedMask);
			if ( 0 == buf[1] ) {
				if (!sLowSignalFlag)
					break;
				i2c_master_send(client, cmd_LowSignalRequestY, sizeof(cmd_LowSignalRequestY));
//				dbg ("[%s-%d] send cmd_LowSignalRequestY\n",__func__,__LINE__);
				input_report_zforcekey(zForce_ir_touch_data.input, BTN_X, lowLedMask);
				sLowSignalXY |= 1;
			}
			else {
				input_report_zforcekey(zForce_ir_touch_data.input, BTN_Y, lowLedMask);
				sLowSignalXY |= 2;
				if (!sLowSignalFlag) {
					input_report_zforcekey(zForce_ir_touch_data.input, BTN_X, 0);
					input_report_zforcekey(zForce_ir_touch_data.input, BTN_Y, 0);
					sLowSignalFlag=0;
					sLowSignalXY = 0;
				}
			}
			input_sync(zForce_ir_touch_data.input);
			break;
		case 0x1E:
			if(8==gptHWCFG->m_val.bTouchCtrl || 11==gptHWCFG->m_val.bTouchCtrl) {
				gwZforce_FW_verA[3] = buf[2]<<8|buf[1];
				gwZforce_FW_verA[2] = buf[4]<<8|buf[3];
				gwZforce_FW_verA[1] = buf[6]<<8|buf[5];
				gwZforce_FW_verA[0] = buf[8]<<8|buf[7];

				printk ("[%s-%d] firmware version %04X %04X %04X %04X \n", __func__, __LINE__, \
					gwZforce_FW_verA[3],gwZforce_FW_verA[2],gwZforce_FW_verA[1],gwZforce_FW_verA[0]);

				if( gwZforce_FW_verA[3]>=0x0005 && gwZforce_FW_verA[2]>=0x0002 && \
						gwZforce_FW_verA[1]>=0x0001 )
				{
					// QF2 firmware version is 0005 0002 0001 0003
					// Q92 firmware version is 0005 0002 0001 0000
					giIRLED_Signal_AutoCalibration=1;
					printk("[%s-%d] IR signal auto calibration enabled\n",__func__,__LINE__);
				}
				printk("[%s-%d] Interface protocol: major=0x%02x, minor=0x%02x\n",__func__,__LINE__,buf[46],buf[47]);
				if ( (buf[46]==0 && buf[47]==0 ) && 11==gptHWCFG->m_val.bTouchCtrl) {
					printk("[%s-%d] Warning! using new(v5.2 or later) format with old NN firmware\n",__func__,__LINE__);
				}
			}
			break;
		case 0x25:
			dbg (KERN_ERR "[%s-%d] command overrun (%d) ...\n",__func__,__LINE__,g_zforce_initial_step);
			switch (g_zforce_initial_step) {
				case 1:
					if(8==gptHWCFG->m_val.bTouchCtrl || 11==gptHWCFG->m_val.bTouchCtrl) {
						i2c_master_send(client, cmd_Resolution_v2, sizeof(cmd_Resolution_v2));
					}else{
						i2c_master_send(client, cmd_Resolution, sizeof(cmd_Resolution));
					}
					dbg ("[%s-%d] send cmd_Resolution\n",__func__,__LINE__);
					break;
				case 2:
					if(8==gptHWCFG->m_val.bTouchCtrl || 11==gptHWCFG->m_val.bTouchCtrl) {
						i2c_master_send(client, cmd_Frequency_v2, sizeof(cmd_Frequency_v2));
					}else{
						i2c_master_send(client, cmd_Frequency, sizeof(cmd_Frequency));
					}
					dbg ("[%s-%d] send cmd_Frequency\n",__func__,__LINE__);
					break;
				case 8:
					if(8==gptHWCFG->m_val.bTouchCtrl || 11==gptHWCFG->m_val.bTouchCtrl) {
						i2c_master_send(client, cmd_TouchData_v2, sizeof(cmd_TouchData_v2));
					}else{
						i2c_master_send(client, cmd_TouchData, sizeof(cmd_TouchData));
					}	
					dbg ("[%s-%d] send cmd_StartTouchData\n",__func__,__LINE__);
					break;
			}
			break;
		case 0x30:
			dbg ("[%s-%d] Low signal notification\n",__func__,__LINE__);
			if ( buf[1] ) {
				dbg("Low signal exists!\n");
				sLowSignalFlag = 1;
				i2c_master_send(client, cmd_LowSignalRequestX, sizeof(cmd_LowSignalRequestX));
//				dbg ("[%s-%d] send cmd_LowSignalRequestX\n",__func__,__LINE__);
			}
			else {
				dbg("No low signals!\n");

				if(g_touch_pressed) {
					unsigned char bTouchID_Bits=(unsigned char)g_touch_pressed;
					// restore the lost touch up .
					for(i=0;i<3;i++)
					{
						if( bTouchID_Bits&(1<<i) ) {
							_zForce_ir_touch_report_touch_up(i);
						}
					}
				}
				if (3 == sLowSignalXY) {
					input_report_zforcekey(zForce_ir_touch_data.input, BTN_X, 0);
					input_report_zforcekey(zForce_ir_touch_data.input, BTN_Y, 0);
					input_sync(zForce_ir_touch_data.input);
					sLowSignalXY = 0;
				}
				sLowSignalFlag = 0;
			}
			break;
		case 0x0F:		// fixed signal strength with 10 step current strength.
			if ((3 > buf_recv[1]) || (buf_recv[1] != (buf[2]+3))) {
//				dbg ("[%s-%d] Fixed SignalStrength response with wrong length. (%02X %02X %02X)\n",__func__,__LINE__, buf_recv[0], buf_recv[1], buf[0]);
				break;
			}
//			dbg ("[%s-%d] Fixed SignalStrength %c Response (%d Signals) : \n",__func__,__LINE__,buf[1]?'Y':'X', buf[2]);
//			dbg ("0x%02X 0x%02X ",buf_recv[0], buf_recv[1]);
			for (i=0; i<(buf[2]+3) ; i++) {
//				dbg ("0x%02X ", buf[i]);
				if (3 <= i && 230 < buf[i])
					zForce_ir_touch_data.lower_amp_flag = 1;
			}
//			dbg ("\n");
			if (buf[1]) {
				if (0x0F != g_zforce_initial_step)
					break;
				if (zForce_ir_touch_data.lower_amp_flag && (0x07 < zForce_ir_touch_data.PD_Resp)) {
					zForce_ir_touch_data.PD_Resp -= 0x08;
					cmd_LowerPDAmpResp[6] = zForce_ir_touch_data.PD_Resp;
					dbg ("[%s-%d] Lower amp of PD response. Set reg RX_FB 0x%02X. \n",__func__,__LINE__,zForce_ir_touch_data.PD_Resp);
					i2c_master_send(client, cmd_LowerPDAmpResp, sizeof(cmd_LowerPDAmpResp));
					zForce_ir_touch_data.lower_amp_flag = 0;
					g_zforce_initial_step = 0x22;
				}
				else {
					if(8==gptHWCFG->m_val.bTouchCtrl || 11==gptHWCFG->m_val.bTouchCtrl) {  // v2 & v3
						i2c_master_send(client, cmd_Resolution_v2, sizeof(cmd_Resolution_v2));
					}else{
						i2c_master_send(client, cmd_Resolution, sizeof(cmd_Resolution));
					}
//					dbg ("[%s-%d] send cmd_Resolution\n",__func__,__LINE__);
					g_zforce_initial_step = 2;
				}
			}
			else if (1 == g_zforce_initial_step){
//				dbg ("[%s-%d] send cmd_FixedSinalStrengthY\n",__func__,__LINE__);
				i2c_master_send(client, cmd_FixedSinalStrengthY, sizeof(cmd_FixedSinalStrengthY));
				g_zforce_initial_step = 0x0F;
			}
			break;
		case 0x22:		// set register RX_FB response
			dbg ("[%s-%d] Lower amp of PD response %x \n",__func__,__LINE__,buf[1]);
			zForce_ir_touch_data.lower_amp_flag = 0;
//			dbg ("[%s-%d] send cmd_FixedSinalStrengthX\n",__func__,__LINE__);
			i2c_master_send(client, cmd_FixedSinalStrengthX, sizeof(cmd_FixedSinalStrengthX));
			g_zforce_initial_step = 1;
			break;
		default:
			dbg (KERN_ERR "[%s-%d] undefined command %d (%d bytes)...\n",__func__,__LINE__, *buf, buf_recv[1]);
			break;
	}
	return result;
}


static void zForce_ir_touch_report_data(struct i2c_client *client, uint8_t *buf)
{
	int state;
	int x,y,packets=buf[1];
	char id, *packet = buf+2;
	while (packets--) {

#ifdef DEBUG
		printk("[%s] pac %i\n",__func__, packets);
		char *i = packet;
		while(i < packet + 9) printk(" %hhx ",*i++);
		printk(" -- \n");
#endif

		if(8==gptHWCFG->m_val.bTouchCtrl) {    //neonode v2
			state = packet[4] & 0x03;
			id = (packet[4]>>2 & 0x0F);
		} else if(11==gptHWCFG->m_val.bTouchCtrl) {    //neonode v3
			state = packet[4] & 0x0F;
			id = (packet[4]>>4 & 0x0F);
		} else {
			state = (packet[4] >> 6) & 0x03;
			id = (packet[4] >> 2)  & 0x0F;
		}
		dbg("[%s] state = %i; id = %i\n",__func__, state, id);

		if (0==state || 1==state) { //key down & move
			if (g_ghost_occurred) {
				printk("ghost occurred! Ignore id %d\n",id);
			} else if (gIsCustomerUi_PB) {
				y = packet[2] | (packet[3] << 8);
				x = packet[0] | (packet[1] << 8);

				input_mt_slot(zForce_ir_touch_data.input, (int)id - 1);
				input_mt_report_slot_state(zForce_ir_touch_data.input,MT_TOOL_FINGER,1);
				input_report_abs(zForce_ir_touch_data.input, ABS_MT_POSITION_X, x);
				input_report_abs(zForce_ir_touch_data.input, ABS_MT_POSITION_Y, y);

				g_touch_pressed |= (1<<id);

				//dbg (KERN_ERR"[%s-%d] touch down (%d, %d, %d)\n",__func__,__LINE__,x1,y1, pressure);
				//dbg (KERN_ERR"[%s-%d] touch %d (%d, %d) down\n",__func__,__LINE__, id,x,y);
			}
			else {
				x = packet[2] | (packet[3] << 8);
//				y = ZFORCE_TS_HIGHT -(packet[0] | (packet[1] << 8)) -1;
				y = packet[0] | (packet[1] << 8);
				input_report_abs(zForce_ir_touch_data.input, ABS_MT_TRACKING_ID, id);
				input_report_abs(zForce_ir_touch_data.input, ABS_MT_TOUCH_MAJOR, 1);
				input_report_abs(zForce_ir_touch_data.input, ABS_MT_WIDTH_MAJOR, 1);
				input_report_abs(zForce_ir_touch_data.input, ABS_MT_POSITION_X, x);
				input_report_abs(zForce_ir_touch_data.input, ABS_MT_POSITION_Y, y);
				/** single touch ABS **/
				input_report_abs(zForce_ir_touch_data.input, ABS_Y, y);
				input_report_abs(zForce_ir_touch_data.input, ABS_X, x);
				input_report_abs(zForce_ir_touch_data.input, ABS_PRESSURE, 1024);
				input_report_key(zForce_ir_touch_data.input, BTN_TOUCH, 1);
				//dbg (KERN_ERR"[%s-%d] touch %d (%d, %d)\n",__func__,__LINE__, id,x,y);
				last_x[id] = x;
				last_y[id] = y;
				g_touch_pressed |= (1<<id);
			}
		}
		else if(state==2){ // key up
			if(g_ghost_occurred){
				if(g_ghost_occurred&(1<<id)){
					printk("ghost remaining finger physically removed. id = %d\n",id);
					g_ghost_occurred = 0;
				}
			} else {
				_zForce_ir_touch_report_touch_up(id);
			}
		}
		else if(state==3) {
			printk("invalid point detected. discard!\n");
		} else if(state==4) {
			printk ("[%s-%d] touch %d remains when ghost touch detected\n",__func__,__LINE__, id);
			_zForce_ir_touch_report_touch_up(id);
			printk ("[%s-%d] force up remaining finger %d\n",__func__,__LINE__, id);
			g_ghost_occurred |= (1<<id);
		}
		else {
			printk("%s() : get unknown state=%d\n",__FUNCTION__);
		}

	//	printk ("[%s-%d] flag %04X, id %d\n",__func__,__LINE__,g_touch_pressed,id);
		//input_mt_sync(zForce_ir_touch_data.input);
		packet += (8==gptHWCFG->m_val.bTouchCtrl || 11==gptHWCFG->m_val.bTouchCtrl)?9:7;
	}

//	input_mt_report_pointer_emulation(zForce_ir_touch_data.input, 1);
	input_sync(zForce_ir_touch_data.input);

	schedule();	// Joseph 20101023
}

static uint8_t gzForceBuffer[IDX_QUEUE_SIZE+2][IDX_PACKET_SIZE];
static int gQueueRear, gQueueFront;

void zForce_ir_touch_enqueue (void)
{
	int packets = zForce_ir_touch_recv_data(zForce_ir_touch_data.client, gzForceBuffer[gQueueRear]);
	if(0 < packets) {
		if ((gQueueRear+packets) >= IDX_QUEUE_SIZE) {
//			printk ("[%s-%d] touch queue overflowed. (R %d, p %d, move %d)  \n",__func__,__LINE__,gQueueRear,packets,((gQueueRear+packets)-(IDX_QUEUE_SIZE-1)));
			memmove (gzForceBuffer, &gzForceBuffer[IDX_QUEUE_SIZE], ((gQueueRear+packets)-(IDX_QUEUE_SIZE-1))*IDX_PACKET_SIZE);
		}
		if (((gQueueRear+packets)%IDX_QUEUE_SIZE) == gQueueFront)
			printk ("[%s-%d] touch queue full\n",__func__,__LINE__);
		else
			gQueueRear = (gQueueRear+packets)%IDX_QUEUE_SIZE;
	}
}

static void zForce_ir_touch_work_func(struct work_struct *work)
{
	zForce_ir_touch_enqueue ();	
	while (gQueueRear != gQueueFront){
		zForce_ir_touch_report_data(zForce_ir_touch_data.client, gzForceBuffer[gQueueFront]);
		gQueueFront = (gQueueFront+1)%IDX_QUEUE_SIZE;
	}
	if (!zForce_ir_touch_detect_int_level ())
		schedule_delayed_work(&zForce_ir_touch_data.work, 1);
	else
		g_touch_triggered = 0;
}

static irqreturn_t zForce_ir_touch_ts_interrupt(int irq, void *dev_id)
{
	lastINT = jiffies;
	g_touch_triggered = 1;
	if (g_touch_suspended)
		printk("!!!!!!  Touched triggered after driver suspend.\n");
	else
		schedule_delayed_work(&zForce_ir_touch_data.work, 0);
	return IRQ_HANDLED;
}

void zForce_ir_touch_ts_triggered(void)
{
	g_touch_triggered = 1;
	schedule_delayed_work(&zForce_ir_touch_data.work, 0);
}

//tatic const struct i2c_device_id neonode_ts_id[] = {
//	{ "neonode_ts", 0 },
//	{ }
//};

static void zForce_suspend(void)
{
	if(8==gptHWCFG->m_val.bTouchCtrl || 11==gptHWCFG->m_val.bTouchCtrl) {
		i2c_master_send(zForce_ir_touch_data.client, cmd_Deactive_v2, sizeof(cmd_Deactive_v2));
	}else{
		i2c_master_send(zForce_ir_touch_data.client, cmd_Deactive, sizeof(cmd_Deactive));
	}
	msleep(200);
//	disable_irq_wake(zForce_ir_touch_data.client->irq);
	free_irq(zForce_ir_touch_data.client->irq, ZFORCE_TS_NAME);

	g_touch_pressed = 0;
	g_touch_triggered = 0;
	g_ghost_occurred = 0;
}

static void zForce_resume(void)
{
	if(8==gptHWCFG->m_val.bTouchCtrl || 11==gptHWCFG->m_val.bTouchCtrl) {
		i2c_master_send(zForce_ir_touch_data.client, cmd_Active_v2, sizeof(cmd_Active_v2));
	}else{
		i2c_master_send(zForce_ir_touch_data.client, cmd_Active, sizeof(cmd_Active));
	}	
	if( custom_touchSizeLimit_set ) {
		printk("command setTouchSizeLimits");
		i2c_master_send(zForce_ir_touch_data.client, cmd_SetTouchSizeLimits, sizeof(cmd_SetTouchSizeLimits));
		msleep(5);
	}
//	enable_irq_wake(zForce_ir_touch_data.client->irq);
	request_irq(zForce_ir_touch_data.client->irq, zForce_ir_touch_ts_interrupt, IRQF_TRIGGER_FALLING, ZFORCE_TS_NAME, ZFORCE_TS_NAME);
	if (!zForce_ir_touch_detect_int_level ()) {
		zForce_ir_touch_ts_triggered ();
	}
}

static ssize_t FW_ver_read(struct device *dev, struct device_attribute *attr,
			char *buf)
{

	sprintf(buf,"%04X %04X %04X %04X\n",gwZforce_FW_verA[3],gwZforce_FW_verA[2],gwZforce_FW_verA[1],gwZforce_FW_verA[0]);
	return strlen(buf);
}

static ssize_t neo_info(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return 0;
}

static ssize_t neo_ctl(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	switch(buf[0])
	{
		case 'a':
			printk("NeoActivate \n");
			if(8==gptHWCFG->m_val.bTouchCtrl || 11==gptHWCFG->m_val.bTouchCtrl) {
				i2c_master_send(zForce_ir_touch_data.client, cmd_Active_v2, sizeof(cmd_Active_v2));
			}else{
				i2c_master_send(zForce_ir_touch_data.client, cmd_Active, sizeof(cmd_Active));
			}	
			break;
		case 'd':
			printk("NeoDeactivate \n");
			if(8==gptHWCFG->m_val.bTouchCtrl || 11==gptHWCFG->m_val.bTouchCtrl) {
				i2c_master_send(zForce_ir_touch_data.client, cmd_Deactive_v2, sizeof(cmd_Deactive_v2));
			}else{
				i2c_master_send(zForce_ir_touch_data.client, cmd_Deactive, sizeof(cmd_Deactive));
			}	
			break;
		case 's':
			printk("NeoSetResolution \n");
			if(8==gptHWCFG->m_val.bTouchCtrl || 11==gptHWCFG->m_val.bTouchCtrl) {
				i2c_master_send(zForce_ir_touch_data.client, cmd_Resolution_v2, sizeof(cmd_Resolution_v2));
			}else{
				i2c_master_send(zForce_ir_touch_data.client, cmd_Resolution, sizeof(cmd_Resolution));
			}	
			break;
		case 'l':
			printk("Get LedSignalLevel \n");
//			i2c_master_send(zForce_ir_touch_data.client, cmd_LedLevel, sizeof(cmd_LedLevel));
			break;
	}
	return count;
}

static ssize_t touchSizeLimit_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int max_enable, max_size, min_enable, min_size;
	sscanf (buf,"%d %d %d %d", &max_enable, &max_size, &min_enable, &min_size);
//	printk( "input values = %d %d %d %d\n", max_enable, max_size, min_enable, min_size);
	cmd_SetTouchSizeLimits[3] = max_enable;
	cmd_SetTouchSizeLimits[4] = max_size;
	cmd_SetTouchSizeLimits[5] = min_enable;
	cmd_SetTouchSizeLimits[6] = min_size;

#if 0
	printk("%s() skipped set to zforce max enable/size(%d/%d),min enable/size(%d/%d)!!\n",
			__FUNCTION__,max_enable,max_size,min_enable,min_size);
#else
	i2c_master_send(zForce_ir_touch_data.client, cmd_SetTouchSizeLimits, sizeof(cmd_SetTouchSizeLimits));
	custom_touchSizeLimit_set = 1;
#endif

	return count;
}

static ssize_t scanningFrequency_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc = 0;
	int freq = 0;
	sscanf (buf,"%d", &freq);
//	printk( "input values = %d\n", freq);
	if (freq < 0 || freq > 65535) {
		printk("Scanning frequency valid range 0~65535\n");
		return -EINVAL;
	}
	cmd_Frequency_v2[5] = freq&0xFF;
	cmd_Frequency_v2[6] = freq>>8;

	rc = i2c_master_send(zForce_ir_touch_data.client, cmd_Frequency_v2, sizeof(cmd_Frequency_v2));

	return rc?rc:count;
}

static DEVICE_ATTR(neocmd, 0644, neo_info, neo_ctl);
static DEVICE_ATTR(touchSizeLimit, S_IWUSR, NULL, touchSizeLimit_set);
static DEVICE_ATTR(scanningFreq, S_IWUSR, NULL, scanningFrequency_set);
static DEVICE_ATTR(FW_ver, S_IRUGO, FW_ver_read, NULL);

// DmitryZ: /proc/keylock

static int proc_keylock_write(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
        char buf[16];

        if (count > sizeof(buf) -1 )
                return -EINVAL;

        if (!count)
                return 0;

        if (copy_from_user(buf, buffer, count))
                return -EFAULT;

        buf[count] = '\0';

        switch (buf[0]) {
                case '0':
			if (in_keylock) zForce_resume();
                        in_keylock = 0;
                        break;
                default:
                        if (! in_keylock) zForce_suspend();
                        in_keylock = 1;
                        break;
        }

        return count;
}

static int proc_keylock_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
        *eof = 1;
        return snprintf(page, PAGE_SIZE, "%u\n", in_keylock ? 1 : 0);
}

static int zForce_ts_lock_init(void)
{
        struct proc_dir_entry *dir, *file;

        dir = proc_mkdir("keylock", NULL);
        if (!dir) {
                printk(KERN_ERR "could not create /proc/keylock\n");
                return -1;
        }

        file = create_proc_entry("lock", S_IRUGO | S_IWUGO, dir);
        if (!file) {
                printk(KERN_ERR "could not create /proc/keylock/lock\n");
                return -1;
        }

        file->read_proc = proc_keylock_read;
        file->write_proc = proc_keylock_write;

        return 0;
}

static const struct attribute *sysfs_zforce_attrs[] = {
	&dev_attr_neocmd.attr,
	&dev_attr_touchSizeLimit.attr,
	&dev_attr_scanningFreq.attr,
	&dev_attr_FW_ver.attr,
	NULL,
};

extern int gSleep_Mode_Suspend;

static int zForce_ir_touch_suspend(struct platform_device *pdev, pm_message_t state)
{
	if (! time_after(jiffies, lastINT+HZ/2)) {
		// may be unhandled events
		return -EBUSY;
	}

	if(g_touch_pressed) {
		printk(KERN_ERR"[WARNING] g_touch_pressed=%d, g_touch_triggered=%d\n in suspending",g_touch_pressed,g_touch_triggered);
		if (time_after(jiffies, lastINT+60*HZ)){
			int i;
			printk("[Warning] seems touch lost, trigger a force touch up!!\n");
			for(i=1;i<3;i++)
			{
				if( g_touch_pressed&(1<<i) ) {
					printk("Force up id %d\n",i);
					_zForce_ir_touch_report_touch_up(i);
				}
			}
			g_touch_pressed = 0;
		}
		return -EBUSY;
	}
	

	/* return immediatly if the driver is still handling touch data */
	if (g_touch_triggered) {
		printk("[%s-%d] zForce still handling touch data (%d, %d)\n",__func__,__LINE__,g_touch_pressed,g_touch_triggered);
		return -EBUSY;
	}

	/* the driver wants to send data, trigger a read */
	if (!zForce_ir_touch_detect_int_level ()) 
	{
		zForce_ir_touch_ts_triggered ();
		printk ("[%s-%d] zForce touch event not processed (%d, %d).\n",__func__,__LINE__,g_touch_pressed,g_touch_triggered);
		return -1;
	}
	else if (g_touch_triggered) {
		printk("[%s-%d] zForce still handling touch data\n");
		return -EBUSY;
	}

	enable_irq_wake(zForce_ir_touch_data.client->irq);
	g_touch_suspended = 1;
	return 0;
}

static int zForce_ir_touch_resume(struct platform_device *pdev)
{
	g_touch_suspended = 0;
	disable_irq_wake(zForce_ir_touch_data.client->irq);
	if (!zForce_ir_touch_detect_int_level ()) {
		zForce_ir_touch_ts_triggered ();
	}
	
	return 0;
}

static int zforce_i2c_open(struct input_dev *dev)
{
	struct i2c_client *client = input_get_drvdata(dev);

	printk ("[%s-%d] %s()\n",__FILE__,__LINE__,__func__);
	if(8==gptHWCFG->m_val.bTouchCtrl || 11==gptHWCFG->m_val.bTouchCtrl) {
		i2c_master_send(client, cmd_Active_v2, sizeof(cmd_Active_v2));
	}else{
		i2c_master_send(client, cmd_Active, sizeof(cmd_Active));
	}
	g_zforce_initial_step = 1;	
//	i2c_master_send(client, cmd_Resolution, sizeof(cmd_Resolution));
	
	return 0;
}

static void zforce_i2c_close(struct input_dev *dev)
{
	struct i2c_client *client = input_get_drvdata(dev);
	
	printk ("[%s-%d] %s()\n",__FILE__,__LINE__,__func__);
	if(8==gptHWCFG->m_val.bTouchCtrl || 11==gptHWCFG->m_val.bTouchCtrl) {
		i2c_master_send(client, cmd_Deactive_v2, sizeof(cmd_Deactive_v2));
	}else{
		i2c_master_send(client, cmd_Deactive, sizeof(cmd_Deactive));
	}	
}

static int zForce_ir_touch_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;

	zForce_ir_touch_data.client = client;
	strlcpy(client->name, ZFORCE_TS_NAME, I2C_NAME_SIZE);

	INIT_DELAYED_WORK(&zForce_ir_touch_data.work, zForce_ir_touch_work_func);
	
	zForce_ir_touch_data.intr_gpio = (client->dev).platform_data;
	
	zForce_ir_touch_data.input = input_allocate_device();
	if (zForce_ir_touch_data.input == NULL) {
		err = -ENOMEM;
		goto fail;
	}

	err = request_irq(zForce_ir_touch_data.client->irq, zForce_ir_touch_ts_interrupt, IRQF_TRIGGER_FALLING, ZFORCE_TS_NAME, ZFORCE_TS_NAME);
	if (err < 0) {
		printk("%s(%s): Can't allocate irq %d\n", __FILE__, __func__, zForce_ir_touch_data.client->irq);
	    goto fail;
	}
//	enable_irq_wake(zForce_ir_touch_data.client->irq);

	err = __zForce_ir_touch_init(client);
	if (err < 0) {
	    printk("%s(%s): initial failed.\n", __FILE__, __func__);
	    goto fail;
	}

	zForce_ir_touch_data.input->name = ZFORCE_TS_NAME;
	zForce_ir_touch_data.input->id.bustype = BUS_I2C;
	zForce_ir_touch_data.input->open = zforce_i2c_open;
	zForce_ir_touch_data.input->close = zforce_i2c_close;

	input_set_drvdata(zForce_ir_touch_data.input, client);
	
	set_bit(EV_SYN, zForce_ir_touch_data.input->evbit);
	
//	set_bit(EV_KEY, zForce_ir_touch_data.input->evbit);
//	set_bit(BTN_TOUCH, zForce_ir_touch_data.input->keybit);
//	set_bit(BTN_2, zForce_ir_touch_data.input->keybit);
//	set_bit(BTN_X, zForce_ir_touch_data.input->keybit);
//	set_bit(BTN_Y, zForce_ir_touch_data.input->keybit);
	
	set_bit(EV_ABS, zForce_ir_touch_data.input->evbit);
//	set_bit(ABS_X, zForce_ir_touch_data.input->absbit);
//	set_bit(ABS_Y, zForce_ir_touch_data.input->absbit);
///	set_bit(ABS_PRESSURE, zForce_ir_touch_data.input->absbit);
//	set_bit(ABS_HAT0X, zForce_ir_touch_data.input->absbit);
//	set_bit(ABS_HAT0Y, zForce_ir_touch_data.input->absbit);

	set_bit(ABS_MT_POSITION_X, zForce_ir_touch_data.input->absbit);
	set_bit(ABS_MT_POSITION_Y, zForce_ir_touch_data.input->absbit);
	//set_bit(ABS_MT_TRACKING_ID, zForce_ir_touch_data.input->absbit);

	{
		if(1==gptHWCFG->m_val.bDisplayResolution) {
			// 1024x758 .
			ZFORCE_TS_WIDTH=1024;
			ZFORCE_TS_HIGHT=758;
		}
		else if(2==gptHWCFG->m_val.bDisplayResolution) {
			// 1024x768
			ZFORCE_TS_WIDTH=1024;
			ZFORCE_TS_HIGHT=768;
		}
		else if(3==gptHWCFG->m_val.bDisplayResolution) {
			// 1440x1080
			ZFORCE_TS_WIDTH=1440;
			ZFORCE_TS_HIGHT=1080;
		}
		else if(5==gptHWCFG->m_val.bDisplayResolution) {
			// 1448x1072
			ZFORCE_TS_WIDTH=1448;
			ZFORCE_TS_HIGHT=1072;
		}
		else if(8==gptHWCFG->m_val.bDisplayResolution) {
			// 1872x1404
			ZFORCE_TS_WIDTH=1872;
			ZFORCE_TS_HIGHT=1404;
		}
		else {
			// 800x600 
			ZFORCE_TS_WIDTH=800;
			ZFORCE_TS_HIGHT=600;
		}

		ZFORCE_TS_X_MAX=ZFORCE_TS_WIDTH;
		ZFORCE_TS_Y_MAX=ZFORCE_TS_HIGHT;
		
		printk ("[%s-%d] Set touch resolution %dx%d\n",__func__,__LINE__,ZFORCE_TS_WIDTH,ZFORCE_TS_HIGHT);
		cmd_Resolution[1] = (uint8_t)(ZFORCE_TS_HIGHT&0xff);
		cmd_Resolution[2] = (uint8_t)(ZFORCE_TS_HIGHT>>8);
		cmd_Resolution[3] = (uint8_t)(ZFORCE_TS_WIDTH&0xff);
		cmd_Resolution[4] = (uint8_t)(ZFORCE_TS_WIDTH>>8);

		cmd_Resolution_v2[3] = (uint8_t)(ZFORCE_TS_HIGHT&0xff);
		cmd_Resolution_v2[4] = (uint8_t)(ZFORCE_TS_HIGHT>>8);
		cmd_Resolution_v2[5] = (uint8_t)(ZFORCE_TS_WIDTH&0xff);
		cmd_Resolution_v2[6] = (uint8_t)(ZFORCE_TS_WIDTH>>8);
	}
//  input_set_abs_params(zForce_ir_touch_data.input, ABS_X, 0, ZFORCE_TS_X_MAX, 0, 0);
//	input_set_abs_params(zForce_ir_touch_data.input, ABS_Y, 0, ZFORCE_TS_Y_MAX, 0, 0);
//	input_set_abs_params(zForce_ir_touch_data.input, ABS_HAT0X, 0, ZFORCE_TS_X_MAX, 0, 0);
//	input_set_abs_params(zForce_ir_touch_data.input, ABS_HAT0Y, 0, ZFORCE_TS_Y_MAX, 0, 0);
	input_set_abs_params(zForce_ir_touch_data.input, ABS_MT_POSITION_X, 0, ZFORCE_TS_X_MAX, 0, 0);
	input_set_abs_params(zForce_ir_touch_data.input, ABS_MT_POSITION_Y, 0, ZFORCE_TS_Y_MAX, 0, 0);
//	input_set_abs_params(zForce_ir_touch_data.input, ABS_MT_TOUCH_MAJOR, 0, 2, 0, 0);
//	input_set_abs_params(zForce_ir_touch_data.input, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
//	input_set_abs_params(zForce_ir_touch_data.input, ABS_MT_TRACKING_ID, 1, 2, 0, 0);
//	input_set_abs_params(zForce_ir_touch_data.input, ABS_PRESSURE, 0, 2048, 0, 0);
	err = input_mt_init_slots(zForce_ir_touch_data.input, 2);
	if (err)
		printk("[%s] Init MT slots fail\n",__func__);

	err = input_register_device(zForce_ir_touch_data.input);
	if (err < 0) {
		pr_debug("Register device file!\n");
		goto fail;
	}

	err = sysfs_create_files(&zForce_ir_touch_data.input->dev.kobj, &sysfs_zforce_attrs);
	if (err) {
		pr_debug("Can't create device file!\n");
		return -ENODEV;
	}

	zForce_ts_lock_init();
	
	if (!zForce_ir_touch_detect_int_level()) {
		g_touch_triggered = 1;
		schedule_delayed_work(&zForce_ir_touch_data.work, 0);
	}
	return 0;

fail:
	input_free_device(zForce_ir_touch_data.input);
	cancel_delayed_work_sync (&zForce_ir_touch_data.work);
//	destroy_workqueue(&zForce_ir_touch_data.work);
	return err;
}

static int zForce_ir_touch_remove(struct i2c_client *client)
{
	sysfs_remove_files(&zForce_ir_touch_data.input->dev.kobj, &sysfs_zforce_attrs);

	cancel_delayed_work_sync (&zForce_ir_touch_data.work);
//	destroy_workqueue(&zForce_ir_touch_data.work);

	input_unregister_device(zForce_ir_touch_data.input);

	free_irq(client->irq, ZFORCE_TS_NAME);
	return 0;
}

/* -------------------------------------------------------------------- */
static const struct i2c_device_id zForce_ir_touch_id[] = {
    {"zforce-ir-touch", 0 },
	{ }
};

static struct i2c_driver zForce_ir_touch_driver = {
	.probe		= zForce_ir_touch_probe,
	.remove		= zForce_ir_touch_remove,
	.suspend	= zForce_ir_touch_suspend,
	.resume		= zForce_ir_touch_resume,
	.id_table	= zForce_ir_touch_id,
	.driver		= {
		.name = "zforce-ir-touch",
		.owner = THIS_MODULE,
	},
};

static int __init zForce_ir_touch_init(void)
{
	return i2c_add_driver(&zForce_ir_touch_driver);
}

static void __exit zForce_ir_touch_exit(void)
{
	i2c_del_driver(&zForce_ir_touch_driver);
}

module_init(zForce_ir_touch_init);
module_exit(zForce_ir_touch_exit);

MODULE_AUTHOR("Joseph Lai. ");
MODULE_DESCRIPTION("NeoNode zForce IR Touch Screen driver");
MODULE_LICENSE("GPL");
