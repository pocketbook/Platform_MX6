/* 
* drivers/input/touchscreen/cntouch_ts.c
*
* CNTouch TouchScreen driver. 
*
* Copyright (c) 2012  SHIH HUA TECHNOLOGY Ltd.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*
*    note: only support mulititouch    ChiachuHsu 2012-02-01
*/

//#define CONFIG_CNTouch_CUSTOME_ENV
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input/cntouch_i2c_ts.h>
#include <linux/gpio.h>
#include  <linux/imx6sl_evk/imx6sl_evk_gpio_cfg.h>
#include <linux/platform_device.h>

/* -------------- global variable definition -----------*/
static struct i2c_client *this_client = NULL;
static bool bInupgrade = false;
static REPORT_FINGER_INFO_T _st_finger_infos[CFG_MAX_POINT_NUM];
//static unsigned int _sui_irq_num= IRQ_EINT(6);
#if defined(CONFIG_SUPPORT_CNT_BUTTON)
int tsp_keycodes[CFG_NUMOFKEYS] ={

	KEY_BACK,
	KEY_MENU,
	KEY_HOME,
	KEY_SEARCH,
#ifdef CNT_PROXIMITY
	KEY_POWER
#endif

};
char *tsp_keyname[CFG_NUMOFKEYS] ={

	"Back",
	"Menu",
	"Home",
	"Search"
};

static bool tsp_keystatus[CFG_NUMOFKEYS];
#endif


#define GTP_SWAP(x, y)                 do{\
	typeof(x) z = x;\
	x = y;\
	y = z;\
}while (0)

#if defined(CONFIG_PB650_DEEP_STANDY)
extern int iDeepStandy;
#endif



#ifdef TP_FIRMWARE_UPDATE
static struct fw_version fw_v;
static unsigned char g_dwiic_info_data[1024];   // Buffer for info data
/*******************************************************	
Description:
Read data from the i2c slave device;
This operation consisted of 2 i2c_msgs,the first msg used
to write the operate address,the second msg used to read data.

Parameter:
client:	i2c device.
buf[0]:operate address.
buf[1]~buf[len]:read data buffer.
len:operate length.

return:
numbers of i2c_msgs to transfer
*********************************************************/
static int i2c_read_Cnt(struct i2c_client *client, uint8_t *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret = -1;
	int retries = 0;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = client->addr;
	msgs[0].len   = 1;
	msgs[0].buf   = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len-1;
	msgs[1].buf   = &buf[1];

	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, msgs, 2);
		if(ret == 2)	break;
		retries++;
	}
	return ret;
}

/*******************************************************	
Description:
write data to the i2c slave device.

Parameter:
client:	i2c device.
buf[0]:operate address.
buf[1]~buf[len]:write data buffer.
len:operate length.

return:
numbers of i2c_msgs to transfer.
*********************************************************/
static int i2c_write_Cnt(struct i2c_client *client, uint8_t *data, int len)
{
	struct i2c_msg msg;
	int ret = -1;
	int retries = 0;

	msg.flags = !I2C_M_RD;
	msg.addr  = client->addr;
	msg.len   = len;
	msg.buf   = data;		
	//printk("%s %s %d \n",__FILE__,__func__,__LINE__);

	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1)	break;
		retries++;
	}
	return ret;
}
static bool msg2133_i2c_read(char *pbt_buf, int dw_lenth)
{
    int ret;
    //printk("The msg_i2c_client->addr=0x%x\n",this_client->addr);
    ret = i2c_master_recv(this_client, pbt_buf, dw_lenth);

    if(ret <= 0){
        printk("msg2133_i2c_read error\n");
        return false;
    }

    return true;
}

static bool msg2133_i2c_write(char *pbt_buf, int dw_lenth)
{
    int ret;
    //printk("The msg_i2c_client->addr=0x%x\n",this_client->addr);
    ret = i2c_master_send(this_client, pbt_buf, dw_lenth);

    if(ret <= 0){
        printk("msg_i2c_read_interface error\n");
        return false;
    }

    return true;
}
static void i2c_write_update_msg2033(u8 *pbt_buf, int dw_lenth)
{
	int ret;
	this_client->addr = FW_UPDATE_ADDR_MSG20XX;
	ret = i2c_master_send(this_client, pbt_buf, dw_lenth);
	this_client->addr = TOUCH_ADDR_MSG20XX;
	if(ret <= 0)
	{
		printk("[TSP]:i2c_write_update_msg2033 error line = %d, ret = %d\n", __LINE__, ret);
	}
}
static void i2c_write_msg2033(u8 *pbt_buf, int dw_lenth)
{
	int ret;
	this_client->addr = FW_ADDR_MSG20XX;
	ret = i2c_master_send(this_client, pbt_buf, dw_lenth);
	this_client->addr = TOUCH_ADDR_MSG20XX;
	///printk("%s %s %d  ret=%d \n",__FILE__,__func__,__LINE__,ret);
//	this_client->timing = 40;
//	this_client->addr = FW_ADDR_MSG20XX;
//	ret = i2c_master_send(this_client, pbt_buf, dw_lenth);
//	this_client->addr = TOUCH_ADDR_MSG20XX;
	//this_client->timing = 240;
	if(ret <= 0)
	{
		printk("[TSP]:i2c_write_msg2033 error line = %d, ret = %d\n", __LINE__, ret);
	}
}
static void i2c_read_update_msg2033(u8 *pbt_buf, int dw_lenth)
{
	int ret;
	this_client->addr = FW_UPDATE_ADDR_MSG20XX;
	ret = i2c_master_recv(this_client, pbt_buf, dw_lenth);
	this_client->addr = TOUCH_ADDR_MSG20XX;
	if(ret <= 0)
	{
		printk("[TSP]:i2c_read_update_msg2033 error line = %d, ret = %d\n", __LINE__, ret);
	}
}
static void i2c_read_msg2033(u8 *pbt_buf, int dw_lenth)
{
	int ret;
	//this_client->timing = 40;
	//this_client->addr = FW_ADDR_MSG20XX;
	//ret = i2c_master_recv(this_client, pbt_buf, dw_lenth);
	//this_client->addr = TOUCH_ADDR_MSG20XX | 0x100;
	//this_client->timing = 240;
	this_client->addr = FW_ADDR_MSG20XX;
	ret = i2c_master_recv(this_client, pbt_buf, dw_lenth);	//0xC5_8bit
	this_client->addr = TOUCH_ADDR_MSG20XX;
	
	if(ret <= 0)
	{
		printk("[TSP]:i2c_read_interface error line = %d, ret = %d\n", __LINE__, ret);
	}
}

void Get_Chip_Version(void)
{
	unsigned char dbbus_tx_data[3];
	unsigned char dbbus_rx_data[2];
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0xCE;
	i2c_write_msg2033(&dbbus_tx_data[0], 3);
	i2c_read_msg2033(&dbbus_rx_data[0], 2);
	if(dbbus_rx_data[1] == 0)
	{
		printk("[TSP]:*** Catch2 ***\n");
	}
	else
	{
		printk("[TSP]:*** Catch1 ***\n");
	}
}


void dbbusDWIICEnterSerialDebugMode(void)
{
	U8 data[5];
	data[0] = 0x53;
	data[1] = 0x45;
	data[2] = 0x52;
	data[3] = 0x44;
	data[4] = 0x42;
	i2c_write_msg2033(data, 5);
}
void dbbusDWIICStopMCU(void)
{
	U8 data[1];
	data[0] = 0x37;
	i2c_write_msg2033(data, 1);
}
void dbbusDWIICIICUseBus(void)
{
	U8 data[1];
	data[0] = 0x35;
	i2c_write_msg2033(data, 1);
}
void dbbusDWIICIICReshape(void)
{
	U8 data[1];
	data[0] = 0x71;
	i2c_write_msg2033(data, 1);
}
void dbbusDWIICIICNotUseBus(void)
{
	U8 data[1];
	data[0] = 0x34;
	i2c_write_msg2033(data, 1);
}
void dbbusDWIICNotStopMCU(void)
{
	U8 data[1];
	data[0] = 0x36;
	i2c_write_msg2033(data, 1);
}
void dbbusDWIICExitSerialDebugMode(void)
{
	U8 data[1];
	data[0] = 0x45;
	i2c_write_msg2033(data, 1);
}
void drvISP_EntryIspMode(void)
{
	U8 bWriteData[5] =
	{
		0x4D, 0x53, 0x54, 0x41, 0x52
	};
	i2c_write_update_msg2033(bWriteData, 5);
	mdelay(10);           // delay about 1ms
}
/*First it needs send 0x11 to notify we want to get flash data back. */
U8 drvISP_Read(U8 n, U8 *pDataToRead)
{

	U8 Read_cmd = 0x11;
	unsigned char dbbus_rx_data[2] = {0};
	i2c_write_update_msg2033(&Read_cmd, 1);

	mdelay(10);// delay about 1000us*****

	if( n == 1 )
	{
		i2c_read_update_msg2033(&dbbus_rx_data[0], 2 );

		// Ideally, the obtained dbbus_rx_data[0~1] stands for the following meaning:
		//	dbbus_rx_data[0]  |  dbbus_rx_data[1]  | status
		// -------------------+--------------------+--------
		//		 0x00		  | 	  0x00		   |  0x00
		// -------------------+--------------------+--------
		//		 0x??		  | 	  0x00		   |  0x??
		// -------------------+--------------------+--------
		//		 0x00		  | 	  0x??		   |  0x??
		//				   
		// Therefore, we build this field patch to return the status to *pDataToRead.
		*pDataToRead = ( (dbbus_rx_data[0] >= dbbus_rx_data[1] )? \
			dbbus_rx_data[0]	: dbbus_rx_data[1] );

	}
	else
	{
		i2c_read_update_msg2033(pDataToRead, n );
	}

	return 0;


}
void drvISP_WriteEnable(void)
{
	U8 bWriteData[2] =
	{
		0x10, 0x06
	};
	U8 bWriteData1 = 0x12;
	i2c_write_update_msg2033(bWriteData, 2);
	i2c_write_update_msg2033(&bWriteData1, 1);
}
void drvISP_ExitIspMode(void)
{
	U8 bWriteData = 0x24;
	i2c_write_update_msg2033(&bWriteData, 1);
}
U8 drvISP_ReadStatus(void)
{
	U8 bReadData = 0;
	U8 bWriteData[2] =
	{
		0x10, 0x05
	};
	U8 bWriteData1 = 0x12;
	i2c_write_update_msg2033(bWriteData, 2);
	drvISP_Read(1, &bReadData);
	i2c_write_update_msg2033(&bWriteData1, 1);
	return bReadData;
}
void drvISP_BlockErase(U32 addr)
{
	U8 bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
	U8 bWriteData1 = 0x12;
	printk("[TSP]:The drvISP_ReadStatus0=%d\n", drvISP_ReadStatus());
	drvISP_WriteEnable();
	printk("[TSP]:The drvISP_ReadStatus1=%d\n", drvISP_ReadStatus());
	bWriteData[0] = 0x10;
	bWriteData[1] = 0x50;
	i2c_write_update_msg2033(bWriteData, 2);
	i2c_write_update_msg2033(&bWriteData1, 1);
	bWriteData[0] = 0x10;
	bWriteData[1] = 0x01;
	bWriteData[2] = 0x00;
	i2c_write_update_msg2033(bWriteData, 3);
	i2c_write_update_msg2033(&bWriteData1, 1);
	bWriteData[0] = 0x10;
	bWriteData[1] = 0x04;
	i2c_write_update_msg2033(bWriteData, 2);
	i2c_write_update_msg2033(&bWriteData1, 1);
	while((drvISP_ReadStatus() & 0x01) == 0x01)
	{
		;
	}
	printk("[TSP]:The drvISP_ReadStatus3=%d\n", drvISP_ReadStatus());
	drvISP_WriteEnable();
	printk("[TSP]:The drvISP_ReadStatus4=%d\n", drvISP_ReadStatus());
	bWriteData[0] = 0x10;
	bWriteData[1] = 0xC7;        //Block Erase
	i2c_write_update_msg2033(bWriteData, 2);
	i2c_write_update_msg2033(&bWriteData1, 1);
	while((drvISP_ReadStatus() & 0x01) == 0x01)
	{
		;
	}
}

void drvISP_SectorErase(U32 addr)
{
	U8 bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
	U8 bWriteData1 = 0x12;
	printk("[TSP]:The drvISP_ReadStatus0=%d\n", drvISP_ReadStatus());
	drvISP_WriteEnable();
	printk("[TSP]:The drvISP_ReadStatus1=%d\n", drvISP_ReadStatus());
	bWriteData[0] = 0x10;
	bWriteData[1] = 0x50;
	i2c_write_update_msg2033(&bWriteData, 2);
	i2c_write_update_msg2033(&bWriteData1, 1);
	bWriteData[0] = 0x10;
	bWriteData[1] = 0x01;
	bWriteData[2] = 0x00;
	i2c_write_update_msg2033(bWriteData, 3);
	i2c_write_update_msg2033(&bWriteData1, 1);
	bWriteData[0] = 0x10;
	bWriteData[1] = 0x04;
	i2c_write_update_msg2033(bWriteData, 2);
	i2c_write_update_msg2033(&bWriteData1, 1); 
	while((drvISP_ReadStatus() & 0x01) == 0x01)
	{
		;
	}
	drvISP_WriteEnable();
	bWriteData[0] = 0x10;
	bWriteData[1] = 0x20;        //Sector Erase
	bWriteData[2] = (( addr >> 16) & 0xFF);
	bWriteData[3] = (( addr >> 8 ) & 0xFF);
	bWriteData[4] = ( addr & 0xFF);      
	i2c_write_update_msg2033(&bWriteData, 5);
	i2c_write_update_msg2033(&bWriteData1, 1);
	while((drvISP_ReadStatus() & 0x01) == 0x01)
	{
		;
	}
}

void drvISP_Program(U16 k, U8 *pDataToWrite)
{
	U16 i = 0;
	U16 j = 0;
	U8 TX_data[37];
	U8 bWriteData1 = 0x12;
	U32 addr = k * 1024;

	for(j = 0; j < 32; j++)    //32*32 cycle
	{
		TX_data[0] = 0x10;
		TX_data[1] = 0x02;// Page Program CMD
		TX_data[2] = (addr + 32 * j) >> 16;
		TX_data[3] = (addr + 32 * j) >> 8;
		TX_data[4] = (addr + 32 * j);
		for(i = 0; i < 32; i++)
		{
			TX_data[5 + i] = pDataToWrite[j * 32 + i];
		}
		while((drvISP_ReadStatus() & 0x01) == 0x01)
		{
			//printk("########### 1  \n")
			;
		}
		drvISP_WriteEnable();
		i2c_write_update_msg2033(TX_data, 37);    //write 37 byte per cycle
		//printk("########### 2  \n");
		i2c_write_update_msg2033(&bWriteData1, 1);
		//printk("########### 3  \n");
	}

}

void drvISP_Verify(U16 k, U8 *pDataToVerify)
{
	U16 i = 0, j = 0;
	U8 bWriteData[5] =
	{
		0x10, 0x03, 0, 0, 0
	};
	U8 bWriteData1 = 0x12;
	U32 addr = k * 1024;
	U8 index = 0;
	U8 RX_data[32];

	for(j = 0; j < 32; j++)    //32*32 cycle
	{
		bWriteData[2] = (U8)((addr + j * 32) >> 16);
		bWriteData[3] = (U8)((addr + j * 32) >> 8);
		bWriteData[4] = (U8)(addr + j * 32);
		while((drvISP_ReadStatus() & 0x01) == 0x01)
		{
			;    //wait until not in write operation
		}
		i2c_write_update_msg2033(bWriteData, 5);     //write read flash addr
		drvISP_Read(32, RX_data);
		i2c_write_update_msg2033(&bWriteData1, 1);     //cmd end
		for(i = 0; i < 32; i++)    //log out if verify error
		{
			if((RX_data[i] != 0) && index < 10)//for debug purpose
			{
				printk("[TSP]:j=%d,RX_data[%d]=0x%x\n", j, i, RX_data[i]);
				index++;
			}
			if(RX_data[i] != pDataToVerify[32 * j + i])
			{
				printk("[TSP]:k=%d,j=%d,RX_data[%d]=0x%x\n===============Update Firmware Error================", k, j, i, RX_data[i]);
			}
		}
	}

}
static void _HalTscrHWReset ( void )//This function must implement by customer
{
	//*******To reset CTP controller, the GPIO API must depend on your platform
	gpio_direction_output(MX6SL_KEY_COL4_TOUCH_RST, MX6SL_GPIO_LOW);
	msleep(100);
	gpio_direction_output(MX6SL_KEY_COL4_TOUCH_RST, MX6SL_GPIO_HIGH);

}
static void drvISP_ChipErase()//new, check
{
    unsigned char bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    unsigned char bWriteData1 = 0x12;
    unsigned int timeOutCount = 0;
    drvISP_WriteEnable();

    //Enable write status register
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    i2c_write_update_msg2033 ( bWriteData, 2 );
    i2c_write_update_msg2033 ( &bWriteData1, 1 );

    //Write Status
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    i2c_write_update_msg2033 ( bWriteData, 3 );
    i2c_write_update_msg2033 ( &bWriteData1, 1 );

    //Write disable
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    i2c_write_update_msg2033 ( bWriteData, 2 );
    i2c_write_update_msg2033 ( &bWriteData1, 1 );
    msleep(1);        // delay about 100us*****
    timeOutCount = 0;
    while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
    {
        timeOutCount++;
        if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
    }
    drvISP_WriteEnable();

    bWriteData[0] = 0x10;
    bWriteData[1] = 0xC7;

    i2c_write_update_msg2033 ( bWriteData, 2 );
    i2c_write_update_msg2033 ( &bWriteData1, 1 );
    msleep(1);        // delay about 100us*****
    timeOutCount = 0;
    while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
    {
        timeOutCount++;
        if ( timeOutCount >= 500000 ) break; /* around 5 sec timeout */
    }
}
static void drvDB_WriteReg ( unsigned char bank, unsigned char addr, unsigned short data )//New. Check
{
    unsigned char tx_data[5] = {0x10, bank, addr, data & 0xFF, data >> 8};
    i2c_write_msg2033 ( tx_data, 5 );
}
static void drvDB_WriteReg8Bit ( unsigned char bank, unsigned char addr, unsigned char data )//New. Check
{
    unsigned char tx_data[4] = {0x10, bank, addr, data};
    i2c_write_msg2033 ( tx_data, 4 );
}

static unsigned short drvDB_ReadReg ( unsigned char bank, unsigned char addr )//New. Check
{
    unsigned char tx_data[3] = {0x10, bank, addr};
    unsigned char rx_data[2] = {0};

    i2c_write_msg2033 ( tx_data, 3 );
    i2c_read_msg2033 ( &rx_data[0], 2 );
    return ( rx_data[1] << 8 | rx_data[0] );
}
static unsigned int Reflect ( unsigned int ref, char ch )////New. Check
{
    unsigned int value = 0;
    unsigned int i = 0;

    for ( i = 1; i < ( ch + 1 ); i++ )
    {
        if ( ref & 1 )
        {
            value |= 1 << ( ch - i );
        }
        ref >>= 1;
    }
    return value;
}

static void Init_CRC32_Table ( unsigned int *crc32_table )//New. Check
{
    unsigned int magicnumber = 0x04c11db7;
    unsigned int i = 0, j;

    for ( i = 0; i <= 0xFF; i++ )
    {
        crc32_table[i] = Reflect ( i, 8 ) << 24;
        for ( j = 0; j < 8; j++ )
        {
            crc32_table[i] = ( crc32_table[i] << 1 ) ^ ( crc32_table[i] & ( 0x80000000L ) ? magicnumber : 0 );
        }
        crc32_table[i] = Reflect ( crc32_table[i], 32 );
    }
}

unsigned int Get_CRC ( unsigned int text, unsigned int prevCRC, unsigned int *crc32_table )//New. Check
{
    unsigned int ulCRC = prevCRC;
    {
        ulCRC = ( ulCRC >> 8 ) ^ crc32_table[ ( ulCRC & 0xFF ) ^ text];
    }
    return ulCRC ;
}


static int drvTP_erase_emem_c33 ( EMEM_TYPE_t emem_type )//New, Check
{
    // stop mcu
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0001 );

    //disable watch dog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    // set PROGRAM password
    drvDB_WriteReg8Bit ( 0x16, 0x1A, 0xBA );
    drvDB_WriteReg8Bit ( 0x16, 0x1B, 0xAB );

    //proto.MstarWriteReg(F1.loopDevice, 0x1618, 0x80);
    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    if ( emem_type == EMEM_ALL )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x08, 0x10 ); //mark
    }

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x40 );
    mdelay ( 10 );

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    // erase trigger
    if ( emem_type == EMEM_MAIN )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x04 ); //erase main
    }
    else
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x08 ); //erase all block
    }

    return ( 1 );
}
static int drvTP_read_info_dwiic_c33 ( void )//New, Check
{
    unsigned char dwiic_tx_data[5];

    //drvDB_EnterDBBUS();
     _HalTscrHWReset();
     mdelay ( 300 );
	dbbusDWIICEnterSerialDebugMode();
	dbbusDWIICStopMCU();
	dbbusDWIICIICUseBus();
	dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Stop Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    drvDB_WriteReg ( 0x3C, 0xE4, 0xA4AB );

    // TP SW reset
    drvDB_WriteReg ( 0x1E, 0x02, 0x829F );

    mdelay ( 50 );

    dwiic_tx_data[0] = 0x72;
    dwiic_tx_data[1] = 0x80;
    dwiic_tx_data[2] = 0x00;
    dwiic_tx_data[3] = 0x04;
    dwiic_tx_data[4] = 0x00;
    msg2133_i2c_write ( dwiic_tx_data, 5 );

    mdelay ( 50 );

    // recive info data
    msg2133_i2c_read ( &g_dwiic_info_data[0], 1024 );

    return ( 1 );
}
static int drvTP_info_updata_C33 ( unsigned short start_index, unsigned char *data, unsigned short size )//New, check
{
    // size != 0, start_index+size !> 1024
    unsigned short i;
    
    for ( i = 0;i < size; i++ )
    {
        g_dwiic_info_data[start_index] = * ( data + i );
        start_index++;
    }
    
    return ( 1 );
}

static ssize_t firmware_update_c33 ( struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t size, EMEM_TYPE_t emem_type )//New, check
{
    unsigned char dbbus_tx_data[4];
    unsigned char  dbbus_rx_data[2] = {0};
    unsigned char  life_counter[2];
    unsigned int i, j;
    unsigned int crc_main, crc_main_tp;
    unsigned int crc_info, crc_info_tp;
    unsigned int crc_tab[256];
   int update_pass = 1;
    unsigned short reg_data = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;

    //drvTP_read_info_dwiic_c33();

    if (0)//g_dwiic_info_data[0] == 'M' && g_dwiic_info_data[1] == 'S' 
		//&& g_dwiic_info_data[2] == 'T' && g_dwiic_info_data[3] == 'A' 
		//&& g_dwiic_info_data[4] == 'R' && g_dwiic_info_data[5] == 'T' 
		//&& g_dwiic_info_data[6] == 'P' && g_dwiic_info_data[7] == 'C' )
    {
        // updata FW Version
        drvTP_info_updata_C33 ( 8, temp[32][8], 4 );

        // updata life counter
        life_counter[0] = ( ( g_dwiic_info_data[12] << 8 + g_dwiic_info_data[13] + 1 ) >> 8 ) & 0xFF;
        life_counter[1] = ( g_dwiic_info_data[12] << 8 + g_dwiic_info_data[13] + 1 ) & 0xFF;
        drvTP_info_updata_C33 ( 10, life_counter, 2 );

        drvDB_WriteReg ( 0x3C, 0xE4, 0x78C5 );

        // TP SW reset
        drvDB_WriteReg ( 0x1E, 0x02, 0x829F );

        mdelay ( 50 );

        //polling 0x3CE4 is 0x2F43
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x2F43 );

        // transmit lk info data
        msg2133_i2c_write ( g_dwiic_info_data, 1024 );

        //polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );

    }


    //erase main
    drvTP_erase_emem_c33 ( EMEM_MAIN );
    mdelay ( 1000 );

    //ResetSlave();
    _HalTscrHWReset();
    msleep(1);
    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    msleep(1);
    dbbusDWIICStopMCU();
    msleep(1);
    dbbusDWIICIICUseBus();
    msleep(1);
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    /////////////////////////
    // Program
    /////////////////////////

    //polling 0x3CE4 is 0x1C70
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x1C70 );
    }

    switch ( emem_type )
    {
        case EMEM_ALL:
            drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks
            break;
        case EMEM_MAIN:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for main block
            break;
        case EMEM_INFO:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for info block


            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x01 );

            drvDB_WriteReg8Bit ( 0x3C, 0xE4, 0xC5 ); //
            drvDB_WriteReg8Bit ( 0x3C, 0xE5, 0x78 ); //

            drvDB_WriteReg8Bit ( 0x1E, 0x04, 0x9F );
            drvDB_WriteReg8Bit ( 0x1E, 0x05, 0x82 );

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x00 );
            mdelay ( 100 );
            break;
    }

    // polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x2F43 );


    // calculate CRC 32
    Init_CRC32_Table ( &crc_tab[0] );


    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
         if( emem_type == EMEM_INFO ) i = 32;

        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
        }
        else  //emem_info
        {
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( g_dwiic_info_data[j], crc_info, &crc_tab[0] );
            }
            if ( emem_type == EMEM_MAIN ) break;
        }

        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
        msg2133_i2c_write ( temp[i], 1024 );

        // polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );

        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // write file done and check crc
        drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );
    }

    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // polling 0x3CE4 is 0x9432
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }while ( reg_data != 0x9432 );
    }

    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // CRC Main from TP
        crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
        crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );

        // CRC Info from TP
        crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
        crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );
    }

    printk ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
               crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();

    update_pass = 1;
	
  if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        if ( crc_main_tp != crc_main )
            update_pass = 0;

        if ( crc_info_tp != crc_info )
            update_pass = 0;
    }

    if ( !update_pass )
    {
        //printk ( "update FAILED\n" );
        FwDataCnt = 0;
        //return ( 0 );
    }

    printk ( "update OK\n" );
    FwDataCnt = 0;

	_HalTscrHWReset();
	bInupgrade = false;
     ///enable_irq(this_client ->irq);
    
    return size;
}
static ssize_t firmware_update_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_update_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size )//New, check
{
    unsigned char i;
    unsigned char dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};

	////printk("%s %s %d \n",__FILE__,__func__,__LINE__);
     bInupgrade = true;
    _HalTscrHWReset();
    msleep(1);
     ////disable_irq(this_client ->irq);

    // Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    msleep(1);
    dbbusDWIICStopMCU();
    msleep(1);
    dbbusDWIICIICUseBus();
    msleep(1);
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Disable the Watchdog
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x60;
	dbbus_tx_data[3] = 0x55;
	i2c_write_msg2033(dbbus_tx_data, 4);
	printk("[TSP]:update\n");
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x61;
	dbbus_tx_data[3] = 0xAA;
	i2c_write_msg2033(dbbus_tx_data, 4);

    // Stop MCU
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x0F;
	dbbus_tx_data[2] = 0xE6;
	dbbus_tx_data[3] = 0x01;
	i2c_write_msg2033(dbbus_tx_data, 4);


    /////////////////////////
    // Difference between C2 and C3
    /////////////////////////

    //check id
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCC;
	i2c_write_msg2033(dbbus_tx_data, 3);
	i2c_read_msg2033(&dbbus_rx_data[0], 2);
    ///printk ( "%s,%d,dbbus_rx id[0]=0x%x\n",__func__,__LINE__, dbbus_rx_data[0] );

    return firmware_update_c33 ( dev, attr, buf, size, EMEM_MAIN );


}

static DEVICE_ATTR(update, 0777, firmware_update_show, firmware_update_store);
static ssize_t firmware_clear_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	U16 k = 0, i = 0, j = 0;
	U8 bWriteData[5] =
	{
		0x10, 0x03, 0, 0, 0
	};
	U8 RX_data[256];
	U8 bWriteData1 = 0x12;
	U32 addr = 0;
	printk("\n");
	for(k = 0; k < 94; i++)    // total  94 KB : 1 byte per R/W
	{
		addr = k * 1024;
		for(j = 0; j < 8; j++)    //128*8 cycle
		{
			bWriteData[2] = (U8)((addr + j * 128) >> 16);
			bWriteData[3] = (U8)((addr + j * 128) >> 8);
			bWriteData[4] = (U8)(addr + j * 128);
			while((drvISP_ReadStatus() & 0x01) == 0x01)
			{
				;    //wait until not in write operation
			}
			i2c_write_update_msg2033(bWriteData, 5);     //write read flash addr
			drvISP_Read(128, RX_data);
			i2c_write_update_msg2033(&bWriteData1, 1);    //cmd end
			for(i = 0; i < 128; i++)    //log out if verify error
			{
				if(RX_data[i] != 0xFF)
				{
					printk("[TSP]:k=%d,j=%d,i=%d===============erase not clean================", k, j, i);
				}
			}
		}
	}
	printk("[TSP]:read finish\n");
	return sprintf(buf, "%s\n", fw_version);
}
static ssize_t firmware_clear_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	U8 dbbus_tx_data[4];
	unsigned char dbbus_rx_data[2] = {0};
	printk("\n");
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x08;
	dbbus_tx_data[2] = 0x0c;
	dbbus_tx_data[3] = 0x08;
	i2c_write_msg2033(dbbus_tx_data, 4);
	{
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x11;
		dbbus_tx_data[2] = 0xE2;
		dbbus_tx_data[3] = 0x00;
		i2c_write_msg2033(dbbus_tx_data, 4);
	}
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x60;
	dbbus_tx_data[3] = 0x55;
	i2c_write_msg2033(dbbus_tx_data, 4);
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x61;
	dbbus_tx_data[3] = 0xAA;
	i2c_write_msg2033(dbbus_tx_data, 4);
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x0F;
	dbbus_tx_data[2] = 0xE6;
	dbbus_tx_data[3] = 0x01;
	i2c_write_msg2033(dbbus_tx_data, 4);
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x02;
	i2c_write_msg2033(dbbus_tx_data, 3);
	i2c_read_msg2033(&dbbus_rx_data[0], 2);
	printk("[TSP]:dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
	dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
	i2c_write_msg2033(dbbus_tx_data, 4);
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x25;
	i2c_write_msg2033(dbbus_tx_data, 3);
	dbbus_rx_data[0] = 0;
	dbbus_rx_data[1] = 0;
	i2c_read_msg2033(&dbbus_rx_data[0], 2);
	printk("[TSP]:dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
	dbbus_tx_data[3] = dbbus_rx_data[0] & 0xFC;  //Clear Bit 1,0
	i2c_write_msg2033(dbbus_tx_data, 4);
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x0E;
	dbbus_tx_data[3] = 0x02;
	i2c_write_msg2033(dbbus_tx_data, 4);
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x10;
	dbbus_tx_data[3] = 0x08;
	i2c_write_msg2033(dbbus_tx_data, 4);
	dbbusDWIICIICNotUseBus();
	dbbusDWIICNotStopMCU();
	dbbusDWIICExitSerialDebugMode();
	drvISP_EntryIspMode();
	// pr_tp("chip erase+\n");
	//drvISP_BlockErase(0x00000);
	// pr_tp("chip erase-\n");
	drvISP_ExitIspMode();
	return size;
}

static DEVICE_ATTR(clear, 0777, firmware_clear_show, firmware_clear_store);
static ssize_t firmware_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned char dbbus_tx_data[3];
	unsigned char dbbus_rx_data[13] ;
	int time = 0;

	//printk("tyd-tp: firmware_version_show\n");
	while(time < 10){
		dbbus_tx_data[0] = 0x53;
		dbbus_tx_data[1] = 0x00;
		dbbus_tx_data[2] = 0x74;
		msg2133_i2c_write(&dbbus_tx_data[0], 3);
		mdelay(20);
		msg2133_i2c_read(&dbbus_rx_data[0], 13);
		fw_v.major = (dbbus_rx_data[1] << 8) + dbbus_rx_data[0];
		fw_v.minor = (dbbus_rx_data[3] << 8) + dbbus_rx_data[2];
		fw_v.VenderID = dbbus_rx_data[4];
				
		time++;
		if((fw_v.major & 0xff00) == 0)
			//printk("[TSP]:#fw_v.major =0x%x,fw_v.minor =0x%x,fw_v.VenderID =0x%x\n",fw_v.major,fw_v.minor,fw_v.VenderID );
			break;
		msleep(50);
	}

	//MSG2133_DBG("*** firmware_version_show fw_version = %03d.%03d.%02d***\n", fw_v.major, fw_v.minor,fw_v.VenderID);
	return sprintf(buf, "%03d.%03d VenderID %02d\n", fw_v.major, fw_v.minor,fw_v.VenderID);
}
static ssize_t firmware_version_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned char dbbus_tx_data[3];
	unsigned char dbbus_rx_data[13] ;
	int time = 0;
	
	//printk("%s\n", __func__);
	while(time < 10){
		dbbus_tx_data[0] = 0x53;
		dbbus_tx_data[1] = 0x00;
		dbbus_tx_data[2] = 0x74;
		msg2133_i2c_write(&dbbus_tx_data[0], 3);
		mdelay(20);
		msg2133_i2c_read(&dbbus_rx_data[0], 13);
		fw_v.major = (dbbus_rx_data[1] << 8) + dbbus_rx_data[0];
		fw_v.minor = (dbbus_rx_data[3] << 8) + dbbus_rx_data[2];
		fw_v.VenderID = dbbus_rx_data[4];
		printk("[TSP]:#fw_v.major =%x,fw_v.minor =%x,fw_v.VenderID =%x\n",fw_v.major,fw_v.minor,fw_v.VenderID );
		time++;
		if((fw_v.major & 0xff00) == 0)
			printk("[TSP]:#fw_v.major =%x,fw_v.minor =%x,fw_v.VenderID =%x\n",fw_v.major,fw_v.minor,fw_v.VenderID );
			break;
		msleep(50);
	}
	return size;
}
static DEVICE_ATTR(version, 0644, firmware_version_show, NULL);
static ssize_t firmware_data_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int i,j;
	printk("[TSP]:***firmware_data_show = %d ***\n", FwDataCnt);
	   for(i=0; i<32; i++)
	   {
	   	for(j=0; j<1024; j++)
	   	{
	   		printk("%x, ",temp[i][j]);
	   		if ((j-1)%16==0)
				printk(" \n");
	   	}
	   }	

	return FwDataCnt;
}
static ssize_t firmware_data_store(struct device *dev,
			struct device_attribute *attr, const  unsigned *buf, size_t size)
{
	int i,j;
	//printk("%s \n",buf);
	printk("[TSP]:***firmware_data_store = %d ***\n", FwDataCnt);
		memcpy(temp[FwDataCnt], buf, 1024);
	FwDataCnt++;

	printk("[TSP]:######### exit firmware_data_store ##########\n");

	return size;
}
static DEVICE_ATTR(data, 0777, firmware_data_show, firmware_data_store);

//static struct attribute *cnt_attributes[] = {
//	&dev_attr_update.attr,
//	&dev_attr_version.attr,
//	&dev_attr_clear.attr,
//	&dev_attr_data.attr,
//	NULL
//};

//static const struct attribute_group cnt_group = {
//	.attrs = cnt_attributes,
//};
#endif
#ifdef TP_FIRMWARE_VERSION
static struct fw_version fw_ver;
static bool firmware_version_i2c_write(char *pbt_buf, int dw_lenth)
{
    int ret;
    //printk("The msg_i2c_client->addr=0x%x\n",this_client->addr);
    ret = i2c_master_send(this_client, pbt_buf, dw_lenth);

    if(ret <= 0){
        printk("msg_i2c_read_interface error\n");
        return false;
    }

    return true;
}
static bool firmware_version_i2c_read(char *pbt_buf, int dw_lenth)
{
    int ret;
    //printk("The msg_i2c_client->addr=0x%x\n",this_client->addr);
    ret = i2c_master_recv(this_client, pbt_buf, dw_lenth);

    if(ret <= 0){
        printk("msg2133_i2c_read error\n");
        return false;
    }

    return true;
}

static ssize_t firmware_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned char dbbus_tx_data[3];
	unsigned char dbbus_rx_data[13] ;
	int time = 0;

	//printk("tyd-tp: firmware_version_show\n");
	while(time < 10){
		dbbus_tx_data[0] = 0x53;
		dbbus_tx_data[1] = 0x00;
		dbbus_tx_data[2] = 0x74;
		firmware_version_i2c_write(&dbbus_tx_data[0], 3);
		mdelay(20);
		firmware_version_i2c_read(&dbbus_rx_data[0], 13);
		fw_ver.major = (dbbus_rx_data[1] << 8) + dbbus_rx_data[0];
		fw_ver.minor = (dbbus_rx_data[3] << 8) + dbbus_rx_data[2];
		fw_ver.VenderID = dbbus_rx_data[4];
				
		time++;
		if((fw_ver.major & 0xff00) == 0)
			//printk("[TSP]:#fw_v.major =0x%x,fw_v.minor =0x%x,fw_v.VenderID =0x%x\n",fw_v.major,fw_v.minor,fw_v.VenderID );
			break;
		msleep(50);
	}

	//MSG2133_DBG("*** firmware_version_show fw_version = %03d.%03d.%02d***\n", fw_v.major, fw_v.minor,fw_v.VenderID);
	return sprintf(buf, "Major:0x%x,Minor:0x%x,VenderID:0x%x\n", fw_ver.major, fw_ver.minor,fw_ver.VenderID);
}

static DEVICE_ATTR(version, 0644, firmware_version_show, NULL);
#endif
#ifdef CNT_PROXIMITY
static struct input_dev *prox_input;
static int prox_enable;

static int prox_i2c_write(char *pbt_buf, int dw_lenth)
{
	int ret;
	//printk("[TSP]:The msg_i2c_client->addr=0x%x\n",this_client->addr);
	ret = i2c_master_send(this_client, pbt_buf, dw_lenth);

	if(ret <= 0)
		printk("[TSP]:msg_i2c_read_interface error\n");

	return ret;
}
static void prox_enable_prox(int enable)
{
	unsigned char  dbbus_tx_data[4];
	unsigned char dbbus_rx_data[4];
	int ret = -1;

	//printk("[TSP]:######%s, enable = %d\n", __func__, enable);	

	if (enable)
	{
		dbbus_tx_data[0] = 0xFF;
		dbbus_tx_data[1] = 0x11;
		dbbus_tx_data[2] = 0xFF;
		dbbus_tx_data[3] = 0x01;
		ret = prox_i2c_write(&dbbus_tx_data[0], 4);
	}
	else if (enable == 0)
	{
		dbbus_tx_data[0] = 0xFF;
		dbbus_tx_data[1] = 0x11;
		dbbus_tx_data[2] = 0xFF;
		dbbus_tx_data[3] = 0x00;
		ret = prox_i2c_write(&dbbus_tx_data[0], 4);
	}

	if(ret > 0)
		prox_enable = enable;
}

static ssize_t prox_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	//printk("[TSP]:%s\n", __func__);
	return sprintf(buf, "%d\n", prox_enable);
}

static ssize_t prox_enable_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	unsigned long enable;

	//printk("[TSP]:%s, prox_enable = %d\n", __func__, prox_enable);

	enable = simple_strtoul(buf, NULL, 10);    
	enable = (enable > 0) ? 1 : 0;
	if(enable != prox_enable){  
		prox_enable_prox(enable);
	}
	return count;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUGO,
				   prox_enable_show, prox_enable_store);

static struct attribute *prox_attributes[] = {
	&dev_attr_enable.attr,
	NULL
};

static const struct attribute_group prox_attr_group = {
	.attrs = prox_attributes,
};


void msg2133_init_prox()
{
	int ret;
	prox_enable = 0;

	prox_input = input_allocate_device();
	if (!prox_input) {
		printk("[TSP]:%s: failed to allocate prox input device\n", __func__);
		return -ENOMEM;
	}

	prox_input->name  = "prox";
	prox_input->id.bustype = BUS_I2C;

	set_bit(EV_ABS, prox_input->evbit);    
	input_set_abs_params(prox_input, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(prox_input);
	if (ret < 0) {
		printk("[TSP]:%s, call input_register_device() error, ret = %d\n",__func__,  ret);
		input_free_device(prox_input);
		return ret;
	}

	ret = sysfs_create_group(&prox_input->dev.kobj, &prox_attr_group);
	if(ret < 0){
		printk("[TSP]:%s, call sysfs_create_group() error, ret = %d\n",__func__,  ret);
		input_unregister_device(prox_input);
		input_free_device(prox_input);
		return ret;
	}

	return 0;
}

#endif //endif CNT_PROXIMITY

#ifdef CONFIG_HAS_EARLYSUSPEND
/***********************************************************************
[function]: 
callback:	early suspend function interface;
[parameters]:
handler:                early suspend callback pointer
[return]:
NULL
************************************************************************/
static void cnt_ts_earlysuspend(struct early_suspend *handler)
{

	/*We use GPIO to control TP reset pin, High Active, Low Power Down*/
	/******************************************************************/
	//The GPIO API depends on your platform
	/******************************************************************/
	printk("\n [TSP]:device will suspend! \n");

}


/***********************************************************************
[function]: 
callback:	power resume function interface;
[parameters]:
handler:                early suspend callback pointer
[return]:
NULL
************************************************************************/
static void cnt_ts_earlyresume(struct early_suspend *handler)
{

	/*We use GPIO to control TP reset pin, High Active, Low Power Down*/
	/******************************************************************/
	//The GPIO API depends on your platform
	/******************************************************************/
	printk("\n [TSP]:device will resume from sleep! \n");

}
#endif 
#if CONFIG_PM_SLEEP
static int cnt_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct CNT_TS_DATA_T *cnt_ts = dev_get_drvdata(&client->dev);

	/*We use GPIO to control TP reset pin, High Active, Low Power Down*/
	/******************************************************************/
	//The GPIO API depends on your platform
	/******************************************************************/
	//printk("\n [TSP]:device will suspend! \n");
#if defined(CONFIG_PB650_DEEP_STANDY)
	printk("%s %s %d iDeepStandy=%d \n",__FILE__,__func__,__LINE__,iDeepStandy); 
	if(iDeepStandy ==0) {
		if(client ->irq)
			disable_irq(client ->irq);

		////close the power
		if(cnt_ts->platform_data->power)
			cnt_ts->platform_data->power(0);
		return 0;
	}
#endif

	if (client->irq && device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);

	if (cnt_ts) {
		unsigned long flags;
		spin_lock_irqsave(&cnt_ts->flag_lock, flags);
		cnt_ts->sleep_flags |= FLAG_IN_SLEEP;
		spin_unlock_irqrestore(&cnt_ts->flag_lock, flags);
	}

	cancel_work_sync(&cnt_ts->pen_event_work);

	return 0;
}
static int cnt_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct CNT_TS_DATA_T *cnt_ts = dev_get_drvdata(&client->dev);
	/*We use GPIO to control TP reset pin, High Active, Low Power Down*/
	/******************************************************************/
	//The GPIO API depends on your platform
	/******************************************************************/
	//printk("\n [TSP]:device will resume from sleep! \n");
#if defined(CONFIG_PB650_DEEP_STANDY)
	if(iDeepStandy ==0) {
		////first,open  the power
		if(cnt_ts->platform_data->power)
			cnt_ts->platform_data->power(1);  

		///then reset
		if(cnt_ts->platform_data->reset)
			cnt_ts->platform_data->reset();  

		if(client ->irq)
			enable_irq(client ->irq);	  	
		return 0;
	}
#endif
	if (client->irq && device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);

	if (cnt_ts) {
		unsigned long flags;
		spin_lock_irqsave(&cnt_ts->flag_lock, flags);
		if (cnt_ts->sleep_flags & FLAG_EVENT_PENDING) {
			// fire
			if (!work_pending(&cnt_ts->pen_event_work)) {
					queue_work(cnt_ts->ts_workqueue, &cnt_ts->pen_event_work);
			}
		}
		cnt_ts->sleep_flags &= ~(FLAG_EVENT_PENDING | FLAG_IN_SLEEP);
		spin_unlock_irqrestore(&cnt_ts->flag_lock, flags);
	}

    return 0;
}
static const struct dev_pm_ops cnt_ts_pm_ops = {
	.suspend = cnt_ts_suspend,
	.resume = cnt_ts_resume,
};
#endif
/***********************************************************************
[function]: 
callback:								calculate data checksum
[parameters]:
msg:             				data buffer which is used to store touch data;
s32Length:             	the length of the checksum ;
[return]:
checksum value;
************************************************************************/
static u8 Calculate_8BitsChecksum( u8 *msg, int s32Length )
{
	int s32Checksum = 0;
	int i;

	for ( i = 0 ; i < s32Length; i++ )
	{
		s32Checksum += msg[i];
	}

	return (u8)( ( -s32Checksum ) & 0xFF );
}


/***********************************************************************
[function]: 
callback:								read touch  data ftom controller via i2c interface;
[parameters]:
rxdata[in]:             data buffer which is used to store touch data;
length[in]:             the length of the data buffer;
[return]:
CNT_TRUE:              	success;
CNT_FALSE:             	fail;
************************************************************************/
static int cnt_i2c_rxdata(u8 *rxdata, int length)
{
	int ret;
	struct i2c_msg msg;

	msg.addr = this_client->addr;
	msg.flags = I2C_M_RD;
	msg.len = length;
	msg.buf = rxdata;
	ret = i2c_transfer(this_client->adapter, &msg, 1);
	if (ret < 0)
		pr_err("msg %s i2c write error: %d\n", __func__, ret);

	return ret;
}

static unsigned long transform_to_screen_x(unsigned long x)
{
#ifdef REVERSE_X
	return ((CNT_RESOLUTION_X-x)*SCREEN_MAX_X)/CNT_RESOLUTION_X;

#else
	return (x*SCREEN_MAX_X)/CNT_RESOLUTION_X;
#endif
}

static unsigned long transform_to_screen_y(unsigned long y)
{
#ifdef REVERSE_Y
	return ((CNT_RESOLUTION_Y-y)*SCREEN_MAX_Y)/CNT_RESOLUTION_Y;
#else
	return (y*SCREEN_MAX_Y)/CNT_RESOLUTION_Y;
#endif
}

/***********************************************************************
[function]: 
callback:            		gather the finger information and calculate the X,Y
coordinate then report them to the input system;
[parameters]:
null;
[return]:
null;
************************************************************************/
int cnt_read_data(void)
{
	struct CNT_TS_DATA_T *data = i2c_get_clientdata(this_client);
	u8 buf[8] = {0};
	int key_id = 0x0, Touch_State = 0, have_VK = 0, temp_x0, temp_y0, dst_x, dst_y;
#ifdef CNT_PROXIMITY
	int change_state = PROX_NORMAL;
#endif
	int i,ret = -1;

	ret = cnt_i2c_rxdata(buf, 8);

	if (ret > 0)  
	{

#ifdef CNT_PROXIMITY
		if(prox_enable == 1)//if android phone is in call
		{		
			if(buf[0] == 0x5A)
			{
				change_state = PROX_CLOSE;
			}
			else if (buf[0] == 0x5E)
			{
				change_state = PROX_OFF;
			}
			if(change_state == PROX_CLOSE)//chage backlight state
			{ 
				//printk("[TSP]:%s, prox close\n", __func__);
				input_report_abs(prox_input, ABS_DISTANCE, 0);
			}
			else if(change_state == PROX_OFF)
			{
				//printk("[TSP]:%s, prox off\n", __func__);
				input_report_abs(prox_input, ABS_DISTANCE, 1);
			}
		}
#endif

		//Judge Touch State
		if((buf[0] != 0x52) || (Calculate_8BitsChecksum(buf, 7) != buf[7])) //Check data packet ID & Check the data if valid
		{
			return 0;
		}
		else if((buf[1]== 0xFF) && (buf[2]== 0xFF) 
			&& (buf[3]== 0xFF) && (buf[4]== 0xFF) 
			&& (buf[6]== 0xFF))//Scan finger number on panel
		{
			if((buf[5]== 0x0) || (buf[5]== 0xFF))
			{
				Touch_State = Touch_State_No_Touch;
			}
#ifdef SUPPORT_VIRTUAL_KEY				
			else//VK 
			{
				Touch_State =  Touch_State_VK;
				key_id = buf[5] & 0x1F;
			}
#endif
		}
		else
		{
			if((buf[4]== 0x0) && (buf[5]== 0x0) && (buf[6]== 0x0))
			{
				Touch_State = Touch_State_One_Finger;
			}
			else
			{
				Touch_State = Touch_State_Two_Finger;	
			}
		} 

		temp_x0 = ((buf[1] & 0xF0) << 4) | buf[2];
		temp_y0 = ((buf[1] & 0x0F) << 8) | buf[3];
		dst_x = ((buf[4] & 0xF0) << 4) | buf[5];
		dst_y = ((buf[4] & 0x0F) <<8 ) | buf[6];
	}
	else
	{
	   if(!bInupgrade){
		pr_err("%s try to recover ts, do reset\n", __func__);
		if(data->platform_data->reset)
		    data->platform_data->reset();
		else
			pr_err("%s No touchscreen reset function\n", __func__);

		return 0;
	   }
	}

	if(Touch_State == Touch_State_One_Finger)
	{
		/*Mapping CNT touch coordinate to Android coordinate*/
		//_st_finger_infos[0].i2_x = ((CNT_RESOLUTION_X-temp_x0)*SCREEN_MAX_Y)/CNT_RESOLUTION_X;//   (temp_x0*SCREEN_MAX_X) / CNT_RESOLUTION_X);
		//_st_finger_infos[0].i2_y = (temp_y0*SCREEN_MAX_Y) / CNT_RESOLUTION_Y ;
		_st_finger_infos[0].i2_x = (temp_x0*SCREEN_MAX_Y) / CNT_RESOLUTION_X ;  		
		_st_finger_infos[0].i2_y = (temp_y0*SCREEN_MAX_X) / CNT_RESOLUTION_Y ;
		_st_finger_infos[0].i2_x=SCREEN_MAX_Y-_st_finger_infos[0].i2_x;		

	///printk("%s one flinger.. temp_x0=%d,,temp_y0=%d,______- x=%d ,y=%d..\n",__func__,temp_x0,temp_y0,_st_finger_infos[0].i2_x, _st_finger_infos[0].i2_y);

		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 1);        	
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, _st_finger_infos[0].i2_y);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, _st_finger_infos[0].i2_x);
		input_report_key(data->input_dev, BTN_TOUCH, 1);	
		input_mt_sync(data->input_dev);	
		input_sync(data->input_dev);


	}        
	else if(Touch_State == Touch_State_Two_Finger)
	{
		/*Mapping CNT touch coordinate to Android coordinate*/
		if (dst_x > 2048)
		{    
			dst_x -= 4096;//transform the unsigh value to sign value
		}
		if (dst_y > 2048)
		{
			dst_y -= 4096;//transform the unsigh value to sign value
		}

		_st_finger_infos[0].i2_x = (temp_x0*SCREEN_MAX_Y) / CNT_RESOLUTION_X ;
		_st_finger_infos[0].i2_y = (temp_y0*SCREEN_MAX_X) / CNT_RESOLUTION_Y ;
		_st_finger_infos[0].i2_x=SCREEN_MAX_Y-_st_finger_infos[0].i2_x;		

		_st_finger_infos[1].i2_x = ((temp_x0 + dst_x)*SCREEN_MAX_Y) / CNT_RESOLUTION_X ;
		_st_finger_infos[1].i2_y = ((temp_y0 + dst_y)*SCREEN_MAX_X) / CNT_RESOLUTION_Y ;
		_st_finger_infos[1].i2_x=SCREEN_MAX_Y-_st_finger_infos[1].i2_x;		

	//printk("%s two flinger  x1=%d ,y1=%d..x2=%d,y2=%d\n",__func__,
	//	_st_finger_infos[0].i2_x, _st_finger_infos[0].i2_y, _st_finger_infos[1].i2_x, _st_finger_infos[1].i2_y);

		/*report first point*/
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 1);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, _st_finger_infos[0].i2_y);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, _st_finger_infos[0].i2_x);
		input_mt_sync(data->input_dev);
		/*report second point*/
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 1);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, _st_finger_infos[1].i2_y);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, _st_finger_infos[1].i2_x);
		input_report_key(data->input_dev, BTN_TOUCH, 1);	
		input_mt_sync(data->input_dev);
		input_sync(data->input_dev);

	}
	else/*Finger up*/
	{
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_key(data->input_dev, BTN_TOUCH, 0);
		input_mt_sync(data->input_dev);
		input_sync(data->input_dev);

	}
}


static void cnt_work_func(struct work_struct *work)
{
	//  printk("\n-shy --work func ---\n");
	cnt_read_data();    
}




static irqreturn_t cnt_ts_irq(int irq, void *dev_id)
{
	struct CNT_TS_DATA_T *cnt_ts = dev_id;

	//printk("%s %s %d  irq=%d \n",__FILE__,__func__,__LINE__,irq);
	pm_wakeup_event(&this_client->dev, 0);

	if (cnt_ts->sleep_flags & FLAG_IN_SLEEP)
		cnt_ts->sleep_flags |= FLAG_EVENT_PENDING;
	else if (!work_pending(&cnt_ts->pen_event_work)) {
		queue_work(cnt_ts->ts_workqueue, &cnt_ts->pen_event_work);
	}

	return IRQ_HANDLED;
}

struct dev_power_domain cnt_ts_pwr_domain;

static int cnt_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct CNT_TS_DATA_T *cnt_ts;
	struct input_dev *input_dev;
	int err = 0;
	int i;
	int retry = 0;
	int ret = 0;
	u8 buf[8] = {0};



	//printk("%s %s %d client->irq=%d \n",__FILE__,__func__,__LINE__,client->irq);

	// client ->irq;//IRQ_EINT(6);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	cnt_ts = kzalloc(sizeof(*cnt_ts), GFP_KERNEL);
	if (!cnt_ts)    {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}
	cnt_ts->flag_lock = __SPIN_LOCK_UNLOCKED(cnt_ts->flag_lock);

	this_client = client;

	this_client->dev.pwr_domain = &cnt_ts_pwr_domain;

        cnt_ts->platform_data = client->dev.platform_data;
	
	i2c_set_clientdata(client, cnt_ts);

	 ///open the  power
	 if(cnt_ts->platform_data->power)
	 	cnt_ts->platform_data->power(1);
        ///init
	if (cnt_ts->platform_data->init &&  cnt_ts->platform_data->init()) {
		ret = -EINVAL;
		return ret;
	}
	///reset
	if(cnt_ts->platform_data->reset)
	    cnt_ts->platform_data->reset();


	INIT_WORK(&cnt_ts->pen_event_work, cnt_work_func);

	cnt_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!cnt_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}


#ifdef CONFIG_HAS_EARLYSUSPEND
	printk("\n [TSP]:register the early suspend \n");
	cnt_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	cnt_ts->early_suspend.suspend = cnt_ts_earlysuspend;
	cnt_ts->early_suspend.resume	 = cnt_ts_earlyresume;
	register_early_suspend(&cnt_ts->early_suspend);
#endif


	err = request_irq(client->irq, cnt_ts_irq, IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND, "qt602240_ts", cnt_ts);

	if (err < 0) {
		dev_err(&client->dev, "cnt_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	disable_irq(client ->irq);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	//printk("%s %s %d client->irq=%d \n",__FILE__,__func__,__LINE__);

	cnt_ts->input_dev = input_dev;

	/***setup coordinate area******/
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

	//  ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	// ts->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);
	//  __set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
	//  input_mt_init_slots(ts->input_dev, 11);


	/****** for multi-touch *******/
	for (i=0; i<CFG_MAX_POINT_NUM; i++)   
		_st_finger_infos[i].u2_pressure = -1;

	input_set_abs_params(input_dev,
		ABS_MT_POSITION_X, 0, SCREEN_MAX_X - 1, 0, 0);
	input_set_abs_params(input_dev,
		ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y - 1, 0, 0);
	input_set_abs_params(input_dev,
		ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
		ABS_MT_TRACKING_ID, 0, 30, 0, 0);
	/*****setup key code area******/
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	
#if defined(CONFIG_SUPPORT_CNT_BUTTON)	
	input_dev->keycode = tsp_keycodes;
	for(i = 0; i < CFG_NUMOFKEYS; i++)
	{
		input_set_capability(input_dev, EV_KEY, ((int*)input_dev->keycode)[i]);
		tsp_keystatus[i] = KEY_RELEASE;
	}
#endif
	input_dev->name = CNT_NAME;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
			"cnt_ts_probe: failed to register input device: %s\n",
			dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

#ifdef CNT_PROXIMITY
	msg2133_init_prox();
#endif

	device_init_wakeup(&client->dev, 1);

	enable_irq(client ->irq);
	
#ifdef TP_FIRMWARE_VERSION
	firmware_version_class = class_create(THIS_MODULE, "cn-ts");
	if(IS_ERR(firmware_version_class))
	{
		printk("[TSP]:Failed to create class(firmware)!\n");
	}
	firmware_version_cmd_dev = device_create(firmware_version_class,
		NULL, 0, NULL, "device");
	if(IS_ERR(firmware_version_cmd_dev))
	{
		printk("[TSP]:Failed to create device(firmware_cmd_dev)!\n");
	}
	if(device_create_file(firmware_version_cmd_dev, &dev_attr_version) < 0)
	{
		printk("[TSP]:Failed to create device file(%s)!\n", dev_attr_version.attr.name);
	}
#endif
	

#ifdef TP_FIRMWARE_UPDATE
	firmware_class = class_create(THIS_MODULE, "cn-ts");
	if(IS_ERR(firmware_class))
	{
		printk("[TSP]:Failed to create class(firmware)!\n");
	}
	firmware_cmd_dev = device_create(firmware_class,
		NULL, 0, NULL, "device");
	if(IS_ERR(firmware_cmd_dev))
	{
		printk("[TSP]:Failed to create device(firmware_cmd_dev)!\n");
	}
	if(device_create_file(firmware_cmd_dev, &dev_attr_version) < 0)
	{
		printk("[TSP]:Failed to create device file(%s)!\n", dev_attr_version.attr.name);
	}
	if(device_create_file(firmware_cmd_dev, &dev_attr_update) < 0)
	{
		printk("[TSP]:Failed to create device file(%s)!\n", dev_attr_update.attr.name);
	}
	if(device_create_file(firmware_cmd_dev, &dev_attr_data) < 0)
	{
		printk("[TSP]:Failed to create device file(%s)!\n", dev_attr_data.attr.name);
	}
	if(device_create_file(firmware_cmd_dev, &dev_attr_clear) < 0)
	{
		printk("[TSP]Failed to create device file(%s)!\n", dev_attr_clear.attr.name);
	}
#endif


	printk("[TSP] file(%s), function (%s), -- end\n", __FILE__, __FUNCTION__);
	return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	free_irq(client ->irq, cnt_ts);
exit_irq_request_failed:
	cancel_work_sync(&cnt_ts->pen_event_work);
	destroy_workqueue(cnt_ts->ts_workqueue);
exit_create_singlethread:
	printk("[TSP] ==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
	kfree(cnt_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}



static int __devexit cnt_ts_remove(struct i2c_client *client)
{
	struct CNT_TS_DATA_T *cnt_ts;

	cnt_ts = (struct CNT_TS_DATA_T *)i2c_get_clientdata(client);
	free_irq(client ->irq, cnt_ts);
	input_unregister_device(cnt_ts->input_dev);
	kfree(cnt_ts);
	cancel_work_sync(&cnt_ts->pen_event_work);
	destroy_workqueue(cnt_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL);
	return 0;
}



static const struct i2c_device_id cnt_ts_id[] = {
	{ CNT_NAME, 0 },{ }
};


MODULE_DEVICE_TABLE(i2c, cnt_ts_id);

struct dev_power_domain cnt_ts_pwr_domain = {
	.ops = {
//		.suspend = pm_ops_suspend,
		.suspend = cnt_ts_suspend,
//		.resume = pm_ops_resume,
//		.resume = cnt_ts_resume,
//		.resume_noirq = pm_ops_resume_noirq,
		.resume_noirq = cnt_ts_resume,
	},
};


static struct i2c_driver cnt_ts_driver = {
	.probe    = cnt_ts_probe,
	.remove   = __devexit_p(cnt_ts_remove),
	.id_table = cnt_ts_id,
	.driver   = {
		.name     = CNT_NAME,
	        .owner    = THIS_MODULE,
#if CONFIG_PM_SLEEP
//		.pm = &cnt_ts_pm_ops,
#endif
	},
};

static int __init cnt_ts_init(void)
{
	return i2c_add_driver(&cnt_ts_driver);
}


static void __exit cnt_ts_exit(void)
{
	i2c_del_driver(&cnt_ts_driver);
}



module_init(cnt_ts_init);
module_exit(cnt_ts_exit);

MODULE_AUTHOR("<chiachu.cc.hsu@cntouch.com>");
MODULE_DESCRIPTION("CNTouch TouchScreen driver");
MODULE_LICENSE("GPL");
