/* drivers/input/touchscreen/NVTtouch_ts.c
 *
 * Copyright (C) 2010 - 2011 Novatek, Inc.
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/unistd.h>

#include <linux/input/NVTtouch.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#if ModeB
#include <linux/input/mt.h>
#endif

#if UPDATE_FIRMWARE
#include "NVT_firmware.h"
#endif

#if NVT_PROXIMITY_FUNC_SUPPORT
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif

static const char *NVTtouch_ts_name = "NVTCapacitiveTouchScreen";
static struct workqueue_struct *NVTtouch_wq;
struct i2c_client * i2c_connect_client_NVTtouch = NULL; 
#ifdef CONFIG_HAS_EARLYSUSPEND
static void NVTtouch_ts_early_suspend(struct early_suspend *h);
static void NVTtouch_ts_late_resume(struct early_suspend *h);
#endif 

#define MAX(a,b) ((a) < (b) ? (b) : (a))
#define MIN(a,b) ((a) > (b) ? (b) : (a))

struct i2c_client *NVT_client;
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
static int i2c_read_bytes(struct i2c_client *client, uint8_t *buf, int len)
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
static int i2c_write_bytes(struct i2c_client *client, uint8_t *data, int len)
{
	struct i2c_msg msg;
	int ret = -1;
	int retries = 0;

	msg.flags = !I2C_M_RD;
	msg.addr  = client->addr;
	msg.len   = len;
	msg.buf   = data;		
	
	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1)	break;
		retries++;
	}
	return ret;
}

#if NVT_PROXIMITY_FUNC_SUPPORT
#define TPD_PROXIMITY_ENABLE_REG                  0x88//01
static u8 tpd_proximity_flag = 0;
static u8 tpd_proximity_detect = 1;	//0-->close ; 1--> far away

static s32 tpd_proximity_get_value(void)
{
    return tpd_proximity_detect;
}

static s32 tpd_proximity_enable(s32 enable)
{
    u8  state;
    s32 ret = -1;
    
	TPD_DMESG("tpd_proximity_enable enable=%d\n",enable);
	
    if (enable)
    {
        state = 1;
        tpd_proximity_flag = 1;
        TPD_DMESG("TPD proximity function to be on.\n");
    }
    else
    {
        state = 0;
        tpd_proximity_flag = 0;
        TPD_DMESG("TPD proximity function to be off.\n");
    }

    ret = i2c_write_bytes(i2c_client, TPD_PROXIMITY_ENABLE_REG, &state, 1);

    if (ret < 0)
    {
        TPD_DMESG("TPD %s proximity cmd failed.\n", state ? "enable" : "disable");
        return ret;
    }

    TPD_DMESG("TPD proximity function %s success.\n", state ? "enable" : "disable");
    return 0;
}

s32 tpd_proximity_operate(void *self, u32 command, void *buff_in, s32 size_in,
                   void *buff_out, s32 size_out, s32 *actualout)
{
    s32 err = 0;
    s32 value;
    hwm_sensor_data *sensor_data;

    switch (command)
    {
        case SENSOR_DELAY:
            if ((buff_in == NULL) || (size_in < sizeof(int)))
            {
                TPD_DMESG("Set delay parameter error!");
                err = -EINVAL;
            }

            // Do nothing
            break;

        case SENSOR_ENABLE:
            if ((buff_in == NULL) || (size_in < sizeof(int)))
            {
                TPD_DMESG("Enable sensor parameter error!");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
                err = tpd_proximity_enable(value);
            }

            break;

        case SENSOR_GET_DATA:
            if ((buff_out == NULL) || (size_out < sizeof(hwm_sensor_data)))
            {
                TPD_DMESG("Get sensor data parameter error!");
                err = -EINVAL;
            }
            else
            {
                sensor_data = (hwm_sensor_data *)buff_out;
                sensor_data->values[0] = tpd_proximity_get_value();
                sensor_data->value_divide = 1;
                sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
            }
            break;

        default:
            TPD_DMESG("proxmy sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;
    }

    return err;
}

static int tpd_proximity_event(u8 buf1, u8 buf2)
{
	int ret = 0;
    s32 err = 0;
    hwm_sensor_data sensor_data;
    u8 proximity_status;
    u8 point_data[20];

	TPD_DMESG("tpd_proximity_flag = %d, buf2 = %d\n", tpd_proximity_flag, buf2);

	if (tpd_proximity_flag == 1)
	{
		proximity_status = buf2;

		if (proximity_status & 0x80)				//proximity or large touch detect,enable hwm_sensor.
		{
			tpd_proximity_detect = 0;
		}
		else
		{
			tpd_proximity_detect = 1;
		}

		TPD_DMESG("PROXIMITY STATUS:0x%02X\n", tpd_proximity_detect);
		//map and store data to hwm_sensor_data
		sensor_data.values[0] = tpd_proximity_get_value();
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
		//report to the up-layer
		ret = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data);

		if (ret)
		{
			TPD_DMESG("Call hwmsen_get_interrupt_data fail = %d\n", err);
		}
	}
}

int tpd_proximity_init(void)
{
    int err = 0;
	struct hwmsen_object obj_ps;

    //obj_ps.self = cm3623_obj;
    obj_ps.polling = 0;         //0--interrupt mode; 1--polling mode;
    obj_ps.sensor_operate = tpd_proximity_operate;

    if ((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
    {
        TPD_DMESG("hwmsen attach fail, return:%d.", err);
    }
}
#endif // NVT_PROXIMITY_FUNC_SUPPORT




#if NVT_TOUCH_CTRL_DRIVER
static struct proc_dir_entry *NVT_proc_entry;
#define DEVICE_NAME	"NVTflash"
struct nvt_flash_data *flash_priv;

/*******************************************************
Description:
	Novatek touchscreen control driver initialize function.

Parameter:
	priv:	i2c client private struct.
	
return:
	Executive outcomes.0---succeed.
*******************************************************/
int nvt_flash_write(struct file *file, const char __user *buff, size_t count, loff_t *offp)
{
 struct i2c_msg msgs[2];	
 char *str;
 int ret=-1;
 int retries = 0;
 file->private_data = (uint8_t *)kmalloc(64, GFP_KERNEL);
 str = file->private_data;
 ret=copy_from_user(str, buff, count);

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = str[0];
	msgs[0].len   = str[1];
	msgs[0].buf   = &str[2];

	while(retries < 20)
	{
		ret = i2c_transfer(flash_priv->client->adapter, msgs, 1);
		if(ret == 1)	break;
		else
			printk("write error %d\n", retries);
		retries++;
	}
 return ret;
}

int nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
 struct i2c_msg msgs[2];	 
 char *str;
 int ret = -1;
 int retries = 0;
 file->private_data = (uint8_t *)kmalloc(64, GFP_KERNEL);
 str = file->private_data;
 if(copy_from_user(str, buff, count))
	return -EFAULT;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = str[0];
	msgs[0].len   = 1;
	msgs[0].buf   = &str[2];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = str[0];
	msgs[1].len   = str[1]-1;
	msgs[1].buf   = &str[3];

	while(retries < 20)
	{
		ret = i2c_transfer(flash_priv->client->adapter, msgs, 2);
		if(ret == 2)	break;
		else
			printk("read error %d\n", retries);
		retries++;
	}
	ret=copy_to_user(buff, str, count);
 return ret;
}

int nvt_flash_open(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev;

	dev = kmalloc(sizeof(struct nvt_flash_data), GFP_KERNEL);
	if (dev == NULL) {
		return -ENOMEM;
	}

	rwlock_init(&dev->lock);
	file->private_data = dev;

	return 0;
}

int nvt_flash_close(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev = file->private_data;

	if (dev) {
		kfree(dev);
	}
	return 0;   
}

struct file_operations nvt_flash_fops = {
	.owner = THIS_MODULE,
	.open = nvt_flash_open,
	.release = nvt_flash_close,
	.write = nvt_flash_write,
	.read = nvt_flash_read,
};

static int nvt_flash_init(struct NVTtouch_ts_data *ts)
{		
	int ret=0;
  	NVT_proc_entry = create_proc_entry(DEVICE_NAME, 0666, NULL);
	if(NVT_proc_entry == NULL)
	{
		printk("Couldn't create proc entry!\n");
		ret = -ENOMEM;
		return ret ;
	}
	else
	{
		printk("Create proc entry success!\n");
		NVT_proc_entry->proc_fops = &nvt_flash_fops;
	}
	flash_priv=kzalloc(sizeof(*flash_priv),GFP_KERNEL);	
	if (ts == NULL) {
                ret = -ENOMEM;
                goto error;
	}
	flash_priv->client = ts->client;
	printk("============================================================\n");
	printk("NVT_flash driver loaded\n");
	printk("============================================================\n");	
	return 0;
error:
	if(ret != 0)
	{
	printk("flash_priv error!\n");
	}
	return -1;
}

#endif
#if ReportRate
struct timeval TimeOrgin;
struct timeval TimeNow;
int TouchCount;
int ShowReportRate(void)
{
	if(TouchCount==0)
		do_gettimeofday(&TimeOrgin);
	do_gettimeofday(&TimeNow);
	if(TimeNow.tv_sec>TimeOrgin.tv_sec)
	{
		do_gettimeofday(&TimeOrgin);
		return 1;
	}		
	else
	{
		return 0;		
	}
}
#endif

/*******************************************************
Description:
	Novatek touchscreen initialize function.

Parameter:
	ts:	i2c client private struct.
	
return:
	Executive outcomes.0---succeed.
*******************************************************/

static int NVTtouch_init_panel(struct NVTtouch_ts_data *ts)
{
	int ret=-1;
	uint8_t rd_cfg_buf[16] = {0x78,};
    	struct i2c_client *client = ts->client;
	ts->int_trigger_type = INT_TRIGGER;
	ret=i2c_read_bytes(ts->client, rd_cfg_buf, 16);
	if(ret != 2)
	{
		dev_info(&client->dev, "Read resolution & max_touch_num failed, use default value!\n");
		ts->abs_x_max = TOUCH_MAX_WIDTH;
		ts->abs_y_max = TOUCH_MAX_HEIGHT;
		ts->max_touch_num = MAX_FINGER_NUM;
		ts->int_trigger_type = INT_TRIGGER;
		#ifdef HAVE_TOUCH_KEY
		ts->max_button_num = MAX_KEY_NUM;
		#endif
		ts->chip_ID = IC;
		return 0;
	}
	ts->abs_x_max = (rd_cfg_buf[5]<<8) + rd_cfg_buf[6];
	ts->abs_y_max = (rd_cfg_buf[7]<<8) + rd_cfg_buf[8];
	ts->max_touch_num = rd_cfg_buf[10];
	#ifdef HAVE_TOUCH_KEY
	ts->max_button_num = rd_cfg_buf[11];
	#endif
	ts->int_trigger_type = rd_cfg_buf[12];
	ts->chip_ID = rd_cfg_buf[15];
	dev_info(&client->dev, "ts->max_touch_num=%d\n",rd_cfg_buf[12]);
	if((!ts->abs_x_max)||(!ts->abs_y_max)||(!ts->max_touch_num))
	{
		dev_info(&client->dev, "Read invalid resolution & max_touch_num, use default value!\n");
		ts->abs_x_max = TOUCH_MAX_WIDTH;
		ts->abs_y_max = TOUCH_MAX_HEIGHT;
		ts->max_touch_num = MAX_FINGER_NUM;
	}
	dev_info(&ts->client->dev,"X_MAX = %d,Y_MAX = %d,MAX_TOUCH_NUM = %d\n",ts->abs_x_max,ts->abs_y_max,ts->max_touch_num);
	if((rd_cfg_buf[13]&0x0f)==0x05)
	{
		dev_info(&ts->client->dev, "Touchscreen works in INT wake up green mode!\n");
		ts->green_wake_mode = 1;
	}
	else
	{
		dev_info(&ts->client->dev, "Touchscreen works in IIC wake up green mode!\n");
		ts->green_wake_mode = 0;
	}
	msleep(10);
	return 0; 
}



/*******************************************************
Description:
	Novatek touchscreen update firmware function.

Parameter:
	ts:	i2c client private strduct.
	
return:
	none.
*******************************************************/


#if UPDATE_FIRMWARE

void CTP_I2C_READ(struct i2c_client *client, uint8_t address, uint8_t *buf, uint8_t len)
{
	struct i2c_msg msgs[2];
	int ret = -1;
	int retries = 0;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = address;
	msgs[0].len   = 1;
	msgs[0].buf   = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = address;
	msgs[1].len   = len-1;
	msgs[1].buf   = &buf[1];

	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, msgs, 2);
		if(ret == 2)	break;
		retries++;
	}
	return;	
}


void CTP_I2C_WRITE (struct i2c_client *client, uint8_t address, uint8_t *data, uint8_t len)
{
	struct i2c_msg msg;
	int ret = -1;
	int retries = 0;

	msg.flags = !I2C_M_RD;
	msg.addr  = address;
	msg.len   = len;
	msg.buf   = data;		
	
	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1)	break;
		retries++;
	}
	return;
}

int Check_FW_Ver(struct NVTtouch_ts_data *ts)
{
	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	I2C_Buf[0] = 0x78;
	CTP_I2C_READ(ts->client, 0x01, I2C_Buf, 2);
	dev_info(&ts->client->dev, "FW Ver= %d", I2C_Buf[1]);
	if(I2C_Buf[1]>FW_VERSION)
		return 1;
	else
		return 0;
}

int Check_CheckSum(struct NVTtouch_ts_data *ts)
{
	uint8_t buf1[64];
	uint8_t buf2[64];
	int i, j, k, Retry_Counter=0;
	int addr;
	uint8_t addrH, addrL;
	unsigned short RD_Filechksum, WR_Filechksum;

	WR_Filechksum = 0;

	buf1[0]=0x00;
	buf1[1]=0x5A;
	CTP_I2C_WRITE(ts->client, HW_Address, buf1, 2);

	msleep(1000);

	
	if(ts->chip_ID==2)
	{
		buf1[0]=0xFF;
		buf1[1]=0x0A;
		buf1[2]=0x0C;
		CTP_I2C_WRITE(ts->client, FW_Address, buf1, 3);

		buf1[0]=0x00;
		buf1[1]=0x00;
		buf1[2]=0x00;
		buf1[3]=0x00;
		CTP_I2C_WRITE(ts->client, FW_Address, buf1, 4);

		buf1[0]=0xFF;
		buf1[1]=0x0F;
		buf1[2]=0xFF;
		CTP_I2C_WRITE(ts->client, FW_Address, buf1, 3);


		buf1[0]=0x00;
		buf1[1]=0xE1;
		CTP_I2C_WRITE(ts->client, FW_Address, buf1, 2);

		for(i=0;i<BUFFER_LENGTH;i++)
		{
			WR_Filechksum+=(BUFFER_DATA[i]);
		}

		msleep(800);


		do
		{
			buf1[0]=0xFF;
			buf1[1]=0x0A;
			buf1[2]=0x0C;
			CTP_I2C_WRITE(ts->client, FW_Address, buf1, 3);

			buf2[0]=0x00;
			buf2[1]=0x00;
			buf2[2]=0x00;
			buf2[3]=0x00;
			CTP_I2C_READ(ts->client, FW_Address, buf2, 4);

			Retry_Counter++;
			msleep(10);
		}while((Retry_Counter<30)&& (buf2[1]!=0xAA));

		//---------------------------------------------------------------------------------------

		if(buf2[1]==0xAA)
		{
			RD_Filechksum=(buf2[2]*256)+buf2[3];
			if(RD_Filechksum==(WR_Filechksum))
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
		else
		{
			return -1;
		}
	}
	else if(ts->chip_ID==3)
	{
		buf1[0]=0xFF;
		buf1[1]=0x8F;
		buf1[2]=0xFF;
		CTP_I2C_WRITE(ts->client, FW_Address, buf1, 3);

		buf1[0]=0x00;
		buf1[1]=0xE1;
		CTP_I2C_WRITE(ts->client, FW_Address, buf1, 2);

		for(i=0;i<(BUFFER_LENGTH)/128;i++)
		{
			for(j=0;j<16;j++)
			{
				unsigned char tmp=0;
				addrH = addr>>8;
				addrL = addr&0xFF;
				for(k=0;k<8;k++)
				{
					tmp+=BUFFER_DATA[i*128+j*8+k];
				}
				tmp = tmp+addrH+addrL+8;
				tmp = ~tmp+1;
				WR_Filechksum+=tmp;
				addr+=8;
			}
		}


		msleep(800);

		do
		{
			msleep(10);
			buf1[0]=0xFF;
			buf1[1]=0x8E;
			buf1[2]=0x0D;
			CTP_I2C_WRITE(ts->client, FW_Address, buf1, 3);
	
			buf2[0]=0x00;
			buf2[1]=0x00;
			buf2[2]=0x00;
			buf2[3]=0x00;
			CTP_I2C_READ(ts->client, FW_Address, buf2, 4);


		}while((Retry_Counter<20)&& (buf2[1]!=0xAA));

		//---------------------------------------------------------------------------------------

		if(buf2[1]==0xAA)
		{
			RD_Filechksum=(buf2[2]<<8)+buf2[3];
			if(RD_Filechksum==WR_Filechksum)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
		else
		{
			return -1;
		}

	}
	else if(ts->chip_ID==4)
	{
		buf1[0]=0xFF;
		buf1[1]=0x0E;
		buf1[2]=0xF9;
		CTP_I2C_WRITE(ts->client, FW_Address, buf1, 3);

		buf1[0]=0x00;
		buf1[1]=0xE1;
		CTP_I2C_WRITE(ts->client, FW_Address, buf1, 2);

		for(i=0;i<BUFFER_LENGTH;i++)
		{
			WR_Filechksum+=(BUFFER_DATA[i]);
		}


		msleep(800);

		do
		{
			msleep(10);
			buf1[0]=0xFF;
			buf1[1]=0x0C;
			buf1[2]=0xD7;
			CTP_I2C_WRITE(ts->client, FW_Address, buf1, 3);
	
			buf2[0]=0x00;
			buf2[1]=0x00;
			buf2[2]=0x00;
			buf2[3]=0x00;
			CTP_I2C_READ(ts->client, FW_Address, buf2, 4);


		}while((Retry_Counter<20)&& (buf2[1]!=0xAA));

		//---------------------------------------------------------------------------------------

		if(buf2[1]==0xAA)
		{
			RD_Filechksum=(buf2[2]<<8)+buf2[3];
			if(RD_Filechksum==WR_Filechksum)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
		else
		{
			return -1;
		}

	}
	else
		return -1;

}

void Update_Firmware(struct NVTtouch_ts_data *ts)
{
	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t i = 0;
	uint8_t j = 0;
	unsigned int Flash_Address = 0;
	unsigned int Row_Address = 0;
	uint8_t CheckSum[16]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};	// 128/8 = 16 times ;
	struct i2c_client *client = ts->client;

	//-------------------------------
	// Step1 --> initial BootLoader
 	// Note. HW_Address -> 0x00 -> 0x00 ;
 	// 須配合 Reset Pin 
	//-------------------------------
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0xA5;
	CTP_I2C_WRITE(ts->client, HW_Address, I2C_Buf, 2);	// Write a “A5H” to NT1100x

	msleep(2);	// Delay.2mS
  
	//Step 1 : Initiate Flash Block
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x00;
	CTP_I2C_WRITE(ts->client, HW_Address, I2C_Buf, 2);	// Write a 0x00 to NT1100x

	msleep(20);	// Delay.5mS

	// Read NT1100x status
	I2C_Buf[0] = 0x00;
	CTP_I2C_READ(ts->client, HW_Address, I2C_Buf, 2);
	// if return “AAH” then going next step
	if (I2C_Buf[1] != 0xAA)
	{
		dev_info(&client->dev, "Program : init get status(0x%2X) error", I2C_Buf[1]);
		return;
	}
	dev_info(&client->dev, "Program : init get status(0x%2X) success", I2C_Buf[1]);

	//---------------------------------------------------------
 	// Step 2 : Erase 
 	//---------------------------------------------------------

	if(ts->chip_ID==3)
	{
	 	for (i = 0 ; i < BUFFER_LENGTH/128 ; i++)	// 26K equals 208 Rows 
	 	{
	 		Row_Address = i * 128; 															
	 						 					
	 		I2C_Buf [0] = 0x00;
	 		I2C_Buf [1] = 0x30;	// Row Erase command : 30H  
	 		I2C_Buf [2] = (uint8_t)((Row_Address & 0xFF00) >> 8 );	// Address High Byte  
	 		I2C_Buf [3] = (uint8_t)(Row_Address & 0x00FF);	// Address Low Byte 
	 						
	 		CTP_I2C_WRITE(ts->client, HW_Address, I2C_Buf, 4);	// Write 30H, Addr_H & Addr_L to NT11003
	 
	 		msleep(15);	// Delay 15 ms 
		          
	    	// Read NT11003 status
	 		CTP_I2C_READ(ts->client, HW_Address, I2C_Buf, 2); 
		          
	    	// if NT1003 return AAH then going next step          
		    	if (I2C_Buf[1] != 0xAA)
		    	{
		    		dev_info(&client->dev, "Program : erase(0x%2X) error", I2C_Buf[1]);
		    		return;
		    	}			
	 	}
	}
	else if(ts->chip_ID==2)
	{
	    	for(i=0;i<BUFFER_LENGTH/2048;i++)
		{
			Row_Address=i>>11;
			I2C_Buf[0]=0x00;
			I2C_Buf[1]=0x33;
			I2C_Buf[2]=i<<3;
			I2C_Buf[3]=0x00;
			CTP_I2C_WRITE(ts->client, HW_Address, I2C_Buf, 4);
			msleep(20);

			I2C_Buf[0]=0x00;

			while(1)
			{
				msleep(1);
				CTP_I2C_READ(ts->client, HW_Address, I2C_Buf, 2);
				if(I2C_Buf[1]==0xAA)
					break;
			    	msleep(1);
			}
		}
	}
	else if(ts->chip_ID==4)
	{

		I2C_Buf[0]=0xFF;
		I2C_Buf[1]=0xF0;
		I2C_Buf[2]=0xAC;
		CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);
		msleep(20);

		I2C_Buf[0]=0x00;
		I2C_Buf[1]=0x21;
		CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 2);
		msleep(20);

		I2C_Buf[0]=0x00;
		I2C_Buf[1]=0x99;
		I2C_Buf[2]=0x00;
		I2C_Buf[3]=0x0E;
		I2C_Buf[4]=0x01;
		CTP_I2C_WRITE(ts->client, HW_Address, I2C_Buf, 5);
		msleep(20);

		I2C_Buf[0]=0x00;
		I2C_Buf[1]=0x81;
		CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 2);
		msleep(20);

		I2C_Buf[0]=0x00;
		I2C_Buf[1]=0x99;
		I2C_Buf[2]=0x00;
		I2C_Buf[3]=0x0F;
		I2C_Buf[4]=0x01;
		CTP_I2C_WRITE(ts->client, HW_Address, I2C_Buf, 5);
		msleep(20);

		I2C_Buf[0]=0x00;
		I2C_Buf[1]=0x01;
		CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 2);
		msleep(20);

		I2C_Buf[0]=0xFF;
		I2C_Buf[1]=0x00;
		I2C_Buf[2]=0x00;
		CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);
		msleep(20);

	    	for(i=0;i<BUFFER_LENGTH/4096;i++)
		{
			Row_Address=(i*16);
			I2C_Buf[2]=0x00;
			I2C_Buf[3]=0x33;
			I2C_Buf[4]=Row_Address;
			CTP_I2C_WRITE(ts->client, HW_Address, I2C_Buf, 3);
			msleep(80);

			I2C_Buf[0]=0x00;

			while(1)
			{
				CTP_I2C_READ(ts->client, HW_Address, I2C_Buf, 2);
				if(I2C_Buf[1]==0xAA)
					break;
			    	msleep(1);
			}
		}
	}
	dev_info(&client->dev, "Program : erase(0x%2X) success", I2C_Buf[1]);

	Flash_Address = 0;
        		
	//////////////////////////////////////////////////////////////////////////////////// 		
	//----------------------------------------
	// Step3. Host write 128 bytes to NT11003  
	//----------------------------------------
	dev_info(&client->dev, "Program : write begin, please wait ...");
	if(ts->chip_ID==3)
	{
		for (j = 0 ; j < BUFFER_LENGTH/128 ; j++)	// Write/ Read 208 times 
		{
			Flash_Address = j * 128 ; 						     		 

		   	for (i = 0 ; i < 16 ; i++)	// 128/8 = 16 times for One Row program 
			{
	    		// Step 3 : write binary data to NT11003  
	  			I2C_Buf[0] = 0x00;
				I2C_Buf[1] = 0x55;	//Flash write command
				I2C_Buf[2] = (uint8_t)(Flash_Address  >> 8);	//Flash address [15:8]
				I2C_Buf[3] = (uint8_t)(Flash_Address & 0xFF);	//Flash address [7:0]
				I2C_Buf[4] = 0x08;	//How many prepare to write to NT1100
				I2C_Buf[6] = BUFFER_DATA[Flash_Address + 0];	//Binary data 1
				I2C_Buf[7] = BUFFER_DATA[Flash_Address + 1];	//Binary data 2
				I2C_Buf[8] = BUFFER_DATA[Flash_Address + 2];	//Binary data 3
				I2C_Buf[9] = BUFFER_DATA[Flash_Address + 3];	//Binary data 4
				I2C_Buf[10] = BUFFER_DATA[Flash_Address + 4];   //Binary data 5
				I2C_Buf[11] = BUFFER_DATA[Flash_Address + 5];	//Binary data 6
				I2C_Buf[12] = BUFFER_DATA[Flash_Address + 6];	//Binary data 7
				I2C_Buf[13] = BUFFER_DATA[Flash_Address + 7];	//Binary data 8
		   
				// Calculate a check sum by Host controller. 
				// Checksum = / (FLASH_ADRH+FLASH_ADRL+LENGTH+
				//               Binary_Data1+Binary_Data2+Binary_Data3+Binary_Data4+
				//               Binary_Data5+Binary_Data6+Binary_Data7+Binary_Data8) + 1 
				CheckSum[i] = ~(I2C_Buf[2] + I2C_Buf[3] + I2C_Buf[4] + I2C_Buf[6] + I2C_Buf[7] + 
		    	          I2C_Buf[8] + I2C_Buf[9] + I2C_Buf[10] + I2C_Buf[11] + I2C_Buf[12] +
		        	      I2C_Buf[13]) + 1; 
		   		
				I2C_Buf[5] = CheckSum[i];	// Load check sum to I2C Buffer 
				CTP_I2C_WRITE(ts->client, HW_Address, I2C_Buf, 14);	//Host write I2C_Buf[0…12] to NT1100x.

				msleep(1);	// Delay 1 ms 

				// Read NT1100x status
	   			I2C_Buf[0] = 0x00;
				CTP_I2C_READ(ts->client, HW_Address, I2C_Buf, 2);

				// if return “AAH” then going next step
				if (I2C_Buf[1] != 0xAA)
				{
					dev_info(&client->dev, "Program : write(j=%d, i=%d, 0x%2X) error", j, i, I2C_Buf[1]);
	      			return;
				}
				Flash_Address += 8 ;	// Increase Flash Address. 8 bytes for 1 time
			}
		   	
			msleep(15);	// Each Row program --> Need 15ms delay time 
		}
	}
	else if(ts->chip_ID==2)
	{
	    	for(j=0;j<BUFFER_LENGTH/32;j++)
		{
	    		Flash_Address=(j)*32;

		    	for (i = 0 ; i < 4 ; i++, Flash_Address += 8)	// 128/8 = 16 times for One Row program
			{
	    		// Step 3 : write binary data to NT11003
	  			I2C_Buf[0] = 0x00;
				I2C_Buf[1] = 0x55;	//Flash write command
				I2C_Buf[2] = (uint8_t)(Flash_Address  >> 8);	//Flash address [15:8]
				I2C_Buf[3] = (uint8_t)(Flash_Address & 0xFF);	//Flash address [7:0]
				I2C_Buf[4] = 0x08;	//How many prepare to write to NT1100
				I2C_Buf[6] = BUFFER_DATA[Flash_Address + 0];	//Binary data 1
				I2C_Buf[7] = BUFFER_DATA[Flash_Address + 1];	//Binary data 2
				I2C_Buf[8] = BUFFER_DATA[Flash_Address + 2];	//Binary data 3
				I2C_Buf[9] = BUFFER_DATA[Flash_Address + 3];	//Binary data 4
				I2C_Buf[10] = BUFFER_DATA[Flash_Address + 4];   //Binary data 5
				I2C_Buf[11] = BUFFER_DATA[Flash_Address + 5];	//Binary data 6
				I2C_Buf[12] = BUFFER_DATA[Flash_Address + 6];	//Binary data 7
				I2C_Buf[13] = BUFFER_DATA[Flash_Address + 7];	//Binary data 8

				// Calculate a check sum by Host controller.
				// Checksum = / (FLASH_ADRH+FLASH_ADRL+LENGTH+
				//               Binary_Data1+Binary_Data2+Binary_Data3+Binary_Data4+
				//               Binary_Data5+Binary_Data6+Binary_Data7+Binary_Data8) + 1
				CheckSum[i] = ~(I2C_Buf[2] + I2C_Buf[3] + I2C_Buf[4] + I2C_Buf[6] + I2C_Buf[7] +
	            	          I2C_Buf[8] + I2C_Buf[9] + I2C_Buf[10] + I2C_Buf[11] + I2C_Buf[12] +
	                	      I2C_Buf[13]) + 1;

				I2C_Buf[5] = CheckSum[i];	// Load check sum to I2C Buffer
				CTP_I2C_WRITE(ts->client, HW_Address, I2C_Buf, 14);

				msleep(2);
			}
			// Read NT1100x status
   			I2C_Buf[0] = 0x00;
			while(1)
			{
				CTP_I2C_READ(ts->client, HW_Address, I2C_Buf, 2);
				if(I2C_Buf[1]==0xAA)
					break;
			}
		}		
	}
	else if(ts->chip_ID==4)
	{
		for (j = 0 ; j < BUFFER_LENGTH/128 ; j++)
		{
			Flash_Address = j * 128 ; 						     		 

		   	for (i = 0 ; i < 16 ; i++)
			{
	  			I2C_Buf[0] = 0x00;
				I2C_Buf[1] = 0x55;	//Flash write command
				I2C_Buf[2] = (uint8_t)(Flash_Address  >> 8);	//Flash address [15:8]
				I2C_Buf[3] = (uint8_t)(Flash_Address & 0xFF);	//Flash address [7:0]
				I2C_Buf[4] = 0x08;	//How many prepare to write to NT1100
				I2C_Buf[6] = BUFFER_DATA[Flash_Address + 0];	//Binary data 1
				I2C_Buf[7] = BUFFER_DATA[Flash_Address + 1];	//Binary data 2
				I2C_Buf[8] = BUFFER_DATA[Flash_Address + 2];	//Binary data 3
				I2C_Buf[9] = BUFFER_DATA[Flash_Address + 3];	//Binary data 4
				I2C_Buf[10] = BUFFER_DATA[Flash_Address + 4];   //Binary data 5
				I2C_Buf[11] = BUFFER_DATA[Flash_Address + 5];	//Binary data 6
				I2C_Buf[12] = BUFFER_DATA[Flash_Address + 6];	//Binary data 7
				I2C_Buf[13] = BUFFER_DATA[Flash_Address + 7];	//Binary data 8
		   
				CheckSum[i] = ~(I2C_Buf[2] + I2C_Buf[3] + I2C_Buf[4] + I2C_Buf[6] + I2C_Buf[7] + 
		    	          I2C_Buf[8] + I2C_Buf[9] + I2C_Buf[10] + I2C_Buf[11] + I2C_Buf[12] +
		        	      I2C_Buf[13]) + 1; 
		   		
				I2C_Buf[5] = CheckSum[i];	// Load check sum to I2C Buffer 
				CTP_I2C_WRITE(ts->client, HW_Address, I2C_Buf, 14);	//Host write I2C_Buf[0…12] to NT1100x.

				msleep(1);	// Delay 1 ms 

				// Read NT1100x status
	   			I2C_Buf[0] = 0x00;
				CTP_I2C_READ(ts->client, HW_Address, I2C_Buf, 2);

				// if return “AAH” then going next step
				if (I2C_Buf[1] != 0xAA)
				{
					dev_info(&client->dev, "Program : write(j=%d, i=%d, 0x%2X) error", j, i, I2C_Buf[1]);
	      			return;
				}
				Flash_Address += 8 ;	// Increase Flash Address. 8 bytes for 1 time
			}
		   	
			msleep(15);	// Each Row program --> Need 15ms delay time 
		}
	}

	dev_info(&client->dev, "Program : write finish ~~");
	/////////////////////////////////////////////////////////////////////
	
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x5A;
	CTP_I2C_WRITE(ts->client, HW_Address, I2C_Buf, 2);	
	msleep(500);
	dev_info(&client->dev, "Program : OK");

}
#endif

/*******************************************************
Description:
	Novatek touchscreen work function.

Parameter:
	ts:	i2c client private struct.
	
return:
	Executive outcomes.0---succeed.
*******************************************************/
#if ModeB
static unsigned char touch_cunt_old;
unsigned char Up_Buf[MAX_FINGER_NUM] = {1,1,1,1,1,1,1,1,1,1};  //紀錄ID 使用狀態 0:使用中 1:未使用 2:上次有用這次沒用
#endif
static void NVTtouch_ts_work_func(struct work_struct *work)
{	
	int ret = -1;
	// Support 10 points maximum
	struct NVTtouch_ts_data *ts = container_of(work, struct NVTtouch_ts_data, work);
	uint8_t  point_data[ (MAX_FINGER_NUM*6)+2+1]={0};
	unsigned int position = 0;	
	uint8_t track_id[MAX_FINGER_NUM] = {0,1,2,3,4,5,6,7,8,9};
	unsigned int input_x = 0;
	unsigned int input_y = 0;
	unsigned char input_w = 0;
	unsigned char index = 0;
	int count;

	unsigned char run_count = 0;
	struct i2c_client *client = ts->client;

#if ModeB
	unsigned char lift_count = 0;
	unsigned char touch_cunt_now = 0;
	unsigned char Up_Index = 0;
	unsigned char tpid=99;
#endif	
	

	ret = i2c_read_bytes(ts->client, point_data,  ts->max_touch_num*6+2+1);

	if (ret < 0) {
		dev_info(&client->dev, "%s: i2c_read_bytes fail", __func__ );
		goto XFER_ERROR;
	}
#if ModeB
  	for (index = 0; index < ts->max_touch_num; index++) //0~9 (10 points)
  	{
		position = 1 + 6*index;
		if ( ( ( (point_data[1+index*6]>>3)&0x1F) > 0) && ( ((point_data[1+index*6]>>3)&0x1F)  <= ts->max_touch_num) )  
		{
			track_id[index] = ( (point_data[1+index*6]>>3)&0x1F )-1;
			touch_cunt_now++;
		}
	}

  	for (index = 0; index < touch_cunt_now; index++) //0~9 (10 points)
  	{
		position = 1 + 6*index;

		if ( (point_data[position]&0x03) == 0x03 )     // 有ID 的break
		{
			Up_Buf[track_id[index]] = 1;  
		    input_mt_slot(ts->input_dev, track_id[index]);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
			#if Qualcomm
				input_report_key(ts->input_dev, BTN_TOUCH, 0);
			#endif
			lift_count++;
			
		} else {	// move
			input_x = (unsigned int)(point_data[position+1]<<4) + (unsigned int) (point_data[position+3]>>4);
			input_y = (unsigned int)(point_data[position+2]<<4) + (unsigned int) (point_data[position+3]&0x0f);
			input_w = (unsigned int) (point_data[position+4])+10;
			
			if (input_x < 0)	input_x = 0;
			if (input_y < 0)	input_y = 0;
				
			if((input_x > ts->abs_x_max)||(input_y > ts->abs_y_max))	continue;
			input_mt_slot(ts->input_dev, track_id[index]);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
		    	        
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);

			#if Qualcomm
				input_report_key(ts->input_dev, BTN_TOUCH, 1);
			#endif

			Up_Buf[track_id[index]] = 0;  
			tpid = index;
		}
	}//end for loop
	
	// 掉break的情形 在這一段補
	for(index=0;index<ts->max_touch_num;index++)
	{
		if(Up_Buf[index]==0)	//這個ID這次有用到
			Up_Buf[index]=2;
		else if(Up_Buf[index]==2)	//這個ID 上次有用 這次沒用 所以報break
		{
		    input_mt_slot(ts->input_dev, index);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
			#if Qualcomm
				input_report_key(ts->input_dev, BTN_TOUCH, 0);
			#endif
			Up_Buf[index]=1;
		}
	}	

//	input_sync(ts->input_dev);

	touch_cunt_old = touch_cunt_now - lift_count;

#else
	run_count = ts->max_touch_num;
  	for (index = 0; index < ts->max_touch_num; index++) //0~9 (10 points)
  	{
		position = 1 + 6*index;
		if ( ( ( (point_data[1+index*6]>>3)&0x1F) > 0) &&
			( ((point_data[1+index*6]>>3)&0x1F)  <= ts->max_touch_num) )  
		{
			track_id[index] = (point_data[1+index*6]>>3) -1;
		}
		else if ( (point_data[position]&0x03) == 0x03 ||
			(point_data[position]&0x03) == 0x00 )
			run_count--;
	}

	if(run_count)
	{
		for (index = 0; index < ts->max_touch_num; index++) //0~9 (10 points)
		{
			position = 1 + 6*index;
			if ( (point_data[position]&0x03) == 0x03 || (point_data[position]&0x03) == 0x00 )
			{
					continue;//input_report_key(ts->input_dev, BTN_TOUCH, 0);
			} 
			else 
			{
				input_x = (unsigned int)(point_data[position+1]<<4) + (unsigned int) (point_data[position+3]>>4);
				input_y = (unsigned int)(point_data[position+2]<<4) + (unsigned int) (point_data[position+3]&0x0f);
				input_w = (unsigned int) (point_data[position+4])+10;
				if (input_x < 0)	input_x = 0;
				if (input_y < 0)	input_y = 0;
				if((input_x > TOUCH_MAX_WIDTH)||(input_y > TOUCH_MAX_HEIGHT))	continue;
				printk(" %s %d input_x=%d,input_y=%d,input_w=%d\n",__func__,__LINE__,input_x,input_y,input_w);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, input_w);
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, track_id[index]);
				input_report_key(ts->input_dev, BTN_TOUCH, 1);
			}
			input_mt_sync(ts->input_dev);
		}//end for loop
	}
	else
	{
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
		input_mt_sync(ts->input_dev);
    }    	
//	input_sync(ts->input_dev);
#endif	


#if NVT_PROXIMITY_FUNC_SUPPORT
	ret = tpd_proximity_event(point_data[(MAX_FINGER_NUM*6)+2+1-2], point_data[(MAX_FINGER_NUM*6)+2+1-1]);
#endif

#ifdef HAVE_TOUCH_KEY
	if(point_data[ts->max_touch_num*6+1]==0xF8)
	{
		for(count = 0; count < MAX_KEY_NUM; count++)
		{
			input_report_key(ts->input_dev, touch_key_array[count], ((point_data[ts->max_touch_num*6+2]>>count)&(0x01)));	
		}
	}
	else
	{
		for(count = 0; count < MAX_KEY_NUM; count++)
		{
			input_report_key(ts->input_dev, touch_key_array[count], 0);	
		}
	}
	
#endif
	input_sync(ts->input_dev);

#if defined(INT_PORT)
	if(ts->int_trigger_type> 1)
	{
		msleep(POLL_TIME);
		//goto COORDINATE_POLL;
	}
#endif
	goto END_WORK_FUNC;

//NO_ACTION:	

#ifdef HAVE_TOUCH_KEY
//	dev_info(&client->dev, "HAVE KEY DOWN!0x%x\n",point_data[1]);
	if(point_data[ts->max_touch_num*6+1]==0x1F)	
	{
		for(count = 0; count < MAX_KEY_NUM; count++)
		{
			input_report_key(ts->input_dev, touch_key_array[count], ((point_data[ts->max_touch_num*6+2]>>count)&(0x01)));	
		}
	}	   
#endif
END_WORK_FUNC:
XFER_ERROR:
	if(ts->use_irq)
		enable_irq(ts->client->irq);
}

/*******************************************************
Description:
	Timer interrupt service routine.

Parameter:
	timer:	timer struct pointer.
	
return:
	Timer work mode. HRTIMER_NORESTART---not restart mode
*******************************************************/
static enum hrtimer_restart NVTtouch_ts_timer_func(struct hrtimer *timer)
{
	struct NVTtouch_ts_data *ts = container_of(timer, struct NVTtouch_ts_data, timer);
	queue_work(NVTtouch_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, (POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

/*******************************************************
Description:
	External interrupt service routine.

Parameter:
	irq:	interrupt number.
	dev_id: private data pointer.
	
return:
	irq execute status.
*******************************************************/
static irqreturn_t NVTtouch_ts_irq_handler(int irq, void *dev_id)
{
	struct NVTtouch_ts_data *ts = dev_id;
	printk("%s %s %d \n",__FILE__,__func__,__LINE__);
#if ReportRate
	if(ShowReportRate()==1)
	{
		printk("Report Rate = %d\n", TouchCount);
		TouchCount=0;
	}
	else
	{
		TouchCount++;
	}

#endif

	disable_irq_nosync(ts->client->irq);
	queue_work(NVTtouch_wq, &ts->work);
	
	return IRQ_HANDLED;
}

/*******************************************************
Description:
	Novatek touchscreen 
 function.

Parameter:
	client:	i2c device struct.
	id:device id.
	
return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int NVTtouch_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	int retry = 0;
	struct NVTtouch_ts_data *ts;
	//struct NVTtouch_i2c_platform_data *pdata;
	uint8_t test_data[7] = {0x00,};
	const char irq_table[4] = {IRQ_TYPE_EDGE_RISING,
							   IRQ_TYPE_EDGE_FALLING,
							   IRQ_TYPE_LEVEL_LOW,
							   IRQ_TYPE_LEVEL_HIGH};

	//struct NVTtouch_i2c_rmi_platform_data *pdata;
	dev_info(&client->dev, "Install touch driver.\n");

	printk("%s %s %d \n",__FILE__,__func__,__LINE__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		dev_info(&client->dev,  "Must have I2C_FUNC_I2C.\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	//init gpio.
	//ts->pw_gpio=
	//nvt_request_io_port(ts);
	  printk("%s %s %d client->irq=%d \n",__FILE__,__func__,__LINE__,client->irq);
	if (client->irq)
	{
		dev_info(&client->dev, "ts->int_trigger_type=%d\n",ts->int_trigger_type);
		ret  = request_irq(client->irq, NVTtouch_ts_irq_handler, irq_table[ts->int_trigger_type],
			client->name, ts);
		if (ret != 0) {
			dev_info(&client->dev, "Cannot allocate ts INT! ERRNO:%d\n", ret);
			goto err_gpio_request_failed;
		}
		else 
		{	
			disable_irq(client->irq);
			ts->use_irq = 1;
			dev_info(&client->dev, "Reques EIRQ %d succesd\n", client->irq);
		}

	}    //End of "if (client->irq)"

err_gpio_request_failed:

	i2c_connect_client_NVTtouch = client;
	for(retry=0; retry < 30; retry++)
	{
		disable_irq(client->irq);
		msleep(5);
		enable_irq(client->irq);
		ret =i2c_read_bytes(client, test_data, 5);
		dev_info(&client->dev, "test_data[1]=%d,test_data[2]=%d,test_data[3]=%d,test_data[4]=%d,test_data[5]=%d\n",
			test_data[1],test_data[2],test_data[3],test_data[4],test_data[5]);
		if (ret > 0)
			break;
		dev_info(&client->dev, "NVTtouch i2c test failed!\n");
	}
	if(ret <= 0)
	{
		dev_info(&client->dev,  "I2C communication ERROR!NVTtouch touchscreen driver become invalid\n");
		goto err_i2c_failed;
	}	

	INIT_WORK(&ts->work, NVTtouch_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	//pdata = client->dev.platform_data;
	


	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		dev_info(&client->dev, "Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	
#if UPDATE_FIRMWARE
	ret = Check_CheckSum(ts);
	if(ret==-1)
	{
		Update_Firmware(ts);
	}
	else if(ret==0&&(Check_FW_Ver(ts)==0))
	{
		Update_Firmware(ts);
	}
	
	
#endif

//#if 1	
	for(retry=0; retry<3; retry++)
	{
		ret=NVTtouch_init_panel(ts);
		msleep(2);
		if(ret != 0)
			continue;
		else
			break;
	}
	if(ret != 0) {
		ts->bad_data=1;
		goto err_init_NVT_ts;
	}
//#else
//		ts->abs_x_max = TOUCH_MAX_WIDTH;
//		ts->abs_y_max = TOUCH_MAX_HEIGHT;
//		ts->max_touch_num = MAX_FINGER_NUM;
//		ts->int_trigger_type = INT_TRIGGER;
//		ts->max_button_num = MAX_KEY_NUM;
//		ts->chip_ID = IC;
//#endif

	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE); 

#if ModeB
	input_mt_init_slots(ts->input_dev, ts->max_touch_num);
#endif	
	
	input_set_abs_params(ts->input_dev, ABS_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);    //pressure = 255

#ifdef NVTTOUCH_MULTI_TOUCH
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);    //area = 255

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);
#endif	

#ifdef HAVE_TOUCH_KEY
	for(retry = 0; retry < MAX_KEY_NUM; retry++)
	{
		input_set_capability(ts->input_dev,EV_KEY,touch_key_array[retry]);	
	}
#endif




	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = NVTtouch_ts_name;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0x0306;
	ts->input_dev->id.product = 0xFF2F;
	
	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_info(&client->dev, "Probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}
	ts->bad_data = 0;
		
	if (!ts->use_irq) 
	{
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = NVTtouch_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
	
	if(ts->use_irq)
		enable_irq(client->irq);
	
	#if NVT_TOUCH_CTRL_DRIVER
		nvt_flash_init(ts);
	#endif

	#if NVT_PROXIMITY_FUNC_SUPPORT
    		tpd_proximity_init();
	#endif

	
	#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = NVTtouch_ts_early_suspend;
	ts->early_suspend.resume = NVTtouch_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	dev_info(&client->dev, "Start %s in %s mode\n", 
		ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");
	dev_info(&client->dev,  "Driver Modify Date:2011-06-27\n");
	return 0;

err_init_NVT_ts:
	if(ts->use_irq)
	{
		ts->use_irq = 0;
		free_irq(client->irq,ts);
	}
	else 
		hrtimer_cancel(&ts->timer);

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
	i2c_set_clientdata(client, NULL);
err_i2c_failed:	
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.

Parameter:
	client:	i2c device struct.
	
return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int NVTtouch_ts_remove(struct i2c_client *client)
{
	struct NVTtouch_ts_data *ts = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	if (ts && ts->use_irq) 
	{
		free_irq(client->irq, ts);
	}	
	else if(ts)
		hrtimer_cancel(&ts->timer);
	
	dev_notice(&client->dev,"The driver is removing...\n");
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int NVTtouch_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct NVTtouch_ts_data *ts = i2c_get_clientdata(client);

	#if NVT_PROXIMITY_FUNC_SUPPORT
	if(tpd_proximity_flag == 1)
	{
		return;
	}
	#endif

	
	if (ts->use_irq)
		disable_irq(client->irq);
	 else
		hrtimer_cancel(&ts->timer);
	  //ret = cancel_work_sync(&ts->work);
	  //if(ret && ts->use_irq)	
		//enable_irq(client->irq);


	if (ts->power) {
		ret = ts->power(ts, 0);
		if (ret < 0)
			dev_info(&client->dev, KERN_ERR "NVTtouch_ts_resume power off failed\n");
	}
	return 0;
}

static int NVTtouch_ts_resume(struct i2c_client *client)
{
	int ret;
	struct NVTtouch_ts_data *ts = i2c_get_clientdata(client);

#if NVT_PROXIMITY_FUNC_SUPPORT
	if(tpd_proximity_flag == 1)
	{
		return;
	}
#endif

	if (ts->power) {
		ret = ts->power(ts, 1);
		if (ret < 0)
			dev_info(&client->dev, KERN_ERR "NVTtouch_ts_resume power on failed\n");
	}

	if (ts->use_irq)
		enable_irq(client->irq);
	 else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void NVTtouch_ts_early_suspend(struct early_suspend *h)
{
	struct NVTtouch_ts_data *ts;
	ts = container_of(h, struct NVTtouch_ts_data, early_suspend);
	NVTtouch_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void NVTtouch_ts_late_resume(struct early_suspend *h)
{
	struct NVTtouch_ts_data *ts;
	ts = container_of(h, struct NVTtouch_ts_data, early_suspend);
	NVTtouch_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id NVTtouch_ts_id[] = {
	{ NVTTOUCH_I2C_NAME, 0 },
	{ }
};

static struct i2c_board_info NVTtouch_i2c_boardinfo = {
	I2C_BOARD_INFO("NVT-ts", 0x01), 
	//.irq = IRQ_EINT(6)
};

static struct i2c_driver NVTtouch_ts_driver = {
	.probe		= NVTtouch_ts_probe,
	.remove		= NVTtouch_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= NVTtouch_ts_suspend,
	.resume		= NVTtouch_ts_resume,
#endif
	.id_table	= NVTtouch_ts_id,
	.driver = {
		.name	= NVTTOUCH_I2C_NAME,
		.owner = THIS_MODULE,
	},
};

int NVT_add_i2c_device(struct i2c_board_info *info)
{
	int err;
	struct i2c_adapter *adapter;

	adapter = i2c_get_adapter(NVT_BUS_NUM);
	if(!adapter)
	{
		printk("%s: can't get i2c adapter %d\n", __func__, NVT_BUS_NUM);
		err=-ENODEV;
		goto err_driver;
	}	

	NVT_client = i2c_new_device(adapter, info);
	if(!NVT_client)
	{
		printk("%s: can't add i2c device at 0x%x\n", __func__, (unsigned int)info->addr);
		err=-ENODEV;
		goto err_driver;
	}	
	i2c_put_adapter(adapter);
	return 0;
err_driver:
	return err;
}


/*******************************************************	
Description:
	Driver Install function.
return:
	Executive Outcomes. 0---succeed.
********************************************************/
static int __devinit NVTtouch_ts_init(void)
{
	int ret;
	
	//NVT_add_i2c_device(&NVTtouch_i2c_boardinfo);
	printk("%s %s %d \n",__FILE__,__func__,__LINE__);

	NVTtouch_wq = create_workqueue("NVTtouch_wq");		//create a work queue and worker thread
	if (!NVTtouch_wq) {
		printk(KERN_ALERT "creat workqueue faiked\n");
		return -ENOMEM;
		
	}
	ret=i2c_add_driver(&NVTtouch_ts_driver);
	return ret; 
}

/*******************************************************	
Description:
	Driver uninstall function.
return:
	Executive Outcomes. 0---succeed.
********************************************************/
static void __exit NVTtouch_ts_exit(void)
{
	printk(KERN_ALERT "Touchscreen driver of guitar exited.\n");
	i2c_del_driver(&NVTtouch_ts_driver);
	i2c_unregister_device(NVT_client);
	if (NVTtouch_wq)
		destroy_workqueue(NVTtouch_wq);		//release our work queue
}

late_initcall(NVTtouch_ts_init);
module_exit(NVTtouch_ts_exit);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
