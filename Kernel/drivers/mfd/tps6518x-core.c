/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

/*!
 * @file pmic/core/tps6518x.c
 * @brief This file contains TPS6518x specific PMIC code. This implementaion
 * may differ for each PMIC chip.
 *
 * @ingroup PMIC_CORE
 */

/*
 * Includes
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/uaccess.h>

#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/pmic_status.h>
#include <linux/mfd/tps6518x.h>
#include <asm/mach-types.h>

struct i2c_client *tps6518x_client;

static const unsigned short tps6518x_normal_i2c[] = {EPDC_PMIC_I2C_ADDR, I2C_CLIENT_END};

int tps6518x_reg_read(int reg_num, unsigned int *reg_val)
{
	int result;

	if (tps6518x_client == NULL)
		return PMIC_ERROR;
	//printk("%s %s %d \n",__FILE__,__func__,__LINE__); 
	result = i2c_smbus_read_byte_data(tps6518x_client, reg_num);
	//result = i2c_smbus_write_byte(tps6518x_client, reg_num);
	if (result < 0) {
		dev_err(&tps6518x_client->dev,
			"Unable to read tps6518x register via I2C\n");
		return PMIC_ERROR;
	}

	*reg_val = result;
	return PMIC_SUCCESS;
}

int tps6518x_reg_write(int reg_num, const unsigned int reg_val)
{
	int result;

	if (tps6518x_client == NULL)
		return PMIC_ERROR;
	//printk("%s %s %d \n",__FILE__,__func__,__LINE__); 
	result = i2c_smbus_write_byte_data(tps6518x_client, reg_num, reg_val);
	if (result < 0) {
		dev_err(&tps6518x_client->dev,
			"Unable to write TPS6518x register via I2C\n");
		return PMIC_ERROR;
	}

	return PMIC_SUCCESS;
}

static int tps6518x_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct tps6518x *tps6518x;
	struct tps6518x_platform_data *pdata = client->dev.platform_data;
	int ret = 0;

	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 

	if (!pdata || !pdata->init)
		return -ENODEV;

	/* Create the PMIC data structure */
	tps6518x = kzalloc(sizeof(struct tps6518x), GFP_KERNEL);
	if (tps6518x == NULL) {
		kfree(client);
		return -ENOMEM;
	}

	/* Initialize the PMIC data structure */
	i2c_set_clientdata(client, tps6518x);
	tps6518x->dev = &client->dev;
	tps6518x->i2c_client = client;

	tps6518x_client = client;

	if (pdata && pdata->init) {
		ret = pdata->init(tps6518x);
		if (ret != 0)
			goto err;
	}
     
       tps6518x_reg_write(REG_TPS65185_ENABLE, 0x3f);
     
	dev_info(&client->dev, "PMIC TPS6518x for eInk display \n");
	

	return ret;
err:
	kfree(tps6518x);

	return ret;
}


static int tps6518x_remove(struct i2c_client *i2c)
{
	struct tps6518x *tps6518x = i2c_get_clientdata(i2c);
	int i;

	for (i = 0; i < ARRAY_SIZE(tps6518x->pdev); i++)
		platform_device_unregister(tps6518x->pdev[i]);

	kfree(tps6518x);

	return 0;
}

static int tps6518x_suspend(struct i2c_client *client, pm_message_t state)
{
	return 0;
}

static int tps6518x_resume(struct i2c_client *client)
{
	return 0;
}

/* Return 0 if detection is successful, -ENODEV otherwise */
static int tps6518x_detect(struct i2c_client *client,
			  struct i2c_board_info *info)
{
	//struct tps6518x_platform_data *pdata = client->dev.platform_data;
	struct i2c_adapter *adapter = client->adapter;
	u8 revId;


	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	/* detection */
	if (i2c_smbus_read_byte_data(client,
		REG_TPS6518x_REVID) != 0) {
		dev_err(&adapter->dev,
			"TPS6518x PMIC not found!\n");
		return -ENODEV;
	}

	/* identification */
	revId = i2c_smbus_read_byte_data(client,
		  REG_TPS6518x_REVID);

	/*
	 * Known rev-ids
	 * tps165180 pass 1 = 0x50, tps65180 pass2 = 0x60, tps65181 pass1 = 0x51, tps65181 pass2 = 0x61, 
	 * tps65182, 
	 * tps65185 pass0 = 0x45, tps65186 pass0 0x46, tps65185 pass1 = 0x55, tps65186 pass1 0x56, tps65185 pass2 = 0x65, tps65186 pass2 0x66
	 */
	if (!((revId == TPS65180_PASS1) ||
		 (revId == TPS65181_PASS1) ||
		 (revId == TPS65180_PASS2) ||
		 (revId == TPS65181_PASS2) ||
		 (revId == TPS65185_PASS0) ||
		 (revId == TPS65186_PASS0) ||
		 (revId == TPS65185_PASS1) ||
		 (revId == TPS65186_PASS1) ||
		 (revId == TPS65185_PASS2) ||
		 (revId == TPS65186_PASS2)))
	{
		dev_info(&adapter->dev,
		    "Unsupported chip (Revision ID=0x%02X).\n",  revId);
		return -ENODEV;
	}

	//strlcpy(info->type, "tps6518x_sensor", I2C_NAME_SIZE);
	strlcpy(info->type,EPDC_PMIC_SENSOR_DRIVE_NAME, I2C_NAME_SIZE);

	return 0;
}

static const struct i2c_device_id tps6518x_id[] = {
       { EPDC_PMIC_DRIVE_NAME, 0 },
       { }
};
MODULE_DEVICE_TABLE(i2c, tps6518x_id);


static struct i2c_driver tps6518x_driver = {
	.driver = {
		   .name = EPDC_PMIC_DRIVE_NAME,
		   .owner = THIS_MODULE,
	},
	.probe = tps6518x_probe,
	.remove = tps6518x_remove,
	.suspend = tps6518x_suspend,
	.resume = tps6518x_resume,
	.id_table = tps6518x_id,
	.detect = tps6518x_detect,
	.address_list = &tps6518x_normal_i2c[0],
};

static int __init tps6518x_init(void)
{
	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 
	return i2c_add_driver(&tps6518x_driver);
}

static void __exit tps6518x_exit(void)
{
	i2c_del_driver(&tps6518x_driver);
}

/*
 * Module entry points
 */
subsys_initcall_sync(tps6518x_init);
module_exit(tps6518x_exit);
