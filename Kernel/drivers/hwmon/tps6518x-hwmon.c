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
/*
 * tps6518x-hwmon.c
 *
 * Based on the MAX1619 driver.
 * Copyright (C) 2003-2004 Alexey Fisher <fishor@mail.ru>
 *                         Jean Delvare <khali@linux-fr.org>
 *
 * The TPS6518x is a sensor chip made by Texass Instruments.
 * It reports up to two temperatures (its own plus up to
 * one external one).
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/mfd/tps6518x.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/consumer.h>

/*
 * Conversions
 */
static int temp_from_reg(int val)
{
	return val;
}

/*
 * Functions declaration
 */
static int tps6518x_sensor_probe(struct platform_device *pdev);
static int tps6518x_sensor_remove(struct platform_device *pdev);

/*
 * Driver data (common to all clients)
 */
static struct platform_driver tps6518x_sensor_driver = {
	.probe = tps6518x_sensor_probe,
	.remove = tps6518x_sensor_remove,
	.driver = {
		.name = EPDC_PMIC_SENSOR_DRIVE_NAME,
	},
};


/*
 * Client data (each client gets its own)
 */
struct tps6518x_data {
	struct device *hwmon_dev;
};

/*
 * Sysfs stuff
 */
static ssize_t show_temp_input(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct regulator *v3p3_regulator;
	unsigned int reg_val;

	v3p3_regulator = regulator_get(dev, EPDC_PMIC_TPS6518X_REG_V3P3_NAME);
	if(IS_ERR(v3p3_regulator))
		return snprintf(buf, PAGE_SIZE, "Unable to find regulator V3P3\n");
	regulator_enable(v3p3_regulator);
	
	/*
	 * begin Temperature conversion
	 */
	tps6518x_reg_read(REG_TPS6518x_REVID,&reg_val);
	switch (reg_val & 0xff)
	{
	   case TPS65180_PASS1 :
	   case TPS65180_PASS2 :
	   case TPS65181_PASS1 :
	   case TPS65181_PASS2 :
		    reg_val = 0x80;
		    tps6518x_reg_write(REG_TPS65180_TMST_CONFIG, reg_val);
		    // wait for completion completed
		    while ((0x20 & reg_val) == 0)
		    {
		        msleep(1);
		        tps6518x_reg_read(REG_TPS65180_TMST_CONFIG, &reg_val);
		    }
	        break;
	   case TPS65185_PASS0 :
	   case TPS65186_PASS0 :
	   case TPS65185_PASS1 :
	   case TPS65186_PASS1 :
	   case TPS65185_PASS2 :
	   case TPS65186_PASS2 :
		    reg_val = 0x80;
		    tps6518x_reg_write(REG_TPS65185_TMST1, reg_val);
		    // wait for completion completed
		    while ((0x20 & reg_val) == 0)
		    {
		        msleep(1);
		        tps6518x_reg_read(REG_TPS65185_TMST1, &reg_val);
		    }
	        break;
	   default:
		break;	

	}

	tps6518x_reg_read(REG_TPS6518x_TMST_VAL, &reg_val);
	regulator_disable(v3p3_regulator);
	regulator_put(v3p3_regulator);
	return snprintf(buf, PAGE_SIZE, "%d\n", temp_from_reg(reg_val));
}


static DEVICE_ATTR(temp_input, S_IRUGO, show_temp_input, NULL);

static struct attribute *tps6518x_attributes[] = {
	&dev_attr_temp_input.attr,
	NULL
};

static const struct attribute_group tps6518x_group = {
	.attrs = tps6518x_attributes,
};

/*
 * Real code
 */
static int tps6518x_sensor_probe(struct platform_device *pdev)
{
	struct tps6518x_data *data;
	int err;


	data = kzalloc(sizeof(struct tps6518x_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}
	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 

	/* Register sysfs hooks */
	err = sysfs_create_group(&pdev->dev.kobj, &tps6518x_group);
	if (err)
		goto exit_free;

	data->hwmon_dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove_files;
	}

	platform_set_drvdata(pdev, data);

	return 0;

exit_remove_files:
	sysfs_remove_group(&pdev->dev.kobj, &tps6518x_group);
exit_free:
	kfree(data);
exit:
	return err;
}

static int tps6518x_sensor_remove(struct platform_device *pdev)
{
	struct tps6518x_data *data = platform_get_drvdata(pdev);

	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&pdev->dev.kobj, &tps6518x_group);

	kfree(data);
	return 0;
}

static int __init sensors_tps6518x_init(void)
{
	//printk("%s %s %d \n",__FILE__,__func__,__LINE__); 
 	return platform_driver_register(&tps6518x_sensor_driver);
}
module_init(sensors_tps6518x_init);

static void __exit sensors_tps6518x_exit(void)
{
	platform_driver_unregister(&tps6518x_sensor_driver);
}
module_exit(sensors_tps6518x_exit);

MODULE_DESCRIPTION("TPS6518x sensor driver");
MODULE_LICENSE("GPL");

