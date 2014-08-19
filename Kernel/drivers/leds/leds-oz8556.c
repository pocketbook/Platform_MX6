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
 * tps6518x-utils.c
 *
 * Based on the MAX1619 driver.
 * Copyright (C) 2003-2004 Alexey Fisher <fishor@mail.ru>
 *                         Jean Delvare <khali@linux-fr.org>
 *
 * The TPS6518x is a chip made by Texass Instruments.
 * It reports up to two temperatures (its own plus up to
 * one external one).
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/imx6sl_evk/imx6sl_evk_gpio_cfg.h>

static int gpio_chrgoff;
static int gpio_led;
static int gpio_chrgoff_state=-1;
/*
 * Client data (each client gets its own)
 */
struct oz8556_data {
	struct device *misc_dev;
};

/*
 * Sysfs stuff
 */
static ssize_t show_chrgoff(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	return snprintf(buf, PAGE_SIZE, "%d\n", gpio_chrgoff_state);

}

static ssize_t set_chrgoff(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	int chrgvalue = simple_strtol(buf,NULL,0);
	//printk("%s %s %d gpio_chrgoff_stat=%d,chrgvalue=%d  \n",__FILE__,__func__,__LINE__,gpio_chrgoff_state,chrgvalue); 
	if (gpio_chrgoff_state == -1)
	   printk(KERN_ERR "pwrctrl2 pin is not initialized!\n");
	else{
	   gpio_set_value(gpio_chrgoff, (gpio_chrgoff_state = chrgvalue));
	   gpio_set_value(gpio_led, (gpio_chrgoff_state = chrgvalue));
	}

	return count;
}


static int init_chrgoff(void)
{
	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 
	gpio_chrgoff = MX6SL_FEC_TX_EN_CHRG_OFF;
	gpio_led=MX6SL_EPDC_PWRCTRL1_SYSTEM_LED;
	//gpio_request(gpio_chrgoff, "chrgoff");
	gpio_set_value(gpio_chrgoff, (gpio_chrgoff_state = 0));
	gpio_set_value(gpio_led, (gpio_chrgoff_state = 0));
	

       return 0;
}
static DEVICE_ATTR(gpio_chrgoff, S_IWUSR | S_IRUGO, show_chrgoff, set_chrgoff);

static struct attribute *oz8556_attributes[] = {
	&dev_attr_gpio_chrgoff.attr,
	NULL
};

static const struct attribute_group oz8556_group = {
	.attrs = oz8556_attributes,
};



/*
 * Real code
 */
static int oz8556_probe(struct platform_device *pdev)
{
	struct oz8556_data *data;
	int err;
	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 

	data = kzalloc(sizeof(struct oz8556_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	init_chrgoff();

	/* Register sysfs hooks */
	err = sysfs_create_group(&pdev->dev.kobj, &oz8556_group);
	if (err)
		goto exit_free;

	platform_set_drvdata(pdev, data);

	return 0;

exit_free:
	kfree(data);
exit:
	return err;
}

static int oz8556_remove(struct platform_device *pdev)
{
	struct oz8556_data *data = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	sysfs_remove_group(&pdev->dev.kobj, &oz8556_group);

	kfree(data);
	return 0;
}

/*
 * Driver data (common to all clients)
 */
static struct platform_driver oz8556_leds_driver = {
	.probe = oz8556_probe,
	.remove = oz8556_remove,
	.driver = {
		.name = "oz8556-leds",
	},
};

static int __init oz8565_init(void)
{
 	return platform_driver_register(&oz8556_leds_driver);
}
module_init(oz8565_init);

static void __exit oz8565_exit(void)
{
	platform_driver_unregister(&oz8556_leds_driver);
}
module_exit(oz8565_exit);

MODULE_DESCRIPTION("oz8556 leds driver");
MODULE_LICENSE("GPL");

