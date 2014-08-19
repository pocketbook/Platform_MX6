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

static int gpio_flashledctl;
static int gpio_torchctl;
static int gpio_flashledctl_state=-1;
static int gpio_torchctl_state=-1;
/*
 * Client data (each client gets its own)
 */
struct sgm3140b_data {
	struct device *misc_dev;
};

/*
 * Sysfs stuff
 */
static ssize_t show_power(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	return snprintf(buf, PAGE_SIZE, "%d\n", gpio_flashledctl_state);

}

static ssize_t set_power(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	int chrgvalue = simple_strtol(buf,NULL,0);
	//printk("%s %s %d gpio_flashledctl_state=%d,chrgvalue=%d  \n",__FILE__,__func__,__LINE__,gpio_flashledctl_state,chrgvalue); 
	if (gpio_flashledctl_state == -1)
	   printk(KERN_ERR "gpio_flashledctl_state pin is not initialized!\n");
	else{
	   gpio_set_value(gpio_flashledctl, (gpio_flashledctl_state = chrgvalue));
	}

	return count;
}
static ssize_t show_torch(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	return snprintf(buf, PAGE_SIZE, "%d\n", gpio_torchctl_state);

}

static ssize_t set_torch(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	int chrgvalue = simple_strtol(buf,NULL,0);
	//printk("%s %s %d gpio_torchctl_state=%d,chrgvalue=%d  \n",__FILE__,__func__,__LINE__,gpio_torchctl_state,chrgvalue); 
	if (gpio_flashledctl_state == -1)
	   printk(KERN_ERR "gpio_torchctl_state pin is not initialized!\n");
	else{
	   gpio_set_value(gpio_torchctl, (gpio_torchctl_state = chrgvalue));
	}

	return count;
}

static int init_sgm3140b_gpio(void)
{
	int ret = 0;
	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 
	gpio_flashledctl = MX6SL_CAMERA_LED_CTL;
	gpio_torchctl=MX6SL_CAMERA_TORCH_CTL;
	/*Camera  FLASH_LED_CTL*/
	ret = gpio_request(gpio_flashledctl, "cam-led-ctl");	 
	if (ret) {	   
		printk("failed to request cam-led-ctl  int  gpio\n");	
		return ret;	
	}	
	gpio_direction_output(gpio_flashledctl, 0);
	gpio_set_value(gpio_flashledctl, (gpio_flashledctl_state = 0));

		/*Camera TORCH_CTL*/
	ret = gpio_request(gpio_torchctl, "cam-torch");	 
	if (ret) {	   
		printk("failed to request cam-torch  int  gpio\n");	
		return ret;	
	}	
	gpio_direction_output(gpio_torchctl, 0);
	gpio_set_value(gpio_torchctl, (gpio_torchctl_state =0));		
	

       return 0;
}
//FLASH_LED_CTL
static DEVICE_ATTR(gpio_flashled_ctl, S_IWUGO | S_IRUGO, show_power, set_power);
//TORCH_CTL
static DEVICE_ATTR(gpio_torch_ctl, S_IWUGO | S_IRUGO, show_torch, set_torch);

static struct attribute *sgm3140b_attributes[] = {
	&dev_attr_gpio_flashled_ctl.attr,
	&dev_attr_gpio_torch_ctl.attr,
	NULL
};

static const struct attribute_group sgm3140b_group = {
	.attrs = sgm3140b_attributes,
};



/*
 * Real code
 */
static int sgm3140b_probe(struct platform_device *pdev)
{
	struct sgm3140b_data *data;
	int err;
	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 

	data = kzalloc(sizeof(struct sgm3140b_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	init_sgm3140b_gpio();

	/* Register sysfs hooks */
	err = sysfs_create_group(&pdev->dev.kobj, &sgm3140b_group);
	if (err)
		goto exit_free;

	platform_set_drvdata(pdev, data);

	return 0;

exit_free:
	kfree(data);
exit:
	return err;
}

static int sgm3140b_remove(struct platform_device *pdev)
{
	struct sgm3140b_data *data = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	sysfs_remove_group(&pdev->dev.kobj, &sgm3140b_group);

	kfree(data);
	return 0;
}

/*
 * Driver data (common to all clients)
 */
static struct platform_driver sgm3140b_driver = {
	.probe = sgm3140b_probe,
	.remove = sgm3140b_remove,
	.driver = {
		.name = "sgm3140b",
	},
};

static int __init sgm3140b_init(void)
{
 	return platform_driver_register(&sgm3140b_driver);
}
module_init(sgm3140b_init);

static void __exit sgm3140b_exit(void)
{
	platform_driver_unregister(&sgm3140b_driver);
}
module_exit(sgm3140b_exit);

MODULE_DESCRIPTION("oz8556 leds driver");
MODULE_LICENSE("GPL");

