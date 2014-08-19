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
#include <linux/mfd/tps6518x.h>
#include <linux/gpio.h>
//#include <linux/imx6sl/mx6sl_gpio_cfg.h>
#include <linux/imx6sl_evk/imx6sl_evk_gpio_cfg.h>

////kernel_imx_alpha/drivers/misc
///


#define EPDC_PWRCTRL0  MX6SL_BRD_EPDC_PWRCTRL0	
#define EPDC_PWRCTRL1  MX6SL_BRD_EPDC_PWRCTRL1	
#define EPDC_PWRCTRL2  MX6SL_BRD_EPDC_PWRCTRL2	
#define EPDC_PWRWAKE  MX6SL_BRD_EPDC_PMIC_WAKE
#define EPDC_VCOM MX6SL_BRD_EPDC_VCOM/*MX6SL_ARM2_EPDC_PWRCOM*/

static int gpio_pwrctl2;
static int gpio_pwrctl2_state=-1;

extern int tps65180_current_Enable_Register;
/*
 * Conversions
 */
/*
 * Functions declaration
 */
static int tps6518x_utils_probe(struct platform_device *pdev);
static int tps6518x_utils_remove(struct platform_device *pdev);

/*
 * Driver data (common to all clients)
 */
static struct platform_driver tps6518x_utils_driver = {
	.probe = tps6518x_utils_probe,
	.remove = tps6518x_utils_remove,
	.driver = {
		.name = EPDC_PMIC_UTILS_DRIVE_NAME,
	},
};

static int tps6518x_utils_wakeup_enable(void)
{
	gpio_set_value(EPDC_PWRWAKE, 1);
	msleep(2);

	return 0;

}
static int tps6518x_utils_wakeup_disable(void)
{

	msleep(21);

	///make sure the tps6518x enter sleep mode:
	gpio_set_value(EPDC_PWRWAKE, 0);

	return 0;
}

/*
 * Client data (each client gets its own)
 */
struct tps6518x_data {
	struct device *misc_dev;
};

/*
 * Sysfs stuff
 */
static ssize_t show_pwr_regs(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int reg_val;
	tps6518x_utils_wakeup_enable();
	/*
	 * report the status register value
	 */
	tps6518x_reg_read(REG_TPS6518x_PG,&reg_val);

	tps6518x_utils_wakeup_disable();
	return snprintf(buf, PAGE_SIZE, "0x%02x\n", reg_val);
}

static ssize_t show_intr_regs(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int reg_val;
	unsigned int intr_reg_val;
	tps6518x_utils_wakeup_enable();
	/*
	 * get the interrupt status register value
	 */
	tps6518x_reg_read(REG_TPS6518x_REVID,&reg_val);
	switch (reg_val & 0xff)
	{
	   case TPS65180_PASS1 :
	   case TPS65180_PASS2 :
	   case TPS65181_PASS1 :
	   case TPS65181_PASS2 :
		    tps6518x_reg_read(REG_TPS65180_INT1, &intr_reg_val);
		    tps6518x_reg_read(REG_TPS65180_INT2, &reg_val);
		    intr_reg_val |= reg_val<<8;
	        break;
	   case TPS65185_PASS0 :
	   case TPS65186_PASS0 :
	   case TPS65185_PASS1 :
	   case TPS65186_PASS1 :
	   case TPS65185_PASS2 :
	   case TPS65186_PASS2 :
		    tps6518x_reg_read(REG_TPS65185_INT1, &intr_reg_val);
		    tps6518x_reg_read(REG_TPS65185_INT2, &reg_val);
		    intr_reg_val |= reg_val<<8;
	        break;
	   default:
		break;	

	}

	tps6518x_utils_wakeup_disable();
	return snprintf(buf, PAGE_SIZE, "%d\n", intr_reg_val);
}

static ssize_t show_vcom(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int reg_val;
	unsigned int vcom_reg_val;
	
	tps6518x_utils_wakeup_enable();
	/*
	 * get the vcom registers
	 */
	tps6518x_reg_read(REG_TPS6518x_REVID,&reg_val);
	switch (reg_val & 0xff)
	{
	   case TPS65180_PASS1 :
	   case TPS65180_PASS2 :
	   case TPS65181_PASS1 :
	   case TPS65181_PASS2 :
	   	   tps6518x_reg_read(REG_TPS65180_VCOM_ADJUST, &vcom_reg_val);
	        break;
	   case TPS65185_PASS0 :
	   case TPS65186_PASS0 :
	   case TPS65185_PASS1 :
	   case TPS65186_PASS1 :
	   case TPS65185_PASS2 :
	   case TPS65186_PASS2 :
		    tps6518x_reg_read(REG_TPS65185_VCOM1, &vcom_reg_val);
		    tps6518x_reg_read(REG_TPS65185_VCOM2, &reg_val);
		    vcom_reg_val |= reg_val<<8;
	        break;
	   default:
		break;	

	}

	//tps6518x_utils_wakeup_disable();
	return snprintf(buf, PAGE_SIZE, "%d\n", vcom_reg_val);
}

static ssize_t set_vcom(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	unsigned int reg_val;
	tps6518x_utils_wakeup_enable();

	long vcom_reg_val = simple_strtol(buf,NULL,0);
	/*
	 * update the vcom register value
	 */
	 vcom_reg_val = vcom_reg_val + 0x4000;
	tps6518x_reg_read(REG_TPS6518x_REVID,&reg_val);
	switch (reg_val & 0xff)
	{
	   case TPS65180_PASS1 :
	   case TPS65180_PASS2 :
	   case TPS65181_PASS1 :
	   case TPS65181_PASS2 :
		    tps6518x_reg_write(REG_TPS65180_VCOM_ADJUST, vcom_reg_val&0xff);
	        break;
	   case TPS65185_PASS0 :
	   case TPS65186_PASS0 :
	   case TPS65185_PASS1 :
	   case TPS65186_PASS1 :
	   case TPS65185_PASS2 :
	   case TPS65186_PASS2 :
		    tps6518x_reg_write(REG_TPS65185_VCOM1, vcom_reg_val&0xff);
		    tps6518x_reg_write(REG_TPS65185_VCOM2, (vcom_reg_val>>8)&0xff);
	        break;
	   default:
		break;	

	}

	return count;
}

static ssize_t show_enable(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int reg_val;
	tps6518x_utils_wakeup_enable();
	/*
	 * get the enable registers
	 */
	tps6518x_reg_read(REG_TPS65185_ENABLE,&reg_val);
	tps6518x_utils_wakeup_disable();

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", reg_val);
}

static ssize_t set_enable(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	long enable_reg_val = simple_strtol(buf,NULL,0);
	tps6518x_utils_wakeup_enable();
	/*
	 * update the ENABLE register
	 */
	tps6518x_reg_write(REG_TPS65185_ENABLE, enable_reg_val);
	tps6518x_utils_wakeup_disable();

	return count;
}

static ssize_t show_upseq(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int reg_val;
	unsigned long upseq_reg_val=0l;
	tps6518x_utils_wakeup_enable();
	/*
	 * get the upseq registers
	 */
	tps6518x_reg_read(REG_TPS6518x_REVID,&reg_val);
	switch (reg_val & 0xff)
	{
	   case TPS65180_PASS1 :
	   case TPS65180_PASS2 :
	   case TPS65181_PASS1 :
	   case TPS65181_PASS2 :
		    tps6518x_reg_read(REG_TPS65180_PWRSEQ0, &reg_val);
		    upseq_reg_val = reg_val << 16;
		    tps6518x_reg_read(REG_TPS65180_PWRSEQ1, &reg_val);
		    upseq_reg_val |= reg_val << 8;
		    tps6518x_reg_read(REG_TPS65180_PWRSEQ2, &reg_val);
		    upseq_reg_val |= reg_val;
	        break;
	   case TPS65185_PASS0 :
	   case TPS65186_PASS0 :
	   case TPS65185_PASS1 :
	   case TPS65186_PASS1 :
	   case TPS65185_PASS2 :
	   case TPS65186_PASS2 :
		    tps6518x_reg_read(REG_TPS65185_UPSEQ0, &reg_val);
		    upseq_reg_val = reg_val << 8;
		    tps6518x_reg_read(REG_TPS65185_UPSEQ1, &reg_val);
		    upseq_reg_val |= reg_val;
	        break;
	   default:
		break;	

	}

	tps6518x_utils_wakeup_disable();
	return snprintf(buf, PAGE_SIZE, "0x%06lx\n", upseq_reg_val);
}

static ssize_t set_upseq(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	unsigned int reg_val;
	long upseq_reg_val = simple_strtol(buf,NULL,0);
	tps6518x_utils_wakeup_enable();
	/*
	 * update the power up sequence registers
	 */
	tps6518x_reg_read(REG_TPS6518x_REVID,&reg_val);
	switch (reg_val & 0xff)
	{
	   case TPS65180_PASS1 :
	   case TPS65180_PASS2 :
	   case TPS65181_PASS1 :
	   case TPS65181_PASS2 :
		    tps6518x_reg_write(REG_TPS65180_PWRSEQ0, (upseq_reg_val >>16) & 0xff);
		    tps6518x_reg_write(REG_TPS65180_PWRSEQ1, (upseq_reg_val >>8) & 0xff);
		    tps6518x_reg_write(REG_TPS65180_PWRSEQ2, (upseq_reg_val) & 0xff);
	        break;
	   case TPS65185_PASS0 :
	   case TPS65186_PASS0 :
	   case TPS65185_PASS1 :
	   case TPS65186_PASS1 :
	   case TPS65185_PASS2 :
	   case TPS65186_PASS2 :
		    tps6518x_reg_write(REG_TPS65185_UPSEQ0, (upseq_reg_val >>8) & 0xff);
		    tps6518x_reg_write(REG_TPS65185_UPSEQ1, (upseq_reg_val) & 0xff);
	        break;
	   default:
		break;	

	}

	tps6518x_utils_wakeup_disable();
	return count;
}

static ssize_t show_dwnseq(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int reg_val;
	unsigned long dwnseq_reg_val=0l;
	tps6518x_utils_wakeup_enable();
	/*
	 * get the power down registers
	 */
	tps6518x_reg_read(REG_TPS6518x_REVID,&reg_val);
	switch (reg_val & 0xff)
	{
	   case TPS65180_PASS1 :
	   case TPS65180_PASS2 :
	   case TPS65181_PASS1 :
	   case TPS65181_PASS2 :
		    printk(KERN_ERR "Warning: This is the same as pmic_upseq register values.\n");
		    tps6518x_reg_read(REG_TPS65180_PWRSEQ0, &reg_val);
		    dwnseq_reg_val = reg_val << 16;
		    tps6518x_reg_read(REG_TPS65180_PWRSEQ1, &reg_val);
		    dwnseq_reg_val |= reg_val << 8;
		    tps6518x_reg_read(REG_TPS65180_PWRSEQ2, &reg_val);
		    dwnseq_reg_val |= reg_val;
	        break;
	   case TPS65185_PASS0 :
	   case TPS65186_PASS0 :
	   case TPS65185_PASS1 :
	   case TPS65186_PASS1 :
	   case TPS65185_PASS2 :
	   case TPS65186_PASS2 :
		    tps6518x_reg_read(REG_TPS65185_DWNSEQ0, &reg_val);
		    dwnseq_reg_val = reg_val << 8;
		    tps6518x_reg_read(REG_TPS65185_DWNSEQ1, &reg_val);
		    dwnseq_reg_val |= reg_val;
	        break;
	   default:
		break;	

	}

	tps6518x_utils_wakeup_disable();
	return snprintf(buf, PAGE_SIZE, "0x%06lx\n", dwnseq_reg_val);
}

static ssize_t set_dwnseq(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	unsigned int reg_val;
	long dwnseq_reg_val = simple_strtol(buf,NULL,0);
	tps6518x_utils_wakeup_enable();
	/*
	 * update the power down sequencing registers
	 */
	tps6518x_reg_read(REG_TPS6518x_REVID,&reg_val);
	switch (reg_val & 0xff)
	{
	   case TPS65180_PASS1 :
	   case TPS65180_PASS2 :
	   case TPS65181_PASS1 :
	   case TPS65181_PASS2 :
		    printk(KERN_ERR "Please use pmic_upseq attr to change power sequencing.\n");
	        break;
	   case TPS65185_PASS0 :
	   case TPS65186_PASS0 :
	   case TPS65185_PASS1 :
	   case TPS65186_PASS1 :
	   case TPS65185_PASS2 :
	   case TPS65186_PASS2 :
		    tps6518x_reg_write(REG_TPS65185_DWNSEQ0, (dwnseq_reg_val >>8) & 0xff);
		    tps6518x_reg_write(REG_TPS65185_DWNSEQ1, (dwnseq_reg_val) & 0xff);
	        break;
	   default:
		break;	

	}

	tps6518x_utils_wakeup_disable();
	return count;
}

static ssize_t set_vcom_ctrl(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	long vcomCtrl = simple_strtol(buf,NULL,0);
	tps6518x_utils_wakeup_enable();

	gpio_set_value(EPDC_VCOM, vcomCtrl);
	tps6518x_utils_wakeup_disable();

	return count;
}

static ssize_t show_pwrctrl2(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	return snprintf(buf, PAGE_SIZE, "%d\n", gpio_pwrctl2_state);

}

static ssize_t set_pwrctrl2(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	long pwrctrl2 = simple_strtol(buf,NULL,0);

	if (gpio_pwrctl2_state == -1)
	   printk(KERN_ERR "pwrctrl2 pin is not initialized!\n");
	else
	   gpio_set_value(gpio_pwrctl2, (gpio_pwrctl2_state = pwrctrl2));

	return count;
}

static ssize_t init_pwrctrl2(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{

	gpio_pwrctl2 = EPDC_PWRCTRL2;
	gpio_request(gpio_pwrctl2, "epdc-pwrctrl2");
	gpio_direction_output(gpio_pwrctl2, (gpio_pwrctl2_state = 1));

	return count;
}

static DEVICE_ATTR(pwr_status, S_IRUGO, show_pwr_regs, NULL);
static DEVICE_ATTR(intr_input, S_IRUGO, show_intr_regs, NULL);
static DEVICE_ATTR(vcom_value, S_IWUSR | S_IRUGO, show_vcom, set_vcom);
static DEVICE_ATTR(pmic_enable, S_IWUSR | S_IRUGO, show_enable, set_enable);
static DEVICE_ATTR(vcom_ctrl, S_IWUSR, NULL, set_vcom_ctrl);
static DEVICE_ATTR(pmic_upseq, S_IWUSR | S_IRUGO, show_upseq, set_upseq);
static DEVICE_ATTR(pmic_dwnseq, S_IWUSR | S_IRUGO, show_dwnseq, set_dwnseq);
static DEVICE_ATTR(gpio_pwrctrl2, S_IWUSR | S_IRUGO, show_pwrctrl2, set_pwrctrl2);
static DEVICE_ATTR(init_pwrctrl2, S_IWUSR,  NULL, init_pwrctrl2);

static struct attribute *tps6518x_attributes[] = {
	&dev_attr_pwr_status.attr,
	&dev_attr_intr_input.attr,
	&dev_attr_vcom_value.attr,
	&dev_attr_pmic_enable.attr,
	&dev_attr_vcom_ctrl.attr,
	&dev_attr_pmic_upseq.attr,
	&dev_attr_pmic_dwnseq.attr,
	&dev_attr_gpio_pwrctrl2.attr,
	&dev_attr_init_pwrctrl2.attr,
	NULL
};

static const struct attribute_group tps6518x_group = {
	.attrs = tps6518x_attributes,
};

/*
 * Real code
 */
static int tps6518x_utils_probe(struct platform_device *pdev)
{
	struct tps6518x_data *data;
	int err;
	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 

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

	platform_set_drvdata(pdev, data);
	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 

	return 0;

exit_free:
	kfree(data);
exit:
	return err;
}

static int tps6518x_utils_remove(struct platform_device *pdev)
{
	struct tps6518x_data *data = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	sysfs_remove_group(&pdev->dev.kobj, &tps6518x_group);

	kfree(data);
	return 0;
}

static int __init utilss_tps6518x_init(void)
{
 	return platform_driver_register(&tps6518x_utils_driver);
}
module_init(utilss_tps6518x_init);

static void __exit utilss_tps6518x_exit(void)
{
	platform_driver_unregister(&tps6518x_utils_driver);
}
module_exit(utilss_tps6518x_exit);

MODULE_DESCRIPTION("TPS6518x utils driver");
MODULE_LICENSE("GPL");

