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

#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/obreey.h>

struct sysled_data {
	int (*control)(bool);
	int (*state)(void);
};

static ssize_t sysled_show_chrgoff(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sysled_data *data = dev_get_drvdata(dev);

	pr_debug("[%s] Return charger state %d\n", __func__, data->state());

	return snprintf(buf, PAGE_SIZE, "%d\n", data->state());
}

static ssize_t sysled_store_chrgoff(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct sysled_data *data = dev_get_drvdata(dev);
//	int value = simple_strtol(buf, NULL, 10);
	int value = kstrtol(buf, NULL, 10);

	pr_debug("[%s] Storing value %d\n", __func__, value);

	data->control(!!value);

	return count;
}

static DEVICE_ATTR(chrgoff, S_IWUSR | S_IRUGO, sysled_show_chrgoff, sysled_store_chrgoff);

static struct attribute *sysled_attributes[] = {
	&dev_attr_chrgoff.attr,
	NULL
};

static const struct attribute_group sysled_group = {
	.attrs = sysled_attributes,
};

static int sysled_probe(struct platform_device *pdev)
{
	struct system_led_platform_data *pdata = pdev->dev.platform_data;
	struct sysled_data *data = platform_get_drvdata(pdev);
	int err = 0;

	pr_debug("[%s] probing driver\n", __func__);

	if (!pdata)
		return -EINVAL;

	data = kzalloc(sizeof(struct sysled_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	if (pdata->init) {
		if (pdata->init() != 0) {
			pr_err("[%s] Failed to init system led I/O\n", __func__);
			goto exit_free;
		}
	} else {
		pr_warn("[%s] No led init function was provided\n", __func__);
	}

	if (pdata->control && pdata->state) {
		data->control = pdata->control;
		data->state = pdata->state;
	} else {
		pr_warn("[%s] No way to control led state\n", __func__);
	}

	/* Turn system led on */
	data->control(true);

	/* Register sysfs hooks */
	err = sysfs_create_group(&pdev->dev.kobj, &sysled_group);
	if (err)
		goto exit_free;

	platform_set_drvdata(pdev, data);

	return 0;

exit_free:
	kfree(data);
exit:
	return err;
}

static int sysled_remove(struct platform_device *pdev)
{
	struct sysled_data *data = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	sysfs_remove_group(&pdev->dev.kobj, &sysled_group);

	kfree(data);
	return 0;
}

static int sysled_suspend(struct device *dev)
{
	struct sysled_data *data = dev_get_drvdata(dev);

	data->control(false);

	return 0;
}

static int sysled_resume(struct device *dev)
{
	struct sysled_data *data = dev_get_drvdata(dev);

	data->control(true);

	return 0;
}

static SIMPLE_DEV_PM_OPS(sysled_pm, sysled_suspend, sysled_resume);

/*
 * Driver data (common to all clients)
 */
static struct platform_driver sysled_leds_driver = {
	.probe = sysled_probe,
	.remove = sysled_remove,
	.driver = {
		.name = "system-leds",
		.pm = &sysled_pm,
	},
};

static int __init sysled_init(void)
{
	return platform_driver_register(&sysled_leds_driver);
}

static void __exit sysled_exit(void)
{
	platform_driver_unregister(&sysled_leds_driver);
}

module_init(sysled_init);
module_exit(sysled_exit);

MODULE_DESCRIPTION("PB650 system leds driver");
MODULE_LICENSE("GPL");

