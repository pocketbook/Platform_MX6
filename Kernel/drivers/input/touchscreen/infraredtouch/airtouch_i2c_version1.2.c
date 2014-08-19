/*
 * AvatarSemi's airTouch with I2C interface
 *
 * Copyright (C) 2012, Avatar Semiconductor Inc.
 *	luwei <wlu@avatarsemi.com>
 *      minzutao <ztmin@avatarsemi.com> led_adc_info support
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/bcd.h>

#define AIRTOUCH_DRIVER_NAME		"airtouch_i2c"

#define EVENT_PENDOWN 0
#define EVENT_PENMOVE 1
#define EVENT_PENUP   2

#define CMD_RESP_OK	0
#define CMD_TIMEOUT	(2 * HZ)

#define CMD_Deactivate			0x00	/* Arg 0, Res 1 */
#define CMD_Activate			0x01	/* Arg 0, Res 1 */
#define CMD_SetResolution		0x02	/* Arg 4, Res 1 */
#define CMD_SetConfiguration		0x03	/* Arg 4, Res 1 */
#define CMD_GetCoordinate		0x04	/* Arg 0, Res 9*n */
#define CMD_SetScanFreq			0x08	/* Arg 2, Res 1 */
#define CMD_GetVersion			0x0A	/* Arg 0, Res 8 */
#define CMD_FixedPulseStrength		0x0F	/* Arg 1, Res 2+x+y */
#define CMD_GetStatus			0x1E	/* Arg 0, Res 60 */

#define FRAME_START		0xEE

/*
 * touchpad Attention line is Active Low and Open Drain,
 * therefore should be connected to pulled up line
 */

static int width = 600;
module_param(width, int, 0644);
MODULE_PARM_DESC(width, "Screen width in pixel. Default = 600");

static int height = 800;
module_param(height, int, 0644);
MODULE_PARM_DESC(height, "Screen height in pixel. Default = 800");

static int rotation = 90;
module_param(rotation, int, 0644);
MODULE_PARM_DESC(rotation, "Screen rotation angle. Default = 0");

static int data_rate = 60;
module_param(data_rate, int, 0644);
MODULE_PARM_DESC(date_rate, "Report data rate in HZ. Default = 60");

static int pen_size = 31;
module_param(pen_size, int, 0644);
MODULE_PARM_DESC(pen_size, "Maximum pen size in mm. Default = 31");

static int mt = 2;////2
module_param(mt, int, 0644);
MODULE_PARM_DESC(mt, "Support multiply touch. Default = 0 (no mt), 1 (serialized mt), 2 (Android mt), 3 (Qt mt)");

#ifdef CONFIG_ARCH_AT91
static int config = 0x10000;	/* interrupt is edge sensitive */
#else
static int config;
#endif
module_param(config, int, 0644);
MODULE_PARM_DESC(alg0, "Main configuration. Default = 0");

static int alt0 = 0;
module_param(alt0, int, 0644);
MODULE_PARM_DESC(alg0, "Alternate configuration 0. Default = 0");

static int alt1 = 0;
module_param(alt1, int, 0644);
MODULE_PARM_DESC(alg0, "Alternate configuration 1. Default = 0");

#define DEBUG_LVL_DATA 1
#define DEBUG_LVL_MSG 1
#define DEBUG_LVL_IRQ  1
#define DEBUG_LVL_DRV  1
static int AirtouchDebug = 1;
module_param(AirtouchDebug, int, 0644);
MODULE_PARM_DESC(AirtouchDebug, "Set AirtouchDebug level. Default = 0, no AirtouchDebug");

#define PAYLOAD_MAX_SIZE 256
/* The main device structure */
struct airtouch_i2c {
	struct i2c_client	*client;
	struct input_dev	*input;
	struct delayed_work	dwork;
	int			irq;
	spinlock_t		lock;
	struct completion	cmd_done;
	struct completion	startup_done;
	u8			resp_activate;
	u8			resp_resolution;
	u8			resp_scanrate;
	u8			payload[PAYLOAD_MAX_SIZE];
};

static int airtouch_activate(struct airtouch_i2c *touch, u8 act)
{
	struct i2c_client *client = touch->client;
	int ret;

	//printk("%s %s %d  \n",__FILE__,__func__,__LINE__);

	ret = i2c_smbus_write_byte(client, act);
	if (ret < 0) {
		dev_err(&client->dev, "Unable %s activate device\n",
			act ? "" : "in");
		return ret;
	}

	return 0;
}

static int airtouch_set_resolution(struct airtouch_i2c *touch)
{
	struct i2c_client *client = touch->client;
	int ret;
	u8 buf[5] = { CMD_SetResolution, 0, 0, 0, 0 };

	memcpy(buf + 1, &width, 2);
	memcpy(buf + 3, &height, 2);
	ret = i2c_master_send(client, buf, sizeof(buf));
	if (ret != sizeof(buf)) {
		dev_err(&client->dev, "Unable set resolution %dx%d\n",
			width, height);
		return -EIO;
	}

	return 0;
}

static int airtouch_set_configuration(struct airtouch_i2c *touch)
{
	struct i2c_client *client = touch->client;
	int ret;
	u8 buf[5] = { CMD_SetConfiguration, 0, 0, 0, 0 };

	memcpy(buf + 1, &config, 4);
	ret = i2c_master_send(client, buf, sizeof(buf));
	if (ret != sizeof(buf)) {
		dev_err(&client->dev, "Unable set configuration\n");
		return -EIO;
	}

	return 0;
}

static int airtouch_set_rate(struct airtouch_i2c *touch)
{
	struct i2c_client *client = touch->client;
	int ret;
	u8 buf[3] = { CMD_SetScanFreq, 33, 0};

	buf[2] = data_rate;
	ret = i2c_master_send(client, buf, sizeof(buf));
	if (ret != sizeof(buf)) {
		dev_err(&client->dev, "Unable set scan rate in %dHz\n",
			data_rate);
		return -EIO;
	}

	return 0;
}

static int airtouch_set_alt(struct airtouch_i2c *touch)
{
	struct i2c_client *client = touch->client;
	int ret;
	u8 buf[9] = { 0xf0, 0, 0, 0, 0, 0, 0, 0, 0 };

	memcpy(buf + 1, &alt0, 4);
	memcpy(buf + 5, &alt1, 4);
	ret = i2c_master_send(client, buf, sizeof(buf));
	if (ret != sizeof(buf)) {
		dev_err(&client->dev, "Unable set alt\n");
		return -EIO;
	}

	return 0;
}

static int airtouch_get_status(struct airtouch_i2c *touch)
{
	int ret;

	ret = i2c_smbus_write_byte(touch->client, CMD_GetStatus);
	if (ret < 0)
		return ret;

	return 0;
}

static int airtouch_get_coordinates(struct airtouch_i2c *touch)
{
	int ret;

	ret = i2c_smbus_write_byte(touch->client, CMD_GetCoordinate);
	if (ret < 0)
		return ret;

	return 0;
}

static int airtouch_startup(struct airtouch_i2c *touch)
{
	int ret;

	ret = airtouch_activate(touch, CMD_Activate);
	if (!ret)
		ret = airtouch_set_resolution(touch);
	if (!ret)
		ret = airtouch_set_configuration(touch);
	if (!ret)
		ret =airtouch_set_rate(touch);
	if (!ret)
		ret =airtouch_set_alt(touch);
	if (!ret)
		ret =airtouch_get_status(touch);
	if (!ret)
		ret =airtouch_get_coordinates(touch);
	return ret;
}

static void rotate_coordinate(int *x, int *y)
{
	int tmp;

	switch (rotation) {
	case 90:
		tmp = *y;
		*y = width - 1 - *x;
		*x = tmp;
		break;
	case 180:
		*x = width - 1 - *x;
		*y = height - 1 - *y;
		break;
	case 270:
		tmp = *x;
		*x = height - 1 - *y;
		*y = tmp;
		break;
	}
}

static void report_xyz(struct input_dev *input, u8 *buf, u8 cnt)
{
	int xpos, ypos, event, z, i;
	if (AirtouchDebug >= DEBUG_LVL_DATA) {
		int touch;
		u8 *b;
		for (touch = 0; touch < cnt; touch++) {
			b = buf + touch * 7;
			printk("touch %s %d: ", __func__,(b[4] >> 2) & 0x0f);
			for (i = 0; i < 7; i++)
				printk("%02x ", b[i]);
			printk(" : (%d,%d)+(%d,%d)\n",
				(b[1] << 8) | b[0], (b[3] << 8) | b[2],
				((b[4] & 0x03) << 3) | (b[5] >> 5), b[5] & 0x1f);
		}
	}

	/* skip the corrupted message if probability is too small */
	if ((cnt == 1) && (buf[6] < 60))
		return;
	if ((cnt == 2) && ((buf[6] < 60) || (buf[13] < 60)))
		return;
	for (i = 0; i < cnt; i++) {
		buf += i*7;
		xpos = (buf[1] << 8) | buf[0];
		ypos = (buf[3] << 8) | buf[2];
		rotate_coordinate(&xpos, &ypos);
		event = buf[4] >> 6;
		z = buf[5] & 0x1f;

		if (AirtouchDebug >= DEBUG_LVL_DATA){
			printk(" %s  z =%d 	event= %d  xpos=%d ypos=%d \n", __func__,z,event,xpos,ypos);
		}
		if (event == EVENT_PENDOWN)
			input_report_key(input, BTN_TOUCH, 1);

		if (event == EVENT_PENDOWN || event == EVENT_PENMOVE) {
			input_report_abs(input, ABS_X, xpos);
			input_report_abs(input, ABS_Y, ypos);
			input_report_abs(input, ABS_PRESSURE, z);
		} else if (event == EVENT_PENUP) {
			input_report_key(input, BTN_TOUCH, 0);
			input_report_abs(input, ABS_X, xpos);
			input_report_abs(input, ABS_Y, ypos);
			input_report_abs(input, ABS_PRESSURE, 0);
		}
		input_sync(input);
	}
}

static int report_finger_data(struct input_dev *input, u8 *buf)
{
	int xpos, ypos, event, id, xsize, ysize, i;

	if (AirtouchDebug >= DEBUG_LVL_DATA) {
		printk("dtouch  %s  %d: ", __func__,(buf[4] >> 2) & 0x0f);
		for (i = 0; i < 7; i++)
			printk("%02x ", buf[i]);
		printk(" : (%d,%d)+(%d,%d)\n",
			(buf[1] << 8) | buf[0], (buf[3] << 8) | buf[2],
			((buf[4] & 0x03) << 3) | (buf[5] >> 5), buf[5] & 0x1f);
	}

	xpos = (buf[1] << 8) | buf[0];
	ypos = (buf[3] << 8) | buf[2];
	event = buf[4] >> 6;
	id = (buf[4] >> 2) & 0x0f;
	xsize = ((buf[4] & 0x03) << 3) | ((buf[5] >> 5) & 0x07);
	ysize = buf[5] & 0x1f;

	if (event == EVENT_PENUP)
		return 0;

	rotate_coordinate(&xpos, &ypos);
	if (AirtouchDebug >= DEBUG_LVL_DATA){
		printk(" %s  xsize =%d 	ysize= %d xpos=%d ypos=%d \n", __func__,xsize,ysize,xpos,ypos);
	}

	input_report_abs(input, ABS_MT_TOUCH_MAJOR, xsize);
	input_report_abs(input, ABS_MT_TOUCH_MINOR, ysize);
	input_report_abs(input, ABS_MT_ORIENTATION, ysize > xsize ? 0 : 1);
	input_report_abs(input, ABS_MT_POSITION_X, xpos);
	input_report_abs(input, ABS_MT_POSITION_Y, ypos);
	input_report_abs(input, ABS_MT_TRACKING_ID, id);
	input_mt_sync(input);

	return 1;
}

static void report_mt(struct input_dev *input, u8 *buf, u8 cnt)
{
	int i, fingers = 0;

	for (i = 0; i < cnt; i++)
		fingers += report_finger_data(input, &buf[i*7]);

	input_report_key(input, BTN_TOUCH, fingers > 0);
	input_report_key(input, BTN_TOOL_FINGER, fingers == 1);
	input_report_key(input, BTN_TOOL_DOUBLETAP, fingers == 2);
	//	input_report_abs(input, ABS_TOOL_WIDTH, abs_w);
	input_sync(input);
	//	report_xyz(input, buf, cnt);
}

static	struct led_adc_info {
	u32 led_adc_num;
	u32 colORrow;
	u32 time;
	u32 strength;
	u32 data[50];
	u32 count;
	u32 really;
}led_info;

static void airtouch_i2c_process_input(struct airtouch_i2c *touch)
{
	struct i2c_client *client = touch->client;
	u8 hdr[2];
	int ret, i;

	if (AirtouchDebug >= DEBUG_LVL_DRV)
		printk("%s:%d\n", __func__, __LINE__);

	ret = i2c_master_recv(client, hdr, sizeof(hdr));
	if (ret <= 0) {
		if (ret != -EAGAIN)
			dev_err(&client->dev, "Unable get the header, ret=%d\n", ret);
		return;
	}

	if (hdr[0] != FRAME_START) {
		dev_err(&client->dev, "Got corrupted header 0x%x\n", hdr[0]);
		return;
	}
	if (hdr[1] == 0) {
		dev_err(&client->dev, "Got wrong payload size %d\n", hdr[1]);
		return;
	}

	memset(touch->payload, 0xff, PAYLOAD_MAX_SIZE);
	ret = i2c_master_recv(client, touch->payload, hdr[1]);
	if (AirtouchDebug >= DEBUG_LVL_MSG) {
		printk("hdr: %02x %02x; body: cmdid=%02x ", hdr[0], hdr[1], touch->payload[0]);
		for (i=1; i<hdr[1]; i++)
			printk("%02x ", touch->payload[i]);
		printk("\n");
	}
	if (ret != hdr[1]) {
		dev_err(&client->dev, "Failed to read paylad, "
			"only %d bytes of %d received\n", ret, hdr[1]);
		return;
	}

	switch (touch->payload[0]) {
	case CMD_Deactivate:
		/* fall through */
	case CMD_Activate:
		/* skip the corrupted message */
		if (hdr[1] == 2)
			touch->resp_activate = touch->payload[1];
		break;

	case CMD_SetResolution:
		/* skip the corrupted message */
		if (hdr[1] == 2)
			touch->resp_resolution = touch->payload[1];
		break;
	case CMD_SetConfiguration:
		break;
	case CMD_SetScanFreq:
		touch->resp_scanrate = touch->payload[1];
		break;

	case CMD_GetStatus:
		if (AirtouchDebug >= DEBUG_LVL_DATA) {
			u8 *buf = &touch->payload[1];
			printk("hw version: %d\n", buf[7] << 8 | buf[6]);
			printk("fw version: %d.%d\t\t",
				buf[1] << 8 | buf[0], bcd2bin(buf[2]));
			printk("build: %02x%02x%02x\n", buf[3], buf[4], buf[5]);
			printk("activated: %d\n", buf[8]);
			printk("scan rate: %d %d %d\n",
				buf[15], buf[16], buf[17]);
			printk("resolution: %d x %d\n",
				(buf[19] << 8) | buf[18],
				(buf[21] << 8) | buf[20]);
		}
		complete(&touch->startup_done);
		break;

	case CMD_GetCoordinate:
		{
			int count = touch->payload[1];
			if (count > 2 || count == 0) {
				dev_err(&client->dev, "Unable support %d touches\n",
					count);
				return;
			}

			if (touch->resp_activate || touch->resp_resolution)
				break;
			switch (mt) {
				case 0:
					/* legacy mouse style */
					if (count == 1)
						report_xyz(touch->input, &touch->payload[2], 1);
					/* else discard multi-touch */
					break;
				case 1:
					/* legacy mouse style with serialized multi-touch */
					report_xyz(touch->input, &touch->payload[2], count);
					break;
				case 2:
					/* Android's Multi-touch Pointer Gestures */
					report_mt(touch->input, &touch->payload[2], count);
					break;
				case 3:
					/* Qt4's QMouseEvent or QTouchEvent */
					if (count == 1)
						report_xyz(touch->input, &touch->payload[2], 1);
					else
						report_mt(touch->input, &touch->payload[2], count);
					break;
				default:
					dev_err(&client->dev, "Wrong parameter mt=%d\n", mt);
					return;
			}
			if ((mt == 1) || (count == 1))
				////*report_xyz*/report_mt(touch->input, &touch->payload[2], count);
				break;
			}

		case CMD_FixedPulseStrength:
			{
				led_info.count = touch->payload[1];
				led_info.really = 1;
				if (AirtouchDebug >= DEBUG_LVL_DATA) {
					printk("[");
					for (i=0; i<led_info.count; i++) {
						printk("%04d ", touch->payload[2 + i*7] + (touch->payload[3 + i*7]<<8));
					}
					printk("\n");
				}
				complete(&touch->cmd_done);
				break;
			}

		case 0x0d:
			if (AirtouchDebug >= DEBUG_LVL_DATA) {
				printk("Alarm colx%2d: ", touch->payload[1]);
				for (i = 0; i < touch->payload[1]; i++)
					printk("%02x ", touch->payload[3+i]);
				printk("\n");
				printk("Alarm rowx%2d: ", touch->payload[2]);
				for (i = 0; i < touch->payload[2]; i++)
					printk("%02x ", touch->payload[3+touch->payload[1]+i]);
				printk("\n");
			}
			/* this command must be put after got the long status */
			i2c_smbus_write_byte(touch->client, 0x1c);
			break;

		case 0x1c:
			if (AirtouchDebug >= DEBUG_LVL_DATA) {
				int off;
				printk("LED colx%2d: ", touch->payload[1]);
				for (i = 0; i < touch->payload[1]; i++) {
					off = 3+i*3;
					printk("%02x      ", touch->payload[off]);
				}
				printk("\n            ");
				for (i = 0; i < touch->payload[1]; i++) {
					off = 3+i*3;
					printk("%3d,%3d ",
						touch->payload[off+1],
						touch->payload[off+2]);
				}
				printk("\n");
				printk("LED rowx%2d: ", touch->payload[2]);
				for (i = 0; i < touch->payload[2]; i++) {
					off = 3+touch->payload[1]*3+i*3;
					printk("%02x      ", touch->payload[off]);
				}
				printk("\n            ");
				for (i = 0; i < touch->payload[2]; i++) {
					off = 3+touch->payload[1]*3+i*3;
					printk("%3d,%3d ",
						touch->payload[off+1],
						touch->payload[off+2]);
				}
				printk("\n");
			}
			break;

		default:
			dev_err(&client->dev, "Invalid command id 0x%02x read back\n",
				touch->payload[0]);
			break;
		}
}

static void airtouch_i2c_reschedule_work(struct airtouch_i2c *touch,
										 unsigned long delay)
{
	unsigned long flags;

	spin_lock_irqsave(&touch->lock, flags);

	/*
	* If work is already scheduled then subsequent schedules will not
	* change the scheduled time that's why we have to cancel it first.
	*/
	__cancel_delayed_work(&touch->dwork);
	schedule_delayed_work(&touch->dwork, delay);

	spin_unlock_irqrestore(&touch->lock, flags);
}

static irqreturn_t airtouch_i2c_irq(int irq, void *dev_id)
{
	struct airtouch_i2c *touch = dev_id;

#ifdef CONFIG_ARCH_AT91
	if (AirtouchDebug >= DEBUG_LVL_IRQ)
		printk("%s edge interrupt\n",
		gpio_get_value(touch->client->irq) ? "Rising" : "Falling");
	if (gpio_get_value(touch->client->irq) == 0)
		airtouch_i2c_process_input(touch);
	return IRQ_HANDLED;
#endif
	/* it is low level sensitive and is not cleared on device */
	disable_irq_nosync(irq);
	if (AirtouchDebug >= DEBUG_LVL_IRQ)
		printk("%s:%d\n", __func__, __LINE__);
	airtouch_i2c_reschedule_work(touch, 0);

	return IRQ_HANDLED;
}

/* Work Handler */
static void airtouch_i2c_work_handler(struct work_struct *work)
{
	struct airtouch_i2c *touch =
		container_of(work, struct airtouch_i2c, dwork.work);

	airtouch_i2c_process_input(touch);
	enable_irq(touch->irq);
}

static int airtouch_i2c_open(struct input_dev *input)
{
	struct airtouch_i2c *touch = input_get_drvdata(input);
	int ret;

	if (AirtouchDebug >= DEBUG_LVL_DRV) printk("%s:%d\n", __func__, __LINE__);

	ret = airtouch_startup(touch);
	if (ret)
		return ret;

	return 0;
}

static void airtouch_i2c_close(struct input_dev *input)
{
	struct airtouch_i2c *touch = input_get_drvdata(input);

	if (AirtouchDebug >= DEBUG_LVL_DRV) printk("%s:%d\n", __func__, __LINE__);

	wait_for_completion_interruptible(&touch->startup_done);
	airtouch_activate(touch, CMD_Deactivate);
	cancel_delayed_work_sync(&touch->dwork);
}

static void airtouch_i2c_set_input_params(struct airtouch_i2c *touch)
{
	struct input_dev *input = touch->input;

	input->name = touch->client->name;
	input->phys = touch->client->adapter->name;
	input->id.bustype = BUS_I2C;
	input->id.version = 0x1003;
	input->dev.parent = &touch->client->dev;
	input->open = airtouch_i2c_open;
	input->close = airtouch_i2c_close;
	input_set_drvdata(input, touch);

	__set_bit(EV_ABS, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);

	if ((rotation == 90) || (rotation == 270)) {
		input_set_abs_params(input, ABS_X, 0, height-1, 0, 0);
		input_set_abs_params(input, ABS_Y, 0, width-1, 0, 0);
	} else {
		input_set_abs_params(input, ABS_X, 0, width-1, 0, 0);
		input_set_abs_params(input, ABS_Y, 0, height-1, 0, 0);
	}
	input_set_abs_params(input, ABS_PRESSURE, 0, pen_size, 0, 0);

	if (mt == 0)
		return;

	__set_bit(BTN_TOOL_FINGER, input->keybit);
	__set_bit(BTN_TOOL_DOUBLETAP, input->keybit);

	//input->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;

	//input->absbit[0] = BIT_MASK(ABS_MT_TRACKING_ID) |
	///	BIT_MASK(ABS_MT_TOUCH_MAJOR)| BIT_MASK(ABS_MT_WIDTH_MAJOR) |
	//	BIT_MASK(ABS_MT_POSITION_X) | BIT_MASK(ABS_MT_POSITION_Y); 	// for android

	/* finger touch area */
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, pen_size, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0, pen_size, 0, 0);
	/* finger approach area */
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, pen_size, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MINOR, 0, pen_size, 0, 0);
	/* finger orientation */
	input_set_abs_params(input, ABS_MT_ORIENTATION, -1, 1, 0, 0);
	/* finger position */
	if ((rotation == 90) || (rotation == 270)) {
		input_set_abs_params(input, ABS_MT_POSITION_X, 0, height-1, 0, 0);
		input_set_abs_params(input, ABS_MT_POSITION_Y, 0, width-1, 0, 0);
	} else {
		input_set_abs_params(input, ABS_MT_POSITION_X, 0, width-1, 0, 0);
		input_set_abs_params(input, ABS_MT_POSITION_Y, 0, height-1, 0, 0);
	}
}

/*
* adc value operations
*/
static ssize_t store_led_num(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct airtouch_i2c *touch = dev_get_drvdata(dev);
	int ret, num;
	u8 buffer[5] = {0, 0, 0,};
	struct i2c_client *client = touch->client;

	num = sscanf(buf,"%u %u %u %u", &led_info.led_adc_num, &led_info.colORrow, &led_info.strength, &led_info.time);

	//#if 0
	//printk(KERN_INFO"led num: %u",led_info.led_adc_num);
	//if (led_info.colORrow == 0)
	//	printk(KERN_INFO"COL ");
	//else if (led_info.colORrow == 1)
	//	printk(KERN_INFO"ROW ");
	//printk(KERN_INFO"strength: %u time delay: %u\n",led_info.strength, led_info.time);
	//#endif

	if (led_info.colORrow != 0 && led_info.colORrow != 1) {
		printk(KERN_ERR "colORrow MUST be 0 or 1\n");
		return -EINVAL;
	}

	if (led_info.strength > 15) {
		printk(KERN_ERR "strength MUST be less than 15.\n");
		return -EINVAL;
	}

	if (led_info.time > 11) {
		printk(KERN_ERR " MUST be less than 11.\n");
		return -EINVAL;	
	}

	buffer[0] = CMD_FixedPulseStrength;
	buffer[1] = led_info.led_adc_num;
	buffer[2] = led_info.colORrow;
	buffer[3] = led_info.time ;
	buffer[4] = led_info.strength;
	ret = i2c_master_send(client, buffer, sizeof(buffer));
	if (ret != sizeof(buffer)) {
		dev_err(&client->dev, "Unable get adc value in led %d\n",
			led_info.led_adc_num);
		return -EIO;
	}
	return count;
}

static ssize_t show_touch_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct airtouch_i2c *touch = dev_get_drvdata(dev);
	int num;
	u8 *b = &touch->payload[2];

	wait_for_completion_interruptible(&touch->cmd_done);
	if (led_info.really == 1) {
		num = sprintf(buf,"[%04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d\n", \
			led_info.colORrow, led_info.led_adc_num,\
			b[0] + (b[1]<<8), b[7] + (b[8]<<8), b[14] + (b[15]<<8), b[21] + (b[22]<<8), \
			b[28] + (b[29]<<8),b[35] + (b[36]<<8),b[42] + (b[43]<<8),b[49] + (b[50]<<8),\
			b[56] + (b[57]<<8), b[63] + (b[64]<<8), b[70] + (b[71]<<8),b[77] + (b[78]<<8),\
			b[84] + (b[85]<<8), b[91] + (b[92]<<8), b[98] + (b[99]<<8),b[105] + (b[106]<<8),\
			b[112] + (b[113]<<8)
			);
		printk("%s", buf);

		led_info.really = 0;
		led_info.count = 0;
	}else {
		printk("No ADC value tranfer out\n");
		return -EIO;
	}
	return num;
}

static DEVICE_ATTR(touch_value, S_IRUGO | S_IWUGO, show_touch_value, store_led_num);
static struct attribute *airtouch_device_attrs[] = {
	&dev_attr_touch_value.attr,
	NULL
};

static struct attribute_group airtouch_group = {
	.attrs = airtouch_device_attrs,
};

static struct airtouch_i2c *airtouch_i2c_touch_create(struct i2c_client *client)
{
	struct airtouch_i2c *touch;

	touch = kzalloc(sizeof(struct airtouch_i2c), GFP_KERNEL);
	if (!touch)
		return NULL;

	touch->client = client;
	INIT_DELAYED_WORK(&touch->dwork, airtouch_i2c_work_handler);
	spin_lock_init(&touch->lock);
	init_completion(&touch->cmd_done);
	init_completion(&touch->startup_done);

	return touch;
}

static int __devinit airtouch_i2c_probe(struct i2c_client *client,
										const struct i2c_device_id *dev_id)
{
	int ret;
	struct airtouch_i2c *touch;
	unsigned long flags;

	if (AirtouchDebug >= DEBUG_LVL_DRV)
		printk("%s:%d\n", __func__, __LINE__);

	touch = airtouch_i2c_touch_create(client);
	if (!touch)
		return -ENOMEM;

	touch->input = input_allocate_device();
	if (!touch->input) {
		ret = -ENOMEM;
		goto err_mem_free;
	}

	airtouch_i2c_set_input_params(touch);

#if !defined(CONFIG_ARCH_AT91) && !defined(CONFIG_ARCH_MXC)
	ret = gpio_request(client->irq, "airtouch irq");
	if (ret) {
		dev_warn(&client->dev, "GPIO %d request failed: %d\n",
			client->irq, ret);
		goto err_mem_free;
	}

	touch->irq = gpio_to_irq(client->irq);
#else
	touch->irq = client->irq;
#endif
	if (AirtouchDebug >= DEBUG_LVL_DRV) printk("Requesting IRQ: %d\n", touch->irq);

	flags = IRQF_DISABLED;
#ifdef CONFIG_ARCH_AT91
	flags |= IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING;
#else
	flags |= IRQF_TRIGGER_LOW;
#endif
	ret = request_irq(touch->irq, airtouch_i2c_irq,
		flags, AIRTOUCH_DRIVER_NAME, touch);
	if (ret) {
		dev_warn(&client->dev,
			"IRQ %d request failed: %d\n", touch->irq, ret);
		goto err_gpio_free;
	}

	/* Register the device in input subsystem */
	ret = input_register_device(touch->input);
	if (ret) {
		dev_err(&client->dev,
			"Input device register failed: %d\n", ret);
		goto err_input_free;
	}

	i2c_set_clientdata(client, touch);
	dev_set_drvdata(&client->dev, touch);

	device_init_wakeup(&client->dev, 1);
	/* FIXME: remove it if this driver's suspend hook can be called */
	enable_irq_wake(touch->irq);

	/* create sysfs interface */
	ret = sysfs_create_group(&touch->input->dev.kobj, &airtouch_group);
	if (ret != 0)
		goto err_input_free;

	if (AirtouchDebug >= DEBUG_LVL_DRV) printk("%s:%d\n", __func__, __LINE__);	
	return 0;

err_input_free:
	input_free_device(touch->input);
err_gpio_free:
#if !defined(CONFIG_ARCH_AT91) && !defined(CONFIG_ARCH_MXC)
	gpio_free(client->irq);
#endif
err_mem_free:
	kfree(touch);

	return ret;
}

static int __devexit airtouch_i2c_remove(struct i2c_client *client)
{
	struct airtouch_i2c *touch = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&touch->dwork);

	device_init_wakeup(&client->dev, 0);
	free_irq(touch->irq, touch);
#if !defined(CONFIG_ARCH_AT91) && !defined(CONFIG_ARCH_MXC)
	gpio_free(client->irq);
#endif
	sysfs_remove_group(&client->dev.kobj, &airtouch_group);
	input_unregister_device(touch->input);
	i2c_set_clientdata(client, NULL);
	kfree(touch);

	return 0;
}

#ifdef CONFIG_PM
static int airtouch_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct airtouch_i2c *touch = i2c_get_clientdata(client);

	if (AirtouchDebug >= DEBUG_LVL_DRV) 
		printk("%s:%d\n", __func__, __LINE__);

	cancel_delayed_work_sync(&touch->dwork);

	/* Save some power */
	airtouch_activate(touch, CMD_Deactivate);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(touch->irq);

	return 0;
}

static int airtouch_i2c_resume(struct i2c_client *client)
{
	int ret;
	struct airtouch_i2c *touch = i2c_get_clientdata(client);

	if (AirtouchDebug >= DEBUG_LVL_DRV) 
		printk("%s:%d\n", __func__, __LINE__);

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(touch->irq);

	ret = airtouch_startup(touch);
	if (ret)
		return ret;

	return 0;
}
#else
#define airtouch_i2c_suspend	NULL
#define airtouch_i2c_resume	NULL
#endif

static const struct i2c_device_id airtouch_i2c_id_table[] = {
#ifdef CONFIG_ARCH_AT91
	/* back-compatible */
	{ "avtir_i2c", 0 },
#else
	{ AIRTOUCH_DRIVER_NAME, 0 },
#endif
	{ },
};
MODULE_DEVICE_TABLE(i2c, airtouch_i2c_id_table);

static struct i2c_driver airtouch_i2c_driver = {
	.driver = {
		.name	= AIRTOUCH_DRIVER_NAME,
		.owner	= THIS_MODULE,
	},

	.probe		= airtouch_i2c_probe,
	.remove		= __devexit_p(airtouch_i2c_remove),

	.suspend	= airtouch_i2c_suspend,
	.resume	= airtouch_i2c_resume,
	.id_table	= airtouch_i2c_id_table,
};

static int __init airtouch_i2c_init(void)
{
	return i2c_add_driver(&airtouch_i2c_driver);
}

static void __exit airtouch_i2c_exit(void)
{
	i2c_del_driver(&airtouch_i2c_driver);
}

module_init(airtouch_i2c_init);
module_exit(airtouch_i2c_exit);

MODULE_DESCRIPTION("AvatarSemi I2C airTouch driver");
MODULE_AUTHOR("wlu@avatarsemi.com");
MODULE_LICENSE("GPL");

