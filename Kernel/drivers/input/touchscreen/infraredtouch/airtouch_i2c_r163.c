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
#include <linux/slab.h>
#include <linux/gpio.h>

#ifdef CONFIG_ARCH_ROCKCHIP
#include <linux/workqueue.h>
#include <asm/arch/gpio.h>
#include <asm/arch/iomux.h>
struct airtouch_i2c *g_touch;

#define AIRTOUCH_I2C_ADDR    0x50
/* Modify pin assignment for custom design */
#define AIRTOUCH_IRQ_PIN     GPIOPortE_Pin3
#define AIRTOUCH_RST_PIN     GPIOPortE_Pin0	/*BSL_RST*/
#define AIRTOUCH_TST_PIN     GPIOPortF_Pin5	/*BSL_TEST*/
#endif

#define DRIVER_NAME		"airtouch_i2c"
#define DRIVER_VERSION		"$Revision: 1.63 $"

#define EVENT_PENDOWN 0
#define EVENT_PENMOVE 1
#define EVENT_PENUP   2

#define CMD_RESP_OK	0
#define CMD_TIMEOUT	(2 * HZ)

#define REG_Dummy			0x5a
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

#define BCD2BIN(val)	(((val) & 0x0f) + ((val) >> 4) * 10)

static int width = 600;
module_param(width, int, 0644);
MODULE_PARM_DESC(width, "Screen width in pixel. Default = 600");

static int height = 800;
module_param(height, int, 0644);
MODULE_PARM_DESC(height, "Screen height in pixel. Default = 800");

static int rotation;
module_param(rotation, int, 0644);
MODULE_PARM_DESC(rotation, "Screen rotation angle. Default = 0");

static int data_rate = 60;
module_param(data_rate, int, 0644);
MODULE_PARM_DESC(date_rate, "Report data rate in HZ. Default = 60");

static int pen_size = 31;
module_param(pen_size, int, 0644);
MODULE_PARM_DESC(pen_size, "Maximum pen size in mm. Default = 31");

static int mt = 2;
module_param(mt, int, 0644);
MODULE_PARM_DESC(mt, "Support multiply touch. Default = 0 (no mt), 1 (serialized mt), 2 (Android mt), 3 (Qt mt)");

#ifdef CONFIG_ARCH_AT91
static int config = 0x10000;	/* interrupt isn't low level sensitive */
#else
static int config;
#endif
module_param(config, int, 0644);
MODULE_PARM_DESC(config, "Main configuration. Default = 0");

static int alt0 = 0;
module_param(alt0, int, 0644);
MODULE_PARM_DESC(alt0, "Alternate configuration 0. Default = 0");

static int alt1 = 0;
module_param(alt1, int, 0644);
MODULE_PARM_DESC(alt1, "Alternate configuration 1. Default = 0");

#define DEBUG_LVL_DATA 1
#define DEBUG_LVL_MSG  2
#define DEBUG_LVL_IRQ  5
#define DEBUG_LVL_DRV  9
static int debug = 1;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Set debug level. Default = 0, no debug");

#define PAYLOAD_MAX_SIZE 256
/* The main device structure */
struct airtouch_i2c {
	struct i2c_client	*client;
	struct input_dev	*input;
	struct delayed_work	dwork;
	int			irq;
	spinlock_t		lock;
	struct completion	done_diag;
	struct completion	done_startup;
	int			started;
	u8			cmd_diag;
	u8			resp_activate;
	u8			resp_resolution;
	u8			resp_scanrate;
	u8			payload[PAYLOAD_MAX_SIZE];
	u8			diag_buf[PAYLOAD_MAX_SIZE];
	u8			fmt_buf[PAGE_SIZE];
};

static int format_output_msg(char *buf, u8 *hdr, u8 *body)
{
	int i, num = 0;

	num += sprintf(buf + num, "hdr: %02x %02x; body: cmdid=%02x ",
		       hdr[0], hdr[1], body[0]);
	for (i=1; i<hdr[1]; i++)
		num += sprintf(buf + num, "%02x ", body[i]);
	num += sprintf(buf + num, "\n");
	return num;
}

static int format_output_0x04(char *buf, u8 *b)
{
	int i, num = 0;

	num += sprintf(buf + num, "touch %d: ", (b[4] >> 2) & 0x0f);
	for (i = 0; i < 7; i++)
		num += sprintf(buf + num, "%02x ", b[i]);
	num += sprintf(buf + num, ": (%d,%d)+(%d,%d)\n",
		       (b[1] << 8) | b[0], (b[3] << 8) | b[2],
		       ((b[4] & 0x03) << 3) | (b[5] >> 5), b[5] & 0x1f);
	return num;
}

static int format_output_0x0d(char *buf, u8 *b)
{
	int i, num = 0;

	for (i = 0; i < b[0]; i++)
		num += sprintf(buf + num, "     %02x ", b[2+i]);
	num += sprintf(buf + num, "\n");
	for (i = 0; i < b[1]; i++)
		num += sprintf(buf + num, "     %02x ", b[2+b[0]+i]);
	num += sprintf(buf + num, "\n");
	return num;
}

static int format_output_0x1c(char *buf, u8 *b)
{
	int i, num = 0;

	for (i = 0; i < b[0]; i++)
		num += sprintf(buf + num, "     %2d ", i);
	num += sprintf(buf + num, "\n");
	for (i = 0; i < b[0]; i++)
		num += sprintf(buf + num, "     %02x ", b[2+i*3]);
	num += sprintf(buf + num, "\n");
	for (i = 0; i < b[0]; i++)
		num += sprintf(buf + num, "%03d,%03d ",
			       b[2+i*3+1], b[2+i*3+2]);
	num += sprintf(buf + num, "\n");
	
	for (i = 0; i < b[1]; i++)
		num += sprintf(buf + num, "     %2d ", i);
	num += sprintf(buf + num, "\n");
	for (i = 0; i < b[1]; i++)
		num += sprintf(buf + num, "     %02x ",
			       b[2+b[0]*3+i*3]);
	num += sprintf(buf + num, "\n");
	for (i = 0; i < b[1]; i++)
		num += sprintf(buf + num, "%03d,%03d ",
			       b[2+b[0]*3+i*3+1], b[2+b[0]*3+i*3+2]);
	num += sprintf(buf + num, "\n");
	return num;
}

static int format_output_0x1e(char *buf, u8 *b)
{
	int i, num = 0;

	num += sprintf(buf + num, "hw version: %d.%d\n", b[7], b[6]);
	num += sprintf(buf + num, "fw version: %d.%d\t\t",
		       b[1] << 8 | b[0], BCD2BIN(b[2]));
	num += sprintf(buf + num, "build: %02x%02x%02x\n", b[3], b[4], b[5]);
	num += sprintf(buf + num, "activated: %d\n", b[8]);
	num += sprintf(buf + num, "scan rate: %d %d %d\n",
		       b[15], b[16], b[17]);
	num += sprintf(buf + num, "resolution: %d x %d\n",
		       (b[19] << 8) | b[18], (b[21] << 8) | b[20]);
	for (i = 0; i < 64; i++)
		num += sprintf(buf + num, "%02x ", b[i]);
	num += sprintf(buf + num, "\n");
	return num;
}

static int format_output_0x0f(char *buf, u8 *b)
{
	int i, num = 0;

	for (i = 0; i < b[0]; i++)
		num += sprintf(buf + num, "%3d ", i);
	num += sprintf(buf + num, "\n");
	for (i = 0; i < b[0]; i++)
		num += sprintf(buf + num, "%03d ", b[2+i]);
	num += sprintf(buf + num, "\n");

	for (i = 0; i < b[1]; i++)
		num += sprintf(buf + num, "%3d ", i);
	num += sprintf(buf + num, "\n");
	for (i = 0; i < b[1]; i++)
		num += sprintf(buf + num, "%03d ", b[2+b[0]+i]);
	num += sprintf(buf + num, "\n");
	return num;
}

static int format_output_0x1f(char *buf, u8 *b)
{
	int i, num = 0;

	num += sprintf(buf + num, "    ");
	for (i = 0; i < b[1]; i++)
		num += sprintf(buf + num, "%3d ", i);
	num += sprintf(buf + num, "\n");
	num += sprintf(buf + num, "%2d: ", b[0]);
	for (i = 0; i < b[1]; i++)
		num += sprintf(buf + num, "%03d ", b[2+i]);
	num += sprintf(buf + num, "\n");
	return num;
}

static int airtouch_activate(struct airtouch_i2c *touch, u8 act)
{
	struct i2c_client *client = touch->client;
	int ret;
#ifdef CONFIG_ARCH_ROCKCHIP
	u8 buf[2] = {REG_Dummy, 0};
	buf[1] = act;
#else
	u8 buf[1];
	buf[0] = act;
#endif

	if (debug >= DEBUG_LVL_DRV)
		printk(KERN_INFO "%s:%d\n", __func__, __LINE__);
	ret = i2c_master_send(client, buf, sizeof(buf));
	if (debug >= DEBUG_LVL_DRV)
		printk(KERN_INFO "command 0x%02x is sent, return %d\n",
		       act, ret);
	if (ret != sizeof(buf)) {
		dev_err(&client->dev, "Unable %sactivate device\n",
			act ? "" : "in");
		return -EIO;
	}

	return 0;
}

static int airtouch_set_resolution(struct airtouch_i2c *touch)
{
	struct i2c_client *client = touch->client;
	int ret;
#ifdef CONFIG_ARCH_ROCKCHIP
	u8 buf[6] = { REG_Dummy, CMD_SetResolution, 0, 0, 0, 0 };
	int arg = 2;
#else
	u8 buf[5] = { CMD_SetResolution, 0, 0, 0, 0 };
	int arg = 1;
#endif

	memcpy(buf + arg, &width, 2);
	memcpy(buf + arg + 2, &height, 2);
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
#ifdef CONFIG_ARCH_ROCKCHIP
	u8 buf[6] = { REG_Dummy, CMD_SetConfiguration, 0, 0, 0, 0 };
	int arg = 2;
#else
	u8 buf[5] = { CMD_SetConfiguration, 0, 0, 0, 0 };
	int arg = 1;
#endif

	if (debug == 1)
		config |= (1 << 20);
	memcpy(buf + arg, &config, 4);
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
#ifdef CONFIG_ARCH_ROCKCHIP
	u8 buf[4] = { REG_Dummy, CMD_SetScanFreq, 33, 0};
	int arg = 3;
#else
	u8 buf[3] = { CMD_SetScanFreq, 33, 0};
	int arg = 2;
#endif

	buf[arg] = data_rate;
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
#ifdef CONFIG_ARCH_ROCKCHIP
	u8 buf[10] = { REG_Dummy, 0xf0, 0, 0, 0, 0, 0, 0, 0, 0 };
	int arg = 2;
#else
	u8 buf[9] = { 0xf0, 0, 0, 0, 0, 0, 0, 0, 0 };
	int arg = 1;
#endif

	memcpy(buf + arg, &alt0, 4);
	memcpy(buf + arg + 4, &alt1, 4);
	ret = i2c_master_send(client, buf, sizeof(buf));
	if (ret != sizeof(buf)) {
		dev_err(&client->dev, "Unable set alt\n");
		return -EIO;
	}

	return 0;
}

static int airtouch_get_status(struct airtouch_i2c *touch)
{
	struct i2c_client *client = touch->client;
	int ret;
#ifdef CONFIG_ARCH_ROCKCHIP
	u8 buf[2] = {REG_Dummy, 0};
	buf[1] = CMD_GetStatus;
#else
	u8 buf[1];
	buf[0] = CMD_GetStatus;
#endif

	ret = i2c_master_send(touch->client, buf, sizeof(buf));
	if (ret != sizeof(buf)) {
		dev_err(&client->dev, "Unable get status\n");
		return -EIO;
	}

	return 0;
}

static int airtouch_get_coordinates(struct airtouch_i2c *touch)
{
	struct i2c_client *client = touch->client;
	int ret;
#ifdef CONFIG_ARCH_ROCKCHIP
	u8 buf[2] = {REG_Dummy, 0};
	buf[1] = CMD_GetCoordinate;
#else
	u8 buf[1];
	buf[0] = CMD_GetCoordinate;
#endif

	ret = i2c_master_send(touch->client, buf, sizeof(buf));	
	if (ret != sizeof(buf)) {
		dev_err(&client->dev, "Unable get coordinates\n");
		return -EIO;
	}

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
		ret = airtouch_set_rate(touch);
	if (!ret)
		ret = airtouch_set_alt(touch);
	if (!ret)
		ret = airtouch_get_status(touch);
	if (!ret)
		ret = airtouch_get_coordinates(touch);
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
	if ((rotation != 0) && (debug >= DEBUG_LVL_DATA))
		printk(KERN_INFO "\t\t\t\t -> (%d,%d)\n", *x, *y);
}

static void report_xyz(struct input_dev *input, u8 *buf, u8 cnt)
{
	struct airtouch_i2c *touch = input_get_drvdata(input);
	int xpos, ypos, event, z, i;

	if (debug >= DEBUG_LVL_DATA) {
		for (i = 0; i < cnt; i++) {
			format_output_0x04(touch->fmt_buf, buf + i*7);
			printk(KERN_INFO "%s", touch->fmt_buf);
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
	struct airtouch_i2c *touch = input_get_drvdata(input);
	int xpos, ypos, event, id, xsize, ysize;

	if (debug >= DEBUG_LVL_DATA) {
		touch->fmt_buf[0] = 'd';
		format_output_0x04(touch->fmt_buf+1, buf);
		printk(KERN_INFO "%s", touch->fmt_buf);
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

	if (fingers == 0)
		input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0);

	input_report_key(input, BTN_TOUCH, fingers > 0);
	input_report_key(input, BTN_TOOL_FINGER, fingers == 1);
	input_report_key(input, BTN_TOOL_DOUBLETAP, fingers == 2);
//	input_report_abs(input, ABS_TOOL_WIDTH, abs_w);
	input_sync(input);
//	report_xyz(input, buf, cnt);
}

static int airtouch_i2c_process_input(struct airtouch_i2c *touch)
{
	struct i2c_client *client = touch->client;
	u8 hdr[2];
	int ret;

	ret = i2c_master_recv(client, hdr, sizeof(hdr));
	if (ret <= 0) {
		if (ret != -EAGAIN)
			dev_err(&client->dev, "Unable get the header, ret=%d\n", ret);
		return ret;
	}

	if (hdr[0] != FRAME_START) {
		dev_err(&client->dev, "Got corrupted header 0x%x\n", hdr[0]);
		return -EIO;
	}
	if (hdr[1] == 0) {
		dev_err(&client->dev, "Got wrong payload size %d\n", hdr[1]);
		return -EIO;
	}

	memset(touch->payload, 0xff, PAYLOAD_MAX_SIZE);
	ret = i2c_master_recv(client, touch->payload, hdr[1]);
	if (debug >= DEBUG_LVL_MSG) {
		format_output_msg(touch->fmt_buf, hdr, touch->payload);
		printk(KERN_INFO "%s", touch->fmt_buf);
	}
	if (ret != hdr[1]) {
		dev_err(&client->dev, "Failed to read paylad, "
			"only %d bytes of %d received\n", ret, hdr[1]);
		return ret;
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
		if (debug >= DEBUG_LVL_DATA) {
			format_output_0x1e(touch->fmt_buf, &touch->payload[1]);
			printk(KERN_INFO "%s", touch->fmt_buf);
		}
		if (touch->cmd_diag != CMD_GetStatus) {
			touch->started = 1;
			complete(&touch->done_startup);
		}
		break;

	case CMD_GetCoordinate:
	{
		int count = touch->payload[1];
		if (count > 2 || count == 0) {
			dev_err(&client->dev, "Unable support %d touches\n",
				count);
			return -EIO;
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
			return -EINVAL;
		}
		break;
	}

	case 0x05:
		if (debug >= DEBUG_LVL_DATA) {
			format_output_0x0f(touch->fmt_buf, &touch->payload[1]);
			printk(KERN_INFO "%s", touch->fmt_buf);
		}
		break;

	case 0x0d:
	{
#ifdef CONFIG_ARCH_ROCKCHIP
		u8 buf[2] = {REG_Dummy, 0x1c};
#else
		u8 buf[1] = {0x1c};
#endif
		if (debug >= DEBUG_LVL_DATA) {
			format_output_0x0d(touch->fmt_buf, &touch->payload[1]);
			printk(KERN_INFO "%s", touch->fmt_buf);
		}
		/* this command must be put after got the long status */
		i2c_master_send(client, buf, sizeof(buf));
		break;
	}

	case 0x0f:
	case 0x1f:
	case 0x2f:
	case 0x09:
	case 0xf0:
		break;

	case 0x1c:
		if (debug >= DEBUG_LVL_DATA) {
			format_output_0x1c(touch->fmt_buf, &touch->payload[1]);
			printk(KERN_INFO "%s", touch->fmt_buf);
		}
		break;

	default:
		dev_err(&client->dev, "Invalid command id 0x%02x read back\n",
			touch->payload[0]);
		break;
	}

	if (touch->cmd_diag == touch->payload[0]) {
		memcpy(touch->diag_buf, &touch->payload[1], hdr[1] - 1);
		complete(&touch->done_diag);
	}
	return 0;
}

static irqreturn_t airtouch_i2c_irq(int irq, void *dev_id)
{
	struct airtouch_i2c *touch = dev_id;

#ifdef CONFIG_ARCH_ROCKCHIP
	if (debug >= DEBUG_LVL_IRQ)
		printk(KERN_INFO "rk interrupt line at %s\n",
		       GPIOGetPinLevel(touch->client->irq) ? "high" : "low");

	gpio_irq_disable(touch->client->irq);
	schedule_delayed_work(&touch->dwork, 0);
#else
	int ret;

	if (debug >= DEBUG_LVL_IRQ)
		printk(KERN_INFO "interrupt line at %s\n",
		       gpio_get_value(touch->client->irq) ? "high" : "low");

	while (gpio_get_value(touch->client->irq) == 0) {
		ret = airtouch_i2c_process_input(touch);
		if (debug >= DEBUG_LVL_IRQ)
			printk(KERN_INFO "return %d after process input\n",
			       ret);
		if (ret != 0)
			break;
	}
#endif

	return IRQ_HANDLED;
}

#ifdef CONFIG_ARCH_ROCKCHIP
static void airtouch_i2c_work_handler(struct work_struct *work)
{
	struct airtouch_i2c *touch =
			container_of(work, struct airtouch_i2c, dwork.work);
	if (debug >= DEBUG_LVL_DRV)
		printk(KERN_INFO "%s:%d\n", __func__, __LINE__);

	airtouch_i2c_process_input(touch);
	
	gpio_irq_enable(touch->client->irq);
}
#endif

static int airtouch_i2c_open(struct input_dev *input)
{
	struct airtouch_i2c *touch = input_get_drvdata(input);
	int ret;

	if (debug >= DEBUG_LVL_DRV)
		printk(KERN_INFO "%s:%d\n", __func__, __LINE__);

	ret = airtouch_startup(touch);
	if (ret)
		return ret;

	return 0;
}

static void airtouch_i2c_close(struct input_dev *input)
{
	struct airtouch_i2c *touch = input_get_drvdata(input);

	if (debug >= DEBUG_LVL_DRV)
		printk(KERN_INFO "%s:%d\n", __func__, __LINE__);

	if (!touch->started)
		wait_for_completion_interruptible(&touch->done_startup);
	touch->started = 0;
	airtouch_activate(touch, CMD_Deactivate);
#ifdef CONFIG_ARCH_ROCKCHIP
	cancel_delayed_work_sync(&touch->dwork);
#endif
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
 * diagnostic operations via sysfs
 */
static ssize_t store_diagnostic(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
#ifdef CONFIG_ARCH_ROCKCHIP
	struct airtouch_i2c *touch = g_touch;
	u8 cmd[4] = {REG_Dummy, 0, 0, 0};
	int idx = 1;
#else
	struct airtouch_i2c *touch = dev_get_drvdata(dev);
	u8 cmd[3] = {0, 0, 0};
	int idx = 0;
#endif
	struct i2c_client *client = touch->client;
	int ret, len;
	char str[8];

	if (debug >= DEBUG_LVL_DRV)
		printk(KERN_INFO "%s:%d\n", __func__, __LINE__);

        sscanf(buf, "%s %x", str, &len);
	if (strncmp(str, "start", 4) == 0) {
		ret = airtouch_startup(touch);
		if (ret)
			return -EIO;
		return count;
	} else 	if (strncmp(str, "stop", 5) == 0) {
		touch->started = 0;
		airtouch_activate(touch, CMD_Deactivate);
		return count;
	} else if (strncmp(str, "pulse", 5) == 0) {
		cmd[idx++] = 0x0f;
		cmd[idx++] = len & 0xff;
	} else if (strncmp(str, "level", 5) == 0) {
		cmd[idx++] = 0x1c;
	} else if (strncmp(str, "led", 3) == 0) {
		cmd[idx++] = 0x1f;
		cmd[idx++] = (len >> 0) & 0xff;
		cmd[idx++] = (len >> 8) & 0xff;
	} else if (strncmp(str, "nei", 3) == 0) {
		cmd[idx++] = 0x2f;
		cmd[idx++] = len & 0xff;
	} else if (strncmp(str, "status", 6) == 0) {
		cmd[idx++] = CMD_GetStatus;
	} else if (strncmp(str, "bsl", 3) == 0) {
#ifdef CONFIG_ARCH_ROCKCHIP
		GPIOSetPinDirection(AIRTOUCH_RST_PIN, GPIO_OUT);
		GPIOSetPinDirection(AIRTOUCH_TST_PIN, GPIO_OUT);

		GPIOSetPinLevel(AIRTOUCH_RST_PIN, GPIO_LOW);
		GPIOSetPinLevel(AIRTOUCH_TST_PIN, GPIO_LOW);
		msleep(80);
		GPIOSetPinLevel(AIRTOUCH_TST_PIN, GPIO_HIGH);
		msleep(10);
		GPIOSetPinLevel(AIRTOUCH_TST_PIN, GPIO_LOW);
		msleep(20);
		GPIOSetPinLevel(AIRTOUCH_TST_PIN, GPIO_HIGH);
		msleep(10);
		GPIOSetPinLevel(AIRTOUCH_RST_PIN, GPIO_HIGH);
		msleep(10);
		GPIOSetPinLevel(AIRTOUCH_TST_PIN, GPIO_LOW);
		msleep(80);

		GPIOSetPinDirection(AIRTOUCH_RST_PIN, GPIO_IN);
		GPIOSetPinDirection(AIRTOUCH_TST_PIN, GPIO_IN);
		return count;
#else
		airtouch_activate(touch, 0xEB);
		return count;
#endif
	} else
		return -EINVAL;
	len = idx;
	touch->cmd_diag = cmd[1];

	/* don't hurt startup sequence */
	if (!touch->started)
		wait_for_completion_interruptible(&touch->done_startup);
	ret = i2c_master_send(client, cmd, len);
	if (ret != len) {
		dev_err(&client->dev, "Unable send command 0x%02x\n", cmd[0]);
		return -EIO;
	}
	return count;
}

static ssize_t show_diagnostic(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_ARCH_ROCKCHIP
	struct airtouch_i2c *touch = g_touch;
#else
	struct airtouch_i2c *touch = dev_get_drvdata(dev);
#endif
	int num = 0;
	u8 *b = touch->diag_buf;

	wait_for_completion_interruptible(&touch->done_diag);
	switch (touch->cmd_diag) {
	case 0x0f:
	case 0x2f:
		num += format_output_0x0f(buf, b);
		break;
	case 0x1c:
		num += format_output_0x1c(buf, b);
		break;
	case 0x1e:
		num += format_output_0x1e(buf, b);
		break;
	case 0x1f:
		num += format_output_0x1f(buf, b);
		break;
	default:
		return -EIO;
	}
	touch->cmd_diag = 0xff;
	return num;
}

static DEVICE_ATTR(diagnostic, S_IRUGO | S_IWUGO,
		   show_diagnostic, store_diagnostic);
static struct attribute *airtouch_device_attrs[] = {
	&dev_attr_diagnostic.attr,
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
#ifdef CONFIG_ARCH_ROCKCHIP
	INIT_DELAYED_WORK(&touch->dwork, airtouch_i2c_work_handler);
#endif
	spin_lock_init(&touch->lock);
	init_completion(&touch->done_diag);
	init_completion(&touch->done_startup);

	return touch;
}

#ifdef CONFIG_ARCH_ROCKCHIP
static struct i2c_driver airtouch_i2c_driver;
static int airtouch_i2c_probe(struct i2c_adapter *adapter, int addr, int kind)
#else
static int __devinit airtouch_i2c_probe(struct i2c_client *client,
			       const struct i2c_device_id *dev_id)
#endif
{
	int ret;
	struct airtouch_i2c *touch;
	unsigned long flags;
#ifdef CONFIG_ARCH_ROCKCHIP
	struct i2c_client *client = NULL; 

	if (debug >= DEBUG_LVL_DRV)
		printk(KERN_INFO "%s:%d\n", __func__, __LINE__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		printk( "Must have I2C_FUNC_I2C.\n");	
		ret = -ENODEV;	
		goto err_check_functionality_failed;
	}
	client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!client) {
		ret = -ENOMEM;	
		goto err_alloc_data_failed;
	}
	strlcpy(client->name, "airtouch_i2c", I2C_NAME_SIZE);	
	client->addr = addr;	
	client->adapter = adapter;
	client->driver = &airtouch_i2c_driver;	
	client->addressBit = I2C_7BIT_ADDRESS_8BIT_REG; 	
	client->mode = DIRECTMODE;	
	client->Channel = I2C_CH0;
	client->speed = 300;
	client->irq = AIRTOUCH_IRQ_PIN;

	ret = i2c_attach_client(client);	
	if (ret) {
		goto exit_i2c_attach_client_failed;
	}

	/* reset airtouch at first */
	GPIOSetPinDirection(AIRTOUCH_RST_PIN, GPIO_OUT);
	GPIOSetPinLevel(AIRTOUCH_RST_PIN, GPIO_LOW);
	msleep(100);
	GPIOSetPinLevel(AIRTOUCH_RST_PIN, GPIO_HIGH);
	/* release reset pin for external debug */
	GPIOSetPinDirection(AIRTOUCH_RST_PIN, GPIO_IN);
	GPIOPullUpDown(AIRTOUCH_RST_PIN, GPIOPullUp);

	rockchip_mux_api_set(GPIOE_SPI1_FLASH_SEL2_NAME,IOMUXA_GPIO1_A3B7);
	GPIOSetPinDirection(client->irq, GPIO_IN);
	GPIOPullUpDown(client->irq, GPIOPullUp);
#endif

	if (debug >= DEBUG_LVL_DRV)
		printk(KERN_INFO "%s:%d\n", __func__, __LINE__);

	touch = airtouch_i2c_touch_create(client);
	if (!touch)
		return -ENOMEM;

	touch->input = input_allocate_device();
	if (!touch->input) {
		ret = -ENOMEM;
		goto err_mem_free;
	}

	airtouch_i2c_set_input_params(touch);

#if defined(CONFIG_ARCH_AT91) || defined(CONFIG_ARCH_MXC)
	touch->irq = client->irq;
	client->irq = irq_to_gpio(client->irq);
#elif defined(CONFIG_ARCH_ROCKCHIP)
	touch->irq = client->irq;
#else
	ret = gpio_request(client->irq, "airtouch irq");
	if (ret) {
		dev_warn(&client->dev, "GPIO %d request failed: %d\n",
			 client->irq, ret);
		goto err_mem_free;
	}

	touch->irq = gpio_to_irq(client->irq);
#endif
	if (debug >= DEBUG_LVL_DRV)
		printk(KERN_INFO "Requesting IRQ: %d\n", touch->irq);

#if defined(CONFIG_ARCH_AT91)
	flags = IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_DISABLED;
	ret = request_irq(touch->irq, airtouch_i2c_irq,
			  flags, DRIVER_NAME, touch);
#elif defined(CONFIG_ARCH_ROCKCHIP)
	g_touch = touch;
	flags = IRQF_TRIGGER_LOW;
	ret = request_gpio_irq(touch->irq,(pFunc)airtouch_i2c_irq,flags,touch);
#else
	flags = IRQF_TRIGGER_LOW | IRQF_ONESHOT;
	ret = request_threaded_irq(touch->irq, NULL, airtouch_i2c_irq,
				   flags, DRIVER_NAME, touch);
#endif
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

	if (debug >= DEBUG_LVL_DRV)
		printk(KERN_INFO "%s:%d\n", __func__, __LINE__);	
	return 0;

err_input_free:
	input_free_device(touch->input);
err_gpio_free:
#if !defined(CONFIG_ARCH_AT91) && !defined(CONFIG_ARCH_MXC)
	gpio_free(client->irq);
#endif
err_mem_free:
	kfree(touch);
#ifdef CONFIG_ARCH_ROCKCHIP
exit_i2c_attach_client_failed:
	kfree(client);
err_alloc_data_failed:
err_check_functionality_failed:
#endif

	return ret;
}

static int __devexit airtouch_i2c_remove(struct i2c_client *client)
{
	struct airtouch_i2c *touch = i2c_get_clientdata(client);

#ifdef CONFIG_ARCH_ROCKCHIP
	cancel_delayed_work_sync(&touch->dwork);
#endif
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

	if (debug >= DEBUG_LVL_DRV)
		printk(KERN_INFO "%s:%d\n", __func__, __LINE__);

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

	if (debug >= DEBUG_LVL_DRV)
		printk(KERN_INFO "%s:%d\n", __func__, __LINE__);

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

#ifdef CONFIG_ARCH_ROCKCHIP

static unsigned short normal_i2c[] = {AIRTOUCH_I2C_ADDR, I2C_CLIENT_END};
static unsigned short ignore = I2C_CLIENT_END;
static struct i2c_client_address_data addr_data = {	
	.normal_i2c	= normal_i2c,	
	.probe		= &ignore,
	.ignore		= &ignore,
};
static int airtouch_attach_adapter(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, airtouch_i2c_probe);
}
static int airtouch_detach_client(struct i2c_client *client)
{	
	airtouch_i2c_remove(client);
	return i2c_detach_client(client);
}

static struct i2c_driver airtouch_i2c_driver = {
	.driver 	= {
		.name	= "airtouch_i2c",
	},
	.id 	= AIRTOUCH_I2C_ADDR,
	.suspend	= airtouch_i2c_suspend,
	.resume		= airtouch_i2c_resume,
	.attach_adapter = airtouch_attach_adapter,
	.detach_client	= airtouch_detach_client,
};

#else

static const struct i2c_device_id airtouch_i2c_id_table[] = {
#ifdef CONFIG_ARCH_AT91
	/* back-compatible */
	{ "avtir_i2c", 0 },
#else
	{ "airtouch_i2c", 0 },
#endif
	{ },
};
MODULE_DEVICE_TABLE(i2c, airtouch_i2c_id_table);

static struct i2c_driver airtouch_i2c_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},

	.probe		= airtouch_i2c_probe,
	.remove		= __devexit_p(airtouch_i2c_remove),

	.suspend	= airtouch_i2c_suspend,
	.resume		= airtouch_i2c_resume,
	.id_table	= airtouch_i2c_id_table,
};

#endif /* CONFIG_ARCH_ROCKCHIP */

static int __init airtouch_i2c_init(void)
{
	printk(KERN_INFO "AvatarSemi I2C airTouch driver: "
	       DRIVER_VERSION ", built on " __DATE__ " at " __TIME__ "\n");
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

