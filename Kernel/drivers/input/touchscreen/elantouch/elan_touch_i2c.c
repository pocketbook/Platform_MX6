/*
 *  ELAN touchscreen driver
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
      
#include <linux/input.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>

#include  <linux/imx6sl/mx6sl_gpio_cfg.h>


#define ELAN_TS_NAME    "elan-touch"
#define IRQ_READY           1    
//#define ELAN_I2C_ID 	 	0x1588
//#define ELAN_I2C_ADDR 	0x10

#define ELAN_TS_X_MAX 		800
#define ELAN_TS_Y_MAX 		600
#define IDX_PACKET_SIZE		8

#define CONTROL_TOUCH_HXM

#if defined(CONFIG_XRZ_EBOOKREADING_MODE)
extern int  iXrzSuspendMode;
static bool bNeedRarlyResumeIrq = false;
static bool bNeedResumeElanTouch= false;
#endif


extern int mx50_rdp_get_hardwareVersion(void);

enum {	
	hello_packet  = 0x55,
	idx_coordinate_packet 	= 0x5a,
	};
enum {
	idx_finger_state = 7,
};
static struct workqueue_struct *elan_wq = NULL;
static struct elan_data {
	int intr_gpio;
	int use_irq;	
	//struct hrtimer timer;	
	struct work_struct work;	
	struct i2c_client *client;
	struct input_dev *input;	
	wait_queue_head_t wait;
}  elan_touch_data = {0};

#ifdef IRQ_READY	
/*--------------------------------------------------------------*/
static int elan_touch_detect_int_level(void)
{
	unsigned v;

	v = gpio_get_value(elan_touch_data.intr_gpio);	
	//printk("%s(%s) v=%d intr_gpio=%d \n", __FILE__, __func__,v,elan_touch_data.intr_gpio);
	return v;
}
static int __elan_touch_poll(struct i2c_client *client)
{	
	int status = 0, retry = 20;	
	do {		
		status = elan_touch_detect_int_level();
		retry--;	
		mdelay(20);
		} while (status == 1 && retry > 0);
	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int elan_touch_poll(struct i2c_client *client)
{	
	return __elan_touch_poll(client);
}
#endif 
static int __hello_packet_handler(struct i2c_client *client)
{	
	int rc;	
	uint8_t buf_recv[4] = { 0 };
#ifdef IRQ_READY	
	rc = elan_touch_poll(client);
	if (rc < 0) {
		 printk("%s(%s) rc=%d \n", __FILE__, __func__,rc);
		return -EINVAL;	
		}
#endif 
	rc = i2c_master_recv(client, buf_recv, 4);
	if (rc != 4) {
		 printk("%s(%s) rc=%d \n", __FILE__, __func__,rc);	
		 printk("hello error packet: [0x%02x 0x%02x 0x%02x 0x%02x]\n",
			buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
		return rc;	
	} else {
		int i;
		printk("hello packet: [0x%02x 0x%02x 0x%02x 0x%02x]\n",
			buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);

		for (i = 0; i < 4; i++)
			if (buf_recv[i] != hello_packet)
				return -EINVAL;
	}

	return 0;
}

static inline int elan_touch_parse_xy(uint8_t *data, uint16_t *x, uint16_t *y)
{
	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];

	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];

	return 0;
}

/*	__elan_touch_init -- hand shaking with touch panel
 *
 *	1.recv hello packet
 */
static int __elan_touch_init(struct i2c_client *client)
{	
	int rc;	
	rc = __hello_packet_handler(client);	
	if (rc < 0)
		goto hand_shake_failed;
hand_shake_failed:
	return rc;
}

static int elan_touch_recv_data(struct i2c_client *client, uint8_t *buf)
{	
	int rc, bytes_to_recv = IDX_PACKET_SIZE;
	if (buf == NULL)
		return -EINVAL;	
	memset(buf, 0, bytes_to_recv);
	rc = i2c_master_recv(client, buf, bytes_to_recv);
	if (rc != bytes_to_recv) {	
		return -EINVAL;	
	}	
	return rc;
}

static void elan_touch_report_data(struct i2c_client *client, uint8_t *buf)
{	
        //static int  iPreFinger =0;
	switch (buf[0]) {
		case idx_coordinate_packet: 
			{		
				uint16_t x1, x2, y1, y2;		
				uint8_t finger_stat;	
				finger_stat = (buf[idx_finger_state] & 0x06) >> 1;
				x1=y1=x2=y2=0;
				if (finger_stat == 0) {	
					if ((buf[idx_finger_state] == 1)
						&& !(buf[1]|buf[2]|buf[3]|buf[4]|buf[5]|buf[6])) {
#if defined(CONTROL_TOUCH_HXM)
						input_report_abs(elan_touch_data.input, ABS_MT_TOUCH_MAJOR, 0 );
						input_mt_sync(elan_touch_data.input);
						input_sync(elan_touch_data.input);
#else
						input_report_key(elan_touch_data.input, BTN_TOUCH, 0);	
						input_report_key(elan_touch_data.input, BTN_2, 0);		
						input_sync(elan_touch_data.input);
#endif
					      //printk("%s(%s):elan touch up\n", __FILE__, __func__);
					    // iPreFinger = finger_stat;
					}
				}else if (finger_stat == 1) {			
					elan_touch_parse_xy(&buf[1], &x1, &y1);
					printk("!*x1:%d y1:%d  \n", x1,y1);
					if(x1 ==0 || y1 ==0)
					    break;
 					
					x1= 800-x1*800/1088;
					y1= 600 -y1*600/768;
					x1= x1*950/1000 + 16;
					y1= y1*952/1000 +14;
					//if(x1<5)
					//x1=5;
					//else if(x1>795)
					//x1= 795;
					//if(y1<5)
					//y1=5;
					//else if(y1>595)
					//y1= 595;
					if(x1 !=0 &&  y1 !=0){
					 printk("x1:%d y1:%d\n", x1,y1);	
#if defined(CONTROL_TOUCH_HXM)
					input_report_abs(elan_touch_data.input, ABS_MT_TOUCH_MAJOR,5);
					input_report_abs(elan_touch_data.input, ABS_MT_POSITION_X, x1);
					input_report_abs(elan_touch_data.input, ABS_MT_POSITION_Y, y1);
					input_report_abs(elan_touch_data.input, ABS_MT_WIDTH_MAJOR, 1);
					input_mt_sync(elan_touch_data.input);
#else
					input_report_abs(elan_touch_data.input, ABS_X, x1);
					input_report_abs(elan_touch_data.input, ABS_Y, y1);
					input_report_key(elan_touch_data.input, BTN_TOUCH, 1);	
					input_report_key(elan_touch_data.input, BTN_2, 0);		
#endif
					}
					//iPreFinger = finger_stat;
					//input_sync(elan_touch_data.input);
				}else if (finger_stat == 2) {
					elan_touch_parse_xy(&buf[1], &x1, &y1);	
					//printk("!2-1 x1:%d y1:%d \n", x1,y1);
					if(x1 ==0 || y1 ==0)
					      break;
					x1= 800-x1*800/1088;
					y1= 600 -y1*600/768;
					x1= x1*950/1000 + 16;
					y1= y1*952/1000 +14;
					//if(x1<5)
					//x1=5;
					//else if(x1>795)
					//x1= 795;
					//if(y1<5)
					//y1=5;
					//else if(y1>595)
					//y1= 595;
					if(x1 != 0 && y1 != 0){
					//printk("2-1 x1:%d y1:%d\n", x1,y1);
#if defined(CONTROL_TOUCH_HXM)
					input_report_abs(elan_touch_data.input, ABS_MT_TOUCH_MAJOR,5);
					input_report_abs(elan_touch_data.input, ABS_MT_POSITION_X, x1);
					input_report_abs(elan_touch_data.input, ABS_MT_POSITION_Y, y1);
					input_report_abs(elan_touch_data.input, ABS_MT_WIDTH_MAJOR, 1);
					input_mt_sync(elan_touch_data.input);
#else
					input_report_abs(elan_touch_data.input, ABS_X, x1);
					input_report_abs(elan_touch_data.input, ABS_Y, y1);
					input_report_key(elan_touch_data.input, BTN_TOUCH, 1);
#endif
					}
					elan_touch_parse_xy(&buf[4], &x2, &y2);
					//printk("!2-2 x2:%d y2:%d\n", x2,y2);
					if(x2 == 0 || y2 ==0)
						break;
					x2= 800-x2*800/1088;
					y2= 600 -y2*600/768;
					x2= x2*950/1000 + 16;
					y2= y2*952/1000 +14;
					//if(x2<5)
					//x2=5;
					//else if(x2>795)
					//x2= 795;
					//if(y2<5)
					//y2=5;
					//else if(y2>595)
					//y2= 595;
					if(x2 != 0 &&  y2 != 0){
					//printk("2-2 x2:%d y2:%d\n", x2,y2);
#if defined(CONTROL_TOUCH_HXM)
					input_report_abs(elan_touch_data.input, ABS_MT_TOUCH_MAJOR, 5);
					input_report_abs(elan_touch_data.input, ABS_MT_POSITION_X, x2);
					input_report_abs(elan_touch_data.input, ABS_MT_POSITION_Y, y2);
					input_report_abs(elan_touch_data.input, ABS_MT_WIDTH_MAJOR, 1);
					input_mt_sync(elan_touch_data.input);
#else
					input_report_abs(elan_touch_data.input, ABS_HAT0X, x2);	
					input_report_abs(elan_touch_data.input, ABS_HAT0Y, y2);
					input_report_key(elan_touch_data.input, BTN_2, 1);
#endif
					}
					//iPreFinger = finger_stat;
					//input_sync(elan_touch_data.input);
				}		
				input_sync(elan_touch_data.input);
				break;	
			}	
		default:	
			break;	
	}
}

static void elan_touch_work_func(struct work_struct *work)
{	
	int rc;	
	uint8_t buf[IDX_PACKET_SIZE] = { 0 };	
	struct i2c_client *client = elan_touch_data.client;
#ifdef IRQ_READY	
	if (elan_touch_detect_int_level())		
		return;	
#endif 
	rc = elan_touch_recv_data(client, buf);
	if (rc < 0)
		return;	
	elan_touch_report_data(client, buf);
}
/*
extern struct timer_list  XrzEbookReadingSuspendModeTimer;
static int elan_touch_start_suspendTimer(void)
{
   static uint64_t time1=0;
   static uint64_t time2=0;
   printk("%s %s %d gpio=%d  \n",__FILE__,__func__,__LINE__,gpio_get_value(TOUCH_PANEL_IRQ) ); 
   if(gpio_get_value(TOUCH_PANEL_IRQ) == 0){
   	time1 = jiffies;
	printk("time1=%lx \n",time1);
   }
   do{
   	time2 = jiffies;
	printk("time2=%lx \n",time2);
   }while((gpio_get_value(TOUCH_PANEL_IRQ) == 1) && ((time2-time1) < msecs_to_jiffies(2000*1000)));///2s

  if((time2-time1) >=  msecs_to_jiffies(2000*1000)){
  	XrzEbookReadingSuspendModeTimer.expires = jiffies + 1*HZ;
  	add_timer(&XrzEbookReadingSuspendModeTimer);
  }
   return 0;
}
*/
//extern int  ResuspendTimer_cancel(void);
static irqreturn_t elan_touch_ts_interrupt(int irq, void *dev_id)
{	
	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 
	queue_work(elan_wq, &elan_touch_data.work);
	//elan_touch_start_suspendTimer();
	//printk("%s %s  %s %d   \n",__FILE__,__func__,"touch irq",__LINE__); 
	//ResuspendTimer_cancel();
	return IRQ_HANDLED;
}

//static enum hrtimer_restart elan_touch_timer_func(struct hrtimer *timer)
//{	
//queue_work(elan_wq, &elan_touch_data.work);
//hrtimer_start(&elan_touch_data.timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
//return HRTIMER_NORESTART;
//}


static int elan_touch_register_interrupt(struct i2c_client *client)
{	
	int err = 0;	

	if (client->irq) {
		printk("%s(%s): allocate irq %d\n", __FILE__, __func__, client->irq);
		elan_touch_data.use_irq = 1;
		err = request_irq(client->irq, elan_touch_ts_interrupt, IRQF_TRIGGER_FALLING,
				  ELAN_TS_NAME, &elan_touch_data);

		if (err < 0) {
			printk("%s(%s): Can't allocate irq %d\n", __FILE__, __func__, client->irq);
			elan_touch_data.use_irq = 0;
		}
#if  defined(CONFIG_XRZ_EBOOKREADING_MODE)		
		else{
		   enable_irq_wake(client->irq);
		}
#endif
	}

	//if (!elan_touch_data.use_irq) {	
	//	hrtimer_init(&elan_touch_data.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);	
	//	elan_touch_data.timer.function = elan_touch_timer_func;
	//	hrtimer_start(&elan_touch_data.timer, ktime_set(1, 0), HRTIMER_MODE_REL);	
	//	}
	return 0;
}
#ifdef CONFIG_HAS_EARLYSUSPEND
static void mxc_elan_touch_early_suspend(struct early_suspend *h)
{
#if defined(CONFIG_XRZ_EBOOKREADING_MODE)
	//iXrzSuspendMode =get_xrz_current_suspendMode();
	printk("%s %s %d iXrzSuspendMode=%d  \n",__FILE__,__func__,__LINE__,iXrzSuspendMode); 
	/* wen modify.for enable the irq wakeup the device. 2012-05-15*/
	if(iXrzSuspendMode == 0){
	/*irq disable,we need sign the state*/	
	 bNeedRarlyResumeIrq = true;
	if (elan_touch_data.use_irq)
		disable_irq(elan_touch_data.client->irq);
	}
#else /*pair with CONFIG_XRZ_EBOOKREADING_MODE*/
	if (elan_touch_data.use_irq)
		disable_irq(elan_touch_data.client->irq);
#endif
}

static void mxc_elan_touch_late_resume(struct early_suspend *h)
{
#if defined(CONFIG_XRZ_EBOOKREADING_MODE)
	//iXrzSuspendMode =get_xrz_current_suspendMode();
	printk("%s %s %d iXrzSuspendMode=%d  \n",__FILE__,__func__,__LINE__,iXrzSuspendMode); 
	/* wen modify.for enable the irq wakeup the device. 2012-05-15*/
	if((iXrzSuspendMode == 0) && bNeedRarlyResumeIrq){
	    bNeedRarlyResumeIrq = false;	
	   if (elan_touch_data.use_irq)
		enable_irq(elan_touch_data.client->irq);
	}
#else /*pair with CONFIG_XRZ_EBOOKREADING_MODE*/
     if (elan_touch_data.use_irq)
	 enable_irq(elan_touch_data.client->irq);

#endif
}

static struct early_suspend mxc_elan_touch_earlysuspend = {
	.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING,
	.suspend = mxc_elan_touch_early_suspend,
	.resume = mxc_elan_touch_late_resume,
};
#endif


static int elan_touch_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		printk("elan_touch_probe i2c_check_functionality erroe! \n");
		return  err;
	}
	elan_wq = create_singlethread_workqueue("elan_wq");	
	if (!elan_wq) {
		printk("elan_touch_probe create workqueue error! \n");
		err = -ENOMEM;
		goto fail;	
	}
	elan_touch_data.client = client;
	strlcpy(client->name, ELAN_TS_NAME, I2C_NAME_SIZE);

	INIT_WORK(&elan_touch_data.work, elan_touch_work_func);
#ifdef IRQ_READY	
	elan_touch_data.intr_gpio =MX6SL_ARM2_ELAN_INT;
#endif 	
	elan_touch_data.input = input_allocate_device();
	if (elan_touch_data.input == NULL) {
		printk("elan_touch_probe input allocte error! \n");
		err = -ENOMEM;
		goto fail;
	}

	//int i=0;
	// do {
	//gpio_direction_output(TOUCH_PANEL_RESET, 1);
	//gpio_set_value(TOUCH_PANEL_RESET, 1);
	err = __elan_touch_init(client);
	//i++;
	//msleep(100);
	//}while((err<0)&&i<10);

	if (err < 0) {
		printk("Read Hello Packet Fail no out the init for test!\n");
		// goto fail;
	}	
	elan_touch_data.input->name = ELAN_TS_NAME;
	elan_touch_data.input->id.bustype = BUS_I2C;

	set_bit(EV_SYN, elan_touch_data.input->evbit);
	set_bit(EV_ABS, elan_touch_data.input->evbit);
	set_bit(EV_KEY, elan_touch_data.input->evbit);
	set_bit(BTN_TOUCH, elan_touch_data.input->keybit); 
#if defined(CONTROL_TOUCH_HXM)
	set_bit(ABS_MT_TOUCH_MAJOR, elan_touch_data.input->absbit);
	set_bit(ABS_MT_POSITION_X, elan_touch_data.input->absbit);
	set_bit(ABS_MT_POSITION_Y, elan_touch_data.input->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, elan_touch_data.input->absbit);
	input_set_abs_params(elan_touch_data.input,ABS_MT_POSITION_X, 0, ELAN_TS_X_MAX, 0, 0);
	input_set_abs_params(elan_touch_data.input,ABS_MT_POSITION_Y, 0, ELAN_TS_Y_MAX, 0, 0);
	input_set_abs_params(elan_touch_data.input,ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(elan_touch_data.input,ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
#else
	set_bit(BTN_2, elan_touch_data.input->keybit);		
	set_bit(ABS_X, elan_touch_data.input->absbit);	
	set_bit(ABS_Y, elan_touch_data.input->absbit);	
	set_bit(ABS_HAT0X, elan_touch_data.input->absbit);	
	set_bit(ABS_HAT0Y, elan_touch_data.input->absbit); 
	input_set_abs_params(elan_touch_data.input, ABS_X, 0, ELAN_TS_X_MAX, 0, 0);	
	input_set_abs_params(elan_touch_data.input, ABS_Y, 0, ELAN_TS_Y_MAX, 0, 0);	
	input_set_abs_params(elan_touch_data.input, ABS_HAT0X, 0, ELAN_TS_X_MAX, 0, 0);
	input_set_abs_params(elan_touch_data.input, ABS_HAT0Y, 0, ELAN_TS_Y_MAX, 0, 0);	
#endif
	err= input_register_device(elan_touch_data.input);	
	if (err < 0) {
		printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 
		goto fail;
	}	

	elan_touch_register_interrupt(elan_touch_data.client);	
#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&mxc_elan_touch_earlysuspend);
#endif
	return 0;	
fail:	
	if (NULL != elan_touch_data.input)
	{
		input_free_device(elan_touch_data.input);
		elan_touch_data.input = NULL;
	}
	if (NULL !=  elan_wq) {
		destroy_workqueue(elan_wq);
		elan_wq = NULL;
	}
	printk("Run elan touch probe error %d!\n",err);
	return err;
}
static int elan_touch_remove(struct i2c_client *client)
{

	//iXrzSuspendMode =get_xrz_current_suspendMode();
	//printk("%s %s %d iXrzSuspendMode=%d  \n",__FILE__,__func__,__LINE__,iXrzSuspendMode); 
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&mxc_elan_touch_earlysuspend);
#endif
	if (elan_wq)		
		destroy_workqueue(elan_wq);	
	input_unregister_device(elan_touch_data.input);

#if defined(CONFIG_XRZ_EBOOKREADING_MODE)	
	/* wen modify.for enable the irq wakeup the device. 2012-05-15*/
	if(iXrzSuspendMode == 0){	
	if (elan_touch_data.use_irq)
		free_irq(client->irq, client);	
	}
#else/*pair with CONFIG_XRZ_EBOOKREADING_MODE*/
	if (elan_touch_data.use_irq)
		free_irq(client->irq, client);	
#endif
	return 0;
}

static int elan_touch_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	uint8_t buf[]={0x54,0x50,0x00,0x01};
#if defined(CONFIG_XRZ_EBOOKREADING_MODE)	
	//iXrzSuspendMode =get_xrz_current_suspendMode();
	printk("%s %s %d iXrzSuspendMode=%d  \n",__FILE__,__func__,__LINE__,iXrzSuspendMode); 
	if(iXrzSuspendMode == 0){
		bNeedResumeElanTouch = true;
		ret = i2c_master_send(client, buf, sizeof(buf));
		if (ret != sizeof(buf)) {
			printk("%s  %s  ret=%d \n", __FILE__, __func__,ret);
			// return -1;
		}
	}
#else/*pair with CONFIG_XRZ_EBOOKREADING_MODE*/
	ret = i2c_master_send(client, buf, sizeof(buf));
	if (ret != sizeof(buf)) {
		printk("%s  %s  ret=%d \n", __FILE__, __func__,ret);
		// return -1;
	}
#endif
	return 0;
}
static int elan_touch_resume(struct i2c_client *client)
{
	int ret;
	uint8_t buf[]={0x54,0x58,0x00,0x01};

#if defined(CONFIG_XRZ_EBOOKREADING_MODE)
	//iXrzSuspendMode =get_xrz_current_suspendMode();
	printk("%s %s %d iXrzSuspendMode=%d  \n",__FILE__,__func__,__LINE__,iXrzSuspendMode); 

	if((iXrzSuspendMode == 0) && bNeedResumeElanTouch){
		bNeedResumeElanTouch = false;
		ret = i2c_master_send(client, buf, sizeof(buf));
		if (ret != sizeof(buf)) {
			printk("%s %s ret=%d \n", __FILE__, __func__,ret);	
			//ljf return -1;
		}
	}
#else/*pair with CONFIG_XRZ_EBOOKREADING_MODE*/
	ret = i2c_master_send(client, buf, sizeof(buf));
	if (ret != sizeof(buf)) {
		printk("%s %s ret=%d \n", __FILE__, __func__,ret);	
		//ljf return -1;
	}
#endif
	return 0;
}
static const struct i2c_device_id elan_touch_id[] = {
	{ ELAN_TS_NAME, 0 },{ }
};

static struct i2c_driver elan_touch_driver = {
	.probe	=elan_touch_probe ,
	.remove	= elan_touch_remove,
	.suspend = elan_touch_suspend,
	.resume =elan_touch_resume,
	.id_table	= elan_touch_id,
	.driver		= {
		.name = ELAN_TS_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init elan_touch_init(void)
{
#if defined(CONFIG_HARDWARE_VERSION_CONTROL)
         int hardwareversion =0;
         hardwareversion = mx50_rdp_get_hardwareVersion();
	 //hardwareversion =0;
	 if(0==hardwareversion){
	    return i2c_add_driver(&elan_touch_driver);
	 }else{
	    return NULL;
	 }
#else
	return i2c_add_driver(&elan_touch_driver);
#endif
}

static void __exit elan_touch_exit(void)
{
	i2c_del_driver(&elan_touch_driver);
}

module_init(elan_touch_init);
module_exit(elan_touch_exit);

MODULE_AUTHOR("Stanley Zeng <stanley.zeng@emc.com.tw>");
MODULE_DESCRIPTION("ELAN Touch Screen driver");
MODULE_LICENSE("GPL");

