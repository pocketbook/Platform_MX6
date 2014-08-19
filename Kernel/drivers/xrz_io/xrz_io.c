//#include <linux/config.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/string.h>

#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/miscdevice.h>
#include <linux/irq.h>

#include <mach/gpio.h>

#define	DEVICE_NAME 		"xrz_io"
#define	DEVICE_MINJOR		190

//////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///3g  control: open and close.
#define IOCTL_3G_ENABLE     125
#define IOCTL_3G_DISABLE     126

///wifi  control: open and close.
#define IOCTL_WIFI_ENABLE     131
#define IOCTL_WIFI_DISABLE     132
////////////////////////////////
extern int  wifi_power_control(bool bEnable);

#if defined(CONFIG_USE_DEVICE_ID)
//extern char __initdata boot_command_line[COMMAND_LINE_SIZE];
//static char deviceID_id[] ="deviceID=";
#define isspace(c)	((c) == ' ')
#define isdigit(c)	('0' <= (c) && (c) <= '9')
static int deviceId=0;
//static char *machineId;
//char *deviceIdoptions;
int atoi(const char *nptr)
{
	int c;/*current char*/
	int total; /* current total */
	int sign; /* if ''-'', then negative, otherwise positive */
	/* skip whitespace */
	while ( isspace((int)(unsigned char)*nptr) )
		++nptr;
	c = (int)(unsigned char)*nptr++;
	sign = c; /* save sign indication */
	if (c == '-' || c == '+'){
		c = (int)(unsigned char)*nptr++; /* skip sign */
	}
	total = 0;
	while (isdigit(c)) {
		total = 10 * total + (c - '0'); /* accumulate digit */
		c = (int)(unsigned char)*nptr++; /* get next char */
	}
	if (sign == '-')
		return -total;
	else
		return total; /* return result, negated if necessary */
}
char *left(char *dst,char *src,int n,int m)
{
	char *p=src;
	char *q=dst;
	int len=strlen(src);
	if(n>len) n=len;
	while(n--)
	   *(q++)=*(p++);
	*(q++)='/0';
	return dst;
}
char *right(char *dst,char *src,int n,int m)
{
	char *p=src;
	char *q=dst;
	int len=strlen(src);
	if(n>len) n=len;
	p+=(len-n);
	while(*(q++)=*(p++));
	return dst;
}


char *substring(char *dst,char *src,int len,int start)
{
	char *p=dst;
	char *q=src;
	printk("dst =%s,src=%s\n",dst,src);
	int length=strlen(src);/*use the funtion strlen,the kernel can't bootup*/
	printk("len=%d,start=%d,length=%d\n",len,start,length);
	if(start>=length||start<0) 
		return NULL;
	if(len>length)
		len=length-start;
	q+=start;
	while(len--)
	{
		*(p++)=*(q++);
	}
	*(p++)='/0';

	return dst;
}
static int __init deviceID_setup(char *options)
{
	//int deviceId=0;
	char *firstOptions;
	char *secondOptions;
	char *thirdOptions;
	////TODO: string to integer conversion and store the ID
	printk("options=%s\n",options);

	//substring(firstOptions,options,9,1);
	//substring(secondOptions,options,9,17);
	//substring(thirdOptions,options,18,19);

	//printk("firstOptions=%s\n",firstOptions);
	//printk("secondOptions=%s\n",secondOptions);
	//printk("thirdOptions=%s\n",thirdOptions);

	/*
	* we can use the funtion atoi or use the funtion sscanf
	*/
	deviceId=atoi(options);
	//sscanf(options,"%d",&deviceId);
	//printk(" deviceId=%d\n",deviceId);

	return 1;

}

__setup("deviceID=", deviceID_setup);


#endif

static int xrz_app_open(struct inode *inode,struct file *filp)
{
	return 0;
}

static int xrz_app_release(struct inode *inode,struct file *filp)
{
	return 0;
}

static long xrz_app_unlockedioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = -ENOTTY;

	switch (cmd) {
	case IOCTL_WIFI_ENABLE:
		wifi_power_control(true);
		//printk("%s,%s,%d ,IOCTL_WIFI_ENABLE.\n",__FILE__,__func__,__LINE__);
		break;
	case IOCTL_WIFI_DISABLE:
		//printk("%s,%s,%d ,IOCTL_WIFI_DISABLE.\n",__FILE__,__func__,__LINE__);
		wifi_power_control(false);
		break;
#if defined(CONFIG_USE_DEVICE_ID)
	   case IOCTL_DEVICEID:
		   printk("received the  read quest\n");
		   //if (put_user(deviceId,(int __user *)argp)){
		   //  retval = -EFAULT;
		   ////}
		   //deviceId=12345679;
		   if(copy_to_user((unsigned char*)arg,&deviceId,sizeof(deviceId))!=0)
			   ret = -EFAULT;
		   ret =0;
		   break;
#endif
	}

	return ret;
}

static struct file_operations xrz_app_driver= {
	.owner  	=   THIS_MODULE,
	.open   	=   	xrz_app_open,
	.unlocked_ioctl	  =  xrz_app_unlockedioctl,
	.compat_ioctl = xrz_app_unlockedioctl,
	.release   =   	xrz_app_release,

};


static struct miscdevice xrz_app_device = {
	.minor		= DEVICE_MINJOR,
	.name		= DEVICE_NAME,
	.fops		= &xrz_app_driver,
};

static int __init xrz_app_init(void)
{
	int ret;
	ret = misc_register(&xrz_app_device);
	if (ret < 0) {
		printk("pvi_io: can't get major number\n");
		return ret;
	}
	return 0;
}

static void __exit xrz_app_exit(void) 
{
	misc_deregister(&xrz_app_device);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jim_dream");
MODULE_VERSION("2011-3-20");
MODULE_DESCRIPTION ("xrz_io driver");

module_init(xrz_app_init);
module_exit(xrz_app_exit);
