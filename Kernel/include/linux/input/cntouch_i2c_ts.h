#ifndef __LINUX_CNTOUCH_TS_H__
#define __LINUX_CNTOUCH_TS_H__
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
//#define CONFIG_SUPPORT_CNT_CTP_UPG
#define TP_FIRMWARE_UPDATE
//#define TP_FIRMWARE_VERSION
//#define SUPPORT_VIRTUAL_KEY
//#define CNT_PROXIMITY
//#define CONFIG_SUPPORT_CNT_BUTTON

#ifdef TP_FIRMWARE_UPDATE
#define U8 unsigned char
#define S8 signed char
#define U16 unsigned short
#define S16 signed short
#define U32 unsigned int
#define S32 signed int
#define TOUCH_ADDR_MSG20XX   0x26
#define FW_ADDR_MSG20XX      0x62
#define FW_UPDATE_ADDR_MSG20XX   0x49
static  char *fw_version;
#define DWIIC_MODE_ISP    0
#define DWIIC_MODE_DBBUS  1
static U8 temp[94][1024];
static int FwDataCnt;
static int FwDataCnt2;
static struct class *firmware_class;
static struct device *firmware_cmd_dev;


typedef enum
{
    EMEM_ALL = 0,
    EMEM_MAIN,
    EMEM_INFO,
} EMEM_TYPE_t;


#endif
#ifdef TP_FIRMWARE_VERSION
static struct class *firmware_version_class;
static struct device *firmware_version_cmd_dev;
#endif


#define CFG_DBG_DUMMY_INFO_SUPPORT   1     /*output touch point information*/
#define CFG_DBG_FUCTION_INFO_SUPPORT 0     /*output fouction name*/
#define CFG_DBG_INPUT_EVENT                   0     /*debug input event*/


#define CFG_MAX_POINT_NUM            0x2    /*max touch points supported*/
#ifdef CNT_PROXIMITY
#define CFG_NUMOFKEYS                    0x5    /*number of touch keys + proximity support*/
#else
#define CFG_NUMOFKEYS                    0x4    /*number of touch keys*/
#endif

#ifdef CONFIG_CNTouch_CUSTOME_ENV  
#define SCREEN_MAX_X           1024
#define SCREEN_MAX_Y           600
#else
#define SCREEN_MAX_X    1024
#define SCREEN_MAX_Y      758
#endif
#define CNT_RESOLUTION_X  2036
#define CNT_RESOLUTION_Y  2036
#define PRESS_MAX                 255

#define KEY_PRESS                 0x1
#define KEY_RELEASE              0x0

#define CNT_NAME    "cnt_ts"  

#define CNT_NULL                    0x0
#define CNT_TRUE                    0x1
#define CNT_FALSE                   0x0
#define I2C_CNT_ADDRESS    0x26

#define Touch_State_No_Touch   0x0
#define Touch_State_One_Finger 0x1
#define Touch_State_Two_Finger 0x2
#define Touch_State_VK 0x3

typedef unsigned char         CNT_BYTE;    
typedef unsigned short        CNT_WORD;   
typedef unsigned int          CNT_DWRD;    
typedef unsigned char         CNT_BOOL;

#if defined(TP_FIRMWARE_UPDATE) || defined(TP_FIRMWARE_VERSION)
struct fw_version {
	unsigned short major;
	unsigned short minor;
	unsigned short VenderID;
};
#endif
 typedef struct _REPORT_FINGER_INFO_T
 {
     short   ui2_id;               /* ID information, from 0 to  CFG_MAX_POINT_NUM - 1*/
     short    u2_pressure;    /* ***pressure information, valid from 0 -63 **********/
     short    i2_x;                /*********** X coordinate, 0 - 2047 ****************/
     short    i2_y;                /* **********Y coordinate, 0 - 2047 ****************/
 } REPORT_FINGER_INFO_T;


typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

#ifdef CNT_PROXIMITY
enum {
	PROX_NORMAL = 0,
	PROX_CLOSE,
	PROX_OFF,
};
#endif
struct cnt_i2c_platform_data {
	int (*power) (int enable);
	int (*init) (void);
	int (*reset) (void);
};

struct CNT_TS_DATA_T {
    struct input_dev    *input_dev;
    struct work_struct     pen_event_work;
    struct workqueue_struct *ts_workqueue;
#ifdef CONFIG_HAS_EARLYSUSPEND	
    struct early_suspend early_suspend;
#endif
        struct cnt_i2c_platform_data   *platform_data;
    spinlock_t flag_lock;
    unsigned sleep_flags;
};

#define FLAG_IN_SLEEP       0x01
#define FLAG_EVENT_PENDING  0x02


#ifndef ABS_MT_TOUCH_MAJOR
#define ABS_MT_TOUCH_MAJOR    0x30    /* touching ellipse */
#define ABS_MT_TOUCH_MINOR    0x31    /* (omit if circular) */
#define ABS_MT_WIDTH_MAJOR    0x32    /* approaching ellipse */
#define ABS_MT_WIDTH_MINOR    0x33    /* (omit if circular) */
#define ABS_MT_ORIENTATION     0x34    /* Ellipse orientation */
#define ABS_MT_POSITION_X       0x35    /* Center X ellipse position */
#define ABS_MT_POSITION_Y       0x36    /* Center Y ellipse position */
#define ABS_MT_TOOL_TYPE        0x37    /* Type of touching device */
#define ABS_MT_BLOB_ID             0x38    /* Group set of pkts as blob */
#endif 


#endif

