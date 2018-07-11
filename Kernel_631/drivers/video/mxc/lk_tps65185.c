

/*
 * purpose : TPS65185 driver
 *
 * author : Gallen Lin
 * versions :
 *
 */


#include <linux/kernel.h>
//#include <linux/config.h>

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/wait.h>
#include <linux/semaphore.h>

#include <linux/input.h>

#ifdef CONFIG_MACH_MX6SL_NTX//[
	#define TPS65185_PLATFORM_MX6		1
#endif//]CONFIG_MACH_MX6SL_NTX


#include <mach/hardware.h>
#include <mach/gpio.h>

#ifdef TPS65185_PLATFORM_MX6//[
	#include <mach/iomux-mx6sl.h>
#else //][!TPS65185_PLATFORM_MX6
	#include <mach/iomux-mx50.h>
#endif //] TPS65185_PLATFORM_MX6

#include "ntx_hwconfig.h"

#define GDEBUG	0
#include <linux/gallen_dbg.h>

#include "lk_tps65185.h"

#define TPS65185_SUSPEND		1	

// command byte definitions ...

#define DRIVERNAME "TPS65185"
#define REG_UNKOWN_VAL	0xcc

#define TOTAL_CHIPS		1

//MODULE_LICENSE("GPL");

//#define TPS65185_EP3V3_PWROFF		1 // turn off the EP_3V3 when EPD ON/OFF .
#define TPS65185_PWR_ONOFF_INT	1
//#define TPS65185_PWR_ONOFF_WAITBYCOMPLETE		1

#define TPS65185_PWROFFDELAYWORK_TICKS		50
#define TPS65185_RESUME_EP3V3_ON					1

#define VCOM_ENABLE		1
#define VCOM_DISABLE	0

#ifdef TPS65185_PLATFORM_MX6//[

	#define GPIO_TPS65185_PWRUP			IMX_GPIO_NR(2,8) // EPDC_PWRCTRL1
	#define GPIO_TPS65185_PWRUP_PADCFG		MX6SL_PAD_EPDC_PWRCTRL1__GPIO_2_8	

	#define GPIO_TPS65185_WAKEUP		IMX_GPIO_NR(2,7) // EPDC_PWRCTRL0
	#define GPIO_TPS65185_WAKEUP_PADCFG		MX6SL_PAD_EPDC_PWRCTRL0__GPIO_2_7	

	#define GPIO_TPS65185_VCOMCTRL	IMX_GPIO_NR(2,3) // EPDC_VCOM0
	#define GPIO_TPS65185_VCOMCTRL_PADCFG		MX6SL_PAD_EPDC_VCOM0__GPIO_2_3	

	#define GPIO_TPS65185_VIN								IMX_GPIO_NR(2,14) // EPDC_PWRWAKEUP
	#define GPIO_TPS65185_VIN_PADCFG				MX6SL_PAD_EPDC_PWRWAKEUP__GPIO_2_14	

	#define GPIO_TPS65185_EP_3V3_IN								IMX_GPIO_NR(4,3) // EPDC_VDD 1.8V/3.3V .
	#define GPIO_TPS65185_EP_3V3_IN_PADCFG				MX6SL_PAD_KEY_ROW5__GPIO_4_3	

		//  GPIO input
	#define GPIO_TPS65185_PWRGOOD							IMX_GPIO_NR(2,13) // EPDC_PWRSTAT
	#define GPIO_TPS65185_PWRGOOD_INT_PADCFG			MX6SL_PAD_EPDC_PWRSTAT__GPIO_2_13_PUINT
	#define GPIO_TPS65185_PWRGOOD_GPIO_PADCFG			MX6SL_PAD_EPDC_PWRSTAT__GPIO_2_13

	#define GPIO_TPS65185_INT						IMX_GPIO_NR(2,9) // EPDC_PWRCTRL2
	#define GPIO_TPS65185_INT_PADCFG		 MX6SL_PAD_EPDC_PWRCTRL2__GPIO_2_9_PUINT
	#define GPIO_TPS65185_INT_GPIO_PADCFG		 MX6SL_PAD_EPDC_PWRCTRL2__GPIO_2_9

	//#define GPIO_TPS65185_SDA	
	//#define GPIO_TPS65185_SDL	
	#define set_irq_type(_irq,_irqtype)		irq_set_irq_type((_irq),(_irqtype))

#else //][!TPS65185_PLATFORM_MX6

//////////////////////////////////////////////////////////////
// definitions gpio ...
//  GPIO output
	#define GPIO_TPS65185_PWRUP		(2*32 + 30) /* GPIO_3_30 */
	#define GPIO_TPS65185_PWRUP_PADCFG			MX50_PAD_EPDC_PWRCTRL1__GPIO_3_30

	#define GPIO_TPS65185_WAKEUP	(2*32 + 29) /* GPIO_3_29 */
	#define GPIO_TPS65185_WAKEUP_PADCFG			MX50_PAD_EPDC_PWRCTRL0__GPIO_3_29

	#define GPIO_TPS65185_VCOMCTRL	(3*32 + 21)	/* GPIO_4_21 */
	#define GPIO_TPS65185_VCOMCTRL_PADCFG		MX50_PAD_EPDC_VCOM0__GPIO_4_21

	#define GPIO_TPS65185_VIN		(0*32 + 27)	/* GPIO_1_27 */
	#define GPIO_TPS65185_VIN_PADCFG				MX50_PAD_EIM_CRE__GPIO_1_27

	//  GPIO input
	#define GPIO_TPS65185_PWRGOOD	(2*32 + 28)	/* GPIO_3_28 */
	#define GPIO_TPS65185_PWRGOOD_INT_PADCFG			MX50_PAD_EPDC_PWRSTAT__GPIO_3_28_INT
	#define GPIO_TPS65185_PWRGOOD_GPIO_PADCFG			MX50_PAD_EPDC_PWRSTAT__GPIO_3_28

	#define GPIO_TPS65185_INT		(3*32 + 15)	/* GPIO_4_15 */
	#define GPIO_TPS65185_INT_PADCFG		 MX50_PAD_ECSPI1_SS0__GPIO_4_15_PUINT
	#define GPIO_TPS65185_INT_GPIO_PADCFG		 MX50_PAD_ECSPI1_SS0__GPIO_4_15

	//#define GPIO_TPS65185_SDA	(5*32+21) /* GPIO_6_21 */
	//#define GPIO_TPS65185_SDL	(5*32+20) /* GPIO_6_20 */
	//
#endif //] TPS65185_PLATFORM_MX6
///////////////////////////////////////////////////////
// definitions for config_epd_timing() ...

typedef struct tagTPS65185_VERSIONS{
	unsigned char bMajor;
	unsigned char bMinor;
	unsigned char bVersion;
	unsigned char bRevID;
} TPS65185_VERSIONS;

typedef struct tagTPS65185_data {
	int iCurrent_temprature;
	unsigned short wTempratureData;
	unsigned long dwCurrent_mode;// active , sleep , standby .
	TPS65185_VERSIONS t65185_versions;
	int iCurrentPwrupState;
	int iCurrentWakeupState;
	int iIsInitPwrON;// is first power on or not , if yes ,you must to delay 1.8ms for i2c protocol initial .
	//unsigned char bRegENABLE;
	//int iLast_temprature;
	struct semaphore i2clock;
	struct semaphore chmod_lock;
	int iCurrentVCOM;
	int iRestoreVCOM;
	unsigned long dwEP3V3_Off_Ticks;
} TPS65185_data;


typedef struct tagTPS65185_PWRDWN_WORK_PARAM {
	struct delayed_work pwrdwn_work;
	unsigned long dwNewMode;
	int iIsWaitPwrOff;
} TPS65185_PWRDWN_WORK_PARAM;

#define LKDRIVER_DATA_INIT(_iChipIdx)	\
{\
	gtTPS65185_DataA[_iChipIdx].iCurrent_temprature=-1;\
	gtTPS65185_DataA[_iChipIdx].wTempratureData=0;\
	gtTPS65185_DataA[_iChipIdx].dwCurrent_mode=TPS65185_MODE_UNKOWN;\
	gtTPS65185_DataA[_iChipIdx].iCurrentPwrupState=-1;\
	gtTPS65185_DataA[_iChipIdx].iCurrentWakeupState=-1;\
	gtTPS65185_DataA[_iChipIdx].iIsInitPwrON=1;\
	sema_init(&gtTPS65185_DataA[_iChipIdx].i2clock,1);\
	sema_init(&gtTPS65185_DataA[_iChipIdx].chmod_lock,1);\
}

// externals ...
extern volatile NTX_HWCONFIG *gptHWCFG;
extern volatile int gSleep_Mode_Suspend;


// 
static TPS65185_PWRDWN_WORK_PARAM gtPwrdwn_work_param;

static struct i2c_adapter *gpI2C_adapter = 0;
static struct i2c_client *gpI2C_clientA[TOTAL_CHIPS] = {0,};
volatile static int giIsTPS65185_inited=0;
volatile static int giIsTPS65185_gpio_inited=0;
volatile static int giIsTPS65185_turnoff_EP3V3=0;
volatile static unsigned long gdwSafeTick_To_TurnON_RailPower;
volatile static unsigned long gdwSafeTick_To_TurnOFF_EP3V3;
volatile static int giTPS65185_int_state=0;
static TPS65185_INTEVT_CB *gpfnINTCB=0;

static struct i2c_board_info gtTPS65185_BIA[TOTAL_CHIPS] = {
	{
	 .type = "tps65185-1",
	 .addr = 0x68,
	 .platform_data = NULL,
	 },
};


static const unsigned short gwTPS65185_AddrA[] = {
	0x68,
	I2C_CLIENT_END
};


static TPS65185_data gtTPS65185_DataA[TOTAL_CHIPS] = {
	{-1,0x0000,TPS65185_MODE_UNKOWN,},
};


extern void ntx_report_event(unsigned int type, unsigned int code, int value);

//////////////////////////////////////////////////////////////////////////
//
// internal hardware helper ...
//



// registers (write)....
#define TPS65185_REG_ENABLE_ACTIVE		0x80
#define TPS65185_REG_ENABLE_STANDBY		0x40
#define TPS65185_REG_ENABLE_V3P3_EN		0x20
#define TPS65185_REG_ENABLE_VCOM_EN		0x10
#define TPS65185_REG_ENABLE_VDDH_EN		0x08
#define TPS65185_REG_ENABLE_VPOS_EN		0x04
#define TPS65185_REG_ENABLE_VEE_EN		0x02
#define TPS65185_REG_ENABLE_VNEG_EN		0x01
#define TPS65185_REG_ENABLE_ALL			0xff
static volatile unsigned char gbTPS65185_REG_ENABLE=0; // default reset value is zero .
static const unsigned char gbTPS65185_REG_ENABLE_addr=0x01;

static volatile unsigned char gbTPS65185_REG_VADJ=0x23; // 15V .
static const unsigned char gbTPS65185_REG_VADJ_addr=0x02;


#define TPS65185_REG_VCOM1_ALL			0xff
static volatile unsigned char gbTPS65185_REG_VCOM1=0x7d; // .
static const unsigned char gbTPS65185_REG_VCOM1_addr=0x03;


#define TPS65185_REG_VCOM2_ACQ			0x80
#define TPS65185_REG_VCOM2_PROG			0x40
#define TPS65185_REG_VCOM2_HiZ			0x20
//#define TPS65185_REG_VCOM2_AVG			0x18
#define TPS65185_REG_VCOM2_VCOM8		0x01
#define TPS65185_REG_VCOM2_ALL			0xff
static volatile unsigned char gbTPS65185_REG_VCOM2=0x04; // .
static const unsigned char gbTPS65185_REG_VCOM2_addr=0x04;

//#define TPS65185_REG_INT_EN1_DTX_EN			0x80
#define TPS65185_REG_INT_EN1_TSD_EN					0x40
#define TPS65185_REG_INT_EN1_HOT_EN					0x20
#define TPS65185_REG_INT_EN1_TMST_HOT_EN			0x10
#define TPS65185_REG_INT_EN1_TMST_COLD_EN			0x08
#define TPS65185_REG_INT_EN1_UVLO_EN			0x04
#define TPS65185_REG_INT_EN1_ACQC_EN			0x02
#define TPS65185_REG_INT_EN1_PRGC_EN			0x01
#define TPS65185_REG_INT_EN1_ALL			0xff
static volatile unsigned char gbTPS65185_REG_INT_EN1=0x7f; // .
static const unsigned char gbTPS65185_REG_INT_EN1_addr=0x05;

#define TPS65185_REG_INT_EN2_VBUVEN				0x80
#define TPS65185_REG_INT_EN2_VDDHUVEN			0x40
#define TPS65185_REG_INT_EN2_VNUV_EN			0x20
#define TPS65185_REG_INT_EN2_VPOSUVEN			0x10
#define TPS65185_REG_INT_EN2_VEEUVEN			0x08
#define TPS65185_REG_INT_EN2_VCOMFEN			0x04
#define TPS65185_REG_INT_EN2_VNEGUVEN			0x02
#define TPS65185_REG_INT_EN2_EOCEN				0x01
#define TPS65185_REG_INT_EN2_ALL			0xff
static volatile unsigned char gbTPS65185_REG_INT_EN2=0xff; // .
static const unsigned char gbTPS65185_REG_INT_EN2_addr=0x06;

#define TPS65185_REG_INT1_ACQC			0x02
#define TPS65185_REG_INT1_PRGC			0x01
#define TPS65185_REG_INT1_UVLO			0x04
#define TPS65185_REG_INT1_HOT				0x20
#define TPS65185_REG_INT1_TSD				0x40
static volatile unsigned char gbTPS65185_REG_INT1=0x0; // .
static const unsigned char gbTPS65185_REG_INT1_addr=0x07;

#define TPS65185_REG_INT2_VB_UV			0x80
#define TPS65185_REG_INT2_VDDH_UV		0x40
#define TPS65185_REG_INT2_VN_UV			0x20
#define TPS65185_REG_INT2_VPOS_UV		0x10
#define TPS65185_REG_INT2_VEE_UV		0x08
#define TPS65185_REG_INT2_VCOMF			0x04
#define TPS65185_REG_INT2_VNEG_UV		0x02
#define TPS65185_REG_INT2_EOC				0x01
static volatile unsigned char gbTPS65185_REG_INT2=0x0; // .
static const unsigned char gbTPS65185_REG_INT2_addr=0x08;



#define TPS65185_REG_UPSEQ0_ALL			0xff
static volatile unsigned char gbTPS65185_REG_UPSEQ0=0xe4; // .
static const unsigned char gbTPS65185_REG_UPSEQ0_addr=0x09;

#define TPS65185_REG_UPSEQ1_ALL			0xff
static volatile unsigned char gbTPS65185_REG_UPSEQ1=0x55; // .
static const unsigned char gbTPS65185_REG_UPSEQ1_addr=0x0a;


#define TPS65185_REG_DWNSEQ0_ALL			0xff
static volatile unsigned char gbTPS65185_REG_DWNSEQ0=0x1e; // .
static const unsigned char gbTPS65185_REG_DWNSEQ0_default=0x1e; // .
//static volatile unsigned char gbTPS65185_REG_DWNSEQ0=0x1b; // .
static const unsigned char gbTPS65185_REG_DWNSEQ0_addr=0x0b;

static volatile unsigned char gbTPS65185_REG_DWNSEQ1=0xe0; // .
static const unsigned char gbTPS65185_REG_DWNSEQ1_addr=0x0c;

#define TPS65185_REG_TMST1_READ_THERM			0x80
#define TPS65185_REG_TMST1_CONV_END				0x20
static volatile unsigned char gbTPS65185_REG_TMST1=0x20; // .
static const unsigned char gbTPS65185_REG_TMST1_addr=0x0d;

//static unsigned char gbTPS65185_REG_TMST2=0x78; // .
//static const unsigned char gbTPS65185_REG_TMST2_addr=0x0e;

// registers (read)....
static volatile unsigned char gbTPS65185_REG_TMST_VALUE=0; //
static const unsigned char gbTPS65185_REG_TMST_VALUE_addr=0x00;

static volatile unsigned char gbTPS65185_REG_PG=0;
static const unsigned char gbTPS65185_REG_PG_addr=0x0f;

static volatile unsigned char gbTPS65185_REG_REVID=0x45; // default is TPS65185 1p0 .
static const unsigned char gbTPS65185_REG_REVID_addr=0x10;



static int tps65185_set_reg(unsigned char bRegAddr,unsigned char bRegSetVal)
{
	int iRet=TPS65185_RET_SUCCESS;
	int iChk;
	unsigned char bA[2] ;
	//int irq_INT,irq_PG;

#if 0
	ASSERT(gpI2C_adapter);
	ASSERT(gpI2C_clientA[0]);
#else
	if(!gpI2C_adapter) {
		WARNING_MSG("%s gpI2C_adapter null \n",__FUNCTION__);
		return (TPS65185_RET_PARAMERR);
	}
	if(!gpI2C_clientA[0]) { 
		WARNING_MSG("%s gpI2C_clientA[0] null \n",__FUNCTION__);
		return (TPS65185_RET_PARAMERR);
	}
#endif
	//irq_INT = gpio_to_irq(GPIO_TPS65185_INT);
	//irq_PG = gpio_to_irq(GPIO_TPS65185_PWRGOOD);

	//disable_irq(irq_INT);
	//disable_irq(irq_PG);
	
	down(&gtTPS65185_DataA[0].i2clock);
	bA[0]=bRegAddr;
	bA[1]=bRegSetVal;
	iChk = i2c_master_send(gpI2C_clientA[0], (const char *)bA, sizeof(bA));
	if (iChk < 0) {
		ERR_MSG("%s(%d):%d=%s(),regAddr=0x%x,regVal=0x%x fail !\n",__FILE__,__LINE__,\
			iChk,"i2c_master_send",bRegAddr,bRegSetVal);
		iRet=TPS65185_RET_I2CTRANS_ERR;
	}
	up(&gtTPS65185_DataA[0].i2clock);
	//enable_irq(irq_PG);
	//enable_irq(irq_INT);
	return iRet;
}

static int tps65185_get_reg(unsigned char bRegAddr,unsigned char  *O_pbRegVal)
{
	int iRet=TPS65185_RET_SUCCESS;
	int iChk;
	unsigned char bA[1] ;
	//int irq_INT , irq_PG;

#if 0
	ASSERT(gpI2C_adapter);
	ASSERT(gpI2C_clientA[0]);
#else
	if(!gpI2C_adapter) {
		WARNING_MSG("%s gpI2C_adapter null \n",__FUNCTION__);
		return (TPS65185_RET_PARAMERR);
	}
	if(!gpI2C_clientA[0]) { 
		WARNING_MSG("%s gpI2C_clientA[0] null \n",__FUNCTION__);
		return (TPS65185_RET_PARAMERR);
	}
#endif

	ASSERT(O_pbRegVal);

	bA[0]=bRegAddr;

		
	if(!in_interrupt()) {
		
		//irq_INT = gpio_to_irq(GPIO_TPS65185_INT);
		//irq_PG = gpio_to_irq(GPIO_TPS65185_PWRGOOD);

		//disable_irq(irq_INT);
		//disable_irq(irq_PG);
		down(&gtTPS65185_DataA[0].i2clock);	
	}

	iChk = i2c_master_send(gpI2C_clientA[0], (const char *)bA, 1);
	if (iChk < 0) {
		ERR_MSG("%s(%d):%s i2c_master_send fail !\n",__FILE__,__LINE__,__FUNCTION__);
		iRet = TPS65185_RET_I2CTRANS_ERR;
	}
	

	iChk = i2c_master_recv(gpI2C_clientA[0], bA, 1);
	if (iChk < 0) {
		ERR_MSG("%s(%d):%s i2c_master_recv fail !\n",__FILE__,__LINE__,__FUNCTION__);
		iRet = TPS65185_RET_I2CTRANS_ERR;
	}

	if(!in_interrupt()) {
		up(&gtTPS65185_DataA[0].i2clock);
		//enable_irq(irq_PG);
		//enable_irq(irq_INT);
	}

	if(iRet>=0) {
		*O_pbRegVal = bA[0];
	}

	return iRet;
}


#define TPS65185_REG_SET(_regName,_bFieldName,_bSetVal)		\
({\
	int _iRet=TPS65185_RET_SUCCESS;\
	int _iChk;\
	unsigned char _bNewReg,_bFieldMask;\
	\
	_bFieldMask=(unsigned char)TPS65185_REG_##_regName##_##_bFieldName;\
	if(0xff==_bFieldMask) {\
		_bNewReg = _bSetVal;\
	}\
	else {\
		_bNewReg=gbTPS65185_REG_##_regName;\
		if(_bSetVal) {\
			_bNewReg |= _bFieldMask ;\
		}\
		else {\
			_bNewReg &= ~_bFieldMask;\
		}\
	}\
	\
	_iChk = tps65185_set_reg(gbTPS65185_REG_##_regName##_##addr,_bNewReg);\
	if(_iChk<0) {\
		_iRet = _iChk;\
	}\
	else {\
		DBG_MSG("%s() : tps65185 write reg%s(%02Xh) 0x%02x->0x%02x\n",__FUNCTION__,\
		#_regName,gbTPS65185_REG_##_regName##_##addr,gbTPS65185_REG_##_regName,_bNewReg);\
		gbTPS65185_REG_##_regName = _bNewReg;\
	}\
	_iRet;\
})

#define TPS65185_REG_GET(_regName)		\
({\
	int _iChk;\
	unsigned char bReadReg=0;\
	unsigned short _wRet=0;\
	\
	_iChk = tps65185_get_reg(gbTPS65185_REG_##_regName##_##addr,&bReadReg);\
	if(_iChk<0) {\
		_wRet = (unsigned short)(-1);\
	}\
	else {\
		_wRet = bReadReg;\
		gbTPS65185_REG_##_regName = bReadReg;\
		DBG_MSG("%s() : tps65185 read reg%s(%02Xh)=0x%02x\n",__FUNCTION__,\
			#_regName,gbTPS65185_REG_##_regName##_##addr,bReadReg);\
	}\
	_wRet;\
})

#define TPS65185_REG(_regName)	gbTPS65185_REG_##_regName

#if 0 //[
#define TPS65185_VCOM_OUT(_out_val)	\
	if(_out_val) { \
		gpio_direction_output(GPIO_TPS65185_VCOMCTRL, VCOM_ENABLE); \
	} \
	else { \
		gpio_direction_output(GPIO_TPS65185_VCOMCTRL, VCOM_DISABLE); \
	}
#else //][!
#define TPS65185_VCOM_OUT(_out_val)	\
	if(_out_val) { \
		unsigned char bReg;\
		gpio_direction_output(GPIO_TPS65185_VCOMCTRL, VCOM_ENABLE); \
		bReg = (unsigned char)TPS65185_REG_GET(ENABLE);\
		if(!(bReg&TPS65185_REG_ENABLE_VCOM_EN)) {\
			WARNING_MSG("[WARNING] %s():ENABLE=0x%x, VOM_EN=0 but gpio=1\n",__FUNCTION__,bReg);\
			TPS65185_REG_SET(ENABLE,VCOM_EN,1);\
		}\
	} \
	else { \
		gpio_direction_output(GPIO_TPS65185_VCOMCTRL, VCOM_DISABLE); \
	}
#endif //]

DECLARE_WAIT_QUEUE_HEAD(tps65185_ACQC_WQ);
DECLARE_WAIT_QUEUE_HEAD(tps65185_PRGC_WQ);


static struct work_struct tps65185_int_work;
static struct workqueue_struct *tps65185_int_workqueue;

static void tps65185_int_func(struct work_struct *work)
{
	unsigned char bRegINT1,bRegINT2;
	unsigned short wReg;
	unsigned long dwTPS65185_mode;

	int iForceStanbyState=0;
	int iChk;

	
	wReg = TPS65185_REG_GET(INT1);
	if(((unsigned short)(-1))==wReg) {
		ERR_MSG("%s(%d):%s regINT1 read fail !\n",__FILE__,__LINE__,__FUNCTION__);
	}
	bRegINT1=(unsigned char)wReg;
	
	wReg = TPS65185_REG_GET(INT2);
	if(((unsigned short)(-1))==wReg) {
		ERR_MSG("%s(%d):%s regINT2 read fail !\n",__FILE__,__LINE__,__FUNCTION__);
	}
	bRegINT2=(unsigned char)wReg;
	
	if(bRegINT1&TPS65185_REG_INT1_ACQC) {
		wake_up_all(&tps65185_ACQC_WQ);
	}
	
	if(bRegINT1&TPS65185_REG_INT1_PRGC) {
		wake_up_all(&tps65185_PRGC_WQ);
	}

	if(bRegINT1&TPS65185_REG_INT1_UVLO) {
		ERR_MSG("%s(%d):%s input voltage is below UVLO threshold !\n",__FILE__,__LINE__,__FUNCTION__);
		giTPS65185_int_state=MSC_RAW_EPD_UNKOWN_ERROR;
#ifdef TPS65185_VDROP_PROC_IN_KERNEL//[
#else //][!TPS65185_VDROP_PROC_IN_KERNEL
		ntx_report_event(EV_MSC,MSC_RAW,MSC_RAW_EPD_UNKOWN_ERROR);
#endif //]TPS65185_VDROP_PROC_IN_KERNEL
	}
	if(bRegINT1&TPS65185_REG_INT1_TSD) {
		ERR_MSG("%s(%d):%s chip is over-temperature shutdown !\n",__FILE__,__LINE__,__FUNCTION__);
		giTPS65185_int_state=MSC_RAW_EPD_OVT_ERROR;
#ifdef TPS65185_VDROP_PROC_IN_KERNEL//[
#else //][!TPS65185_VDROP_PROC_IN_KERNEL
		ntx_report_event(EV_MSC,MSC_RAW,MSC_RAW_EPD_OVT_ERROR);
#endif //]TPS65185_VDROP_PROC_IN_KERNEL
		iForceStanbyState = 1;
	}
	if(bRegINT1&TPS65185_REG_INT1_HOT) {
		ERR_MSG("%s(%d):%s chip is approaching over-temperature shutdown !\n",__FILE__,__LINE__,__FUNCTION__);
		giTPS65185_int_state=MSC_RAW_EPD_OVT_ERROR;
#ifdef TPS65185_VDROP_PROC_IN_KERNEL//[
#else //][!TPS65185_VDROP_PROC_IN_KERNEL
		ntx_report_event(EV_MSC,MSC_RAW,MSC_RAW_EPD_OVT_ERROR);
#endif //]TPS65185_VDROP_PROC_IN_KERNEL
	}

	if(bRegINT2&TPS65185_REG_INT2_VB_UV) {
		ERR_MSG("%s(%d):%s under-voltage on DCDC1 detected !\n",__FILE__,__LINE__,__FUNCTION__);
		giTPS65185_int_state=MSC_RAW_EPD_DCDC1_ERROR;

#ifdef TPS65185_VDROP_PROC_IN_KERNEL//[
#else //][!TPS65185_VDROP_PROC_IN_KERNEL
		ntx_report_event(EV_MSC,MSC_RAW,MSC_RAW_EPD_DCDC1_ERROR);
#endif //]TPS65185_VDROP_PROC_IN_KERNEL

		iForceStanbyState = 1;
	}
	if(bRegINT2&TPS65185_REG_INT2_VDDH_UV) {
		ERR_MSG("%s(%d):%s under-voltage on VDDH charge pump detected !\n",__FILE__,__LINE__,__FUNCTION__);
		giTPS65185_int_state=MSC_RAW_EPD_VDDH_ERROR;
#ifdef TPS65185_VDROP_PROC_IN_KERNEL//[
#else //][!TPS65185_VDROP_PROC_IN_KERNEL
		ntx_report_event(EV_MSC,MSC_RAW,MSC_RAW_EPD_VDDH_ERROR);
#endif //]TPS65185_VDROP_PROC_IN_KERNEL
		
	}
	if(bRegINT2&TPS65185_REG_INT2_VN_UV) {
		ERR_MSG("%s(%d):%s under-voltage on DCDC2 detected !\n",__FILE__,__LINE__,__FUNCTION__);
		giTPS65185_int_state=MSC_RAW_EPD_DCDC2_ERROR;
#ifdef TPS65185_VDROP_PROC_IN_KERNEL//[
#else //][!TPS65185_VDROP_PROC_IN_KERNEL
		ntx_report_event(EV_MSC,MSC_RAW,MSC_RAW_EPD_DCDC2_ERROR);
#endif //]TPS65185_VDROP_PROC_IN_KERNEL
		iForceStanbyState = 1;
	}
	if(bRegINT2&TPS65185_REG_INT2_VPOS_UV) {
		ERR_MSG("%s(%d):%s under-voltage on LDO1(VPOS) detected !\n",__FILE__,__LINE__,__FUNCTION__);
		giTPS65185_int_state=MSC_RAW_EPD_VPOS_ERROR;
#ifdef TPS65185_VDROP_PROC_IN_KERNEL//[
#else //][!TPS65185_VDROP_PROC_IN_KERNEL
		ntx_report_event(EV_MSC,MSC_RAW,MSC_RAW_EPD_VPOS_ERROR);
#endif //]TPS65185_VDROP_PROC_IN_KERNEL
	}
	if(bRegINT2&TPS65185_REG_INT2_VEE_UV) {
		ERR_MSG("%s(%d):%s under-voltage on VEE charge pump detected !\n",__FILE__,__LINE__,__FUNCTION__);
		giTPS65185_int_state=MSC_RAW_EPD_VEE_ERROR;
#ifdef TPS65185_VDROP_PROC_IN_KERNEL//[
#else //][!TPS65185_VDROP_PROC_IN_KERNEL
		ntx_report_event(EV_MSC,MSC_RAW,MSC_RAW_EPD_VEE_ERROR);
#endif //]TPS65185_VDROP_PROC_IN_KERNEL
	}
	if(bRegINT2&TPS65185_REG_INT2_VCOMF) {
		ERR_MSG("%s(%d):%s fault on VCOM detected !\n",__FILE__,__LINE__,__FUNCTION__);
		giTPS65185_int_state=MSC_RAW_EPD_VCOM_ERROR;
#ifdef TPS65185_VDROP_PROC_IN_KERNEL//[
#else //][!TPS65185_VDROP_PROC_IN_KERNEL
		ntx_report_event(EV_MSC,MSC_RAW,MSC_RAW_EPD_VCOM_ERROR);
#endif //]TPS65185_VDROP_PROC_IN_KERNEL
	}
	if(bRegINT2&TPS65185_REG_INT2_VNEG_UV) {
		ERR_MSG("%s(%d):%s under-voltage on LDO2(VNEG) detected !\n",__FILE__,__LINE__,__FUNCTION__);
		giTPS65185_int_state=MSC_RAW_EPD_VNEG_ERROR;
#ifdef TPS65185_VDROP_PROC_IN_KERNEL//[
#else //][!TPS65185_VDROP_PROC_IN_KERNEL
		ntx_report_event(EV_MSC,MSC_RAW,MSC_RAW_EPD_VNEG_ERROR);
#endif //]TPS65185_VDROP_PROC_IN_KERNEL
		iForceStanbyState = 1;
	}

	if(iForceStanbyState) {

#if 1 //[
		//tps65185_ONOFF(0);
		//tps65185_ONOFF(1);
		gpio_direction_output(GPIO_TPS65185_PWRUP, 0);
		gtTPS65185_DataA[0].dwCurrent_mode=TPS65185_MODE_STANDBY;
#else //][!
		down(&gtTPS65185_DataA[0].chmod_lock);
		gpio_direction_output(GPIO_TPS65185_PWRUP, 0);
		gpio_direction_output(GPIO_TPS65185_WAKEUP, 0);
		gtTPS65185_DataA[0].dwCurrent_mode=TPS65185_MODE_SLEEP;
		up(&gtTPS65185_DataA[0].chmod_lock);
#endif //]

	}

	if(	gpfnINTCB && iForceStanbyState) {
		gpfnINTCB(giTPS65185_int_state);
	}
#if 0
	if(bRegINT2&TPS65185_REG_INT2_EOC) {
		ERR_MSG("%s(%d):%s ADC conversion is complete (temperature acquisition is complete !\n",__FILE__,__LINE__,__FUNCTION__);
	}
#endif 
	
	//DBG0_MSG("%s() : INT1=0x%x,INT2=0x%x\n",__FUNCTION__,bRegINT1,bRegINT2);
}


static irqreturn_t tps65185_int(int irq, void *dev_id)
{
	DBG_MSG("[%s-%d] tps65185 interrupt triggered !!!\n",__func__,__LINE__);
	queue_work(tps65185_int_workqueue,&tps65185_int_work);

	return 0;
}


#ifdef TPS65185_PWR_ONOFF_WAITBYCOMPLETE//[

DECLARE_COMPLETION(tps65185_pwrgood_on_completion);
DECLARE_COMPLETION(tps65185_pwrgood_off_completion);
#else //][!TPS65185_PWR_ONOFF_WAITBYCOMPLETE
DECLARE_WAIT_QUEUE_HEAD(tps65185_pwron_wq);
DECLARE_WAIT_QUEUE_HEAD(tps65185_pwroff_wq);
#endif //] TPS65185_PWR_ONOFF_WAITBYCOMPLETE

static struct work_struct tps65185_pwrgood_work;
static struct workqueue_struct *tps65185_pwrgood_workqueue;
volatile int giIsTPS65185_PwrOn=0;
static void tps65185_pwrgood_func(struct work_struct *work)
{
	int iIsPwrOn;

	#if 0
	{
		unsigned char bRegPG;
		unsigned short wReg;

		wReg = TPS65185_REG_GET(PG);
		if(((unsigned short)(-1))==wReg) {
			ERR_MSG("%s(%d):%s regPG read fail !\n",__FILE__,__LINE__,__FUNCTION__);
		}
		else {
			bRegPG=(unsigned char)wReg;
			DBG_MSG("%s() : PG=0x%x \n",__FUNCTION__,bRegPG);
		}
	}
	#endif
	
	iIsPwrOn=giIsTPS65185_PwrOn=gpio_get_value(GPIO_TPS65185_PWRGOOD);
	DBG_MSG("%s() : powergood signal=%d \n",__FUNCTION__,iIsPwrOn);
	
	if(iIsPwrOn) {
#ifdef TPS65185_PWR_ONOFF_WAITBYCOMPLETE//[

		complete_all(&tps65185_pwrgood_on_completion);
#else //][!TPS65185_PWR_ONOFF_WAITBYCOMPLETE
		wake_up_interruptible_all(&tps65185_pwron_wq);
#endif //] TPS65185_PWR_ONOFF_WAITBYCOMPLETE
	}
	else{
#ifdef TPS65185_PWR_ONOFF_WAITBYCOMPLETE//[

		complete_all(&tps65185_pwrgood_off_completion);
#else//][!TPS65185_PWR_ONOFF_WAITBYCOMPLETE
		wake_up_interruptible_all(&tps65185_pwroff_wq);
#endif//] TPS65185_PWR_ONOFF_WAITBYCOMPLETE
	}
}


static irqreturn_t tps65185_pwrgood_inthandler(int irq, void *dev_id)
{
	DBG_MSG("[%s-%d] tps65185 pwrgood interrupt triggered !!!\n",__func__,__LINE__);
	//queue_work(tps65185_pwrgood_workqueue,&tps65185_pwrgood_work);
	//
	tps65185_pwrgood_func(&tps65185_pwrgood_work);
	return 0;
}

static void _tps65185_pwrdwn(void)
{
	unsigned long dwCurrentMode,dwNewMode;
	int iIsWaitPwrOff;

	// parameters setup ...
	dwCurrentMode = gtTPS65185_DataA[0].dwCurrent_mode;
	dwNewMode = gtPwrdwn_work_param.dwNewMode ;
	iIsWaitPwrOff = gtPwrdwn_work_param.iIsWaitPwrOff;

	DBG_MSG("%s : mode=%ld begin\n",__FUNCTION__,dwNewMode);

	ASSERT(giIsTPS65185_inited);

	if(dwCurrentMode==dwNewMode) {
		DBG_MSG("%s : skip same mode\n",__FUNCTION__);
		goto exit ;
	}

	gpio_direction_output(GPIO_TPS65185_PWRUP, 0);
	if( TPS65185_MODE_ACTIVE==dwCurrentMode) {
		if(3==gptHWCFG->m_val.bUIConfig) {
			// MP/RD mode .
			gdwSafeTick_To_TurnOFF_EP3V3 = jiffies+0;
		}
		else {
			gdwSafeTick_To_TurnOFF_EP3V3 = jiffies+ \
				gtTPS65185_DataA[0].dwEP3V3_Off_Ticks;
		}
		if(iIsWaitPwrOff) {
			tps65185_wait_panel_poweroff();
		}
	}


	if( dwNewMode == TPS65185_MODE_SLEEP) {
		if( giIsTPS65185_turnoff_EP3V3 ) 
		{
			// Turn off EV_3V3 ...
			//TPS65185_REG_SET(ENABLE,V3P3_EN,0);
#ifdef GPIO_TPS65185_EP_3V3_IN //[
			//udelay(10);gpio_direction_output(GPIO_TPS65185_EP_3V3_IN, 0);
#endif //]GPIO_TPS65185_EP_3V3_IN
		}
		gpio_direction_output(GPIO_TPS65185_WAKEUP, 0);
	}

	gtTPS65185_DataA[0].dwCurrent_mode = dwNewMode;

exit:
	DBG_MSG("%s : mode=%ld end .\n",__FUNCTION__,dwNewMode);
}

static void tps65185_pwrdwn_work_func(struct work_struct *work)
{
	down(&gtTPS65185_DataA[0].chmod_lock);
	_tps65185_pwrdwn();
	up(&gtTPS65185_DataA[0].chmod_lock);
}



//
static int tps65185_gpio_init(void)
{
	int iRet=TPS65185_RET_SUCCESS;
	int iChk;
	int irq;

	if(giIsTPS65185_gpio_inited) {
		return TPS65185_RET_SUCCESS;
	}

	// inputs
	mxc_iomux_v3_setup_pad(GPIO_TPS65185_PWRGOOD_INT_PADCFG);
	gpio_request(GPIO_TPS65185_PWRGOOD, "tps65185_PWRGOOD");
	gpio_direction_input(GPIO_TPS65185_PWRGOOD);
	//giIsTPS65185_PwrOn=gpio_get_value(GPIO_TPS65185_PWRGOOD);

#ifdef TPS65185_PWR_ONOFF_INT//[

	tps65185_pwrgood_workqueue=create_singlethread_workqueue("tps65185_PWRGOOD");
	INIT_WORK(&tps65185_pwrgood_work, tps65185_pwrgood_func);

	irq = gpio_to_irq(GPIO_TPS65185_PWRGOOD);
	set_irq_type(irq, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING);//
	//disable_irq_wake(irq);
#endif //] TPS65185_PWR_ONOFF_INT


	
	mxc_iomux_v3_setup_pad(GPIO_TPS65185_INT_PADCFG);
	gpio_request(GPIO_TPS65185_INT, "tps65185_INT");
	gpio_direction_input (GPIO_TPS65185_INT);
	
	tps65185_int_workqueue=create_singlethread_workqueue("tps65185_INT");
	INIT_WORK(&tps65185_int_work, tps65185_int_func);

	irq = gpio_to_irq(GPIO_TPS65185_INT);
	set_irq_type(irq, IRQF_TRIGGER_FALLING);//IRQF_TRIGGER_RISING|
	//disable_irq_wake(irq);


	// outputs
#ifdef GPIO_TPS65185_EP_3V3_IN //[
	mxc_iomux_v3_setup_pad(GPIO_TPS65185_EP_3V3_IN_PADCFG);
	gpio_request(GPIO_TPS65185_EP_3V3_IN, "tps65185_EP_3V3_IN");
	gpio_direction_output(GPIO_TPS65185_EP_3V3_IN, 1);
#endif //]GPIO_TPS65185_EP_3V3_IN
	gdwSafeTick_To_TurnOFF_EP3V3 = jiffies;

	mxc_iomux_v3_setup_pad(GPIO_TPS65185_VIN_PADCFG);
	gpio_request(GPIO_TPS65185_VIN, "tps65185_VIN");
	gpio_direction_output(GPIO_TPS65185_VIN, 1);
	mdelay(5);
	gdwSafeTick_To_TurnON_RailPower = jiffies+4;


	mxc_iomux_v3_setup_pad(GPIO_TPS65185_PWRUP_PADCFG);
	gpio_request(GPIO_TPS65185_PWRUP, "tps65185_PWRUP");
	gpio_direction_output(GPIO_TPS65185_PWRUP, 0);

	mxc_iomux_v3_setup_pad(GPIO_TPS65185_WAKEUP_PADCFG);
	gpio_request(GPIO_TPS65185_WAKEUP, "tps65185_WAKEUP");
	gpio_direction_output(GPIO_TPS65185_WAKEUP, 1);
	mdelay(2);
	gtTPS65185_DataA[0].dwCurrent_mode=TPS65185_MODE_STANDBY;

	mxc_iomux_v3_setup_pad(GPIO_TPS65185_VCOMCTRL_PADCFG);
	gpio_request(GPIO_TPS65185_VCOMCTRL, "tps65185_VCOMCTRL");
	gpio_direction_output(GPIO_TPS65185_VCOMCTRL, VCOM_DISABLE);

	giIsTPS65185_gpio_inited=1;

	return iRet;
}


static int tps65185_gpio_release(void)
{
	int iRet=TPS65185_RET_SUCCESS;
	int irq;
	

	// release gpios ...
	

#if 0
#ifdef TPS65185_PWR_ONOFF_INT//[

	irq = gpio_to_irq(GPIO_TPS65185_PWRGOOD);
	free_irq(irq,0);
	flush_workqueue(tps65185_pwrgood_workqueue);
	destroy_workqueue(tps65185_pwrgood_workqueue);

#endif //] TPS65185_PWR_ONOFF_INT

	irq = gpio_to_irq(GPIO_TPS65185_INT);
	free_irq(irq,0);
	flush_workqueue(tps65185_int_workqueue);
	destroy_workqueue(tps65185_int_workqueue);

	if(giIsTPS65185_gpio_inited) {
		gpio_free(GPIO_TPS65185_PWRGOOD);
		gpio_free(GPIO_TPS65185_INT);
		gpio_free(GPIO_TPS65185_VIN);
		gpio_free(GPIO_TPS65185_PWRUP);
		gpio_free(GPIO_TPS65185_WAKEUP);
		gpio_free(GPIO_TPS65185_VCOMCTRL);
	}
#endif

	return iRet;
}







//////////////////////////////////////////////////////////////////////////
//
// internal helper ...
//



static int tps65185_chk_PG(unsigned char I_bChkMask)
{
	int iRet;
	unsigned short wReg;
	unsigned char bReg;


	wReg = TPS65185_REG_GET(PG);

	if(((unsigned short)(-1))==wReg) {
		return TPS65185_RET_REGREADFAIL;
	}

	bReg=(unsigned char)wReg;
	//bChkMask=0xfa;

	bReg &= ~I_bChkMask;
	if(I_bChkMask==bReg) {
		iRet = TPS65185_RET_ALLPOWERGOOD;
	}
	else {
		//printk(KERN_ERR "PG reg=0x%02X,Chk=0x02%X\n",bReg,I_bChkMask);
		iRet = TPS65185_RET_POWERNOTGOOD;
	}

	return iRet;
}



static int tps65185_get_versions(TPS65185_VERSIONS *O_pt65185ver)
{
	int iRet=TPS65185_RET_SUCCESS;
	int iChk;
	unsigned short wReg;
	unsigned char bReg;

	ASSERT(gpI2C_adapter);
	ASSERT(O_pt65185ver);

	wReg = TPS65185_REG_GET(REVID);

	if(((unsigned short)(-1))==wReg) {
		return TPS65185_RET_REGREADFAIL;
	}

	bReg=(unsigned char)wReg;
	O_pt65185ver->bMajor = (bReg>>6)&0x3;
	O_pt65185ver->bMinor  = (bReg>>4)&0x3;
	O_pt65185ver->bVersion = (bReg)&0xf;
	O_pt65185ver->bRevID = bReg;

	return iRet;
}



static int tps65185_config_epd_timing(int iEPDTimingType)
{
	int iRet=TPS65185_RET_SUCCESS;


	return iRet;
}

static int tps65185_reg_init(int I_iIsEP_3V3_ON)
{
	int iRet=TPS65185_RET_SUCCESS;
	unsigned char bRegVal;

	GALLEN_DBGLOCAL_BEGIN();

	bRegVal = TPS65185_REG_ENABLE_VCOM_EN|TPS65185_REG_ENABLE_VDDH_EN|\
			TPS65185_REG_ENABLE_VPOS_EN|TPS65185_REG_ENABLE_VEE_EN|TPS65185_REG_ENABLE_VNEG_EN;
	if(1==I_iIsEP_3V3_ON) {
		GALLEN_DBGLOCAL_RUNLOG(0);
		bRegVal |= TPS65185_REG_ENABLE_V3P3_EN;
	}
	else if(0==I_iIsEP_3V3_ON) {
		GALLEN_DBGLOCAL_RUNLOG(5);
		bRegVal &= ~TPS65185_REG_ENABLE_V3P3_EN;
	}
	else if(-1==I_iIsEP_3V3_ON) {
		GALLEN_DBGLOCAL_RUNLOG(6);
		if( TPS65185_REG(ENABLE) & TPS65185_REG_ENABLE_V3P3_EN) {
			bRegVal |= TPS65185_REG_ENABLE_V3P3_EN;
		}
		else {
			bRegVal &= ~TPS65185_REG_ENABLE_V3P3_EN;
		}
	}

	DBG_MSG("%s() EP3V3ON=%d,ENABLE REG=>%x\n",__FUNCTION__,I_iIsEP_3V3_ON,bRegVal);

	iRet = TPS65185_REG_SET(ENABLE,ALL,bRegVal);
	if(iRet<0) {
		GALLEN_DBGLOCAL_RUNLOG(1);
		goto error;
	}


	//bRegVal = 0;
	bRegVal = 0x7f;
	iRet = TPS65185_REG_SET(INT_EN1,ALL,bRegVal);
	if(iRet<0) {
		GALLEN_DBGLOCAL_RUNLOG(2);
		goto error;
	}

	bRegVal = 0xff;
	iRet = TPS65185_REG_SET(INT_EN2,ALL,bRegVal);
	if(iRet<0) {
		GALLEN_DBGLOCAL_RUNLOG(3);
		goto error;
	}

	if(8==gptHWCFG->m_val.bDisplayResolution) {
		// change power on seqence for EPD 1872x1404 (ED078KH1) .

		bRegVal = 0xe1; // strobe seqence -20 -> -15 ....
		iRet = TPS65185_REG_SET(UPSEQ0,ALL,bRegVal);
		if(iRet<0) {
			GALLEN_DBGLOCAL_RUNLOG(5);
			goto error;
		}

		bRegVal = 0x00; // strobe delay set to 3ms .
		iRet = TPS65185_REG_SET(UPSEQ1,ALL,bRegVal);
		if(iRet<0) {
			GALLEN_DBGLOCAL_RUNLOG(6);
			goto error;
		}

	}

	bRegVal = gbTPS65185_REG_DWNSEQ0_default;
	iRet = TPS65185_REG_SET(DWNSEQ0,ALL,bRegVal);
	if(iRet<0) {
		GALLEN_DBGLOCAL_RUNLOG(4);
		goto error;
	}

error:
	GALLEN_DBGLOCAL_END();
	return iRet;
}

static void tps65185_restore_vcom(void)
{
	if(gtTPS65185_DataA[0].iRestoreVCOM) {
		int iRestoreVCOM = gtTPS65185_DataA[0].iRestoreVCOM;
		int iCurrentVCOM ;

		gtTPS65185_DataA[0].iRestoreVCOM=0;
		tps65185_vcom_get(&iCurrentVCOM);
		printk("restore vcom from %dmV to %dmV\n",iCurrentVCOM,iRestoreVCOM);
		tps65185_vcom_set(iRestoreVCOM,0);
	}
}


// auto detect tps65185 .
// parameters :
// 	iPort : i2c channel in system (from 1~3) .
//  iEPDTimingType :
int tps65185_init(int iPort,int iEPDTimingType)
{
	int iRet = TPS65185_RET_SUCCESS;
	int iChipIdx;
	unsigned long dw65185mode;
	int iChk;
	int irq;
	
	GALLEN_DBGLOCAL_BEGIN();
	
	//printk ("%s(%d) \n",__func__,iPort);

	if(0!=giIsTPS65185_inited) {
		WARNING_MSG("%s(%d):skip ps65185 init twice !\n",__FILE__,__LINE__);
		return TPS65185_RET_SUCCESS;
	}

	iChk = tps65185_gpio_init();
	if(iChk<0) {
		ERR_MSG("[Error] %s : gpio init fail !\n",__FUNCTION__);
		GALLEN_DBGLOCAL_ESC();
		return iChk;
	}

	gpI2C_adapter = i2c_get_adapter(iPort-1);//
	if( NULL == gpI2C_adapter )
	{
		ERR_MSG ("[Error] %s : TPS65185_RET_I2CCHN_NOTFOUND\n",__FUNCTION__);
		GALLEN_DBGLOCAL_ESC();
		return TPS65185_RET_I2CCHN_NOTFOUND;
	}


	for(iChipIdx=0;iChipIdx<TOTAL_CHIPS;iChipIdx++) {
		LKDRIVER_DATA_INIT(iChipIdx);

#if 0 //[
		gpI2C_clientA[iChipIdx] = i2c_new_device(gpI2C_adapter, &gtTPS65185_BIA[iChipIdx]);
#else //][!

#ifdef TPS65185_PLATFORM_MX6//[
		gpI2C_clientA[iChipIdx] = \
			i2c_new_probed_device(gpI2C_adapter, &gtTPS65185_BIA[iChipIdx],gwTPS65185_AddrA,0);
#else //][!TPS65185_PLATFORM_MX6
		gpI2C_clientA[iChipIdx] = \
			i2c_new_probed_device(gpI2C_adapter, &gtTPS65185_BIA[iChipIdx],gwTPS65185_AddrA);
#endif //]TPS65185_PLATFORM_MX6

#endif//]

		if(NULL == gpI2C_clientA[iChipIdx]) {
			tps65185_release();
			GALLEN_DBGLOCAL_ESC();
			ERR_MSG("[Error] %s : TPS65185_RET_NEWDEVFAIL\n",__FUNCTION__);
			return TPS65185_RET_NEWDEVFAIL;
		}
		printk("client%d@i2c%d ,addr=0x%x,name=%s\n",iChipIdx,iPort,
			gpI2C_clientA[iChipIdx]->addr,gpI2C_clientA[iChipIdx]->name);

	}
	

	INIT_DELAYED_WORK(&gtPwrdwn_work_param.pwrdwn_work,tps65185_pwrdwn_work_func);


	if( gptHWCFG->m_val.bPCB>=34) 
	{
		// new HW design after E606FXA...
    giIsTPS65185_turnoff_EP3V3=1;
		if(5==gptHWCFG->m_val.bDisplayResolution) {
			// resolution is 1448x1072 ...
#if 1
			// measured on Q92 .
			gtTPS65185_DataA[0].dwEP3V3_Off_Ticks = 800;// times wait VEE to 0V .
#else
			gtTPS65185_DataA[0].dwEP3V3_Off_Ticks = 200;// times wait VEE >= -7.4V .
#endif
		}
		else if(8==gptHWCFG->m_val.bDisplayResolution) {
			// resolution is 1872x1404 ...
			gtTPS65185_DataA[0].dwEP3V3_Off_Ticks = 900;//measured on E70Q02 .
		}
		else if(1==gptHWCFG->m_val.bDisplayResolution) {
			// resolution is 1024x758 ...
			gtTPS65185_DataA[0].dwEP3V3_Off_Ticks = 400;//measured on Q32 .
		}
		else if(0==gptHWCFG->m_val.bDisplayResolution) {
			// resolution is 800x600 ...
			gtTPS65185_DataA[0].dwEP3V3_Off_Ticks = 650;//measured on Q92 .
		}
		else if(3==gptHWCFG->m_val.bDisplayResolution) {
			// resolution is 1440x1080 ...
			gtTPS65185_DataA[0].dwEP3V3_Off_Ticks = 200;//measured on E60Q62 .
		}
		else {
			gtTPS65185_DataA[0].dwEP3V3_Off_Ticks = 0;
		}
  }
	printk("%s(%d): EP_3V3 ON/OFF control enabled !EP3V3 OFF ticks=%d\n",__FILE__,__LINE__,gtTPS65185_DataA[0].dwEP3V3_Off_Ticks);
	giIsTPS65185_inited=1;

	// change TPS65185 to standby mode .
	dw65185mode = TPS65185_MODE_STANDBY;
	iChk = tps65185_chg_mode(&dw65185mode,1);
	if(iChk<0) {
		ERR_MSG("[Error] %s : change to standby mode fail !\n",__FUNCTION__);
		tps65185_release();
		GALLEN_DBGLOCAL_ESC();
		return iChk;
	}

#if 0
	iChk = tps65185_reg_init(0);
	if(iChk<0) {
		ERR_MSG("[Error] %s : tps65185 regs init fail !\n",__FUNCTION__);
		tps65185_release();
		GALLEN_DBGLOCAL_ESC();
		return iChk;
	}
#endif


	iChk = tps65185_get_versions(&gtTPS65185_DataA[0].t65185_versions);
	printk("TPS65185 versions : major=0x%x,minor=0x%x,version=0x%x,RevID=0x%x\n",
			gtTPS65185_DataA[0].t65185_versions.bMajor,
			gtTPS65185_DataA[0].t65185_versions.bMinor,
			gtTPS65185_DataA[0].t65185_versions.bVersion,
			gtTPS65185_DataA[0].t65185_versions.bRevID);

	if(iChk<0) {
		ERR_MSG("[Error] %s : get TPS65185 version fail !\n",__FUNCTION__);
		tps65185_release();
		GALLEN_DBGLOCAL_ESC();
		return iChk;
	}


	//tps65185_get_temperature(0,0);// test .
	
	
	//
	// config registers of TPS65185 ...
	//
	iChk = tps65185_config_epd_timing(iEPDTimingType);
	if(iChk<0) {
		ERR_MSG("[Error] %s : config EPD timing fail !\n",__FUNCTION__);
		tps65185_release();
		GALLEN_DBGLOCAL_ESC();
		return iChk;
	}
	printk ("[%s-%d] gpio_to_irq ()\n",__func__,__LINE__);

#ifdef TPS65185_PWR_ONOFF_INT//[

	irq = gpio_to_irq(GPIO_TPS65185_PWRGOOD);
	iChk = request_irq(irq, tps65185_pwrgood_inthandler, 0, "tps65185_PWRGOOD", 0);
	if (iChk) {
		pr_info("register TPS65185 pwrgood interrupt failed\n");
	}
	else {
		//enable_irq_wake(irq);
		//disable_irq_wake(irq);
		//disable_irq(irq);
	}
#endif //]TPS65185_PWR_ONOFF_INT

#if 1
	irq = gpio_to_irq(GPIO_TPS65185_INT);
	iChk = request_irq(irq, tps65185_int, 0, "tps65185_INT", 0);
	if (iChk) {
		pr_info("register TPS65185 interrupt failed\n");
	}
	else {
		//enable_irq_wake(irq);
		//disable_irq_wake(irq);
		//disable_irq(irq);
	}
#endif
	

	/*
	dw65185mode = TPS65185_MODE_ACTIVE;
	iChk = tps65185_chg_mode(&dw65185mode,1);
	if(iChk<0) {
		ERR_MSG("[Error] %s : change to active mode fail !\n",__FUNCTION__);
		tps65185_release();
		GALLEN_DBGLOCAL_ESC();
		return iChk;
	}
	*/
	GALLEN_DBGLOCAL_END();
	printk("tps65185_init: ret=%d\n", iRet);
	return iRet;
}
EXPORT_SYMBOL(tps65185_init);




int tps65185_release(void)
{
	int iRet = TPS65185_RET_SUCCESS;
	int iChipIdx ;
	int iChk;
	unsigned long dw65185mode ;
	
	GALLEN_DBGLOCAL_BEGIN();
	//printk("%s(%d):%s()\n",__FILE__,__LINE__,__FUNCTION__);
	for(iChipIdx=0;iChipIdx<TOTAL_CHIPS;iChipIdx++) {
		if(gpI2C_clientA[iChipIdx]) {
			i2c_unregister_device(gpI2C_clientA[iChipIdx]);
			gpI2C_clientA[iChipIdx] = NULL;
		}
		gtTPS65185_DataA[iChipIdx].iCurrent_temprature = -1;
		//gtTPS65185_DataA[iChipIdx].iLast_temprature = -1;
	}

	if(giIsTPS65185_inited) {
		dw65185mode = TPS65185_MODE_SLEEP;
		iChk = tps65185_chg_mode(&dw65185mode,1);
		if(iChk<0) {
			ERR_MSG("[Error] %s : change to power down mode fail !\n",__FUNCTION__);
			tps65185_release();
			GALLEN_DBGLOCAL_ESC();
			return iChk;
		}
	// release interrupt ...
	
		iChk = tps65185_gpio_release();
		if(iChk<0) {
			WARNING_MSG("[Warnig] %s : gpio release fail !\n",__FUNCTION__);
		}
	}


	gpI2C_adapter = NULL;
	
	GALLEN_DBGLOCAL_END();
	return iRet;
}
EXPORT_SYMBOL(tps65185_release);


int tps65185_get_temperature(int *O_piTemperature)
{
	int iRet = TPS65185_RET_SUCCESS;
	int iChk;
	
	unsigned short wTemp,wReg;
	
	unsigned char bTemp,bReg;
	int iTemp;
	
	int iChipIdx = 0;

	unsigned long ulTimeoutTick;
	unsigned long ulCurTick;
	unsigned long dw65185mode;

	GALLEN_DBGLOCAL_BEGIN();

	if(TPS65185_MODE_STANDBY!=gtTPS65185_DataA[0].dwCurrent_mode&&TPS65185_MODE_ACTIVE!=gtTPS65185_DataA[0].dwCurrent_mode)
	{
		dw65185mode = TPS65185_MODE_STANDBY;
		iChk = tps65185_chg_mode(&dw65185mode,1);
		if(iChk<0) {
			ERR_MSG("[Error] %s : change to standby mode fail !\n",__FUNCTION__);
			GALLEN_DBGLOCAL_ESC();
			return iChk;
		}
	}

	//printk("%s()\n",__FUNCTION__);
	iChk = TPS65185_REG_SET(TMST1,READ_THERM,1);

	//udelay(1);
	ulTimeoutTick = jiffies + 50;
	do {

		wReg = TPS65185_REG_GET(TMST1);
		if(((unsigned short)(-1))==wReg) {
			GALLEN_DBGLOCAL_ESC();
			return TPS65185_RET_REGREADFAIL;
		}
		if(wReg&TPS65185_REG_TMST1_CONV_END) {
			break;
		}

		schedule_timeout(1);

		ulCurTick = jiffies;
		if(ulCurTick>ulTimeoutTick) {
			ERR_MSG("%s(%d):wait TMST1 ADC timeout \n",__FILE__,__LINE__);
			GALLEN_DBGLOCAL_ESC();
			return TPS65185_RET_TIMEOUT;
		}
	} while(1);
	
	wReg = TPS65185_REG_GET(TMST_VALUE);

	if(((unsigned short)(-1))==wReg) {
		GALLEN_DBGLOCAL_ESC();
		return TPS65185_RET_REGREADFAIL;
	}
	
	bReg = (unsigned char)wReg;
	gtTPS65185_DataA[iChipIdx].wTempratureData = wReg;
	if(bReg&0x80) {
		GALLEN_DBGLOCAL_RUNLOG(0);
		// negative .
		bTemp=(~bReg)+1;
		iTemp = bTemp;
		iTemp = (~iTemp)+1;
	}
	else {
		GALLEN_DBGLOCAL_RUNLOG(1);
		// positive .
		iTemp = (int)(bReg);
	}
	gtTPS65185_DataA[iChipIdx].iCurrent_temprature = iTemp;
	printk("%s temprature data = 0x%x,%d\n",DRIVERNAME,wReg,gtTPS65185_DataA[iChipIdx].iCurrent_temprature);
	
	//gtTPS65185_DataA[iChipIdx].iCurrent_temprature = bA[0];
	if(O_piTemperature) {
		GALLEN_DBGLOCAL_RUNLOG(2);
		*O_piTemperature = gtTPS65185_DataA[iChipIdx].iCurrent_temprature;
	}
	
	GALLEN_DBGLOCAL_END();
	return iRet;
}
EXPORT_SYMBOL(tps65185_get_temperature);

int tps65185_set_ep3v3_pwrdn_delay_ticks(unsigned long dwTicks)
{
	printk("%s(%d)\n",__FUNCTION__,dwTicks);
	gtTPS65185_DataA[0].dwEP3V3_Off_Ticks=dwTicks;
	return 0;
}

int tps65185_is_panel_poweron(void)
{
	int iRet;
	#if 0
	int iChk;
	
	iChk = tps65185_chk_PG(0xfa);
	if(TPS65185_RET_ALLPOWERGOOD==iChk) {
		iRet = 1;
	}
	else {
		iRet = 0;
	}
	#else
	iRet = gpio_get_value(GPIO_TPS65185_PWRGOOD)?1:0;
	#endif
	
	return iRet;
}

int tps65185_wait_panel_poweron(void)
{
#if 0
	//printk("%s(%d);\n",__FILE__,__LINE__);
	//msleep(30);
	return TPS65185_RET_SUCCESS;
#else
	int iRet = TPS65185_RET_SUCCESS;
	int iChk;
	int ilpcnt=0;


	#ifdef TPS65185_PWR_ONOFF_INT//[

	if(in_interrupt()) {
		printk("%s(%d):[warning] call %s in interrupt !!\n",
			__FILE__,__LINE__,__FUNCTION__);
	}
	else 
	{
		do {
			#ifdef TPS65185_PWR_ONOFF_WAITBYCOMPLETE//[

			if(!tps65185_is_panel_poweron()) {
				ERR_MSG("%s(),before wait PG,jiffies=%ld\n",__FUNCTION__,jiffies);
				wait_for_completion_timeout(&tps65185_pwrgood_on_completion,500);
				ERR_MSG("%s(),after wait PG,jiffies=%ld\n",__FUNCTION__,jiffies);
			}

			#else //][!TPS65185_PWR_ONOFF_WAITBYCOMPLETE
			iChk = wait_event_interruptible_timeout(tps65185_pwron_wq,tps65185_is_panel_poweron(),50);
			//iChk = wait_event_timeout(tps65185_pwron_wq,tps65185_is_panel_poweron(),50);
			#endif//]TPS65185_PWR_ONOFF_WAITBYCOMPLETE
			if(!tps65185_is_panel_poweron()) {
				iRet = TPS65185_RET_TIMEOUT;
				ERR_MSG("%s(%d):wait power on timeout lpcnt=%d!\n",__FILE__,__LINE__,ilpcnt);
			}
			else {
				iRet = TPS65185_RET_SUCCESS;
				break;
			}

		} while(++ilpcnt<10);
	}

	#else //][!TPS65185_PWR_ONOFF_INT

	volatile unsigned long dwCnt=0;

	do {
		++dwCnt;
		if(dwCnt>=100) {
			ERR_MSG("%s timeout >=%ld !!\n",__FUNCTION__,dwCnt);
			break;
		}

		//if(tps65185_is_panel_poweron()) 
		if(giIsTPS65185_PwrOn) 
		{
			DBG_MSG("%s poweron@cnt=%ld\n",__FUNCTION__,dwCnt);
			break;
		}

		schedule_timeout(1);

	}while(1) ;


	#endif //]TPS65185_PWR_ONOFF_INT

	DBG_MSG("%s:PMIC poweron=%d,%d\n",__FUNCTION__,tps65185_is_panel_poweron(),giIsTPS65185_PwrOn);
	return iRet;
#endif
}

int tps65185_wait_panel_poweroff(void)
{
#if 0
	//schedule_timeout(1);
	//printk("%s(%d);\n",__FILE__,__LINE__);
	//msleep(30);
	return TPS65185_RET_SUCCESS;
#else

	int iRet = TPS65185_RET_SUCCESS;
	int iChk;
	int ilpcnt=0;


	#ifdef TPS65185_PWR_ONOFF_INT//[

	if(in_interrupt()) {
		printk("%s(%d):[warning] call %s in interrupt !!\n",
			__FILE__,__LINE__,__FUNCTION__);
	}
	else 
	{
		do {
			#ifdef TPS65185_PWR_ONOFF_WAITBYCOMPLETE//[

			if(tps65185_is_panel_poweron()) {
				ERR_MSG("%s(),before wait PGoff,jiffies=%ld\n",__FUNCTION__,jiffies);
				wait_for_completion_timeout(&tps65185_pwrgood_off_completion,500);
				ERR_MSG("%s(),after wait PGoff,jiffies=%ld\n",__FUNCTION__,jiffies);
			}
			#else //][!TPS65185_PWR_ONOFF_WAITBYCOMPLETE
			iChk = wait_event_interruptible_timeout(tps65185_pwroff_wq,!tps65185_is_panel_poweron(),50);
			//iChk = wait_event_timeout(tps65185_pwroff_wq,!tps65185_is_panel_poweron(),50);
			#endif //] TPS65185_PWR_ONOFF_WAITBYCOMPLETE
			if(tps65185_is_panel_poweron()) {
				iRet = TPS65185_RET_TIMEOUT;
				ERR_MSG("%s(%d):wait power off timeout lpcnt=%d!\n",__FILE__,__LINE__,ilpcnt);
			}
			else {
				iRet = TPS65185_RET_SUCCESS;
				break;
			}

		}while(++ilpcnt<10);

	}

	#else	//][!TPS65185_PWR_ONOFF_INT

	volatile unsigned long dwCnt=0;

	do {
		++dwCnt;
		if(dwCnt>=100) {
			ERR_MSG("%s timeout >=%ld !!\n",__FUNCTION__,dwCnt);
			break;
		}
		//if(!tps65185_is_panel_poweron()) 
		if(!giIsTPS65185_PwrOn) 
		{
			DBG_MSG("%s poweroff@cnt=%ld\n",__FUNCTION__,dwCnt);
			break;
		}

		schedule_timeout(1);
	}while(1) ;

	#endif //] TPS65185_PWR_ONOFF_INT

	DBG_MSG("%s:PMIC poweron=%d,%d\n",__FUNCTION__,tps65185_is_panel_poweron(),giIsTPS65185_PwrOn);
	return iRet;
#endif
}


int tps65185_chg_mode(unsigned long *IO_pdwMode,int iIsWaitPwrOff)
{
	int iRet=TPS65185_RET_SUCCESS;
	int iChk;
	unsigned long dwCurrent_mode;
	unsigned long dwNewMode;
	int iRetryCnt;
	unsigned long dwCurTick,dwTicks;
	int iPoweringOn;
	//int irq_INT,irq_PG;

	GALLEN_DBGLOCAL_BEGIN();

	//int iNewWakeupState,iNewPwrupState;

	//irq_INT = gpio_to_irq(GPIO_TPS65185_INT);
	//irq_PG = gpio_to_irq(GPIO_TPS65185_PWRGOOD);

	//disable_irq(irq_INT);
	//disable_irq(irq_PG);
	if(*IO_pdwMode==TPS65185_MODE_ACTIVE) {
		tps65185_restore_vcom();
	}

	down(&gtTPS65185_DataA[0].chmod_lock);

	dwCurrent_mode = gtTPS65185_DataA[0].dwCurrent_mode;
	dwNewMode = *IO_pdwMode;

	if(0==giIsTPS65185_inited) {
		ERR_MSG("[Error] %s : tps65185 must be initialized first !\n",__FUNCTION__);
		GALLEN_DBGLOCAL_ESC();
		iRet = TPS65185_RET_INITNOTYET;
		goto exit;
	}

	if(!IO_pdwMode) {
		ERR_MSG("[Warning] %s : parameter error !\n",__FUNCTION__);
		GALLEN_DBGLOCAL_ESC();
		iRet = TPS65185_RET_PARAMERR;
		goto exit;
	}

	if(dwNewMode!=TPS65185_MODE_ACTIVE && dwCurrent_mode == dwNewMode) {
		DBG_MSG("%s : skip same mode \n",__FUNCTION__);
		GALLEN_DBGLOCAL_ESC();
		iRet = TPS65185_RET_SUCCESS;
		goto exit;
	}


	// change tps65185 work mode ...
	//gpio_direction_input(GPIO_TPS65185_WAKEUP);
	//iCurrentWakeupState = gpio_get_value(GPIO_TPS65185_WAKEUP);
	//gpio_direction_input(GPIO_TPS65185_PWRUP);
	//iCurrentPwrupState = gpio_get_value(GPIO_TPS65185_PWRUP);

	DBG_MSG("%s begin %ld->%ld\n",__FUNCTION__,dwCurrent_mode,dwNewMode);
	DBG_MSG("%s():TPS65185 wakeup=%d,pwrup=%d\n",__FUNCTION__,\
			gpio_get_value(GPIO_TPS65185_WAKEUP),\
			gpio_get_value(GPIO_TPS65185_PWRUP));

	switch(dwNewMode) {
	case TPS65185_MODE_ACTIVE:GALLEN_DBGLOCAL_RUNLOG(0);
		#if 0
		if( TPS65185_MODE_SLEEP==dwCurrent_mode || 
				TPS65185_MODE_STANDBY==dwCurrent_mode||
				TPS65185_MODE_UNKOWN==dwCurrent_mode) 
		{
			tps65185_wait_panel_poweroff();
		}	
		#endif

		up(&gtTPS65185_DataA[0].chmod_lock);
		iChk = cancel_delayed_work_sync(&gtPwrdwn_work_param.pwrdwn_work);
		down(&gtTPS65185_DataA[0].chmod_lock);

		//iNewWakeupState = 1;
		//iNewPwrupState = 1;
		
		if(TPS65185_MODE_SLEEP==dwCurrent_mode || TPS65185_MODE_UNKOWN==dwCurrent_mode) {
			GALLEN_DBGLOCAL_RUNLOG(3);
			gpio_direction_output(GPIO_TPS65185_WAKEUP, 1);
			if(gtTPS65185_DataA[0].iIsInitPwrON) {
				GALLEN_DBGLOCAL_RUNLOG(2);
				gtTPS65185_DataA[0].iIsInitPwrON = 0;
			}

			iRetryCnt = 0;
			do {
				mdelay(2);
				//if(giIsTPS65185_turnoff_EP3V3) {
					//iRet = tps65185_reg_init(0);
					//iRet = tps65185_reg_init(-1);
					iRet = tps65185_reg_init(1);
				//}
				//else {
				//	iRet = tps65185_reg_init(1);
				//}

				if(iRet>=0) {
					dwCurTick=jiffies;
					if(time_before(dwCurTick,gdwSafeTick_To_TurnON_RailPower)) {
						dwTicks = gdwSafeTick_To_TurnON_RailPower-dwCurTick;	
						msleep(jiffies_to_msecs(dwTicks));
						DBG_MSG("msleep %ld ticks for resume times\n",dwTicks);
					}

#ifdef GPIO_TPS65185_EP_3V3_IN //[
					//gpio_direction_output(GPIO_TPS65185_EP_3V3_IN, 1);udelay(30);
#endif //]GPIO_TPS65185_EP_3V3_IN

					//TPS65185_REG_SET(ENABLE,V3P3_EN,1);udelay(100);
					gpio_direction_output(GPIO_TPS65185_PWRUP, 1);
					break;
				}
				WARNING_MSG("PMIC sleep->active, retry %d\n",iRetryCnt);
			} while(++iRetryCnt<10);

			iPoweringOn = 1;
		}
		else if(TPS65185_MODE_STANDBY==dwCurrent_mode) {
			//gpio_direction_output(GPIO_TPS65185_WAKEUP, 1);
			//gpio_direction_output(GPIO_TPS65185_PWRUP, 0);
			//ERR_MSG(".");
			//udelay(1900);
			if(giIsTPS65185_turnoff_EP3V3) {
				if(!(TPS65185_REG_GET(ENABLE)&TPS65185_REG_ENABLE_V3P3_EN)) {
					dwCurTick=jiffies;
					if(time_before(dwCurTick,gdwSafeTick_To_TurnON_RailPower)) {
						dwTicks = gdwSafeTick_To_TurnON_RailPower-dwCurTick;	
						msleep(jiffies_to_msecs(dwTicks));
						DBG_MSG("msleep %ld ticks for resume times\n",dwTicks);
					}

#ifdef GPIO_TPS65185_EP_3V3_IN //[
					//gpio_direction_output(GPIO_TPS65185_EP_3V3_IN, 1);udelay(10);
#endif //]GPIO_TPS65185_EP_3V3_IN

					//TPS65185_REG_SET(ENABLE,V3P3_EN,1);udelay(100);
				}
			}

			gpio_direction_output(GPIO_TPS65185_PWRUP, 1);
			//ERR_MSG(".");
			iPoweringOn = 1;
		}
		else if(TPS65185_MODE_ACTIVE==dwCurrent_mode) {
			iPoweringOn = 0;
		}
		else 
		{
			GALLEN_DBGLOCAL_RUNLOG(4);
			ERR_MSG("wrong mode !!? (mode=%ld)\n",dwCurrent_mode);
			iRet = TPS65185_REG_SET(ENABLE,ACTIVE,1);
			iPoweringOn = 0;
		}

		if(iPoweringOn) 
		{
			//ERR_MSG(".");
			tps65185_wait_panel_poweron();
		}

		msleep(5);
		TPS65185_VCOM_OUT(1);
		msleep(5);
		//ERR_MSG(".\n");
		gtTPS65185_DataA[0].dwCurrent_mode = dwNewMode;

		break;

	case TPS65185_MODE_SLEEP:GALLEN_DBGLOCAL_RUNLOG(5);
		//iNewWakeupState = 0;
		//iNewPwrupState = 0;

		gtPwrdwn_work_param.dwNewMode = dwNewMode;
		gtPwrdwn_work_param.iIsWaitPwrOff = iIsWaitPwrOff;

		if(TPS65185_MODE_ACTIVE==dwCurrent_mode) 
		{
			msleep(2);TPS65185_VCOM_OUT(0);

#ifdef TPS65185_PWROFFDELAYWORK_TICKS//[

			if(!delayed_work_pending(&gtPwrdwn_work_param.pwrdwn_work)) {
				GALLEN_DBGLOCAL_RUNLOG(13);
				up(&gtTPS65185_DataA[0].chmod_lock);
				iChk = cancel_delayed_work_sync(&gtPwrdwn_work_param.pwrdwn_work);
				down(&gtTPS65185_DataA[0].chmod_lock);
				schedule_delayed_work(&gtPwrdwn_work_param.pwrdwn_work, TPS65185_PWROFFDELAYWORK_TICKS);
			}
#else//][! TPS65185_PWROFFDELAYWORK_TICKS
			GALLEN_DBGLOCAL_RUNLOG(14);
			_tps65185_pwrdwn();
#endif //] TPS65185_PWROFFDELAYWORK_TICKS
			
		}
		else 
		{
			GALLEN_DBGLOCAL_RUNLOG(15);
			_tps65185_pwrdwn();
		}

		break;

	case TPS65185_MODE_STANDBY:GALLEN_DBGLOCAL_RUNLOG(6);
		//iNewWakeupState = 1;
		//iNewPwrupState = 0;

		if(TPS65185_MODE_SLEEP==dwCurrent_mode || TPS65185_MODE_UNKOWN==dwCurrent_mode) 
		{
			//GALLEN_DBGLOCAL_RUNLOG(9);
			GALLEN_DBGLOCAL_RUNLOG(7);


			gpio_direction_output(GPIO_TPS65185_WAKEUP, 1);
			if(gtTPS65185_DataA[0].iIsInitPwrON) {
				GALLEN_DBGLOCAL_RUNLOG(8);
				gtTPS65185_DataA[0].iIsInitPwrON = 0;
			}

			
			//gpio_direction_output(GPIO_TPS65185_PWRUP, 0);
			iRetryCnt = 0;
			do {
				mdelay(2);
				//if(giIsTPS65185_turnoff_EP3V3) {
					//iRet = tps65185_reg_init(-1);
					iRet = tps65185_reg_init(1);
				//}
				//else {
					//iRet = tps65185_reg_init(1);
				//}
				if(iRet>=0) {
					gtTPS65185_DataA[0].dwCurrent_mode = dwNewMode;

#ifdef TPS65185_RESUME_EP3V3_ON //[

#ifdef GPIO_TPS65185_EP_3V3_IN //[
					//gpio_direction_output(GPIO_TPS65185_EP_3V3_IN, 1);udelay(10);
#endif //]GPIO_TPS65185_EP_3V3_IN

					//TPS65185_REG_SET(ENABLE,V3P3_EN,1);udelay(100);
#endif //]TPS65185_RESUME_EP3V3_ON
					break;
				}
				WARNING_MSG("PMIC sleep->standby, retry %d\n",iRetryCnt);
			} while(++iRetryCnt<10);


		}
		else if(TPS65185_MODE_ACTIVE==dwCurrent_mode) {
			msleep(2);TPS65185_VCOM_OUT(0);

			gtPwrdwn_work_param.dwNewMode = dwNewMode;
			gtPwrdwn_work_param.iIsWaitPwrOff = iIsWaitPwrOff;
#ifdef TPS65185_PWROFFDELAYWORK_TICKS//[

			if(!delayed_work_pending(&gtPwrdwn_work_param.pwrdwn_work)) {
				GALLEN_DBGLOCAL_RUNLOG(16);
				up(&gtTPS65185_DataA[0].chmod_lock);
				iChk = cancel_delayed_work_sync(&gtPwrdwn_work_param.pwrdwn_work);
				down(&gtTPS65185_DataA[0].chmod_lock);
				schedule_delayed_work(&gtPwrdwn_work_param.pwrdwn_work, TPS65185_PWROFFDELAYWORK_TICKS);
			}
#else//][! TPS65185_PWROFFDELAYWORK_TICKS
			GALLEN_DBGLOCAL_RUNLOG(17);
			_tps65185_pwrdwn();
#endif //] TPS65185_PWROFFDELAYWORK_TICKS
			
		}
		else {
			GALLEN_DBGLOCAL_RUNLOG(10);
			ERR_MSG("wrong mode !!? (mode=%ld)\n",dwCurrent_mode);
			iRet = TPS65185_REG_SET(ENABLE,STANDBY,1);
			gtTPS65185_DataA[0].dwCurrent_mode = dwNewMode;
		}


		break;

	default:
		GALLEN_DBGLOCAL_RUNLOG(11);
		WARNING_MSG("%s : mode unsupported (0x%x) !!\n",__FUNCTION__,
			(unsigned int)dwNewMode);
		GALLEN_DBGLOCAL_ESC();
		iRet = TPS65185_RET_PARAMERR;
		goto exit;
	}

	DBG_MSG("%s :[mode] 0x%x->0x%x \n",__FUNCTION__,\
			(unsigned int)dwCurrent_mode,(unsigned int)dwNewMode);

	//gtTPS65185_DataA[0].iCurrentPwrupState = iCurrentPwrupState;
	//gtTPS65185_DataA[0].iCurrentWakeupState = iCurrentWakeupState;

	*IO_pdwMode = dwCurrent_mode;

exit:

	DBG_MSG("%s end,mode %lu->%lu \n",__FUNCTION__,dwCurrent_mode,dwNewMode);

	GALLEN_DBGLOCAL_END();

	up(&gtTPS65185_DataA[0].chmod_lock);
	//enable_irq(irq_PG);
	//enable_irq(irq_INT);

	return iRet;
}
EXPORT_SYMBOL(tps65185_chg_mode);

int tps65185_vcom_set(int I_iVCOM_mv,int iIsWriteToFlash)
{
	int iRet = TPS65185_RET_SUCCESS;
	int iChk;
	unsigned short wVCOM_val;
	unsigned long dwTPS65185_mode;
	int iChkVCOM;
	//
	
	GALLEN_DBGLOCAL_BEGIN();

	// accept positive and negative form
	if (I_iVCOM_mv < 0) I_iVCOM_mv = -I_iVCOM_mv;

	wVCOM_val = (unsigned short)(I_iVCOM_mv/10);
	
	// TPS65185 should be in Standby or Active mode .
	//dwTPS65185_mode = TPS65185_MODE_STANDBY;
	//iChk = tps65185_chg_mode(&dwTPS65185_mode,1);
	
	//printk(KERN_ERR"====>[wVCOM_val]:%x\n",wVCOM_val);
	if(wVCOM_val&0x100) {
		GALLEN_DBGLOCAL_RUNLOG(2);
		TPS65185_REG_SET(VCOM2,VCOM8,1);
	}
	else {
		TPS65185_REG_SET(VCOM2,VCOM8,0);
	}
	TPS65185_REG_SET(VCOM1,ALL,(unsigned char)wVCOM_val);
	
	if(iIsWriteToFlash) {
#if 1//[

		GALLEN_DBGLOCAL_RUNLOG(3);
		TPS65185_REG_SET(INT_EN1,PRGC_EN,1);
		TPS65185_REG_SET(VCOM2,PROG,1);
		iChk = wait_event_timeout(tps65185_PRGC_WQ,
			TPS65185_REG(INT1)&TPS65185_REG_INT1_PRGC,100);
		if(!(TPS65185_REG(INT1)&TPS65185_REG_INT1_PRGC)) {
			iRet = TPS65185_RET_TIMEOUT;
			ERR_MSG("%s(%d):wait TPS65185 PRGC timeout !\n",__FILE__,__LINE__);
		} else {
			iChk = tps65185_vcom_get(&iChkVCOM);
			printk("VCOM written: %d read: %d\n", I_iVCOM_mv, iChkVCOM);
		}
#else//!][
		{
			unsigned long ulTimeoutTick,ulCurTick;
			unsigned short wReg;

			GALLEN_DBGLOCAL_RUNLOG(4);
			//printk("%s()\n",__FUNCTION__);
			iChk = TPS65185_REG_SET(INT_EN1,PRGC_EN,1);
			iChk = TPS65185_REG_SET(VCOM2,PROG,1);

			//udelay(1);
			ulTimeoutTick = jiffies + 100;
			do {

				wReg = TPS65185_REG(INT1);
				if(((unsigned short)(-1))==wReg) {
					GALLEN_DBGLOCAL_ESC();
					return TPS65185_RET_REGREADFAIL;
				}
				if(wReg&TPS65185_REG_INT1_PRGC) {
					GALLEN_DBGLOCAL_RUNLOG(5);
					break;
				}


				GALLEN_DBGLOCAL_RUNLOG(6);
				schedule_timeout(1);

				ulCurTick = jiffies;
				if(ulCurTick>ulTimeoutTick) {

					ERR_MSG("%s(%d):wait TMST1 ADC timeout ,wReg=0x%x\n",
							__FILE__,__LINE__,wReg);

					GALLEN_DBGLOCAL_ESC();
					return TPS65185_RET_TIMEOUT;
				}

			} while(1);
		}
#endif//]

#if 0
		dwTPS65185_mode = TPS65185_MODE_SLEEP;
		iChk = tps65185_chg_mode(&dwTPS65185_mode,1);
	
		dwTPS65185_mode = TPS65185_MODE_STANDBY;
		iChk = tps65185_chg_mode(&dwTPS65185_mode,1);

		iChk = tps65185_vcom_get(&iChkVCOM);
		//printk("%s:iChkVCOM = %d mV\n",__FUNCTION__,iChkVCOM);
		if(iChkVCOM!=(I_iVCOM_mv/10*10)) {

			ERR_MSG("%s(%d):VCOM check fail (%d!=%d)!\n",
					__FILE__,__LINE__,iChk,I_iVCOM_mv);

			GALLEN_DBGLOCAL_ESC();
			return TPS65185_RET_VCOMWRFAIL;
		}
#endif

	}
	else {
		iChk = tps65185_vcom_get(&iChkVCOM);
	}
	GALLEN_DBGLOCAL_END();

	return iRet;
}
EXPORT_SYMBOL(tps65185_vcom_set);

int tps65185_vcom_get(int *O_piVCOM_mv)
{
	int iRet = TPS65185_RET_SUCCESS;
	
	unsigned char bRegVCOM1,bRegVCOM2;
	unsigned short wTemp;
	int iTemp;
	

	bRegVCOM2 = TPS65185_REG_GET(VCOM2);
	bRegVCOM1 = TPS65185_REG_GET(VCOM1);
	wTemp = (bRegVCOM1|bRegVCOM2<<8)&0x1ff;
	iTemp = -(wTemp*10);
	gtTPS65185_DataA[0].iCurrentVCOM = iTemp;
	//printk("%s():VCOM=%d\n",__FUNCTION__,iTemp);
	if(O_piVCOM_mv) {
		*O_piVCOM_mv = iTemp;
	}
	else {
		iRet = TPS65185_RET_PARAMERR;
	}
	
	return iRet;
}
EXPORT_SYMBOL(tps65185_vcom_get);

int tps65185_vcom_get_cached(int *O_piVCOM_mv)
{
	int iRet = TPS65185_RET_SUCCESS;
	if(O_piVCOM_mv) {
		if(0!=gtTPS65185_DataA[0].iCurrentVCOM) {
			*O_piVCOM_mv = gtTPS65185_DataA[0].iCurrentVCOM;
		}
		else {
			return tps65185_vcom_get(O_piVCOM_mv);
		}
	}
	return iRet;
}
EXPORT_SYMBOL(tps65185_vcom_get_cached);

int tps65185_vcom_kickback_measurement(int *O_piVCOM_mv)
{
	int iRet = TPS65185_RET_SUCCESS;
	int iChk;

	GALLEN_DBGLOCAL_BEGIN();
	
	if(O_piVCOM_mv) {
		unsigned long dwTPS65185_mode;
	
		
		// Pull the WAKEUP pin and the PWRUP pin to enable all output rails .
		dwTPS65185_mode = TPS65185_MODE_ACTIVE;
		iChk = tps65185_chg_mode(&dwTPS65185_mode,1);
		// Set the HiZ bit in the VCOM2 register. This puts the VCOM pin in high-impedance state .
		TPS65185_REG_SET(VCOM2,HiZ,1);
		
		// Drive the panel with the Null waveform. 
		
		
		// Set ACQ bt in the VCOM2 register to 1. This starts the mesurement routine .
		TPS65185_REG_SET(VCOM2,ACQ,1);
		
		// When the measurement is complete, the ACQC (Acquisition Complete) 
		//   bit in the INT1 register is set and the nINT pin is pulled low .
		
		iChk = wait_event_timeout(tps65185_ACQC_WQ,
			TPS65185_REG(INT1)&TPS65185_REG_INT1_ACQC,50);
		if(!(TPS65185_REG(INT1)&TPS65185_REG_INT1_ACQC)) {
			iRet = TPS65185_RET_TIMEOUT;
			ERR_MSG("%s(%d):wait TPS65185 ACQC timeout !\n",__FILE__,__LINE__);
		}
		
		// The measurement result is stored in the VCOM[8:0] bits of the VCOM1 and VCOM2 register .
		iRet = tps65185_vcom_get(O_piVCOM_mv);
		
	}
	else {
		iRet = TPS65185_RET_PARAMERR;
	}
	
	GALLEN_DBGLOCAL_END();
	return iRet;
}


int tps65185_ONOFF(int iIsON)
{
#ifdef TPS65185_SUSPEND //[
	int iRet=0;

	if(!giIsTPS65185_turnoff_EP3V3) {
		return 1;
	}

	if(!(6==gptHWCFG->m_val.bDisplayCtrl||7==gptHWCFG->m_val.bDisplayCtrl)) {
		WARNING_MSG("%s() display controller (%d) not match !\n",
				__FUNCTION__,(int)gptHWCFG->m_val.bDisplayCtrl);
		return 1;
	}

	//printk("%s(%d)\n",__FUNCTION__,iIsON);

	if(iIsON) {
		// Turn on the TPS65185 power .
		 
		//gpio_free(GPIO_TPS65185_SDL);
		//gpio_free(GPIO_TPS65185_SDA);

		//mxc_iomux_v3_setup_pad(MX50_PAD_I2C2_SCL__I2C2_SCL);
		//mxc_iomux_v3_setup_pad(MX50_PAD_I2C2_SDA__I2C2_SDA);

		//TPS65185_VCOM_OUT(1);

#ifdef GPIO_TPS65185_EP_3V3_IN //[
		gpio_direction_output(GPIO_TPS65185_EP_3V3_IN, 1);
#endif //]GPIO_TPS65185_EP_3V3_IN

		gpio_direction_output(GPIO_TPS65185_VIN, 1);
		gdwSafeTick_To_TurnON_RailPower = jiffies+4;

		mxc_iomux_v3_setup_pad(GPIO_TPS65185_PWRGOOD_INT_PADCFG); // POWERGOOD .
		mxc_iomux_v3_setup_pad(GPIO_TPS65185_INT_PADCFG); // EP_INT
		gpio_direction_input(GPIO_TPS65185_PWRGOOD);
		gpio_direction_input(GPIO_TPS65185_INT);


		mdelay(5);

	}
	else {


		// Cut off the TPS65185 power .
		//
		mxc_iomux_v3_setup_pad(GPIO_TPS65185_PWRGOOD_GPIO_PADCFG); // POWERGOOD
		mxc_iomux_v3_setup_pad(GPIO_TPS65185_INT_GPIO_PADCFG); // EP_INT
		//udelay(50);
		//mxc_iomux_v3_setup_pad(MX50_PAD_I2C2_SCL__GPIO_6_20);
		//gpio_request(GPIO_TPS65185_SDL, "tps65185_i2c_sdl");
		//gpio_direction_input (GPIO_TPS65185_SDL);
		//gpio_direction_output(GPIO_TPS65185_SDL, 1);

		//mxc_iomux_v3_setup_pad(MX50_PAD_I2C2_SDA__GPIO_6_21);
		//gpio_request(GPIO_TPS65185_SDA, "tps65185_i2c_sda");
		//gpio_direction_input (GPIO_TPS65185_SDL);
		//gpio_direction_output(GPIO_TPS65185_SDA, 1);


		gpio_direction_output(GPIO_TPS65185_PWRUP, 0);
		gpio_direction_output(GPIO_TPS65185_WAKEUP, 0);

		gpio_direction_output(GPIO_TPS65185_PWRGOOD, 0);
		gpio_direction_output(GPIO_TPS65185_INT, 0);

		TPS65185_VCOM_OUT(0);

		gpio_direction_output(GPIO_TPS65185_VIN, 0);
		gtTPS65185_DataA[0].iRestoreVCOM=gtTPS65185_DataA[0].iCurrentVCOM;
		gtTPS65185_DataA[0].iCurrentVCOM=0;

		down(&gtTPS65185_DataA[0].chmod_lock);
		gtTPS65185_DataA[0].dwCurrent_mode=TPS65185_MODE_SLEEP;
		up(&gtTPS65185_DataA[0].chmod_lock);
#ifdef GPIO_TPS65185_EP_3V3_IN //[
		gpio_direction_output(GPIO_TPS65185_EP_3V3_IN, 0);
#endif //]GPIO_TPS65185_EP_3V3_IN
	}

	return iRet;
#else //][!TPS65185_SUSPEND
	return 0;
#endif //] TPS65185_SUSPEND
}

// get latest interrupt state of TPS65185 .
int tps65185_int_state_get(void)
{
	if(giTPS65185_int_state>0) {
		return giTPS65185_int_state;
	}
	return -1;
}

// clear latest interrupt state of TPS65185 .
int tps65185_int_state_clear(void)
{
	giTPS65185_int_state=0;
	return 0;
}

int tps65185_int_callback_setup(TPS65185_INTEVT_CB fnCB)
{
	gpfnINTCB=fnCB;

	return 0;
}

void tps65185_irqs_enable(int iIsEnable)
{
	int irq,iChk;

	if(iIsEnable) {
#ifdef TPS65185_PWR_ONOFF_INT//[
		irq = gpio_to_irq(GPIO_TPS65185_INT);
		#ifdef TPS65185_PLATFORM_MX6//[
		iChk = request_irq(irq, tps65185_int, 0, "tps65185_INT", 0);
		if (iChk) {
			pr_info("register TPS65185 interrupt failed\n");
		}
		#else //][!TPS65185_PLATFORM_MX6
		enable_irq(irq);
		#endif //] TPS65185_PLATFORM_MX6


		irq = gpio_to_irq(GPIO_TPS65185_PWRGOOD);
		#ifdef TPS65185_PLATFORM_MX6//[
		iChk = request_irq(irq, tps65185_pwrgood_inthandler, 0, "tps65185_PWRGOOD", 0);
		if (iChk) {
			pr_info("register TPS65185 pwrgood interrupt failed\n");
		}
		#else //][!TPS65185_PLATFORM_MX6
		enable_irq(irq);
		#endif //] TPS65185_PLATFORM_MX6
#endif //]TPS65185_PWR_ONOFF_INT
	}
	else {
#ifdef TPS65185_PWR_ONOFF_INT//[
		irq = gpio_to_irq(GPIO_TPS65185_INT);
		#ifdef TPS65185_PLATFORM_MX6//[
		free_irq(irq,0);
		#else //][!TPS65185_PLATFORM_MX6
		disable_irq(irq);
		#endif //] TPS65185_PLATFORM_MX6


		irq = gpio_to_irq(GPIO_TPS65185_PWRGOOD);
		#ifdef TPS65185_PLATFORM_MX6//[
		free_irq(irq,0);
		#else //][!TPS65185_PLATFORM_MX6
		disable_irq(irq);
		#endif //] TPS65185_PLATFORM_MX6
#endif //] TPS65185_PWR_ONOFF_INT
	}
}

int tps65185_suspend(void)
{
#ifdef TPS65185_SUSPEND //[
	unsigned long dwTPS65185_mode;
	unsigned char bVal;
	int irq;
	int iRet = 0;
	unsigned long dwCurTick;
	int iIsIRQ_Disabled = 0;
	int iIsChkVEE_Stable=0;
	
	dbgENTER();


	//ERR_MSG(KERN_ERR "%s()\n",__FUNCTION__);
	if(gSleep_Mode_Suspend) {
		DBG_MSG("%s:Go to sleep mode\n",__FUNCTION__);
		//dwTPS65185_mode = TPS65185_MODE_SLEEP; // sleep mode will turn off EP_3V3 .
		dwTPS65185_mode = TPS65185_MODE_STANDBY;
	}
	else {
		dwTPS65185_mode = TPS65185_MODE_STANDBY;
	}

	if( TPS65185_MODE_SLEEP == dwTPS65185_mode ) {
		tps65185_irqs_enable(0);iIsIRQ_Disabled=1;
	}

	tps65185_chg_mode(&dwTPS65185_mode,1);

	//flush_workqueue(tps65185_pwrgood_workqueue);
	//flush_workqueue(tps65185_int_workqueue);
		
	//tps65185_wait_panel_poweroff();

	//bVal=0;
	//TPS65185_REG_SET(INT_EN1,ALL,bVal);
	//TPS65185_REG_SET(INT_EN2,ALL,bVal);
	//TPS65185_REG_SET(ENABLE,ALL,bVal);

	if(delayed_work_pending(&gtPwrdwn_work_param.pwrdwn_work)) {
		WARNING_MSG("pmic pwrdwn delay work pending !!\n");
		//flush_delayed_work(&gtPwrdwn_work_param.pwrdwn_work);
		iRet = -1;goto error_out;
	}

#if 0
	if( (1==gptHWCFG->m_val.bPMIC) && (/*PMIC is Ricoh */
	NTXHWCFG_TST_FLAG(gptHWCFG->m_val.bPCB_Flags,4)/*Panel is designed for low voltage */ ||
	NTXHWCFG_TST_FLAG(gptHWCFG->m_val.bPCB_Flags,5)/*EPD VDD source is standalone */ 
	) )
	{
		if(gSleep_Mode_Suspend) {
			iIsChkVEE_Stable=1;
		}
		else {
			iIsChkVEE_Stable=0;
		}
	}
	else {
		iIsChkVEE_Stable=1;
	}
#else
	if(gSleep_Mode_Suspend) {
		iIsChkVEE_Stable=1;
	}
	else {
		iIsChkVEE_Stable=0;
	}
#endif
	dwCurTick=jiffies;
	if(iIsChkVEE_Stable && time_before(dwCurTick,gdwSafeTick_To_TurnOFF_EP3V3))
	{
		WARNING_MSG("waiting for VEE stable ,please retry suspend later !!!\n");
		iRet = -2;goto error_out;
	}
	else {
		if(!iIsIRQ_Disabled) {
			tps65185_irqs_enable(0);iIsIRQ_Disabled=1;
		}
		dwTPS65185_mode = TPS65185_MODE_SLEEP;
		tps65185_chg_mode(&dwTPS65185_mode,1);
	}


	dbgLEAVE();
	return iRet;
error_out:
	if(iIsIRQ_Disabled) {
		tps65185_irqs_enable(1);
	}
	return iRet;
#else
	printk("%s() skipped !!\n",__FUNCTION__);	
	return 0;
#endif //]TPS65185_SUSPEND
}
EXPORT_SYMBOL(tps65185_suspend);

void tps65185_shutdown(void)
{
	unsigned long dwTPS65185_mode;
	unsigned long dwCurTick;

#if 0
	if(!delayed_work_pending(&gtPwrdwn_work_param.pwrdwn_work)) {
		dwTPS65185_mode = TPS65185_MODE_STANDBY;
		tps65185_chg_mode(&dwTPS65185_mode,1);
	}
#endif

	while (1) {
		if(delayed_work_pending(&gtPwrdwn_work_param.pwrdwn_work)) {
			DBG0_MSG("%s() : waiting for TPS65185 pwrdwn !!\n",__FUNCTION__);
			msleep(100);
		}
		else {
			break;
		}
	}

	while (1) {
		dwCurTick=jiffies;
		if(time_before(dwCurTick,gdwSafeTick_To_TurnOFF_EP3V3)) {
			DBG0_MSG("%s() : waiting for VEE stable to turn off EP3V3 .\n",__FUNCTION__);
			msleep(100);
		}
		else {
			dwTPS65185_mode = TPS65185_MODE_SLEEP;
			tps65185_chg_mode(&dwTPS65185_mode,1);
			break;
		}
	}
	
}
EXPORT_SYMBOL(tps65185_shutdown);

void tps65185_resume(void)
{
#ifdef TPS65185_SUSPEND //[
	unsigned long dwTPS65185_mode,dwTPS65185_current_mode;

	dbgENTER();
	dwTPS65185_current_mode=gtTPS65185_DataA[0].dwCurrent_mode;

	dwTPS65185_mode = TPS65185_MODE_STANDBY;
	tps65185_chg_mode(&dwTPS65185_mode,1);

	if( TPS65185_MODE_SLEEP == dwTPS65185_current_mode ) {
		tps65185_irqs_enable(1);
	}

	//tps65185_reg_init(1);

	dbgLEAVE();
#else
	printk("%s() skipped !!\n",__FUNCTION__);	
#endif //]TPS65185_SUSPEND
}
EXPORT_SYMBOL(tps65185_resume);


int tps65185_v3p3_on(void)
{
	return 0;
}
EXPORT_SYMBOL(tps65185_v3p3_on);

void tps65185_v3p3_off(void)
{
}
EXPORT_SYMBOL(tps65185_v3p3_off);


//EXPORT_SYMBOL(tps65185_init);
//EXPORT_SYMBOL(tps65185_release);
//EXPORT_SYMBOL(tps65185_get_temperature);

