#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <mach/upmu_common.h>

#include <mach/mt_gpio.h>		// For gpio control

/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "leds_strobe.c"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        xlog_printk(ANDROID_LOG_WARNING, TAG_NAME, KERN_WARNING  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_NOTICE  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        xlog_printk(ANDROID_LOG_INFO   , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME,  "<%s>\n", __FUNCTION__);
#define PK_TRC_VERBOSE(fmt, arg...) xlog_printk(ANDROID_LOG_VERBOSE, TAG_NAME,  fmt, ##arg)
#define PK_ERROR(fmt, arg...)       xlog_printk(ANDROID_LOG_ERROR  , TAG_NAME, KERN_ERR "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/
static DEFINE_SPINLOCK(g_strobeSMPLock);
static u32 strobe_Res = 0;
static BOOL g_strobe_On = 0;
static int g_duty=-1;
static int g_step=-1;
static int g_timeOutTimeMs=0;

static struct work_struct workTimeOut;
/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);

#define LEDS_TORCH_MODE 		0
#define LEDS_FLASH_MODE 		1
#define LEDS_CUSTOM_MODE_THRES 	20

/*****************************************************************************
Functions
*****************************************************************************/
//#define GPIO_ENF GPIO_CAMERA_FLASH_EN_PIN
//#define GPIO_ENT GPIO_CAMERA_FLASH_MODE_PIN

#define GPIO_CAMERA_FLASH_MODE GPIO_CAMERA_FLASH_MODE_PIN //GPIO28
#define GPIO_CAMERA_FLASH_MODE_M_GPIO  GPIO_MODE_00

#define GPIO_CAMERA_FLASH_EN GPIO_CAMERA_FLASH_EN_PIN	//GPIO31
#define GPIO_CAMERA_FLASH_EN_M_GPIO  GPIO_MODE_00

#define GPIO_FLASH_LEVEL GPIO_CAMERA_FLASH_LEVEL_PIN	//GPIO141
#define GPIO_FLASH_LEVEL_M_GPIO  GPIO_MODE_00
    /*CAMERA-FLASH-EN */


int FL_enable(void)
{
	if(mt_set_gpio_mode(GPIO_CAMERA_FLASH_EN,GPIO_CAMERA_FLASH_MODE_M_GPIO)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
	if(mt_set_gpio_dir(GPIO_CAMERA_FLASH_EN,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
	if(mt_set_gpio_out(GPIO_CAMERA_FLASH_EN,GPIO_OUT_ONE)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
	PK_DBG("[constant_flashlight] set gpio %d %s \n",GPIO_CAMERA_FLASH_EN,GPIO_OUT_ONE?"HIGH":"LOW");

	return 0;

}

int FL_disable(void)
{
	if(mt_set_gpio_mode(GPIO_CAMERA_FLASH_EN,GPIO_CAMERA_FLASH_MODE_M_GPIO)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
	if(mt_set_gpio_dir(GPIO_CAMERA_FLASH_EN,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
	if(mt_set_gpio_out(GPIO_CAMERA_FLASH_EN,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
		PK_DBG("[constant_flashlight] set gpio %d %s \n",GPIO_CAMERA_FLASH_EN,GPIO_OUT_ONE?"HIGH":"LOW");
	if(g_duty == 0)
	{
		if(mt_set_gpio_mode(GPIO_CAMERA_FLASH_MODE,GPIO_CAMERA_FLASH_MODE_M_GPIO)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
		if(mt_set_gpio_dir(GPIO_CAMERA_FLASH_MODE,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
		if(mt_set_gpio_out(GPIO_CAMERA_FLASH_MODE,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
	}else if(g_duty == 2)
	{
		if(mt_set_gpio_mode(GPIO_FLASH_LEVEL, GPIO_FLASH_LEVEL_M_GPIO)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
		if(mt_set_gpio_dir(GPIO_FLASH_LEVEL,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
		if(mt_set_gpio_out(GPIO_FLASH_LEVEL,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
	}
	return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	g_duty=duty;
	switch (duty){
	case 0://in torch mode
		PK_DBG("set torch mode\n");
		if(mt_set_gpio_mode(GPIO_FLASH_LEVEL, GPIO_FLASH_LEVEL_M_GPIO)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
		if(mt_set_gpio_dir(GPIO_FLASH_LEVEL,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
		if(mt_set_gpio_out(GPIO_FLASH_LEVEL,GPIO_OUT_ONE)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
		if(mt_set_gpio_mode(GPIO_CAMERA_FLASH_MODE,GPIO_CAMERA_FLASH_MODE_M_GPIO)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
		if(mt_set_gpio_dir(GPIO_CAMERA_FLASH_MODE,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
		if(mt_set_gpio_out(GPIO_CAMERA_FLASH_MODE,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
	break;

	case 1://in pre flash & af flash mode
		PK_DBG("set pre flash & af flash mdoe\n");
		if(mt_set_gpio_mode(GPIO_FLASH_LEVEL, GPIO_FLASH_LEVEL_M_GPIO)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
		if(mt_set_gpio_dir(GPIO_FLASH_LEVEL,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
		if(mt_set_gpio_out(GPIO_FLASH_LEVEL,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
		if(mt_set_gpio_mode(GPIO_CAMERA_FLASH_MODE,GPIO_CAMERA_FLASH_MODE_M_GPIO)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
		if(mt_set_gpio_dir(GPIO_CAMERA_FLASH_MODE,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
		if(mt_set_gpio_out(GPIO_CAMERA_FLASH_MODE,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
	break;

	case 2://in main flash mode
		PK_DBG("set main flash mode\n");
		if(mt_set_gpio_mode(GPIO_FLASH_LEVEL, GPIO_FLASH_LEVEL_M_GPIO)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
		if(mt_set_gpio_dir(GPIO_FLASH_LEVEL,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
		if(mt_set_gpio_out(GPIO_FLASH_LEVEL,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
		if(mt_set_gpio_mode(GPIO_CAMERA_FLASH_MODE,GPIO_CAMERA_FLASH_MODE_M_GPIO)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
		if(mt_set_gpio_dir(GPIO_CAMERA_FLASH_MODE,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
		if(mt_set_gpio_out(GPIO_CAMERA_FLASH_MODE,GPIO_OUT_ONE)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
	break;

     default:
		PK_DBG("error duty=%d value !!!%d\n",duty);
	break;
	}
	PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
	return 0;
}

int FL_step(kal_uint32 step)
{
	//int sTab[8]={0,2,4,6,9,11,13,15};
	PK_DBG("FL_step");
//	upmu_set_flash_sel(sTab[step]);
    return 0;
}

int FL_init(void)
{
//	upmu_set_flash_dim_duty(0);
//	upmu_set_flash_sel(0);
    if(mt_set_gpio_mode(GPIO_CAMERA_FLASH_MODE,GPIO_CAMERA_FLASH_MODE_M_GPIO)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_CAMERA_FLASH_MODE,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_CAMERA_FLASH_MODE,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
    /*Init. to disable*/
    if(mt_set_gpio_mode(GPIO_CAMERA_FLASH_EN,GPIO_CAMERA_FLASH_MODE_M_GPIO)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_CAMERA_FLASH_EN,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_CAMERA_FLASH_EN,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
    if(mt_set_gpio_mode(GPIO_FLASH_LEVEL,GPIO_FLASH_LEVEL_M_GPIO)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_FLASH_LEVEL,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_FLASH_LEVEL,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}

	INIT_WORK(&workTimeOut, work_timeOutFunc);
    return 0;
}

int FL_uninit(void)
{
	PK_DBG("FL_uninit");

	FL_disable();
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data)
{
	FL_disable();
    PK_DBG("ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}
enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	PK_DBG("ledTimeOut_callback\n");
	schedule_work(&workTimeOut);

    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

}

static int constant_flashlight_ioctl(MUINT32 cmd, MUINT32 arg)
{
	int i4RetValue = 0;
	int iFlashType = (int)FLASHLIGHT_NONE;
	int ior;
	int iow;
	int iowr;
	ior = _IOR(FLASHLIGHT_MAGIC,0, int);
	iow = _IOW(FLASHLIGHT_MAGIC,0, int);
	iowr = _IOWR(FLASHLIGHT_MAGIC,0, int);
	PK_DBG("constant_flashlight_ioctl() line=%d cmd=%d, ior=%d, iow=%d iowr=%d arg=%d\n",__LINE__, cmd, ior, iow, iowr, arg);
	PK_DBG("constant_flashlight_ioctl() line=%d cmd-ior=%d, cmd-iow=%d cmd-iowr=%d arg=%d\n",__LINE__, cmd-ior, cmd-iow, cmd-iowr, arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);
			g_timeOutTimeMs=arg;
			break;

    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",arg);
    		g_duty=arg;
    		FL_dim_duty(arg);
    		break;

    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",arg);
    		g_step=arg;
    		FL_step(arg);
    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",arg);
    		if(arg==1)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
    			FL_enable();
    			g_strobe_On=1;
    		}
    		else
    		{
    			FL_disable();
				hrtimer_cancel( &g_timeOutTimer );
				g_strobe_On=0;
    		}
    		break;
        case FLASHLIGHTIOC_G_FLASHTYPE:
            iFlashType = FLASHLIGHT_LED_CONSTANT;
            if(copy_to_user((void __user *) arg , (void*)&iFlashType , _IOC_SIZE(cmd)))
            {
                PK_DBG("[strobe_ioctl] ioctl copy to user failed\n");
                return -EFAULT;
            }
            break;
		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}

static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    FL_init();
		timerInit();
	}
	spin_lock_irq(&g_strobeSMPLock);

    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }

    spin_unlock_irq(&g_strobeSMPLock);

    return i4RetValue;

}

static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

    	FL_uninit();
    }

    PK_DBG(" Done\n");

    return 0;

}

FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};

MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}

/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

    return 0;
}

EXPORT_SYMBOL(strobe_VDIrq);
