/* drivers/hwmon/mt6516/amit/tmd2772.c - TMD2772 ALS/PS driver
 *
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
//#include <mach/mt_gpio.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include <linux/wakelock.h>
#include "tmd2772.h"
extern struct alsps_hw *tmd2772_get_cust_alsps_hw(void);
struct wake_lock psensor_lock;
/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/
#define TMD2772_DEV_NAME     "TMD2772"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(APS_TAG fmt, ##args)
static int TMD2772_CMM_PPCOUNT_VALUE =6;//0xA;// ppcount from 10 to 6 modified by jy
static int ZOOM_TIME = 50;//60
static int TMD2772_CMM_CONTROL_VALUE = 0x60;//0x20;//current from 100ma to 50ma modified by jy
static u8  offset_data=0;
#define DO_CALIBARTION 1  //modify by tao.zhang. 2012-12-18. calibartion donot work good.
#define PRO_OFFSET 1
static u16 tmp_data=0;
static u16 tmp_als_value = 0;
static u16 als_sample = 0;
#define OFFDATA_DEFAULT 1
#define ps_thd_high (80+40) //jy modify
#define ps_thd_low  (50+40)// jy modify

/* Sampling 20 data to get average value */
struct mutex mutex_lux;
#define lux_num 5
typedef struct
{
	u16 lux_value[lux_num];
	u16 wr_index;
	u16 num;
}lux_queue;
static lux_queue *lux_FIFO;
/******************************************************************************
 * extern functions
*******************************************************************************/
//jy modify 2014.6.23
/*for interrup work mode support --add by liaoxl.lenovo 12.08.2011*/

#ifdef CUST_EINT_ALS_TYPE
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);
#else
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);

#endif

//end//

/*----------------------------------------------------------------------------*/
static struct i2c_client *tmd2772_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id tmd2772_i2c_id[] = {{TMD2772_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_TMD2772={ I2C_BOARD_INFO("TMD2772", (0X72>>1))};
/*the adapter id & i2c address will be available in customization*/
static unsigned short tmd2772_force[] = {0x02, 0X72, I2C_CLIENT_END, I2C_CLIENT_END};
static const unsigned short *const tmd2772_forces[] = { tmd2772_force, NULL };
//static struct i2c_client_address_data tmd2772_addr_data = { .forces = tmd2772_forces,};
/*----------------------------------------------------------------------------*/
static int tmd2772_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tmd2772_i2c_remove(struct i2c_client *client);
static int tmd2772_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int  tmd2772_local_init(void);
static int tmd2772_remove(void);
/*----------------------------------------------------------------------------*/
static int tmd2772_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int tmd2772_i2c_resume(struct i2c_client *client);
static int store_status(unsigned int *flag);
static unsigned int alsps_enable_log=0;
static u8 store_enable_register=0;
static struct tmd2772_priv *g_tmd2772_ptr = NULL;
static int tmd2772_init_client(struct i2c_client *client);
static void tmd2772_ps_calibrate_call(struct i2c_client *client);
static void tmd2772_ps_calibrate(struct i2c_client *client);
static int tmd2772_init_client_for_cali_call(struct i2c_client *client);
static int tmd2772_init_client_for_cali_restore(struct i2c_client *client);
static int en_ps = 1,ps_value;
static u8 store_als_level = 0,temp_als_level=0, store_als_idx = 0;
static unsigned int als_value_arry[4]  = {10, 630, 6000, 20000};
 struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
} ;
static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
static int intr_flag_value = 0;
static unsigned int temp_ps_data = 0;
struct mutex mutex;
/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;
/*----------------------------------------------------------------------------*/
struct tmd2772_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;
    u8  ps_thd;     /*PS INT threshold*/
};
/*----------------------------------------------------------------------------*/
struct tmd2772_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
	struct work_struct  eint_work;
    //struct delayed_work  eint_work;
	//struct hwmsen_convert   cvt;
    /*i2c address group*/
    struct tmd2772_i2c_addr  addr;

    /*misc*/
    u16		    als_modulus;
    atomic_t    i2c_retry;
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;
    /*data*/
    u16         als;
    u16          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL];
    u32         als_value[C_CUST_ALS_LEVEL];
    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val_high;     /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_low;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif
};
static struct sensor_init_info tmd2772_init_info = {
		.name = "tmd2772",
		.init = tmd2772_local_init,
		.uninit = tmd2772_remove,
};
static int tmd2772_init_flag = -1;
static unsigned char i2c_addr[C_CUST_I2C_ADDR_NUM] = {0x72, 0x48, 0x78, 0x00};
//#ifdef GEMINI
static unsigned int als_level[C_CUST_ALS_LEVEL] = { 4,  8,  10, 180, 260, 300, 400, 630,   3500, 4500, 6000, 7000, 7500, 9000, 20000, 65535};
static unsigned int als_value[C_CUST_ALS_LEVEL]  = {10, 10, 10, 280, 280, 280, 280,  280,  2200, 2200, 2200, 6000, 6000,  6000,  6000, 10240};
//#else
//static unsigned int als_level[C_CUST_ALS_LEVEL]  = { 4, 38, 85, 130, 160, 300, 400, 2500, 3000, 4500, 6000, 7000, 10000, 10000, 11000, 65535};
//static unsigned int als_value[C_CUST_ALS_LEVEL]  = {10, 10, 10, 280, 280, 280, 280,  280, 2200, 2200, 2200, 2200,  6000,  6000,  6000, 10240};
//#endif
static unsigned int ps_threshold_high = 260;//500,set off screen distance is 3.5cm,test on phantom
static unsigned int ps_threshold_low = 240;//400,set bright screen distanch is about 3.7cm
static unsigned int ps_threshold = 260;
/*----------------------------------------------------------------------------*/
static struct i2c_driver tmd2772_i2c_driver = {
	.probe      = tmd2772_i2c_probe,
	.remove     = tmd2772_i2c_remove,
	.suspend    = tmd2772_i2c_suspend,
	.resume     = tmd2772_i2c_resume,
	.id_table   = tmd2772_i2c_id,
	//.address_data = &tmd2772_addr_data,
	.detect		  = tmd2772_i2c_detect,
	.driver = {
		.owner          = THIS_MODULE,
		.name           = TMD2772_DEV_NAME,
	},
};
static struct tmd2772_priv *tmd2772_obj = NULL;
static int product_id = 0;
/*----------------------------------------------------------------------------*/
int tmd2772_get_addr(struct alsps_hw *hw, struct tmd2772_i2c_addr *addr)
{
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static void tmd2772_power(struct alsps_hw *hw, unsigned int on)
{
	static unsigned int power_on = 0;
	//APS_LOG("power %s\n", on ? "on" : "off");
	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "TMD2772"))
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "TMD2772"))
			{
				APS_ERR("power off fail!!\n");
			}
		}
	}
	power_on = on;
}
/*----------------------------------------------------------------------------*/
static long tmd2772_enable_als(struct i2c_client *client, int enable)
{
		struct tmd2772_priv *obj = i2c_get_clientdata(client);
		u8 databuf[2];
		long res = 0;
		//u8 buffer[1];
		//u8 reg_value[1];
		uint32_t testbit_PS;

		if(client == NULL)
		{
			APS_DBG("CLIENT CANN'T EQUL NULL\n");
			return -1;
		}

		#if 0	/*yucong MTK enable_als function modified for fixing reading register error problem 2012.2.16*/
		buffer[0]=TMD2772_CMM_ENABLE;
		res = i2c_master_send(client, buffer, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		res = i2c_master_recv(client, reg_value, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);

		if(enable)
		{
			databuf[0] = TMD2772_CMM_ENABLE;
			databuf[1] = reg_value[0] |0x0B;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			/*Lenovo-sw chenlj2 add 2011-06-03,modify ps to ALS below two lines */
			atomic_set(&obj->als_deb_on, 1);
			atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
			APS_DBG("tmd2772 power on\n");
		}
		else
		{
			databuf[0] = TMD2772_CMM_ENABLE;
			databuf[1] = reg_value[0] &0xFD;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			/*Lenovo-sw chenlj2 add 2011-06-03,modify ps_deb_on to als_deb_on */
			atomic_set(&obj->als_deb_on, 0);
			APS_DBG("tmd2772 power off\n");
		}
		#endif
		#if 1
		/*yucong MTK enable_als function modified for fixing reading register error problem 2012.2.16*/
		testbit_PS = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
		if(enable)
		{
			if(testbit_PS){
			databuf[0] = TMD2772_CMM_ENABLE; //ox80
			if(0 == obj->hw->polling_mode_ps)/*ckt sw shutao.lan 2013.02.06,set psensor mode*/
				databuf[1] = 0x2F;
			else
				databuf[1] = 0x0F;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			/*debug code for reading register value*/
			#if 0
			res = i2c_master_recv(client, reg_value, 0x1);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
			#endif
			}
			else{
			databuf[0] = TMD2772_CMM_ENABLE;//TMD2772_CMM_ENABLE = 0x80
			if(0 == obj->hw->polling_mode_ps)/*ckt sw shutao.lan 2013.02.06,set psensor mode*/
				databuf[1] = 0x2B;
			else
				databuf[1] = 0x0B;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			/*debug code for reading register value*/
			#if 0
			res = i2c_master_recv(client, reg_value, 0x1);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
			#endif
			}
			atomic_set(&obj->als_deb_on, 1);
			atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
			APS_DBG("tmd2772 power on\n");
		}
		else
		{
			if(testbit_PS){
			databuf[0] = TMD2772_CMM_ENABLE;
			if(0 == obj->hw->polling_mode_ps)/*ckt sw shutao.lan 2013.02.06,set psensor mode*/
				databuf[1] = 0x2D;
			else
				databuf[1] = 0x0D;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			}
			else{
			databuf[0] = TMD2772_CMM_ENABLE;
			databuf[1] = 0x00;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			}
			/*Lenovo-sw chenlj2 add 2011-06-03,modify ps_deb_on to als_deb_on */
			atomic_set(&obj->als_deb_on, 0);
			APS_DBG("tmd2772 power off\n");
		}
		#endif
		#if 0 /*yucong add for debug*/
			buffer[0]=TMD2772_CMM_ENABLE;
			res = i2c_master_send(client, buffer, 0x1);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			res = i2c_master_recv(client, reg_value, 0x1);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
		#endif

		return 0;

	EXIT_ERR:
		APS_ERR("tmd2772_enable_als fail\n");
		return res;
}
/*----------------------------------------------------------------------------*/
static long tmd2772_enable_ps(struct i2c_client *client, int enable)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	long res = 0;
//	u8 buffer[1];
//	u8 reg_value[1];
	uint32_t testbit_ALS;
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
#if 0	/*yucong MTK modified for fixing reading register error problem 2012.2.16*/
	buffer[0]=TMD2772_CMM_ENABLE;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, reg_value, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	/*yucong MTK: lenovo orignal code*/
	if(enable)
	{
		databuf[0] = TMD2772_CMM_ENABLE;
		databuf[1] = reg_value[0] |0x0d;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
		APS_DBG("tmd2772 power on\n");
		/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
		if(0 == obj->hw->polling_mode_ps)
		{
			if(1 == ps_cali.valid)
			{
				databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;
				databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;
				databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;
				databuf[1] = (u8)(ps_cali.close & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH;
				databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);;
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
			}
			else
			{
				databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;
				databuf[1] = (u8)(480 & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;
				databuf[1] = (u8)((480 & 0xFF00) >> 8);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;
				databuf[1] = (u8)(700 & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH;
				databuf[1] = (u8)((700 & 0xFF00) >> 8);;
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}

			}

			databuf[0] = TMD2772_CMM_Persistence;
			databuf[1] = 0x20;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
			databuf[0] = TMD2772_CMM_ENABLE;
			databuf[1] = reg_value[0] | 0x0d | 0x20;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}

			mt_eint_unmask(CUST_EINT_ALS_NUM);
		}
	}
	else
	{
		databuf[0] = TMD2772_CMM_ENABLE;
		databuf[1] = reg_value[0] &0xfb;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 0);
		APS_DBG("tmd2772 power off\n");
		/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
		if(0 == obj->hw->polling_mode_ps)
		{
			cancel_work_sync(&obj->eint_work);
			mt_eint_mask(CUST_EINT_ALS_NUM);
		}
	}
#endif
#if 1
	/*yucong MTK: enable_ps function modified for fixing reading register error problem 2012.2.16*/
	testbit_ALS = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
	if(enable)
	{
		if(testbit_ALS){
		databuf[0] = TMD2772_CMM_ENABLE;
		databuf[1] = 0x0F;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
			{
				goto EXIT_ERR;
			}
		/*debug code for reading register value*/
		#if 0
		res = i2c_master_recv(client, reg_value, 0x1);
		if(res <= 0)
			{
				goto EXIT_ERR;
			}
		printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
		#endif
		}else{
		databuf[0] = TMD2772_CMM_ENABLE;
		databuf[1] = 0x0D;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
			{
				goto EXIT_ERR;
			}
		}
		/*debug code for reading register value*/
		#if 0
		res = i2c_master_recv(client, reg_value, 0x1);
		if(res <= 0)
			{
				goto EXIT_ERR;
			}
		printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
		#endif
		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
		APS_DBG("tmd2772 power on\n");

#if CKT_HALL_SWITCH_SUPPORT
		g_is_calling = 1;
#endif
	
		/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
		if(0 == obj->hw->polling_mode_ps)
		{
			if(1 == ps_cali.valid)
			{
				databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;
				databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;
				databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;
				databuf[1] = (u8)(ps_cali.close & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH;
				databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);;
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
			}
			else
			{	/*ckt sw shutao.lan 2013.02.06,unified psensor threshold*/
				databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;
				databuf[1] = (u8)(atomic_read(&obj->ps_thd_val_low) & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;
				databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low) & 0xFF00) >> 8);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;
				databuf[1] = (u8)(atomic_read(&obj->ps_thd_val_high) & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH;
				databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high) & 0xFF00) >> 8);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}

			}

			databuf[0] = TMD2772_CMM_Persistence;
			databuf[1] = 0x20;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
			if(testbit_ALS){
			databuf[0] = TMD2772_CMM_ENABLE;
			databuf[1] = 0x2F;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			/*debug code for reading register value*/
			#if 0
			res = i2c_master_recv(client, reg_value, 0x1);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
			#endif
			}else{
			databuf[0] = TMD2772_CMM_ENABLE;
			databuf[1] = 0x2D;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			}
			/*debug code for reading register value*/
			#if 0
			res = i2c_master_recv(client, reg_value, 0x1);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
			#endif
			#ifdef CUST_EINT_ALS_TYPE
			mt_eint_unmask(CUST_EINT_ALS_NUM);
			#else
			mt65xx_eint_mask(CUST_EINT_ALS_NUM);
			#endif
		}
	}
	else
	{
	/*yucong MTK: enable_ps function modified for fixing reading register error problem 2012.2.16*/
	if(testbit_ALS){
		databuf[0] = TMD2772_CMM_ENABLE;
		if(0 == obj->hw->polling_mode_ps)/*ckt sw shutao.lan 2013.02.06,set psensor mode*/
			databuf[1] = 0x2B;
		else
			databuf[1] = 0x0B;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
			{
				goto EXIT_ERR;
			}
		}else{
		databuf[0] = TMD2772_CMM_ENABLE;
		databuf[1] = 0x00;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
			{
				goto EXIT_ERR;
			}
		}
		atomic_set(&obj->ps_deb_on, 0);
		APS_DBG("tmd2772 power off\n");

#if CKT_HALL_SWITCH_SUPPORT
		g_is_calling = 0;
#endif

		/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
		if(0 == obj->hw->polling_mode_ps)
		{
			cancel_work_sync(&obj->eint_work);
			#ifdef CUST_EINT_ALS_TYPE
			mt_eint_mask(CUST_EINT_ALS_NUM);
			#else
			mt65xx_eint_mask(CUST_EINT_ALS_NUM);
			#endif
			
		}
	}
#endif
	return 0;

EXIT_ERR:
	APS_ERR("tmd2772_enable_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
#if 0
static int tmd2772_enable(struct i2c_client *client, int enable)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	int res = 0;
	u8 buffer[1];
	u8 reg_value[1];
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	/* modify to restore reg setting after cali ---liaoxl.lenovo */
	buffer[0]=TMD2772_CMM_ENABLE;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, reg_value, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	if(enable)
	{
		databuf[0] = TMD2772_CMM_ENABLE;
		databuf[1] = reg_value[0] | 0x01;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		APS_DBG("tmd2772 power on\n");
	}
	else
	{
		databuf[0] = TMD2772_CMM_ENABLE;
		databuf[1] = reg_value[0] & 0xFE;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 0);
		/*Lenovo-sw chenlj2 add 2011-06-03,close als_deb_on */
		atomic_set(&obj->als_deb_on, 0);
		APS_DBG("tmd2772 power off\n");
	}
	return 0;

EXIT_ERR:
	APS_ERR("tmd2772_enable fail\n");
	return res;
}
#endif
/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
static int tmd2772_check_and_clear_intr(struct i2c_client *client)
{
	//struct tmd2772_priv *obj = i2c_get_clientdata(client);
	int res,intp,intl;
	u8 buffer[2];
	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/
	//    return 0;
	buffer[0] = TMD2772_CMM_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong tmd2772_check_and_clear_intr status=0x%x\n", buffer[0]);
	res = 1;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x20))
	{
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x10))
	{
		res = 0;
		intl = 1;
	}
	if(0 == res)
	{
		if((1 == intp) && (0 == intl))
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x05);
		}
		else if((0 == intp) && (1 == intl))
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x06);
		}
		else
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);
		}
		res = i2c_master_send(client, buffer, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		else
		{
			res = 0;
		}
	}
	return 0; /*ckt sw shutao.lan added 2013.01.31,if no irq,it will return error*/
EXIT_ERR:
	APS_ERR("tmd2772_check_and_clear_intr fail\n");
	return 1;
}
/*----------------------------------------------------------------------------*/
/*yucong add for interrupt mode support MTK inc 2012.3.7*/
static int tmd2772_check_intr(struct i2c_client *client)
{
//	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	int res,intp,intl;
	u8 buffer[2];
	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/
	//    return 0;
	buffer[0] = TMD2772_CMM_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//APS_ERR("tmd2772_check_and_clear_intr status=0x%x\n", buffer[0]);
	res = 1;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x20))
	{
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x10))
	{
		res = 0;
		intl = 1;
	}
	return res;
EXIT_ERR:
	APS_ERR("tmd2772_check_intr fail\n");
	return 1;
}
static int tmd2772_clear_intr(struct i2c_client *client)
{
//	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 buffer[2];
#if 0
	if((1 == intp) && (0 == intl))
	{
		buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x05);
	}
	else if((0 == intp) && (1 == intl))
	{
		buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x06);
	}
	else
#endif
	{
		buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);
	}
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	else
	{
		res = 0;
	}
	return res;
EXIT_ERR:
	APS_ERR("tmd2772_check_and_clear_intr fail\n");
	return 1;
}
/*-----------------------------------------------------------------------------*/
void tmd2772_eint_func(void)
{
	APS_FUN();
	struct tmd2772_priv *obj = tmd2772_obj;
	if(!obj)
	{
		return;
	}

	schedule_work(&obj->eint_work);
	//schedule_delayed_work(&obj->eint_work);
}
/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
int tmd2772_setup_eint(struct i2c_client *client)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	tmd2772_obj = obj;

	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);
	#ifdef CUST_EINT_ALS_TYPE
	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, tmd2772_eint_func, 0);
	mt_eint_unmask(CUST_EINT_ALS_NUM);
       #else
	mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
       mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
	mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
	mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, tmd2772_eint_func, 0);
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
	#endif
	
	
    return 0;
}
/*----------------------------------------------------------------------------*/
#if 1
static int tmd2772_init_client_for_cali_call(struct i2c_client *client)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	int res = 0;

	databuf[0] = TMD2772_CMM_ENABLE;
	databuf[1] = 0x01;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	databuf[0] = TMD2772_CMM_ATIME;
	databuf[1] = 0xff;//0xEE
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	databuf[0] = TMD2772_CMM_PTIME;
	databuf[1] = 0xFF;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	databuf[0] = TMD2772_CMM_WTIME;
	databuf[1] = 0xFF;//0xFF
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	databuf[0] = TMD2772_CMM_CONFIG;
	databuf[1] = 0x00;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	databuf[0] = TMD2772_CMM_PPCOUNT;
	databuf[1] = TMD2772_CMM_PPCOUNT_VALUE;//0x02
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	databuf[0] = TMD2772_CMM_CONTROL;
	databuf[1] = TMD2772_CMM_CONTROL_VALUE;//0x22
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
#if DO_CALIBARTION
  #if PRO_OFFSET
	databuf[0] = TMD2772_CMM_OFFSET;
	databuf[1] = offset_data;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
        goto EXIT_ERR;
        return TMD2772_ERR_I2C;
	}
  #endif
#endif
	databuf[0] = TMD2772_CMM_ENABLE;
	databuf[1] = 0x05;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	return TMD2772_SUCCESS;
EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}
static int tmd2772_init_client_for_cali(struct i2c_client *client)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	int res = 0;

	databuf[0] = TMD2772_CMM_ENABLE;
	databuf[1] = 0x01;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	databuf[0] = TMD2772_CMM_ATIME;
	databuf[1] = 0xEE;//0xEE
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	databuf[0] = TMD2772_CMM_PTIME;
	databuf[1] = 0xFF;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	databuf[0] = TMD2772_CMM_WTIME;
	databuf[1] = 0xFF;//0xFF
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	databuf[0] = TMD2772_CMM_CONFIG;
	databuf[1] = 0x00;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	databuf[0] = TMD2772_CMM_PPCOUNT;
	databuf[1] = TMD2772_CMM_PPCOUNT_VALUE;//0x02
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	databuf[0] = TMD2772_CMM_CONTROL;
	databuf[1] = TMD2772_CMM_CONTROL_VALUE;//0x22
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
#if DO_CALIBARTION //ckt shutao.lan 2013.2.25,setting the offset
  #if PRO_OFFSET
	databuf[0] = TMD2772_CMM_OFFSET;
	databuf[1] = 0x00;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
        goto EXIT_ERR;
        return TMD2772_ERR_I2C;
	}
  #endif
#endif
	databuf[0] = TMD2772_CMM_ENABLE;
	databuf[1] = 0x05;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return TMD2772_ERR_I2C;
		}
	return TMD2772_SUCCESS;
EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}
#endif
static int tmd2772_init_client(struct i2c_client *client)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	int res = 0;

	databuf[0] = TMD2772_CMM_ENABLE;
	databuf[1] = 0x00;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	databuf[0] = TMD2772_CMM_ATIME;
	databuf[1] = 0xEE;//0xF6
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	databuf[0] = TMD2772_CMM_PTIME;
	databuf[1] = 0xFF;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	databuf[0] = TMD2772_CMM_WTIME;
	databuf[1] = 0xEE;//0xFC,this is suggest by FAE
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if(0 == obj->hw->polling_mode_ps)
	{
		if(1 == ps_cali.valid)
		{
			databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;
			databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
			databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;
			databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
			databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;
			databuf[1] = (u8)(ps_cali.close & 0x00FF);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
			databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH;
			databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
		}
		else
		{   /*ckt sw shutao.lan 2013.02.06,unified psensor threshold*/
			databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;
			databuf[1] = (u8)(atomic_read(&obj->ps_thd_val_low) & 0x00FF);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
			databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;
			databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low) & 0xFF00) >> 8);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
			databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;
			databuf[1] = (u8)(atomic_read(&obj->ps_thd_val_high) & 0x00FF);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
			databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH;
			databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high) & 0xFF00) >> 8);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
		}
		databuf[0] = TMD2772_CMM_Persistence;
		databuf[1] = 0x20;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return TMD2772_ERR_I2C;
		}
		databuf[0] = TMD2772_CMM_ENABLE;
		databuf[1] = 0x20;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return TMD2772_ERR_I2C;
		}
	}
	databuf[0] = TMD2772_CMM_CONFIG;
	databuf[1] = 0x00;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
       /*Lenovo-sw chenlj2 add 2011-06-03,modified pulse 2  to 4 */
	databuf[0] = TMD2772_CMM_PPCOUNT;
	databuf[1] = TMD2772_CMM_PPCOUNT_VALUE;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
        /*Lenovo-sw chenlj2 add 2011-06-03,modified gain 16  to 1 */
	databuf[0] = TMD2772_CMM_CONTROL;
	databuf[1] = TMD2772_CMM_CONTROL_VALUE;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	databuf[0] = TMD2772_CMM_ID;/*ckt shutao.lan 2013.2.25,get the device ID number*/
	res = i2c_master_send(client, databuf, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	res = i2c_master_recv(client, databuf, 0x1);
	if(res <= 0)
	{
	    goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	product_id = databuf[0];
    printk("ALSPS ID: %x\n",databuf[0]);//0x30 or 0x39
 #if DO_CALIBARTION /*ckt shutao.lan 2013.2.25*/
   #if PRO_OFFSET
     //printk("--@init client set offset_data:%d\n",offset_data);
     databuf[0] = TMD2772_CMM_OFFSET;
     databuf[1] = offset_data;
     res = i2c_master_send(client, databuf, 0x2);
     if(res <= 0)
     {
        goto EXIT_ERR;
        return TMD2772_ERR_I2C;
     }
  #endif
#else
   #if PRO_OFFSET
     databuf[0] = TMD2772_CMM_OFFSET;
     databuf[1] = 0x00;
     res = i2c_master_send(client, databuf, 0x2);
     if(res <= 0)
     {
         goto EXIT_ERR;
         return TMD2772_ERR_I2C;
     }
   #endif
#endif
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if(0 == obj->hw->polling_mode_ps)
	{
		if((res = tmd2772_setup_eint(client))!=0)
		{
			APS_ERR("setup eint: %d\n", res);
			return res;
        }

		if((res = tmd2772_check_and_clear_intr(client)))
		{
			APS_ERR("check/clear intr: %d\n", res);
			//    return res;
		}
	}
	return TMD2772_SUCCESS;
EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}
/******************************************************************************
 * Function Configuration
******************************************************************************/
int tmd2772_read_als(struct i2c_client *client, u16 *data)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	u16 c0_value, c1_value;
	u32 c0_nf, c1_nf;
	u8 als_value_low[1], als_value_high[1];
	u8 buffer[1],alsbuffer[4] = {0};
	u16 atio;
	//u16 als_value;
	int res = 0, i =0;
//	u8 reg_value[1];
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	/*debug tag for yucong*/
	#if 0
	buffer[0]=TMD2772_CMM_ENABLE;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, reg_value, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
	#endif
	#if 1
	buffer[0]=TMD2772_CMM_C0DATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, alsbuffer, 0x4);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	c0_value = alsbuffer[0] | (alsbuffer[1]<<8);
	c0_nf = obj->als_modulus*c0_value;
	c1_value = alsbuffer[2] | (alsbuffer[3]<<8);
	c1_nf = obj->als_modulus*c1_value;
	#else
	buffer[0]=TMD2772_CMM_C0DATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong: TMD2772_CMM_C0DATA_L = 0x%x\n", als_value_low[0]);
	buffer[0]=TMD2772_CMM_C0DATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong: TMD2772_CMM_C0DATA_H = 0x%x\n", als_value_high[0]);
	c0_value = als_value_low[0] | (als_value_high[0]<<8);
	c0_nf = obj->als_modulus*c0_value;
	//APS_DBG("c0_value=%d, c0_nf=%d, als_modulus=%d\n", c0_value, c0_nf, obj->als_modulus);
	buffer[0]=TMD2772_CMM_C1DATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong: TMD2772_CMM_C1DATA_L = 0x%x\n", als_value_low[0]);
	buffer[0]=TMD2772_CMM_C1DATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong: TMD2772_CMM_C1DATA_H = 0x%x\n", als_value_high[0]);
	c1_value = als_value_low[0] | (als_value_high[0]<<8);
	//c0_nf = obj->als_modulus*c0_value; /*ckt sw shutao.lan added 2013.01.31,calculate c0_nf again*/
	c1_nf = obj->als_modulus*c1_value;
	//APS_DBG("c1_value=%d, c1_nf=%d, als_modulus=%d\n", c1_value, c1_nf, obj->als_modulus);
	#endif
	if((c0_value > c1_value) &&(c0_value < 50000))
	{  	/*Lenovo-sw chenlj2 add 2011-06-03,add {*/
		atio = (c1_nf*100)/c0_nf;
	//APS_DBG("atio = %d\n", atio);
	if(atio<30)
	{
		*data = (13*c0_nf - 24*c1_nf)/10000;
	}
	else if(atio>= 30 && atio<38) /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
	{
		*data = (16*c0_nf - 35*c1_nf)/10000;
	}
	else if(atio>= 38 && atio<45)  /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
	{
		*data = (9*c0_nf - 17*c1_nf)/10000;
	}
	else if(atio>= 45 && atio<54) /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
	{
		*data = (6*c0_nf - 10*c1_nf)/10000;
	}
	else
		*data = 0;
	/*Lenovo-sw chenlj2 add 2011-06-03,add }*/
    }
	else if (c0_value > 50000)
	{
		*data = 65535;
	}
	else if(c0_value == 0)
        {
                *data = 0;
        }
        else
	{
		APS_DBG("als_value is invalid!!\n");
		return -1;
	}
	//APS_DBG("als_value_lux = %d\n", *data);
	/* Save the data into array */
	lux_FIFO->lux_value[lux_FIFO->wr_index++] = *data;
	if(lux_FIFO->wr_index >= lux_num)
		lux_FIFO->wr_index = 0;
	if(++lux_FIFO->num > lux_num)
		lux_FIFO->num = lux_num;
	if(1 == alsps_enable_log)
		printk("ALSPS ALS= %d\n", *data);
	return 0;


EXIT_ERR:
	APS_ERR("tmd2772_read_ps fail\n");
	return res;
}
int tmd2772_read_als_ch0(struct i2c_client *client, u16 *data)
{
//	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	u16 c0_value;
	u8 als_value_low[1], als_value_high[1];
	u8 buffer[1];
	int res = 0;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	buffer[0]=TMD2772_CMM_C0DATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	buffer[0]=TMD2772_CMM_C0DATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	c0_value = als_value_low[0] | (als_value_high[0]<<8);
	*data = c0_value;
	return 0;


EXIT_ERR:
	APS_ERR("tmd2772_read_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
static int tmd2772_get_als_value(struct tmd2772_priv *obj, u16 als)
{
	int idx,tmp_arry = 0;
	int invalid = 0;
	u16 i = 0,als_arry_low = 0,als_arry_high = 0;
	u32 sum = 0;
	/* get the average value */
	for(i = 0; i< lux_FIFO->num; i++)
	{
		sum += lux_FIFO->lux_value[i];
	}
	tmp_als_value = sum / lux_FIFO->num;
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(tmp_als_value < obj->hw->als_level[idx])
		{
			break;
		}
	}

	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n");
		idx = obj->als_value_num - 1;
	}
    if(tmp_als_value < 10) //temp_level
         temp_als_level =0;
	else if((10 <= tmp_als_value) && (tmp_als_value < 630))
		temp_als_level =1;
	else if((630 <= tmp_als_value) && (tmp_als_value < 6000))
		temp_als_level =2;
	else if((6000 <= tmp_als_value) && (tmp_als_value < 20000))
		temp_als_level =3;
    else
		temp_als_level =4;

	if(((temp_als_level - store_als_level) == 1) || ((store_als_level - temp_als_level) == 1))
	{
	   tmp_arry = (store_als_level > temp_als_level)?temp_als_level:store_als_level;
	   if((tmp_arry < 0) || (tmp_arry > 4))
            return -1;
	   if(tmp_arry == 0)
	   {
	      if((tmp_als_value >= (als_value_arry[0]-2)) &&(tmp_als_value <= (als_value_arry[0]+2)))
		  {
		     idx = store_als_idx;
			 temp_als_level = store_als_level;
	      }
		  else
		  {
		     store_als_level =  temp_als_level;
             store_als_idx   = idx;
		  }
	   }
	   else
	   {
		   als_arry_low = (u16)((als_value_arry[tmp_arry] * 95) / 100);
		   als_arry_high= (u16)((als_value_arry[tmp_arry] * 105) / 100);
		   if((tmp_als_value >= als_arry_low) && (tmp_als_value <= als_arry_high))
		   {
			   idx = store_als_idx;
			   temp_als_level = store_als_level;
		   }
		   else
		   {
	           store_als_level =  temp_als_level;
	           store_als_idx   = idx;
		   }
	   }
	}
	else
	{
	   store_als_level =  temp_als_level;
	   store_als_idx   = idx;
	}
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}

		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}
	if(!invalid)
	{
		if(1 == alsps_enable_log)
			APS_DBG("ALSPS ALS: raw data %05d => value = %05d\n", tmp_als_value, obj->hw->als_value[idx]);
		return obj->hw->als_value[idx];
	}
	else
	{
		APS_ERR("ALS: %05d => %05d (-1)\n", tmp_als_value, obj->hw->als_value[idx]);
		return -1;
	}
}
/*----------------------------------------------------------------------------*/
long tmd2772_read_ps(struct i2c_client *client, u16 *data)
{
//	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	u16 ps_value;
	u8 ps_value_low[1], ps_value_high[1];
	u8 buffer[1];
	//add zms
	u16 ps_min_value = 0;
	u16 ps_max_value = 0;
	//end
	long res = 0;
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	buffer[0]=TMD2772_CMM_PDATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, ps_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	buffer[0]=TMD2772_CMM_PDATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, ps_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	ps_value = ps_value_low[0] | (ps_value_high[0]<<8);
	if(ps_value > 1023)    //modified zms
		*data = 1023; //modified zms
	else if( ps_value < 0)
		*data = 0;
	else
	{
	    *data = ps_value;
	    tmp_data = ps_value;
	}
	if(1 == alsps_enable_log)
		APS_DBG("ALSPS PS=%d, low:%d  high:%d", *data, ps_value_low[0], ps_value_high[0]);
	return 0;
EXIT_ERR:
	APS_ERR("tmd2772_read_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
static int tmd2772_get_ps_value(struct tmd2772_priv *obj, u16 ps)
{
	//APS_FUN();
	//APS_DBG("PS raw data:  %05d => \n", ps);
	int val = 1;// mask = atomic_read(&obj->ps_mask);
	int invalid = 0;
	static volatile int val_temp=1;
	int type = -1;
	 /*Lenovo-sw chenlj2 add 2011-10-12 begin*/
	 u16 temp_ps[1];
	 /*Lenovo-sw chenlj2 add 2011-10-12 end*/


	//APS_LOG("tmd2772_get_ps_value  1 %d," ,ps_cali.close);
	//APS_LOG("tmd2772_get_ps_value  2 %d," ,ps_cali.far_away);
	//APS_LOG("tmd2772_get_ps_value  3 %d,", ps_cali.valid);
	//APS_LOG("tmd2772_get_ps_value  ps %d,", ps);
    /*Lenovo-sw zhuhc delete 2011-10-12 begin*/
	//return 1;
    /*Lenovo-sw zhuhc delete 2011-10-12 end*/
	/*ckt shutao.lan 2013.2.25,sampled twice*/
     msleep(70);
	//tmd2772_read_ps(obj->client,temp_ps);
	if(ps_cali.valid == 1)
		{
			//APS_LOG("tmd2772_get_ps_value val_temp  = %d",val_temp);
			//if((ps >ps_cali.close)&&(temp_ps[0] >ps_cali.close))
			//APS_DBG("ps_cali.close = %d ,ps_cali.far_away =%d \n",ps_cali.close,ps_cali.far_away);
			if((ps >ps_cali.close))
			{
				val = 0;  /*close*/
				val_temp = 0;
				intr_flag_value = 1;
				type = 1;
			}
			//else if((ps <ps_cali.far_away)&&(temp_ps[0] < ps_cali.far_away))
			else if((ps <ps_cali.far_away))
			{
				val = 1;  /*far away*/
				val_temp = 1;
				intr_flag_value = 0;
				type = 2;
			}
			else
				val = val_temp;
			//APS_LOG("tmd2772_get_ps_value val  = %d",val);
	}
	else
	{
			//APS_DBG("ps_thd_val_high = %d ,ps_thd_val_low =%d \n", atomic_read(&obj->ps_thd_val_high),atomic_read(&obj->ps_thd_val_low));
			//if((ps > atomic_read(&obj->ps_thd_val_high))&&(temp_ps[0]  > atomic_read(&obj->ps_thd_val_high)))
			if((ps > atomic_read(&obj->ps_thd_val_high)))
			{
				val = 0;  /*close*/
				val_temp = 0;
				intr_flag_value = 1;
				type = 3;
			}
			//else if((ps < atomic_read(&obj->ps_thd_val_low))&&(temp_ps[0]  < atomic_read(&obj->ps_thd_val_low)))
			else if((ps < atomic_read(&obj->ps_thd_val_low)))
			{
				val = 1;  /*far away*/
				val_temp = 1;
				intr_flag_value = 0;
				type = 4;
			}
			else
			       val = val_temp;
	}

	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
		type = 5;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
			type = 6;
		}
	}
	else if (obj->als > 45000)
	{
		//invalid = 1;
		APS_DBG("ligh too high will result to failt proximiy\n");
		return 1;  /*far away*/
	}
	if(!invalid)
	{
		if(1 == alsps_enable_log)
			APS_DBG("ALSPS PS:  %05d => %05d,   type = %d \n", ps, val,type);
		return val;
	}
	else
	{
		APS_DBG("wrong PS:  %05d => %05d,   type = %d \n", ps, val,type);
		return -1;
	}
}
/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
static void tmd2772_eint_work(struct work_struct *work)
{
	struct tmd2772_priv *obj = (struct tmd2772_priv *)container_of(work, struct tmd2772_priv, eint_work);
	int err;
	hwm_sensor_data sensor_data;
//	u8 buffer[1];
//	u8 reg_value[1];
	u8 databuf[2];
	int res = 0;
	APS_FUN();
	if((err = tmd2772_check_intr(obj->client)))
	{
		APS_ERR("tmd2772_eint_work check intrs: %d\n", err);
	}
	else
	{
		tmd2772_read_ps(obj->client, &obj->ps);
		//mdelay(160);
		tmd2772_read_als_ch0(obj->client, &obj->als);
		APS_DBG("tmd2772_eint_work rawdata ps=%d als_ch0=%d!\n",obj->ps,obj->als);
		//printk("tmd2772_eint_work rawdata ps=%d als_ch0=%d!\n",obj->ps,obj->als);
		sensor_data.values[0] = tmd2772_get_ps_value(obj, obj->ps);
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
/*singal interrupt function add*/
#if 1
		if(intr_flag_value){
				//printk("yucong interrupt value ps will < 750");
				databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;
				databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;
				databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;
				databuf[1] = (u8)(0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH;
				databuf[1] = (u8)((0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
		}
		else{
				//printk("yucong interrupt value ps will > 900");
				databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;
				databuf[1] = (u8)(0 & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;
				databuf[1] = (u8)((0 & 0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;
				databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH;
				databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);;
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
		}
#endif
		//let up layer to know
		if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
		{
		  APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
		}
	}
	tmd2772_clear_intr(obj->client);
	#ifdef CUST_EINT_ALS_TYPE
	mt_eint_unmask(CUST_EINT_ALS_NUM);
	#else
	mt65xx_eint_mask(CUST_EINT_ALS_NUM);
	#endif
}
/******************************************************************************
 * Function Configuration
******************************************************************************/
static int tmd2772_open(struct inode *inode, struct file *file)
{
	file->private_data = tmd2772_i2c_client;
	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int tmd2772_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
#if 0
static void tmd2772_WriteCalibration(struct PS_CALI_DATA_STRUCT *data_cali)
{
	   APS_LOG("tmd2772_WriteCalibration  1 %d," ,data_cali->close);
		   APS_LOG("tmd2772_WriteCalibration  2 %d," ,data_cali->far_away);
		   APS_LOG("tmd2772_WriteCalibration  3 %d,", data_cali->valid);

	  if(data_cali->valid == 1)
	  {
	      if(data_cali->close < 100)
          {
            ps_cali.close = 200;
			ps_cali.far_away= 150;
			ps_cali.valid = 1;
           }
		  else if(data_cali->close > 900)
		  {
            ps_cali.close = 900;
			ps_cali.far_away= 750;
			ps_cali.valid = 1;
          }
		  else
		  {
			  ps_cali.close = data_cali->close;
			ps_cali.far_away= data_cali->far_away;
			ps_cali.valid = 1;
		  }
	  }
}
#endif
#if 0
static int tmd2772_read_data_for_cali(struct i2c_client *client, struct PS_CALI_DATA_STRUCT *ps_data_cali)
{
	 struct tmd2772_priv *obj = i2c_get_clientdata(client);
     int i=0 ,err = 0,j = 0;
	 u16 data[21],sum,data_cali;
	 sum = 0;
	 for(i = 0;i<20;i++)
     {
            mdelay(5);//50
			if(err = tmd2772_read_ps(client,&data[i]))
			{
				APS_ERR("tmd2772_read_data_for_cali fail: %d\n", i);
				break;
			}
			else
				{
					sum += data[i];
			}
			mdelay(55);//160
      }

	 //for(j = 0;j<20;j++)
	 //	APS_LOG("%d\t",data[j]);

	 if(i == 20)
     {
			data_cali = sum/20;
			APS_LOG("tmd2772_read_data_for_cali data = %d",data_cali);
			if(data_cali>600)
			return -1;
			if(data_cali<=120)
			{
				ps_data_cali->close =data_cali*21/10;
				ps_data_cali->far_away = data_cali*19/10;
				ps_data_cali->valid =1;
			}
			else if(100<data_cali&&data_cali<300)
			{
				ps_data_cali->close = data_cali*2;
				ps_data_cali->far_away =data_cali*17/10;
				ps_data_cali->valid = 1;
			}
			else
			{
				ps_data_cali->close = data_cali*18/10;
				ps_data_cali->far_away =data_cali*15/10;
				ps_data_cali->valid = 1;
			}
            if(ps_data_cali->close > 900)
            {
            ps_data_cali->close = 900;
			ps_data_cali->far_away = 750;
			err= 0;
            }
			else  if(ps_data_cali->close < 100)
			{
			   ps_data_cali->close = 200;
			   ps_data_cali->far_away = 150;
			   err= 0;
            }
			ps_cali.close = ps_data_cali->close;
			ps_cali.far_away= ps_data_cali->far_away;
			ps_cali.valid = 1;
			APS_LOG("tmd2772_read_data_for_cali close  = %d,far_away = %d,valid = %d",ps_data_cali->close,ps_data_cali->far_away,ps_data_cali->valid);

       }
	 else
     {
        ps_data_cali->valid = 0;
        err=  -1;
     }
	 return err;

}
#endif
static int tmd2772_init_client_factory(struct i2c_client *client)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	int res = 0;
	databuf[0] = TMD2772_CMM_ENABLE;
	databuf[1] = 0x00;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	databuf[0] = TMD2772_CMM_ATIME;
	databuf[1] = 0xEE;//0xF6
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	databuf[0] = TMD2772_CMM_PTIME;
	databuf[1] = 0xFF;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	databuf[0] = TMD2772_CMM_WTIME;
	databuf[1] = 0xEE;//0xFC,this is suggest by FAE
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	databuf[0] = TMD2772_CMM_CONFIG;
	databuf[1] = 0x00;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
    /*Lenovo-sw chenlj2 add 2011-06-03,modified pulse 2  to 4 */
	databuf[0] = TMD2772_CMM_PPCOUNT;
	databuf[1] = TMD2772_CMM_PPCOUNT_VALUE;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
    /*Lenovo-sw chenlj2 add 2011-06-03,modified gain 16  to 1 */
	databuf[0] = TMD2772_CMM_CONTROL;
	databuf[1] = TMD2772_CMM_CONTROL_VALUE;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
     databuf[0] = TMD2772_CMM_OFFSET;
     databuf[1] = offset_data;
     res = i2c_master_send(client, databuf, 0x2);
     if(res <= 0)
     {
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
     }
	 return TMD2772_SUCCESS;
EXIT_ERR:
	APS_ERR("reinit dev: %d\n", res);
	return res;
}
//add by shutao, for store or restore the enable register
static int store_status(unsigned int *flag)
{
	u8 databuf[1];
	int res;
	if((*flag == 0)&&(0 != store_enable_register))
	{
		databuf[0] = TMD2772_CMM_ENABLE;
		databuf[1] = store_enable_register;
		res = i2c_master_send(tmd2772_i2c_client, databuf, 0x2);
		if(res <= 0)
		{
			return -1;
		}
	}
	if(*flag == 1)
	{
       databuf[0] = TMD2772_CMM_ENABLE;
       res = i2c_master_send(tmd2772_i2c_client, databuf, 0x1);
       if(res <= 0)
       {
          return -1;
       }
       res = i2c_master_recv(tmd2772_i2c_client, databuf, 0x1);
       if(res <= 0)
       {
          return -1;
       }
       store_enable_register = databuf[0];
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static long tmd2772_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	long err = 0;
	void __user *ptr = (void __user*) arg;
	unsigned int dat = 0;
	uint32_t enable;
	//struct PS_CALI_DATA_STRUCT ps_cali_temp;
	uint32_t value=0;
	unsigned int enable_flag=0;
	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
			#if 0
				if(err = tmd2772_init_client_factory(client))
				{
					APS_ERR("init factory ps fail: %ld\n", err);
					goto err_out;
				}
				offset_data = 0;
			#endif
				if((err = tmd2772_enable_ps(obj->client, 1)))
				{
					APS_ERR("enable ps fail: %ld\n", err);
					goto err_out;
				}

				set_bit(CMC_BIT_PS, &obj->enable);
			}
			else
			{
				if((err = tmd2772_enable_ps(obj->client, 0)))
				{
					APS_ERR("disable ps fail: %ld\n", err);
					goto err_out;
				}

				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;
		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
		case ALSPS_GET_PS_DATA:
			if((err = tmd2772_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}

			dat = tmd2772_get_ps_value(obj, obj->ps);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
		case ALSPS_GET_PS_RAW_DATA:
			if((err = tmd2772_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}

			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if((err = tmd2772_enable_als(obj->client, 1)))
				{
					APS_ERR("enable als fail: %ld\n", err);
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			}
			else
			{
				if((err = tmd2772_enable_als(obj->client, 0)))
				{
					APS_ERR("disable als fail: %ld\n", err);
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;
		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
		case ALSPS_GET_ALS_DATA:
			if((err = tmd2772_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}
			APS_ERR("zms ioctl %d ",obj->als);
			dat = tmd2772_get_als_value(obj, obj->als);
			APS_ERR("zms ioctrl dat = %d",dat);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
		case ALSPS_GET_ALS_RAW_DATA:
			if((err = tmd2772_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}
			APS_ERR("zms ioctrl err = %d",err);
			dat = obj->als;
			APS_ERR("zms ioctrl dat = %d",dat);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			APS_ERR("zms ioctrl ptr = %d",&ptr);
			break;
		case ALSPS_GET_PS_CALI:
            mutex_lock(&mutex);
            tmd2772_ps_calibrate(tmd2772_obj->client);
            mutex_unlock(&mutex);
			if(copy_to_user(ptr, &offset_data, sizeof(offset_data)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
		case ALSPS_RESET_PS:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
                mutex_lock(&mutex);
                offset_data = 0;
                tmd2772_ps_calibrate_call(tmd2772_obj->client);
                tmd2772_init_client(tmd2772_obj->client);
                if(err = tmd2772_enable_ps(tmd2772_obj->client, 1))
                {
                    mutex_unlock(&mutex);
                	goto err_out;
                }
                mutex_unlock(&mutex);
			}
			break;
		case ALSPS_SET_PS_CALI:
			//dat = (void __user*)arg;
			if(ptr == NULL)
			{
				APS_LOG("dat == NULL\n");
				err = -EINVAL;
				break;
			}
			if(copy_from_user(&dat, ptr, sizeof(dat)))
			{
				APS_LOG("copy_from_user\n");
				err = -EFAULT;
				break;
			}
			#if OFFDATA_DEFAULT
			   enable_flag = 1;
			   if((err = store_status(&enable_flag)))
					goto err_out;
			   offset_data = (dat & 0xff);
			   printk("ALSPS_SET_PS_CALI data:%d\n",offset_data);
			   tmd2772_init_client(client);
			   enable_flag = 0;
			   if((err = store_status(&enable_flag)))
					goto err_out;
			#endif
			break;
/*		case ALSPS_GET_PS_RAW_DATA_FOR_CALI:
			tmd2772_init_client_for_cali(obj->client);
			err = tmd2772_read_data_for_cali(obj->client,&ps_cali_temp);
			if(err)
			{
			   goto err_out;
			}
			tmd2772_init_client(obj->client);
			// tmd2772_enable_ps(obj->client, 1);
			tmd2772_enable(obj->client, 0);
			if(copy_to_user(ptr, &ps_cali_temp, sizeof(ps_cali_temp)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
*/
		case ALSPS_GET_ID:
			if(copy_to_user(ptr, &product_id, sizeof(product_id)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}
	err_out:
	return err;
}
/**
 *create the alsps attribute under sys file system
 *including:als ps config store status
 *ckt sw shutao.lan 2013.02.25,porting from TCL
 */
static ssize_t tmd2772_show_als(struct device_driver *ddri, char *buf)
{
	int res;
	u16 als_locate = 0;
	if(!tmd2772_obj)
	{
		APS_ERR("tmd2772_obj is null!!\n");
		return 0;
	}
	/*  tmd2772_read_als: we can get the intensity of light */
	if(res = tmd2772_read_als(tmd2772_obj->client, &als_locate))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
		APS_ERR("zms tmd2772_show_als = %d " ,res);
	}	
	else
	{
		return snprintf(buf, PAGE_SIZE, "%d\n%d\n%d\n", als_locate, als_sample, tmp_als_value);
	}
}
static ssize_t tmd2772_store_als(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned int enable_als = 0,enable_setting = 0;
	unsigned int err = 0;
	if(!tmd2772_obj)
	{
		APS_ERR("tmd2772_obj is null!!\n");
		return 0;
	}
	//if(3 == sscanf(buf,"%d %d %d",&en_ps,&ps_value,&alsps_log_enable))
	if(2 == sscanf(buf,"%d %d",&enable_setting, &enable_als))
	{
		printk("enable_setting:%d,enable_als:%d\n",enable_setting,enable_als);
	}
	else
	{
		printk("tmd2772_store_als is wrong!\n");
	}
	if(1 == enable_setting)
	{
		if(1 == enable_als)
		{
			set_bit(CMC_BIT_ALS, &tmd2772_obj->enable);
			if((err = tmd2772_enable_als(tmd2772_obj->client, 1)))
			{
				APS_ERR("enable als fail: %d\n", err);
				return -1;
			}
		}
		else
		{
			clear_bit(CMC_BIT_ALS, &tmd2772_obj->enable);
			if((err = tmd2772_enable_als(tmd2772_obj->client, 0)))
			{
				APS_ERR("enable als fail: %d\n", err);
				return -1;
			}

		}
	}
	return count;
}
static ssize_t tmd2772_show_ps(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!tmd2772_obj)
	{
		APS_ERR("tmd2772_obj is null!!\n");
		return 0;
	}
	msleep(60);
	if((res = tmd2772_read_ps(tmd2772_obj->client, &tmd2772_obj->ps)))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		return snprintf(buf, PAGE_SIZE, "%d\n", tmd2772_obj->ps);
	}
}
static ssize_t tmd2772_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	if(!tmd2772_obj)
	{
		APS_ERR("tmd2772_obj is null!!\n");
		return 0;
	}
	res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d %d %d %d %d %d)\n",
		atomic_read(&tmd2772_obj->i2c_retry), atomic_read(&tmd2772_obj->als_debounce),
		atomic_read(&tmd2772_obj->ps_mask), atomic_read(&tmd2772_obj->ps_debounce),
	        atomic_read(&tmd2772_obj->ps_thd_val_high),atomic_read(&tmd2772_obj->ps_thd_val_low),
	     offset_data,ps_cali.valid,ps_cali.close,ps_cali.far_away);
	return res;
}
static ssize_t tmd2772_store_ps(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned int enable_ps = 0,enable_setting = 0;
	unsigned int err = 0;
	if(!tmd2772_obj)
	{
		APS_ERR("tmd2772_obj is null!!\n");
		return 0;
	}
	//if(3 == sscanf(buf,"%d %d %d",&en_ps,&ps_value,&alsps_log_enable))
	if(5 == sscanf(buf,"%d %d %d %d %d",&en_ps,&ps_value,&alsps_enable_log,&enable_setting,&enable_ps))
	{
		printk("en_ps:%d,ps_value:%d,alsps_enable_log=%d,enable_setting:%d,enable_ps:%d\n",\
			              en_ps,ps_value,alsps_enable_log,enable_setting,enable_ps);
	}
	else
	{
		printk("tmd2772_store_ps is wrong!\n");
	}
	if(1 == enable_setting)
	{
		if(1 == enable_ps)
		{
			set_bit(CMC_BIT_PS, &tmd2772_obj->enable);
			//set_bit(CMC_BIT_ALS, &tmd2772_obj->enable);
			if((err = tmd2772_enable_ps(tmd2772_obj->client, 1)))
			{
				APS_ERR("enable ps fail: %d\n", err);
				return -1;
			}
		}
		else
		{
			clear_bit(CMC_BIT_PS, &tmd2772_obj->enable);
			//set_bit(CMC_BIT_ALS, &tmd2772_obj->enable);
			if((err = tmd2772_enable_ps(tmd2772_obj->client, 0)))
			{
				APS_ERR("enable ps fail: %d\n", err);
				return -1;
			}
		}
	}
	return count;
}
static ssize_t tmd2772_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
	int retry, als_deb, ps_deb, mask, thres, thrh, thrl,valid,ps_close,ps_far_away,setdata;
	if(!tmd2772_obj)
	{
		APS_ERR("tmd2772_obj is null!!\n");
		return 0;
	}

	if(10 == sscanf(buf, "%d %d %d %d %d %d %d %d %d %d", &retry, &als_deb, &mask, &ps_deb,&thrh,&thrl,\
						&setdata,&valid,&ps_close,&ps_far_away))
	{
		atomic_set(&tmd2772_obj->i2c_retry, retry);
		atomic_set(&tmd2772_obj->als_debounce, als_deb);
		atomic_set(&tmd2772_obj->ps_mask, mask);
	//	atomic_set(&tmd2772_obj->ps_thd_val, thres);
		atomic_set(&tmd2772_obj->ps_debounce, ps_deb);
		atomic_set(&tmd2772_obj->ps_thd_val_high, thrh);
		atomic_set(&tmd2772_obj->ps_thd_val_low, thrl);
		offset_data = setdata;
		ps_cali.valid = valid;
		ps_cali.close = ps_close;
		ps_cali.far_away = ps_far_away;
	}
	else
	{
		APS_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	return count;
}
static ssize_t tmd2772_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;

	if(!tmd2772_obj)
	{
		APS_ERR("tmd2772_obj is null!!\n");
		return 0;
	}

	if(tmd2772_obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST:i2c_num= %d\nppcount=%x\ncmm=%x\nhigh=%d\nlow=%d\n",
			tmd2772_obj->hw->i2c_num, TMD2772_CMM_PPCOUNT_VALUE,  TMD2772_CMM_CONTROL_VALUE,
			tmd2772_obj->hw->ps_threshold_high,tmd2772_obj->hw->ps_threshold_low);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "REGS: %02X %02X %02lX %02lX\n",
				atomic_read(&tmd2772_obj->als_cmd_val), atomic_read(&tmd2772_obj->ps_cmd_val),
				tmd2772_obj->enable, tmd2772_obj->pending_intr);

	len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&tmd2772_obj->als_suspend), atomic_read(&tmd2772_obj->ps_suspend));
	return len;
}
static ssize_t tmd2772_show_pscalibrate(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	mutex_lock(&mutex);
	if(!tmd2772_obj)
	{
		APS_ERR("tmd2772_obj is null!!\n");
		mutex_unlock(&mutex);
		return 0;
	}
	tmd2772_ps_calibrate(tmd2772_obj->client);
	mutex_unlock(&mutex);
	return snprintf(buf, PAGE_SIZE, "%d\n",offset_data);
}
static ssize_t tmd2772_store_pscalibrate(struct device_driver *ddri, const char *buf, size_t count)
{
	int flag = -1,err;

	if(!tmd2772_obj)
	{
		APS_ERR("tmd2772_obj is null!!\n");
		return 0;
	}

	if(1 != sscanf(buf, "%d", &flag))
	{
		APS_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	if(0 == flag)
	{
       offset_data = 0;
	   tmd2772_ps_calibrate_call(tmd2772_obj->client);
	   tmd2772_init_client(tmd2772_obj->client);
	   err = tmd2772_enable_ps(tmd2772_obj->client, 1);
    }
	else//added by jy
	{
	   offset_data = flag&0xff;
	   tmd2772_init_client(tmd2772_obj->client);
	   err = tmd2772_enable_ps(tmd2772_obj->client, 1);
	}

	return count;

}
static ssize_t tmd2772_show_product_id(struct device_driver *ddri, char *buf)
{
	int res;
	if(!tmd2772_obj)
	{
		APS_ERR("tmd2772_obj is null!!\n");
		return 0;
	}
	return snprintf(buf, PAGE_SIZE, "%x\n", product_id);
}

static ssize_t tmd2772_show_reg_config(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	const unsigned char reg[11] = {0x80,0x81,0x82,0x83,0x8c,0x8d,0x8e,0x8f,0x92,0x93,0x9e};
	unsigned char buffer[11],i,databuf;
	int res;
	if(!tmd2772_obj)
	{
		APS_ERR("tmd2772_obj is null!!\n");
		return 0;
	}
	for(i = 0; i < sizeof(reg); i++)
	{
       databuf = reg[i];
       res = i2c_master_send(tmd2772_i2c_client, &databuf, 0x1);
       if(res <= 0)
       {
          return -1;
       }
       res = i2c_master_recv(tmd2772_i2c_client, &databuf, 0x1);
       if(res <= 0)
       {
          return -1;
       }
       buffer[i] = databuf;
	}

	return snprintf(buf,PAGE_SIZE,"REGS: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
				buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],
				buffer[7],buffer[8],buffer[9],buffer[10]);
}


static DRIVER_ATTR(als,     S_IWUSR | S_IRUGO, tmd2772_show_als,   tmd2772_store_als);
static DRIVER_ATTR(ps,      S_IWUSR | S_IRUGO, tmd2772_show_ps,    tmd2772_store_ps);
static DRIVER_ATTR(config,  S_IWUSR | S_IRUGO, tmd2772_show_config,tmd2772_store_config);
static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, tmd2772_show_status,  NULL);
//static DRIVER_ATTR(pscalibrate,  0666, tmd2772_show_pscalibrate,  tmd2772_store_pscalibrate);// jy modiy for CTS
static DRIVER_ATTR(pscalibrate,  S_IWUSR | S_IRUGO, tmd2772_show_pscalibrate,  tmd2772_store_pscalibrate);//jy modify  for CTS

static DRIVER_ATTR(id,      S_IWUSR | S_IRUGO, tmd2772_show_product_id,   NULL);
static DRIVER_ATTR(reg_config,      S_IWUSR | S_IRUGO, tmd2772_show_reg_config,   NULL);
static struct driver_attribute *tmd2772_attr_list[] = {
    &driver_attr_als,
    &driver_attr_ps,
    &driver_attr_config,
    &driver_attr_status,
    &driver_attr_pscalibrate,
    &driver_attr_id,
    &driver_attr_reg_config,
};
static int tmd2772_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(tmd2772_attr_list)/sizeof(tmd2772_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}
	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, tmd2772_attr_list[idx])))
		{
			APS_ERR("driver_create_file (%s) = %d\n", tmd2772_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}
static int tmd2772_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(tmd2772_attr_list)/sizeof(tmd2772_attr_list[0]));
	if (!driver)
	return -EINVAL;
	for (idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, tmd2772_attr_list[idx]);
	}

	return err;
}
static struct file_operations tmd2772_fops = {
	.owner = THIS_MODULE,
	.open = tmd2772_open,
	.release = tmd2772_release,
	.unlocked_ioctl = tmd2772_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tmd2772_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &tmd2772_fops,
};
/*----------------------------------------------------------------------------*/
static int tmd2772_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	//struct tmd2772_priv *obj = i2c_get_clientdata(client);
	//int err;
	APS_FUN();
#if 0
	if(msg.event == PM_EVENT_SUSPEND)
	{
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return -EINVAL;
		}

		atomic_set(&obj->als_suspend, 1);
		if(err = tmd2772_enable_als(client, 0))
		{
			APS_ERR("disable als: %d\n", err);
			return err;
		}
		atomic_set(&obj->ps_suspend, 1);
		if(err = tmd2772_enable_ps(client, 0))
		{
			APS_ERR("disable ps:  %d\n", err);
			return err;
		}

		tmd2772_power(obj->hw, 0);
	}
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static int tmd2772_i2c_resume(struct i2c_client *client)
{
	//struct tmd2772_priv *obj = i2c_get_clientdata(client);
	//int err;
	APS_FUN();
#if 0
	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	tmd2772_power(obj->hw, 1);
	if(err = tmd2772_init_client(client))
	{
		APS_ERR("initialize client fail!!\n");
		return err;
	}
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if(err = tmd2772_enable_als(client, 1))
		{
			APS_ERR("enable als fail: %d\n", err);
		}
	}
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
		if(err = tmd2772_enable_ps(client, 1))
		{
			APS_ERR("enable ps fail: %d\n", err);
		}
	}
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static void tmd2772_early_suspend(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	struct tmd2772_priv *obj = container_of(h, struct tmd2772_priv, early_drv);
	int err;
	APS_FUN();
	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
	#if 0
	atomic_set(&obj->als_suspend, 1);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if((err = tmd2772_enable_als(obj->client, 0)))
		{
			APS_ERR("disable als fail: %d\n", err);
		}
	}
	#endif
}
/*----------------------------------------------------------------------------*/
static void tmd2772_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	struct tmd2772_priv *obj = container_of(h, struct tmd2772_priv, early_drv);
	int err;
	APS_FUN();
	/*  reacquire mean when screen on */
	mutex_lock(&mutex_lux);
	lux_FIFO->num = 0;
	lux_FIFO->wr_index = 0;
	mutex_unlock(&mutex_lux);
	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
        #if 0
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if((err = tmd2772_enable_als(obj->client, 1)))
		{
			APS_ERR("enable als fail: %d\n", err);
		}
	}
	#endif
}
int tmd2772_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct tmd2772_priv *obj = (struct tmd2772_priv *)self;

	//APS_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;
		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value)
				{
					#if DO_CALIBARTION
					tmd2772_ps_calibrate_call(obj->client);
					tmd2772_init_client(obj->client);
					#endif
					wake_lock(&psensor_lock);
					if((err = tmd2772_enable_ps(obj->client, 1)))
					{
						APS_ERR("enable ps fail: %d\n", err);
						return -1;
					}
					set_bit(CMC_BIT_PS, &obj->enable);
					#if 0
					if(err = tmd2772_enable_als(obj->client, 1))
					{
						APS_ERR("enable als fail: %d\n", err);
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
					#endif
				}
				else
				{
				    wake_unlock(&psensor_lock);
					if((err = tmd2772_enable_ps(obj->client, 0)))
					{
						APS_ERR("disable ps fail: %d\n", err);
						return -1;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
					#if 0
					if(err = tmd2772_enable_als(obj->client, 0))
					{
						APS_ERR("disable als fail: %d\n", err);
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
					#endif
				}
			}
			break;
		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
				if(en_ps)
				{
					tmd2772_read_ps(obj->client, &obj->ps);

                                //mdelay(160);
					tmd2772_read_als_ch0(obj->client, &obj->als);
					//APS_ERR("tmd2772_ps_operate als data=%d!\n",obj->als);
					sensor_data->values[0] = tmd2772_get_ps_value(obj, obj->ps);
				}
				else
				{
					sensor_data->values[0] = ps_value;
				}
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				//APS_LOG("tmd2772_ps_operate ps raw data=%d!, value=%d\n", obj->ps, sensor_data->values[0]);
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}
static int temp_als = 0;
int tmd2772_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0,i;
	int value;
	hwm_sensor_data* sensor_data;
	u16 tmp_als;
	u32 sum = 0;

	struct tmd2772_priv *obj = (struct tmd2772_priv *)self;
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;
		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value)
				{
					if((err = tmd2772_enable_als(obj->client, 1)))
					{
						APS_ERR("enable als fail: %d\n", err);
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
					if((err = tmd2772_enable_als(obj->client, 0)))
					{
						APS_ERR("disable als fail: %d\n", err);
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
			}
			break;
		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
				APS_ERR("zms SIZE-OUT = %d\r\n",size_out);
				APS_ERR("zms sizeof(hwm_sensor_data) = %d \r\n",sizeof(hwm_sensor_data));	
			}
					
			else
			{
				APS_ERR("zms SIZE-OUT = %d\r\n",size_out);
				APS_ERR("zms sizeof(hwm_sensor_data) = %d \r\n",sizeof(hwm_sensor_data));	
				sensor_data = (hwm_sensor_data *)buff_out;
				/*yucong MTK add for fixing know issue*/
				/* Only sampled once */
				tmd2772_read_als(obj->client, &obj->als);
				#if defined(MTK_AAL_SUPPORT)
				/* get the average value */
				/*
                		mutex_lock(&mutex_lux);
                		for(i = 0; i< lux_FIFO->num; i++)
                		{
                			sum += lux_FIFO->lux_value[i];
                		}
                		tmp_als = sum / lux_FIFO->num;
                		mutex_unlock(&mutex_lux);*/
				sensor_data->values[0] = obj->als;
				APS_ERR("zms obj->als= %d\r\n",obj->als);
				APS_ERR("zms sensor_data->values[0]= %d\r\n",sensor_data->values[0]);
				
				
				
				
				//sensor_data->values[0] = tmp_als;
				#else
				if(obj->als < 0) /* als data is 0 when low light,so we do not need temp_als */
				{
					sensor_data->values[0] = temp_als;
				}else{
					//u16 b[3];
					//int i;
					//for(i = 0;i < 3;i++){
					//msleep(30);
					//tmd2772_read_als(obj->client, &obj->als);
					//b[i] = obj->als;
					//msleep(30);
					//if(b[0] > b[i])
					//{
					//	b[0] += b[i];
					//	b[i] = b[0] - b[i];
					//	b[0] = b[0] - b[i];
					//}
					//}
					//(b[2] > b[1])?(als_sample = obj->als = b[1]):(als_sample = obj->als = b[2]);
					//

					
					sensor_data->values[0] = tmd2772_get_als_value(obj, obj->als);
					APS_ERR("zms_1 sensor_data->values[0]= %d\r\n",sensor_data->values[0]);
					temp_als = sensor_data->values[0];
					APS_ERR("zms_temp_als %d\r\n",temp_als);
				}
				#endif
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM; //SENSOR_STATUS_ACCURACY_MEDIUM
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}
int tmd2772_read_mean_call(struct i2c_client *client , int n)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	int prox_sum = 0, prox_mean = 0;
	int i, ret = 0;
	u16 prox_data[20];
//add zms
	int min_value = 0;
	int max_value = 0;
//end 
	mdelay(10);
	for(i = 0; i < n; i++)
	{
		if(ret = tmd2772_read_ps(client, &prox_data[i]))
		{
			APS_ERR("tmd2772_read_data_for_cali fail: %d\n", i);
			return ret;
		}
		prox_sum += prox_data[i];
		mdelay(10);
		//printk("%d %d \n", i, prox_data[i]);
	}
	prox_mean = prox_sum/n;
	//printk("prox_mean %d \n", prox_mean);

	return prox_mean;
}
static void tmd2772_ps_calibrate_call(struct i2c_client *client)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	int prox_mean = 0;
	int ret = 0;
	u8 buffer[2];

	tmd2772_init_client_for_cali_call(obj->client);
	prox_mean = tmd2772_read_mean_call(client, 6);//3  modified by zms
	//adde by jy if haven't calibrated in factory mode 
	/*if (prox_mean>=1023)
	{
		buffer[0] = TMD2772_CMM_OFFSET;
	       offset_data = buffer[1] = 0x00 | 0x78; 	
		ret= i2c_master_send(client, buffer, 0x2);
		if(ret<= 0)
		{
			printk("prox_mean>=1023 error \n");
		}
		msleep(5);//5ms
		prox_mean = tmd2772_read_mean_call(client, 3);
		if (prox_mean<=943)
		{
			atomic_set(&obj->ps_thd_val_high, prox_mean+ps_thd_high);
			atomic_set(&obj->ps_thd_val_low, prox_mean+ps_thd_low);
		}
	}*/
	
	//end
		if(prox_mean < 100)
		{
			atomic_set(&obj->ps_thd_val_high, 200);
			atomic_set(&obj->ps_thd_val_low,  160);
		}
		else
		{
			if(850 <=(prox_mean + ps_thd_high))
			{
				atomic_set(&obj->ps_thd_val_high, 850);
				atomic_set(&obj->ps_thd_val_low, 810);
			}
			else
			{
				atomic_set(&obj->ps_thd_val_high, prox_mean+ps_thd_high);
				atomic_set(&obj->ps_thd_val_low, prox_mean+ps_thd_low);
			}
		}

}
int tmd2772_read_mean(struct i2c_client *client , int n)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	int prox_sum = 0, prox_mean = 0;
	int i, ret = 0;
	u16 prox_data[20];
	u16 min_value = 0; //zms
	u16 max_value = 0;  //zms
	mdelay(10);
	for(i = 0; i < n; i++)
	{
		if(ret = tmd2772_read_ps(client, &prox_data[i]))
		{
			APS_ERR("tmd2772_read_data_for_cali fail: %d\n", i);
			return ret;
		}
		prox_sum += prox_data[i];
		//add zms 
		if (i==0){
			min_value=prox_data[i];
			max_value=prox_data[i];
		}
		else {
			min_value=min(min_value,prox_data[i]);
			max_value=max(max_value,prox_data[i]);		

		}
		printk("[ps]currunt i=%d,prox_data= %d \n", i, prox_data[i]);
		//remove max and min value 
		 prox_sum=prox_sum-min_value;
		prox_sum=prox_sum-max_value;
		//end zms 
		//printk("%d %d \n", i, prox_data[i]);
		mdelay(10);
	}
	prox_mean = prox_sum/(n-2); //modified by zms
	printk("zms prox_mean %d \n", prox_mean);

	return prox_mean;
}

//jy added
/*********************************************************************************/
//function: when is offset is negtive and data is <200 ,to re-calibrate the data ;if re-calibrate pass
//             offset_data will change to new value
//
//parameter:(int prox_mean,u8 offset)
//
//return value : -1  is  err, >0 is the prox_mean
//date: 2014/4/4
/*********************************************************************************/

static int  tmd2772_ps_calibrate_to_check(int prox_mean,u8 offset)
{
	u8 temp=offset;
	u8 buffer[2];
	int err=0;
	if((offset&0x80)==0)// offset is negtive
	{
		while(temp>0)
		{
			if(prox_mean<200)//the offset is too larger
			{
			   	printk("[ALS/PS]  prox_mean < 200 recheck  \n");
				temp-=10;
				buffer[0] = TMD2772_CMM_OFFSET;
				buffer[1] = 0x00 | temp;
				err= i2c_master_send(tmd2772_obj->client, buffer, 0x2);
				if(err<=0)
				{
					msleep(5);//5ms
					err= i2c_master_send(tmd2772_obj->client, buffer, 0x2);
					printk("[ALS/PS]  prox_mean < 200 recheck error \n");
					break;
				}
				msleep(5);//5ms
				prox_mean = tmd2772_read_mean(tmd2772_obj->client, 10);
			}
			else
			{
			   offset_data=temp;
			   return prox_mean;
			}
			
		}
		return -1;//err
	}
	return prox_mean;// no need to recheck
}
//end

static void tmd2772_ps_calibrate(struct i2c_client *client)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	int prox_mean = 0;
	u8 buffer[2];
	int err,ret;
	//add zms
	buffer[0] = TMD2772_CMM_OFFSET;
	buffer[1] = 0;
	err = i2c_master_send(client, buffer, 0x2);
	 if(err<= 0){
		printk("clear offset fail  error \n");
	}
	 msleep(5);//5ms 
	//end
	tmd2772_init_client_for_cali(obj->client);
	prox_mean = tmd2772_read_mean(client, 15);
	offset_data = 0;
	//printk("tmd2772_ps_calibrate prox_mean 1 %d \n", prox_mean);
	if((0 <=prox_mean)&&(prox_mean <50))//if prox_mean_clai is less than 200,plus prox_mean_clai
	{
		buffer[0] = TMD2772_CMM_OFFSET;
		offset_data = buffer[1] =0x80|0x28;//0x80 | 0x1C;  //jy modify 2014.06.14 for losting "0x80|"
		err= i2c_master_send(client, buffer, 0x2);
		if(err<= 0)
		{
			printk("prox_mean<50 error \n");
		}
		msleep(5);//5ms
		prox_mean = tmd2772_read_mean(client, 10);
	}
	else if((50 <= prox_mean)&&(prox_mean< 120))//if prox_mean_clai is less than 200,plus prox_mean_clai
	{
		buffer[0] = TMD2772_CMM_OFFSET;
		offset_data = buffer[1] = 0x80 | 0x1e; ;//0x80 | 0x15;  //jy modify
		err= i2c_master_send(client, buffer, 0x2);
		if(err<= 0)
		{
			printk("50 <prox_mean<120 error \n");
		}
		msleep(5);//5ms
		prox_mean = tmd2772_read_mean(client, 10);
	}
	else if((120 <= prox_mean)&&(prox_mean< 200))//if prox_mean_clai is less than 200,plus prox_mean_clai
	{
		buffer[0] = TMD2772_CMM_OFFSET;
		offset_data = buffer[1] =0x80 | 0x10;//0x80 | 0xB; //jy modify
		err= i2c_master_send(client, buffer, 0x2);
		if(err<= 0)
		{
			printk("120< prox_mean<200 error \n");
		}
		msleep(5);//5ms
		prox_mean = tmd2772_read_mean(client, 10);
	}
	else if((300 <= prox_mean)&&(prox_mean < 400))
	{
		buffer[0] = TMD2772_CMM_OFFSET;
		offset_data = buffer[1] = 0x00 | 0x39;//0x00 | 0xD;   //jy modify
		err= i2c_master_send(client, buffer, 0x2);
		if(err<= 0)
		{
			printk("300 < prox_mean < 400 error \n");
		}
		msleep(5);//5ms
		prox_mean = tmd2772_read_mean(client, 10);
    }
	else if((400 <= prox_mean)&&(prox_mean < 600))
	{
		buffer[0] = TMD2772_CMM_OFFSET;
		offset_data = buffer[1] = 0x00 | 0x55; //0x00 | 0x1C;   // jy modify
		err= i2c_master_send(client, buffer, 0x2);
		if(err<= 0)
		{
			printk("600 < prox_mean < 800 error \n");
		}
		msleep(5);//5ms
		prox_mean = tmd2772_read_mean(client, 10);
    }
	#if 0
	else if((500 <= prox_mean)&&(prox_mean < 600))
	{
		buffer[0] = TMD2772_CMM_OFFSET;
		offset_data = buffer[1] = 0x00 | 0x72;//0x00 | 0x2B;   //jy modify
		err= i2c_master_send(client, buffer, 0x2);
		if(err<= 0)
		{
			printk("600 < prox_mean < 800 error \n");
		}
		msleep(5);//5ms
		prox_mean = tmd2772_read_mean(client, 10);
    }
	#endif 
	else if((600 <= prox_mean)&&(prox_mean < 700))
	{
		buffer[0] = TMD2772_CMM_OFFSET;
		offset_data = buffer[1] = 0x00 | 0x64;//0x78;//0x00 | 0x39;  //jy modify
		err= i2c_master_send(client, buffer, 0x2);
		if(err<= 0)
		{
			printk("600 < prox_mean < 800 error \n");
		}
		msleep(5);//5ms
		prox_mean = tmd2772_read_mean(client, 10);
	}
	else if((700 <= prox_mean)&&(prox_mean <800))//added by jy
	{
		buffer[0] = TMD2772_CMM_OFFSET;
		offset_data = buffer[1] = 0x00 | 0x7e; 
		err= i2c_master_send(client, buffer, 0x2);
		if(err<= 0)
		{
			printk("700<prox_mean<800 error \n");
		}
		msleep(5);//5ms
		prox_mean = tmd2772_read_mean(client, 10);
		//printk("--@line=%d\n",__LINE__);
	}
	else if((800 <= prox_mean)&&(prox_mean <900))//added by jy
	{
		buffer[0] = TMD2772_CMM_OFFSET;
		offset_data = buffer[1] = 0x00 | 0x7e; 
		err= i2c_master_send(client, buffer, 0x2);
		if(err<= 0)
		{
			printk("800<prox_mean<900 error \n");
		}
		msleep(5);//5ms
		prox_mean = tmd2772_read_mean(client, 10);
		//printk("--@line=%d\n",__LINE__);
	}
	else if((900 <= prox_mean)&&(prox_mean <=1023))//added by jy
	{
		buffer[0] = TMD2772_CMM_OFFSET;
		offset_data = buffer[1] = 0x00 | 0x7e; //0x00 | 0x56;  //jy modify
		err= i2c_master_send(client, buffer, 0x2);
		if(err<= 0)
		{
			printk("900<prox_mean<1023 error \n");
		}
		msleep(5);//5ms
		prox_mean = tmd2772_read_mean(client, 10);
		//printk("--@line=%d\n",__LINE__);
	}
	else
	{
		offset_data = 0;
		//printk("--@line=%d\n",__LINE__);
	}
//--jy added start--//
       ret=0;
	ret=tmd2772_ps_calibrate_to_check(prox_mean,offset_data);
	if (ret<0)//if err to recover the value 
	{
		buffer[1] = offset_data ;
		err= i2c_master_send(client, buffer, 0x2);
		if(err<= 0)
		{
			printk("[ALS/PS] ,prox_mean(%x)<200 fix fail error ,offset_data=%x \n",prox_mean,offset_data);
		}
	}
	else
	{
		prox_mean=ret;
	}
//--jy end	--//	
	//printk("prox_mean  %d \n", prox_mean);
	if((prox_mean + ps_thd_high) >= 850)
	{
		atomic_set(&obj->ps_thd_val_high, 850);
		atomic_set(&obj->ps_thd_val_low, 810);
		printk("prox_mean>900 \n");
	}
	else
	{
		atomic_set(&obj->ps_thd_val_high, prox_mean+ps_thd_high);
		atomic_set(&obj->ps_thd_val_low, prox_mean+ps_thd_low);
	}
	//printk("--@probe set offset_data:%d,prox_mean:%d\n",offset_data,prox_mean);
}
static int tmd2772_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, TMD2772_DEV_NAME);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int tmd2772_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tmd2772_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
	int err = 0;
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	tmd2772_obj = obj;
	/* alloc struct  lux_queue */
	if(!(lux_FIFO = kzalloc(sizeof(*lux_FIFO), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(lux_FIFO, 0, sizeof(*lux_FIFO));
	mutex_init(&mutex_lux);
	lux_FIFO->num = 0;
	lux_FIFO->wr_index = 0;

	obj->hw = tmd2772_get_cust_alsps_hw();
	memcpy(obj->hw->i2c_addr, i2c_addr, C_CUST_I2C_ADDR_NUM);
	BUG_ON(sizeof(obj->hw->als_level) != sizeof(als_level));
	memcpy(obj->hw->als_level, als_level, sizeof(obj->hw->als_level));
	BUG_ON(sizeof(obj->hw->als_value) != sizeof(als_value));
	memcpy(obj->hw->als_value, als_value, sizeof(obj->hw->als_value));

	obj->hw->ps_threshold_high = ps_threshold_high;
	obj->hw->ps_threshold_low  = ps_threshold_low;
	obj->hw->ps_threshold = ps_threshold;
	tmd2772_get_addr(obj->hw, &obj->addr);
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	//INIT_DELAYED_WORK(&obj->eint_work, tmd2772_eint_work);
	if(0 == obj->hw->polling_mode_ps)
	{
		INIT_WORK(&obj->eint_work, tmd2772_eint_work);
	}
	obj->client = client;
	i2c_set_clientdata(client, obj);
	atomic_set(&obj->als_debounce, 50);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 10);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
	/*Lenovo-sw chenlj2 add 2011-06-03,modified gain 16 to 1/5 accoring to actual thing */
	obj->als_modulus = (400*100*ZOOM_TIME)/(1*150);//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
										//(400)/16*2.72 here is amplify *100 //16
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);
	mutex_init(&mutex);
	tmd2772_i2c_client = client;
//add by luosen
#if DO_CALIBARTION//Added by jrd.john for boot-up calibration
    //tmd2772_ps_calibrate(client);
#else
	tmd2772_init_client_for_cali(client);
	tmd2772_read_data_for_cali(client,&ps_cali);
#endif
//end

	if((err = tmd2772_init_client(client)))
	{
		goto exit_init_failed;
	}
	APS_LOG("tmd2772_init_client() OK!\n");
	if((err = misc_register(&tmd2772_device)))
	{
		APS_ERR("tmd2772_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	if(err = tmd2772_create_attr(&(tmd2772_init_info.platform_diver_addr->driver)))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	obj_ps.self = tmd2772_obj;
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if(1 == obj->hw->polling_mode_ps)
	//if(1)
	{
		obj_ps.polling = 1;
		wake_lock_init(&psensor_lock,WAKE_LOCK_SUSPEND,"psensor wakelock");
	}
	else
	{
		obj_ps.polling = 0;
	}
	obj_ps.sensor_operate = tmd2772_ps_operate;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	obj_als.self = tmd2772_obj;
	obj_als.polling = 1;
	obj_als.sensor_operate = tmd2772_als_operate;
	if((err = hwmsen_attach(ID_LIGHT, &obj_als)))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}
#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
	obj->early_drv.suspend  = tmd2772_early_suspend,
	obj->early_drv.resume   = tmd2772_late_resume,
	register_early_suspend(&obj->early_drv);
#endif
	APS_LOG("%s: OK\n", __func__);
	tmd2772_init_flag = 0;

	return 0;
	exit_create_attr_failed:
	misc_deregister(&tmd2772_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(client);
	//exit_kfree:
	kfree(obj);
	exit:
	tmd2772_i2c_client = NULL;
//	MT6516_EINTIRQMask(CUST_EINT_ALS_NUM);  /*mask interrupt if fail*/
	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/
static int tmd2772_i2c_remove(struct i2c_client *client)
{
	int err;
	if(err = tmd2772_delete_attr(&tmd2772_i2c_driver.driver))
	{
		APS_ERR("tmd2772_delete_attr fail: %d\n", err);
	}
	if((err = misc_deregister(&tmd2772_device)))
	{
		APS_ERR("misc_deregister fail: %d\n", err);
	}
	tmd2772_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
static int tmd2772_remove(void)
{
    struct alsps_hw *hw = tmd2772_get_cust_alsps_hw();

    tmd2772_power(hw, 0);
    i2c_del_driver(&tmd2772_i2c_driver);
    return 0;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static int  tmd2772_local_init(void)
{
    struct alsps_hw *hw = tmd2772_get_cust_alsps_hw();

	tmd2772_power(hw, 1);
	if(i2c_add_driver(&tmd2772_i2c_driver))
	{
		printk("add driver error\n");
		return -1;
	}
	if(-1 == tmd2772_init_flag)
	{
	   return -1;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int __init tmd2772_init(void)
{
	APS_FUN();
	struct alsps_hw *hw = tmd2772_get_cust_alsps_hw();
	APS_LOG("%s: mm i2c_number=%d\n", __func__,hw->i2c_num);
	i2c_register_board_info(hw->i2c_num, &i2c_TMD2772, 1);
	hwmsen_alsps_sensor_add(&tmd2772_init_info);

	

	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit tmd2772_exit(void)
{
	APS_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(tmd2772_init);
module_exit(tmd2772_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Dexiang Liu");
MODULE_DESCRIPTION("tmd2772 driver");
MODULE_LICENSE("GPL");
