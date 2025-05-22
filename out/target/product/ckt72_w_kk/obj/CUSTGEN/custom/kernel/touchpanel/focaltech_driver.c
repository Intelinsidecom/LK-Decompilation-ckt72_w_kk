#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#include <mach/mt_wdt.h>

#include <linux/slab.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>

#include "tpd.h"
#include "cust_gpio_usage.h"
#include "tpd_custom_ft5x36i.h"
#include "ft5336i_ex_fun.h"

#define TPD_MAX_PONIT       5 
 
extern struct tpd_device *tpd;
static struct i2c_client *ft5336_i2c_client = NULL;
static struct task_struct *ft5336_thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);

 #if 0
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif

static void tpd_eint_interrupt_handler(void); 
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
 
static int boot_mode = 0;
static int tpd_halt=0; 
static int tpd_flag = 0;
static int point_num = 0;
static int p_point_num = 0;
static bool discard_resume_first_eint = KAL_FALSE;
static int tpd_state = 0;
extern kal_uint32 g_charging_enable;//added by jy 
unsigned char fw_version_flag=0;//modified by jy 2014.05.08
unsigned char id_version_flag=0;//modified by jy 2014.05.08

//#define TPD_CLOSE_POWER_IN_SLEEP

#define TPD_OK 0

//register define
#define DEVICE_MODE 0x00
#define GEST_ID 0x01
#define TD_STATUS 0x02

#define TOUCH1_XH 0x03
#define TOUCH1_XL 0x04
#define TOUCH1_YH 0x05
#define TOUCH1_YL 0x06

#define TOUCH2_XH 0x09
#define TOUCH2_XL 0x0A
#define TOUCH2_YH 0x0B
#define TOUCH2_YL 0x0C

#define TOUCH3_XH 0x0F
#define TOUCH3_XL 0x10
#define TOUCH3_YH 0x11
#define TOUCH3_YL 0x12

//if need fw upgrade function define it
#define CONFIG_SUPPORT_FTS_CTP_UPG
#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
#define FTS_CTL_IIC
#ifdef FTS_CTL_IIC
#include "ft5336i_ctl.h"
#endif
#define CONFIG_AUTO_FTS_CTP_UPG
#endif

//#define ESD_CHECK //jy closed ESD check for input no reponse after touch faster for 1~2 seconds
#define TPD_RESET_ISSUE_WORKAROUND
#define TPD_MAX_RESET_COUNT 3

#ifdef ESD_CHECK
static struct delayed_work ctp_read_id_work;
static struct workqueue_struct * ctp_read_id_workqueue = NULL;
#endif
#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

#define VELOCITY_CUSTOM_FT5206
#ifdef VELOCITY_CUSTOM_FT5206
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

//for magnify velocity********************************************
#ifndef TPD_VELOCITY_CUSTOM_X
#define TPD_VELOCITY_CUSTOM_X 10
#endif
#ifndef TPD_VELOCITY_CUSTOM_Y
#define TPD_VELOCITY_CUSTOM_Y 10
#endif

#define TOUCH_IOC_MAGIC 'A'

#define TPD_GET_VELOCITY_CUSTOM_X _IO(TOUCH_IOC_MAGIC,0)
#define TPD_GET_VELOCITY_CUSTOM_Y _IO(TOUCH_IOC_MAGIC,1)

#if defined (CONFIG_SUPPORT_FTS_CTP_UPG)
#define TPD_UPGRADE_CKT _IO(TOUCH_IOC_MAGIC,2)
static unsigned char CtpFwUpgradeForIOCTRL(unsigned char* pbt_buf, unsigned int dw_lenth);
static DEFINE_MUTEX(fwupgrade_mutex);
atomic_t upgrading;

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit

#endif /* CONFIG_SUPPORT_FTS_CTP_UPG */

#ifdef CONFIG_AUTO_FTS_CTP_UPG
enum wk_wdt_type {
	WK_WDT_LOC_TYPE,
	WK_WDT_EXT_TYPE,
	WK_WDT_LOC_TYPE_NOLOCK,
	WK_WDT_EXT_TYPE_NOLOCK,
};
extern mtk_wdt_restart(enum wk_wdt_type type);

#endif

/*add by hzy for dev info*/
//#define SLT_DEVINFO_CTP
#ifdef SLT_DEVINFO_CTP
//#error "*********SLT_DEVINFO_CTP*********"
#include <linux/dev_info.h>
static struct devinfo_struct *devinfo_tp = NULL;
static char v_info[10];
#define __LCT_ADD_TP_VERSION__
#ifdef __LCT_ADD_TP_VERSION__
#define CTP_PROC_FILE "tp_info"
static struct proc_dir_entry *g_ctp_proc = NULL;
static struct proc_dir_entry *g_ctp_vender = NULL;
static int id_version=0,fw_version=0;
static unsigned char ft5x0x_read_fw_ver(void);
static unsigned char ft5x0x_read_ID_ver(void);
static int ctp_proc_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int cnt = 0;
	id_version=ft5x0x_read_ID_ver();
	cnt = sprintf(page, "pid:0x%2x\n",id_version);
	return cnt;
}
static int ctp_vender_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int cnt = 0;
	id_version=ft5x0x_read_ID_ver();
	fw_version=ft5x0x_read_fw_ver();
	//cnt = sprintf(page, "ft5436i[yeji], fw_v.major :0x%x, fw_v.minor :0x%02x\n",ctp_major,ctp_minor);
	cnt = sprintf(page, "ft5436i[yeji], id_version :0x%2x,fw_version :0x%2x\n",id_version,fw_version);
	return cnt;
}
#endif
static void tp_devinfo_init()
{
	int cnt = 0;
	id_version=ft5x0x_read_ID_ver();
	fw_version=ft5x0x_read_fw_ver();
	sprintf(v_info,"0x%x,0x%x",id_version,fw_version);
	devinfo_tp =(struct devinfo_struct*) kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);
	if(NULL != devinfo_tp)
	{
		devinfo_tp->device_type = "TP";
		devinfo_tp->device_ic = "ft5336i";
		devinfo_tp->device_used = DEVINFO_USED;
		devinfo_tp->device_module = "unknow";
		devinfo_tp->device_vendor = "Yeji";
		devinfo_tp->device_version = v_info;
		devinfo_tp->device_info = "800*480";
		devinfo_check_add_device(devinfo_tp);
	}
}
#endif
/*add by hzy end*/

int g_v_magnify_x =TPD_VELOCITY_CUSTOM_X;
int g_v_magnify_y =TPD_VELOCITY_CUSTOM_Y;


#ifdef FTS_CTL_IIC

static int esd_flag=0;

static int ft5x0x_i2c_rxdata(char *rxdata, int length);
static int ft5x0x_i2c_txdata(char *txdata, int length);

static int ft5x0x_i2c_Read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen);
static int ft5x0x_i2c_Write(struct i2c_client *client, char *writebuf, int writelen);

static int CTPDMA_i2c_write(FTS_BYTE slave,FTS_BYTE* pbt_buf, FTS_DWRD dw_len);
static int CTPDMA_i2c_read(FTS_BYTE slave, FTS_BYTE *buf, FTS_DWRD len);

static int ft_rw_iic_drv_major = FT_RW_IIC_DRV_MAJOR;
struct ft_rw_i2c_dev {
	struct cdev cdev;
	struct mutex ft_rw_i2c_mutex;
	struct i2c_client *client;
};
struct ft_rw_i2c_dev *ft_rw_i2c_dev_tt;
static struct class *fts_class;

static int ft_rw_iic_drv_myread(struct i2c_client *client, u8 *buf, int length)
{
	int ret = 0;
	int i = 0;
	
	ret = ft5x0x_i2c_Read(client, NULL, 0, buf, length);
	//ret = ft5x0x_i2c_rxdata(buf, length);

	if(ret<0)
		dev_err(&client->dev, "%s:IIC Read failed\n",
				__func__);
/*	for(i=0;i<length;i++){
		printk("hzy ft_rw_iic_drv_myread:buf[i] = %x\n",buf[i]);
	}*/
    	return ret;
}

static int ft_rw_iic_drv_mywrite(struct i2c_client *client, u8 *buf, int length)
{
	int ret = 0;
	int i = 0;
/*	for(i=0;i<length;i++){
		printk("hzy ft_rw_iic_drv_mywrite:buf[i] = %x\n",buf[i]);
	}*/
	ret = ft5x0x_i2c_Write(client, buf, length);
	//ret = ft5x0x_i2c_txdata(buf, length);
	if(ret<0)
		dev_err(&client->dev, "%s:IIC Write failed\n",
				__func__);
	return ret;
}

static int ft_rw_iic_drv_RDWR(struct i2c_client *client, unsigned long arg)
{
	struct ft_rw_i2c_queue i2c_rw_queue;
	u8 __user **data_ptrs;
	struct ft_rw_i2c * i2c_rw_msg;
	int ret = 0;
	int i;

	if (!access_ok(VERIFY_READ, (struct ft_rw_i2c_queue *)arg, sizeof(struct ft_rw_i2c_queue)))
		return -EFAULT;

	if (copy_from_user(&i2c_rw_queue,
		(struct ft_rw_i2c_queue *)arg, 
		sizeof(struct ft_rw_i2c_queue)))
		return -EFAULT;

	if (i2c_rw_queue.queuenum > FT_I2C_RDWR_MAX_QUEUE)
		return -EINVAL;


	i2c_rw_msg = (struct ft_rw_i2c*)
		kmalloc(i2c_rw_queue.queuenum *sizeof(struct ft_rw_i2c),
		GFP_KERNEL);
	if (!i2c_rw_msg)
		return -ENOMEM;

	if (copy_from_user(i2c_rw_msg, i2c_rw_queue.i2c_queue,
			i2c_rw_queue.queuenum*sizeof(struct ft_rw_i2c))) {
		kfree(i2c_rw_msg);
		return -EFAULT;
	}

	data_ptrs = kmalloc(i2c_rw_queue.queuenum * sizeof(u8 __user *), GFP_KERNEL);
	if (data_ptrs == NULL) {
		kfree(i2c_rw_msg);
		return -ENOMEM;
	}
	
	ret = 0;
	for (i=0; i< i2c_rw_queue.queuenum; i++) {
		if ((i2c_rw_msg[i].length > 8192)||
			(i2c_rw_msg[i].flag & I2C_M_RECV_LEN)) {
			ret = -EINVAL;
			break;
		}
		data_ptrs[i] = (u8 __user *)i2c_rw_msg[i].buf;
		i2c_rw_msg[i].buf = kmalloc(i2c_rw_msg[i].length, GFP_KERNEL);
		if (i2c_rw_msg[i].buf == NULL) {
			ret = -ENOMEM;
			break;
		}

		if (copy_from_user(i2c_rw_msg[i].buf, data_ptrs[i], i2c_rw_msg[i].length)) {
			++i;
			ret = -EFAULT;
			break;
		}
	}

	if (ret < 0) {
		int j;
		for (j=0; j<i; ++j)
			kfree(i2c_rw_msg[j].buf);
		kfree(data_ptrs);
		kfree(i2c_rw_msg);
		return ret;
	}

	for (i=0; i< i2c_rw_queue.queuenum; i++) 
	{
		if (i2c_rw_msg[i].flag) {
			printk("[FTS] readlen=%d\n",i2c_rw_msg[i].length);
   	   		ret = ft_rw_iic_drv_myread(client,i2c_rw_msg[i].buf, i2c_rw_msg[i].length);
   	   		//ret = CTPDMA_i2c_read(0x70,i2c_rw_msg[i].buf, i2c_rw_msg[i].length);
			if(ret<0)
			{
				printk("iic read error\n");
			}
			if (ret>=0)
   	   			ret = copy_to_user(data_ptrs[i], i2c_rw_msg[i].buf, i2c_rw_msg[i].length);
   	   	}
		else
			printk("[FTS] writelen=%d\n",i2c_rw_msg[i].length);
			ret = ft_rw_iic_drv_mywrite(client,i2c_rw_msg[i].buf, i2c_rw_msg[i].length);
			//ret = CTPDMA_i2c_write(0x70,i2c_rw_msg[i].buf, i2c_rw_msg[i].length);
			if(ret<0)
			{
				printk("iic write error\n");
			}
	}

	return ret;
	
}

/*
*char device open function interface int ft5x0x_i2c_rxdata(char *rxdata, int length)
*/
static int ft_rw_iic_drv_open(struct inode *inode, struct file *filp)
{
	filp->private_data = ft_rw_i2c_dev_tt;
	esd_flag=1;
	return 0;
}

/*
*char device close function interface 
*/
static int ft_rw_iic_drv_release(struct inode *inode, struct file *filp)
{
	esd_flag=0;
	return 0;
}

static int ft_rw_iic_drv_ioctl(struct file *filp, unsigned
  int cmd, unsigned long arg)
{
	int ret = 0;
	struct ft_rw_i2c_dev *ftdev = filp->private_data;
	ftdev = filp->private_data;

	ftdev->client=ft5336_i2c_client;

	printk("jy befor RDWR 0x%x",ftdev->client->addr);
	
	mutex_lock(&ft_rw_i2c_dev_tt->ft_rw_i2c_mutex);
	switch (cmd)
	{
	case FT_I2C_RW:
		ret = ft_rw_iic_drv_RDWR(ftdev->client, arg);	
		break; 
	default:
		ret =  -ENOTTY;
		break;
	}
	mutex_unlock(&ft_rw_i2c_dev_tt->ft_rw_i2c_mutex);
	return ret;	
}


/*    
*char device file operation which will be put to register the char device
*/
static const struct file_operations ft_rw_iic_drv_fops = {
	.owner			= THIS_MODULE,
	.open			= ft_rw_iic_drv_open,
	.release			= ft_rw_iic_drv_release,
	.unlocked_ioctl	= ft_rw_iic_drv_ioctl,
};


static void ft_rw_iic_drv_setup_cdev(struct ft_rw_i2c_dev *dev, int index)
{
	int err, devno = MKDEV(ft_rw_iic_drv_major, index);

	cdev_init(&dev->cdev, &ft_rw_iic_drv_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &ft_rw_iic_drv_fops;
	err = cdev_add(&dev->cdev, devno, 1);
	if (err)
		printk(KERN_NOTICE "Error %d adding LED%d", err, index);
}

static int ft_rw_iic_drv_myinitdev(struct i2c_client *client)
{
	int err = 0;
	dev_t devno = MKDEV(ft_rw_iic_drv_major, 0);

	if (ft_rw_iic_drv_major)
		err = register_chrdev_region(devno, 1, FT_RW_IIC_DRV);
	else {
		err = alloc_chrdev_region(&devno, 0, 1, FT_RW_IIC_DRV);
		ft_rw_iic_drv_major = MAJOR(devno);
	}
	if (err < 0) {
		dev_err(&client->dev, "%s:ft_rw_iic_drv failed  error code=%d---\n",
				__func__, err);
		return err;
	}

	ft_rw_i2c_dev_tt = kmalloc(sizeof(struct ft_rw_i2c_dev), GFP_KERNEL);
	if (!ft_rw_i2c_dev_tt){
		err = -ENOMEM;
		unregister_chrdev_region(devno, 1);
		dev_err(&client->dev, "%s:ft_rw_iic_drv failed\n",
				__func__);
		return err;
	}
	ft_rw_i2c_dev_tt->client = client;
	//init_MUTEX(&ft_rw_i2c_dev_tt->ft_rw_i2c_sem);
	mutex_init(&ft_rw_i2c_dev_tt->ft_rw_i2c_mutex);
	ft_rw_iic_drv_setup_cdev(ft_rw_i2c_dev_tt, 0); 

	fts_class = class_create(THIS_MODULE, "fts_class");
	if (IS_ERR(fts_class)) {
		dev_err(&client->dev, "%s:failed in creating class.\n",
				__func__);
		return -1; 
	} 
	/*create device node*/
	device_create(fts_class, NULL, MKDEV(ft_rw_iic_drv_major, 0), 
			NULL, FT_RW_IIC_DRV);

	return 0;
}

int ft_rw_iic_drv_init(struct i2c_client *client)
{
    	dev_dbg(&client->dev, "[FTS]----ft_rw_iic_drv init ---\n");
	return ft_rw_iic_drv_myinitdev(client);
}

void  ft_rw_iic_drv_exit(void)
{
	device_destroy(fts_class, MKDEV(ft_rw_iic_drv_major, 0)); 
	/*delete class created by us*/
	class_destroy(fts_class); 
	/*delet the cdev*/
	cdev_del(&ft_rw_i2c_dev_tt->cdev);
	kfree(ft_rw_i2c_dev_tt);
	unregister_chrdev_region(MKDEV(ft_rw_iic_drv_major, 0), 1); 
}

#endif

#endif/*end for VELOCITY_CUSTOM_FT5206*/

// ***jy added for upgrade using argument start***//

static int tpd_misc_open(struct inode *inode, struct file *file)
{

	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int tpd_misc_release(struct inode *inode, struct file *file)
{
	
	return 0;
}
/*----------------------------------------------------------------------------*/

static long tpd_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	
	void __user *data;
	
	long err = 0;
	int size=0;
	char * ctpdata=NULL;

	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		printk("tpd: access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case TPD_GET_VELOCITY_CUSTOM_X:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_x, sizeof(g_v_magnify_x)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

	   case TPD_GET_VELOCITY_CUSTOM_Y:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_y, sizeof(g_v_magnify_y)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

#if defined (CONFIG_SUPPORT_FTS_CTP_UPG)  // jy
		case TPD_UPGRADE_CKT:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}	
			if(copy_from_user(&size, data, sizeof(int)))
			{
				err = -EFAULT;
				break;	  
			}
			ctpdata=kmalloc(size, GFP_KERNEL);
			if(ctpdata==NULL)
			{
				err = -EFAULT;
				break;
			}
			
			if(copy_from_user(ctpdata, data+sizeof(int), size))
			{
				kfree(ctpdata);
				err = -EFAULT;
				break;	  
			}
			err=CtpFwUpgradeForIOCTRL(ctpdata, size);
 			kfree(ctpdata);
			break;
#endif
		default:
			printk("tpd: unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}


static struct file_operations tpd_fops = {
//	.owner = THIS_MODULE,
	.open = tpd_misc_open,
	.release = tpd_misc_release,
	.unlocked_ioctl = tpd_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tpd_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ft5336",
	.fops = &tpd_fops,
};

//**********************************************


//***jy end***//

struct touch_info {
	int y[5];
	int x[5];
	int p[5];
	int id[5];
	int count;
};

//unsigned short force[] = {0,0x70,I2C_CLIENT_END,I2C_CLIENT_END}; 
//static const unsigned short * const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces, };
static const struct i2c_device_id ft5336_tpd_id[] = {{"ft5336",0},{}};
static struct i2c_board_info __initdata ft5336_i2c_tpd={ I2C_BOARD_INFO("ft5336", (0x70>>1))};

static struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.name = "ft5336",//.name = TPD_DEVICE,
		//.owner = THIS_MODULE,
	},
	.probe = tpd_probe,
	.remove = __devexit_p(tpd_remove),
	.id_table = ft5336_tpd_id,
	.detect = tpd_detect,
	//.address_data = &addr_data,
};

//hzy+ begin
static int ft5x0x_read_reg(u8 addr, u8 *pdata)
{
	int ret;
	u8 buf[2] = {0};

	buf[0] = addr;
	struct i2c_msg msgs[] = {
		{
			.addr	= ft5336_i2c_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= ft5336_i2c_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= buf,
		},
	};

	//msleep(1);
	ret = i2c_transfer(ft5336_i2c_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	*pdata = buf[0];
	return ret;
  
}
static unsigned char ft5x0x_read_fw_ver(void)
{
	unsigned char ver;
	ft5x0x_read_reg(0xa6, &ver);

	return(ver);
}
static unsigned char ft5x0x_read_ID_ver(void)
{
	unsigned char ver;
	ft5x0x_read_reg(0xa8, &ver);

	return(ver);
}
//hzy+ end

#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
static u8 *CTPI2CDMABuf_va = NULL;
static u32 CTPI2CDMABuf_pa = NULL;
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



#define FTS_NULL                0x0
#define FTS_TRUE                0x01
#define FTS_FALSE               0x0

#define I2C_CTPM_ADDRESS       	0x70


static int ft5x0x_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}
/*write data by i2c*/
static int ft5x0x_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);

	return ret;
}


/***********************************************************************************************
Name	:	ft5x0x_i2c_rxdata 

Input	:	*rxdata
                     *length

Output	:	ret

function	:	
***********************************************************************************************/
static int ft5x0x_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= ft5336_i2c_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= ft5336_i2c_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

	//msleep(1);
	ret = i2c_transfer(ft5336_i2c_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	
	return ret;
}
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static int ft5x0x_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= ft5336_i2c_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

   	//msleep(1);
	ret = i2c_transfer(ft5336_i2c_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}
/***********************************************************************************************
Name	 :	 ft5x0x_write_reg
Input	 :	addr -- address
                para -- parameter
Output	 :	
function :	write register of ft5x0x
***********************************************************************************************/
static int ft5x0x_write_reg(u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret = ft5x0x_i2c_txdata(buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}
	
	return 0;
}

void delay_qt_ms(unsigned long  w_ms)
{
	unsigned long i;
	unsigned long j;

	for (i = 0; i < w_ms; i++)
	{
        	for (j = 0; j < 1000; j++)
        	{
			udelay(1);
        	}
	}
}


/*
[function]  : callback: read data from ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[out]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]    :
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
	int ret;
    
	ret=i2c_master_recv(ft5336_i2c_client, pbt_buf, dw_lenth);

	if(ret<=0)
	{
        	printk("[TSP]i2c_read_interface error\n");
        	return FTS_FALSE;
	}
  
    return FTS_TRUE;
}

/*
[function]: 
    callback: write data to ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[in]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
	int ret;
	ret=i2c_master_send(ft5336_i2c_client, pbt_buf, dw_lenth);
	if(ret<=0)
	{
        	printk("[TSP]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
        	return FTS_FALSE;
	}

	return FTS_TRUE;
}

/*
[function]: 
    send a command to ctpm.
[parameters]:
    btcmd[in]        :command code;
    btPara1[in]    :parameter 1;    
    btPara2[in]    :parameter 2;    
    btPara3[in]    :parameter 3;    
    num[in]        :the valid input parameter numbers, if only command code needed and no parameters followed,then the num is 1;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL cmd_write(FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
{
	FTS_BYTE write_cmd[4] = {0};

	write_cmd[0] = btcmd;
	write_cmd[1] = btPara1;
	write_cmd[2] = btPara2;
	write_cmd[3] = btPara3;
	return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);
}

/*
[function]: 
    write data to ctpm , the destination address is 0.
[parameters]:
    pbt_buf[in]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{   
	return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}


static int CTPDMA_i2c_write(FTS_BYTE slave,FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{
    
	int i = 0;
	int err = 0;
	for(i = 0 ; i < dw_len; i++)
	{
		CTPI2CDMABuf_va[i] = pbt_buf[i];
	}

	if(dw_len <= 8)
	{
		//i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
		//MSE_ERR("Sensor non-dma write timing is %x!\r\n", this_client->timing);
		return i2c_master_send(ft5336_i2c_client, pbt_buf, dw_len);
	}
	else
	{
		ft5336_i2c_client->addr = ft5336_i2c_client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		//MSE_ERR("Sensor dma timing is %x!\r\n", this_client->timing);
		err= i2c_master_send(ft5336_i2c_client, CTPI2CDMABuf_pa, dw_len);
		ft5336_i2c_client->addr = ft5336_i2c_client->addr & I2C_MASK_FLAG;
		return err;
	}    
}

static int CTPDMA_i2c_read(FTS_BYTE slave, FTS_BYTE *buf, FTS_DWRD len)
{
	int i = 0, err = 0;

	if(len < 8)
	{
		ft5336_i2c_client->addr = ft5336_i2c_client->addr & I2C_MASK_FLAG;
		//MSE_ERR("Sensor non-dma read timing is %x!\r\n", this_client->timing);
		return i2c_master_recv(ft5336_i2c_client, buf, len);
	}
	else
	{
		ft5336_i2c_client->addr = ft5336_i2c_client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		//MSE_ERR("Sensor dma read timing is %x!\r\n", this_client->timing);
		err = i2c_master_recv(ft5336_i2c_client, CTPI2CDMABuf_pa, len);
		ft5336_i2c_client->addr = ft5336_i2c_client->addr & I2C_MASK_FLAG;
		if(err < 0)
		{
			return err;
		}

		for(i = 0; i < len; i++)
		{
			buf[i] = CTPI2CDMABuf_va[i];
		}
		return err;
	}
}

/*
[function]: 
    read out data from ctpm,the destination address is 0.
[parameters]:
    pbt_buf[out]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
	return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}

/*
[function]: 
    burn the FW to ctpm.
[parameters]:(ref. SPEC)
    pbt_buf[in]    :point to Head+FW ;
    dw_lenth[in]:the length of the FW + 6(the Head length);    
    bt_ecc[in]    :the ECC of the FW
[return]:
    ERR_OK        :no error;
    ERR_MODE    :fail to switch to UPDATE mode;
    ERR_READID    :read id fail;
    ERR_ERASE    :erase chip fail;
    ERR_STATUS    :status error;
    ERR_ECC        :ecc error.
*/

//#define    FTS_PACKET_LENGTH        2
#define    FTS_PACKET_LENGTH        128
//---start added by jy to compatible with Truly and BaoMing TP 2014.05.08

static unsigned char CTPM_FW_TOP[]=
{
	#include "FT5336_TOP.i"
};

static unsigned char CTPM_FW_HRC[]=
{
	//#include "FT5336_HRC.i"
	#include "FT5336_CKT_Doro820-mini_HRC0x43_Ver0x11_20140924_app.i"
};

//----end

unsigned char *CTPM_FW=NULL;//modified by jy 2014.05.08
unsigned short g_length = 0;

#define IC_FT5X06	0
#define IC_FT5606	1
#define IC_FT5316	2
#define IC_FT5X36	3
#define DEVICE_IC_TYPE	IC_FT5X36

#define    BL_VERSION_LZ4        0
#define    BL_VERSION_Z7         1
#define    BL_VERSION_GZF        2

#define FTS_UPGRADE_LOOP	3

struct Upgrade_Info{
	u16		delay_aa;		/*delay of write FT_UPGRADE_AA*/
	u16		delay_55;		/*delay of write FT_UPGRADE_55*/
	u8		upgrade_id_1;	/*upgrade id 1*/
	u8		upgrade_id_2;	/*upgrade id 2*/
	u16		delay_readid;	/*delay of read id*/
};


static void fts_get_upgrade_info(struct Upgrade_Info * upgrade_info)
{
	switch(DEVICE_IC_TYPE)
	{
	case IC_FT5X06:
		upgrade_info->delay_55 = FT5X06_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5X06_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5X06_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5X06_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5X06_UPGRADE_READID_DELAY;
		break;
	case IC_FT5606:
		upgrade_info->delay_55 = FT5606_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5606_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5606_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5606_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5606_UPGRADE_READID_DELAY;
		break;
	case IC_FT5316:
		upgrade_info->delay_55 = FT5316_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5316_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5316_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5316_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5316_UPGRADE_READID_DELAY;
		break;
	case IC_FT5X36:
		upgrade_info->delay_55 = FT5X36_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5X36_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5X36_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5X36_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5X36_UPGRADE_READID_DELAY;
		break;
	default:
		break;
	}
}

static int  fts_ctpm_fw_upgrade(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth)
{
	int ret=0;
	u8 reg_val[2] = {0};
	u32 i = 0;
	u8 is_5336_new_bootloader = 0;
	u8 is_5336_fwsize_30 = 0;
	u32  packet_number;
	u32  j;
	u32  temp;
	u32  lenght;
	u8 	packet_buf[FTS_PACKET_LENGTH + 6];
	u8  	auc_i2c_write_buf[10];
	u8  	bt_ecc;
	int	i_ret;
	unsigned short  fw_filenth=dw_lenth;

	
#ifdef ESD_CHECK	
 	cancel_delayed_work_sync(&ctp_read_id_work);
#endif
	//printk(KERN_WARNING "fw_filenth %d \n",fw_filenth);
	struct Upgrade_Info upgradeinfo;

	fts_get_upgrade_info(&upgradeinfo);

	if(pbt_buf[fw_filenth-12] == 30)
	//jy modified 2014.05.09
	//if((*(CTPM_FW+(fw_filenth-12)))==3)//jy modified 2014.05.09
	{
		is_5336_fwsize_30 = 1;
	}
	else 
	{
		is_5336_fwsize_30 = 0;
	}

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
    	/*********Step 1:Reset  CTPM *****/
    	/*write 0xaa to register 0xfc*/
	   	//ft5x0x_write_reg(client, 0xfc, FT_UPGRADE_AA);
		//msleep(upgradeinfo.delay_aa);
		
		 /*write 0x55 to register 0xfc*/
		//ft5x0x_write_reg(client, 0xfc, FT_UPGRADE_55);   
		//msleep(upgradeinfo.delay_55);   

		/*write 0xaa to register 0xfc*/
	   	ft5x0x_write_reg(0xfc, FT_UPGRADE_AA);
		msleep(upgradeinfo.delay_aa);
		
		 /*write 0x55 to register 0xfc*/
		ft5x0x_write_reg(0xfc, FT_UPGRADE_55);   
		msleep(upgradeinfo.delay_55);   


		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		auc_i2c_write_buf[1] = FT_UPGRADE_AA;
		
	    i_ret = ft5x0x_i2c_Write(client, auc_i2c_write_buf, 2);
	  
	    /*********Step 3:check READ-ID***********************/   
		msleep(upgradeinfo.delay_readid);
	   	auc_i2c_write_buf[0] = 0x90; 
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;

		ft5x0x_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
		
		if (reg_val[0] == upgradeinfo.upgrade_id_1 
			&& reg_val[1] == upgradeinfo.upgrade_id_2)
		{
			//printk(KERN_WARNING "[FTS] Step 3: CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
			dev_dbg(&client->dev, "[FTS] Step 3: CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
	    		break;
		}
		else
		{
			dev_err(&client->dev, "[FTS] Step 3: CTPM ID FAILD,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
	    		continue;
		}
	}
	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;
		
	auc_i2c_write_buf[0] = 0xcd;
	ft5x0x_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);

	printk("[TSP]reg_val[0] =%d\n ",reg_val[0] );

	/*********0705 mshl ********************/
	/*if (reg_val[0] > 4)
		is_5336_new_bootloader = 1;*/

	if (reg_val[0] <= 4)
	{
		is_5336_new_bootloader = BL_VERSION_LZ4 ;
	}
	else if(reg_val[0] == 7)
	{
		is_5336_new_bootloader = BL_VERSION_Z7 ;
	}
	else if(reg_val[0] >= 0x0f)
	{
		is_5336_new_bootloader = BL_VERSION_GZF ;
	}

     /*********Step 4:erase app and panel paramenter area ********************/
	printk("[TSP] step 4 start\n");
	if(is_5336_fwsize_30)
	{
		auc_i2c_write_buf[0] = 0x61;
		ft5x0x_i2c_Write(client, auc_i2c_write_buf, 1); /*erase app area*/	
   		 msleep(FT_UPGRADE_EARSE_DELAY); 

		 auc_i2c_write_buf[0] = 0x63;
		ft5x0x_i2c_Write(client, auc_i2c_write_buf, 1); /*erase app area*/	
   		 msleep(50);
	}
	else
	{
		auc_i2c_write_buf[0] = 0x61;
		ft5x0x_i2c_Write(client, auc_i2c_write_buf, 1); /*erase app area*/	
   		msleep(FT_UPGRADE_EARSE_DELAY); 
	}
	printk("[TSP] step 4 end\n");

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;

	if(is_5336_new_bootloader == BL_VERSION_LZ4 || is_5336_new_bootloader == BL_VERSION_Z7 )
	{
		dw_lenth = dw_lenth - 8;
	}
	else if(is_5336_new_bootloader == BL_VERSION_GZF) dw_lenth = dw_lenth - 14;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	for (j=0;j<packet_number;j++)
	{
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8)(temp>>8);
		packet_buf[3] = (u8)temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8)(lenght>>8);
		packet_buf[5] = (u8)lenght;

		for (i=0;i<FTS_PACKET_LENGTH;i++)
		{
		    packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
		    bt_ecc ^= packet_buf[6+i];
		}

		//ft5x0x_i2c_Write(client, packet_buf, FTS_PACKET_LENGTH+6);
		ret=CTPDMA_i2c_write(0x70, &packet_buf[0],FTS_PACKET_LENGTH + 6);
		if(ret <0)
		{
			printk("<jy> <%d>,%s(),ret=%d\n",__LINE__,__func__,ret );
		}
		msleep(FTS_PACKET_LENGTH/6 + 1);
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
	{
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8)(temp>>8);
		packet_buf[3] = (u8)temp;

		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8)(temp>>8);
		packet_buf[5] = (u8)temp;

		for (i=0;i<temp;i++)
		{
		    packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
		    bt_ecc ^= packet_buf[6+i];
		}
  
		//ft5x0x_i2c_Write(client, packet_buf, temp+6);
		ret = CTPDMA_i2c_write(0x70, &packet_buf[0],temp+6);
		if(ret <0)
		{
			printk("<jy> <%d>,%s(),ret=%d\n",__LINE__,__func__,ret );
		}
		msleep(20);
	}

	/*send the last six byte*/
	if(is_5336_new_bootloader == BL_VERSION_LZ4 || is_5336_new_bootloader == BL_VERSION_Z7 )
	{
		for (i = 0; i<6; i++)
		{
			if (is_5336_new_bootloader  == BL_VERSION_Z7 && DEVICE_IC_TYPE==IC_FT5X36) 
			{
				temp = 0x7bfa + i;
			}
			else if(is_5336_new_bootloader == BL_VERSION_LZ4)
			{
				temp = 0x6ffa + i;
			}
			packet_buf[2] = (u8)(temp>>8);
			packet_buf[3] = (u8)temp;
			temp =1;
			packet_buf[4] = (u8)(temp>>8);
			packet_buf[5] = (u8)temp;
			packet_buf[6] = pbt_buf[ dw_lenth + i]; 
			bt_ecc ^= packet_buf[6];
  
			ft5x0x_i2c_Write(client, packet_buf, 7);
			msleep(10);
		}
	}
	else if(is_5336_new_bootloader == BL_VERSION_GZF)
	{
		for (i = 0; i<12; i++)
		{
			if (is_5336_fwsize_30 && DEVICE_IC_TYPE==IC_FT5X36) 
			{
				temp = 0x7ff4 + i;
			}
			else if (DEVICE_IC_TYPE==IC_FT5X36) 
			{
				temp = 0x7bf4 + i;
			}
			packet_buf[2] = (u8)(temp>>8);
			packet_buf[3] = (u8)temp;
			temp =1;
			packet_buf[4] = (u8)(temp>>8);
			packet_buf[5] = (u8)temp;
			packet_buf[6] = pbt_buf[ dw_lenth + i]; 
			bt_ecc ^= packet_buf[6];
  
			//ft5x0x_i2c_Write(client, packet_buf, 7);
			ret=CTPDMA_i2c_write(0x70, &packet_buf[0],7);
			if(ret <0)
			{
				printk("<jy> <%d>,%s(),ret=%d\n",__LINE__,__func__,ret );
			}
			msleep(10);

		}
	}

	/*********Step 6: read out checksum***********************/
	/*send the opration head*/
	auc_i2c_write_buf[0] = 0xcc;
	ft5x0x_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1); 

	if(reg_val[0] != bt_ecc)
	{
		dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n", reg_val[0], bt_ecc);
	    	return -EIO;
	}

	/*********Step 7: reset the new FW***********************/
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(1);  
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	
       printk("<jy> <%d>,%s(),update OK-----------OK\n",__LINE__,__func__);
	return 0;
}


static int fts_ctpm_fw_upgrade_with_i_file(struct i2c_client *client)
{
	FTS_BYTE*     pbt_buf = FTS_NULL;
	int i_ret;
	unsigned char version=0;
	FTS_BYTE flag;
	FTS_DWRD i = 0;
	//=========FW upgrade========================*/
	pbt_buf = CTPM_FW;
	version = fw_version_flag;
	unsigned short fw_filenth=g_length;
	//printk("version=%x ,pbt_buf[sizeof(CTPM_FW)-2]=%d\n",version,pbt_buf[sizeof(CTPM_FW)-2]);
	printk("version=%x ,pbt_buf[sizeof(CTPM_FW)-2]=%d\n",version,pbt_buf[fw_filenth-2]);
	printk("[TSP]ID_ver=%x, fw_ver=%x\n", ft5x0x_read_ID_ver(), ft5x0x_read_fw_ver());
/*
	if(0xa8 != ft5x0x_read_ID_ver())
	{
		if(ft5x0x_read_ID_ver() != pbt_buf[sizeof(CTPM_FW)-1])
		{
	        	return;
		}
		
		do
		{
			i ++;
	        	version =ft5x0x_read_fw_ver();
	        	delay_qt_ms(2);
		}while( i < 5 );
	    
		if(version==pbt_buf[sizeof(CTPM_FW)-2])
		{
			return;
		}
	}
*/
	/*call the upgrade function*/

	printk("[TSP] CTPM_FW=%d\n",fw_filenth);
	
	i_ret =  fts_ctpm_fw_upgrade(client,pbt_buf,fw_filenth);//jy modify 2014.05.09
	if (i_ret != 0)
	{
		printk("[TSP]upgrade error\n");
		//error handling ...
		//TBD
	}
	msleep(200);  
	ft5x0x_write_reg(0xfc,0x04);
	msleep(4000);
	flag=0;
	i2c_smbus_read_i2c_block_data(ft5336_i2c_client, 0xFC, 1, &flag);
	//printk("flag=%d\n",flag);
	return i_ret;
}

unsigned char fts_ctpm_get_upg_ver(void)
{
	unsigned int ui_sz;
	ui_sz = sizeof(CTPM_FW_HRC);
	if (ui_sz > 2)
	{
        	return CTPM_FW[ui_sz - 2];
        	//return (*(CTPM_FW+(ui_sz - 2)));//jy modify 2014.05.09
	}
	else
	{
        	//TBD, error handling?
        	return 0xff; //default value
	}
}

static void tpd_resume( struct early_suspend *h );
static unsigned char CtpFwUpgradeForIOCTRL(unsigned char* pbt_buf, unsigned int dw_lenth)
{
	int ret=0;
	
	tpd_resume((struct early_suspend *)0); 
	ret=fts_ctpm_fw_upgrade(ft5336_i2c_client,pbt_buf,dw_lenth);

	msleep(200);  
	ft5x0x_write_reg(0xfc,0x04);
	msleep(4000);
   	return ret;
}
#endif

#ifdef ESD_CHECK	
static void ESD_read_id_workqueue(struct work_struct *work)
{
	char data;
	if(esd_flag)
	{
		return;
	}
	if(tpd_halt) 
		return;

	i2c_smbus_read_i2c_block_data(ft5336_i2c_client, 0x88, 1, &data);
	//TPD_DEBUG("ESD_read_id_workqueue data: %d\n", data);
	printk("TPD ESD_read_id_workqueue data: %d\n", data);
	if((data > 5)&&(data < 10))
	{
		//add_timer();
	}
	else
	{

		mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
		if(tpd_state)
		{
			input_mt_sync(tpd->dev);
	                input_sync(tpd->dev);
			tpd_state = 0;
		}
		msleep(5);  
	
//#ifdef MT6575
//power on, need confirm with SA
#ifdef TPD_POWER_SOURCE_CUSTOM
  //  		hwPowerDown(TPD_POWER_SOURCE_CUSTOM,  "TP");
#else
    		hwPowerDown(MT65XX_POWER_LDO_VGP2,  "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
    		hwPowerDown(TPD_POWER_SOURCE_1800,  "TP");
#endif    
		msleep(5);  
#ifdef TPD_POWER_SOURCE_CUSTOM
    		//hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#else
    		hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
    		hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif    
//#endif	
		msleep(100);
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
		msleep(10);  
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		 
		msleep(200);
	}

	if(tpd_halt) 
		mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM); 
	else 
		queue_delayed_work(ctp_read_id_workqueue, &ctp_read_id_work,400); //schedule a work for the first detection					

}
#endif
static  void tpd_down(int x, int y, int p) {
	// input_report_abs(tpd->dev, ABS_PRESSURE, p);
	if (RECOVERY_BOOT == get_boot_mode())
	{
	}
	else
	{
		input_report_key(tpd->dev, BTN_TOUCH, 1);
	}
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 20);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	/* Lenovo-sw yexm1, optimize the code, 2012-9-19 begin */
	// printk("D[%4d %4d %4d] ", x, y, p);
	/* Lenovo-sw yexm1, optimize the code, 2012-9-19 end */
	/* track id Start 0 */
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, p); 
	input_mt_sync(tpd->dev);
	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	{   
		tpd_button(x, y, 1);  
	}
	if(y > TPD_RES_Y) //virtual key debounce to avoid android ANR issue
	{
		/* Lenovo-sw yexm1 modify, 2012-10-15, delete the delay */
		//msleep(50);
		printk("D virtual key \n");
	}
	TPD_EM_PRINT(x, y, x, y, p-1, 1);
 }
 
static void tpd_up(int x, int y,int *count) {
	 //if(*count>0) {
		 //input_report_abs(tpd->dev, ABS_PRESSURE, 0);
		 input_report_key(tpd->dev, BTN_TOUCH, 0);
		 //input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
		 //input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
		 //input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
		 //printk("U[%4d %4d %4d] ", x, y, 0);
		 input_mt_sync(tpd->dev);
		 TPD_EM_PRINT(x, y, x, y, 0, 0);
	//	 (*count)--;
	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	{   
        	tpd_button(x, y, 0); 
	}   		 

}

 static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
 {

	int i = 0;
	char data[(3+6*(TPD_MAX_PONIT-1)+3+1+7)/8*8] = {0}; // 3+6*(TPD_MAX_PONIT-1)+3+1 reserve the low byte of the last point
	u16 high_byte,low_byte;
	u8 report_rate =0;
	u8 version=0xff;
	u8 chargerset=0;//jy added

	p_point_num = point_num;

	for (i<0;i<(3+6*(TPD_MAX_PONIT-1)+3+1+7)/8;i++)
	{
		i2c_smbus_read_i2c_block_data(ft5336_i2c_client, 0x00+i*8, 8, &(data[i*8]));
	}
	i2c_smbus_read_i2c_block_data(ft5336_i2c_client, 0xa6, 1, &version);


	i2c_smbus_read_i2c_block_data(ft5336_i2c_client, 0x88, 1, &report_rate);
	i2c_smbus_read_i2c_block_data(ft5336_i2c_client, 0x8B, 1, &chargerset);//jy added
	
	//TPD_DEBUG("FW version=%x\n",version);	
	//TPD_DEBUG("received raw data from touch panel as following:\n");
	//TPD_DEBUG("[data[0]=%x,data[1]= %x ,data[2]=%x ,data[3]=%x ,data[4]=%x ,data[5]=%x,data[6]=%x]\n",data[0],data[1],data[2],data[3],data[4],data[5],data[6]);	//TPD_DEBUG("[data[9]=%x,data[10]= %x ,data[11]=%x ,data[12]=%x]\n",data[9],data[10],data[11],data[12]);
	//TPD_DEBUG("[data[9]=%x,data[10]= %x ,data[11]=%x ,data[12]=%x]\n",data[9],data[10],data[11],data[12]);
	//TPD_DEBUG("[data[15]=%x,data[16]= %x ,data[17]=%x ,data[18]=%x]\n",data[15],data[16],data[17],data[18]);

	//we have  to re update report rate
	// TPD_DMESG("report rate =%x\n",report_rate);
	if(report_rate < 8)
	{
		report_rate = 0x8;
		if((i2c_smbus_write_i2c_block_data(ft5336_i2c_client, 0x88, 1, &report_rate))< 0)
		{
			TPD_DMESG("TPD I2C read report rate error, line: %d\n", __LINE__);
		}
	}
	//added by jy
	//if(g_usb_connected!=chargerset)
	//TPD_DMESG("TPD I2C set  0x8B, line: %d,g_charging_enable=%d,chargerset=%d\n", __LINE__,g_charging_enable,chargerset);
	if(g_charging_enable!=chargerset)//charging is 1
		{
			TPD_DMESG("TPD I2C differ line:%d,set  0x8B:%d,g_charging_enable=%d\n", __LINE__,chargerset,g_charging_enable);
			if((i2c_smbus_write_i2c_block_data(ft5336_i2c_client, 0x8B, 1, &g_charging_enable))< 0)
			{
				TPD_DMESG("TPD I2C set  0x8B to 0 error, line: %d\n", __LINE__);
			}
			
		}
	
	//end
	 
	/* Device Mode[2:0] == 0 :Normal operating Mode*/
	if(data[0] & 0x70 != 0) return false; 

	/*get the number of the touch points*/
	point_num= data[2] & 0x0f;

	if(point_num>TPD_MAX_PONIT)
	{
		printk("TPD error ft5336 point_num(%d)>TPD_MAX_PONIT(%d)\n",point_num,TPD_MAX_PONIT);
		point_num=TPD_MAX_PONIT;
	}

	//TPD_DEBUG("point_num =%d\n",point_num);	
	//if(point_num == 0) return false;
	//TPD_DEBUG("Procss raw data...\n");
	
	for(i = 0; i < point_num; i++)
	{
		cinfo->p[i] = data[3+6*i] >> 6; //event flag 
                cinfo->id[i] = data[3+6*i+2] >> 4; //touch id

	       /*get the X coordinate, 2 bytes*/
		high_byte = data[3+6*i];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3+6*i + 1];
		cinfo->x[i] = high_byte |low_byte;
		//cinfo->x[i] =  cinfo->x[i] * 480 >> 11; //calibra
		
		/*get the Y coordinate, 2 bytes*/	
		high_byte = data[3+6*i+2];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3+6*i+3];
		cinfo->y[i] = high_byte |low_byte;
		//cinfo->y[i]=  cinfo->y[i] * 800 >> 11;
		
		cinfo->count++;
			
	}
	//printk("ft5336 cinfo->x[0] = %d, cinfo->y[0] = %d, cinfo->p[0] = %d\n", cinfo->x[0], cinfo->y[0], cinfo->p[0]);	
	//printk("ft5336 cinfo->x[1] = %d, cinfo->y[1] = %d, cinfo->p[1] = %d\n", cinfo->x[1], cinfo->y[1], cinfo->p[1]);		
	//printk("ft5336 cinfo->x[2]= %d, cinfo->y[2]= %d, cinfo->p[2] = %d\n", cinfo->x[2], cinfo->y[2], cinfo->p[2]);	
		  
	return true;

};

static int touch_event_handler(void *unused)
{
  
	struct touch_info cinfo, pinfo;
	int i=0;

	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	sched_setscheduler(current, SCHED_RR, &param);
 
	do
	{
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		set_current_state(TASK_INTERRUPTIBLE); 
		wait_event_interruptible(waiter,tpd_flag!=0);
						 
		tpd_flag = 0;
			 
		set_current_state(TASK_RUNNING);

		if (tpd_touchinfo(&cinfo, &pinfo)) 
		{
			TPD_DEBUG("TPD point_num = %d\n",point_num);
			TPD_DEBUG_SET_TIME;
			if(point_num >0) 
			{
				int i=0;
				for (i=0;i<point_num;i++)
		     		{
					//tpd_down(cinfo.x[i], cinfo.y[i], i+1);
					tpd_down(cinfo.x[i], cinfo.y[i], cinfo.id[i]);
		     		}
			 

				input_sync(tpd->dev);
				//TPD_DEBUG("press --->\n");
				
			} 
			else  
			{
				tpd_up(cinfo.x[0], cinfo.y[0], 0);
				//TPD_DEBUG("release --->\n"); 
				//input_mt_sync(tpd->dev);
				input_sync(tpd->dev);
			}
		}

	}while(!kthread_should_stop());
 
	 return 0;
}
 
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info) 
{
	strcpy(info->type, TPD_DEVICE);	
	return 0;
}
 
static void tpd_eint_interrupt_handler(void)
{
	//TPD_DEBUG("TPD interrupt has been triggered\n");
	TPD_DEBUG_PRINT_INT;
	tpd_flag = 1;
	wake_up_interruptible(&waiter);	 
}

static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{	
	int retval = TPD_OK;
#ifdef ESD_CHECK	
	int ret;
#endif
	char data;
	u8 report_rate=0;
	int err=0;
	int reset_count = 0;

reset_proc:   
	ft5336_i2c_client = client;
	
	//power on, need confirm with SA 
#ifdef TPD_POWER_SOURCE_CUSTOM
	//hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#else
    	hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
#endif

#ifdef TPD_POWER_SOURCE_1800
	#error "TPD_POWER_SOURCE_1800"
	hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif 

	
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(100);
	TPD_DMESG(" ft5336 reset\n");
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(300);
	
	TPD_DMESG(" ft5336 init eint\n");
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
 
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	msleep(100);

#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
	CTPI2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &CTPI2CDMABuf_pa, GFP_KERNEL);
    	if(!CTPI2CDMABuf_va)
	{
    		printk("[TSP] dma_alloc_coherent error\n");
	}
#endif 
	if((i2c_smbus_read_i2c_block_data(ft5336_i2c_client, 0x00, 1, &data))< 0)
	{
		TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
#ifdef TPD_RESET_ISSUE_WORKAROUND
        	if ( reset_count < TPD_MAX_RESET_COUNT )
        	{
			reset_count++;
			goto reset_proc;
        	}
#endif
		return -1; 
	}

	//set report rate 80Hz
	report_rate = 0x8; 
	if((i2c_smbus_write_i2c_block_data(ft5336_i2c_client, 0x88, 1, &report_rate))< 0)
	{
		if((i2c_smbus_write_i2c_block_data(ft5336_i2c_client, 0x88, 1, &report_rate))< 0)
		{
			TPD_DMESG("I2C read report rate error, line: %d\n", __LINE__);
		}
		   
	}

	tpd_load_status = 1;

#if defined (CONFIG_SUPPORT_FTS_CTP_UPG)  

#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(ft5336_i2c_client) < 0)
		dev_err(&ft5336_i2c_client->dev, "%s:[FTS] create fts control iic driver failed\n",
				__func__);
#endif

//jy add start
	if((err = misc_register(&tpd_misc_device)))

	{
		printk("mtk_tpd: tpd_misc_device register failed\n");

	}
//jy add end
#endif /* CONFIG_SUPPORT_FTS_CTP_UPG */


//start --peng.liu added for reading the Vendor ID of TP 2014.08.08
	id_version_flag=ft5x0x_read_ID_ver();
	   printk("[TSP] checked TP  vendor id_version_flag =0x%x\n",id_version_flag);

	if (id_version_flag == 0x51 || id_version_flag ==  0x43)//HRC
	{
#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
		CTPM_FW=&CTPM_FW_HRC;
		g_length=sizeof(CTPM_FW_HRC);
#endif
		printk("[TSP] checked TP  vendor is HRC\n");
	}
	else if (id_version_flag == 0xa0)
	{
#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
				CTPM_FW=&CTPM_FW_TOP;
				g_length=sizeof(CTPM_FW_TOP);
#endif        
	    printk("[TSP] checked TP  vendor is TopTouch\n");
	}
	else
	{
		printk("[TSP] checked TP  vendor is not match");
        goto JUMP_OUT_FW;
	}
//---end  

//#ifdef CONFIG_AUTO_FTS_CTP_UPG
#if 0
	fw_version_flag=ft5x0x_read_fw_ver();

   	printk("[TSP] TP  firmware file version is 0x%x,FW_version in TP is:0x%x\n", CTPM_FW[g_length-2],fw_version_flag);
	
	if(fw_version_flag< CTPM_FW[g_length-2])//modify by jy 2014.05.09
	{
			
			mtk_wdt_restart(WK_WDT_EXT_TYPE);
			mtk_wdt_restart(WK_WDT_LOC_TYPE);
		    	printk("[TSP] Step 0:init \n");
			msleep(100);
			fts_ctpm_fw_upgrade_with_i_file(ft5336_i2c_client);
		    	printk("[TSP] Step 8:init stop\n");
			printk("[wj]the version is 0x%02x.\n", fw_version_flag);
			mtk_wdt_restart(WK_WDT_EXT_TYPE);
			mtk_wdt_restart(WK_WDT_LOC_TYPE);
			printk("[TSP] after upgrade FW_version in TP is:%x\n", fw_version_flag);
	}
#endif

JUMP_OUT_FW:

#ifdef SLT_DEVINFO_CTP
#ifdef __LCT_ADD_TP_VERSION__
	g_ctp_proc = create_proc_entry(CTP_PROC_FILE, 0444, NULL);
	if (g_ctp_proc == NULL)
	{
		printk("create_proc_entry failed\n");
	}
	else
	{
		g_ctp_proc->read_proc = ctp_proc_read;
		//g_ctp_proc->write_proc = ctp_proc_write;	 //open it for ADB update
		printk("create_proc_entry success\n");
	}
	g_ctp_vender=create_proc_entry("ctp_vender", 0444, NULL);
	if (g_ctp_vender == NULL)
	{
		printk("create_proc_entry vender failed\n");
	}
	else
	{
		g_ctp_vender->read_proc = ctp_vender_read;
		//g_ctp_proc->write_proc = ctp_proc_write;	 //open it for ADB update
		printk("create_proc_entry vender success\n");
	}
#endif
	tp_devinfo_init();
#endif

#ifdef ESD_CHECK	
	ctp_read_id_workqueue = create_workqueue("ctp_read_id");
	INIT_DELAYED_WORK(&ctp_read_id_work, ESD_read_id_workqueue);
	ret = queue_delayed_work(ctp_read_id_workqueue, &ctp_read_id_work,400); //schedule a work for the first detection					
    	printk("[TSP] ret =%d\n",ret);
#endif

	ft5336_thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(ft5336_thread))
	{ 
		retval = PTR_ERR(ft5336_thread);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
	}

	TPD_DMESG("ft5336 Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");

#if defined (CONFIG_SUPPORT_FTS_CTP_UPG)  

	atomic_set(&upgrading, 0);
#endif /* CONFIG_SUPPORT_FTS_CTP_UPG */

	return 0;
}

static int __devexit tpd_remove(struct i2c_client *client) 
{
        int err;
	TPD_DEBUG("TPD removed\n");
#ifdef CONFIG_SUPPORT_FTS_CTP_UPG 
	if(CTPI2CDMABuf_va)
	{
		dma_free_coherent(NULL, 4096, CTPI2CDMABuf_va, CTPI2CDMABuf_pa);
		CTPI2CDMABuf_va = NULL;
		CTPI2CDMABuf_pa = 0;
	}

	#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
	#endif

#endif

#ifdef ESD_CHECK	
	destroy_workqueue(ctp_read_id_workqueue);
#endif	

	return 0;
}
 
 
static int tpd_local_init(void)
{
	TPD_DMESG("Focaltech FT5366 I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
	if(i2c_add_driver(&tpd_i2c_driver)!=0)
   	{
  		TPD_DMESG("ft5366 unable to add i2c driver.\n");
      		return -1;
	}
	if(tpd_load_status == 0) 
	{
    		TPD_DMESG("ft5366 add error touch panel driver.\n");
    		i2c_del_driver(&tpd_i2c_driver);
    		return -1;
	}
	
#ifdef TPD_HAVE_BUTTON     
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif   
  
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
	memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
	TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
	tpd_type_cap = 1;
	return 0; 
}

static void tpd_resume( struct early_suspend *h )
{
	//int retval = TPD_OK;
	char data;
 
	TPD_DMESG("TPD wake up\n");
#if defined (CONFIG_SUPPORT_FTS_CTP_UPG) 
   	if(1 == atomic_read(&upgrading))
	{
		return;
	}
#endif /* CONFIG_SUPPORT_FTS_CTP_UPG */


	discard_resume_first_eint = KAL_TRUE;
#ifdef TPD_POWER_SOURCE_CUSTOM
    	//hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#else
    	hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
    	hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif	
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(10);  
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);

	msleep(200);//add this line 
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

#ifdef ESD_CHECK	
    	msleep(1);  
	queue_delayed_work(ctp_read_id_workqueue, &ctp_read_id_work,400); //schedule a work for the first detection					
#endif
	
	msleep(20);
	if((i2c_smbus_read_i2c_block_data(ft5336_i2c_client, 0x00, 1, &data))< 0)
	{
		TPD_DMESG("TPD resume I2C transfer error, line: %d\n", __LINE__);
	}
	tpd_halt = 0;//add this line 
	tpd_up(0,0,0);
	input_sync(tpd->dev);
	TPD_DMESG("TPD wake up done\n");
	//return retval;
 }

static void tpd_suspend( struct early_suspend *h )
{
	//int retval = TPD_OK;
	static char data = 0x3;

#if defined (CONFIG_SUPPORT_FTS_CTP_UPG)  
	if(1 == atomic_read(&upgrading))
	{
		return;
	}
#endif /* CONFIG_SUPPORT_FTS_CTP_UPG */

#ifdef ESD_CHECK	
 	cancel_delayed_work_sync(&ctp_read_id_work);
#endif

	TPD_DMESG("TPD enter sleep\n");
	tpd_halt = 1; //add this line 
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	i2c_smbus_write_i2c_block_data(ft5336_i2c_client, 0xA5, 1, &data);  //TP enter sleep mode
#ifdef TPD_POWER_SOURCE_CUSTOM
    	//hwPowerDown(TPD_POWER_SOURCE_CUSTOM,  "TP");
#else
    	hwPowerDown(MT65XX_POWER_LDO_VGP2,  "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
    	hwPowerDown(TPD_POWER_SOURCE_1800,  "TP");
#endif

        TPD_DMESG("TPD enter sleep done\n");
	 
 } 


static ssize_t show_chipinfo(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct i2c_client *client =ft5336_i2c_client;
	unsigned char ver=0;
	unsigned char id=0;
	
	if(NULL == client)
	{
		printk("i2c client is null!!\n");
		return 0;
	}

	id=ft5x0x_read_ID_ver();
	ver=ft5x0x_read_fw_ver();

     switch(id)
     {
          case 0x43:
		      return sprintf(buf,"ID:0x%x VER:0x%x IC:ft5336 VENDOR:HRC\n",id, ver);
              break;

		  case 0xa0:		  	
		      return sprintf(buf,"ID:0x%x VER:0x%x IC:ft5336 VENDOR:TopTouch\n",id, ver);
              break;
			  
		  default:		  	
			  return sprintf(buf,"ID:0x%x VER:0x%x IC:ft5336 VENDOR:others\n",id, ver);	
		  	
	 }

}

static DEVICE_ATTR(chipinfo, 0444, show_chipinfo, NULL);

static const struct device_attribute * const ctp_attributes[] = {
	&dev_attr_chipinfo
};

static struct tpd_driver_t tpd_device_driver = {
		 .tpd_device_name = "FT5336",
		 .tpd_local_init = tpd_local_init,
		 .suspend = tpd_suspend,
		 .resume = tpd_resume,
		 .attrs=
		 {
			.attr=ctp_attributes,
			.num=1
		 },
#ifdef TPD_HAVE_BUTTON
		 .tpd_have_button = 1,
#else
		 .tpd_have_button = 0,
#endif		
 };

/* called when loaded into kernel */
#if defined(CONFIG_SUPPORT_FTS_CTP_UPG)
static ssize_t tp_test(struct kobject *kobj,struct bin_attribute *attr,char *buf, loff_t off, size_t count)
{
	uint16_t val;
	printk("tp_test\n");
	if(fts_ctpm_fw_upgrade_with_i_file(ft5336_i2c_client)!=0){
		TPD_DMESG(TPD_DEVICE " luosen failed to upgrade firmware, line: %d\n", __LINE__);
	}
	return count;
}

static ssize_t tp_read(struct kobject *kobj,struct bin_attribute *attr,char *buf, loff_t off,size_t count)
{
	printk("tp_read!!!!!\n");
	int i=300;
	while(i>0)
	{
		mdelay(100);
		i--;
	}
	return count;
}
static struct bin_attribute tp_mode_attr = {
	.attr = {
		.name = "tp",
		.mode = S_IRUGO | S_IWUSR,
	},
	.size = 4,
	.read = tp_read,
	.write = tp_test,
};
#endif

static int __init tpd_driver_init(void) {
	printk("MediaTek FT5336 touch panel driver init\n");
#if defined(CONFIG_SUPPORT_FTS_CTP_UPG)	 
	int ret;
	ret = sysfs_create_bin_file(&(module_kset->kobj), &tp_mode_attr);
	if (ret) {
		printk(KERN_ERR "<CTP> Failed to create sys file\n");
		return -ENOMEM;
	}
#endif	
	i2c_register_board_info(1, &ft5336_i2c_tpd, 1);
	if(tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("add FT5336 driver failed\n");
	return 0;
}
 
/* should never be called */
static void __exit tpd_driver_exit(void) {
	TPD_DMESG("MediaTek FT5336 touch panel driver exit\n");
	//input_unregister_device(tpd->dev);
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
