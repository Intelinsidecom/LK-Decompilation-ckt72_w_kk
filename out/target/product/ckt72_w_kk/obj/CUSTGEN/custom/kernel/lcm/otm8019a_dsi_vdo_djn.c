#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif



// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(800)

#define REGFLAG_DELAY             							0XFD
#define REGFLAG_END_OF_TABLE      							0xFE   // END OF REGISTERS MARKER

#define LCM_ID (0x018B)

#ifndef TRUE
    #define   TRUE     1
#endif

#ifndef FALSE
    #define   FALSE    0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)       

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		// enable tearing-free
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 2;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.vertical_sync_active				= 8;
		params->dsi.vertical_backporch					= 8;
		params->dsi.vertical_frontporch					= 8;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 6;
		params->dsi.horizontal_backporch				= 37;
		params->dsi.horizontal_frontporch				= 37;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		// Bit rate calculation
	//	params->dsi.pll_div1=29;		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
	//	params->dsi.pll_div2=1; 		// div2=0~15: fout=fvo/(2*div2)
    params->dsi.PLL_CLOCK = 210; //dsi clock customization: should config clock value directly
		/* ESD or noise interference recovery For video mode LCM only. */ // Send TE packet to LCM in a period of n frames and check the response. 
		params->dsi.lcm_int_te_monitor = FALSE; 
		params->dsi.lcm_int_te_period = 1; // Unit : frames 
 
		// Need longer FP for more opportunity to do int. TE monitor applicably. 
		if(params->dsi.lcm_int_te_monitor) 
			params->dsi.vertical_frontporch *= 2; 
 
		// Monitor external TE (or named VSYNC) from LCM once per 2 sec. (LCM VSYNC must be wired to baseband TE pin.) 
		params->dsi.lcm_ext_te_monitor = FALSE; 
		// Non-continuous clock 
		params->dsi.noncont_clock = TRUE; 
		params->dsi.noncont_clock_period = 2; // Unit : frames
}

static unsigned int lcm_compare_id(void)
{
    unsigned char buffer[5];
    unsigned int data_array[16];

    SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(50);	

    data_array[0] = 0x00053700;// read id return two byte,version and id
    dsi_set_cmdq(data_array, 1, 1);

    read_reg_v2(0xA1, buffer, 5);
	
#ifdef BUILD_LK
	printf("%s, id = %x id1=%x id2 = %x id1=%x id2 = %x\n", __func__, buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);						 
#else
	printk("%s, id = %x id1=%x id2 = %x id1=%x id2 = %x\n", __func__, buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);						 
#endif  

    return (LCM_ID == ((buffer[0] << 8) | buffer[1]))? 1: 0;                                                                                 
			
}


static struct LCM_setting_table lcm_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1 => Data ID = 0x39
		count of parameters = 1 => Data ID = 0x15
		count of parameters = 0 => Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/

{0x00, 1 , {0x00}},
{0xFF,  3 ,{0x80,0x19,0x01}},

{0x00, 1 , {0x80}},
{0xFF,  2 ,{0x80,0x19}},

{0x00, 1 , {0xB4}},
{0xC0,  1 ,{0x00}},

{0x00, 1 , {0x81}},
{0xC5,  1 ,{0x66}},

{0x00, 1 , {0x82}},
{0xC5,  1 ,{0xB0}},

{0x00, 1 , {0x90}},
{0xC5,  2 ,{0x4E,0x76}},

{0x00, 1 , {0x00}},
{0xD8,  2 ,{0x6F,0x6F}},

{0x00, 1 , {0x00}},
{0xD9,  1 ,{0x4F}},

{0x00, 1 , {0x81}},
{0xC1,  1 ,{0x33}},

{0x00, 1 , {0xA1}},
{0xC1,  1 ,{0x08}},

{0x00, 1 , {0x81}},
{0xC4,  1 ,{0x83}},

{0x00, 1 , {0xB1}},
{0xC5,  1 ,{0xA9}},

{0x00, 1 , {0x93}},
{0xC5,  1 ,{0x03}},

{0x00, 1 , {0x92}},
{0xB3,  1 ,{0x40}},

{0x00, 1 , {0x90}},
{0xB3,  1 ,{0x02}},

{0x00, 1 , {0x80}},
{0xC0,  9 ,{0x00,0x58,0x00,0x15,0x15,0x00,0x58,0x15,0x15}},

{0x00, 1 , {0x90}},
{0xC0,  6 ,{0x00,0x15,0x00,0x00,0x00,0x03}},

{0x00, 1 , {0x80}},
{0xCE, 12 ,{0x8B,0x03,0x00,0x8A,0x03,0x00,0x89,0x03,0x00,0x88,0x03,0x00}},

{0x00, 1 , {0xA0}},
{0xCE, 14 ,{0x38,0x07,0x03,0x54,0x00,0x02,0x00,0x38,0x06,0x03,0x55,0x00,0x02,0x00}},

{0x00, 1 , {0xB0}},
{0xCE, 14 ,{0x38,0x05,0x03,0x56,0x00,0x02,0x00,0x38,0x04,0x03,0x57,0x00,0x02,0x00}},

{0x00, 1 , {0xC0}},
{0xCE, 14 ,{0x38,0x03,0x03,0x58,0x00,0x02,0x00,0x38,0x02,0x03,0x59,0x00,0x02,0x00}},

{0x00, 1 , {0xD0}},
{0xCE, 14 ,{0x38,0x01,0x03,0x5A,0x00,0x02,0x00,0x38,0x00,0x03,0x5B,0x00,0x02,0x00}},

{0x00, 1 , {0xC0}},
{0xCF, 10 ,{0x02,0x02,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00}},

{0x00, 1 , {0x90}},
{0xCB, 15 ,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00, 1 , {0xA0}},
{0xCB,  1 ,{0x00}},

{0x00, 1 , {0xA5}},
{0xCB, 10 ,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00, 1 , {0xB0}},
{0xCB,  6 ,{0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00, 1 , {0xC0}},
{0xCB, 15 ,{0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00}},

{0x00, 1 , {0xD0}},
{0xCB,  1 ,{0x00}},

{0x00, 1 , {0xD5}},
{0xCB, 10 ,{0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01}},

{0x00, 1 , {0xE0}},
{0xCB,  6 ,{0x01,0x01,0x01,0x01,0x01,0x01}},

{0x00, 1 , {0x80}},
{0xCC, 10 ,{0x26,0x25,0x21,0x22,0x0C,0x0A,0x10,0x0E,0x02,0x04}},

{0x00, 1 , {0x90}},
{0xCC, 15 ,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00, 1 , {0xA0}},
{0xCC, 11 ,{0x00,0x03,0x01,0x0D,0x0F,0x09,0x0B,0x22,0x21,0x25,0x26}},

{0x00, 1 , {0xB0}},
{0xCC, 10 ,{0x25,0x26,0x21,0x22,0x10,0x0A,0x0C,0x0E,0x04,0x02}},

{0x00, 1 , {0xC0}},
{0xCC,  6 ,{0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00, 1 , {0xCA}},
{0xCC,  5 ,{0x00,0x00,0x00,0x00,0x00}},

{0x00, 1 , {0xD0}},
{0xCC, 11 ,{0x00,0x01,0x03,0x0D,0x0B,0x09,0x0F,0x22,0x21,0x26,0x25}},

{0x00, 1 , {0x00}},
{0xE1, 10 ,{0x00,0x05,0x11,0x1A,0x28,0x34,0x36,0x61,0x52,0x6D}},
{0xE1, 10 ,{0x94,0x7B,0x8A,0x60,0x5A,0x49,0x38,0x2A,0x19,0x00}},

{0x00, 1 , {0x00}},
{0xE2, 10 ,{0x00,0x05,0x10,0x1A,0x29,0x35,0x36,0x60,0x52,0x6D}},
{0xE2, 10 ,{0x94,0x7C,0x8A,0x60,0x5A,0x49,0x38,0x2A,0x19,0x00}},

{0x00, 1 , {0x80}},
{0xC4,  1 ,{0x30}},

{0x00, 1 , {0x98}},
{0xC0,  1 ,{0x00}},

{0x00, 1 , {0xA9}},
{0xC0,  1 ,{0x0A}},

{0x00, 1 , {0xB0}},
{0xC1,  3 ,{0x20,0x00,0x00}},

{0x00, 1 , {0xE1}},
{0xC0,  2 ,{0x40,0x30}},

{0x00, 1 , {0x80}},
{0xC1,  2 ,{0x03,0x33}},

{0x00, 1 , {0xA0}},
{0xC1,  1 ,{0xE8}},

{0x00, 1 , {0x90}},
{0xB6,  1 ,{0xB4}},
{REGFLAG_DELAY, 10, {}},

{0x00, 1 , {0x00}},
{0xFB,  1 ,{0x01}},

{0x00, 1 , {0x00}},
{0xFF,  3 ,{0xFF,0xFF,0xFF}},
	
	{0x11,	1,	{0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29,	1,	{0x00}},
	{REGFLAG_DELAY, 10, {}},
	{0x2C,	0,	{}},
	
	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.


	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(50);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
#ifdef BUILD_LK
  printf("[Wenleon]LK---cmd---otm8019a_dsi_vdo_init----%s------\n",__func__);
#else
  printk("[Wenleon]kernel---cmd---otm8019a_dsi_vdo_init----%s------\n",__func__);
#endif

}

static void lcm_suspend(void)
{
    unsigned int data_arry[16];

    //mipi_1data(0x28,0X00); 
    data_arry[0] = 0x00280500;
    dsi_set_cmdq(&data_arry,1,1);
    MDELAY(120);
    //mipi_1data(0x10,0X00); 
    data_arry[0] = 0x00100500;
    dsi_set_cmdq(&data_arry,1,1);
    //MDELAY(150);
}


static void lcm_resume(void)
{
    unsigned int data_arry[16];

    //mipi_1data(0x51,0XFF); 
    //data_arry[0] = 0xFF511500;
   // dsi_set_cmdq(&data_arry,1,1);
   // MDELAY(50);
    //mipi_1data(0x11,0X00); 
    data_arry[0] = 0x00110500;
    dsi_set_cmdq(&data_arry,1,1);
    MDELAY(150);
    //mipi_1data(0x29,0X00); 
    data_arry[0] = 0x00290500;
    dsi_set_cmdq(&data_arry,1,1);


}


#ifndef BUILD_LK
//#define ESD_DEBUG
#endif

static unsigned int lcm_esd_check(void)
{
    static int count = 0;
    static int err_count = 0;
    static int uncount = 0;
    int i;
    unsigned char fResult;
    unsigned char buffer[12];
    unsigned int array[16];

#ifdef ESD_DEBUG
    printk("lcm_esd_check <<<\n");
#endif
    for (i = 0; i < 12; i++)
        buffer[i] = 0x00;

    //---------------------------------
    // Set Maximum Return Size
    //---------------------------------
    array[0] = 0x00013700;
    dsi_set_cmdq(array, 1, 1);

    //---------------------------------
    // Read [9Ch, 00h, ECC] + Error Report(4 Bytes)
    //---------------------------------
    read_reg_v2(0x0A, buffer, 7);

#ifdef ESD_DEBUG
    printk("lcm_esd_check : read(0x0A)\n");
#endif

#ifdef ESD_DEBUG
    for (i = 0; i < 7; i++)
        printk("buffer[%d] : 0x%x \n", i, buffer[i]);
#endif

    //---------------------------------
    // Judge Readout & Error Report
    //---------------------------------
    if (buffer[3] == 0x02) // Check data identifier of error report
    {
        if (buffer[4] & 0x02) // Check SOT sync error
            err_count ++;
        else
            err_count = 0;
    }
    else
    {
        err_count = 0;
    }

#ifdef ESD_DEBUG
    printk("lcm_esd_check err_count = %d\n", err_count);
#endif
    if ((buffer[0] != 0x9C) || (err_count >= 2))
    {
        err_count = 0;
        uncount++;

#ifndef BUILD_LK
        printk("lcm_esd_check failed, err_count = %d\n", err_count);
        for (i = 0; i < 7; i++)
	        printk("buffer[%d] : 0x%x \n", i, buffer[i]);
#endif

#ifdef ESD_DEBUG
        printk("lcm_esd_check unnormal uncount = %d\n", uncount);
        printk("lcm_esd_check >>>\n");
#endif
        fResult = 1;
    }
    else
    {
        count ++;
#ifdef ESD_DEBUG
        printk("lcm_esd_check normal count = %d\n", count);
        printk("lcm_esd_check >>>\n");
#endif
        fResult = 0;
    }

    //---------------------------------
    // Set Maximum Return Size
    //---------------------------------
    array[0] = 0x00033700;
    dsi_set_cmdq(array, 1, 1);

    //---------------------------------
    // Clear D-PHY Buffer
    // Read [WC, WC, ECC, P1, P2, P3, CRC0, CRC1]+ Error Report(4 Bytes)
    //---------------------------------
    read_reg_v2(0xBC, buffer, 12);

#ifdef ESD_DEBUG
    printk("lcm_esd_check : read(0xBC)\n");
#endif

#ifdef ESD_DEBUG
    for (i = 0; i < 12; i++)                 		
        printk("buffer[%d] : 0x%x \n", i, buffer[i]); 
#endif

    if (fResult)
        return TRUE;
    else
        return FALSE;
} 

static unsigned int lcm_esd_recover(void)
{
    static int recount = 0;

#ifdef ESD_DEBUG
    printk("lcm_esd_recover\n");
#endif

    lcm_init();
    recount ++;

#ifndef BUILD_LK
    printk("lcm_esd_recover recover recount = %d\n", recount);
#endif
    return TRUE;
}

LCM_DRIVER otm8019a_dsi_vdo_lcsh72_lcm_drv = 
{
    .name           = "otm8019a_dsi_vdo_lcm_drv_dnj",
    .set_util_funcs = lcm_set_util_funcs,
    .compare_id     = lcm_compare_id,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .esd_check      = lcm_esd_check,      //only for command mode, no use in video mode
    .esd_recover    = lcm_esd_recover,    //only for command mode, no use in video mode
};

