/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
#if 1
#if defined(BUILD_LK)
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif

#if !defined(BUILD_LK)
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#else

#ifdef BUILD_LK
#include <string.h>
#else
#include <linux/string.h>
#endif


#ifdef BUILD_LK
#include "cust_gpio_usage.h"
#else
#include "cust_gpio_usage.h"
#endif

#ifndef BUILD_LK
#include <mach/mt_gpio.h>
#endif

/*
#ifndef BUILD_LK
#include <linux/printk.h>
#endif
*/

#ifndef BUILD_LK
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(800)
#define LCM_ID       (0x00)
#define REGFLAG_DELAY             							0XFB
#define REGFLAG_END_OF_TABLE      							0xFA   // END OF REGISTERS MARKER


#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
#define LCM_DSI_CMD_MODE									0

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

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {

	/*
	Note :

	Data ID will depends on the following rule.

		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag

	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},     // Change to Page 1
	{REGFLAG_DELAY, 10, {}},////{REGFLAG_DELAY, 10, {}},//DELAY,10
	{0x20,1,{0x00}},
	
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},     // Change to Page 1
	{REGFLAG_DELAY, 10, {}},////{REGFLAG_DELAY, 10, {}},//DELAY,10

	{0x08,1,{0x10}},  
	
	{0x21,1,{0x01}},                 // DE = 1 Active
	{0x30,1,{0x02}},                 // 480 X 854
	{0x31,1,{0x02}},                 // 
	
	{0x40,1,{0x15}},                // BT
	{0x41,1,{0x55}},                // DDVDH/DDVDL 44 ·|¦³²Ê¾î½u
	{0x42,1,{0x11}},                // VGH/VGL
	{0x43,1,{0x09}},                // VGH_CP_OFF
	{0x44,1,{0x0C}},                // VGL_CP_OFF	  
	{0x45,1,{0x1B}},

	
	{0x50,1,{0x50}},                // VREG1
	{0x51,1,{0x50}},                // VREG2
	{0x52,1,{0x00}},  // flicker MSB
	{0x53,1,{0x43}},  // flicker LSB


	{0x60,1,{0x07}},                 
	{0x61,1,{0x06}},                 
	{0x62,1,{0x06}},                
	{0x63,1,{0x04}},             
//+++++++++++++ Gamma Setting ++++++++++++++++++//
	{0xA0,1,{0x10}},  // Gamma 0 
	{0xA1,1,{0x10}},  // Gamma 4 
	{0xA2,1,{0x10}},  // Gamma 8
	{0xA3,1,{0x0B}},  // Gamma 16
	{0xA4,1,{0x04}},  // Gamma 24
	{0xA5,1,{0x08}},  // Gamma 52
	{0xA6,1,{0x08}},  // Gamma 80
	{0xA7,1,{0x17}},  // Gamma 108
	{0xA8,1,{0x05}},  // Gamma 147
	{0xA9,1,{0x08}},  // Gamma 175
	{0xAA,1,{0x12}},  // Gamma 203
	{0xAB,1,{0x08}},  // Gamma 231
	{0xAC,1,{0x0E}},  // Gamma 239
	{0xAD,1,{0x1A}},  // Gamma 247
	{0xAE,1,{0x12}},  // Gamma 251
	{0xAF,1,{0x00}},  // Gamma 255
	///==============Nagitive
	{0xC0,1,{0x10}},  // Gamma 0 
	{0xC1,1,{0x10}},  // Gamma 4
	{0xC2,1,{0x10}},  // Gamma 8
	{0xC3,1,{0x0B}},  // Gamma 16
	{0xC4,1,{0x04}},  // Gamma 24
	{0xC5,1,{0x08}},  // Gamma 52
	{0xC6,1,{0x07}},  // Gamma 80
	{0xC7,1,{0x17}},  // Gamma 108
	{0xC8,1,{0x05}},  // Gamma 147
	{0xC9,1,{0x08}},  // Gamma 175
	{0xCA,1,{0x12}},  // Gamma 203
	{0xCB,1,{0x08}},  // Gamma 231
	{0xCC,1,{0x0E}},  // Gamma 239
	{0xCD,1,{0x1A}},  // Gamma 247
	{0xCE,1,{0x12}},  // Gamma 251
	{0xCF,1,{0x00}},  // Gamma 255
//+++++++++++++++++++++++++++++++++++++++++++++++++++//

//****************************************************************************//
//****************************** Page 6 Command ******************************//
//****************************************************************************//
// For P1_R22=00h (GS=0)
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x06}},     // Change to Page 6
	{0x00,1,{0xA0}},
	{0x01,1,{0x05}},
	{0x02,1,{0x00}},    
	{0x03,1,{0x00}},
	{0x04,1,{0x01}},
	{0x05,1,{0x01}},
	{0x06,1,{0x88}},    
	{0x07,1,{0x04}},
	{0x08,1,{0x01}},
	{0x09,1,{0x90}},    
	{0x0A,1,{0x04}},    
	{0x0B,1,{0x01}},    
	{0x0C,1,{0x01}},
	{0x0D,1,{0x01}},
	{0x0E,1,{0x00}},
	{0x0F,1,{0x00}},
	{0x10,1,{0x55}},
	{0x11,1,{0x50}},
	{0x12,1,{0x01}},
	{0x13,1,{0x85}},
	{0x14,1,{0x85}},
	{0x15,1,{0xC0}},
	{0x16,1,{0x0B}},
	{0x17,1,{0x00}},
	{0x18,1,{0x00}},
	{0x19,1,{0x00}},
	{0x1A,1,{0x00}},
	{0x1B,1,{0x00}},
	{0x1C,1,{0x00}},
	{0x1D,1,{0x00}},

	{0x20,1,{0x01}},
	{0x21,1,{0x23}},
	{0x22,1,{0x45}},
	{0x23,1,{0x67}},
	{0x24,1,{0x01}},
	{0x25,1,{0x23}},
	{0x26,1,{0x45}},
	{0x27,1,{0x67}},

	{0x30,1,{0x02}},
	{0x31,1,{0x22}},
	{0x32,1,{0x11}},
	{0x33,1,{0xAA}},
	{0x34,1,{0xBB}},
	{0x35,1,{0x66}},
	{0x36,1,{0x00}},
	{0x37,1,{0x22}},
	{0x38,1,{0x22}},
	{0x39,1,{0x22}},
	{0x3A,1,{0x22}},
	{0x3B,1,{0x22}},
	{0x3C,1,{0x22}},
	{0x3D,1,{0x22}},
	{0x3E,1,{0x22}},
	{0x3F,1,{0x22}},

	{0x40,1,{0x22}},
	{0x53,1,{0x12}},
//***********************************************************************//
//************************* Page 7 Command ******************************//
//***********************************************************************//
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x07}},     // Change to Page 7
	{0x18,1,{0x1D}},                 // VGH_REG/VGL_REG Out
	{0x17,1,{0x32}},                 // VREG1/2OUT Out
	{0x02,1,{0x77}}, 
	{0xE1,1,{0x79}},   

//****************************************************************************//
//****************************** Page 0 Command ******************************//
//****************************************************************************//
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},     // Change to Page 0
	{0x11,1,{0x00}},                 // Sleep-Out
	{REGFLAG_DELAY, 120, {}},//DELAY,10
	{0x29,1,{0x00}},                 // Display On
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.
	// Setting ending by predefined flag
 
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {

	{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}}, // Change to Page 0 
	{0x11,1,{0x00}}, // Sleep-Out 
	{REGFLAG_DELAY, 120, {}},//DELAY,120 
	{0x29,1,{0x00}}, // Display On 
	{REGFLAG_DELAY, 10, {}},//DELAY,10
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x08}}, // Change to Page 8	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},

	// Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x08}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
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
		params->dsi.mode   = BURST_VDO_MODE;
#endif

		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		params->dsi.packet_size=256;

        params->dsi.intermediat_buffer_num = 2;	
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

        params->dsi.word_count=480*3;
		params->dsi.vertical_sync_active=4;  
		params->dsi.vertical_backporch=16;
		params->dsi.vertical_frontporch=20;
		params->dsi.vertical_active_line=FRAME_HEIGHT;
	      
		params->dsi.horizontal_sync_active=10;  
		params->dsi.horizontal_backporch=41;      
		params->dsi.horizontal_frontporch=41;    
        params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	//	params->dsi.vertical_active_line = 800;
	//	params->dsi.compatibility_for_nvk = 0;
		
		 params->dsi.horizontal_blanking_pixel				 = 60;

		// Bit rate calculation
		//params->dsi.pll_div1=29;		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
		//params->dsi.pll_div2=1; 		// div2=0~15: fout=fvo/(2*div2)

        //params->dsi.pll_div1=1;     // div1=0,1,2,3;div1_real=1,2,4,4
        //params->dsi.pll_div2=1;     // div2=0,1,2,3;div1_real=1,2,4,4
        //params->dsi.fbk_div =30;     // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)

		params->dsi.PLL_CLOCK = 221;
		
		/* ESD or noise interference recovery For video mode LCM only. */ // Send TE packet to LCM in a period of n frames and check the response. 
		params->dsi.lcm_int_te_monitor = FALSE; 
		params->dsi.lcm_int_te_period = 1; // Unit : frames 
 
		// Need longer FP for more opportunity to do int. TE monitor applicably. 
		if(params->dsi.lcm_int_te_monitor) 
			params->dsi.vertical_frontporch *= 2; 
 
		// Monitor external TE (or named VSYNC) from LCM once per 2 sec. (LCM VSYNC must be wired to baseband TE pin.) 
		params->dsi.lcm_ext_te_monitor = FALSE; 
		// Non-continuous clock 
		params->dsi.noncont_clock = FALSE; 
		params->dsi.noncont_clock_period = 2; // Unit : frames

}


static void lcm_init(void)
{
	unsigned int data_array[16];
	
	//lcm_compare_id();

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

#ifdef BUILD_LK
  printf("[Wenleon]LK---cmd---ili9806e_dsi_cmd_init----%s------\n",__func__);
#else
  printk("[Wenleon]kernel---cmd---ili9806e_dsi_cmd_init----%s------\n",__func__);
#endif


}


static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	SET_RESET_PIN(0);
	MDELAY(50);
	SET_RESET_PIN(1);
	MDELAY(50);
}


static void lcm_resume(void)
{
	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
	lcm_init();
}


static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	data_array[3]= 0x00053902;
	data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[5]= (y1_LSB);
	data_array[6]= 0x002c3909;

	dsi_set_cmdq(data_array, 7, 0);

}


static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_UBOOT

       unsigned char buffer_vcom[4];

       unsigned char buffer_0a[1];

       unsigned int array[16];

	array[0]=0x00063902; 
	array[1]=0x0698FFFF; 
	array[2]=0x00000004; 
	dsi_set_cmdq(array, 3, 1);

       array[0] = 0x00013700;

       dsi_set_cmdq(array, 1, 1);

      read_reg_v2(0x0A,buffer_0a, 1);

      if ((buffer_0a[0]==0x9C))
	  {

               return 0;
	   }
	  else
	  {

              return 1;
          }
#endif
}


static unsigned int lcm_esd_recover(void)
{

   #ifndef BUILD_UBOOT


       lcm_init();

       return 1;

      #endif 
}


static unsigned int lcm_compare_id(void)
{
      unsigned char buffer_vcom[4];
      unsigned char buffer[4];
      unsigned int array[16];
      unsigned int id=0;
      unsigned int id1=0; 
       SET_RESET_PIN(1);	//NOTE:should reset LCM firstly
       MDELAY(10);
       SET_RESET_PIN(0);
       MDELAY(10);
       SET_RESET_PIN(1);
       MDELAY(120);	

       array[0]=0x00063902;
       array[1]=0x0698FFFF; 
       array[2]=0x00000104;
       dsi_set_cmdq(array, 3, 1);
       MDELAY(10);


       array[0] = 0x00013700;// set return byte number
       dsi_set_cmdq(array, 1, 1);
	 
       read_reg_v2(0x00, buffer_vcom, 1);
       buffer[0]=buffer_vcom[0];

       array[0] = 0x00013700;// set return byte number
       dsi_set_cmdq(array, 1, 1);

       read_reg_v2(0x01, buffer_vcom, 1);
        buffer[1]=buffer_vcom[0];

        array[0] = 0x00013700;// set return byte number
       dsi_set_cmdq(array, 1, 1);
	 //MDELAY(10); 
	 
       read_reg_v2(0x02, buffer_vcom, 1);
        buffer[2]=buffer_vcom[0];

#ifdef BUILD_LK
			printf("%s, id = %x id1=%x id2 = %x\n", __func__, buffer[0],buffer[1],buffer[2]);						 
#else
			printk("%s, id = %x id1=%x id2 = %x\n", __func__, buffer[0],buffer[1],buffer[2]);						 
#endif                                                                                   
			


    if((buffer[0]==0x98) && (buffer[1]==0x06) && (buffer[2]==0x04))
      {
      		return 1;
      }
      else 
      {
      		return 0;
      }
}
// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER ili9806e_dsi_cmd_6572_drv =
{
    .name			= "ili9806e_dsi_cmd_6572",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
//#if (LCM_DSI_CMD_MODE)
//	.update         = lcm_update,
//	.set_backlight	= lcm_setbacklight,
//	.set_pwm        = lcm_setpwm,
//	.get_pwm        = lcm_getpwm,
   
//	.esd_check   = lcm_esd_check,
// 	.esd_recover   = lcm_esd_recover,
	.compare_id    = lcm_compare_id,
//#endif
};
