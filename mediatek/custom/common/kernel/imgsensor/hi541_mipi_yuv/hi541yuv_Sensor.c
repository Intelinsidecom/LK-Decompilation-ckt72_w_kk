/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 *
 * Author:
 * -------
 *   Anyuan Huang (MTK70663)
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
  #include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/io.h>
#include <asm/system.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "hi541yuv_Sensor.h"
#include "hi541yuv_Camera_Sensor_para.h"
#include "hi541yuv_CameraCustomized.h" 

#define HI541YUV_DEBUG
#ifdef HI541YUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif
int zsd=0;
//extern int zsd ;
typedef enum
{
    PRV_W=1280,
    PRV_H=960
}PREVIEW_VIEW_SIZE;
typedef enum
{
    AE_SECTION_INDEX_BEGIN=0, 
    AE_SECTION_INDEX_1=AE_SECTION_INDEX_BEGIN, 
    AE_SECTION_INDEX_2, 
    AE_SECTION_INDEX_3, 
    AE_SECTION_INDEX_4, 
    AE_SECTION_INDEX_5, 
    AE_SECTION_INDEX_6, 
    AE_SECTION_INDEX_7, 
    AE_SECTION_INDEX_8, 
    AE_SECTION_INDEX_9, 
    AE_SECTION_INDEX_10, 
    AE_SECTION_INDEX_11, 
    AE_SECTION_INDEX_12, 
    AE_SECTION_INDEX_13, 
    AE_SECTION_INDEX_14, 
    AE_SECTION_INDEX_15, 
    AE_SECTION_INDEX_16,  
    AE_SECTION_INDEX_MAX
}AE_SECTION_INDEX;
typedef enum
{
    AE_VERTICAL_BLOCKS=4,
    AE_VERTICAL_BLOCKS_MAX,
    AE_HORIZONTAL_BLOCKS=4,
    AE_HORIZONTAL_BLOCKS_MAX
}AE_VERTICAL_HORIZONTAL_BLOCKS;
static UINT32 line_coordinate[AE_VERTICAL_BLOCKS_MAX] = {0};//line[0]=0      line[1]=160     line[2]=320     line[3]=480     line[4]=640
static UINT32 row_coordinate[AE_HORIZONTAL_BLOCKS_MAX] = {0};//line[0]=0       line[1]=120     line[2]=240     line[3]=360     line[4]=480
static BOOL AE_1_ARRAY[AE_SECTION_INDEX_MAX] = {FALSE};
static BOOL AE_2_ARRAY[AE_HORIZONTAL_BLOCKS][AE_VERTICAL_BLOCKS] = {{FALSE},{FALSE},{FALSE},{FALSE}};//how to ....
kal_uint16 af_xcoordinate = 0;
kal_uint16 af_ycoordinate = 0;

kal_uint32 hi541_exposuretime = 0;
kal_uint8  hi541_isospeed = AE_ISO_100;
kal_bool   hi541_strobe = 0;

struct
{
  kal_bool    NightMode;
  kal_bool   VideoMode;
  kal_uint8   ZoomFactor; /* Zoom Index */
  kal_uint16  Banding;
  kal_uint32  PvShutter;
  kal_uint32  CapShutter;
  kal_uint32  PvDummyPixels;
  kal_uint32  PvDummyLines;
  kal_uint32  CapDummyPixels;
  kal_uint32  CapDummyLines;
  kal_uint32  PvOpClk;
  kal_uint32  CapOpClk;
  
  /* Video frame rate 300 means 30.0fps. Unit Multiple 10. */
  kal_uint32  MaxFrameRate; 
  kal_uint32  MiniFrameRate; 
  /* Sensor Register backup. */
  kal_uint8   VDOCTL2; /* P0.0x11. */
  kal_uint8   ISPCTL3; /* P10.0x12. */
  kal_uint8   AECTL1;  /* P20.0x10. */
  kal_uint8   AWBCTL1; /* P22.0x10. */
  kal_uint16 ScenMode;
  kal_uint16 SensorMode;
  kal_uint32 currentExposureTime ;
  kal_uint32 currentAxDGain ;
} HI541Status;

typedef struct
{
  UINT16  iSensorVersion;
  UINT16  iNightMode;
  UINT16  iWB;
  UINT16  iEffect;
  UINT16  iEV;
  UINT16  iBanding;
  UINT16  iMirror;
  UINT16  iFrameRate;
  //0    :No-Fix FrameRate 
  //other:Fixed FrameRate
}HI541_Status;
HI541_Status HI541CurrentStatus;

static DEFINE_SPINLOCK(HI541_drv_lock);

static int CAPTURE_FLAG = 0;
static int CAPTURE_FLAG1 = 0;
// added sk hynix for touch AF
static int TOUCHAF_FLAG = 0;

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
extern int iBurstWriteReg(u8 *pData, u32 bytes, u16 i2cId) ;



#define Sleep(ms) mdelay(ms)


extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);


#define HI541WriteCmosSensor(addr, para) iWriteReg((u16) addr , (u32) para , 2, HI541_WRITE_ID)


kal_uint16 HI541ReadCmosSensor(kal_uint32 addr)
{
kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,HI541_WRITE_ID);
    return get_byte;
}


void HI541SetPage(kal_uint8 Page)
{
  HI541WriteCmosSensor(0x03, Page);
}





void HI541InitSetting(void)
{
#if 0
HI541WriteCmosSensor(0xffff, 0x0040);

#else
if (zsd==1)
	{return ;
}else{

//================================
//========= SYSTEM Start =========
//================================
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x0100,0x0000); //streaming disable
HI541WriteCmosSensor(0x0102,0x0001); //System SW Reset
HI541WriteCmosSensor(0x4814,0xff37); //MCU CLK enable
HI541WriteCmosSensor(0x4816,0x0100); //MIPI TX CLK enable
HI541WriteCmosSensor(0x4856,0x00F0); //Preventing parallel PAD toggling when output is JPEG+MIPI mode//PAD HiZ Disable
HI541WriteCmosSensor(0x4834,0x0401); //MIPI enable
HI541WriteCmosSensor(0x4808,0x190a); //PLL enable
HI541WriteCmosSensor(0xC41C,0x0100); //SSD INT enable
HI541WriteCmosSensor(0x5614,0x1f00); //TG INT enable
HI541WriteCmosSensor(0x2000,0x1b00); //I2CM PreScale
HI541WriteCmosSensor(0x2004,0x0000); //I2CM PreScale
HI541WriteCmosSensor(0x2008,0x8000); //I2CM enable for AF
HI541WriteCmosSensor(0x0106,0x0100); //fast_standby_ctrl
HI541WriteCmosSensor(0x0600,0x0000); //test_pattern_mode

HI541WriteCmosSensor(0x4838,0x0199); //mem_cfg2
HI541WriteCmosSensor(0x483a,0x0919); //mem_cfg3
HI541WriteCmosSensor(0x483c,0x9959); //mem_cfg5
HI541WriteCmosSensor(0x483e,0x9619); //mem_cfg7
HI541WriteCmosSensor(0x4840,0x5996); //mem_cfg9
HI541WriteCmosSensor(0x4842,0x9949); //mem_cfg11
HI541WriteCmosSensor(0x4844,0x6559); //mem_cfg13
HI541WriteCmosSensor(0x4846,0x9699); //mem_cfg15
HI541WriteCmosSensor(0x4848,0x9999); //mem_cfg17

HI541WriteCmosSensor(0xffff,0x0140);
HI541WriteCmosSensor(0x0004,0x0000); //GPIO DATAOUT
HI541WriteCmosSensor(0x0010,0xffff); //GPIO Output enable
HI541WriteCmosSensor(0x0018,0x05a0); //GPIO Altfunc = I2CM for AF
////SREG////
HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x4bec,0x1121);
HI541WriteCmosSensor(0x4bee,0x5e04);
HI541WriteCmosSensor(0x4bf0,0xcf23);
HI541WriteCmosSensor(0x4bf2,0x8c0a);//PCLK 42MHz
HI541WriteCmosSensor(0x4bf4,0x1837);
HI541WriteCmosSensor(0x4bf6,0xc000);
HI541WriteCmosSensor(0x4bf8,0x0008);
HI541WriteCmosSensor(0x4bfa,0x2401);
HI541WriteCmosSensor(0x4bfc,0x0000);
HI541WriteCmosSensor(0x4bfe,0x0000);

HI541WriteCmosSensor(0x1fe6,0x2018);
HI541WriteCmosSensor(0x1fe8,0x0404);
HI541WriteCmosSensor(0x1fea,0x9870);



HI541WriteCmosSensor(0xffff, 0x0020);
SENSORDB("HHL_enter Init AF pengliu 0x%04x\n", HI541ReadCmosSensor(0x0002));

if (0x00AE == HI541ReadCmosSensor(0x0002))
{

		HI541WriteCmosSensor(0x4530, 0x0A48);//FOR GT9760S
		HI541WriteCmosSensor(0x4532, 0x0949); 
HI541WriteCmosSensor(0x4534, 0x4160);
		HI541WriteCmosSensor(0x4536, 0x0B49); 
		HI541WriteCmosSensor(0x4538, 0x094A); 
HI541WriteCmosSensor(0x453A, 0x0A67);
		HI541WriteCmosSensor(0x453C, 0x0B4B); 
		HI541WriteCmosSensor(0x453E, 0x0A4A); 
HI541WriteCmosSensor(0x4540, 0x1A60);
		HI541WriteCmosSensor(0x4542, 0x0C4B); 
		HI541WriteCmosSensor(0x4544, 0x0A4A); 
HI541WriteCmosSensor(0x4546, 0x5A62);
		HI541WriteCmosSensor(0x4548, 0x0B4A); 
		HI541WriteCmosSensor(0x454A, 0x0260); 
		HI541WriteCmosSensor(0x454C, 0x0B48); 
		HI541WriteCmosSensor(0x454E, 0xC865); 
		HI541WriteCmosSensor(0x4550, 0x0C49); 
		HI541WriteCmosSensor(0x4552, 0x0B48); 
		HI541WriteCmosSensor(0x4554, 0x0863); 
		HI541WriteCmosSensor(0x4556, 0x7047); 
		HI541WriteCmosSensor(0x4558, 0x8945); 
		HI541WriteCmosSensor(0x455A, 0x0020); 
		HI541WriteCmosSensor(0x455C, 0x0041); 
		HI541WriteCmosSensor(0x455E, 0x0020); 
		HI541WriteCmosSensor(0x4560, 0xBB47); 
HI541WriteCmosSensor(0x4562, 0x0020); 
		HI541WriteCmosSensor(0x4564, 0x1C1B); 
HI541WriteCmosSensor(0x4566, 0x0020); 
		HI541WriteCmosSensor(0x4568, 0x2148); 
HI541WriteCmosSensor(0x456A, 0x0020); 
		HI541WriteCmosSensor(0x456C, 0x6040); 
HI541WriteCmosSensor(0x456E, 0x0020); 
		HI541WriteCmosSensor(0x4570, 0x1949); 
HI541WriteCmosSensor(0x4572, 0x0020); 
		HI541WriteCmosSensor(0x4574, 0x4805); 
HI541WriteCmosSensor(0x4576, 0x0020);
		HI541WriteCmosSensor(0x4578, 0x3149); 
HI541WriteCmosSensor(0x457A, 0x0020);
		HI541WriteCmosSensor(0x457C, 0x5549); 
HI541WriteCmosSensor(0x457E, 0x0020);
		HI541WriteCmosSensor(0x4580, 0x2D4A); 
HI541WriteCmosSensor(0x4582, 0x0020);
		HI541WriteCmosSensor(0x4584, 0x5C1C); 
HI541WriteCmosSensor(0x4586, 0x0020);
		HI541WriteCmosSensor(0x4588, 0xFEB5); 
		HI541WriteCmosSensor(0x458A, 0xF84C); 
		HI541WriteCmosSensor(0x458C, 0x0020); 
		HI541WriteCmosSensor(0x458E, 0xF849); 
		HI541WriteCmosSensor(0x4590, 0xF84B); 
		HI541WriteCmosSensor(0x4592, 0x6073); 
		HI541WriteCmosSensor(0x4594, 0x8200); 
		HI541WriteCmosSensor(0x4596, 0xD218); 
		HI541WriteCmosSensor(0x4598, 0x956B); 
		HI541WriteCmosSensor(0x459A, 0x8D42); 
		HI541WriteCmosSensor(0x459C, 0x00D9); 
		HI541WriteCmosSensor(0x459E, 0x9163); 
		HI541WriteCmosSensor(0x45A0, 0x156E); 
		HI541WriteCmosSensor(0x45A2, 0x8D42); 
		HI541WriteCmosSensor(0x45A4, 0x00D9); 
		HI541WriteCmosSensor(0x45A6, 0x1166); 
		HI541WriteCmosSensor(0x45A8, 0x401C); 
		HI541WriteCmosSensor(0x45AA, 0xC0B2); 
		HI541WriteCmosSensor(0x45AC, 0x0528); 
		HI541WriteCmosSensor(0x45AE, 0xF1D3); 
		HI541WriteCmosSensor(0x45B0, 0xF048); 
		HI541WriteCmosSensor(0x45B2, 0x6422); 
		HI541WriteCmosSensor(0x45B4, 0x416A); 
		HI541WriteCmosSensor(0x45B6, 0x806B); 
		HI541WriteCmosSensor(0x45B8, 0x5043); 
		HI541WriteCmosSensor(0x45BA, 0x00F0); 
		HI541WriteCmosSensor(0x45BC, 0xB1FA); 
		HI541WriteCmosSensor(0x45BE, 0xEB49); 
		HI541WriteCmosSensor(0x45C0, 0xFD23); 
		HI541WriteCmosSensor(0x45C2, 0xA031); 
		HI541WriteCmosSensor(0x45C4, 0x0291); 
		HI541WriteCmosSensor(0x45C6, 0x4E7E); 
		HI541WriteCmosSensor(0x45C8, 0xA039); 
		HI541WriteCmosSensor(0x45CA, 0x4A78); 
		HI541WriteCmosSensor(0x45CC, 0x0D46); 
		HI541WriteCmosSensor(0x45CE, 0x1A40); 
		HI541WriteCmosSensor(0x45D0, 0x0192); 
		HI541WriteCmosSensor(0x45D2, 0x1227); 
		HI541WriteCmosSensor(0x45D4, 0x0422); 
		HI541WriteCmosSensor(0x45D6, 0x6035); 
		HI541WriteCmosSensor(0x45D8, 0xB042); 
		HI541WriteCmosSensor(0x45DA, 0xCF5F); 
		HI541WriteCmosSensor(0x45DC, 0x8A56); 
		HI541WriteCmosSensor(0x45DE, 0xE548); 
		HI541WriteCmosSensor(0x45E0, 0x0092); 
		HI541WriteCmosSensor(0x45E2, 0x03D8); 
		HI541WriteCmosSensor(0x45E4, 0xC089); 
		HI541WriteCmosSensor(0x45E6, 0xE082); 
		HI541WriteCmosSensor(0x45E8, 0x0120); 
		HI541WriteCmosSensor(0x45EA, 0x4CE0); 
		HI541WriteCmosSensor(0x45EC, 0x816A); 
		HI541WriteCmosSensor(0x45EE, 0xC06B); 
		HI541WriteCmosSensor(0x45F0, 0x6422); 
		HI541WriteCmosSensor(0x45F2, 0x5043); 
		HI541WriteCmosSensor(0x45F4, 0x00F0); 
		HI541WriteCmosSensor(0x45F6, 0x94FA); 
		HI541WriteCmosSensor(0x45F8, 0xB042); 
		HI541WriteCmosSensor(0x45FA, 0xDE48); 
		HI541WriteCmosSensor(0x45FC, 0x03D8); 
		HI541WriteCmosSensor(0x45FE, 0x008A); 
		HI541WriteCmosSensor(0x4600, 0xE082); 
		HI541WriteCmosSensor(0x4602, 0x0220); 
		HI541WriteCmosSensor(0x4604, 0x3FE0); 
		HI541WriteCmosSensor(0x4606, 0x416B); 
		HI541WriteCmosSensor(0x4608, 0x806C); 
		HI541WriteCmosSensor(0x460A, 0x6422); 
		HI541WriteCmosSensor(0x460C, 0x5043); 
		HI541WriteCmosSensor(0x460E, 0x00F0); 
		HI541WriteCmosSensor(0x4610, 0x87FA); 
		HI541WriteCmosSensor(0x4612, 0xB042); 
		HI541WriteCmosSensor(0x4614, 0xD748); 
		HI541WriteCmosSensor(0x4616, 0x03D8); 
		HI541WriteCmosSensor(0x4618, 0xC08A); 
		HI541WriteCmosSensor(0x461A, 0xE082); 
		HI541WriteCmosSensor(0x461C, 0x0320); 
		HI541WriteCmosSensor(0x461E, 0x32E0); 
		HI541WriteCmosSensor(0x4620, 0xC16A); 
		HI541WriteCmosSensor(0x4622, 0x006C); 
		HI541WriteCmosSensor(0x4624, 0x6422); 
		HI541WriteCmosSensor(0x4626, 0x5043); 
		HI541WriteCmosSensor(0x4628, 0x00F0); 
		HI541WriteCmosSensor(0x462A, 0x7AFA); 
		HI541WriteCmosSensor(0x462C, 0xB042); 
		HI541WriteCmosSensor(0x462E, 0xD148); 
		HI541WriteCmosSensor(0x4630, 0x03D8); 
		HI541WriteCmosSensor(0x4632, 0x408A); 
		HI541WriteCmosSensor(0x4634, 0xE082); 
		HI541WriteCmosSensor(0x4636, 0x0420); 
		HI541WriteCmosSensor(0x4638, 0x25E0); 
		HI541WriteCmosSensor(0x463A, 0x016B); 
		HI541WriteCmosSensor(0x463C, 0x406C); 
		HI541WriteCmosSensor(0x463E, 0x6422); 
		HI541WriteCmosSensor(0x4640, 0x5043); 
		HI541WriteCmosSensor(0x4642, 0x00F0); 
		HI541WriteCmosSensor(0x4644, 0x6DFA); 
		HI541WriteCmosSensor(0x4646, 0xB042); 
		HI541WriteCmosSensor(0x4648, 0xCA48); 
		HI541WriteCmosSensor(0x464A, 0x03D8); 
		HI541WriteCmosSensor(0x464C, 0x808A); 
		HI541WriteCmosSensor(0x464E, 0xE082); 
		HI541WriteCmosSensor(0x4650, 0x0520); 
		HI541WriteCmosSensor(0x4652, 0x18E0); 
		HI541WriteCmosSensor(0x4654, 0x016D); 
		HI541WriteCmosSensor(0x4656, 0x406E); 
		HI541WriteCmosSensor(0x4658, 0x6422); 
		HI541WriteCmosSensor(0x465A, 0x5043); 
		HI541WriteCmosSensor(0x465C, 0x00F0); 
		HI541WriteCmosSensor(0x465E, 0x60FA); 
		HI541WriteCmosSensor(0x4660, 0xB042); 
		HI541WriteCmosSensor(0x4662, 0xC448); 
		HI541WriteCmosSensor(0x4664, 0x03D8); 
		HI541WriteCmosSensor(0x4666, 0x408B); 
		HI541WriteCmosSensor(0x4668, 0xE082); 
		HI541WriteCmosSensor(0x466A, 0x0720); 
		HI541WriteCmosSensor(0x466C, 0x0BE0); 
		HI541WriteCmosSensor(0x466E, 0xC16D); 
		HI541WriteCmosSensor(0x4670, 0x006F); 
		HI541WriteCmosSensor(0x4672, 0x6422); 
		HI541WriteCmosSensor(0x4674, 0x5043); 
		HI541WriteCmosSensor(0x4676, 0x00F0); 
		HI541WriteCmosSensor(0x4678, 0x53FA); 
		HI541WriteCmosSensor(0x467A, 0xB042); 
		HI541WriteCmosSensor(0x467C, 0x10D8); 
		HI541WriteCmosSensor(0x467E, 0xBD48); 
		HI541WriteCmosSensor(0x4680, 0x008C); 
		HI541WriteCmosSensor(0x4682, 0xE082); 
		HI541WriteCmosSensor(0x4684, 0x0820); 
		HI541WriteCmosSensor(0x4686, 0x6073); 
		HI541WriteCmosSensor(0x4688, 0x0198); 
		HI541WriteCmosSensor(0x468A, 0x6070); 
		HI541WriteCmosSensor(0x468C, 0x0320); 
		HI541WriteCmosSensor(0x468E, 0xE070); 
		HI541WriteCmosSensor(0x4690, 0xE08A); 
		HI541WriteCmosSensor(0x4692, 0x391A); 
		HI541WriteCmosSensor(0x4694, 0x0098); 
		HI541WriteCmosSensor(0x4696, 0x0818); 
		HI541WriteCmosSensor(0x4698, 0x00D5); 
		HI541WriteCmosSensor(0x469A, 0x4042); 
		HI541WriteCmosSensor(0x469C, 0xE882); 
		HI541WriteCmosSensor(0x469E, 0xFEBD); 
		HI541WriteCmosSensor(0x46A0, 0xA08C); 
		HI541WriteCmosSensor(0x46A2, 0x0121); 
		HI541WriteCmosSensor(0x46A4, 0xC902); 
		HI541WriteCmosSensor(0x46A6, 0x0843); 
		HI541WriteCmosSensor(0x46A8, 0xB349); 
		HI541WriteCmosSensor(0x46AA, 0x8843); 
		HI541WriteCmosSensor(0x46AC, 0xA084); 
		HI541WriteCmosSensor(0x46AE, 0x0298); 
		HI541WriteCmosSensor(0x46B0, 0x418B); 
		HI541WriteCmosSensor(0x46B2, 0x781A); 
		HI541WriteCmosSensor(0x46B4, 0x00D5); 
		HI541WriteCmosSensor(0x46B6, 0xC81B); 
		HI541WriteCmosSensor(0x46B8, 0x0099); 
		HI541WriteCmosSensor(0x46BA, 0x4018); 
		HI541WriteCmosSensor(0x46BC, 0xE882); 
		HI541WriteCmosSensor(0x46BE, 0x0198); 
		HI541WriteCmosSensor(0x46C0, 0x6070); 
		HI541WriteCmosSensor(0x46C2, 0x0320); 
		HI541WriteCmosSensor(0x46C4, 0xE070); 
		HI541WriteCmosSensor(0x46C6, 0xFEBD); 
		HI541WriteCmosSensor(0x46C8, 0xF8B5); 
		HI541WriteCmosSensor(0x46CA, 0xA84C); 
		HI541WriteCmosSensor(0x46CC, 0x0025); 
		HI541WriteCmosSensor(0x46CE, 0x8034); 
		HI541WriteCmosSensor(0x46D0, 0x6069); 
		HI541WriteCmosSensor(0x46D2, 0x6562); 
		HI541WriteCmosSensor(0x46D4, 0xA562); 
		HI541WriteCmosSensor(0x46D6, 0xA061); 
		HI541WriteCmosSensor(0x46D8, 0xA648); 
		HI541WriteCmosSensor(0x46DA, 0x0521); 
		HI541WriteCmosSensor(0x46DC, 0x7430); 
		HI541WriteCmosSensor(0x46DE, 0x00F0); 
		HI541WriteCmosSensor(0x46E0, 0x25FA); 
		HI541WriteCmosSensor(0x46E2, 0xA448); 
		HI541WriteCmosSensor(0x46E4, 0x0521); 
		HI541WriteCmosSensor(0x46E6, 0x8830); 
		HI541WriteCmosSensor(0x46E8, 0x00F0); 
		HI541WriteCmosSensor(0x46EA, 0x20FA); 
		HI541WriteCmosSensor(0x46EC, 0xA348); 
		HI541WriteCmosSensor(0x46EE, 0xC36B); 
		HI541WriteCmosSensor(0x46F0, 0x2246); 
		HI541WriteCmosSensor(0x46F2, 0x4032); 
		HI541WriteCmosSensor(0x46F4, 0x1069); 
		HI541WriteCmosSensor(0x46F6, 0x9F4E); 
		HI541WriteCmosSensor(0x46F8, 0x0103); 
		HI541WriteCmosSensor(0x46FA, 0x090F); 
		HI541WriteCmosSensor(0x46FC, 0x4B43); 
		HI541WriteCmosSensor(0x46FE, 0xF367); 
		HI541WriteCmosSensor(0x4700, 0x6362); 
		HI541WriteCmosSensor(0x4702, 0x9A4B); 
		HI541WriteCmosSensor(0x4704, 0x9B8C); 
		HI541WriteCmosSensor(0x4706, 0x5B06); 
		HI541WriteCmosSensor(0x4708, 0x45D4); 
		HI541WriteCmosSensor(0x470A, 0x9C49); 
		HI541WriteCmosSensor(0x470C, 0x4031); 
		HI541WriteCmosSensor(0x470E, 0xCF68); 
		HI541WriteCmosSensor(0x4710, 0x537D); 
		HI541WriteCmosSensor(0x4712, 0x3546); 
		HI541WriteCmosSensor(0x4714, 0x5F43); 
		HI541WriteCmosSensor(0x4716, 0x6F67); 
		HI541WriteCmosSensor(0x4718, 0x0E68); 
		HI541WriteCmosSensor(0x471A, 0xBC46); 
		HI541WriteCmosSensor(0x471C, 0x0502); 
		HI541WriteCmosSensor(0x471E, 0x2D0F); 
		HI541WriteCmosSensor(0x4720, 0x6E43); 
		HI541WriteCmosSensor(0x4722, 0x944F); 
		HI541WriteCmosSensor(0x4724, 0xED18); 
		HI541WriteCmosSensor(0x4726, 0xBE67); 
		HI541WriteCmosSensor(0x4728, 0x8B68); 
		HI541WriteCmosSensor(0x472A, 0x6644); 
		HI541WriteCmosSensor(0x472C, 0x070F); 
		HI541WriteCmosSensor(0x472E, 0xBC46); 
		HI541WriteCmosSensor(0x4730, 0x5F43); 
		HI541WriteCmosSensor(0x4732, 0x904B); 
		HI541WriteCmosSensor(0x4734, 0x6544); 
		HI541WriteCmosSensor(0x4736, 0x8033); 
		HI541WriteCmosSensor(0x4738, 0xBE19); 
		HI541WriteCmosSensor(0x473A, 0x5F60); 
		HI541WriteCmosSensor(0x473C, 0x4F68); 
		HI541WriteCmosSensor(0x473E, 0x0101); 
		HI541WriteCmosSensor(0x4740, 0x090F); 
		HI541WriteCmosSensor(0x4742, 0x4F43); 
		HI541WriteCmosSensor(0x4744, 0xBE19); 
		HI541WriteCmosSensor(0x4746, 0x4919); 
		HI541WriteCmosSensor(0x4748, 0x1F60); 
		HI541WriteCmosSensor(0x474A, 0x8C4D); 
		HI541WriteCmosSensor(0x474C, 0x6662); 
		HI541WriteCmosSensor(0x474E, 0xAE6B); 
		HI541WriteCmosSensor(0x4750, 0xC9B2); 
		HI541WriteCmosSensor(0x4752, 0x127D); 
		HI541WriteCmosSensor(0x4754, 0x5643); 
		HI541WriteCmosSensor(0x4756, 0x9E60); 
		HI541WriteCmosSensor(0x4758, 0xEF6A); 
		HI541WriteCmosSensor(0x475A, 0xB646); 
		HI541WriteCmosSensor(0x475C, 0x0606); 
		HI541WriteCmosSensor(0x475E, 0x360F); 
		HI541WriteCmosSensor(0x4760, 0x7743); 
		HI541WriteCmosSensor(0x4762, 0xB218); 
		HI541WriteCmosSensor(0x4764, 0xDF60); 
		HI541WriteCmosSensor(0x4766, 0x6E6B); 
		HI541WriteCmosSensor(0x4768, 0xBE44); 
		HI541WriteCmosSensor(0x476A, 0x0704); 
		HI541WriteCmosSensor(0x476C, 0x3F0F); 
		HI541WriteCmosSensor(0x476E, 0x7E43); 
		HI541WriteCmosSensor(0x4770, 0xBA18); 
		HI541WriteCmosSensor(0x4772, 0x9E61); 
		HI541WriteCmosSensor(0x4774, 0xAF6A); 
		HI541WriteCmosSensor(0x4776, 0xB644); 
		HI541WriteCmosSensor(0x4778, 0x0607); 
		HI541WriteCmosSensor(0x477A, 0x360F); 
		HI541WriteCmosSensor(0x477C, 0x7743); 
		HI541WriteCmosSensor(0x477E, 0x1F61); 
		HI541WriteCmosSensor(0x4780, 0x2D6B); 
		HI541WriteCmosSensor(0x4782, 0x7744); 
		HI541WriteCmosSensor(0x4784, 0xB218); 
		HI541WriteCmosSensor(0x4786, 0x0005); 
		HI541WriteCmosSensor(0x4788, 0x000F); 
		HI541WriteCmosSensor(0x478A, 0x4543); 
		HI541WriteCmosSensor(0x478C, 0x5D61); 
		HI541WriteCmosSensor(0x478E, 0xEB19); 
		HI541WriteCmosSensor(0x4790, 0x8018); 
		HI541WriteCmosSensor(0x4792, 0xC5B2); 
		HI541WriteCmosSensor(0x4794, 0xA362); 
		HI541WriteCmosSensor(0x4796, 0x0029); 
		HI541WriteCmosSensor(0x4798, 0x03D0); 
		HI541WriteCmosSensor(0x479A, 0x606A); 
		HI541WriteCmosSensor(0x479C, 0x00F0); 
		HI541WriteCmosSensor(0x479E, 0xC0F9); 
		HI541WriteCmosSensor(0x47A0, 0x6062); 
		HI541WriteCmosSensor(0x47A2, 0x002D); 
		HI541WriteCmosSensor(0x47A4, 0x04D0); 
		HI541WriteCmosSensor(0x47A6, 0x2946); 
		HI541WriteCmosSensor(0x47A8, 0xA06A); 
		HI541WriteCmosSensor(0x47AA, 0x00F0); 
		HI541WriteCmosSensor(0x47AC, 0xB9F9); 
		HI541WriteCmosSensor(0x47AE, 0xA062); 
		HI541WriteCmosSensor(0x47B0, 0xA16A); 
		HI541WriteCmosSensor(0x47B2, 0x606A); 
		HI541WriteCmosSensor(0x47B4, 0x4018); 
		HI541WriteCmosSensor(0x47B6, 0x6061); 
		HI541WriteCmosSensor(0x47B8, 0xF8BD); 
		HI541WriteCmosSensor(0x47BA, 0x10B5); 
		HI541WriteCmosSensor(0x47BC, 0x6B4C); 
		HI541WriteCmosSensor(0x47BE, 0x2078); 
		HI541WriteCmosSensor(0x47C0, 0xC107); 
		HI541WriteCmosSensor(0x47C2, 0x1CD0); 
		HI541WriteCmosSensor(0x47C4, 0xC009); 
		HI541WriteCmosSensor(0x47C6, 0x1AD0); 
		HI541WriteCmosSensor(0x47C8, 0xFFF7); 
		HI541WriteCmosSensor(0x47CA, 0x7EFF); 
		HI541WriteCmosSensor(0x47CC, 0xA178); 
		HI541WriteCmosSensor(0x47CE, 0x6C48); 
		HI541WriteCmosSensor(0x47D0, 0x0B00); 
		HI541WriteCmosSensor(0x47D2, 0x00F0); 
		HI541WriteCmosSensor(0x47D4, 0xB1F9); 
		HI541WriteCmosSensor(0x47D6, 0x0714); 
		HI541WriteCmosSensor(0x47D8, 0x0518); 
		HI541WriteCmosSensor(0x47DA, 0x1507); 
		HI541WriteCmosSensor(0x47DC, 0x1A18); 
		HI541WriteCmosSensor(0x47DE, 0x1400); 
		HI541WriteCmosSensor(0x47E0, 0x4069); 
		HI541WriteCmosSensor(0x47E2, 0x0EE0); 
HI541WriteCmosSensor(0x47E4, 0xC069); 
HI541WriteCmosSensor(0x47E6, 0x8047); 
		HI541WriteCmosSensor(0x47E8, 0x6048); 
		HI541WriteCmosSensor(0x47EA, 0x8030); 
		HI541WriteCmosSensor(0x47EC, 0xC16A); 
		HI541WriteCmosSensor(0x47EE, 0x4069); 
		HI541WriteCmosSensor(0x47F0, 0x8142); 
		HI541WriteCmosSensor(0x47F2, 0x04D1); 
		HI541WriteCmosSensor(0x47F4, 0x617C); 
		HI541WriteCmosSensor(0x47F6, 0x608A); 
		HI541WriteCmosSensor(0x47F8, 0x4900); 
		HI541WriteCmosSensor(0x47FA, 0x4018); 
		HI541WriteCmosSensor(0x47FC, 0xE082); 
		HI541WriteCmosSensor(0x47FE, 0x10BD); 
		HI541WriteCmosSensor(0x4800, 0xC069); 
		HI541WriteCmosSensor(0x4802, 0x8047); 
		HI541WriteCmosSensor(0x4804, 0x10BD); 
		HI541WriteCmosSensor(0x4806, 0x8069); 
		HI541WriteCmosSensor(0x4808, 0xFBE7); 
		HI541WriteCmosSensor(0x480A, 0x006A); 
		HI541WriteCmosSensor(0x480C, 0x8047); 
		HI541WriteCmosSensor(0x480E, 0x5748); 
		HI541WriteCmosSensor(0x4810, 0x1621); 
		HI541WriteCmosSensor(0x4812, 0x6030); 
		HI541WriteCmosSensor(0x4814, 0x415E); 
		HI541WriteCmosSensor(0x4816, 0x0029); 
		HI541WriteCmosSensor(0x4818, 0xF4DA); 
		HI541WriteCmosSensor(0x481A, 0x0021); 
		HI541WriteCmosSensor(0x481C, 0xC182); 
		HI541WriteCmosSensor(0x481E, 0x10BD); 
		HI541WriteCmosSensor(0x4820, 0xFEB5); 
		HI541WriteCmosSensor(0x4822, 0x584C); 
		HI541WriteCmosSensor(0x4824, 0x584F); 
		HI541WriteCmosSensor(0x4826, 0x594E); 
		HI541WriteCmosSensor(0x4828, 0x0025); 
		HI541WriteCmosSensor(0x482A, 0xE07E); 
		HI541WriteCmosSensor(0x482C, 0x0028); 
		HI541WriteCmosSensor(0x482E, 0x2BD0); 
		HI541WriteCmosSensor(0x4830, 0x0328); 
		HI541WriteCmosSensor(0x4832, 0x3ED0); 
		HI541WriteCmosSensor(0x4834, 0x627E); 
		HI541WriteCmosSensor(0x4836, 0x0628); 
		HI541WriteCmosSensor(0x4838, 0x54D0); 
		HI541WriteCmosSensor(0x483A, 0x207D); 
		HI541WriteCmosSensor(0x483C, 0xEA21); 
		HI541WriteCmosSensor(0x483E, 0x4843); 
		HI541WriteCmosSensor(0x4840, 0xC019); 
		HI541WriteCmosSensor(0x4842, 0x4119); 
		HI541WriteCmosSensor(0x4844, 0x6031); 
		HI541WriteCmosSensor(0x4846, 0xF368); 
		HI541WriteCmosSensor(0x4848, 0x497D); 
		HI541WriteCmosSensor(0x484A, 0x405D); 
HI541WriteCmosSensor(0x484C, 0x9847); 
		HI541WriteCmosSensor(0x484E, 0x0090); 
		HI541WriteCmosSensor(0x4850, 0x207D); 
		HI541WriteCmosSensor(0x4852, 0xEA21); 
		HI541WriteCmosSensor(0x4854, 0x4843); 
		HI541WriteCmosSensor(0x4856, 0xC019); 
		HI541WriteCmosSensor(0x4858, 0xFF21); 
		HI541WriteCmosSensor(0x485A, 0x4019); 
		HI541WriteCmosSensor(0x485C, 0x6031); 
		HI541WriteCmosSensor(0x485E, 0x095C); 
		HI541WriteCmosSensor(0x4860, 0xE030); 
		HI541WriteCmosSensor(0x4862, 0x627E); 
		HI541WriteCmosSensor(0x4864, 0x807A); 
		HI541WriteCmosSensor(0x4866, 0xF368); 
		HI541WriteCmosSensor(0x4868, 0x9847); 
		HI541WriteCmosSensor(0x486A, 0x0146); 
		HI541WriteCmosSensor(0x486C, 0xF368); 
		HI541WriteCmosSensor(0x486E, 0xE27D); 
		HI541WriteCmosSensor(0x4870, 0x0098); 
		HI541WriteCmosSensor(0x4872, 0x9847); 
		HI541WriteCmosSensor(0x4874, 0x7A19); 
		HI541WriteCmosSensor(0x4876, 0x1D21); 
		HI541WriteCmosSensor(0x4878, 0x4901); 
		HI541WriteCmosSensor(0x487A, 0x5118); 
		HI541WriteCmosSensor(0x487C, 0x6D1C); 
		HI541WriteCmosSensor(0x487E, 0xEDB2); 
		HI541WriteCmosSensor(0x4880, 0x0872); 
		HI541WriteCmosSensor(0x4882, 0x752D); 
		HI541WriteCmosSensor(0x4884, 0xD1D3); 
		HI541WriteCmosSensor(0x4886, 0xFEBD); 
		HI541WriteCmosSensor(0x4888, 0x627E); 
		HI541WriteCmosSensor(0x488A, 0x795D); 
		HI541WriteCmosSensor(0x488C, 0x6019); 
		HI541WriteCmosSensor(0x488E, 0x6123); 
		HI541WriteCmosSensor(0x4890, 0x5B01); 
		HI541WriteCmosSensor(0x4892, 0x0190); 
		HI541WriteCmosSensor(0x4894, 0xC018); 
		HI541WriteCmosSensor(0x4896, 0xF368); 
		HI541WriteCmosSensor(0x4898, 0x807D); 
		HI541WriteCmosSensor(0x489A, 0x9847); 
		HI541WriteCmosSensor(0x489C, 0x0090); 
		HI541WriteCmosSensor(0x489E, 0x7819); 
		HI541WriteCmosSensor(0x48A0, 0xE030); 
		HI541WriteCmosSensor(0x48A2, 0x817A); 
		HI541WriteCmosSensor(0x48A4, 0x5123); 
		HI541WriteCmosSensor(0x48A6, 0x0198); 
		HI541WriteCmosSensor(0x48A8, 0x5B01); 
		HI541WriteCmosSensor(0x48AA, 0xC018); 
		HI541WriteCmosSensor(0x48AC, 0x627E); 
		HI541WriteCmosSensor(0x48AE, 0xC07F); 
		HI541WriteCmosSensor(0x48B0, 0xD9E7); 
		HI541WriteCmosSensor(0x48B2, 0x627E); 
		HI541WriteCmosSensor(0x48B4, 0x7819); 
		HI541WriteCmosSensor(0x48B6, 0x0290); 
		HI541WriteCmosSensor(0x48B8, 0xE030); 
		HI541WriteCmosSensor(0x48BA, 0x817A); 
		HI541WriteCmosSensor(0x48BC, 0x6019); 
		HI541WriteCmosSensor(0x48BE, 0x5123); 
		HI541WriteCmosSensor(0x48C0, 0x5B01); 
		HI541WriteCmosSensor(0x48C2, 0x0190); 
		HI541WriteCmosSensor(0x48C4, 0xC018); 
		HI541WriteCmosSensor(0x48C6, 0xF368); 
		HI541WriteCmosSensor(0x48C8, 0xC07F); 
		HI541WriteCmosSensor(0x48CA, 0x9847); 
		HI541WriteCmosSensor(0x48CC, 0x0090); 
		HI541WriteCmosSensor(0x48CE, 0x0298); 
		HI541WriteCmosSensor(0x48D0, 0x2123); 
		HI541WriteCmosSensor(0x48D2, 0xFF30); 
		HI541WriteCmosSensor(0x48D4, 0xC130); 
		HI541WriteCmosSensor(0x48D6, 0x017D); 
		HI541WriteCmosSensor(0x48D8, 0x0198); 
		HI541WriteCmosSensor(0x48DA, 0x9B01); 
		HI541WriteCmosSensor(0x48DC, 0xC018); 
		HI541WriteCmosSensor(0x48DE, 0x627E); 
		HI541WriteCmosSensor(0x48E0, 0x007A); 
		HI541WriteCmosSensor(0x48E2, 0xC0E7); 
		HI541WriteCmosSensor(0x48E4, 0x7819); 
		HI541WriteCmosSensor(0x48E6, 0x0290); 
		HI541WriteCmosSensor(0x48E8, 0xFF30); 
		HI541WriteCmosSensor(0x48EA, 0xC130); 
		HI541WriteCmosSensor(0x48EC, 0x017D); 
		HI541WriteCmosSensor(0x48EE, 0x6019); 
		HI541WriteCmosSensor(0x48F0, 0x2123); 
HI541WriteCmosSensor(0x48F2, 0x9B01); 
		HI541WriteCmosSensor(0x48F4, 0x0190); 
		HI541WriteCmosSensor(0x48F6, 0xC018); 
		HI541WriteCmosSensor(0x48F8, 0xF368); 
		HI541WriteCmosSensor(0x48FA, 0x007A); 
		HI541WriteCmosSensor(0x48FC, 0x9847); 
		HI541WriteCmosSensor(0x48FE, 0x0090); 
		HI541WriteCmosSensor(0x4900, 0x1521); 
		HI541WriteCmosSensor(0x4902, 0x0298); 
		HI541WriteCmosSensor(0x4904, 0x4901); 
		HI541WriteCmosSensor(0x4906, 0x4018); 
		HI541WriteCmosSensor(0x4908, 0x817F); 
		HI541WriteCmosSensor(0x490A, 0x1923); 
		HI541WriteCmosSensor(0x490C, 0x0198); 
		HI541WriteCmosSensor(0x490E, 0x9B01); 
		HI541WriteCmosSensor(0x4910, 0xC018); 
		HI541WriteCmosSensor(0x4912, 0x627E); 
		HI541WriteCmosSensor(0x4914, 0x407C); 
		HI541WriteCmosSensor(0x4916, 0xA6E7); 
		HI541WriteCmosSensor(0x4918, 0x10B5); 
		HI541WriteCmosSensor(0x491A, 0x00F0); 
		HI541WriteCmosSensor(0x491C, 0x13F9); 
		HI541WriteCmosSensor(0x491E, 0x1C48); 
		HI541WriteCmosSensor(0x4920, 0x016F); 
		HI541WriteCmosSensor(0x4922, 0x1C48); 
		HI541WriteCmosSensor(0x4924, 0xC27B); 
		HI541WriteCmosSensor(0x4926, 0x9142); 
		HI541WriteCmosSensor(0x4928, 0x01D9); 
		HI541WriteCmosSensor(0x492A, 0x817B); 
		HI541WriteCmosSensor(0x492C, 0x0173); 
		HI541WriteCmosSensor(0x492E, 0x10BD); 
		HI541WriteCmosSensor(0x4930, 0x10B5); 
		HI541WriteCmosSensor(0x4932, 0x00F0); 
		HI541WriteCmosSensor(0x4934, 0x0DF9); 
		HI541WriteCmosSensor(0x4936, 0x0F4A); 
		HI541WriteCmosSensor(0x4938, 0x0020); 
		HI541WriteCmosSensor(0x493A, 0x8100); 
		HI541WriteCmosSensor(0x493C, 0x8918); 
		HI541WriteCmosSensor(0x493E, 0x8823); 
		HI541WriteCmosSensor(0x4940, 0x5B58); 
		HI541WriteCmosSensor(0x4942, 0x0C6E); 
		HI541WriteCmosSensor(0x4944, 0xA342); 
		HI541WriteCmosSensor(0x4946, 0x00D2); 
		HI541WriteCmosSensor(0x4948, 0x0B66); 
		HI541WriteCmosSensor(0x494A, 0x401C); 
		HI541WriteCmosSensor(0x494C, 0xC0B2); 
		HI541WriteCmosSensor(0x494E, 0x0528); 
		HI541WriteCmosSensor(0x4950, 0xF3D3); 
		HI541WriteCmosSensor(0x4952, 0x10BD); 
		HI541WriteCmosSensor(0x4954, 0xF8B5); 
		HI541WriteCmosSensor(0x4956, 0x00F0); 
		HI541WriteCmosSensor(0x4958, 0x01F9); 
		HI541WriteCmosSensor(0x495A, 0x0E48); 
		HI541WriteCmosSensor(0x495C, 0x2038); 
		HI541WriteCmosSensor(0x495E, 0x816D); 
		HI541WriteCmosSensor(0x4960, 0xC06D); 
		HI541WriteCmosSensor(0x4962, 0x8142); 
		HI541WriteCmosSensor(0x4964, 0x18D9); 
		HI541WriteCmosSensor(0x4966, 0x0121); 
		HI541WriteCmosSensor(0x4968, 0x18E0); 
		HI541WriteCmosSensor(0x496A, 0x0000); 
		HI541WriteCmosSensor(0x496C, 0xB005); 
		HI541WriteCmosSensor(0x496E, 0x0020); 
		HI541WriteCmosSensor(0x4970, 0x285C); 
		HI541WriteCmosSensor(0x4972, 0x8F02); 
		HI541WriteCmosSensor(0x4974, 0x6440); 
		HI541WriteCmosSensor(0x4976, 0x0020); 
		HI541WriteCmosSensor(0x4978, 0x0230); 
		HI541WriteCmosSensor(0x497A, 0x0000); 
		HI541WriteCmosSensor(0x497C, 0x00CC); 
		HI541WriteCmosSensor(0x497E, 0x0040); 
		HI541WriteCmosSensor(0x4980, 0xC007); 
		HI541WriteCmosSensor(0x4982, 0x0020); 
		HI541WriteCmosSensor(0x4984, 0xB00D); 
		HI541WriteCmosSensor(0x4986, 0x0020); 
		HI541WriteCmosSensor(0x4988, 0x1041); 
		HI541WriteCmosSensor(0x498A, 0x0020); 
		HI541WriteCmosSensor(0x498C, 0x1C1B); 
		HI541WriteCmosSensor(0x498E, 0x0020); 
		HI541WriteCmosSensor(0x4990, 0x4400); 
		HI541WriteCmosSensor(0x4992, 0x0020); 
		HI541WriteCmosSensor(0x4994, 0x0003); 
HI541WriteCmosSensor(0x4996, 0x0020); 
		HI541WriteCmosSensor(0x4998, 0x0021); 
		HI541WriteCmosSensor(0x499A, 0xC943); 
		HI541WriteCmosSensor(0x499C, 0x5748); 
		HI541WriteCmosSensor(0x499E, 0x564A); 
		HI541WriteCmosSensor(0x49A0, 0x048A); 
		HI541WriteCmosSensor(0x49A2, 0x1379); 
		HI541WriteCmosSensor(0x49A4, 0x5648); 
		HI541WriteCmosSensor(0x49A6, 0x6343); 
		HI541WriteCmosSensor(0x49A8, 0x8569); 
		HI541WriteCmosSensor(0x49AA, 0x4B43); 
		HI541WriteCmosSensor(0x49AC, 0x5D19); 
		HI541WriteCmosSensor(0x49AE, 0x554B); 
		HI541WriteCmosSensor(0x49B0, 0x1646); 
		HI541WriteCmosSensor(0x49B2, 0xDB68); 
		HI541WriteCmosSensor(0x49B4, 0x403E); 
		HI541WriteCmosSensor(0x49B6, 0x9D42); 
		HI541WriteCmosSensor(0x49B8, 0x08D8); 
		HI541WriteCmosSensor(0x49BA, 0x0120); 
		HI541WriteCmosSensor(0x49BC, 0xF073); 
		HI541WriteCmosSensor(0x49BE, 0x5148); 
		HI541WriteCmosSensor(0x49C0, 0x5149); 
		HI541WriteCmosSensor(0x49C2, 0x2030); 
		HI541WriteCmosSensor(0x49C4, 0xC27B); 
		HI541WriteCmosSensor(0x49C6, 0x8A76); 
		HI541WriteCmosSensor(0x49C8, 0x407E); 
		HI541WriteCmosSensor(0x49CA, 0x1EE0); 
		HI541WriteCmosSensor(0x49CC, 0x5579); 
		HI541WriteCmosSensor(0x49CE, 0xC769); 
		HI541WriteCmosSensor(0x49D0, 0x6543); 
		HI541WriteCmosSensor(0x49D2, 0x4D43); 
		HI541WriteCmosSensor(0x49D4, 0xED19); 
		HI541WriteCmosSensor(0x49D6, 0x9D42); 
		HI541WriteCmosSensor(0x49D8, 0x08D8); 
		HI541WriteCmosSensor(0x49DA, 0x0220); 
		HI541WriteCmosSensor(0x49DC, 0xF073); 
		HI541WriteCmosSensor(0x49DE, 0x4A48); 
		HI541WriteCmosSensor(0x49E0, 0x9178); 
		HI541WriteCmosSensor(0x49E2, 0x8176); 
		HI541WriteCmosSensor(0x49E4, 0x4449); 
		HI541WriteCmosSensor(0x49E6, 0x2031); 
		HI541WriteCmosSensor(0x49E8, 0x4978); 
		HI541WriteCmosSensor(0x49EA, 0x1DE0); 
		HI541WriteCmosSensor(0x49EC, 0x9579); 
		HI541WriteCmosSensor(0x49EE, 0x076A); 
		HI541WriteCmosSensor(0x49F0, 0x6543); 
		HI541WriteCmosSensor(0x49F2, 0x4D43); 
		HI541WriteCmosSensor(0x49F4, 0xED19); 
		HI541WriteCmosSensor(0x49F6, 0x9D42); 
		HI541WriteCmosSensor(0x49F8, 0x09D8); 
		HI541WriteCmosSensor(0x49FA, 0x0321); 
		HI541WriteCmosSensor(0x49FC, 0xF173); 
		HI541WriteCmosSensor(0x49FE, 0x4049); 
		HI541WriteCmosSensor(0x4A00, 0x6039); 
		HI541WriteCmosSensor(0x4A02, 0x4A78); 
		HI541WriteCmosSensor(0x4A04, 0x4049); 
		HI541WriteCmosSensor(0x4A06, 0x8A76); 
		HI541WriteCmosSensor(0x4A08, 0xC07A); 
		HI541WriteCmosSensor(0x4A0A, 0xC876); 
		HI541WriteCmosSensor(0x4A0C, 0xF8BD); 
		HI541WriteCmosSensor(0x4A0E, 0xD279); 
		HI541WriteCmosSensor(0x4A10, 0x6243); 
		HI541WriteCmosSensor(0x4A12, 0x4A43); 
		HI541WriteCmosSensor(0x4A14, 0x416A); 
		HI541WriteCmosSensor(0x4A16, 0x5118); 
		HI541WriteCmosSensor(0x4A18, 0x9942); 
		HI541WriteCmosSensor(0x4A1A, 0xF7D8); 
		HI541WriteCmosSensor(0x4A1C, 0x0421); 
		HI541WriteCmosSensor(0x4A1E, 0xF173); 
		HI541WriteCmosSensor(0x4A20, 0xC189); 
		HI541WriteCmosSensor(0x4A22, 0x3948); 
		HI541WriteCmosSensor(0x4A24, 0x0A0A); 
		HI541WriteCmosSensor(0x4A26, 0x8276); 
		HI541WriteCmosSensor(0x4A28, 0xC176); 
		HI541WriteCmosSensor(0x4A2A, 0xF8BD); 
		HI541WriteCmosSensor(0x4A2C, 0xFEB5); 
		HI541WriteCmosSensor(0x4A2E, 0x374C); 
		HI541WriteCmosSensor(0x4A30, 0xE078); 
		HI541WriteCmosSensor(0x4A32, 0x2546); 
		HI541WriteCmosSensor(0x4A34, 0xC107); 
		HI541WriteCmosSensor(0x4A36, 0x0120); 
		HI541WriteCmosSensor(0x4A38, 0x8035); 
		HI541WriteCmosSensor(0x4A3A, 0x0029); 
		HI541WriteCmosSensor(0x4A3C, 0x04D0); 
		HI541WriteCmosSensor(0x4A3E, 0x3449); 
		HI541WriteCmosSensor(0x4A40, 0x497F); 
		HI541WriteCmosSensor(0x4A42, 0x0140); 
		HI541WriteCmosSensor(0x4A44, 0xE972); 
		HI541WriteCmosSensor(0x4A46, 0x00E0); 
		HI541WriteCmosSensor(0x4A48, 0xE872); 
		HI541WriteCmosSensor(0x4A4A, 0x0021); 
		HI541WriteCmosSensor(0x4A4C, 0x2973); 
		HI541WriteCmosSensor(0x4A4E, 0xA278); 
		HI541WriteCmosSensor(0x4A50, 0x304E); 
		HI541WriteCmosSensor(0x4A52, 0xD306); 
		HI541WriteCmosSensor(0x4A54, 0x327D); 
		HI541WriteCmosSensor(0x4A56, 0x07D5); 
		HI541WriteCmosSensor(0x4A58, 0x284F); 
		HI541WriteCmosSensor(0x4A5A, 0x2F4B); 
		HI541WriteCmosSensor(0x4A5C, 0xC03F); 
		HI541WriteCmosSensor(0x4A5E, 0xDB69); 
		HI541WriteCmosSensor(0x4A60, 0x3F6C); 
		HI541WriteCmosSensor(0x4A62, 0xBB42); 
		HI541WriteCmosSensor(0x4A64, 0x03D1); 
		HI541WriteCmosSensor(0x4A66, 0x03E0); 
		HI541WriteCmosSensor(0x4A68, 0x637E); 
		HI541WriteCmosSensor(0x4A6A, 0x9A42); 
		HI541WriteCmosSensor(0x4A6C, 0x00D8); 
		HI541WriteCmosSensor(0x4A6E, 0x2873); 
		HI541WriteCmosSensor(0x4A70, 0x264F); 
		HI541WriteCmosSensor(0x4A72, 0x8023); 
		HI541WriteCmosSensor(0x4A74, 0x2037); 
		HI541WriteCmosSensor(0x4A76, 0x0297); 
		HI541WriteCmosSensor(0x4A78, 0xBB77); 
		HI541WriteCmosSensor(0x4A7A, 0x2F7B); 
		HI541WriteCmosSensor(0x4A7C, 0x012F); 
		HI541WriteCmosSensor(0x4A7E, 0x3AD1); 
		HI541WriteCmosSensor(0x4A80, 0x2779); 
		HI541WriteCmosSensor(0x4A82, 0x3F06); 
		HI541WriteCmosSensor(0x4A84, 0x06D4); 
		HI541WriteCmosSensor(0x4A86, 0x402A); 
		HI541WriteCmosSensor(0x4A88, 0x02D9); 
		HI541WriteCmosSensor(0x4A8A, 0xE175); 
		HI541WriteCmosSensor(0x4A8C, 0x2376); 
		HI541WriteCmosSensor(0x4A8E, 0x01E0); 
		HI541WriteCmosSensor(0x4A90, 0xE075); 
		HI541WriteCmosSensor(0x4A92, 0x2176); 
		HI541WriteCmosSensor(0x4A94, 0x214F); 
		HI541WriteCmosSensor(0x4A96, 0x786B); 
		HI541WriteCmosSensor(0x4A98, 0x8047); 
		HI541WriteCmosSensor(0x4A9A, 0xE06D); 
		HI541WriteCmosSensor(0x4A9C, 0x00F0); 
		HI541WriteCmosSensor(0x4A9E, 0x64F8); 
		HI541WriteCmosSensor(0x4AA0, 0x0190); 
		HI541WriteCmosSensor(0x4AA2, 0x1D48); 
		HI541WriteCmosSensor(0x4AA4, 0xC069); 
		HI541WriteCmosSensor(0x4AA6, 0x0090); 
HI541WriteCmosSensor(0x4AA8, 0x00F0); 
		HI541WriteCmosSensor(0x4AAA, 0x5EF8); 
		HI541WriteCmosSensor(0x4AAC, 0x0199); 
		HI541WriteCmosSensor(0x4AAE, 0x00F0); 
		HI541WriteCmosSensor(0x4AB0, 0x61F8); 
		HI541WriteCmosSensor(0x4AB2, 0x7F21); 
		HI541WriteCmosSensor(0x4AB4, 0xC905); 
		HI541WriteCmosSensor(0x4AB6, 0x6860); 
		HI541WriteCmosSensor(0x4AB8, 0x8842); 
		HI541WriteCmosSensor(0x4ABA, 0x02DC); 
		HI541WriteCmosSensor(0x4ABC, 0x2179); 
		HI541WriteCmosSensor(0x4ABE, 0x0906); 
		HI541WriteCmosSensor(0x4AC0, 0x12D4); 
		HI541WriteCmosSensor(0x4AC2, 0xA178); 
		HI541WriteCmosSensor(0x4AC4, 0x0906); 
		HI541WriteCmosSensor(0x4AC6, 0x04D5); 
		HI541WriteCmosSensor(0x4AC8, 0xB96B); 
		HI541WriteCmosSensor(0x4ACA, 0x8847); 
		HI541WriteCmosSensor(0x4ACC, 0xA078); 
		HI541WriteCmosSensor(0x4ACE, 0x0006); 
		HI541WriteCmosSensor(0x4AD0, 0x02D4); 
		HI541WriteCmosSensor(0x4AD2, 0xF96B); 
		HI541WriteCmosSensor(0x4AD4, 0x6868); 
HI541WriteCmosSensor(0x4AD6, 0x8847); 
		HI541WriteCmosSensor(0x4AD8, 0xF96C); 
		HI541WriteCmosSensor(0x4ADA, 0x6868); 
		HI541WriteCmosSensor(0x4ADC, 0x8847); 
		HI541WriteCmosSensor(0x4ADE, 0x00F0); 
		HI541WriteCmosSensor(0x4AE0, 0x4FF8); 
		HI541WriteCmosSensor(0x4AE2, 0x0299); 
		HI541WriteCmosSensor(0x4AE4, 0x8877); 
		HI541WriteCmosSensor(0x4AE6, 0x01E0); 
		HI541WriteCmosSensor(0x4AE8, 0x0098); 
		HI541WriteCmosSensor(0x4AEA, 0xE065); 
		HI541WriteCmosSensor(0x4AEC, 0x317D); 
		HI541WriteCmosSensor(0x4AEE, 0xB07D); 
		HI541WriteCmosSensor(0x4AF0, 0x8142); 
		HI541WriteCmosSensor(0x4AF2, 0x00D2); 
		HI541WriteCmosSensor(0x4AF4, 0x3075); 
		HI541WriteCmosSensor(0x4AF6, 0xFEBD); 
		HI541WriteCmosSensor(0x4AF8, 0x9012); 
		HI541WriteCmosSensor(0x4AFA, 0x0020); 
		HI541WriteCmosSensor(0x4AFC, 0xA003); 
		HI541WriteCmosSensor(0x4AFE, 0x0020); 
		HI541WriteCmosSensor(0x4B00, 0xF01A); 
		HI541WriteCmosSensor(0x4B02, 0x0020); 
		HI541WriteCmosSensor(0x4B04, 0xB00D); 
HI541WriteCmosSensor(0x4B06, 0x0020); 
HI541WriteCmosSensor(0x4B08, 0x807C); 
HI541WriteCmosSensor(0x4B0A, 0x0040); 
		HI541WriteCmosSensor(0x4B0C, 0xA01B); 
HI541WriteCmosSensor(0x4B0E, 0x0020); 
		HI541WriteCmosSensor(0x4B10, 0x40CC); 
		HI541WriteCmosSensor(0x4B12, 0x0040); 
		HI541WriteCmosSensor(0x4B14, 0x001C); 
HI541WriteCmosSensor(0x4B16, 0x0020); 
		HI541WriteCmosSensor(0x4B18, 0x381C); 
		HI541WriteCmosSensor(0x4B1A, 0x0020); 
		HI541WriteCmosSensor(0x4B1C, 0x5C1C); 
HI541WriteCmosSensor(0x4B1E, 0x0020); 
		HI541WriteCmosSensor(0x4B20, 0x03B4); 
		HI541WriteCmosSensor(0x4B22, 0x0148); 
		HI541WriteCmosSensor(0x4B24, 0x0190); 
		HI541WriteCmosSensor(0x4B26, 0x01BD); 
		HI541WriteCmosSensor(0x4B28, 0xF1DA); 
		HI541WriteCmosSensor(0x4B2A, 0x0000); 
HI541WriteCmosSensor(0x4B2C, 0x03B4); 
HI541WriteCmosSensor(0x4B2E, 0x0148); 
HI541WriteCmosSensor(0x4B30, 0x0190); 
HI541WriteCmosSensor(0x4B32, 0x01BD); 
		HI541WriteCmosSensor(0x4B34, 0xAF6F); 
HI541WriteCmosSensor(0x4B36, 0x0000); 
HI541WriteCmosSensor(0x4B38, 0x03B4); 
HI541WriteCmosSensor(0x4B3A, 0x0148); 
HI541WriteCmosSensor(0x4B3C, 0x0190); 
HI541WriteCmosSensor(0x4B3E, 0x01BD); 
		HI541WriteCmosSensor(0x4B40, 0x4FE9); 
HI541WriteCmosSensor(0x4B42, 0x0000); 
HI541WriteCmosSensor(0x4B44, 0x03B4); 
HI541WriteCmosSensor(0x4B46, 0x0148); 
HI541WriteCmosSensor(0x4B48, 0x0190); 
HI541WriteCmosSensor(0x4B4A, 0x01BD); 
		HI541WriteCmosSensor(0x4B4C, 0xAF3C); 
HI541WriteCmosSensor(0x4B4E, 0x0000); 
HI541WriteCmosSensor(0x4B50, 0x03B4); 
HI541WriteCmosSensor(0x4B52, 0x0148); 
HI541WriteCmosSensor(0x4B54, 0x0190); 
HI541WriteCmosSensor(0x4B56, 0x01BD); 
		HI541WriteCmosSensor(0x4B58, 0xCDBC); 
HI541WriteCmosSensor(0x4B5A, 0x0000); 
HI541WriteCmosSensor(0x4B5C, 0x03B4); 
HI541WriteCmosSensor(0x4B5E, 0x0148); 
HI541WriteCmosSensor(0x4B60, 0x0190); 
HI541WriteCmosSensor(0x4B62, 0x01BD); 
		HI541WriteCmosSensor(0x4B64, 0x2F94); 
HI541WriteCmosSensor(0x4B66, 0x0000); 
HI541WriteCmosSensor(0x4B68, 0x03B4); 
HI541WriteCmosSensor(0x4B6A, 0x0148); 
HI541WriteCmosSensor(0x4B6C, 0x0190); 
HI541WriteCmosSensor(0x4B6E, 0x01BD); 
		HI541WriteCmosSensor(0x4B70, 0xA1E7); 
HI541WriteCmosSensor(0x4B72, 0x0000); 
HI541WriteCmosSensor(0x4B74, 0x03B4); 
HI541WriteCmosSensor(0x4B76, 0x0148); 
HI541WriteCmosSensor(0x4B78, 0x0190); 
HI541WriteCmosSensor(0x4B7A, 0x01BD); 
		HI541WriteCmosSensor(0x4B7C, 0x6DE5); 
HI541WriteCmosSensor(0x4B7E, 0x0000); 
HI541WriteCmosSensor(0x4B80, 0x03B4); 
HI541WriteCmosSensor(0x4B82, 0x0148); 
HI541WriteCmosSensor(0x4B84, 0x0190); 
HI541WriteCmosSensor(0x4B86, 0x01BD); 
		HI541WriteCmosSensor(0x4B88, 0x19E7); 
HI541WriteCmosSensor(0x4B8A, 0x0000); 


}

else {


		HI541WriteCmosSensor(0x3A00, 0x0548);
		HI541WriteCmosSensor(0x3A02, 0x0449);
		HI541WriteCmosSensor(0x3A04, 0x4161);
		HI541WriteCmosSensor(0x3A06, 0x064A);
		HI541WriteCmosSensor(0x3A08, 0x0449);
		HI541WriteCmosSensor(0x3A0A, 0x1167);
		HI541WriteCmosSensor(0x3A0C, 0x0549);
		HI541WriteCmosSensor(0x3A0E, 0xC160);
		HI541WriteCmosSensor(0x3A10, 0x7047);
		HI541WriteCmosSensor(0x3A12, 0x0000);
		HI541WriteCmosSensor(0x3A14, 0x293A);
		HI541WriteCmosSensor(0x3A16, 0x0020);
		HI541WriteCmosSensor(0x3A18, 0x4805);
		HI541WriteCmosSensor(0x3A1A, 0x0020);
		HI541WriteCmosSensor(0x3A1C, 0x933B);
		HI541WriteCmosSensor(0x3A1E, 0x0020);
		HI541WriteCmosSensor(0x3A20, 0x9C2C);
		HI541WriteCmosSensor(0x3A22, 0x0020);
		HI541WriteCmosSensor(0x3A24, 0xB33B);
		HI541WriteCmosSensor(0x3A26, 0x0020);
		HI541WriteCmosSensor(0x3A28, 0xF0B5);
		HI541WriteCmosSensor(0x3A2A, 0x734D);
		HI541WriteCmosSensor(0x3A2C, 0x87B0);
		HI541WriteCmosSensor(0x3A2E, 0x0026);
		HI541WriteCmosSensor(0x3A30, 0x2846);
		HI541WriteCmosSensor(0x3A32, 0xEE74);
		HI541WriteCmosSensor(0x3A34, 0xA038);
		HI541WriteCmosSensor(0x3A36, 0xAE74);
		HI541WriteCmosSensor(0x3A38, 0x0179);
		HI541WriteCmosSensor(0x3A3A, 0x0446);
		HI541WriteCmosSensor(0x3A3C, 0x2046);
		HI541WriteCmosSensor(0x3A3E, 0xFF30);
		HI541WriteCmosSensor(0x3A40, 0x4130);
		HI541WriteCmosSensor(0x3A42, 0xC034);
		HI541WriteCmosSensor(0x3A44, 0x0691);
		HI541WriteCmosSensor(0x3A46, 0x0090);
		HI541WriteCmosSensor(0x3A48, 0xE07C);
		HI541WriteCmosSensor(0x3A4A, 0x0590);
		HI541WriteCmosSensor(0x3A4C, 0xA07C);
		HI541WriteCmosSensor(0x3A4E, 0x0490);
		HI541WriteCmosSensor(0x3A50, 0x0129);
		HI541WriteCmosSensor(0x3A52, 0x12D1);
		HI541WriteCmosSensor(0x3A54, 0x00F0);
		HI541WriteCmosSensor(0x3A56, 0xDEF8);
		HI541WriteCmosSensor(0x3A58, 0x8327);
		HI541WriteCmosSensor(0x3A5A, 0x0146);
		HI541WriteCmosSensor(0x3A5C, 0xFF05);
		HI541WriteCmosSensor(0x3A5E, 0x3846);
		HI541WriteCmosSensor(0x3A60, 0x00F0);
		HI541WriteCmosSensor(0x3A62, 0xDEF8);
		HI541WriteCmosSensor(0x3A64, 0x0099);
		HI541WriteCmosSensor(0x3A66, 0xC861);
		HI541WriteCmosSensor(0x3A68, 0x0598);
		HI541WriteCmosSensor(0x3A6A, 0x00F0);
		HI541WriteCmosSensor(0x3A6C, 0xD3F8);
		HI541WriteCmosSensor(0x3A6E, 0x0146);
		HI541WriteCmosSensor(0x3A70, 0x3846);
		HI541WriteCmosSensor(0x3A72, 0x00F0);
		HI541WriteCmosSensor(0x3A74, 0xD5F8);
		HI541WriteCmosSensor(0x3A76, 0x0099);
		HI541WriteCmosSensor(0x3A78, 0x0862);
		HI541WriteCmosSensor(0x3A7A, 0x5F48);
		HI541WriteCmosSensor(0x3A7C, 0x417F);
		HI541WriteCmosSensor(0x3A7E, 0x0909);
		HI541WriteCmosSensor(0x3A80, 0x0391);
		HI541WriteCmosSensor(0x3A82, 0xC07F);
		HI541WriteCmosSensor(0x3A84, 0x0290);
		HI541WriteCmosSensor(0x3A86, 0x56E0);
		HI541WriteCmosSensor(0x3A88, 0x0020);
		HI541WriteCmosSensor(0x3A8A, 0xE874);
		HI541WriteCmosSensor(0x3A8C, 0x4CE0);
		HI541WriteCmosSensor(0x3A8E, 0x0698);
		HI541WriteCmosSensor(0x3A90, 0x0128);
		HI541WriteCmosSensor(0x3A92, 0x1AD0);
		HI541WriteCmosSensor(0x3A94, 0x0199);
		HI541WriteCmosSensor(0x3A96, 0x3E46);
		HI541WriteCmosSensor(0x3A98, 0xF007);
		HI541WriteCmosSensor(0x3A9A, 0x2BD0);
		HI541WriteCmosSensor(0x3A9C, 0xC800);
		HI541WriteCmosSensor(0x3A9E, 0x7208);
		HI541WriteCmosSensor(0x3AA0, 0x8218);
		HI541WriteCmosSensor(0x3AA2, 0x5648);
		HI541WriteCmosSensor(0x3AA4, 0x805C);
		HI541WriteCmosSensor(0x3AA6, 0x0007);

		HI541WriteCmosSensor(0x3AA8, 0x000F);
		HI541WriteCmosSensor(0x3AAA, 0x039A);
		HI541WriteCmosSensor(0x3AAC, 0x5207);
		HI541WriteCmosSensor(0x3AAE, 0x28D5);
		HI541WriteCmosSensor(0x3AB0, 0x0A01);
		HI541WriteCmosSensor(0x3AB2, 0x5349);
		HI541WriteCmosSensor(0x3AB4, 0x5118);
		HI541WriteCmosSensor(0x3AB6, 0x895D);
		HI541WriteCmosSensor(0x3AB8, 0x029A);
		HI541WriteCmosSensor(0x3ABA, 0x5143);
		HI541WriteCmosSensor(0x3ABC, 0x2268);
		HI541WriteCmosSensor(0x3ABE, 0x4143);
		HI541WriteCmosSensor(0x3AC0, 0x8918);
		HI541WriteCmosSensor(0x3AC2, 0x2160);
		HI541WriteCmosSensor(0x3AC4, 0xA188);
		HI541WriteCmosSensor(0x3AC6, 0x0818);
		HI541WriteCmosSensor(0x3AC8, 0x2BE0);
		HI541WriteCmosSensor(0x3ACA, 0x3846);
		HI541WriteCmosSensor(0x3ACC, 0x00F0);
		HI541WriteCmosSensor(0x3ACE, 0xA2F8);
		HI541WriteCmosSensor(0x3AD0, 0x0099);
		HI541WriteCmosSensor(0x3AD2, 0xC969);
		HI541WriteCmosSensor(0x3AD4, 0x00F0);
		HI541WriteCmosSensor(0x3AD6, 0xAAF8);
		HI541WriteCmosSensor(0x3AD8, 0x00F0);
		HI541WriteCmosSensor(0x3ADA, 0xAEF8);
		HI541WriteCmosSensor(0x3ADC, 0xC6B2);
		HI541WriteCmosSensor(0x3ADE, 0x0198);
		HI541WriteCmosSensor(0x3AE0, 0x00F0);
		HI541WriteCmosSensor(0x3AE2, 0x98F8);
		HI541WriteCmosSensor(0x3AE4, 0x0099);
		HI541WriteCmosSensor(0x3AE6, 0x096A);
		HI541WriteCmosSensor(0x3AE8, 0x00F0);
		HI541WriteCmosSensor(0x3AEA, 0xA0F8);
		HI541WriteCmosSensor(0x3AEC, 0x00F0);
		HI541WriteCmosSensor(0x3AEE, 0xA4F8);
		HI541WriteCmosSensor(0x3AF0, 0xC1B2);
		HI541WriteCmosSensor(0x3AF2, 0xD1E7);
		HI541WriteCmosSensor(0x3AF4, 0xC800);
		HI541WriteCmosSensor(0x3AF6, 0x7208);
		HI541WriteCmosSensor(0x3AF8, 0x8218);
		HI541WriteCmosSensor(0x3AFA, 0x4048);
		HI541WriteCmosSensor(0x3AFC, 0x805C);
		HI541WriteCmosSensor(0x3AFE, 0x0009);
		HI541WriteCmosSensor(0x3B00, 0xD3E7);
		HI541WriteCmosSensor(0x3B02, 0x0A01);
		HI541WriteCmosSensor(0x3B04, 0x3E49);
		HI541WriteCmosSensor(0x3B06, 0x5118);
		HI541WriteCmosSensor(0x3B08, 0x895D);
		HI541WriteCmosSensor(0x3B0A, 0x401E);
		HI541WriteCmosSensor(0x3B0C, 0x8140);
		HI541WriteCmosSensor(0x3B0E, 0x4A00);
		HI541WriteCmosSensor(0x3B10, 0x0299);
		HI541WriteCmosSensor(0x3B12, 0x4A43);
		HI541WriteCmosSensor(0x3B14, 0x2168);
		HI541WriteCmosSensor(0x3B16, 0x5118);
		HI541WriteCmosSensor(0x3B18, 0x2160);
		HI541WriteCmosSensor(0x3B1A, 0xA288);
		HI541WriteCmosSensor(0x3B1C, 0x0221);
		HI541WriteCmosSensor(0x3B1E, 0x8140);
		HI541WriteCmosSensor(0x3B20, 0x5018);
		HI541WriteCmosSensor(0x3B22, 0xA080);
		HI541WriteCmosSensor(0x3B24, 0x7F1C);
		HI541WriteCmosSensor(0x3B26, 0xEF74);
		HI541WriteCmosSensor(0x3B28, 0xEF7C);
		HI541WriteCmosSensor(0x3B2A, 0x0498);
		HI541WriteCmosSensor(0x3B2C, 0x8742);
		HI541WriteCmosSensor(0x3B2E, 0xAED3);
		HI541WriteCmosSensor(0x3B30, 0x0198);
		HI541WriteCmosSensor(0x3B32, 0x401C);
		HI541WriteCmosSensor(0x3B34, 0xA874);
		HI541WriteCmosSensor(0x3B36, 0xA97C);
		HI541WriteCmosSensor(0x3B38, 0x0598);
		HI541WriteCmosSensor(0x3B3A, 0x0191);
		HI541WriteCmosSensor(0x3B3C, 0x8142);
		HI541WriteCmosSensor(0x3B3E, 0xA3D3);
		HI541WriteCmosSensor(0x3B40, 0x3048);
		HI541WriteCmosSensor(0x3B42, 0xA288);
		HI541WriteCmosSensor(0x3B44, 0x8260);
		HI541WriteCmosSensor(0x3B46, 0x2168);
		HI541WriteCmosSensor(0x3B48, 0x4160);
		HI541WriteCmosSensor(0x3B4A, 0xC168);
		HI541WriteCmosSensor(0x3B4C, 0x2A4C);
		HI541WriteCmosSensor(0x3B4E, 0xC9B2);
		HI541WriteCmosSensor(0x3B50, 0x603C);
		HI541WriteCmosSensor(0x3B52, 0xA172);
		HI541WriteCmosSensor(0x3B54, 0x0069);
		HI541WriteCmosSensor(0x3B56, 0x5208);
		HI541WriteCmosSensor(0x3B58, 0x8242);
		HI541WriteCmosSensor(0x3B5A, 0x01D2);
		HI541WriteCmosSensor(0x3B5C, 0x491C);
		HI541WriteCmosSensor(0x3B5E, 0xA172);
		HI541WriteCmosSensor(0x3B60, 0x0398);
		HI541WriteCmosSensor(0x3B62, 0x8007);
		HI541WriteCmosSensor(0x3B64, 0x0ED5);
		HI541WriteCmosSensor(0x3B66, 0x2448);
		HI541WriteCmosSensor(0x3B68, 0xCAB2);
		HI541WriteCmosSensor(0x3B6A, 0xA038);
		HI541WriteCmosSensor(0x3B6C, 0x017A);
		HI541WriteCmosSensor(0x3B6E, 0x501A);
		HI541WriteCmosSensor(0x3B70, 0x00D5);
		HI541WriteCmosSensor(0x3B72, 0x881A);
		HI541WriteCmosSensor(0x3B74, 0x2049);
		HI541WriteCmosSensor(0x3B76, 0x8039);
		HI541WriteCmosSensor(0x3B78, 0x497B);
		HI541WriteCmosSensor(0x3B7A, 0x8842);
		HI541WriteCmosSensor(0x3B7C, 0x02DA);
		HI541WriteCmosSensor(0x3B7E, 0x2248);
		HI541WriteCmosSensor(0x3B80, 0x0068);
		HI541WriteCmosSensor(0x3B82, 0x8047);
		HI541WriteCmosSensor(0x3B84, 0xA07A);
		HI541WriteCmosSensor(0x3B86, 0x0028);
		HI541WriteCmosSensor(0x3B88, 0x01D1);
		HI541WriteCmosSensor(0x3B8A, 0x0120);
		HI541WriteCmosSensor(0x3B8C, 0xA072);
		HI541WriteCmosSensor(0x3B8E, 0x07B0);
		HI541WriteCmosSensor(0x3B90, 0xF0BD);
		HI541WriteCmosSensor(0x3B92, 0x10B5);
		HI541WriteCmosSensor(0x3B94, 0x00F0);
		HI541WriteCmosSensor(0x3B96, 0x56F8);
		HI541WriteCmosSensor(0x3B98, 0x1C48);
		HI541WriteCmosSensor(0x3B9A, 0x8078);
		HI541WriteCmosSensor(0x3B9C, 0x0528);
		HI541WriteCmosSensor(0x3B9E, 0x07D1);
		HI541WriteCmosSensor(0x3BA0, 0x1A48);
		HI541WriteCmosSensor(0x3BA2, 0x1621);
		HI541WriteCmosSensor(0x3BA4, 0x6030);
		HI541WriteCmosSensor(0x3BA6, 0x415E);
		HI541WriteCmosSensor(0x3BA8, 0x0029);
		HI541WriteCmosSensor(0x3BAA, 0x01DA);
		HI541WriteCmosSensor(0x3BAC, 0x0021);
		HI541WriteCmosSensor(0x3BAE, 0xC182);
		HI541WriteCmosSensor(0x3BB0, 0x10BD);
		HI541WriteCmosSensor(0x3BB2, 0x70B5);
		HI541WriteCmosSensor(0x3BB4, 0x1048);
		HI541WriteCmosSensor(0x3BB6, 0x0021);
		HI541WriteCmosSensor(0x3BB8, 0x2030);
		HI541WriteCmosSensor(0x3BBA, 0x0160);
		HI541WriteCmosSensor(0x3BBC, 0x0E4D);
		HI541WriteCmosSensor(0x3BBE, 0x8180);
		HI541WriteCmosSensor(0x3BC0, 0x687F);
		HI541WriteCmosSensor(0x3BC2, 0x134C);
		HI541WriteCmosSensor(0x3BC4, 0x0007);
		HI541WriteCmosSensor(0x3BC6, 0x000F);
		HI541WriteCmosSensor(0x3BC8, 0x06D0);
		HI541WriteCmosSensor(0x3BCA, 0x0128);
		HI541WriteCmosSensor(0x3BCC, 0x06D0);
		HI541WriteCmosSensor(0x3BCE, 0x0228);
		HI541WriteCmosSensor(0x3BD0, 0x0DD0);
		HI541WriteCmosSensor(0x3BD2, 0x0328);
		HI541WriteCmosSensor(0x3BD4, 0x04D1);
		HI541WriteCmosSensor(0x3BD6, 0x0CE0);
		HI541WriteCmosSensor(0x3BD8, 0x2069);
		HI541WriteCmosSensor(0x3BDA, 0x00E0);
		HI541WriteCmosSensor(0x3BDC, 0x6069);
		HI541WriteCmosSensor(0x3BDE, 0x8047);
		HI541WriteCmosSensor(0x3BE0, 0x687F);
		HI541WriteCmosSensor(0x3BE2, 0x0009);
		HI541WriteCmosSensor(0x3BE4, 0xC007);
		HI541WriteCmosSensor(0x3BE6, 0x01D0);
		HI541WriteCmosSensor(0x3BE8, 0xE069);
		HI541WriteCmosSensor(0x3BEA, 0x8047);
		HI541WriteCmosSensor(0x3BEC, 0x70BD);
		HI541WriteCmosSensor(0x3BEE, 0xA069);
		HI541WriteCmosSensor(0x3BF0, 0xF5E7);
		HI541WriteCmosSensor(0x3BF2, 0x206A);
		HI541WriteCmosSensor(0x3BF4, 0xF3E7);
		HI541WriteCmosSensor(0x3BF6, 0x0000);
		HI541WriteCmosSensor(0x3BF8, 0x8003);
		HI541WriteCmosSensor(0x3BFA, 0x0020);
		HI541WriteCmosSensor(0x3BFC, 0x4804);
		HI541WriteCmosSensor(0x3BFE, 0x0020);
		HI541WriteCmosSensor(0x3C00, 0x1021);
		HI541WriteCmosSensor(0x3C02, 0x0020);
		HI541WriteCmosSensor(0x3C04, 0x0010);
		HI541WriteCmosSensor(0x3C06, 0x0140);
		HI541WriteCmosSensor(0x3C08, 0x5820);
		HI541WriteCmosSensor(0x3C0A, 0x0020);
		HI541WriteCmosSensor(0x3C0C, 0xB005);
		HI541WriteCmosSensor(0x3C0E, 0x0020);
		HI541WriteCmosSensor(0x3C10, 0x4805);
		HI541WriteCmosSensor(0x3C12, 0x0020);
		HI541WriteCmosSensor(0x3C14, 0x03B4);
		HI541WriteCmosSensor(0x3C16, 0x0148);
		HI541WriteCmosSensor(0x3C18, 0x0190);
		HI541WriteCmosSensor(0x3C1A, 0x01BD);
		HI541WriteCmosSensor(0x3C1C, 0xA1E7);
		HI541WriteCmosSensor(0x3C1E, 0x0000);
		HI541WriteCmosSensor(0x3C20, 0x03B4);
		HI541WriteCmosSensor(0x3C22, 0x0148);
		HI541WriteCmosSensor(0x3C24, 0x0190);
		HI541WriteCmosSensor(0x3C26, 0x01BD);
		HI541WriteCmosSensor(0x3C28, 0x6DE5);
		HI541WriteCmosSensor(0x3C2A, 0x0000);
		HI541WriteCmosSensor(0x3C2C, 0x03B4);
		HI541WriteCmosSensor(0x3C2E, 0x0148);
		HI541WriteCmosSensor(0x3C30, 0x0190);
		HI541WriteCmosSensor(0x3C32, 0x01BD);
		HI541WriteCmosSensor(0x3C34, 0x49EA);
		HI541WriteCmosSensor(0x3C36, 0x0000);
		HI541WriteCmosSensor(0x3C38, 0x03B4);
		HI541WriteCmosSensor(0x3C3A, 0x0148);
		HI541WriteCmosSensor(0x3C3C, 0x0190);
		HI541WriteCmosSensor(0x3C3E, 0x01BD);
		HI541WriteCmosSensor(0x3C40, 0x19E7);
		HI541WriteCmosSensor(0x3C42, 0x0000);
		HI541WriteCmosSensor(0x3C44, 0x03B4);
		HI541WriteCmosSensor(0x3C46, 0x0148);
		HI541WriteCmosSensor(0x3C48, 0x0190);
		HI541WriteCmosSensor(0x3C4A, 0x01BD);
		HI541WriteCmosSensor(0x3C4C, 0xA586);
		HI541WriteCmosSensor(0x3C4E, 0x0000);
}

HI541WriteCmosSensor(0x003c,0xDEC0); // ETC EN


HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x5400,0x0c0a); //tg_ctl1
HI541WriteCmosSensor(0x5402,0x0004); //tg_ctl3
HI541WriteCmosSensor(0x5432,0x0000); //analog_func_ctl1
HI541WriteCmosSensor(0x0008,0x0000); //data_pedestal
HI541WriteCmosSensor(0x0100,0x0000); //image_orientation
HI541WriteCmosSensor(0x0900,0x0002); //binning
HI541WriteCmosSensor(0x0200,0x9600); //fine_integration_time
		HI541WriteCmosSensor(0x0202,0x9800); //coarse_integration_time 100fps
HI541WriteCmosSensor(0x0204,0xe000); //analog_gain_code_global
HI541WriteCmosSensor(0x0340,0xc207);	//frame_length_lines
HI541WriteCmosSensor(0x0342,0xca0a);//line_length_pck
HI541WriteCmosSensor(0x0344,0x1a00);	//x_addr_start
HI541WriteCmosSensor(0x0346,0x1800);	//y_addr_start
HI541WriteCmosSensor(0x0348,0x4d0a);	//x_addr_end
HI541WriteCmosSensor(0x034a,0xc107);	//y_addr_end
HI541WriteCmosSensor(0x034c,0x200a);	//x_output_size
HI541WriteCmosSensor(0x034e,0x9807);	//y_output_size
HI541WriteCmosSensor(0x0380,0x0100);	//x_even_inc
HI541WriteCmosSensor(0x0382,0x0100);	//x_odd_inc
HI541WriteCmosSensor(0x0384,0x0100);	//y_even_inc
HI541WriteCmosSensor(0x0386,0x0100);	//y_odd_inc
HI541WriteCmosSensor(0xb400, 0x1000);
//================================
//========= SYSTEM End ===========
//================================

//===================================
//======= Analog Start        =======
//===================================
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x0b04,0x0100); //global BLC
//HI541WriteCmosSensor(0x54a6,0x4600); //global BLC option
HI541WriteCmosSensor(0x54a0,0x054d); //line blc,digital off  (Steve)
//HI541WriteCmosSensor(0x54a0,0x014d); //line blc,digital off  
HI541WriteCmosSensor(0x5450,0x308c); //comp1 bias
HI541WriteCmosSensor(0x5452,0x8c8c); //soft_stanby_comp1 bias, dark comp1 bias
HI541WriteCmosSensor(0x5488,0x123F); //comp2_bias  18
HI541WriteCmosSensor(0x544e,0x3030); //pix_bias int,px bias read
HI541WriteCmosSensor(0x5484,0x3030); //off_pix_bias,dark_px bias
HI541WriteCmosSensor(0x5486,0x308c); //outdoor_pix_bias,outdoor cds bias
HI541WriteCmosSensor(0x5442,0x0078); //vrst  48 
HI541WriteCmosSensor(0x5440,0x040b); //rng
HI541WriteCmosSensor(0x5482,0x0f00); //bw & vref2     
HI541WriteCmosSensor(0x5454,0x5836); //clamp lvl
HI541WriteCmosSensor(0x5438,0x000a); //vbias1&2 sample
HI541WriteCmosSensor(0x55d6,0x6802); //vbias1&2 sample  7002
HI541WriteCmosSensor(0x55d4,0x0200); //vbias1&2 sample
HI541WriteCmosSensor(0x5562,0x3b00); // rx read
HI541WriteCmosSensor(0x5584,0x4000); //s2 pos
HI541WriteCmosSensor(0x5586,0xb400); //s2 neg
HI541WriteCmosSensor(0x5588,0x4b00); //s3 pos
HI541WriteCmosSensor(0x558a,0x9d00); //s3 neg
HI541WriteCmosSensor(0x556c,0x4200); //rstclamp pos
HI541WriteCmosSensor(0x556e,0xbf00); //rstclamp neg  b800
HI541WriteCmosSensor(0x543c,0x0c00); //pxl_pwr_ctrl 
HI541WriteCmosSensor(0x5548,0x061c); //pxl_pwr_ctrl neg 
HI541WriteCmosSensor(0x544a,0xf100); //col_prcharge_pw

HI541WriteCmosSensor(0x54ae,0x0100); //column obp dpc thres
HI541WriteCmosSensor(0x54c0,0x1080); //row obp dpc thres
HI541WriteCmosSensor(0x5444,0xff7f); // poffset init,poffset rst
HI541WriteCmosSensor(0x5446,0x0050); //poffset sig
HI541WriteCmosSensor(0x547c,0x0f7f); // abs offset init
HI541WriteCmosSensor(0x547e,0x7f7f); //abs offset rst,abs offset sig
HI541WriteCmosSensor(0x54ac,0x02d2); // d offset h,d offset l
HI541WriteCmosSensor(0x5554,0x0702); //tx pos l,tx pos h 
HI541WriteCmosSensor(0x5556,0x4002); //tx neg l,tx neg h  2e02
HI541WriteCmosSensor(0x5514,0x0f01); // posset rst start l,posset rst start h
HI541WriteCmosSensor(0x5516,0x0002); // posset sig start l,posset sig start h
HI541WriteCmosSensor(0x558c,0x0d01); //s4 
HI541WriteCmosSensor(0x551c,0x0f01); //  rst flag neg l,rst flag neg h 
HI541WriteCmosSensor(0x551e,0x0002); //  rst flag pos l,rst flag pos h
HI541WriteCmosSensor(0x550e,0x5701); //  rst clkmsk_nge l,rst clkmsk_nge h
HI541WriteCmosSensor(0x550c,0xe901); //rst clkmsk_nge l,rst clkmsk_nge h
HI541WriteCmosSensor(0x5512,0x8802); //  sig clkmsk_nge h  88-->8a  8702
HI541WriteCmosSensor(0x5506,0x5501); //  preset neg l,preset neg h
HI541WriteCmosSensor(0x5504,0xf001); //  preset pos l,preset pos h
HI541WriteCmosSensor(0x545a,0x881c); //vpp,vbb 
HI541WriteCmosSensor(0x545c,0x1c00); //vbb 
HI541WriteCmosSensor(0x5524,0x0200); //int add pos
HI541WriteCmosSensor(0x5526,0x4800); //int add neg
HI541WriteCmosSensor(0x5528,0x0446); //int add_en pos,int add_en neg
HI541WriteCmosSensor(0x5530,0x0600); //int rx1 pos
HI541WriteCmosSensor(0x5532,0x4800); //int rx1 neg  2600
HI541WriteCmosSensor(0x553c,0x0800); //int tx1 pos
HI541WriteCmosSensor(0x553e,0x4200); //int tx1 neg
HI541WriteCmosSensor(0x0200,0x9600); //fine_integration_time_l (dec:150clk),fine_integration_time_h
////////////////CNTFREE//////////////////////
HI541WriteCmosSensor(0x5546,0xF406); //scn_addr_ful_neg
HI541WriteCmosSensor(0x554e,0xF206); //scn_en_ful_neg
HI541WriteCmosSensor(0x5552,0xEE06); //scn_sx_full_neg
HI541WriteCmosSensor(0x555c,0xF006); //tx3 pos
HI541WriteCmosSensor(0x555e,0xF406); //tx3 neg
HI541WriteCmosSensor(0x5568,0xF006); //rx3  pos
HI541WriteCmosSensor(0x556a,0xF406); //rx3 neg
HI541WriteCmosSensor(0x5510,0xE806); //ramp sig pos 
HI541WriteCmosSensor(0x5512,0xE803); //ramp sig neg
HI541WriteCmosSensor(0x5508,0xEE06); // preset sig pos
HI541WriteCmosSensor(0x550a,0xE803); // preset sig neg
HI541WriteCmosSensor(0x558e,0xEE06); // s4 end
HI541WriteCmosSensor(0x5576,0xF001); // test reset fall
HI541WriteCmosSensor(0x5578,0xF301); // test sig start  l 
HI541WriteCmosSensor(0x557a,0xEE06); // sig_t end 
HI541WriteCmosSensor(0x5518,0xEE06); //ramp poffset sig end 
HI541WriteCmosSensor(0x5554,0x7003); //tx pos l,tx pos h 
HI541WriteCmosSensor(0x5556,0xa403); //tx neg l,tx neg h  2e02
HI541WriteCmosSensor(0x55d6,0xa803); //vbias1&2 sample  7002
HI541WriteCmosSensor(0x540c,0x4388); //flush contol


HI541WriteCmosSensor(0x559c,0xf406); //  col_load pos 
HI541WriteCmosSensor(0x559e,0xff06); //  col_load neg 
HI541WriteCmosSensor(0x6002,0x7800); //  scaler fifo delay_l 
/////////////////////////////////////////
/////////////overblc///////////////////
//HI541WriteCmosSensor(0x54a6,0x0600); //  blc option
HI541WriteCmosSensor(0x54a6,0x0600); // blc option every frame update (STEVE) 20140711 don't touch
//HI541WriteCmosSensor(0x540a,0x0003); //  frame addres
HI541WriteCmosSensor(0x540a,0x0001); //r_drk_y_addr_start, end (STEVE) 
HI541WriteCmosSensor(0x54a4,0x0610);// 20140626 for row noise@dark 15 -> 10
////////////////////////////////////

HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x4bec,0xff21); //2221
HI541WriteCmosSensor(0x4bee,0x5e04); //5e04
HI541WriteCmosSensor(0x4bf0,0xef23); //c623 --> option on
///////////////////////////////////////////////////

//===================================
//======= ANALOG End        =========
//===================================

//Yscaler 2592x1944
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0xa800,0x2000); //mode_zoom1
HI541WriteCmosSensor(0xa804,0x200a); //zoom_dst_width
HI541WriteCmosSensor(0xa806,0x9807); //zoom_dst_height
HI541WriteCmosSensor(0xa810,0x0008); //zoom_hor_step
HI541WriteCmosSensor(0xa812,0x0008); //zoom_ver_step
HI541WriteCmosSensor(0xa814,0x0000); //zoom_hor_step_remain
HI541WriteCmosSensor(0xa816,0x0000); //zoom_ver_step_remain
HI541WriteCmosSensor(0xa818,0x6400); //zoom_fifo_delay
HI541WriteCmosSensor(0xa824,0x007f); //zoom_intpol1
HI541WriteCmosSensor(0xa826,0x7f00); //zoom_intpol3

//================================
//=========== LSC Start ==========
//================================
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x4828,0x4040);
HI541WriteCmosSensor(0xffff,0x00a0);

HI541WriteCmosSensor(0x0000,0xA102); //AF DRIVER IC BU64240GWZ
HI541WriteCmosSensor(0x0004,0x5F02);
HI541WriteCmosSensor(0x0008,0xFA01);
HI541WriteCmosSensor(0x000c,0xA201);
HI541WriteCmosSensor(0x0010,0x5D01);
HI541WriteCmosSensor(0x0014,0x2F01);
HI541WriteCmosSensor(0x0018,0x0E01);
HI541WriteCmosSensor(0x001c,0xF900);
HI541WriteCmosSensor(0x0020,0xF300);
HI541WriteCmosSensor(0x0024,0xF900);
HI541WriteCmosSensor(0x0028,0x0E01);
HI541WriteCmosSensor(0x002c,0x2F01);
HI541WriteCmosSensor(0x0030,0x5D01);
HI541WriteCmosSensor(0x0034,0xA201);
HI541WriteCmosSensor(0x0038,0xFA01);
HI541WriteCmosSensor(0x003c,0x5F02);
HI541WriteCmosSensor(0x0040,0xA102);
HI541WriteCmosSensor(0x0044,0x8402);
HI541WriteCmosSensor(0x0048,0x3202);
HI541WriteCmosSensor(0x004c,0xC001);
HI541WriteCmosSensor(0x0050,0x6C01);
HI541WriteCmosSensor(0x0054,0x2F01);
HI541WriteCmosSensor(0x0058,0xFF00);
HI541WriteCmosSensor(0x005c,0xDC00);
HI541WriteCmosSensor(0x0060,0xC800);
HI541WriteCmosSensor(0x0064,0xC300);
HI541WriteCmosSensor(0x0068,0xC800);
HI541WriteCmosSensor(0x006c,0xDC00);
HI541WriteCmosSensor(0x0070,0xFF00);
HI541WriteCmosSensor(0x0074,0x2F01);
HI541WriteCmosSensor(0x0078,0x6C01);
HI541WriteCmosSensor(0x007c,0xC001);
HI541WriteCmosSensor(0x0080,0x3202);
HI541WriteCmosSensor(0x0084,0x8402);
HI541WriteCmosSensor(0x0088,0x4B02);
HI541WriteCmosSensor(0x008c,0xE801);
HI541WriteCmosSensor(0x0090,0x8301);
HI541WriteCmosSensor(0x0094,0x3301);
HI541WriteCmosSensor(0x0098,0xF700);
HI541WriteCmosSensor(0x009c,0xC500);
HI541WriteCmosSensor(0x00a0,0xA100);
HI541WriteCmosSensor(0x00a4,0x8A00);
HI541WriteCmosSensor(0x00a8,0x8300);
HI541WriteCmosSensor(0x00ac,0x8A00);
HI541WriteCmosSensor(0x00b0,0xA100);
HI541WriteCmosSensor(0x00b4,0xC500);
HI541WriteCmosSensor(0x00b8,0xF700);
HI541WriteCmosSensor(0x00bc,0x3301);
HI541WriteCmosSensor(0x00c0,0x8301);
HI541WriteCmosSensor(0x00c4,0xE801);
HI541WriteCmosSensor(0x00c8,0x4B02);
HI541WriteCmosSensor(0x00cc,0x1002);
HI541WriteCmosSensor(0x00d0,0xB001);
HI541WriteCmosSensor(0x00d4,0x4F01);
HI541WriteCmosSensor(0x00d8,0x0601);
HI541WriteCmosSensor(0x00dc,0xC400);
HI541WriteCmosSensor(0x00e0,0x9000);
HI541WriteCmosSensor(0x00e4,0x6800);
HI541WriteCmosSensor(0x00e8,0x5100);
HI541WriteCmosSensor(0x00ec,0x4900);
HI541WriteCmosSensor(0x00f0,0x5100);
HI541WriteCmosSensor(0x00f4,0x6800);
HI541WriteCmosSensor(0x00f8,0x9000);
HI541WriteCmosSensor(0x00fc,0xC400);
HI541WriteCmosSensor(0x0100,0x0601);
HI541WriteCmosSensor(0x0104,0x4F01);
HI541WriteCmosSensor(0x0108,0xB001);
HI541WriteCmosSensor(0x010c,0x1002);
HI541WriteCmosSensor(0x0110,0xE401);
HI541WriteCmosSensor(0x0114,0x8D01);
HI541WriteCmosSensor(0x0118,0x2D01);
HI541WriteCmosSensor(0x011c,0xE300);
HI541WriteCmosSensor(0x0120,0xA100);
HI541WriteCmosSensor(0x0124,0x6B00);
HI541WriteCmosSensor(0x0128,0x4100);
HI541WriteCmosSensor(0x012c,0x2900);
HI541WriteCmosSensor(0x0130,0x2000);
HI541WriteCmosSensor(0x0134,0x2900);
HI541WriteCmosSensor(0x0138,0x4100);
HI541WriteCmosSensor(0x013c,0x6B00);
HI541WriteCmosSensor(0x0140,0xA100);
HI541WriteCmosSensor(0x0144,0xE300);
HI541WriteCmosSensor(0x0148,0x2D01);
HI541WriteCmosSensor(0x014c,0x8D01);
HI541WriteCmosSensor(0x0150,0xE401);
HI541WriteCmosSensor(0x0154,0xCD01);
HI541WriteCmosSensor(0x0158,0x7701);
HI541WriteCmosSensor(0x015c,0x1D01);
HI541WriteCmosSensor(0x0160,0xCE00);
HI541WriteCmosSensor(0x0164,0x8700);
HI541WriteCmosSensor(0x0168,0x4F00);
HI541WriteCmosSensor(0x016c,0x2800);
HI541WriteCmosSensor(0x0170,0x0F00);
HI541WriteCmosSensor(0x0174,0x0900);
HI541WriteCmosSensor(0x0178,0x0F00);
HI541WriteCmosSensor(0x017c,0x2800);
HI541WriteCmosSensor(0x0180,0x4F00);
HI541WriteCmosSensor(0x0184,0x8700);
HI541WriteCmosSensor(0x0188,0xCE00);
HI541WriteCmosSensor(0x018c,0x1D01);
HI541WriteCmosSensor(0x0190,0x7701);
HI541WriteCmosSensor(0x0194,0xCD01);
HI541WriteCmosSensor(0x0198,0xC701);
HI541WriteCmosSensor(0x019c,0x6D01);
HI541WriteCmosSensor(0x01a0,0x1701);
HI541WriteCmosSensor(0x01a4,0xC700);
HI541WriteCmosSensor(0x01a8,0x8100);
HI541WriteCmosSensor(0x01ac,0x4700);
HI541WriteCmosSensor(0x01b0,0x1F00);
HI541WriteCmosSensor(0x01b4,0x0900);
HI541WriteCmosSensor(0x01b8,0x0000);
HI541WriteCmosSensor(0x01bc,0x0900);
HI541WriteCmosSensor(0x01c0,0x1F00);
HI541WriteCmosSensor(0x01c4,0x4700);
HI541WriteCmosSensor(0x01c8,0x8100);
HI541WriteCmosSensor(0x01cc,0xC700);
HI541WriteCmosSensor(0x01d0,0x1701);
HI541WriteCmosSensor(0x01d4,0x6D01);
HI541WriteCmosSensor(0x01d8,0xC701);
HI541WriteCmosSensor(0x01dc,0xCD01);
HI541WriteCmosSensor(0x01e0,0x7701);
HI541WriteCmosSensor(0x01e4,0x1D01);
HI541WriteCmosSensor(0x01e8,0xCE00);
HI541WriteCmosSensor(0x01ec,0x8700);
HI541WriteCmosSensor(0x01f0,0x4F00);
HI541WriteCmosSensor(0x01f4,0x2800);
HI541WriteCmosSensor(0x01f8,0x0F00);
HI541WriteCmosSensor(0x01fc,0x0900);
HI541WriteCmosSensor(0x0200,0x0F00);
HI541WriteCmosSensor(0x0204,0x2800);
HI541WriteCmosSensor(0x0208,0x4F00);
HI541WriteCmosSensor(0x020c,0x8700);
HI541WriteCmosSensor(0x0210,0xCE00);
HI541WriteCmosSensor(0x0214,0x1D01);
HI541WriteCmosSensor(0x0218,0x7701);
HI541WriteCmosSensor(0x021c,0xCD01);
HI541WriteCmosSensor(0x0220,0xE401);
HI541WriteCmosSensor(0x0224,0x8D01);
HI541WriteCmosSensor(0x0228,0x2D01);
HI541WriteCmosSensor(0x022c,0xE300);
HI541WriteCmosSensor(0x0230,0xA100);
HI541WriteCmosSensor(0x0234,0x6B00);
HI541WriteCmosSensor(0x0238,0x4100);
HI541WriteCmosSensor(0x023c,0x2900);
HI541WriteCmosSensor(0x0240,0x2000);
HI541WriteCmosSensor(0x0244,0x2900);
HI541WriteCmosSensor(0x0248,0x4100);
HI541WriteCmosSensor(0x024c,0x6B00);
HI541WriteCmosSensor(0x0250,0xA100);
HI541WriteCmosSensor(0x0254,0xE300);
HI541WriteCmosSensor(0x0258,0x2D01);
HI541WriteCmosSensor(0x025c,0x8D01);
HI541WriteCmosSensor(0x0260,0xE401);
HI541WriteCmosSensor(0x0264,0x1002);
HI541WriteCmosSensor(0x0268,0xB001);
HI541WriteCmosSensor(0x026c,0x4F01);
HI541WriteCmosSensor(0x0270,0x0601);
HI541WriteCmosSensor(0x0274,0xC400);
HI541WriteCmosSensor(0x0278,0x9000);
HI541WriteCmosSensor(0x027c,0x6800);
HI541WriteCmosSensor(0x0280,0x5100);
HI541WriteCmosSensor(0x0284,0x4900);
HI541WriteCmosSensor(0x0288,0x5100);
HI541WriteCmosSensor(0x028c,0x6800);
HI541WriteCmosSensor(0x0290,0x9000);
HI541WriteCmosSensor(0x0294,0xC400);
HI541WriteCmosSensor(0x0298,0x0601);
HI541WriteCmosSensor(0x029c,0x4F01);
HI541WriteCmosSensor(0x02a0,0xB001);
HI541WriteCmosSensor(0x02a4,0x1002);
HI541WriteCmosSensor(0x02a8,0x4B02);
HI541WriteCmosSensor(0x02ac,0xE801);
HI541WriteCmosSensor(0x02b0,0x8301);
HI541WriteCmosSensor(0x02b4,0x3301);
HI541WriteCmosSensor(0x02b8,0xF700);
HI541WriteCmosSensor(0x02bc,0xC500);
HI541WriteCmosSensor(0x02c0,0xA100);
HI541WriteCmosSensor(0x02c4,0x8A00);
HI541WriteCmosSensor(0x02c8,0x8300);
HI541WriteCmosSensor(0x02cc,0x8A00);
HI541WriteCmosSensor(0x02d0,0xA100);
HI541WriteCmosSensor(0x02d4,0xC500);
HI541WriteCmosSensor(0x02d8,0xF700);
HI541WriteCmosSensor(0x02dc,0x3301);
HI541WriteCmosSensor(0x02e0,0x8301);
HI541WriteCmosSensor(0x02e4,0xE801);
HI541WriteCmosSensor(0x02e8,0x4B02);
HI541WriteCmosSensor(0x02ec,0x8402);
HI541WriteCmosSensor(0x02f0,0x3202);
HI541WriteCmosSensor(0x02f4,0xC001);
HI541WriteCmosSensor(0x02f8,0x6C01);
HI541WriteCmosSensor(0x02fc,0x2F01);
HI541WriteCmosSensor(0x0300,0xFF00);
HI541WriteCmosSensor(0x0304,0xDC00);
HI541WriteCmosSensor(0x0308,0xC800);
HI541WriteCmosSensor(0x030c,0xC300);
HI541WriteCmosSensor(0x0310,0xC800);
HI541WriteCmosSensor(0x0314,0xDC00);
HI541WriteCmosSensor(0x0318,0xFF00);
HI541WriteCmosSensor(0x031c,0x2F01);
HI541WriteCmosSensor(0x0320,0x6C01);
HI541WriteCmosSensor(0x0324,0xC001);
HI541WriteCmosSensor(0x0328,0x3202);
HI541WriteCmosSensor(0x032c,0x8402);
HI541WriteCmosSensor(0x0330,0xA102);
HI541WriteCmosSensor(0x0334,0x5F02);
HI541WriteCmosSensor(0x0338,0xFA01);
HI541WriteCmosSensor(0x033c,0xA201);
HI541WriteCmosSensor(0x0340,0x5D01);
HI541WriteCmosSensor(0x0344,0x2F01);
HI541WriteCmosSensor(0x0348,0x0E01);
HI541WriteCmosSensor(0x034c,0xF900);
HI541WriteCmosSensor(0x0350,0xF300);
HI541WriteCmosSensor(0x0354,0xF900);
HI541WriteCmosSensor(0x0358,0x0E01);
HI541WriteCmosSensor(0x035c,0x2F01);
HI541WriteCmosSensor(0x0360,0x5D01);
HI541WriteCmosSensor(0x0364,0xA201);
HI541WriteCmosSensor(0x0368,0xFA01);
HI541WriteCmosSensor(0x036c,0x5F02);
HI541WriteCmosSensor(0x0370,0xA102);
HI541WriteCmosSensor(0x0374,0xC903);
HI541WriteCmosSensor(0x0378,0x8B03);
HI541WriteCmosSensor(0x037c,0x1603);
HI541WriteCmosSensor(0x0380,0xA402);
HI541WriteCmosSensor(0x0384,0x4202);
HI541WriteCmosSensor(0x0388,0xF601);
HI541WriteCmosSensor(0x038c,0xC001);
HI541WriteCmosSensor(0x0390,0x9F01);
HI541WriteCmosSensor(0x0394,0x9401);
HI541WriteCmosSensor(0x0398,0x9F01);
HI541WriteCmosSensor(0x039c,0xC001);
HI541WriteCmosSensor(0x03a0,0xF601);
HI541WriteCmosSensor(0x03a4,0x4202);
HI541WriteCmosSensor(0x03a8,0xA402);
HI541WriteCmosSensor(0x03ac,0x1603);
HI541WriteCmosSensor(0x03b0,0x8B03);
HI541WriteCmosSensor(0x03b4,0xC903);
HI541WriteCmosSensor(0x03b8,0xB103);
HI541WriteCmosSensor(0x03bc,0x4E03);
HI541WriteCmosSensor(0x03c0,0xCB02);
HI541WriteCmosSensor(0x03c4,0x5602);
HI541WriteCmosSensor(0x03c8,0xF401);
HI541WriteCmosSensor(0x03cc,0xA401);
HI541WriteCmosSensor(0x03d0,0x6A01);
HI541WriteCmosSensor(0x03d4,0x4701);
HI541WriteCmosSensor(0x03d8,0x3C01);
HI541WriteCmosSensor(0x03dc,0x4701);
HI541WriteCmosSensor(0x03e0,0x6A01);
HI541WriteCmosSensor(0x03e4,0xA401);
HI541WriteCmosSensor(0x03e8,0xF401);
HI541WriteCmosSensor(0x03ec,0x5602);
HI541WriteCmosSensor(0x03f0,0xCB02);
HI541WriteCmosSensor(0x03f4,0x4E03);
HI541WriteCmosSensor(0x03f8,0xB103);
HI541WriteCmosSensor(0x03fc,0x7103);
HI541WriteCmosSensor(0x0400,0xF602);
HI541WriteCmosSensor(0x0404,0x6D02);
HI541WriteCmosSensor(0x0408,0xF501);
HI541WriteCmosSensor(0x040c,0x8C01);
HI541WriteCmosSensor(0x0410,0x3701);
HI541WriteCmosSensor(0x0414,0xFA00);
HI541WriteCmosSensor(0x0418,0xD400);
HI541WriteCmosSensor(0x041c,0xC800);
HI541WriteCmosSensor(0x0420,0xD400);
HI541WriteCmosSensor(0x0424,0xFA00);
HI541WriteCmosSensor(0x0428,0x3701);
HI541WriteCmosSensor(0x042c,0x8C01);
HI541WriteCmosSensor(0x0430,0xF501);
HI541WriteCmosSensor(0x0434,0x6D02);
HI541WriteCmosSensor(0x0438,0xF602);
HI541WriteCmosSensor(0x043c,0x7103);
HI541WriteCmosSensor(0x0440,0x2D03);
HI541WriteCmosSensor(0x0444,0xAE02);
HI541WriteCmosSensor(0x0448,0x2202);
HI541WriteCmosSensor(0x044c,0xA501);
HI541WriteCmosSensor(0x0450,0x3501);
HI541WriteCmosSensor(0x0454,0xDC00);
HI541WriteCmosSensor(0x0458,0x9E00);
HI541WriteCmosSensor(0x045c,0x7900);
HI541WriteCmosSensor(0x0460,0x6D00);
HI541WriteCmosSensor(0x0464,0x7900);
HI541WriteCmosSensor(0x0468,0x9E00);
HI541WriteCmosSensor(0x046c,0xDC00);
HI541WriteCmosSensor(0x0470,0x3501);
HI541WriteCmosSensor(0x0474,0xA501);
HI541WriteCmosSensor(0x0478,0x2202);
HI541WriteCmosSensor(0x047c,0xAE02);
HI541WriteCmosSensor(0x0480,0x2D03);
HI541WriteCmosSensor(0x0484,0xFB02);
HI541WriteCmosSensor(0x0488,0x7B02);
HI541WriteCmosSensor(0x048c,0xEC01);
HI541WriteCmosSensor(0x0490,0x6A01);
HI541WriteCmosSensor(0x0494,0xF600);
HI541WriteCmosSensor(0x0498,0x9C00);
HI541WriteCmosSensor(0x049c,0x5D00);
HI541WriteCmosSensor(0x04a0,0x3800);
HI541WriteCmosSensor(0x04a4,0x2D00);
HI541WriteCmosSensor(0x04a8,0x3800);
HI541WriteCmosSensor(0x04ac,0x5D00);
HI541WriteCmosSensor(0x04b0,0x9C00);
HI541WriteCmosSensor(0x04b4,0xF600);
HI541WriteCmosSensor(0x04b8,0x6A01);
HI541WriteCmosSensor(0x04bc,0xEC01);
HI541WriteCmosSensor(0x04c0,0x7B02);
HI541WriteCmosSensor(0x04c4,0xFB02);
HI541WriteCmosSensor(0x04c8,0xDC02);
HI541WriteCmosSensor(0x04cc,0x5C02);
HI541WriteCmosSensor(0x04d0,0xCB01);
HI541WriteCmosSensor(0x04d4,0x4401);
HI541WriteCmosSensor(0x04d8,0xCF00);
HI541WriteCmosSensor(0x04dc,0x7400);
HI541WriteCmosSensor(0x04e0,0x3600);
HI541WriteCmosSensor(0x04e4,0x1300);
HI541WriteCmosSensor(0x04e8,0x0B00);
HI541WriteCmosSensor(0x04ec,0x1300);
HI541WriteCmosSensor(0x04f0,0x3600);
HI541WriteCmosSensor(0x04f4,0x7400);
HI541WriteCmosSensor(0x04f8,0xCF00);
HI541WriteCmosSensor(0x04fc,0x4401);
HI541WriteCmosSensor(0x0500,0xCB01);
HI541WriteCmosSensor(0x0504,0x5C02);
HI541WriteCmosSensor(0x0508,0xDC02);
HI541WriteCmosSensor(0x050c,0xD202);
HI541WriteCmosSensor(0x0510,0x5102);
HI541WriteCmosSensor(0x0514,0xC001);
HI541WriteCmosSensor(0x0518,0x3801);
HI541WriteCmosSensor(0x051c,0xC200);
HI541WriteCmosSensor(0x0520,0x6700);
HI541WriteCmosSensor(0x0524,0x2900);
HI541WriteCmosSensor(0x0528,0x0800);
HI541WriteCmosSensor(0x052c,0x0000);
HI541WriteCmosSensor(0x0530,0x0800);
HI541WriteCmosSensor(0x0534,0x2900);
HI541WriteCmosSensor(0x0538,0x6700);
HI541WriteCmosSensor(0x053c,0xC200);
HI541WriteCmosSensor(0x0540,0x3801);
HI541WriteCmosSensor(0x0544,0xC001);
HI541WriteCmosSensor(0x0548,0x5102);
HI541WriteCmosSensor(0x054c,0xD202);
HI541WriteCmosSensor(0x0550,0xDC02);
HI541WriteCmosSensor(0x0554,0x5C02);
HI541WriteCmosSensor(0x0558,0xCB01);
HI541WriteCmosSensor(0x055c,0x4401);
HI541WriteCmosSensor(0x0560,0xCF00);
HI541WriteCmosSensor(0x0564,0x7400);
HI541WriteCmosSensor(0x0568,0x3600);
HI541WriteCmosSensor(0x056c,0x1300);
HI541WriteCmosSensor(0x0570,0x0B00);
HI541WriteCmosSensor(0x0574,0x1300);
HI541WriteCmosSensor(0x0578,0x3600);
HI541WriteCmosSensor(0x057c,0x7400);
HI541WriteCmosSensor(0x0580,0xCF00);
HI541WriteCmosSensor(0x0584,0x4401);
HI541WriteCmosSensor(0x0588,0xCB01);
HI541WriteCmosSensor(0x058c,0x5C02);
HI541WriteCmosSensor(0x0590,0xDC02);
HI541WriteCmosSensor(0x0594,0xFB02);
HI541WriteCmosSensor(0x0598,0x7B02);
HI541WriteCmosSensor(0x059c,0xEC01);
HI541WriteCmosSensor(0x05a0,0x6A01);
HI541WriteCmosSensor(0x05a4,0xF600);
HI541WriteCmosSensor(0x05a8,0x9C00);
HI541WriteCmosSensor(0x05ac,0x5D00);
HI541WriteCmosSensor(0x05b0,0x3800);
HI541WriteCmosSensor(0x05b4,0x2D00);
HI541WriteCmosSensor(0x05b8,0x3800);
HI541WriteCmosSensor(0x05bc,0x5D00);
HI541WriteCmosSensor(0x05c0,0x9C00);
HI541WriteCmosSensor(0x05c4,0xF600);
HI541WriteCmosSensor(0x05c8,0x6A01);
HI541WriteCmosSensor(0x05cc,0xEC01);
HI541WriteCmosSensor(0x05d0,0x7B02);
HI541WriteCmosSensor(0x05d4,0xFB02);
HI541WriteCmosSensor(0x05d8,0x2D03);
HI541WriteCmosSensor(0x05dc,0xAE02);
HI541WriteCmosSensor(0x05e0,0x2202);
HI541WriteCmosSensor(0x05e4,0xA501);
HI541WriteCmosSensor(0x05e8,0x3501);
HI541WriteCmosSensor(0x05ec,0xDC00);
HI541WriteCmosSensor(0x05f0,0x9E00);
HI541WriteCmosSensor(0x05f4,0x7900);
HI541WriteCmosSensor(0x05f8,0x6D00);
HI541WriteCmosSensor(0x05fc,0x7900);
HI541WriteCmosSensor(0x0600,0x9E00);
HI541WriteCmosSensor(0x0604,0xDC00);
HI541WriteCmosSensor(0x0608,0x3501);
HI541WriteCmosSensor(0x060c,0xA501);
HI541WriteCmosSensor(0x0610,0x2202);
HI541WriteCmosSensor(0x0614,0xAE02);
HI541WriteCmosSensor(0x0618,0x2D03);
HI541WriteCmosSensor(0x061c,0x7103);
HI541WriteCmosSensor(0x0620,0xF602);
HI541WriteCmosSensor(0x0624,0x6D02);
HI541WriteCmosSensor(0x0628,0xF501);
HI541WriteCmosSensor(0x062c,0x8C01);
HI541WriteCmosSensor(0x0630,0x3701);
HI541WriteCmosSensor(0x0634,0xFA00);
HI541WriteCmosSensor(0x0638,0xD400);
HI541WriteCmosSensor(0x063c,0xC800);
HI541WriteCmosSensor(0x0640,0xD400);
HI541WriteCmosSensor(0x0644,0xFA00);
HI541WriteCmosSensor(0x0648,0x3701);
HI541WriteCmosSensor(0x064c,0x8C01);
HI541WriteCmosSensor(0x0650,0xF501);
HI541WriteCmosSensor(0x0654,0x6D02);
HI541WriteCmosSensor(0x0658,0xF602);
HI541WriteCmosSensor(0x065c,0x7103);
HI541WriteCmosSensor(0x0660,0xB103);
HI541WriteCmosSensor(0x0664,0x4E03);
HI541WriteCmosSensor(0x0668,0xCB02);
HI541WriteCmosSensor(0x066c,0x5602);
HI541WriteCmosSensor(0x0670,0xF401);
HI541WriteCmosSensor(0x0674,0xA401);
HI541WriteCmosSensor(0x0678,0x6A01);
HI541WriteCmosSensor(0x067c,0x4701);
HI541WriteCmosSensor(0x0680,0x3C01);
HI541WriteCmosSensor(0x0684,0x4701);
HI541WriteCmosSensor(0x0688,0x6A01);
HI541WriteCmosSensor(0x068c,0xA401);
HI541WriteCmosSensor(0x0690,0xF401);
HI541WriteCmosSensor(0x0694,0x5602);
HI541WriteCmosSensor(0x0698,0xCB02);
HI541WriteCmosSensor(0x069c,0x4E03);
HI541WriteCmosSensor(0x06a0,0xB103);
HI541WriteCmosSensor(0x06a4,0xC903);
HI541WriteCmosSensor(0x06a8,0x8B03);
HI541WriteCmosSensor(0x06ac,0x1603);
HI541WriteCmosSensor(0x06b0,0xA402);
HI541WriteCmosSensor(0x06b4,0x4202);
HI541WriteCmosSensor(0x06b8,0xF601);
HI541WriteCmosSensor(0x06bc,0xC001);
HI541WriteCmosSensor(0x06c0,0x9F01);
HI541WriteCmosSensor(0x06c4,0x9401);
HI541WriteCmosSensor(0x06c8,0x9F01);
HI541WriteCmosSensor(0x06cc,0xC001);
HI541WriteCmosSensor(0x06d0,0xF601);
HI541WriteCmosSensor(0x06d4,0x4202);
HI541WriteCmosSensor(0x06d8,0xA402);
HI541WriteCmosSensor(0x06dc,0x1603);
HI541WriteCmosSensor(0x06e0,0x8B03);
HI541WriteCmosSensor(0x06e4,0xC903);
HI541WriteCmosSensor(0x06e8,0x7302);
HI541WriteCmosSensor(0x06ec,0x3302);
HI541WriteCmosSensor(0x06f0,0xCF01);
HI541WriteCmosSensor(0x06f4,0x7C01);
HI541WriteCmosSensor(0x06f8,0x3C01);
HI541WriteCmosSensor(0x06fc,0x0D01);
HI541WriteCmosSensor(0x0700,0xEE00);
HI541WriteCmosSensor(0x0704,0xDB00);
HI541WriteCmosSensor(0x0708,0xD500);
HI541WriteCmosSensor(0x070c,0xDB00);
HI541WriteCmosSensor(0x0710,0xEE00);
HI541WriteCmosSensor(0x0714,0x0D01);
HI541WriteCmosSensor(0x0718,0x3C01);
HI541WriteCmosSensor(0x071c,0x7C01);
HI541WriteCmosSensor(0x0720,0xCF01);
HI541WriteCmosSensor(0x0724,0x3302);
HI541WriteCmosSensor(0x0728,0x7302);
HI541WriteCmosSensor(0x072c,0x4802);
HI541WriteCmosSensor(0x0730,0xF701);
HI541WriteCmosSensor(0x0734,0x9301);
HI541WriteCmosSensor(0x0738,0x4501);
HI541WriteCmosSensor(0x073c,0x0B01);
HI541WriteCmosSensor(0x0740,0xE000);
HI541WriteCmosSensor(0x0744,0xC100);
HI541WriteCmosSensor(0x0748,0xAF00);
HI541WriteCmosSensor(0x074c,0xA900);
HI541WriteCmosSensor(0x0750,0xAF00);
HI541WriteCmosSensor(0x0754,0xC100);
HI541WriteCmosSensor(0x0758,0xE000);
HI541WriteCmosSensor(0x075c,0x0B01);
HI541WriteCmosSensor(0x0760,0x4501);
HI541WriteCmosSensor(0x0764,0x9301);
HI541WriteCmosSensor(0x0768,0xF701);
HI541WriteCmosSensor(0x076c,0x4802);
HI541WriteCmosSensor(0x0770,0x0902);
HI541WriteCmosSensor(0x0774,0xAF01);
HI541WriteCmosSensor(0x0778,0x5101);
HI541WriteCmosSensor(0x077c,0x0C01);
HI541WriteCmosSensor(0x0780,0xD500);
HI541WriteCmosSensor(0x0784,0xA900);
HI541WriteCmosSensor(0x0788,0x8800);
HI541WriteCmosSensor(0x078c,0x7400);
HI541WriteCmosSensor(0x0790,0x6D00);
HI541WriteCmosSensor(0x0794,0x7400);
HI541WriteCmosSensor(0x0798,0x8800);
HI541WriteCmosSensor(0x079c,0xA900);
HI541WriteCmosSensor(0x07a0,0xD500);
HI541WriteCmosSensor(0x07a4,0x0C01);
HI541WriteCmosSensor(0x07a8,0x5101);
HI541WriteCmosSensor(0x07ac,0xAF01);
HI541WriteCmosSensor(0x07b0,0x0902);
HI541WriteCmosSensor(0x07b4,0xCE01);
HI541WriteCmosSensor(0x07b8,0x7801);
HI541WriteCmosSensor(0x07bc,0x2101);
HI541WriteCmosSensor(0x07c0,0xDF00);
HI541WriteCmosSensor(0x07c4,0xA700);
HI541WriteCmosSensor(0x07c8,0x7900);
HI541WriteCmosSensor(0x07cc,0x5700);
HI541WriteCmosSensor(0x07d0,0x4300);
HI541WriteCmosSensor(0x07d4,0x3C00);
HI541WriteCmosSensor(0x07d8,0x4300);
HI541WriteCmosSensor(0x07dc,0x5700);
HI541WriteCmosSensor(0x07e0,0x7900);
HI541WriteCmosSensor(0x07e4,0xA700);
HI541WriteCmosSensor(0x07e8,0xDF00);
HI541WriteCmosSensor(0x07ec,0x2101);
HI541WriteCmosSensor(0x07f0,0x7801);
HI541WriteCmosSensor(0x07f4,0xCE01);
HI541WriteCmosSensor(0x07f8,0xA401);
HI541WriteCmosSensor(0x07fc,0x5301);
HI541WriteCmosSensor(0x0800,0x0101);
HI541WriteCmosSensor(0x0804,0xBF00);
HI541WriteCmosSensor(0x0808,0x8500);
HI541WriteCmosSensor(0x080c,0x5600);
HI541WriteCmosSensor(0x0810,0x3400);
HI541WriteCmosSensor(0x0814,0x2000);
HI541WriteCmosSensor(0x0818,0x1A00);
HI541WriteCmosSensor(0x081c,0x2000);
HI541WriteCmosSensor(0x0820,0x3400);
HI541WriteCmosSensor(0x0824,0x5600);
HI541WriteCmosSensor(0x0828,0x8500);
HI541WriteCmosSensor(0x082c,0xBF00);
HI541WriteCmosSensor(0x0830,0x0101);
HI541WriteCmosSensor(0x0834,0x5301);
HI541WriteCmosSensor(0x0838,0xA401);
HI541WriteCmosSensor(0x083c,0x8C01);
HI541WriteCmosSensor(0x0840,0x3E01);
HI541WriteCmosSensor(0x0844,0xEF00);
HI541WriteCmosSensor(0x0848,0xAB00);
HI541WriteCmosSensor(0x084c,0x7000);
HI541WriteCmosSensor(0x0850,0x4000);
HI541WriteCmosSensor(0x0854,0x1F00);
HI541WriteCmosSensor(0x0858,0x0C00);
HI541WriteCmosSensor(0x085c,0x0600);
HI541WriteCmosSensor(0x0860,0x0C00);
HI541WriteCmosSensor(0x0864,0x1F00);
HI541WriteCmosSensor(0x0868,0x4000);
HI541WriteCmosSensor(0x086c,0x7000);
HI541WriteCmosSensor(0x0870,0xAB00);
HI541WriteCmosSensor(0x0874,0xEF00);
HI541WriteCmosSensor(0x0878,0x3E01);
HI541WriteCmosSensor(0x087c,0x8C01);
HI541WriteCmosSensor(0x0880,0x8501);
HI541WriteCmosSensor(0x0884,0x3701);
HI541WriteCmosSensor(0x0888,0xE800);
HI541WriteCmosSensor(0x088c,0xA500);
HI541WriteCmosSensor(0x0890,0x6900);
HI541WriteCmosSensor(0x0894,0x3900);
HI541WriteCmosSensor(0x0898,0x1800);
HI541WriteCmosSensor(0x089c,0x0500);
HI541WriteCmosSensor(0x08a0,0x0000);
HI541WriteCmosSensor(0x08a4,0x0500);
HI541WriteCmosSensor(0x08a8,0x1800);
HI541WriteCmosSensor(0x08ac,0x3900);
HI541WriteCmosSensor(0x08b0,0x6900);
HI541WriteCmosSensor(0x08b4,0xA500);
HI541WriteCmosSensor(0x08b8,0xE800);
HI541WriteCmosSensor(0x08bc,0x3701);
HI541WriteCmosSensor(0x08c0,0x8501);
HI541WriteCmosSensor(0x08c4,0x8C01);
HI541WriteCmosSensor(0x08c8,0x3E01);
HI541WriteCmosSensor(0x08cc,0xEF00);
HI541WriteCmosSensor(0x08d0,0xAB00);
HI541WriteCmosSensor(0x08d4,0x7000);
HI541WriteCmosSensor(0x08d8,0x4000);
HI541WriteCmosSensor(0x08dc,0x1F00);
HI541WriteCmosSensor(0x08e0,0x0C00);
HI541WriteCmosSensor(0x08e4,0x0600);
HI541WriteCmosSensor(0x08e8,0x0C00);
HI541WriteCmosSensor(0x08ec,0x1F00);
HI541WriteCmosSensor(0x08f0,0x4000);
HI541WriteCmosSensor(0x08f4,0x7000);
HI541WriteCmosSensor(0x08f8,0xAB00);
HI541WriteCmosSensor(0x08fc,0xEF00);
HI541WriteCmosSensor(0x0900,0x3E01);
HI541WriteCmosSensor(0x0904,0x8C01);
HI541WriteCmosSensor(0x0908,0xA401);
HI541WriteCmosSensor(0x090c,0x5301);
HI541WriteCmosSensor(0x0910,0x0101);
HI541WriteCmosSensor(0x0914,0xBF00);
HI541WriteCmosSensor(0x0918,0x8500);
HI541WriteCmosSensor(0x091c,0x5600);
HI541WriteCmosSensor(0x0920,0x3400);
HI541WriteCmosSensor(0x0924,0x2000);
HI541WriteCmosSensor(0x0928,0x1A00);
HI541WriteCmosSensor(0x092c,0x2000);
HI541WriteCmosSensor(0x0930,0x3400);
HI541WriteCmosSensor(0x0934,0x5600);
HI541WriteCmosSensor(0x0938,0x8500);
HI541WriteCmosSensor(0x093c,0xBF00);
HI541WriteCmosSensor(0x0940,0x0101);
HI541WriteCmosSensor(0x0944,0x5301);
HI541WriteCmosSensor(0x0948,0xA401);
HI541WriteCmosSensor(0x094c,0xCE01);
HI541WriteCmosSensor(0x0950,0x7801);
HI541WriteCmosSensor(0x0954,0x2101);
HI541WriteCmosSensor(0x0958,0xDF00);
HI541WriteCmosSensor(0x095c,0xA700);
HI541WriteCmosSensor(0x0960,0x7900);
HI541WriteCmosSensor(0x0964,0x5700);
HI541WriteCmosSensor(0x0968,0x4300);
HI541WriteCmosSensor(0x096c,0x3C00);
HI541WriteCmosSensor(0x0970,0x4300);
HI541WriteCmosSensor(0x0974,0x5700);
HI541WriteCmosSensor(0x0978,0x7900);
HI541WriteCmosSensor(0x097c,0xA700);
HI541WriteCmosSensor(0x0980,0xDF00);
HI541WriteCmosSensor(0x0984,0x2101);
HI541WriteCmosSensor(0x0988,0x7801);
HI541WriteCmosSensor(0x098c,0xCE01);
HI541WriteCmosSensor(0x0990,0x0902);
HI541WriteCmosSensor(0x0994,0xAF01);
HI541WriteCmosSensor(0x0998,0x5101);
HI541WriteCmosSensor(0x099c,0x0C01);
HI541WriteCmosSensor(0x09a0,0xD500);
HI541WriteCmosSensor(0x09a4,0xA900);
HI541WriteCmosSensor(0x09a8,0x8800);
HI541WriteCmosSensor(0x09ac,0x7400);
HI541WriteCmosSensor(0x09b0,0x6D00);
HI541WriteCmosSensor(0x09b4,0x7400);
HI541WriteCmosSensor(0x09b8,0x8800);
HI541WriteCmosSensor(0x09bc,0xA900);
HI541WriteCmosSensor(0x09c0,0xD500);
HI541WriteCmosSensor(0x09c4,0x0C01);
HI541WriteCmosSensor(0x09c8,0x5101);
HI541WriteCmosSensor(0x09cc,0xAF01);
HI541WriteCmosSensor(0x09d0,0x0902);
HI541WriteCmosSensor(0x09d4,0x4802);
HI541WriteCmosSensor(0x09d8,0xF701);
HI541WriteCmosSensor(0x09dc,0x9301);
HI541WriteCmosSensor(0x09e0,0x4501);
HI541WriteCmosSensor(0x09e4,0x0B01);
HI541WriteCmosSensor(0x09e8,0xE000);
HI541WriteCmosSensor(0x09ec,0xC100);
HI541WriteCmosSensor(0x09f0,0xAF00);
HI541WriteCmosSensor(0x09f4,0xA900);
HI541WriteCmosSensor(0x09f8,0xAF00);
HI541WriteCmosSensor(0x09fc,0xC100);
HI541WriteCmosSensor(0x0a00,0xE000);
HI541WriteCmosSensor(0x0a04,0x0B01);
HI541WriteCmosSensor(0x0a08,0x4501);
HI541WriteCmosSensor(0x0a0c,0x9301);
HI541WriteCmosSensor(0x0a10,0xF701);
HI541WriteCmosSensor(0x0a14,0x4802);
HI541WriteCmosSensor(0x0a18,0x7302);
HI541WriteCmosSensor(0x0a1c,0x3302);
HI541WriteCmosSensor(0x0a20,0xCF01);
HI541WriteCmosSensor(0x0a24,0x7C01);
HI541WriteCmosSensor(0x0a28,0x3C01);
HI541WriteCmosSensor(0x0a2c,0x0D01);
HI541WriteCmosSensor(0x0a30,0xEE00);
HI541WriteCmosSensor(0x0a34,0xDB00);
HI541WriteCmosSensor(0x0a38,0xD500);
HI541WriteCmosSensor(0x0a3c,0xDB00);
HI541WriteCmosSensor(0x0a40,0xEE00);
HI541WriteCmosSensor(0x0a44,0x0D01);
HI541WriteCmosSensor(0x0a48,0x3C01);
HI541WriteCmosSensor(0x0a4c,0x7C01);
HI541WriteCmosSensor(0x0a50,0xCF01);
HI541WriteCmosSensor(0x0a54,0x3302);
HI541WriteCmosSensor(0x0a58,0x7302);
HI541WriteCmosSensor(0x0a5c,0xA102);
HI541WriteCmosSensor(0x0a60,0x5F02);
HI541WriteCmosSensor(0x0a64,0xFA01);
HI541WriteCmosSensor(0x0a68,0xA201);
HI541WriteCmosSensor(0x0a6c,0x5D01);
HI541WriteCmosSensor(0x0a70,0x2F01);
HI541WriteCmosSensor(0x0a74,0x0E01);
HI541WriteCmosSensor(0x0a78,0xF900);
HI541WriteCmosSensor(0x0a7c,0xF300);
HI541WriteCmosSensor(0x0a80,0xF900);
HI541WriteCmosSensor(0x0a84,0x0E01);
HI541WriteCmosSensor(0x0a88,0x2F01);
HI541WriteCmosSensor(0x0a8c,0x5D01);
HI541WriteCmosSensor(0x0a90,0xA201);
HI541WriteCmosSensor(0x0a94,0xFA01);
HI541WriteCmosSensor(0x0a98,0x5F02);
HI541WriteCmosSensor(0x0a9c,0xA102);
HI541WriteCmosSensor(0x0aa0,0x8402);
HI541WriteCmosSensor(0x0aa4,0x3202);
HI541WriteCmosSensor(0x0aa8,0xC001);
HI541WriteCmosSensor(0x0aac,0x6C01);
HI541WriteCmosSensor(0x0ab0,0x2F01);
HI541WriteCmosSensor(0x0ab4,0xFF00);
HI541WriteCmosSensor(0x0ab8,0xDC00);
HI541WriteCmosSensor(0x0abc,0xC800);
HI541WriteCmosSensor(0x0ac0,0xC300);
HI541WriteCmosSensor(0x0ac4,0xC800);
HI541WriteCmosSensor(0x0ac8,0xDC00);
HI541WriteCmosSensor(0x0acc,0xFF00);
HI541WriteCmosSensor(0x0ad0,0x2F01);
HI541WriteCmosSensor(0x0ad4,0x6C01);
HI541WriteCmosSensor(0x0ad8,0xC001);
HI541WriteCmosSensor(0x0adc,0x3202);
HI541WriteCmosSensor(0x0ae0,0x8402);
HI541WriteCmosSensor(0x0ae4,0x4B02);
HI541WriteCmosSensor(0x0ae8,0xE801);
HI541WriteCmosSensor(0x0aec,0x8301);
HI541WriteCmosSensor(0x0af0,0x3301);
HI541WriteCmosSensor(0x0af4,0xF700);
HI541WriteCmosSensor(0x0af8,0xC500);
HI541WriteCmosSensor(0x0afc,0xA100);
HI541WriteCmosSensor(0x0b00,0x8A00);
HI541WriteCmosSensor(0x0b04,0x8300);
HI541WriteCmosSensor(0x0b08,0x8A00);
HI541WriteCmosSensor(0x0b0c,0xA100);
HI541WriteCmosSensor(0x0b10,0xC500);
HI541WriteCmosSensor(0x0b14,0xF700);
HI541WriteCmosSensor(0x0b18,0x3301);
HI541WriteCmosSensor(0x0b1c,0x8301);
HI541WriteCmosSensor(0x0b20,0xE801);
HI541WriteCmosSensor(0x0b24,0x4B02);
HI541WriteCmosSensor(0x0b28,0x1002);
HI541WriteCmosSensor(0x0b2c,0xB001);
HI541WriteCmosSensor(0x0b30,0x4F01);
HI541WriteCmosSensor(0x0b34,0x0601);
HI541WriteCmosSensor(0x0b38,0xC400);
HI541WriteCmosSensor(0x0b3c,0x9000);
HI541WriteCmosSensor(0x0b40,0x6800);
HI541WriteCmosSensor(0x0b44,0x5100);
HI541WriteCmosSensor(0x0b48,0x4900);
HI541WriteCmosSensor(0x0b4c,0x5100);
HI541WriteCmosSensor(0x0b50,0x6800);
HI541WriteCmosSensor(0x0b54,0x9000);
HI541WriteCmosSensor(0x0b58,0xC400);
HI541WriteCmosSensor(0x0b5c,0x0601);
HI541WriteCmosSensor(0x0b60,0x4F01);
HI541WriteCmosSensor(0x0b64,0xB001);
HI541WriteCmosSensor(0x0b68,0x1002);
HI541WriteCmosSensor(0x0b6c,0xE401);
HI541WriteCmosSensor(0x0b70,0x8D01);
HI541WriteCmosSensor(0x0b74,0x2D01);
HI541WriteCmosSensor(0x0b78,0xE300);
HI541WriteCmosSensor(0x0b7c,0xA100);
HI541WriteCmosSensor(0x0b80,0x6B00);
HI541WriteCmosSensor(0x0b84,0x4100);
HI541WriteCmosSensor(0x0b88,0x2900);
HI541WriteCmosSensor(0x0b8c,0x2000);
HI541WriteCmosSensor(0x0b90,0x2900);
HI541WriteCmosSensor(0x0b94,0x4100);
HI541WriteCmosSensor(0x0b98,0x6B00);
HI541WriteCmosSensor(0x0b9c,0xA100);
HI541WriteCmosSensor(0x0ba0,0xE300);
HI541WriteCmosSensor(0x0ba4,0x2D01);
HI541WriteCmosSensor(0x0ba8,0x8D01);
HI541WriteCmosSensor(0x0bac,0xE401);
HI541WriteCmosSensor(0x0bb0,0xCD01);
HI541WriteCmosSensor(0x0bb4,0x7701);
HI541WriteCmosSensor(0x0bb8,0x1D01);
HI541WriteCmosSensor(0x0bbc,0xCE00);
HI541WriteCmosSensor(0x0bc0,0x8700);
HI541WriteCmosSensor(0x0bc4,0x4F00);
HI541WriteCmosSensor(0x0bc8,0x2800);
HI541WriteCmosSensor(0x0bcc,0x0F00);
HI541WriteCmosSensor(0x0bd0,0x0900);
HI541WriteCmosSensor(0x0bd4,0x0F00);
HI541WriteCmosSensor(0x0bd8,0x2800);
HI541WriteCmosSensor(0x0bdc,0x4F00);
HI541WriteCmosSensor(0x0be0,0x8700);
HI541WriteCmosSensor(0x0be4,0xCE00);
HI541WriteCmosSensor(0x0be8,0x1D01);
HI541WriteCmosSensor(0x0bec,0x7701);
HI541WriteCmosSensor(0x0bf0,0xCD01);
HI541WriteCmosSensor(0x0bf4,0xC701);
HI541WriteCmosSensor(0x0bf8,0x6D01);
HI541WriteCmosSensor(0x0bfc,0x1701);
HI541WriteCmosSensor(0x0c00,0xC700);
HI541WriteCmosSensor(0x0c04,0x8100);
HI541WriteCmosSensor(0x0c08,0x4700);
HI541WriteCmosSensor(0x0c0c,0x1F00);
HI541WriteCmosSensor(0x0c10,0x0900);
HI541WriteCmosSensor(0x0c14,0x0000);
HI541WriteCmosSensor(0x0c18,0x0900);
HI541WriteCmosSensor(0x0c1c,0x1F00);
HI541WriteCmosSensor(0x0c20,0x4700);
HI541WriteCmosSensor(0x0c24,0x8100);
HI541WriteCmosSensor(0x0c28,0xC700);
HI541WriteCmosSensor(0x0c2c,0x1701);
HI541WriteCmosSensor(0x0c30,0x6D01);
HI541WriteCmosSensor(0x0c34,0xC701);
HI541WriteCmosSensor(0x0c38,0xCD01);
HI541WriteCmosSensor(0x0c3c,0x7701);
HI541WriteCmosSensor(0x0c40,0x1D01);
HI541WriteCmosSensor(0x0c44,0xCE00);
HI541WriteCmosSensor(0x0c48,0x8700);
HI541WriteCmosSensor(0x0c4c,0x4F00);
HI541WriteCmosSensor(0x0c50,0x2800);
HI541WriteCmosSensor(0x0c54,0x0F00);
HI541WriteCmosSensor(0x0c58,0x0900);
HI541WriteCmosSensor(0x0c5c,0x0F00);
HI541WriteCmosSensor(0x0c60,0x2800);
HI541WriteCmosSensor(0x0c64,0x4F00);
HI541WriteCmosSensor(0x0c68,0x8700);
HI541WriteCmosSensor(0x0c6c,0xCE00);
HI541WriteCmosSensor(0x0c70,0x1D01);
HI541WriteCmosSensor(0x0c74,0x7701);
HI541WriteCmosSensor(0x0c78,0xCD01);
HI541WriteCmosSensor(0x0c7c,0xE401);
HI541WriteCmosSensor(0x0c80,0x8D01);
HI541WriteCmosSensor(0x0c84,0x2D01);
HI541WriteCmosSensor(0x0c88,0xE300);
HI541WriteCmosSensor(0x0c8c,0xA100);
HI541WriteCmosSensor(0x0c90,0x6B00);
HI541WriteCmosSensor(0x0c94,0x4100);
HI541WriteCmosSensor(0x0c98,0x2900);
HI541WriteCmosSensor(0x0c9c,0x2000);
HI541WriteCmosSensor(0x0ca0,0x2900);
HI541WriteCmosSensor(0x0ca4,0x4100);
HI541WriteCmosSensor(0x0ca8,0x6B00);
HI541WriteCmosSensor(0x0cac,0xA100);
HI541WriteCmosSensor(0x0cb0,0xE300);
HI541WriteCmosSensor(0x0cb4,0x2D01);
HI541WriteCmosSensor(0x0cb8,0x8D01);
HI541WriteCmosSensor(0x0cbc,0xE401);
HI541WriteCmosSensor(0x0cc0,0x1002);
HI541WriteCmosSensor(0x0cc4,0xB001);
HI541WriteCmosSensor(0x0cc8,0x4F01);
HI541WriteCmosSensor(0x0ccc,0x0601);
HI541WriteCmosSensor(0x0cd0,0xC400);
HI541WriteCmosSensor(0x0cd4,0x9000);
HI541WriteCmosSensor(0x0cd8,0x6800);
HI541WriteCmosSensor(0x0cdc,0x5100);
HI541WriteCmosSensor(0x0ce0,0x4900);
HI541WriteCmosSensor(0x0ce4,0x5100);
HI541WriteCmosSensor(0x0ce8,0x6800);
HI541WriteCmosSensor(0x0cec,0x9000);
HI541WriteCmosSensor(0x0cf0,0xC400);
HI541WriteCmosSensor(0x0cf4,0x0601);
HI541WriteCmosSensor(0x0cf8,0x4F01);
HI541WriteCmosSensor(0x0cfc,0xB001);
HI541WriteCmosSensor(0x0d00,0x1002);
HI541WriteCmosSensor(0x0d04,0x4B02);
HI541WriteCmosSensor(0x0d08,0xE801);
HI541WriteCmosSensor(0x0d0c,0x8301);
HI541WriteCmosSensor(0x0d10,0x3301);
HI541WriteCmosSensor(0x0d14,0xF700);
HI541WriteCmosSensor(0x0d18,0xC500);
HI541WriteCmosSensor(0x0d1c,0xA100);
HI541WriteCmosSensor(0x0d20,0x8A00);
HI541WriteCmosSensor(0x0d24,0x8300);
HI541WriteCmosSensor(0x0d28,0x8A00);
HI541WriteCmosSensor(0x0d2c,0xA100);
HI541WriteCmosSensor(0x0d30,0xC500);
HI541WriteCmosSensor(0x0d34,0xF700);
HI541WriteCmosSensor(0x0d38,0x3301);
HI541WriteCmosSensor(0x0d3c,0x8301);
HI541WriteCmosSensor(0x0d40,0xE801);
HI541WriteCmosSensor(0x0d44,0x4B02);
HI541WriteCmosSensor(0x0d48,0x8402);
HI541WriteCmosSensor(0x0d4c,0x3202);
HI541WriteCmosSensor(0x0d50,0xC001);
HI541WriteCmosSensor(0x0d54,0x6C01);
HI541WriteCmosSensor(0x0d58,0x2F01);
HI541WriteCmosSensor(0x0d5c,0xFF00);
HI541WriteCmosSensor(0x0d60,0xDC00);
HI541WriteCmosSensor(0x0d64,0xC800);
HI541WriteCmosSensor(0x0d68,0xC300);
HI541WriteCmosSensor(0x0d6c,0xC800);
HI541WriteCmosSensor(0x0d70,0xDC00);
HI541WriteCmosSensor(0x0d74,0xFF00);
HI541WriteCmosSensor(0x0d78,0x2F01);
HI541WriteCmosSensor(0x0d7c,0x6C01);
HI541WriteCmosSensor(0x0d80,0xC001);
HI541WriteCmosSensor(0x0d84,0x3202);
HI541WriteCmosSensor(0x0d88,0x8402);
HI541WriteCmosSensor(0x0d8c,0xA102);
HI541WriteCmosSensor(0x0d90,0x5F02);
HI541WriteCmosSensor(0x0d94,0xFA01);
HI541WriteCmosSensor(0x0d98,0xA201);
HI541WriteCmosSensor(0x0d9c,0x5D01);
HI541WriteCmosSensor(0x0da0,0x2F01);
HI541WriteCmosSensor(0x0da4,0x0E01);
HI541WriteCmosSensor(0x0da8,0xF900);
HI541WriteCmosSensor(0x0dac,0xF300);
HI541WriteCmosSensor(0x0db0,0xF900);
HI541WriteCmosSensor(0x0db4,0x0E01);
HI541WriteCmosSensor(0x0db8,0x2F01);
HI541WriteCmosSensor(0x0dbc,0x5D01);
HI541WriteCmosSensor(0x0dc0,0xA201);
HI541WriteCmosSensor(0x0dc4,0xFA01);
HI541WriteCmosSensor(0x0dc8,0x5F02);
HI541WriteCmosSensor(0x0dcc,0xA102);

HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x6400,0x0001);
HI541WriteCmosSensor(0x4828,0x0040);

//================================
//=========== LSC End ============
//================================

//================================
//========= OTP CFG Start ========
//================================
// >>>> OTP Timing Register for 24MHz
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x2c00,0x0007);//otp_ctl
HI541WriteCmosSensor(0x2c4c,0x8501);//otp_ip_cfg1
HI541WriteCmosSensor(0x2c4e,0x0900);//otp_ip_cfg2
HI541WriteCmosSensor(0x2c02,0x0000);//otp_clock register set
HI541WriteCmosSensor(0x2c40,0x1a00);//otp_timing register set for 24mhz
HI541WriteCmosSensor(0x2c42,0x1a1a);//otp_timing register set for 24mhz
HI541WriteCmosSensor(0x2c44,0x1a04);//otp_timing register set for 24mhz
HI541WriteCmosSensor(0x2c46,0x3407);//otp_timing register set for 24mhz
HI541WriteCmosSensor(0x2c48,0x0180);//otp_timing register set for 24mhz
HI541WriteCmosSensor(0x2c4a,0x1ad1);//otp_timing register set for 24mhz
//================================
//========= OTP CFG End ==========
//================================

//================================
//========= DMA ECC OTP Start ====
//================================
// >>>> DMA operation (OTP to Register Write Mode)
HI541WriteCmosSensor(0x4000,0x9404);//dma otp mode1 (otp to register write mode) enable
HI541WriteCmosSensor(0x4002,0x0000);//dma control
//HI541WriteCmosSensor(0x4020,0x0100);//dma interrupt pending clear

//// ECC OTP download start
HI541WriteCmosSensor(0x4004,0x801d);//dma src addr[7:0]//dma src addr[15:8]
HI541WriteCmosSensor(0x4006,0x0000);//dma src addr[23:16]//dma src addr[31:24]
HI541WriteCmosSensor(0x4000,0x9504);//dma start

mDELAY(1);//dma operation delay = 1ms

//HI541WriteCmosSensor(0x4000,0x9400);//dma otp mode1 (otp to register write mode) disable
//================================
//========= DMA ECC OTP End ======
//================================

//================================
//========= DMA LSC OTP Start ====
//================================
//// LSC OTP download start
HI541WriteCmosSensor(0x4004,0x4019);//dma src addr[7:0]//dma src addr[15:8]
HI541WriteCmosSensor(0x4006,0x0000);//dma src addr[23:16]//dma src addr[31:24]
HI541WriteCmosSensor(0x4000,0x9504);//dma start

mDELAY(1);//dma operation delay = 1ms

HI541WriteCmosSensor(0x4000,0x9400);//dma otp mode1 (otp to register write mode) disable
//================================
//========= DMA LSC OTP End ======
//================================

//================================
//========= SYSTEM Start ==========
//================================
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x3824,0x0000);
HI541WriteCmosSensor(0x3826,0x0100);// MCU initial wakeup cmd
//================================
//========= SYSTEM End   ==========
//================================

//================================
//=========== ADP Start ==========
//================================
HI541WriteCmosSensor(0xffff,0x0020);

HI541WriteCmosSensor(0x0DB0,0x0300);// ADP B[0]On/Off // B[1] DMA  On/Off // ADP Std Copy
HI541WriteCmosSensor(0x0DB2,0x0000);// ADP CTemp Copy
//HI541WriteCmosSensor(0x0DB4,0x0003);
HI541WriteCmosSensor(0x0DB6,0x0501);// AE, AWB?????? Std?? CTemp ????, CTemp?? RGain-BGain?????? ????
//HI541WriteCmosSensor(0x0DB8,0x050b);// Y Target, Contrast, Gamma, Y Offset Enable?? off ???? // CCM, Saturation, LSC Offset, LSC Gain, 1 LSC, 2 LSC, 3 LSC, CCM Gain Enable?? off ????                             
HI541WriteCmosSensor(0x0DB8,0x058b);// ADP MCMC ON Y Target, Contrast, Gamma, Y Offset Enable?? off ???? // CCM, Saturation, LSC Offset, LSC Gain, 1 LSC, 2 LSC, 3 LSC, CCM Gain Enable?? off ????                             

HI541WriteCmosSensor(0x0DCC,0x9CA8);// EV TH (Dark2) 6.67fps AGx8
HI541WriteCmosSensor(0x0DCE,0x0003);

HI541WriteCmosSensor(0x0DD0,0x4E54);// EV TH (Dark1) 6.67fps AGx4
HI541WriteCmosSensor(0x0DD2,0x8001);

HI541WriteCmosSensor(0x0DD4,0xf508);// EV TH (Indoor) 33.33fps AGx1.25
HI541WriteCmosSensor(0x0DD6,0x1800);

HI541WriteCmosSensor(0x0DD8,0x10a4);// EV TH (Outdoor) 1000fps AGx1
HI541WriteCmosSensor(0x0DDA,0x0000);

HI541WriteCmosSensor(0x0Dfc,0x0000);// EV
HI541WriteCmosSensor(0x0Dfe,0x0000);


HI541WriteCmosSensor(0x0DDC,0x4A3A);// Std High
HI541WriteCmosSensor(0x0DDE,0x1000);// Std Low
HI541WriteCmosSensor(0x0DE0,0x8B01);// CTemp  High
HI541WriteCmosSensor(0x0DE2,0x0D01);// CTemp Middle
HI541WriteCmosSensor(0x0DE4,0xe000);// CTemp Low


// Y Target
HI541WriteCmosSensor(0x0E00,0x3030);// 0?? ????
HI541WriteCmosSensor(0x0E02,0x3030);// 2?? ????
HI541WriteCmosSensor(0x0E04,0x3030);// 4?? ????
HI541WriteCmosSensor(0x0E06,0x3030);// 6?? ????	// ????
HI541WriteCmosSensor(0x0E08,0x3030);// 8?? ????
HI541WriteCmosSensor(0x0E0A,0x3030);// 10?? ????

// Y TargetOffset
HI541WriteCmosSensor(0x0E0C,0x8080);// 0?? ????
HI541WriteCmosSensor(0x0E0E,0x8080);// 2?? ????
HI541WriteCmosSensor(0x0E10,0x8080);// 4?? ????
HI541WriteCmosSensor(0x0E12,0x8080);// 6?? ????
HI541WriteCmosSensor(0x0E14,0x8080);// 8?? ????
HI541WriteCmosSensor(0x0E16,0x8080);// 10?? ????

// Contrast
HI541WriteCmosSensor(0x0E18,0x9090);// 0?? ????
HI541WriteCmosSensor(0x0E1A,0x9090);// 2?? ????
HI541WriteCmosSensor(0x0E1C,0x9090);// 4?? ????
HI541WriteCmosSensor(0x0E1E,0x9090);// 6?? ????
HI541WriteCmosSensor(0x0E20,0x9090);// 8?? ????
HI541WriteCmosSensor(0x0E22,0x9090);// 10?? ????

// GAMMA 0
HI541WriteCmosSensor(0x0E24,0x0005);
HI541WriteCmosSensor(0x0E26,0x0D16);
HI541WriteCmosSensor(0x0E28,0x1F30);
HI541WriteCmosSensor(0x0E2A,0x3F4D);
HI541WriteCmosSensor(0x0E2C,0x5862);
HI541WriteCmosSensor(0x0E2E,0x6A71);
HI541WriteCmosSensor(0x0E30,0x777D);
HI541WriteCmosSensor(0x0E32,0x8388);
HI541WriteCmosSensor(0x0E34,0x8C91);
HI541WriteCmosSensor(0x0E36,0x9498);
HI541WriteCmosSensor(0x0E38,0x9CA2);
HI541WriteCmosSensor(0x0E3A,0xA9AF);
HI541WriteCmosSensor(0x0E3C,0xBAC4);
HI541WriteCmosSensor(0x0E3E,0xCDD6);
HI541WriteCmosSensor(0x0E40,0xDEE5);
HI541WriteCmosSensor(0x0E42,0xECF3);
HI541WriteCmosSensor(0x0E44,0xF9FF);

// GAMMA 1     
HI541WriteCmosSensor(0x0E46,0x0005);
HI541WriteCmosSensor(0x0E48,0x0D16);
HI541WriteCmosSensor(0x0E4A,0x1F30);
HI541WriteCmosSensor(0x0E4C,0x3F4D);
HI541WriteCmosSensor(0x0E4E,0x5862);
HI541WriteCmosSensor(0x0E50,0x6A71);
HI541WriteCmosSensor(0x0E52,0x777D);
HI541WriteCmosSensor(0x0E54,0x8388);
HI541WriteCmosSensor(0x0E56,0x8C91);
HI541WriteCmosSensor(0x0E58,0x9498);
HI541WriteCmosSensor(0x0E5A,0x9CA2);
HI541WriteCmosSensor(0x0E5C,0xA9AF);
HI541WriteCmosSensor(0x0E5E,0xBAC4);
HI541WriteCmosSensor(0x0E60,0xCDD6);
HI541WriteCmosSensor(0x0E62,0xDEE5);
HI541WriteCmosSensor(0x0E64,0xECF3);
HI541WriteCmosSensor(0x0E66,0xF9FF);

// GAMMA 2     
HI541WriteCmosSensor(0x0E68,0x0005);
HI541WriteCmosSensor(0x0E6A,0x0D16);
HI541WriteCmosSensor(0x0E6C,0x1F30);
HI541WriteCmosSensor(0x0E6E,0x3F4D);
HI541WriteCmosSensor(0x0E70,0x5862);
HI541WriteCmosSensor(0x0E72,0x6A71);
HI541WriteCmosSensor(0x0E74,0x777D);
HI541WriteCmosSensor(0x0E76,0x8388);
HI541WriteCmosSensor(0x0E78,0x8C91);
HI541WriteCmosSensor(0x0E7A,0x9498);
HI541WriteCmosSensor(0x0E7C,0x9CA2);
HI541WriteCmosSensor(0x0E7E,0xA9AF);
HI541WriteCmosSensor(0x0E80,0xBAC4);
HI541WriteCmosSensor(0x0E82,0xCDD6);
HI541WriteCmosSensor(0x0E84,0xDEE5);
HI541WriteCmosSensor(0x0E86,0xECF3);
HI541WriteCmosSensor(0x0E88,0xF9FF);

// GAMMA 3    
HI541WriteCmosSensor(0x0E8A,0x0002);
HI541WriteCmosSensor(0x0E8C,0x0408);
HI541WriteCmosSensor(0x0E8E,0x0c1e);
HI541WriteCmosSensor(0x0E90,0x3040);
HI541WriteCmosSensor(0x0E92,0x4b54);
HI541WriteCmosSensor(0x0E94,0x5e66);
HI541WriteCmosSensor(0x0E96,0x6e76);
HI541WriteCmosSensor(0x0E98,0x7e86);
HI541WriteCmosSensor(0x0E9A,0x8d93);
HI541WriteCmosSensor(0x0E9C,0x9aa1);
HI541WriteCmosSensor(0x0E9E,0xa6b1);
HI541WriteCmosSensor(0x0EA0,0xbac4);
HI541WriteCmosSensor(0x0EA2,0xd3e1);
HI541WriteCmosSensor(0x0EA4,0xeaf2);
HI541WriteCmosSensor(0x0EA6,0xf6fa);
HI541WriteCmosSensor(0x0EA8,0xfcfe);
HI541WriteCmosSensor(0x0EAA,0xffff);

// GAMMA 4    
HI541WriteCmosSensor(0x0EAC,0x0002);
HI541WriteCmosSensor(0x0EAE,0x0408);
HI541WriteCmosSensor(0x0EB0,0x0c1e);
HI541WriteCmosSensor(0x0EB2,0x3040);
HI541WriteCmosSensor(0x0EB4,0x4b54);
HI541WriteCmosSensor(0x0EB6,0x5e66);
HI541WriteCmosSensor(0x0EB8,0x6e76);
HI541WriteCmosSensor(0x0EBA,0x7e86);
HI541WriteCmosSensor(0x0EBC,0x8d93);
HI541WriteCmosSensor(0x0EBE,0x9aa1);
HI541WriteCmosSensor(0x0EC0,0xa6b1);
HI541WriteCmosSensor(0x0EC2,0xbac4);
HI541WriteCmosSensor(0x0EC4,0xd3e1);
HI541WriteCmosSensor(0x0EC6,0xeaf2);
HI541WriteCmosSensor(0x0EC8,0xf6fa);
HI541WriteCmosSensor(0x0ECA,0xfcfe);
HI541WriteCmosSensor(0x0ECC,0xffff);

// GAMMA 5    
HI541WriteCmosSensor(0x0ECE,0x0002);
HI541WriteCmosSensor(0x0ED0,0x0408);
HI541WriteCmosSensor(0x0ED2,0x0c1e);
HI541WriteCmosSensor(0x0ED4,0x3040);
HI541WriteCmosSensor(0x0ED6,0x4b54);
HI541WriteCmosSensor(0x0ED8,0x5e66);
HI541WriteCmosSensor(0x0EDA,0x6e76);
HI541WriteCmosSensor(0x0EDC,0x7e86);
HI541WriteCmosSensor(0x0EDE,0x8d93);
HI541WriteCmosSensor(0x0EE0,0x9aa1);
HI541WriteCmosSensor(0x0EE2,0xa6b1);
HI541WriteCmosSensor(0x0EE4,0xbac4);
HI541WriteCmosSensor(0x0EE6,0xd3e1);
HI541WriteCmosSensor(0x0EE8,0xeaf2);
HI541WriteCmosSensor(0x0EEA,0xf6fa);
HI541WriteCmosSensor(0x0EEC,0xfcfe);
HI541WriteCmosSensor(0x0EEE,0xffff);

// GAMMA 6    
HI541WriteCmosSensor(0x0EF0,0x0002);
HI541WriteCmosSensor(0x0EF2,0x0408);
HI541WriteCmosSensor(0x0EF4,0x0c1e);
HI541WriteCmosSensor(0x0EF6,0x3040);
HI541WriteCmosSensor(0x0EF8,0x4b54);
HI541WriteCmosSensor(0x0EFA,0x5e66);
HI541WriteCmosSensor(0x0EFC,0x6e76);
HI541WriteCmosSensor(0x0EFE,0x7e86);
HI541WriteCmosSensor(0x0F00,0x8d93);
HI541WriteCmosSensor(0x0F02,0x9aa1);
HI541WriteCmosSensor(0x0F04,0xa6b1);
HI541WriteCmosSensor(0x0F06,0xbac4);
HI541WriteCmosSensor(0x0F08,0xd3e1);
HI541WriteCmosSensor(0x0F0A,0xeaf2);
HI541WriteCmosSensor(0x0F0C,0xf6fa);
HI541WriteCmosSensor(0x0F0E,0xfcfe);
HI541WriteCmosSensor(0x0F10,0xffff);

// GAMMA 7    
HI541WriteCmosSensor(0x0F12,0x0002);
HI541WriteCmosSensor(0x0F14,0x0408);
HI541WriteCmosSensor(0x0F16,0x0c1e);
HI541WriteCmosSensor(0x0F18,0x3040);
HI541WriteCmosSensor(0x0F1A,0x4b54);
HI541WriteCmosSensor(0x0F1C,0x5e66);
HI541WriteCmosSensor(0x0F1E,0x6e76);
HI541WriteCmosSensor(0x0F20,0x7e86);
HI541WriteCmosSensor(0x0F22,0x8d93);
HI541WriteCmosSensor(0x0F24,0x9aa1);
HI541WriteCmosSensor(0x0F26,0xa6b1);
HI541WriteCmosSensor(0x0F28,0xbac4);
HI541WriteCmosSensor(0x0F2A,0xd3e1);
HI541WriteCmosSensor(0x0F2C,0xeaf2);
HI541WriteCmosSensor(0x0F2E,0xf6fa);
HI541WriteCmosSensor(0x0F30,0xfcfe);
HI541WriteCmosSensor(0x0F32,0xffff);

// GAMMA 8    
HI541WriteCmosSensor(0x0F34,0x0002);
HI541WriteCmosSensor(0x0F36,0x0408);
HI541WriteCmosSensor(0x0F38,0x0c1e);
HI541WriteCmosSensor(0x0F3A,0x3040);
HI541WriteCmosSensor(0x0F3C,0x4b54);
HI541WriteCmosSensor(0x0F3E,0x5e66);
HI541WriteCmosSensor(0x0F40,0x6e76);
HI541WriteCmosSensor(0x0F42,0x7e86);
HI541WriteCmosSensor(0x0F44,0x8d93);
HI541WriteCmosSensor(0x0F46,0x9aa1);
HI541WriteCmosSensor(0x0F48,0xa6b1);
HI541WriteCmosSensor(0x0F4A,0xbac4);
HI541WriteCmosSensor(0x0F4C,0xd3e1);
HI541WriteCmosSensor(0x0F4E,0xeaf2);
HI541WriteCmosSensor(0x0F50,0xf6fa);
HI541WriteCmosSensor(0x0F52,0xfcfe);
HI541WriteCmosSensor(0x0F54,0xffff);

// GAMMA 9(Outdoor1)   
HI541WriteCmosSensor(0x0F56,0x0002);
HI541WriteCmosSensor(0x0F58,0x0408);
HI541WriteCmosSensor(0x0F5A,0x0c1e);
HI541WriteCmosSensor(0x0F5C,0x3040);
HI541WriteCmosSensor(0x0F5E,0x4b54);
HI541WriteCmosSensor(0x0F60,0x5e66);
HI541WriteCmosSensor(0x0F62,0x6e76);
HI541WriteCmosSensor(0x0F64,0x7e86);
HI541WriteCmosSensor(0x0F66,0x8d93);
HI541WriteCmosSensor(0x0F68,0x9aa1);
HI541WriteCmosSensor(0x0F6A,0xa6b1);
HI541WriteCmosSensor(0x0F6C,0xbac4);
HI541WriteCmosSensor(0x0F6E,0xd3e1);
HI541WriteCmosSensor(0x0F70,0xeaf2);
HI541WriteCmosSensor(0x0F72,0xf6fa);
HI541WriteCmosSensor(0x0F74,0xfcfe);
HI541WriteCmosSensor(0x0F76,0xffff);

// GAMMA 10 (Outdoor2)    
HI541WriteCmosSensor(0x0F78,0x0002);
HI541WriteCmosSensor(0x0F7A,0x0408);
HI541WriteCmosSensor(0x0F7C,0x0c1e);
HI541WriteCmosSensor(0x0F7E,0x3040);
HI541WriteCmosSensor(0x0F80,0x4b54);
HI541WriteCmosSensor(0x0F82,0x5e66);
HI541WriteCmosSensor(0x0F84,0x6e76);
HI541WriteCmosSensor(0x0F86,0x7e86);
HI541WriteCmosSensor(0x0F88,0x8d93);
HI541WriteCmosSensor(0x0F8A,0x9aa1);
HI541WriteCmosSensor(0x0F8C,0xa6b1);
HI541WriteCmosSensor(0x0F8E,0xbac4);
HI541WriteCmosSensor(0x0F90,0xd3e1);
HI541WriteCmosSensor(0x0F92,0xeaf2);
HI541WriteCmosSensor(0x0F94,0xf6fa);
HI541WriteCmosSensor(0x0F96,0xfcfe);
HI541WriteCmosSensor(0x0F98,0xffff);

// GAMMA 11  (Outdoor2)  
HI541WriteCmosSensor(0x0F9A,0x0002);
HI541WriteCmosSensor(0x0F9C,0x0408);
HI541WriteCmosSensor(0x0F9E,0x0c1e);
HI541WriteCmosSensor(0x0FA0,0x3040);
HI541WriteCmosSensor(0x0FA2,0x4b54);
HI541WriteCmosSensor(0x0FA4,0x5e66);
HI541WriteCmosSensor(0x0FA6,0x6e76);
HI541WriteCmosSensor(0x0FA8,0x7e86);
HI541WriteCmosSensor(0x0FAA,0x8d93);
HI541WriteCmosSensor(0x0FAC,0x9aa1);
HI541WriteCmosSensor(0x0FAE,0xa6b1);
HI541WriteCmosSensor(0x0FB0,0xbac4);
HI541WriteCmosSensor(0x0FB2,0xd3e1);
HI541WriteCmosSensor(0x0FB4,0xeaf2);
HI541WriteCmosSensor(0x0FB6,0xf6fa);
HI541WriteCmosSensor(0x0FB8,0xfcfe);
HI541WriteCmosSensor(0x0FBA,0xffff);

// OFFSET
HI541WriteCmosSensor(0x0FBC,0x0000);// 0?? ???? Offset
HI541WriteCmosSensor(0x0FBE,0x0000);// 2?? ???? Offset
HI541WriteCmosSensor(0x0FC0,0x0000);// 4?? ???? Offset
HI541WriteCmosSensor(0x0FC2,0x0000);// 6?? ???? Offset
HI541WriteCmosSensor(0x0FC4,0x0000);// 8?? ???? Offset
HI541WriteCmosSensor(0x0FC6,0x0000);// 10?? ???? Offset

// CB Saturation
HI541WriteCmosSensor(0x0FC8,0x8080);// 0?? ???? CB Saturation
		HI541WriteCmosSensor(0x0FCA,0x8088);// 2?? ???? CB Saturation
		HI541WriteCmosSensor(0x0FCC,0x9090);// 4?? ???? CB Saturation
HI541WriteCmosSensor(0x0FCE,0x949a);// 6?? ???? CB Saturation
HI541WriteCmosSensor(0x0FD0,0x9ca0);// 8?? ???? CB Saturation
HI541WriteCmosSensor(0x0FD2,0xa0a0);// 10?? ???? CB Saturation

// CR Saturation
HI541WriteCmosSensor(0x0FD4,0x8080);// 0?? ???? CR Saturation
		HI541WriteCmosSensor(0x0FD6,0x8088);// 2?? ???? CR Saturation
		HI541WriteCmosSensor(0x0FD8,0x9090);// 4?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDA,0x9498);// 6?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDC,0x8ca0);// 8?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDE,0xa0a0);// 10?? ???? CR Saturation

// CMC 0?? ????, Low light
HI541WriteCmosSensor(0x0FE0,0x2D73);//CMC sign bit,CMC00
HI541WriteCmosSensor(0x0FE2,0x3b08);// CMC01,CMC02      
HI541WriteCmosSensor(0x0FE4,0x0e66);// CMC10,CMC11                
HI541WriteCmosSensor(0x0FE6,0x1804);// CMC12,CMC20                
HI541WriteCmosSensor(0x0FE8,0x306c);// CMC21,CMC22                

// CMC 1?? ????, Low light
HI541WriteCmosSensor(0x0FEA,0x2D73);//CMC sign bit,CMC00
HI541WriteCmosSensor(0x0FEC,0x3b08);// CMC01,CMC02      
HI541WriteCmosSensor(0x0FEE,0x0e66);// CMC10,CMC11                 
HI541WriteCmosSensor(0x0FF0,0x1804);// CMC12,CMC20                 
HI541WriteCmosSensor(0x0FF2,0x306c);// CMC21,CMC22                 

// CMC 2?? ????, Low light
HI541WriteCmosSensor(0x0FF4,0x2D73);//CMC sign bit,CMC00
HI541WriteCmosSensor(0x0FF6,0x3b08);// CMC01,CMC02      
HI541WriteCmosSensor(0x0FF8,0x0e66);// CMC10,CMC11                 
HI541WriteCmosSensor(0x0FFA,0x1804);// CMC12,CMC20                 
HI541WriteCmosSensor(0x0FFC,0x306c);// CMC21,CMC22                 

// CMC 3?? ????, D65
HI541WriteCmosSensor(0x0FFE,0x2D73);//CMC sign bit,CMC00
HI541WriteCmosSensor(0x1000,0x3b08);// CMC01,CMC02      
HI541WriteCmosSensor(0x1002,0x0f66);// CMC10,CMC11                
HI541WriteCmosSensor(0x1004,0x160a);// CMC12,CMC20                
HI541WriteCmosSensor(0x1006,0x366c);// CMC21,CMC22                

// CMC 4?? ????, TL84
HI541WriteCmosSensor(0x1008,0x3d6f);//CMC sign bit,CMC00
HI541WriteCmosSensor(0x100A,0x2d02);// CMC01,CMC02      
HI541WriteCmosSensor(0x100C,0x1c5f);// CMC10,CMC11                 
HI541WriteCmosSensor(0x100E,0x0305);// CMC12,CMC20                 
HI541WriteCmosSensor(0x1010,0x2f6a);// CMC21,CMC22                 

// CMC 5?? ????, A
HI541WriteCmosSensor(0x1012,0x2D73);//CMC sign bit,CMC00
HI541WriteCmosSensor(0x1014,0x3b08);// CMC01,CMC02      
HI541WriteCmosSensor(0x1016,0x0e66);// CMC10,CMC11               
HI541WriteCmosSensor(0x1018,0x1804);// CMC12,CMC20               
HI541WriteCmosSensor(0x101A,0x306c);// CMC21,CMC22               

// CMC 6?? ????, D65
HI541WriteCmosSensor(0x101C,0x2D73);//CMC sign bit,CMC00
HI541WriteCmosSensor(0x101E,0x3b08);// CMC01,CMC02      
HI541WriteCmosSensor(0x1020,0x0f66);// CMC10,CMC11               
HI541WriteCmosSensor(0x1022,0x160a);// CMC12,CMC20               
HI541WriteCmosSensor(0x1024,0x366c);// CMC21,CMC22               

// CMC 7?? ????, TL84
HI541WriteCmosSensor(0x1026,0x3d6f);//CMC sign bit,CMC00
HI541WriteCmosSensor(0x1028,0x2d02);// CMC01,CMC02      
HI541WriteCmosSensor(0x102A,0x1c5f);// CMC10,CMC11            
HI541WriteCmosSensor(0x102C,0x0305);// CMC12,CMC20            
HI541WriteCmosSensor(0x102E,0x2f6a);// CMC21,CMC22            

// CMC 8?? ????, A
HI541WriteCmosSensor(0x1030,0x3f69);//CMC sign bit,CMC00
HI541WriteCmosSensor(0x1032,0x2702);// CMC01,CMC02      
HI541WriteCmosSensor(0x1034,0x2172);// CMC10,CMC11           
HI541WriteCmosSensor(0x1036,0x1109);// CMC12,CMC20           
HI541WriteCmosSensor(0x1038,0x3f88);// CMC21,CMC22                 

// CMC 9?? ????
HI541WriteCmosSensor(0x103A,0x2D72);//CMC sign bit
HI541WriteCmosSensor(0x103C,0x3402);// CMC01
HI541WriteCmosSensor(0x103E,0x1267);// CMC10                 
HI541WriteCmosSensor(0x1040,0x1505);// CMC12                 
HI541WriteCmosSensor(0x1042,0x2b66);// CMC21                 

// CMC 10?? ????
HI541WriteCmosSensor(0x1044,0x2D72);//CMC sign bit
HI541WriteCmosSensor(0x1046,0x3402);// CMC01
HI541WriteCmosSensor(0x1048,0x1267);// CMC10                 
HI541WriteCmosSensor(0x104A,0x1505);// CMC12                 
HI541WriteCmosSensor(0x104C,0x2b66);// CMC21                 

// CMC 11?? ????
HI541WriteCmosSensor(0x104E,0x2D72);
HI541WriteCmosSensor(0x1050,0x3402);
HI541WriteCmosSensor(0x1052,0x1267);
HI541WriteCmosSensor(0x1054,0x1505);
HI541WriteCmosSensor(0x1056,0x2b66);

// LSC 0?? ????               
HI541WriteCmosSensor(0x1088,0x3838);//Gr,R         
HI541WriteCmosSensor(0x108a,0x3838);//B,Gb         
               
// LSC 1?? ????               
HI541WriteCmosSensor(0x108c,0x3838);               
HI541WriteCmosSensor(0x108e,0x3838);               
               
// LSC 2?? ????               
HI541WriteCmosSensor(0x1090,0x3838);               
HI541WriteCmosSensor(0x1092,0x3838);               

// LSC 3?? ????
HI541WriteCmosSensor(0x1094,0x8080);
HI541WriteCmosSensor(0x1096,0x8080);

// LSC 4?? ????
HI541WriteCmosSensor(0x1098,0x8080);
HI541WriteCmosSensor(0x109a,0x8080);

// LSC 5?? ????
HI541WriteCmosSensor(0x109c,0x8080);
HI541WriteCmosSensor(0x109e,0x8080);

// LSC 6?? ????
HI541WriteCmosSensor(0x10a0,0x8080);
HI541WriteCmosSensor(0x10a2,0x8080);

// LSC 7?? ????
HI541WriteCmosSensor(0x10a4,0x807a);
HI541WriteCmosSensor(0x10a6,0x8080);

// LSC 8?? ????
HI541WriteCmosSensor(0x10a8,0x8088);//Gr,R
HI541WriteCmosSensor(0x10aa,0x7a80);//B, Gb

// LSC 9?? ????
HI541WriteCmosSensor(0x10ac,0x8080);
HI541WriteCmosSensor(0x10ae,0x8080);

// LSC 10?? ????
HI541WriteCmosSensor(0x10b0,0x8080);
HI541WriteCmosSensor(0x10b2,0x8080);

// LSC 11?? ????
HI541WriteCmosSensor(0x10b4,0x8080);
HI541WriteCmosSensor(0x10b6,0x8080);
       
if (0x00AE == HI541ReadCmosSensor(0x0002))
{
/////////// MCMC x1 setting
HI541WriteCmosSensor(0x19e6,0x2132);// 1 Magen Delta, Center 
HI541WriteCmosSensor(0x19e8,0x0074);// 2 Skin Delta, Center      
HI541WriteCmosSensor(0x19ea,0x1ba7);// 3 Yellow Delta, Center  
HI541WriteCmosSensor(0x19ec,0x00e4);// 4 Green Delta, Center   
HI541WriteCmosSensor(0x19ee,0xad32);// 5 Cyan Delta, Center    
HI541WriteCmosSensor(0x19f0,0x8063);// 6 Blue Delta, Center    
HI541WriteCmosSensor(0x19f2,0x4040);// 1,2 SAT                 
HI541WriteCmosSensor(0x19f4,0x4040);// 3,4 SAT                 
HI541WriteCmosSensor(0x19f6,0x4040);// 5,6 SAT                 
HI541WriteCmosSensor(0x19f8,0x0000);// 1,2 HUE                 
HI541WriteCmosSensor(0x19fa,0x0000);// 3,4 HUE                 
HI541WriteCmosSensor(0x19fc,0x0000);// 5,6 HUE                 
                    
//region 1        
HI541WriteCmosSensor(0x4110,0x2132);// 1 Magen Delta, Center            
HI541WriteCmosSensor(0x4112,0x0074);// 2 Skin Delta, Center              
HI541WriteCmosSensor(0x4114,0x1ba7);// 3 Yellow Delta, Center            
HI541WriteCmosSensor(0x4116,0x00e4);// 4 Green Delta, Center             
HI541WriteCmosSensor(0x4118,0xad32);// 5 Cyan Delta, Center              
HI541WriteCmosSensor(0x411A,0x8063);// 6 Blue Delta, Center              
HI541WriteCmosSensor(0x411C,0x4040);// 1,2 SAT                           
HI541WriteCmosSensor(0x411E,0x4040);// 3,4 SAT                           
HI541WriteCmosSensor(0x4120,0x4040);// 5,6 SAT                           
HI541WriteCmosSensor(0x4122,0x0000);// 1,2 HUE                           
HI541WriteCmosSensor(0x4124,0x0000);// 3,4 HUE                           
HI541WriteCmosSensor(0x4126,0x0000);// 5,6 HUE                           
                   
//region 2        
HI541WriteCmosSensor(0x4184,0x1021);
HI541WriteCmosSensor(0x4186,0x3200); 
HI541WriteCmosSensor(0x4188,0x741b);
HI541WriteCmosSensor(0x418A,0xa700);
HI541WriteCmosSensor(0x418C,0xe4ad);
HI541WriteCmosSensor(0x418E,0x3280);
HI541WriteCmosSensor(0x4190,0x6340);// x,1 SAT
HI541WriteCmosSensor(0x4192,0x4040);// 2,3 SAT                
HI541WriteCmosSensor(0x4194,0x4040);// 4,5 SAT  
HI541WriteCmosSensor(0x4196,0x4000);// 6,1 HUE  
HI541WriteCmosSensor(0x4198,0x0000);// 2,3 HUE  
HI541WriteCmosSensor(0x419A,0x0000);// 4,5 HUE  
HI541WriteCmosSensor(0x419C,0x0000);// 6,x      

//region 3          
HI541WriteCmosSensor(0x17ee,0x1e21);
HI541WriteCmosSensor(0x17f0,0x3200);
HI541WriteCmosSensor(0x17f2,0x741b);
HI541WriteCmosSensor(0x17f4,0xa71e);
HI541WriteCmosSensor(0x17f6,0xe4ad);
HI541WriteCmosSensor(0x17f8,0x3280);
HI541WriteCmosSensor(0x17fa,0x6340);// x,1 SAT
HI541WriteCmosSensor(0x17fc,0x4040);// 2,3 SAT
HI541WriteCmosSensor(0x17fe,0x4040);// 4,5 SAT   
HI541WriteCmosSensor(0x1800,0x4000);// 6,1 HUE   
HI541WriteCmosSensor(0x1802,0x0000);// 2,3 HUE   
HI541WriteCmosSensor(0x1804,0x1000);// 4,5 HUE   
HI541WriteCmosSensor(0x1806,0x0000);// 6,x       
                     
//region 4           
HI541WriteCmosSensor(0x41FA,0x2132);
HI541WriteCmosSensor(0x41FC,0x0074);
HI541WriteCmosSensor(0x41FE,0x1ba7);
HI541WriteCmosSensor(0x4200,0x1ee4);
HI541WriteCmosSensor(0x4202,0xad32);
HI541WriteCmosSensor(0x4204,0x8063);
HI541WriteCmosSensor(0x4206,0x4040);// 1,2 SAT
		HI541WriteCmosSensor(0x4208,0x4040);// 3,4 SAT
HI541WriteCmosSensor(0x420A,0x4040);// 5,6 SAT  
HI541WriteCmosSensor(0x420C,0x0000);// 1,2 HUE  
HI541WriteCmosSensor(0x420E,0x0010);// 3,4 HUE  
HI541WriteCmosSensor(0x4210,0x0000);// 5,6 HUE  
                    
//region 5          
HI541WriteCmosSensor(0x426E,0x1021);
HI541WriteCmosSensor(0x4270,0x3200);
HI541WriteCmosSensor(0x4272,0x741b); 
HI541WriteCmosSensor(0x4274,0xa71e);
HI541WriteCmosSensor(0x4276,0xe4ad);
HI541WriteCmosSensor(0x4278,0x3280);
HI541WriteCmosSensor(0x427A,0x6340);// x,1 SAT 
HI541WriteCmosSensor(0x427C,0x4040);// 2,3 SAT 
HI541WriteCmosSensor(0x427E,0x4040);// 4,5 SAT   
HI541WriteCmosSensor(0x4280,0x4000);// 6,1 HUE   
HI541WriteCmosSensor(0x4282,0x0000);// 2,3 HUE   
HI541WriteCmosSensor(0x4284,0x1000);// 4,5 HUE   
HI541WriteCmosSensor(0x4286,0x0000);// 6,x       
                    
//region 6          
HI541WriteCmosSensor(0x15F8,0x216e);
HI541WriteCmosSensor(0x15FA,0x0074);
HI541WriteCmosSensor(0x15FC,0x1ba7);
HI541WriteCmosSensor(0x15FE,0x1eda);
HI541WriteCmosSensor(0x1600,0xad32);
HI541WriteCmosSensor(0x1602,0x8063);
HI541WriteCmosSensor(0x1604,0x2d40);// 1,2 SAT 
HI541WriteCmosSensor(0x1606,0x4040);// 3,4 SAT 
HI541WriteCmosSensor(0x1608,0x4040);// 5,6 SAT  
HI541WriteCmosSensor(0x160A,0x0e00);// 1,2 HUE  
HI541WriteCmosSensor(0x160C,0x0010);// 3,4 HUE  
HI541WriteCmosSensor(0x160E,0x0000);// 5,6 HUE  
                   
//region 7         
HI541WriteCmosSensor(0x42E4,0x216e);
HI541WriteCmosSensor(0x42E6,0x0074);
HI541WriteCmosSensor(0x42E8,0x1ba7);
HI541WriteCmosSensor(0x42EA,0x1eda);
HI541WriteCmosSensor(0x42EC,0xad32);
HI541WriteCmosSensor(0x42EE,0x8063);
HI541WriteCmosSensor(0x42F0,0x2d40);// 1,2 SAT
HI541WriteCmosSensor(0x42F2,0x4040);// 3,4 SAT
HI541WriteCmosSensor(0x42F4,0x4040);// 5,6 SAT 
HI541WriteCmosSensor(0x42F6,0x0e00);// 1,2 HUE 
HI541WriteCmosSensor(0x42F8,0x0010);// 3,4 HUE 
HI541WriteCmosSensor(0x42FA,0x0000);// 5,6 HUE 
                   
//region 8         
HI541WriteCmosSensor(0x4358,0x1021);
HI541WriteCmosSensor(0x435A,0x6e00);
HI541WriteCmosSensor(0x435C,0x741b); 
HI541WriteCmosSensor(0x435E,0xa71e);
HI541WriteCmosSensor(0x4360,0xdaad);
HI541WriteCmosSensor(0x4362,0x3280);
HI541WriteCmosSensor(0x4364,0x6336);// x,1 SAT
HI541WriteCmosSensor(0x4366,0x4040);// 2,3 SAT
HI541WriteCmosSensor(0x4368,0x4040);// 4,5 SAT  
HI541WriteCmosSensor(0x436A,0x4010);// 6 SAT, 1 HUE  
HI541WriteCmosSensor(0x436C,0x0000);// 2,3 HUE  
HI541WriteCmosSensor(0x436E,0x1000);// 4,5 HUE  
HI541WriteCmosSensor(0x4370,0x0000);// 6,x      
                    
//region 9          
HI541WriteCmosSensor(0x1400,0x1e21);
HI541WriteCmosSensor(0x1402,0x3200);
HI541WriteCmosSensor(0x1404,0x741b);
HI541WriteCmosSensor(0x1406,0xa71e);
HI541WriteCmosSensor(0x1408,0xe4ad);
HI541WriteCmosSensor(0x140A,0x3280);
HI541WriteCmosSensor(0x140C,0x6340);
		HI541WriteCmosSensor(0x140E,0x4040);// 2,3 SAT
		HI541WriteCmosSensor(0x1410,0x4040);// 4,5 SAT
		HI541WriteCmosSensor(0x1412,0x4000);// 6 SAT, 1 HUE 
		HI541WriteCmosSensor(0x1414,0x0000);// 2,3 HUE  
HI541WriteCmosSensor(0x1416,0x1000);// 4,5 HUE  
HI541WriteCmosSensor(0x1418,0x0000);
                  
//region 10       
HI541WriteCmosSensor(0x43CE,0x2132);
HI541WriteCmosSensor(0x43D0,0x0074);
HI541WriteCmosSensor(0x43D2,0x1ba7);
HI541WriteCmosSensor(0x43D4,0x1ee4);
HI541WriteCmosSensor(0x43D6,0xad32);
HI541WriteCmosSensor(0x43D8,0x8063);
HI541WriteCmosSensor(0x43DA,0x4040);
		HI541WriteCmosSensor(0x43DC,0x4040);// 3,4 SAT
		HI541WriteCmosSensor(0x43DE,0x4040);// 5,6 SAT
		HI541WriteCmosSensor(0x43E0,0x0000);// 1,2 HUE 
HI541WriteCmosSensor(0x43E2,0x0010);// 3,4 HUE 
		HI541WriteCmosSensor(0x43E4,0x0000);// 5,6 HUE 
                  
//region 11       
HI541WriteCmosSensor(0x4442,0x1021);
HI541WriteCmosSensor(0x4444,0x3200);
HI541WriteCmosSensor(0x4446,0x741b);
HI541WriteCmosSensor(0x4448,0xa71e);
HI541WriteCmosSensor(0x444A,0xe4ad);
HI541WriteCmosSensor(0x444C,0x3280);
HI541WriteCmosSensor(0x444E,0x6340);
		HI541WriteCmosSensor(0x4450,0x4040);// 2,3 SAT
		HI541WriteCmosSensor(0x4452,0x4040);// 4,5 SAT
		HI541WriteCmosSensor(0x4454,0x4000);// 6 SAT, 1 HUE 
		HI541WriteCmosSensor(0x4456,0x0000);// 2,3 HUE
HI541WriteCmosSensor(0x4458,0x1000);// 4,5 HUE  
HI541WriteCmosSensor(0x445A,0x0000);// 6 HUE,x   

}

else {  			
	/////////// MCMC x1 setting
HI541WriteCmosSensor(0x19e6,0x2132);// 1 Magen Delta, Center 
HI541WriteCmosSensor(0x19e8,0x0074);// 2 Skin Delta, Center      
HI541WriteCmosSensor(0x19ea,0x1ba7);// 3 Yellow Delta, Center  
HI541WriteCmosSensor(0x19ec,0x00e4);// 4 Green Delta, Center   
HI541WriteCmosSensor(0x19ee,0xad32);// 5 Cyan Delta, Center    
HI541WriteCmosSensor(0x19f0,0x8063);// 6 Blue Delta, Center    
HI541WriteCmosSensor(0x19f2,0x4040);// 1,2 SAT                 
HI541WriteCmosSensor(0x19f4,0x4040);// 3,4 SAT                 
HI541WriteCmosSensor(0x19f6,0x4040);// 5,6 SAT                 
HI541WriteCmosSensor(0x19f8,0x0000);// 1,2 HUE                 
HI541WriteCmosSensor(0x19fa,0x0000);// 3,4 HUE                 
HI541WriteCmosSensor(0x19fc,0x0000);// 5,6 HUE                 

//region 1        
HI541WriteCmosSensor(0x31e0,0x2132);// 1 Magen Delta, Center            
HI541WriteCmosSensor(0x31e2,0x0074);// 2 Skin Delta, Center              
HI541WriteCmosSensor(0x31e4,0x1ba7);// 3 Yellow Delta, Center            
HI541WriteCmosSensor(0x31e6,0x00e4);// 4 Green Delta, Center             
HI541WriteCmosSensor(0x31e8,0xad32);// 5 Cyan Delta, Center              
HI541WriteCmosSensor(0x31eA,0x8063);// 6 Blue Delta, Center              
HI541WriteCmosSensor(0x31eC,0x4040);// 1,2 SAT                           
HI541WriteCmosSensor(0x31eE,0x4040);// 3,4 SAT                           
HI541WriteCmosSensor(0x31f0,0x4040);// 5,6 SAT                           
HI541WriteCmosSensor(0x31f2,0x0000);// 1,2 HUE                           
HI541WriteCmosSensor(0x31f4,0x0000);// 3,4 HUE                           
HI541WriteCmosSensor(0x31f6,0x0000);// 5,6 HUE                           

//region 2        
HI541WriteCmosSensor(0x3254,0x1021);
HI541WriteCmosSensor(0x3256,0x3200); 
HI541WriteCmosSensor(0x3258,0x741b);
HI541WriteCmosSensor(0x325A,0xa700);
HI541WriteCmosSensor(0x325C,0xe4ad);
HI541WriteCmosSensor(0x325E,0x3280);
HI541WriteCmosSensor(0x3260,0x6340);// x,1 SAT
HI541WriteCmosSensor(0x3262,0x4040);// 2,3 SAT                
HI541WriteCmosSensor(0x3264,0x4040);// 4,5 SAT  
HI541WriteCmosSensor(0x3266,0x4000);// 6 SAT, 1 HUE  
HI541WriteCmosSensor(0x3268,0x0000);// 2,3 HUE  
HI541WriteCmosSensor(0x326A,0x0000);// 4,5 HUE  
HI541WriteCmosSensor(0x326C,0x0000);// 6 HUE,x            

//region 3          
HI541WriteCmosSensor(0x17ee,0x1e21);
HI541WriteCmosSensor(0x17f0,0x3200);
HI541WriteCmosSensor(0x17f2,0x741b);
HI541WriteCmosSensor(0x17f4,0xa71e);
HI541WriteCmosSensor(0x17f6,0xe4ad);
HI541WriteCmosSensor(0x17f8,0x3280);
HI541WriteCmosSensor(0x17fa,0x6340);// x,1 SAT
HI541WriteCmosSensor(0x17fc,0x4040);// 2,3 SAT
HI541WriteCmosSensor(0x17fe,0x4040);// 4,5 SAT   
HI541WriteCmosSensor(0x1800,0x4000);// 6 SAT, 1 HUE   
HI541WriteCmosSensor(0x1802,0x0000);// 2,3 HUE   
HI541WriteCmosSensor(0x1804,0x1000);// 4,5 HUE   
HI541WriteCmosSensor(0x1806,0x0000);// 6 HUE,x     

//region 4           
HI541WriteCmosSensor(0x32cA,0x2132);
HI541WriteCmosSensor(0x32cC,0x0074);
HI541WriteCmosSensor(0x32cE,0x1ba7);
HI541WriteCmosSensor(0x32d0,0x1ee4);
HI541WriteCmosSensor(0x32d2,0xad32);
HI541WriteCmosSensor(0x32d4,0x8063);
HI541WriteCmosSensor(0x32d6,0x4040);// 1,2 SAT
HI541WriteCmosSensor(0x32d8,0x4040);// 3,4 SAT
HI541WriteCmosSensor(0x32dA,0x4040);// 5,6 SAT  
HI541WriteCmosSensor(0x32dC,0x0000);// 1,2 HUE  
HI541WriteCmosSensor(0x32dE,0x0010);// 3,4 HUE  
HI541WriteCmosSensor(0x32d0,0x0000);// 5,6 HUE  

//region 5          
HI541WriteCmosSensor(0x333E,0x1021);
HI541WriteCmosSensor(0x3340,0x3200);
HI541WriteCmosSensor(0x3342,0x741b); 
HI541WriteCmosSensor(0x3344,0xa71e);
HI541WriteCmosSensor(0x3346,0xe4ad);
HI541WriteCmosSensor(0x3348,0x3280);
HI541WriteCmosSensor(0x334A,0x6340);// x,1 SAT 
HI541WriteCmosSensor(0x334C,0x4040);// 2,3 SAT 
HI541WriteCmosSensor(0x334E,0x4040);// 4,5 SAT   
HI541WriteCmosSensor(0x3350,0x4000);// 6 SAT, 1 HUE   
HI541WriteCmosSensor(0x3352,0x0000);// 2,3 HUE   
HI541WriteCmosSensor(0x3354,0x1000);// 4,5 HUE   
HI541WriteCmosSensor(0x3356,0x0000);// 6 HUE,x     

//region 6          
HI541WriteCmosSensor(0x15F8,0x216e);
HI541WriteCmosSensor(0x15FA,0x0074);
HI541WriteCmosSensor(0x15FC,0x1ba7);
HI541WriteCmosSensor(0x15FE,0x1eda);
HI541WriteCmosSensor(0x1600,0xad32);
HI541WriteCmosSensor(0x1602,0x8063);
HI541WriteCmosSensor(0x1604,0x2d40);// 1,2 SAT 
HI541WriteCmosSensor(0x1606,0x4040);// 3,4 SAT 
HI541WriteCmosSensor(0x1608,0x4040);// 5,6 SAT  
HI541WriteCmosSensor(0x160A,0x0e00);// 1,2 HUE  
HI541WriteCmosSensor(0x160C,0x0010);// 3,4 HUE  
HI541WriteCmosSensor(0x160E,0x0000);// 5,6 HUE  

//region 7         
HI541WriteCmosSensor(0x33b4,0x216e);
HI541WriteCmosSensor(0x33b6,0x0074);
HI541WriteCmosSensor(0x33b8,0x1ba7);
HI541WriteCmosSensor(0x33bA,0x1eda);
HI541WriteCmosSensor(0x33bC,0xad32);
HI541WriteCmosSensor(0x33bE,0x8063);
HI541WriteCmosSensor(0x33c0,0x2d40);// 1,2 SAT
HI541WriteCmosSensor(0x33c2,0x4040);// 3,4 SAT
HI541WriteCmosSensor(0x33c4,0x4040);// 5,6 SAT 
HI541WriteCmosSensor(0x33c6,0x0e00);// 1,2 HUE 
HI541WriteCmosSensor(0x33c8,0x0010);// 3,4 HUE 
HI541WriteCmosSensor(0x33cA,0x0000);// 5,6 HUE 

//region 8         
HI541WriteCmosSensor(0x3428,0x1021);
HI541WriteCmosSensor(0x342A,0x6e00);
HI541WriteCmosSensor(0x342C,0x741b); 
HI541WriteCmosSensor(0x342E,0xa71e);
HI541WriteCmosSensor(0x3430,0xdaad);
HI541WriteCmosSensor(0x3432,0x3280);
HI541WriteCmosSensor(0x3434,0x6336);// x,1 SAT
HI541WriteCmosSensor(0x3436,0x4040);// 2,3 SAT
HI541WriteCmosSensor(0x3438,0x4040);// 4,5 SAT  
HI541WriteCmosSensor(0x343A,0x4010);// 6 SAT, 1 HUE  
HI541WriteCmosSensor(0x343C,0x0000);// 2,3 HUE  
HI541WriteCmosSensor(0x343E,0x1000);// 4,5 HUE  
HI541WriteCmosSensor(0x3440,0x0000);// 6 HUE,x       

//region 9          
HI541WriteCmosSensor(0x1400,0x1e21);
HI541WriteCmosSensor(0x1402,0x3200);
HI541WriteCmosSensor(0x1404,0x741b);
HI541WriteCmosSensor(0x1406,0xa71e);
HI541WriteCmosSensor(0x1408,0xe4ad);
HI541WriteCmosSensor(0x140A,0x3280);
HI541WriteCmosSensor(0x140C,0x6340);// x,1 SAT
HI541WriteCmosSensor(0x140E,0x4040);// 2,3 SAT
HI541WriteCmosSensor(0x1410,0x4040);// 4,5 SAT
HI541WriteCmosSensor(0x1412,0x4000);// 6 SAT, 1 HUE 
HI541WriteCmosSensor(0x1414,0x0000);// 2,3 HUE  
HI541WriteCmosSensor(0x1416,0x1000);// 4,5 HUE  
HI541WriteCmosSensor(0x1418,0x0000);// 6 HUE,x   
                
//region 10       
HI541WriteCmosSensor(0x349E,0x2132);
HI541WriteCmosSensor(0x34a0,0x0074);
HI541WriteCmosSensor(0x34a2,0x1ba7);
HI541WriteCmosSensor(0x34a4,0x1ee4);
HI541WriteCmosSensor(0x34a6,0xad32);
HI541WriteCmosSensor(0x34a8,0x8063);
HI541WriteCmosSensor(0x34aA,0x4040);// 1,2 SAT
HI541WriteCmosSensor(0x34aC,0x4040);// 3,4 SAT
HI541WriteCmosSensor(0x34aE,0x4040);// 5,6 SAT
HI541WriteCmosSensor(0x34b0,0x0000);// 1,2 HUE 
HI541WriteCmosSensor(0x34b2,0x0010);// 3,4 HUE 
HI541WriteCmosSensor(0x34b4,0x0000);// 5,6 HUE 
                
//region 11       
HI541WriteCmosSensor(0x3512,0x1021);
HI541WriteCmosSensor(0x3514,0x3200);
HI541WriteCmosSensor(0x3516,0x741b);
HI541WriteCmosSensor(0x3518,0xa71e);
HI541WriteCmosSensor(0x351A,0xe4ad);
HI541WriteCmosSensor(0x351C,0x3280);
HI541WriteCmosSensor(0x351E,0x6340);// x,1 SAT
HI541WriteCmosSensor(0x3520,0x4040);// 2,3 SAT
HI541WriteCmosSensor(0x3522,0x4040);// 4,5 SAT
HI541WriteCmosSensor(0x3524,0x4000);// 6 SAT, 1 HUE 
HI541WriteCmosSensor(0x3526,0x0000);// 2,3 HUE
HI541WriteCmosSensor(0x3528,0x1000);// 4,5 HUE  
HI541WriteCmosSensor(0x352A,0x0000);// 6 HUE,x  
	}
// Adaptive RX Flushing 20140714 YSHwang
HI541WriteCmosSensor(0x1260,0x3255); //0x5532
HI541WriteCmosSensor(0x1262,0x0040); //0x4000
 
HI541WriteCmosSensor(0x10b8,0x4848);//0,1
HI541WriteCmosSensor(0x10ba,0x4848);//
HI541WriteCmosSensor(0x10bc,0x4848);//
HI541WriteCmosSensor(0x10be,0x4444);//6,7
HI541WriteCmosSensor(0x10c0,0x4444);//8,9
HI541WriteCmosSensor(0x10c2,0x4444);//10,11 0x44~0x48 (marginally) OFF,0x46 : ON


// Adaptive BLC
HI541WriteCmosSensor(0x1268,0xb254); //0x54b2
HI541WriteCmosSensor(0x126a,0x0040); //0x4000
HI541WriteCmosSensor(0x126c,0xb354); //0x54b3
HI541WriteCmosSensor(0x126e,0x0040); //0x4000
 
//54b2 : Blue
HI541WriteCmosSensor(0x10d0,0x0202); //0,1
HI541WriteCmosSensor(0x10d2,0x0200); //
HI541WriteCmosSensor(0x10d4,0x0000); //
HI541WriteCmosSensor(0x10d6,0x0202); //6,7
HI541WriteCmosSensor(0x10d8,0x0202); //8,9
HI541WriteCmosSensor(0x10da,0x0202); //10,11 

//54b3 : Red
HI541WriteCmosSensor(0x10dc,0x0000); //0,1
HI541WriteCmosSensor(0x10de,0x0000); //
HI541WriteCmosSensor(0x10e0,0x0000); //
HI541WriteCmosSensor(0x10e2,0x0101); //6,7
HI541WriteCmosSensor(0x10e4,0x0101); //8,9
HI541WriteCmosSensor(0x10e6,0x0101);//10,11 
 
HI541WriteCmosSensor(0x125c,0x0103); //enable


//================================
//=========== ADP End ============
//================================

//================================
//======= ISP HW Start ===========
//================================
HI541WriteCmosSensor(0xffff,0x0040);

HI541WriteCmosSensor(0x4830,0xfeef);////20140325 HW Trans function enb 0x4831 B[7]
HI541WriteCmosSensor(0x4832,0x7f7a);//[3]MCMC Enable,2A2D STEVE 20140522,AFFilter enable
HI541WriteCmosSensor(0x4834,0x0401);//


HI541WriteCmosSensor(0x7006,0x08c8);// 2A2D
HI541WriteCmosSensor(0x7026,0x0808);// 2A2D
HI541WriteCmosSensor(0x7028,0x0664);// 2A2D

HI541WriteCmosSensor(0x700e,0x2b21);// 2A2D //20140313 Bayer skin th 16 -> 2b 


HI541WriteCmosSensor(0x740e,0x7840);//

HI541WriteCmosSensor(0x7424,0x2406);////20140313 impulse th
HI541WriteCmosSensor(0x7426,0x101f);////20140313 impulse th
HI541WriteCmosSensor(0x7428,0x3252);////20140313 impulse th
HI541WriteCmosSensor(0x742a,0x71a9);////20140313 impulse th

HI541WriteCmosSensor(0x7466,0x1631);// 2A2D, Rev 31 : [7:5] HigFeqFilt type 0 : bilinear sum, 1 : 56 center weight, 6 : median, 7:hpf  
HI541WriteCmosSensor(0x7828,0x1715);// 2A2D //20140614 blue text
HI541WriteCmosSensor(0x782a,0x1410);// 2A2D //20140614 blue text
HI541WriteCmosSensor(0x782c,0x1010);// 2A2D


//GbGr Correction fnc
HI541WriteCmosSensor(0x745c,0x3756);// 2A2D //20140320 GBGR 24 -> 56
HI541WriteCmosSensor(0x745e,0x8862);        //20140320 GBGR 

HI541WriteCmosSensor(0x7c00,0x2f3c);// 2A2D
HI541WriteCmosSensor(0x7c78,0x1f1c);// 2A2D
HI541WriteCmosSensor(0x7c7a,0x1a18);// 2A2D
HI541WriteCmosSensor(0x7c7c,0x1600);// 2A2D

//dir noise 20140618
HI541WriteCmosSensor(0x7c9c,0x0a09);// 2A2D // 0x7c9d // 20140616 weak_hv graph(weak->HV)
HI541WriteCmosSensor(0x7c9e,0x0b0e);// 2A2D //20140616 weak_hv graph(weak->HV)
HI541WriteCmosSensor(0x7ca0,0x1014);// 2A2D //20140616 weak_hv graph(weak->HV)
HI541WriteCmosSensor(0x7ca2,0x2030);// 2A2D //20140616 weak_hv graph(weak->HV)
HI541WriteCmosSensor(0x7ca4,0x3013);// 2A2D // 0x7ca4 // 20140616 weak_hv graph(weak->HV)
HI541WriteCmosSensor(0x9400,0x800e);// 2A2D 
HI541WriteCmosSensor(0x9420,0x202d);// 2A2D
HI541WriteCmosSensor(0x9428,0x0032);// 2A2D

HI541WriteCmosSensor(0x9834,0x2d23);// 2A2D //20140614 blue text ????
HI541WriteCmosSensor(0x9836,0x1d1c);// 2A2D
HI541WriteCmosSensor(0x9838,0x1b1a);// 2A2D
HI541WriteCmosSensor(0x983a,0x1918);// 2A2D
HI541WriteCmosSensor(0x985c,0x0000);// 2A2D
HI541WriteCmosSensor(0x985e,0x0000);// 2A2D
HI541WriteCmosSensor(0x9860,0x0100);// 2A2D


HI541WriteCmosSensor(0x9c00,0x7f7f);//2150);// 2A2D
//region HW
HI541WriteCmosSensor(0x9c02,0x2132);// 1 Magen Delta, Center
HI541WriteCmosSensor(0x9c04,0x0074);// 2 Skin Delta, Center
HI541WriteCmosSensor(0x9c06,0x1ba7);// 3 Yellow Delta, Center
HI541WriteCmosSensor(0x9c08,0x00e4);// 4 Green Delta, Center
HI541WriteCmosSensor(0x9c0a,0xad32);// 5 Cyan Delta, Center
HI541WriteCmosSensor(0x9c0c,0x8056);// 6 Blue Delta, Center
HI541WriteCmosSensor(0x9c0e,0x4040);// 1,2 SAT     
HI541WriteCmosSensor(0x9c10,0x4840);// 3,4 SAT      
HI541WriteCmosSensor(0x9c12,0x4240);// 5,6 SAT   
HI541WriteCmosSensor(0x9c14,0x0800);// 1,2 HUE     
HI541WriteCmosSensor(0x9c16,0x0b00);// 3,4 HUE     
HI541WriteCmosSensor(0x9c18,0x1100);// 5,6 HUE     

/////////////
HI541WriteCmosSensor(0x9c42,0x0000);// 2A2D // Global Adap. sat Disable
HI541WriteCmosSensor(0x9c44,0x1b39);// 2A2D
HI541WriteCmosSensor(0x9c46,0x5a80);// 2A2D
HI541WriteCmosSensor(0x9c48,0xa6c1);// 2A2D
HI541WriteCmosSensor(0x9c4a,0xe838);// 2A2D
HI541WriteCmosSensor(0x9c4c,0x3a3c);// 2A2D
HI541WriteCmosSensor(0x9c4e,0x3f3f);// 2A2D
HI541WriteCmosSensor(0x9c50,0x3f3f);// 2A2D
HI541WriteCmosSensor(0x9c52,0x3f00);// 2A2D
HI541WriteCmosSensor(0x9c54,0x0000);// 2A2D
HI541WriteCmosSensor(0x9c5a,0x003f);// 2A2D 20140626 jhlee 
//HI541WriteCmosSensor(0x9c5c,0x0000);// 2A2D
HI541WriteCmosSensor(0x9c5c,0x1000);// 2A2D
HI541WriteCmosSensor(0x9c5e,0x003f);// 2A2D
HI541WriteCmosSensor(0x9c8e,0xb000);// 2A2D
HI541WriteCmosSensor(0x9c90,0x001c);//for MCMC,0x011c);// 2A2D

// False Color Suppression fnc
HI541WriteCmosSensor(0x7c4e,0x2b00);// 2A2D, Rev 00 : sp_ln_ctl, Ln Sharp off Intp 
HI541WriteCmosSensor(0x8000,0x302d);// 2A2D, CMC
HI541WriteCmosSensor(0x8002,0xbb00);// 2A2D, Rev 90 : ps_ln_cmc_gain, 15/16->9/16 CMC //20140315 false color
HI541WriteCmosSensor(0x8800,0x0000);// 2A2D, Rev 00 : gma_ctl, ps_off_ln, ps_off_dy alloff GMA
HI541WriteCmosSensor(0x8802,0xbb00);// 2A2D, Rev 90 : GMA 15/16->9/16 //20140315 false color


// DPC
HI541WriteCmosSensor(0x7048,0x0000);
HI541WriteCmosSensor(0x704a,0x0000);
HI541WriteCmosSensor(0x704e,0x0000);
HI541WriteCmosSensor(0x7050,0x0000);
HI541WriteCmosSensor(0x7052,0x8200);
HI541WriteCmosSensor(0x7054,0x0000);


// Iridix
HI541WriteCmosSensor(0x8404,0x240a);//hdr_frame_width
HI541WriteCmosSensor(0x8406,0x9c07);//hdr_frame_height
//================================
//========= ISP HW END ===========
//================================

//================================
//========= DMA AWB OTP Start ====
//================================
// >>>> DMA operation (OTP to Register Write Mode)
HI541WriteCmosSensor(0x4000,0x9404);//dma otp mode1 (otp to register write mode) enable
HI541WriteCmosSensor(0x4002,0x0000);//dma control
HI541WriteCmosSensor(0x4020,0x0100);//dma interrupt pending clear

//// AWB OTP download start
HI541WriteCmosSensor(0x4004,0xf018);//dma src addr[7:0]//dma src addr[15:8]
HI541WriteCmosSensor(0x4006,0x0000);//dma src addr[23:16]//dma src addr[31:24]
HI541WriteCmosSensor(0x4000,0x9504);//dma start

mDELAY(1);//dma operation delay = 1ms

HI541WriteCmosSensor(0x4000,0x9400);//dma otp mode1 (otp to register write mode) disable
//================================
//========= DMA AWB OTP End ======
//================================

//================================
//========= SYSTEM Start =========
//================================
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x3824,0x0000);
HI541WriteCmosSensor(0x3826,0x001c);// MCU cam start cmd
HI541WriteCmosSensor(0x3824,0x0000);
HI541WriteCmosSensor(0x3826,0xa010);
//================================
//========= SYSTEM End  ==========
//================================
//========================================
//=========== AF Filter Start =============
//========================================
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0xC800, 0x4840);	// sobel& spmd  
HI541WriteCmosSensor(0xc802, 0x0044);
HI541WriteCmosSensor(0xc804, 0x0044);
HI541WriteCmosSensor(0xC806, 0xff03); // af_ctl8 // manual window setting + x_half[6] 
//HI541WriteCmosSensor(0xC806, 0x1002);
//HI541WriteCmosSensor(0xC808, 0x0006); // af_ctl9// manual window setting + y_half[7]

// AF window control
HI541WriteCmosSensor(0xC814, 0x1303); // AF window Region #1       	// Y Start
HI541WriteCmosSensor(0xC828, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC83C, 0xa601); 					// X Start	
HI541WriteCmosSensor(0xC850, 0x4a03); 					// X end	
HI541WriteCmosSensor(0xC816, 0x9f02); // AF window Region #2       	// Y Start
HI541WriteCmosSensor(0xC82A, 0xe504); 					// Y end	
HI541WriteCmosSensor(0xC83E, 0x3204); 					// X Start	
HI541WriteCmosSensor(0xC852, 0x5206); 					// X end	
HI541WriteCmosSensor(0xC818, 0x1303); // AF window Region #3       	// Y Start
HI541WriteCmosSensor(0xC82C, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC840, 0x3107); 					// X Start	
HI541WriteCmosSensor(0xC854, 0xd508); 					// X end	
HI541WriteCmosSensor(0xC81A, 0x5b05); // AF window Region #4       	// Y Start
HI541WriteCmosSensor(0xC82E, 0x8206); 					// Y end	
HI541WriteCmosSensor(0xC842, 0x6e04); 					// X Start	
HI541WriteCmosSensor(0xC856, 0x1206); 					// X end	
//HI541WriteCmosSensor(0xC81C, 0x4803); // AF window Region #5       	// Y Start
//HI541WriteCmosSensor(0xC830, 0x3804); 					// Y end	
HI541WriteCmosSensor(0xC81C, 0x3803); // AF window Region #5       	// Y Start
HI541WriteCmosSensor(0xC830, 0x6804); 					// Y end	
HI541WriteCmosSensor(0xC844, 0xEA04); 					// X Start	
HI541WriteCmosSensor(0xC858, 0x9E05); 					// X end	
HI541WriteCmosSensor(0xC81E, 0x1303); // AF window Region #6       	// Y Start
HI541WriteCmosSensor(0xC832, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC846, 0xa601); 					// X Start	
HI541WriteCmosSensor(0xC85A, 0x4a03); 					// X end	
HI541WriteCmosSensor(0xC820, 0x9f02); // AF window Region #7       	// Y Start
HI541WriteCmosSensor(0xC834, 0xe504); 					// Y end	
HI541WriteCmosSensor(0xC848, 0x3204); 					// X Start	
HI541WriteCmosSensor(0xC85C, 0x5206); 					// X end	
HI541WriteCmosSensor(0xC822, 0x1303); // AF window Region #8       	// Y Start
HI541WriteCmosSensor(0xC836, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC84A, 0x3107); 					// X Start	
HI541WriteCmosSensor(0xC85E, 0xd508); 					// X end	
HI541WriteCmosSensor(0xC824, 0x5b05); // AF window Region #9       	// Y Start
HI541WriteCmosSensor(0xC838, 0x8206); 					// Y end	
HI541WriteCmosSensor(0xC84C, 0x6e04); 					// X Start	
HI541WriteCmosSensor(0xC860, 0x1206); 					// X end	
//HI541WriteCmosSensor(0xC826, 0x4803); // AF window Region #10       	// Y Start
//HI541WriteCmosSensor(0xC83A, 0x3804); 					// Y end	
//HI541WriteCmosSensor(0xC826, 0x3803); // AF window Region #10       	// Y Start
//HI541WriteCmosSensor(0xC83A, 0x6804); 					// Y end	
//HI541WriteCmosSensor(0xC84E, 0xEA04); 					// X Start	
//HI541WriteCmosSensor(0xC862, 0x9E05); 					// X end	
HI541WriteCmosSensor(0xC826, 0x9f02); // AF window Region #10       	// Y Start
HI541WriteCmosSensor(0xC83A, 0xe504); 					// Y end	
HI541WriteCmosSensor(0xC84E, 0x3204); 					// X Start	
HI541WriteCmosSensor(0xC862, 0x5206); 					// X end	

//========================================
//=========== AF Filter End ===============
//========================================
//================================
//======== FINDBAND Start ========
//================================
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0xc000, 0xbf30); //
HI541WriteCmosSensor(0xc002, 0x5108); //
HI541WriteCmosSensor(0xc004, 0x3005); //
HI541WriteCmosSensor(0xc006, 0x0f00); //
HI541WriteCmosSensor(0xc008, 0x0f13);//FLK200,FLK240
HI541WriteCmosSensor(0xc00a, 0x3278); //
HI541WriteCmosSensor(0xc00c, 0x000f); //
HI541WriteCmosSensor(0xc00e, 0x1000); //
//HI541WriteCmosSensor(0xc010, 0x5000);
HI541WriteCmosSensor(0xc094, 0x0100); //

//HI541WriteCmosSensor(0xc000, 0xbf30); //2A2D
//HI541WriteCmosSensor(0xc002, 0x5108); //2A2D
//HI541WriteCmosSensor(0xc004, 0x3001); //2A2D
//HI541WriteCmosSensor(0xc006, 0x0f00); //2A2D
//HI541WriteCmosSensor(0xc008, 0x1e3a); //2A2D
//HI541WriteCmosSensor(0xc00a, 0x0f78); //2A2D
//HI541WriteCmosSensor(0xc00c, 0x0300); //2A2D
//HI541WriteCmosSensor(0xc00e, 0x1000); //2A2D
//HI541WriteCmosSensor(0xc010, 0x5000); //2A2D 
//HI541WriteCmosSensor(0xc094, 0x0100); //2A2D

HI541WriteCmosSensor(0x548a,0x0004); 
//================================
//======== FINDBAND End   ========
//================================


//===================================
//=========== AE Start ==============
//===================================
HI541WriteCmosSensor(0xffff,0x0020);
//HI541WriteCmosSensor(0x02e0,0xca40);//fast AE 60Hz
HI541WriteCmosSensor(0x02e0,0xc540); //fast AE, Band 50Hz
HI541WriteCmosSensor(0x02e8,0x3d00);//AE target Y140 level (STEVE)
HI541WriteCmosSensor(0x0368,0x0300);//frame ??? ????
//HI541WriteCmosSensor(0x0398,0x0701);//AE speed for dark
HI541WriteCmosSensor(0x0398,0x0501);//AE speed for dark
HI541WriteCmosSensor(0x0328,0x015c);//AE speed //2c
HI541WriteCmosSensor(0x02e2,0xcb80); //digital gain enable
HI541WriteCmosSensor(0x030a,0x0001); //digital gain max 1x

HI541WriteCmosSensor(0x0370,0xca0a); //out band
HI541WriteCmosSensor(0x0372,0x0000);
HI541WriteCmosSensor(0x0310,0x282b); //exp min = outbandx4 140819
HI541WriteCmosSensor(0x0312,0x0000);
HI541WriteCmosSensor(0x0320,0x80e7); //exp max
HI541WriteCmosSensor(0x0322,0x4c00);

HI541WriteCmosSensor(0x0384,0x80e7);//L
HI541WriteCmosSensor(0x0386,0x4c00);//H exposure max 100 8.33 fps
HI541WriteCmosSensor(0x0388,0x80e7);//L
HI541WriteCmosSensor(0x038a,0x4c00);//H exposure max 120 8.57 fps


HI541WriteCmosSensor(0x0374,0xa068); //exp band for 100
HI541WriteCmosSensor(0x0376,0x0600);
HI541WriteCmosSensor(0x0378,0x3057); //exp band for 120
HI541WriteCmosSensor(0x037a,0x0500);


HI541WriteCmosSensor(0x0324,0x0304); //exp time th1
HI541WriteCmosSensor(0x0326,0x0500); //exp time th3
HI541WriteCmosSensor(0x0390,0x0d0d); //band free exp100, exp120

//HI541WriteCmosSensor(0x02ee,0x2040); //AG th1,2AG MIN x1.468
//HI541WriteCmosSensor(0x02f0,0x5064); //AG th3,4
HI541WriteCmosSensor(0x02ee,0x2020); //AG th1,2AG MIN x1.468 not use the Band gain
HI541WriteCmosSensor(0x02f0,0x2020); //AG th3,4
HI541WriteCmosSensor(0x0394,0xc2bb); // outdoor limit 3, 2                  

HI541WriteCmosSensor(0x039c,0xff41); // full average, colortendency off
//HI541WriteCmosSensor(0x02ec,0x4910); // AE seed slow 
//HI541WriteCmosSensor(0x030c,0x6666);
HI541WriteCmosSensor(0x030e,0x0200);
		HI541WriteCmosSensor(0x0330,0x4016); //U32ExpVal_ro 100fps
		HI541WriteCmosSensor(0x0332,0x4000); 
HI541WriteCmosSensor(0x03b0,0xca0a); //line length for enter the camera
//HI541WriteCmosSensor(0x043a,0x0107); //divider for fast AE
HI541WriteCmosSensor(0x043a,0x0105); //divider for fast AE
//AE weight table_Center
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0448, 0x1111);
HI541WriteCmosSensor(0x044a, 0x1111);
HI541WriteCmosSensor(0x044c, 0x1111);
HI541WriteCmosSensor(0x044e, 0x1111);
HI541WriteCmosSensor(0x0450, 0x1111);
HI541WriteCmosSensor(0x0452, 0x1111);
HI541WriteCmosSensor(0x0454, 0x1111);
HI541WriteCmosSensor(0x0456, 0x1111);
HI541WriteCmosSensor(0x0458, 0x1111);
HI541WriteCmosSensor(0x045a, 0x1111);
HI541WriteCmosSensor(0x045c, 0x1111);
HI541WriteCmosSensor(0x045e, 0x1111);
HI541WriteCmosSensor(0x0460, 0x1111);
HI541WriteCmosSensor(0x0462, 0x1111);
HI541WriteCmosSensor(0x0464, 0x1111);
HI541WriteCmosSensor(0x0466, 0x1111);
HI541WriteCmosSensor(0x0468, 0x1111);
HI541WriteCmosSensor(0x046a, 0x3333);
HI541WriteCmosSensor(0x046c, 0x3333);
HI541WriteCmosSensor(0x046e, 0x1111);
HI541WriteCmosSensor(0x0470, 0x1111);
HI541WriteCmosSensor(0x0472, 0x3333);
HI541WriteCmosSensor(0x0474, 0x3333);
HI541WriteCmosSensor(0x0476, 0x1111);
HI541WriteCmosSensor(0x0478, 0x1111);
HI541WriteCmosSensor(0x047a, 0x3333);
HI541WriteCmosSensor(0x047c, 0x3333);
HI541WriteCmosSensor(0x047e, 0x1111);
HI541WriteCmosSensor(0x0480, 0x1111);
HI541WriteCmosSensor(0x0482, 0x3344);
HI541WriteCmosSensor(0x0484, 0x4433);
HI541WriteCmosSensor(0x0486, 0x1111);
HI541WriteCmosSensor(0x0488, 0x1111);
HI541WriteCmosSensor(0x048a, 0x3344);
HI541WriteCmosSensor(0x048c, 0x4433);
HI541WriteCmosSensor(0x048e, 0x1111);
HI541WriteCmosSensor(0x0490, 0x1111);
HI541WriteCmosSensor(0x0492, 0x3333);
HI541WriteCmosSensor(0x0494, 0x3333);
HI541WriteCmosSensor(0x0496, 0x1111);
HI541WriteCmosSensor(0x0498, 0x1111);
HI541WriteCmosSensor(0x049a, 0x3333);
HI541WriteCmosSensor(0x049c, 0x3333);
HI541WriteCmosSensor(0x049e, 0x1111);
HI541WriteCmosSensor(0x04a0, 0x1111);
HI541WriteCmosSensor(0x04a2, 0x3333);
HI541WriteCmosSensor(0x04a4, 0x3333);
HI541WriteCmosSensor(0x04a6, 0x1111);
HI541WriteCmosSensor(0x04a8, 0x1111);
HI541WriteCmosSensor(0x04aa, 0x3333);
HI541WriteCmosSensor(0x04ac, 0x3333);
HI541WriteCmosSensor(0x04ae, 0x1111);
HI541WriteCmosSensor(0x04b0, 0x3333);
HI541WriteCmosSensor(0x04b2, 0x3333);
HI541WriteCmosSensor(0x04b4, 0x3333);
HI541WriteCmosSensor(0x04b6, 0x3333);
HI541WriteCmosSensor(0x04b8, 0x3333);
HI541WriteCmosSensor(0x04ba, 0x3333);
HI541WriteCmosSensor(0x04bc, 0x3333);
HI541WriteCmosSensor(0x04be, 0x3333);
HI541WriteCmosSensor(0x04c0, 0x3333);
HI541WriteCmosSensor(0x04c2, 0x3333);
HI541WriteCmosSensor(0x04c4, 0x3333);
HI541WriteCmosSensor(0x04c6, 0x3333);

HI541WriteCmosSensor(0xffff, 0x0020);
		if (0x00AE == HI541ReadCmosSensor(0x0002))
		{
		//dpc flag option_20140901 for BF
HI541WriteCmosSensor(0x3678, 0x0120);  
HI541WriteCmosSensor(0x367a, 0x4080);  
HI541WriteCmosSensor(0x367c, 0xaacc);  
HI541WriteCmosSensor(0x367e, 0xff01);  
HI541WriteCmosSensor(0x3680, 0x0305);  
HI541WriteCmosSensor(0x3682, 0x0709);  
HI541WriteCmosSensor(0x3684, 0x0e02);  
HI541WriteCmosSensor(0x3686, 0x0508);  
HI541WriteCmosSensor(0x3688, 0x0f0f);  
HI541WriteCmosSensor(0x368a, 0x2020);  
		 }
		 
		else {  			
		//dpc flag option_20140901 for BG
HI541WriteCmosSensor(0x07d0, 0x0120);  
HI541WriteCmosSensor(0x07d2, 0x4080);  
HI541WriteCmosSensor(0x07d4, 0xaacc);  
HI541WriteCmosSensor(0x07d6, 0xff01);  
HI541WriteCmosSensor(0x07d8, 0x0508);  
HI541WriteCmosSensor(0x07da, 0x0b10);  
HI541WriteCmosSensor(0x07dc, 0x2800);  
HI541WriteCmosSensor(0x07de, 0x0007);  
HI541WriteCmosSensor(0x07e0, 0x0d0d);  
HI541WriteCmosSensor(0x07e2, 0x0f0f);  
			}

//===================================
//=========== AE END ================
//===================================

//===================================
//=========== AWB Start ================
//===================================
//STE AE,AWB SIZE in Hardware Logic
HI541WriteCmosSensor(0xffff,0x0040);         
HI541WriteCmosSensor(0xC404,0x1700);//ssd_ctl1
HI541WriteCmosSensor(0x6C00,0x1000);//[7,6]rev,[5]round,[4]awb_g_opt,[3,2]rev,[1]AWB gain Precision (0: x1024,1: x2048), [0]rev
		HI541WriteCmosSensor(0x6C02, 0xd005); //r_dgain_dyr
		HI541WriteCmosSensor(0x6C04, 0x9007); //b_dgain_dyr
		HI541WriteCmosSensor(0x6C06, 0x0004); //gr_gain_dry
		HI541WriteCmosSensor(0x6C08, 0x0004); //gb_gain-dry
HI541WriteCmosSensor(0xc414,0x0000);//ae_patch_xy_offset
HI541WriteCmosSensor(0xc416,0xa078);//ae_patch_xy_w_h x 16
HI541WriteCmosSensor(0xc418,0x0404);//awb_size_xy_offset
HI541WriteCmosSensor(0xc41a,0xa070);//awb_size_xy_w_h x 16
HI541WriteCmosSensor(0xffff,0x0020);
//AWB Control Register 
HI541WriteCmosSensor(0x0830,0xDAC0);  //AWB_en=1,ADPBlc_on, msADPccm_off, highTExEn ,htzSkyUse = 1, htzNormUse = 1
HI541WriteCmosSensor(0x0924,0xCC02);  //Manual WB, awb y min
		HI541WriteCmosSensor(0x0926,0xA00D);  //u8YMax_rw, crCtrl
//STE(for AE,AWB) AWB SIZE Start in Software Logic
HI541WriteCmosSensor(0x1fe4,0x0404);//awb_size_xy_offset
HI541WriteCmosSensor(0x1fe6,0x2018);//awb_size_xy_w_h x 16
HI541WriteCmosSensor(0x1fe8,0x0404);//ae_size_xy_offset
HI541WriteCmosSensor(0x1fea,0xa070);//ae_size_xy_w_h x 16

		HI541WriteCmosSensor(0x0a18, 0x7a01); //BufRgain_0 1
		HI541WriteCmosSensor(0x0a1a, 0x7a01); //BufRgain_2 3
		HI541WriteCmosSensor(0x0a1c, 0x7a01); //BufRgain_4 5
		HI541WriteCmosSensor(0x0a1e, 0x7a01); //BufRgain_6 7
		HI541WriteCmosSensor(0x0a20, 0xa401); //BufBgain_0 1
		HI541WriteCmosSensor(0x0a22, 0xa401); //BufBgain_2 3
		HI541WriteCmosSensor(0x0a24, 0xa401); //BufBgain_4 5
		HI541WriteCmosSensor(0x0a26, 0xa401); //BufBgain_6 7
//AWB Color Map Start
HI541WriteCmosSensor(0x0a78, 0xBC28); //Start_X 1, Start_Y 1
HI541WriteCmosSensor(0x0a7a, 0xC024); //End_X 
HI541WriteCmosSensor(0x0a7c, 0xBD28);  
HI541WriteCmosSensor(0x0a7e, 0xC125);  
HI541WriteCmosSensor(0x0a80, 0x6648);  
HI541WriteCmosSensor(0x0a82, 0x6C42);  
HI541WriteCmosSensor(0x0a84, 0xBC2E);  
HI541WriteCmosSensor(0x0a86, 0xC028);  
HI541WriteCmosSensor(0x0a88, 0x5F4E);  
HI541WriteCmosSensor(0x0a8a, 0x6844);  
HI541WriteCmosSensor(0x0a8c, 0x5852);  
HI541WriteCmosSensor(0x0a8e, 0x6247);  
HI541WriteCmosSensor(0x0a90, 0x505B);  
HI541WriteCmosSensor(0x0a92, 0x5B4C);  
HI541WriteCmosSensor(0x0a94, 0x485E);  
HI541WriteCmosSensor(0x0a96, 0x5450);  
HI541WriteCmosSensor(0x0a98, 0x3F67);  
HI541WriteCmosSensor(0x0a9a, 0x4E56);  
HI541WriteCmosSensor(0x0a9c, 0x3B6C);  
HI541WriteCmosSensor(0x0a9e, 0x4860);  
HI541WriteCmosSensor(0x0aa0, 0x3875);  
HI541WriteCmosSensor(0x0aa2, 0x4469);  
HI541WriteCmosSensor(0x0aa4, 0x3778);  
HI541WriteCmosSensor(0x0aa6, 0x416D);  
HI541WriteCmosSensor(0x0aa8, 0x3085);  
HI541WriteCmosSensor(0x0aaa, 0x3C73);  
HI541WriteCmosSensor(0x0aac, 0x2E90);  
HI541WriteCmosSensor(0x0aae, 0x3A84);  
HI541WriteCmosSensor(0x0ab0, 0x279C);  
HI541WriteCmosSensor(0x0ab2, 0x338C);  
HI541WriteCmosSensor(0x0ab4, 0x24A6);  
HI541WriteCmosSensor(0x0ab6, 0x3099);  
HI541WriteCmosSensor(0x0ab8,  0x6F4A);  
HI541WriteCmosSensor(0x0aba,  0x7644);  
HI541WriteCmosSensor(0x0abc,  0x724A);  
HI541WriteCmosSensor(0x0abe,  0x7B43);  
HI541WriteCmosSensor(0x0ac0,  0xBF30);  
HI541WriteCmosSensor(0x0ac2,  0xC42C);  
HI541WriteCmosSensor(0x0ac4,  0xC12A);  
HI541WriteCmosSensor(0x0ac6,  0xC527);  
HI541WriteCmosSensor(0x0ac8,  0xC12A);  
HI541WriteCmosSensor(0x0aca,  0xC527);  
HI541WriteCmosSensor(0x0acc,  0xC12A);  
HI541WriteCmosSensor(0x0ace,  0xC527);  
HI541WriteCmosSensor(0x0ad0,  0xC12A);  
HI541WriteCmosSensor(0x0ad2,  0xC527);  
HI541WriteCmosSensor(0x0ad4,  0xC12A);  
HI541WriteCmosSensor(0x0ad6,  0xC527);  

//Normal(Indoor) Weight TBL
//4,5LV
HI541WriteCmosSensor(0x0b1e,0x1111);
HI541WriteCmosSensor(0x0b20,0x1111);
//6,7LV
HI541WriteCmosSensor(0x0b22,0x1111);
HI541WriteCmosSensor(0x0b24,0x1111);
//8,9LV
HI541WriteCmosSensor(0x0b26,0x1232);
HI541WriteCmosSensor(0x0b28,0x1111);
//10,11LV
HI541WriteCmosSensor(0x0b2a,0x1232);
HI541WriteCmosSensor(0x0b2c,0x2111);
//12,13LV
HI541WriteCmosSensor(0x0b2e,0x1233);
HI541WriteCmosSensor(0x0b30,0x1111);
//14,15LV
HI541WriteCmosSensor(0x0b32,0x1133);
HI541WriteCmosSensor(0x0b34,0x1111);
//16,17LV
HI541WriteCmosSensor(0x0b36,0x1133);
HI541WriteCmosSensor(0x0b38,0x1111);
//Special(Outdoor) Weight TBL
//4,5LV
HI541WriteCmosSensor(0x0b3a,0x1111);
HI541WriteCmosSensor(0x0b3c,0x1111);
//6,7LV
HI541WriteCmosSensor(0x0b3e,0x1111);
HI541WriteCmosSensor(0x0b40,0x1111);
//8LV
HI541WriteCmosSensor(0x0b42,0x1232);
HI541WriteCmosSensor(0x0b44,0x1111);
//9LV
HI541WriteCmosSensor(0x1fc8,0x1232);
HI541WriteCmosSensor(0x1fca,0x1111);
//10LV
HI541WriteCmosSensor(0x0b46,0x1232);
HI541WriteCmosSensor(0x0b48,0x1111);
//11LV
HI541WriteCmosSensor(0x1fcc,0x1232);
HI541WriteCmosSensor(0x1fce,0x1111);
//12LV
HI541WriteCmosSensor(0x0b4a,0x1233);
HI541WriteCmosSensor(0x0b4c,0x1111);
//13LV
HI541WriteCmosSensor(0x1fd0,0x1233);
HI541WriteCmosSensor(0x1fd2,0x1000);
//14LV
HI541WriteCmosSensor(0x0b4e,0x1123);
HI541WriteCmosSensor(0x0b50,0x1000);
//15LV
HI541WriteCmosSensor(0x1fd4,0x1123);
HI541WriteCmosSensor(0x1fd6,0x1000);
//16LV
HI541WriteCmosSensor(0x0b52,0x1123);
HI541WriteCmosSensor(0x0b54,0x0000);
//17LV
HI541WriteCmosSensor(0x1fd8,0x1123);
HI541WriteCmosSensor(0x1fda,0x0000);
//Valid Illuminant Choose for Special Case
HI541WriteCmosSensor(0x0b56,0x1111);
HI541WriteCmosSensor(0x0b58,0x1111);

HI541WriteCmosSensor(0x20E8,0x0C0C);//Adaptive BLC stable count

HI541WriteCmosSensor(0x0AE8,0xffff);
//Color Map Offset Control(A, Horizon)
HI541WriteCmosSensor(0x0AEA,0xffff); //1 1'//illuminant R gain offset//1 1'//illuminant R gain offset
HI541WriteCmosSensor(0x0AEC,0xffff); //2 2'//2 2'
HI541WriteCmosSensor(0x0AEE,0xffff); //3 3'//3 3' D65
HI541WriteCmosSensor(0x0AF0,0xffff); //4 4'//4 4' Office
HI541WriteCmosSensor(0x0AF2,0xffff); //5 5'//5 5' CWF
HI541WriteCmosSensor(0x0AF4,0xffff); //6 6'//6 6' TL84
HI541WriteCmosSensor(0x0AF6,0xffff); //7 7'//7 7' A
HI541WriteCmosSensor(0x0AF8,0xffff); //8 8'//8 8' Horizon

HI541WriteCmosSensor(0x0AFA,0xffff); //1 1' //illuminant B gain offset
HI541WriteCmosSensor(0x0AFC,0xffff); //2 2'//2 2'
HI541WriteCmosSensor(0x0AFE,0xffff); //3 3' D65//3 3'
HI541WriteCmosSensor(0x0B00,0xffff); //4 4' Office//4 4'
HI541WriteCmosSensor(0x0B02,0xffff); //5 5' CWF//5 5'
HI541WriteCmosSensor(0x0B04,0xffff); //6 6' TL84//6 6'
HI541WriteCmosSensor(0x0B06,0xffff); //7 7' A//7 7'
HI541WriteCmosSensor(0x0B08,0xffff); //8 8' Horizon//8 8'



//Blue sky tracking Exception
HI541WriteCmosSensor(0x0B0C,0x00C3);

HI541WriteCmosSensor(0x0836,0x8f18);

//Standard light select
//HI541WriteCmosSensor(0x0a2e,0x0305);

//Extra map 
//HI541WriteCmosSensor(0x0b0c,0x00c3);

HI541WriteCmosSensor(0x0b0c,0x00c3);

//Green color tracking Exception
HI541WriteCmosSensor(0x0B10,0x6040);//Manual R gain//10,10'
HI541WriteCmosSensor(0x0B18,0x7040);//Manual B gain//10,10'

//Dark AWB Processing
HI541WriteCmosSensor(0x0B98,0x0005);
HI541WriteCmosSensor(0x0B9A,0x0500);
//HI541WriteCmosSensor(0x0834,0x0013);
HI541WriteCmosSensor(0x0834,0x0003);
HI541WriteCmosSensor(0x0a02,0x0103);// Hitcout min
HI541WriteCmosSensor(0x0a06,0x0001);// Hitcout min Hysteresis
//=================================
//=========== AWB END ==============
//=================================
//========================================
//=========== F/W AF Start ===============
//========================================
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x003E, 0x8080);			// AWB Lock
HI541WriteCmosSensor(0x0020, 0xff00); 			// IICM Delay

HI541WriteCmosSensor(0x05B4, 0x0af6); 			//0x05b5 : f8(8), f7(9), f6(10),  EC(20) macro shift 2014.05.27
HI541WriteCmosSensor(0x05B6, 0x3219);			// Intelligent Step Size = 20

HI541WriteCmosSensor(0x05B8, 0x0819);			// Fine Scan Step Size
HI541WriteCmosSensor(0x05Ba, 0x0003);			// Falling Threshold Number
//HI541WriteCmosSensor(0x05Bc, 0x0064);			// 삭제 (rev BE)
//HI541WriteCmosSensor(0x05C8, 0xC819);			// softlanding step / first scan outdoor/ 	 
//HI541WriteCmosSensor(0x05C0, 0x3300);			// Skip Frame
HI541WriteCmosSensor(0x05C0, 0x1100);			// Skip Frame
//HI541WriteCmosSensor(0x05C8, 0x6410);			// softlanding step / first scan outdoor/ 	 
//HI541WriteCmosSensor(0x05CA, 0x1010);			// first scan indoor / first scan dark
//HI541WriteCmosSensor(0x05C8, 0x6414);			// softlanding step / first scan outdoor/ 	 
//HI541WriteCmosSensor(0x05CA, 0x1414);			// first scan indoor / first scan dark
HI541WriteCmosSensor(0x05C8, 0x640C);			// softlanding step / first scan outdoor/ 	 
HI541WriteCmosSensor(0x05CA, 0x0C0C);			// first scan indoor / first scan dark
HI541WriteCmosSensor(0x05F4, 0x0200);	// Ae STD thresholg
HI541WriteCmosSensor(0x062A, 0x0E01);			// min duty		// 270
HI541WriteCmosSensor(0x062C, 0xBC02);			// max duty
HI541WriteCmosSensor(0x062e, 0x9001);			// marco min duty

HI541WriteCmosSensor(0x0668, 0x4F4F);			// Uniform Th (rev BE)
//HI541WriteCmosSensor(0x0668, 0x5050);			// Uniform Th (rev BE)
HI541WriteCmosSensor(0x066a, 0x4001);			// Hyper Position (rev BE)

HI541WriteCmosSensor(0x0680, 0x1111);			// weight
HI541WriteCmosSensor(0x0682, 0x1111);			// weight
HI541WriteCmosSensor(0x0684, 0x2828);			// center weight  


// Contrast chart
HI541WriteCmosSensor(0x0714, 0x2201);			// outdoor step LUT 1
HI541WriteCmosSensor(0x0716, 0x4A01);			// outdoor step LUT 2	
HI541WriteCmosSensor(0x0718, 0x7201);			// outdoor step LUT 3	
HI541WriteCmosSensor(0x071A, 0x9A01);			// outdoor step LUT 4	
HI541WriteCmosSensor(0x071C, 0xC201);			// outdoor step LUT 5	
HI541WriteCmosSensor(0x071E, 0xEA01);			// outdoor step LUT 6	
HI541WriteCmosSensor(0x0720, 0x1202);			// outdoor step LUT 7	
HI541WriteCmosSensor(0x0722, 0x3A02);			// outdoor step LUT 8	
HI541WriteCmosSensor(0x0724, 0x6202);			// outdoor step LUT 9	
HI541WriteCmosSensor(0x0726, 0x8A02);			// outdoor step LUT 10	
HI541WriteCmosSensor(0x0728, 0xBC02);			// outdoor step LUT 11	   
HI541WriteCmosSensor(0x072a, 0xBC02);			// outdoor step LUT 12
HI541WriteCmosSensor(0x072c, 0xBC02);			// outdoor step LUT 13
HI541WriteCmosSensor(0x072e, 0xBC02);			// outdoor step LUT 14
HI541WriteCmosSensor(0x0730, 0xBC02);			// outdoor step LUT 15
HI541WriteCmosSensor(0x0732, 0xBC02);			// outdoor step LUT 16
HI541WriteCmosSensor(0x0734, 0xBC02);			// outdoor step LUT 17
HI541WriteCmosSensor(0x0736, 0xBC02);			// outdoor step LUT 18
HI541WriteCmosSensor(0x0738, 0xBC02);			// outdoor step LUT 19
HI541WriteCmosSensor(0x073A, 0xBC02);			// outdoor step LUT 20
HI541WriteCmosSensor(0x073C, 0xBC02);			// outdoor step LUT 20


HI541WriteCmosSensor(0x0748, 0x2201);			// indoor step LUT 1
HI541WriteCmosSensor(0x074A, 0x4A01);			// indoor step LUT 2	
HI541WriteCmosSensor(0x074C, 0x7201);			// indoor step LUT 3	
HI541WriteCmosSensor(0x074E, 0x9A01);			// indoor step LUT 4	
HI541WriteCmosSensor(0x0750, 0xC201);			// indoor step LUT 5	
HI541WriteCmosSensor(0x0752, 0xEA01);			// indoor step LUT 6	
HI541WriteCmosSensor(0x0754, 0x1202);			// indoor step LUT 7	
HI541WriteCmosSensor(0x0756, 0x3A02);			// indoor step LUT 8	
HI541WriteCmosSensor(0x0758, 0x6202);			// indoor step LUT 9	
HI541WriteCmosSensor(0x075A, 0x8A02);			// indoor step LUT 10	
HI541WriteCmosSensor(0x075C, 0xBC02);
HI541WriteCmosSensor(0x075E, 0xBC02);			// 
HI541WriteCmosSensor(0x0760, 0xBC02);			//
HI541WriteCmosSensor(0x0762, 0xBC02);			//
HI541WriteCmosSensor(0x0764, 0xBC02);			//
HI541WriteCmosSensor(0x0766, 0xBC02);			//
HI541WriteCmosSensor(0x0768, 0xBC02);			//
HI541WriteCmosSensor(0x076A, 0xBC02);			//
HI541WriteCmosSensor(0x076C, 0xBC02);			//
HI541WriteCmosSensor(0x076E, 0xBC02);			//
HI541WriteCmosSensor(0x0770, 0xBC02);			//


HI541WriteCmosSensor(0x077C, 0x2201);			// dark step LUT 1
HI541WriteCmosSensor(0x077E, 0x4A01);			// dark step LUT 2	
HI541WriteCmosSensor(0x0780, 0x7201);			// dark step LUT 3	
HI541WriteCmosSensor(0x0782, 0x9A01);			// dark step LUT 4	
HI541WriteCmosSensor(0x0784, 0xC201);			// dark step LUT 5	
HI541WriteCmosSensor(0x0786, 0xEA01);			// dark step LUT 6	
HI541WriteCmosSensor(0x0788, 0x1202);			// dark step LUT 7	
HI541WriteCmosSensor(0x078A, 0x3A02);			// dark step LUT 8	
HI541WriteCmosSensor(0x078C, 0x6202);			// dark step LUT 9	
HI541WriteCmosSensor(0x078E, 0x8A02);			// dark step LUT 10	
HI541WriteCmosSensor(0x0790, 0xBC02);
HI541WriteCmosSensor(0x0792, 0xBC02);
HI541WriteCmosSensor(0x0794, 0xBC02);
HI541WriteCmosSensor(0x0796, 0xBC02);
HI541WriteCmosSensor(0x0798, 0xBC02);
HI541WriteCmosSensor(0x079A, 0xBC02);
HI541WriteCmosSensor(0x079C, 0xBC02);
HI541WriteCmosSensor(0x079E, 0xBC02);
HI541WriteCmosSensor(0x07A0, 0xBC02);
HI541WriteCmosSensor(0x07A2, 0xBC02);
HI541WriteCmosSensor(0x07A4, 0xBC02);


//// Continuous AF tune
//HI541WriteCmosSensor(0x05EA, 0x0005);  
//HI541WriteCmosSensor(0x05EC, 0x0c00);  
//HI541WriteCmosSensor(0x05F0, 0x0a0a);  
//HI541WriteCmosSensor(0x05F2, 0x0a00);                  
////HI541WriteCmosSensor(0x05F4, 0x0200);                
//HI541WriteCmosSensor(0x05F6, 0x0a28);                  
////HI541WriteCmosSensor(0x05F8, 0x0008);                
 
//HI541WriteCmosSensor(0x05EA, 0x0008);  			// First Ratio - FV Compare(Previous FV to Current FV)
//HI541WriteCmosSensor(0x05EC, 0x0800);  			// Second Ratio - FV Compare(Previous AF Done FV to Current FV)
//HI541WriteCmosSensor(0x05F0, 0x0408);  			// Unifome Ratio - 
//HI541WriteCmosSensor(0x05F2, 0x0800);                  
////HI541WriteCmosSensor(0x05F4, 0x0200);                
//HI541WriteCmosSensor(0x05F6, 0x0828);                  
////HI541WriteCmosSensor(0x05F8, 0x0008);                

HI541WriteCmosSensor(0x05EA, 0x0008);  			// First Ratio - FV Compare(Previous FV to Current FV)
HI541WriteCmosSensor(0x05EC, 0x0800);  			// Second Ratio - FV Compare(Previous AF Done FV to Current FV)
HI541WriteCmosSensor(0x05F0, 0x0606);  			// Unifome Ratio - 
HI541WriteCmosSensor(0x05F2, 0x0600);                  
HI541WriteCmosSensor(0x05F6, 0x0828);                  


  
HI541WriteCmosSensor(0x0628, 0x1E15);  			// Slpope Step / Delay 

HI541WriteCmosSensor(0x406E, 0x0014);			// Skip Frame	 
HI541WriteCmosSensor(0x4070, 0x1414);			// Skip Frame	 
              

HI541WriteCmosSensor(0x05C0, 0x1100);			// Skip Frame	 
HI541WriteCmosSensor(0x05b0, 0x8616);			//  	 

//===================================================
//========= Luminance Adaptive ISP Start  ===========
//===================================================
HI541WriteCmosSensor(0xffff,0x0020);

//luminance ADP th
HI541WriteCmosSensor(0x1B08,0x9CA8);// Dark2 TH 6.67fps AGx8 
HI541WriteCmosSensor(0x1B0a,0x0003);

HI541WriteCmosSensor(0x1B0c,0x4E54);// Dark1 TH 6.67fps AGx4
HI541WriteCmosSensor(0x1B0e,0x8001);

HI541WriteCmosSensor(0x1B10,0xa068);// Indoor TH 100fps AGx1
HI541WriteCmosSensor(0x1B12,0x0600);

HI541WriteCmosSensor(0x1B14,0xd90e);// Outdoor TH Expmin AGx1
HI541WriteCmosSensor(0x1B16,0x0000);

//// AD Outdoor
HI541WriteCmosSensor(0x12B4, 0x0001); // 0x9000 : Saturation ENB Transfer Function //20140325 global Sat. On 0x12b5 B[0] on:1 off:0 
HI541WriteCmosSensor(0x12B6, 0x0000); // 0x7034 : pre_flat_ctl, 	0x7035 : flat_max_luml1
HI541WriteCmosSensor(0x12B8, 0x0000); // 0x7036 : flat_max_lumh1, 	0x7037 : flat_max_luml2
HI541WriteCmosSensor(0x12BA, 0x0000); // 0x7038 : flat_max_lumh2, 	0x7039 : flat_max_dyl1
HI541WriteCmosSensor(0x12BC, 0x0000); // 0x703a : flat_max_dyh1,	0x703b : flat_max_dyl2
HI541WriteCmosSensor(0x12BE, 0x0000); // 0x703c : flat_max_dyh2, 	0x703d : flat_max_stdl1
HI541WriteCmosSensor(0x12C0, 0x0000); // 0x703e : flat_max_stdh1,  	0x703f : flat_max_stdl2
HI541WriteCmosSensor(0x12C2, 0x003f); // 0x7040 : flat_max_stdh2, 	0x742c : nr_lum_ofs_1
HI541WriteCmosSensor(0x12C4, 0x3f3f); // 0x742d : nr_lum_ofs_2,	0x742e : nr_lum_ofs_3 //20140325 black hair
HI541WriteCmosSensor(0x12C6, 0x3f3f); // 0x742f : nr_lum_ofs_4,	0x7430 : nr_lum_ofs_5 //20140325 black hair
HI541WriteCmosSensor(0x12C8, 0x3f3C); // 0x7431 : nr_lum_ofs_6,	0x7432 : nr_lum_ofs_7
HI541WriteCmosSensor(0x12CA, 0x3A2e); // 0x7433 : nr_lum_ofs_8,	0x7434 : nr_imp_th_1 //20130523 impulse noise
HI541WriteCmosSensor(0x12CC, 0x2d2c); // 0x7435 : nr_imp_th_2,	0x7436 : nr_imp_th_3 //20130523 impulse noise
HI541WriteCmosSensor(0x12CE, 0x241a); // 0x7437 : nr_imp_th_4,	0x7438 : nr_imp_th_5
HI541WriteCmosSensor(0x12D0, 0x1715); // 0x7439 : nr_imp_th_6, 	0x743a : nr_imp_th_7
HI541WriteCmosSensor(0x12D2, 0x1400); // error 0x12d3 0x743b : nr_imp_th_8,	0x743c : nr_rate
HI541WriteCmosSensor(0x12D4, 0x003f); // error 0x12d4 0x743d : line_rate,	0x7800 : splum_gain_pos_1
HI541WriteCmosSensor(0x12D6, 0x3d30); // 0x7801 : splum_gain_pos_2, 0x7802 : splum_gain_pos_3
HI541WriteCmosSensor(0x12D8, 0x2524); // 0x7803 : splum_gain_pos_4, 0x7804 : splum_gain_pos_5
HI541WriteCmosSensor(0x12DA, 0x2322); // 0x7805 : splum_gain_pos_6, 0x7806 : splum_gain_pos_7
HI541WriteCmosSensor(0x12DC, 0x203f); // 0x7807 : splum_gain_pos_8, 0x7808 : splum_gain_neg_1
HI541WriteCmosSensor(0x12DE, 0x3d32); // 0x7809 : splum_gain_neg_2, 0x780a : splum_gain_neg_3
HI541WriteCmosSensor(0x12E0, 0x2c26); // 0x780b : splum_gain_neg_4, 0x780c : splum_gain_neg_5
HI541WriteCmosSensor(0x12E2, 0x2625); // 0x780d : splum_gain_neg_6, 0x780e : splum_gain_neg_7
HI541WriteCmosSensor(0x12E4, 0x2526); // 0x780f : splum_gain_neg_8, 0x7810 : spdy_gain_pos_1
HI541WriteCmosSensor(0x12E6, 0x2627); // 0x7811 : spdy_gain_pos_2,	0x7812 : spdy_gain_pos_3
HI541WriteCmosSensor(0x12E8, 0x2727); // 0x7813 : spdy_gain_pos_4,	0x7814 : spdy_gain_pos_5
HI541WriteCmosSensor(0x12EA, 0x2522); // 0x7815 : spdy_gain_pos_6,	0x7816 : spdy_gain_pos_7
HI541WriteCmosSensor(0x12EC, 0x2124); // 0x7817 : spdy_gain_pos_8,	0x7818 : spdy_gain_neg_1
HI541WriteCmosSensor(0x12EE, 0x2525); // 0x7819 : spdy_gain_neg_2,	0x781a : spdy_gain_neg_3
HI541WriteCmosSensor(0x12F0, 0x2525); // 0x781b : spdy_gain_neg_4,	0x781c : spdy_gain_neg_5
HI541WriteCmosSensor(0x12F2, 0x2525); // 0x781d : spdy_gain_neg_6,	0x781e : spdy_gain_neg_7
		HI541WriteCmosSensor(0x12F4, 0x2524); // 0x781f : spdy_gain_neg_8,	0x7820 : spedge_gain_1
		HI541WriteCmosSensor(0x12F6, 0x2830); // 0x7821 : spedge_gain_2,	0x7822 : spedge_gain_3
		HI541WriteCmosSensor(0x12F8, 0x3434); // 0x7823 : spedge_gain_4, 	0x7824 : spedge_gain_5
		HI541WriteCmosSensor(0x12FA, 0x3434); // 0x7825 : spedge_gain_6,	0x7826 : spedge_gain_7
HI541WriteCmosSensor(0x12FC, 0x221E); // 0x7827 : spedge_gain_8,	0x7830 : spstd_gain_1
HI541WriteCmosSensor(0x12FE, 0x2226); // 0x7831 : spstd_gain_2,	0x7832 : spstd_gain_3
HI541WriteCmosSensor(0x1300, 0x2828); // 0x7833 : spstd_gain_4,	0x7834 : spstd_gain_5
HI541WriteCmosSensor(0x1302, 0x2420); // 0x7835 : spstd_gain_6, 	0x7836 : spstd_gain_7
HI541WriteCmosSensor(0x1304, 0x1B68); // 0x7837 : spstd_gain_8,     0x7838 : post_std_sel //20140522 skin detail 79 -> 68 -> 46 
HI541WriteCmosSensor(0x1306, 0x0b36); // 0x7440 : flt_luml1,	0x7441 : flt_lumh1	//20140325 Black hair 08 -> 0b
HI541WriteCmosSensor(0x1308, 0x43e4); // 0x7442 : flt_luml2,	0x7443 : flt_lumh2	
HI541WriteCmosSensor(0x130A, 0x0407); // 0x7444 : flt_stdl1,	0x7445 : flt_stdh1	
HI541WriteCmosSensor(0x130C, 0x0121); // 0x7446 : flt_stdl2,	0x7447 : flt_stdh2 	
HI541WriteCmosSensor(0x130E, 0x0105); // 0x7448 : flt_dyl1,		0x7449 : flt_dyh1		
HI541WriteCmosSensor(0x1310, 0x0125); // 0x744a : flt_dyl2,		0x744b : flt_dyh2		
HI541WriteCmosSensor(0x1312, 0x2232); // 0x7458 : flt_rate1,	0x744c : flt_lumll1	
HI541WriteCmosSensor(0x1314, 0x462b); // 0x744d : flt_lumhh1,	0x744e : flt_lumll2 	//20140512_flat for ???
HI541WriteCmosSensor(0x1316, 0xe401); // 0x744f : flt_lumhh2,	0x7450 : flt_stdll1	
HI541WriteCmosSensor(0x1318, 0x1001); // 0x7451 : flt_stdhh1,	0x7452 : flt_stdll2 	
HI541WriteCmosSensor(0x131A, 0x1001); // 0x7453 : flt_stdhh2,	0x7454 : flt_dyll1
HI541WriteCmosSensor(0x131C, 0x0601); // 0x7455 : flt_dyhh1,	0x7456 : flt_dyll2 	
HI541WriteCmosSensor(0x131E, 0x0a51); // 0x7457 : flt_dyhh2, 	0x7459 : flt_rate2 //20140527 filter rate 6 -> 30 for neck
HI541WriteCmosSensor(0x1320, 0x1516); // 0x7465 : highgain,		0x7466 : lowgain //20140528 high gain 1a -> 15
HI541WriteCmosSensor(0x1322, 0x772F); // 0x746b : cent_flt_sel,	0x746c : sharp_sel
HI541WriteCmosSensor(0x1324, 0x0950); // 0x746d : resol_ctl,	0x746e : flat_sharpgainh
HI541WriteCmosSensor(0x1326, 0x1060); // 0x746f : flat_sharpgainl,  0x7470 : skin_sharpgainh //20140523 skin detail 69 -> 65
HI541WriteCmosSensor(0x1328, 0x3050); // 0x7471 : skin_sharpgainl, 	0x7472 : byr_sp_line //20140325 skin detail 34 -> 31 -> 30
HI541WriteCmosSensor(0x132A, 0x7F33); // 0x7488 : skin_cyan_ctl,	0x7489 : flat_max_th // 20140313 bayer skin rate c -> 9 -> 3
HI541WriteCmosSensor(0x132C, 0x91A4); // 0x7c04 : ci_lpf_ctl_01,	0x7c05 : ci_lpf_ctl_02
HI541WriteCmosSensor(0x132E, 0x3C3f); // 0x7c06 : ci_lpf_ctl_03,	0x7c0e : ci_lum_ofs_1
HI541WriteCmosSensor(0x1330, 0x3f3f); // 0x7c0f : ci_lum_ofs_2,  	0x7c10 : ci_lum_ofs_3
HI541WriteCmosSensor(0x1332, 0x3f3f); // 0x7c11 : ci_lum_ofs_4,  	0x7c12 : ci_lum_ofs_5
HI541WriteCmosSensor(0x1334, 0x3f3f); // 0x7c13 : ci_lum_ofs_6, 	0x7c14 : ci_lum_ofs_7
HI541WriteCmosSensor(0x1336, 0x3f55); // 0x7c15 : ci_lum_ofs_8,  	0x7c16 : ci_ps_dy_ctl //20140315 false color
HI541WriteCmosSensor(0x1338, 0xe003); // 0x7c17 : ci_ps_max_th,	0x7c18 : ci_ps_min_th //20140512 false color 84 -> e0 20140523 06->03
HI541WriteCmosSensor(0x133A, 0x1012); // 0x7c19 : ci_ln_ps_gain_1,	0x7c1a : ci_ln_ps_gain_2
HI541WriteCmosSensor(0x133C, 0x171d); // 0x7c1b : ci_ln_ps_gain_3,	0x7c1c : ci_ln_ps_gain_4  //20140324 R3 : 17
HI541WriteCmosSensor(0x133E, 0x1b1c); // 0x7c1d : ci_ln_ps_gain_5,	0x7c1e : ci_ln_ps_gain_6
HI541WriteCmosSensor(0x1340, 0x1012); // 0x7c1f : ci_dy_ps_gain_1,	0x7c20 : ci_dy_ps_gain_2 //20140514 for ringing
HI541WriteCmosSensor(0x1342, 0x181c); // 0x7c21 : ci_dy_ps_gain_3,	0x7c22 : ci_dy_ps_gain_4 //20140514 for ringing
HI541WriteCmosSensor(0x1344, 0x1b1c); // 0x7c23 : ci_dy_ps_gain_5,	0x7c24 : ci_dy_ps_gain_6 //20140514 for ringing
HI541WriteCmosSensor(0x1346, 0x00e0); // 0x7c25 : ci_ps_type_ctl, 	0x7c26 : ci_ln_min_th //20140315 false color // 20140611 false color
HI541WriteCmosSensor(0x1348, 0x223F); // 0x7c27 : ci_dy_min_th,	0x940c : ynr_lum_gain_1 //2014527 false color // 20140611 false color
HI541WriteCmosSensor(0x134A, 0x3F3F); // 0x940d : ynr_lum_gain_2, 	0x940e : ynr_lum_gain_3 
HI541WriteCmosSensor(0x134C, 0x3F3F); // 0x940f : ynr_lum_gain_4,	0x9410 : ynr_lum_gain_5
HI541WriteCmosSensor(0x134E, 0x332F); // 0x9411 : ynr_lum_gain_6, 	0x9412 : ynr_lum_gain_7
HI541WriteCmosSensor(0x1350, 0x2E02); // 0x9413 : ynr_lum_gain_8,	0x940a : ynr_line_rate
HI541WriteCmosSensor(0x1352, 0x1a25); // 0x940b : ynr_flat_ctl,	0x9414 : ynr_flat_luml1
HI541WriteCmosSensor(0x1354, 0x6263); // 0x9415 : ynr_flat_lumh1,  	0x9416 : ynr_flat_luml2 
HI541WriteCmosSensor(0x1356, 0xff04); // 0x9417 : ynr_flat_lumh2,	0x9418 : ynr_flat_stdl1
HI541WriteCmosSensor(0x1358, 0x2005); // 0x9419 : ynr_flat_stdh1,	0x941a : ynr_flat_stdl2
HI541WriteCmosSensor(0x135A, 0x2002); // 0x941b : ynr_flat_stdh2,	0x941c : ynr_flat_dyl1
HI541WriteCmosSensor(0x135C, 0x2006); // 0x941d : ynr_flat_dyh1,	0x941e : ynr_flat_dyl2
HI541WriteCmosSensor(0x135E, 0x2008); // 0x941f : ynr_flat_dyh2,	0x9426 : cnr_cf_cb_awm_th
HI541WriteCmosSensor(0x1360, 0x0AFD); // 0x9427 : cnr_cf_cr_awm_th,	0x9448 : ac_mul_ctl12_cb
HI541WriteCmosSensor(0x1362, 0xA7FD); // 0x9449 : ac_mul_ctl34_cb,	0x944a : ac_mul_ctl12_cr
HI541WriteCmosSensor(0x1364, 0xA754); // 0x944b : ac_mul_ctl34_cr,	0x944e : cnr_lum_min_ctl
HI541WriteCmosSensor(0x1366, 0x0123); // 0x9455 : cnr_dc_stdm12,	0x9456 : cnr_dc_stdm34
HI541WriteCmosSensor(0x1368, 0x44B3); // 0x9457 : cnr_dc_stdm56,	0x9800 : yee_ctl
HI541WriteCmosSensor(0x136A, 0xD810); // 0x9801 : yee_index_gain,	0x9802 : yee_status_gain
HI541WriteCmosSensor(0x136C, 0x0310); // 0x9803 : yee_lo_filter,	0x9804 : yee_lo_gain
HI541WriteCmosSensor(0x136E, 0x7B1E); // 0x9805 : yee_hi_filter,	0x9806 : yee_hi_gain
HI541WriteCmosSensor(0x1370, 0x400C); // 0x9807 : yee_flat_ctl,	0x9808 : yee_skin_low_gain
HI541WriteCmosSensor(0x1372, 0x0C18); // 0x9809 : yee_skin_higgain,	0x980a : yee_std_post_gain_pos //22->14->16
HI541WriteCmosSensor(0x1374, 0x1626); // 0x980b : yee_std_gain,	0x980c : yee_lum_gain_p1 //24->0c->14
HI541WriteCmosSensor(0x1376, 0x2423); // 0x980d : yee_lum_gain_p2,	0x980e : yee_lum_gain_p3
HI541WriteCmosSensor(0x1378, 0x1c13); // 0x980f : yee_lum_gain_p4,  0x9810 : yee_lum_gain_p5 
HI541WriteCmosSensor(0x137A, 0x1218); // 0x9811 : yee_lum_gain_p6,  0x9812 : yee_lum_gain_p7
HI541WriteCmosSensor(0x137C, 0x1A23); // 0x9813 : yee_lum_gain_p8,  0x9814 : yee_lum_gain_n1
HI541WriteCmosSensor(0x137E, 0x2322); // 0x9815 : yee_lum_gain_n2,	0x9816 : yee_lum_gain_n3
HI541WriteCmosSensor(0x1380, 0x2226); // 0x9817 : yee_lum_gain_n4,	0x9818 : yee_lum_gain_n5 //20140512 white flower
HI541WriteCmosSensor(0x1382, 0x2730); // 0x9819 : yee_lum_gain_n6,	0x981a : yee_lum_gain_n7 //20140512 white flower
HI541WriteCmosSensor(0x1384, 0x2725); // 0x981b : yee_lum_gain_n8,	0x981c : yee_dy_gain_p1  //20140512 white flower
HI541WriteCmosSensor(0x1386, 0x2525); // 0x981d : yee_dy_gain_p2,	0x981e : yee_dy_gain_p3
HI541WriteCmosSensor(0x1388, 0x2221); // 0x981f : yee_dy_gain_p4,	0x9820 : yee_dy_gain_p5
HI541WriteCmosSensor(0x138A, 0x201E); // 0x9821 : yee_dy_gain_p6,	0x9822 : yee_dy_gain_p7
HI541WriteCmosSensor(0x138C, 0x1B21); // 0x9823 : yee_dy_gain_p8,	0x9824 : yee_dy_gain_n1
HI541WriteCmosSensor(0x138E, 0x2121); // 0x9825 : yee_dy_gain_n2,	0x9826 : yee_dy_gain_n3
HI541WriteCmosSensor(0x1390, 0x2020); // 0x9827 : yee_dy_gain_n4,	0x9828 : yee_dy_gain_n5
HI541WriteCmosSensor(0x1392, 0x2020); // 0x9829 : yee_dy_gain_n6,	0x982a : yee_dy_gain_n7
HI541WriteCmosSensor(0x1394, 0x1F22); // 0x982b : yee_dy_gain_n8,	0x982c : yee_edge_gain_1
HI541WriteCmosSensor(0x1396, 0x2222); // 0x982d : yee_edge_gain_2,	0x982e : yee_edge_gain_3
HI541WriteCmosSensor(0x1398, 0x2120); // 0x982f : yee_edge_gain_4,	0x9830 : yee_edge_gain_5
HI541WriteCmosSensor(0x139A, 0x2020); // 0x9831 : yee_edge_gain_6,	0x9832 : yee_edge_gain_7
HI541WriteCmosSensor(0x139C, 0x1F1C); // 0x9833 : yee_edge_gain_8,  0x983c : yee_std_gain_1
HI541WriteCmosSensor(0x139E, 0x2022); // 0x983d : yee_std_gain_2,	0x983e : yee_std_gain_3
HI541WriteCmosSensor(0x13A0, 0x2426); // 0x983f : yee_std_gain_4, 	0x9840 : yee_std_gain_5
HI541WriteCmosSensor(0x13A2, 0x2622); // 0x9841 : yee_std_gain_6,	0x9842 : yee_std_gain_7
HI541WriteCmosSensor(0x13A4, 0x2013); // 0x9843 : yee_std_gain_8,	0x9865 : cnr_color_region_ctl
HI541WriteCmosSensor(0x13A6, 0x503F); // 0x9866 : cnr_color_sat_th, 0x7c2a : ci_nrlum_ofst_1 //20140610 ?????
HI541WriteCmosSensor(0x13A8, 0x3F3F); // 0x7c2b : ci_nrlum_ofst_2, 	0x7c2c : ci_nrlum_ofst_3
HI541WriteCmosSensor(0x13AA, 0x3F3F); // 0x7c2d : ci_nrlum_ofst_4, 	0x7c2e : ci_nrlum_ofst_5
HI541WriteCmosSensor(0x13AC, 0x3C38); // 0x7c2f : ci_nrlum_ofst_6, 	0x7c30 : ci_nrlum_ofst_7
HI541WriteCmosSensor(0x13AE, 0x35D2); // 0x7c31 : ci_nrlum_ofst_8,	0x7c3f : ci_highgain
HI541WriteCmosSensor(0x13B0, 0x1C00); // 0x7c40 : ci_lowgain,	0x7c41 : ci_sp_pre_dif_ctl
HI541WriteCmosSensor(0x13B2, 0x40FF); // 0x7c42 : ci_flt_luml1,	0x7c43 : ci_flt_lumh1	
HI541WriteCmosSensor(0x13B4, 0x0f3f); // 0x7c44 : ci_flt_luml2,	0x7c45 : ci_flt_lumh2 //20140522 black hair 0c -> 0f
HI541WriteCmosSensor(0x13B6, 0x0111); // 0x7c46 : ci_flt_stdl1,	0x7c47 : ci_flt_stdh1 //20140610 ?????
HI541WriteCmosSensor(0x13B8, 0x0107); // 0x7c48 : ci_flt_stdl2,	0x7c49 : ci_flt_stdh2
HI541WriteCmosSensor(0x13BA, 0x0111); // 0x7c4a : ci_flt_dyl1,	0x7c4b : ci_flt_dyh1 //20140610 ?????
HI541WriteCmosSensor(0x13BC, 0x010E); // 0x7c4c : ci_flt_dyl2,	0x7c4d : ci_flt_dyh2
HI541WriteCmosSensor(0x13BE, 0x313c); // 0x7c4e : ci_flat_ctl,	0x7c50 : ci_splum_n1
HI541WriteCmosSensor(0x13C0, 0x2f1e); // 0x7c51 : ci_splum_n2,	0x7c52 : ci_splum_n3
HI541WriteCmosSensor(0x13C2, 0x1c1c); // 0x7c53 : ci_splum_n4,	0x7c54 : ci_splum_n5
HI541WriteCmosSensor(0x13C4, 0x1c1c); // 0x7c55 : ci_splum_n6,	0x7c56 : ci_splum_n7
HI541WriteCmosSensor(0x13C6, 0x1c20); // 0x7c57 : ci_splum_n8,	0x7c58 : ci_splum_p1 //20140527 hair for pattern noise
HI541WriteCmosSensor(0x13C8, 0x2820); // 0x7c59 : ci_splum_p2,	0x7c5a : ci_splum_p3 //20140527 hair for pattern noise
HI541WriteCmosSensor(0x13CA, 0x1f20); // 0x7c5b : ci_splum_p4,	0x7c5c : ci_splum_p5 //20140522 for pattern noise
HI541WriteCmosSensor(0x13CC, 0x1F1E); // 0x7c5d : ci_splum_p6,	0x7c5e : ci_splum_p7
HI541WriteCmosSensor(0x13CE, 0x1C20); // 0x7c5f : ci_splum_p8,	0x7c60 : ci_spdy_n1
HI541WriteCmosSensor(0x13D0, 0x201a); // 0x7c61 : ci_spdy_n2,	0x7c62 : ci_spdy_n3 // 20140613 blue text
HI541WriteCmosSensor(0x13D2, 0x101a); // 0x7c63 : ci_spdy_n4,	0x7c64 : ci_spdy_n5 // 20140613 blue text
HI541WriteCmosSensor(0x13D4, 0x2021); // 0x7c65 : ci_spdy_n6,	0x7c66 : ci_spdy_n7
HI541WriteCmosSensor(0x13D6, 0x2326); // 0x7c67 : ci_spdy_n8,	0x7c68 : ci_spdy_p1
HI541WriteCmosSensor(0x13D8, 0x2626); // 0x7c69 : ci_spdy_p2,	0x7c6a : ci_spdy_p3
HI541WriteCmosSensor(0x13DA, 0x2625); // 0x7c6b : ci_spdy_p4,	0x7c6c : ci_spdy_p5
HI541WriteCmosSensor(0x13DC, 0x2523); // 0x7c6d : ci_spdy_p6,	0x7c6e : ci_spdy_p7
HI541WriteCmosSensor(0x13DE, 0x201E); // 0x7c6f : ci_spdy_p8,	0x7c70 : ci_spedge_1
HI541WriteCmosSensor(0x13E0, 0x1E1E); // 0x7c71 : ci_spedge_2,	0x7c72 : ci_spedge_3
HI541WriteCmosSensor(0x13E2, 0x1F1F); // 0x7c73 : ci_spedge_4,	0x7c74 : ci_spedge_5
HI541WriteCmosSensor(0x13E4, 0x1F1F); // 0x7c75 : ci_spedge_6,	0x7c76 : ci_spedge_7
HI541WriteCmosSensor(0x13E6, 0x1F14); // 0x7c77 : ci_spedge_8,	0x7c7d : ci_spstd_1
HI541WriteCmosSensor(0x13E8, 0x1822); // 0x7c7e : ci_spstd_2,	0x7c7f : ci_spstd_3
HI541WriteCmosSensor(0x13EA, 0x2626); // 0x7c80 : ci_spstd_4,	0x7c81 : ci_spstd_5
HI541WriteCmosSensor(0x13EC, 0x2421); // 0x7c82 : ci_spstd_6,	0x7c83 : ci_spstd_7
HI541WriteCmosSensor(0x13EE, 0x1812); // 0x7c84 : ci_spstd_8,	0x7c85 : ci_post_std_ctl

HI541WriteCmosSensor(0x13F0, 0x0000); // 0x7c97 : ci_skin_blue_ctl,	0x9004 : sat_tra_sign_dk																//20140714 for CKT
HI541WriteCmosSensor(0x13F2, 0x00d0); // 0x9005 : sat_tra_deltah_dk,0x9008 : sat_tra_delta1_dk                                                              //20140714 for CKT
HI541WriteCmosSensor(0x13F4, 0x7c58); // 0x9009 : sat_tra_delta2_dk,0x900a : sat_tra_delta3_dk                                                              //20140714 for CKT
HI541WriteCmosSensor(0x13F6, 0x0530); // 0x900b : sat_tra_delta4_dk,0x900c : sat_tra_yb1_dk //20140325 add Trans func. graph for suppress R3                //20140714 for CKT
HI541WriteCmosSensor(0x13F8, 0x3500); // 0x900e : sat_tra_yb2_dk_h,	0x900f : sat_tra_yb2_dk_l //20140325 add Trans func. graph for suppress R3 // skin      //20140714 for CKT
HI541WriteCmosSensor(0x13FA, 0x38b0); // 0x9010 : sat_tra_yb3_dk_h,	0x9011 : sat_tra_yb3_dk_l                                                               //20140714 for CKT
HI541WriteCmosSensor(0x13FC, 0x3e00); // 0x9012 : sat_tra_yb4_dk_h,	0x9013 : sat_tra_yb4_dk_l //20140325 add Trans func. graph for suppress R3 // skin      //20140714 for CKT
HI541WriteCmosSensor(0x13FE, 0x070f); // 0x9014 : sat_tra_th1_dk,	0x9015 : sat_tra_th2_dk
HI541WriteCmosSensor(0x1400, 0x1e10); // 0x9016 : sat_tra_th3_dk,   0x9c02 : mcmc_ang_dlta1 //seperating MCMC Adaptive register

HI541WriteCmosSensor(0x1476, 0x0087); // 0x7c3c : ci_gain_std_ofst,	0x7c3d : ci_spflt_std_ctl
HI541WriteCmosSensor(0x1478, 0x1E7F); // 0x7400 : lensd_lpf_ctl1,	0x7401 : lpf_flt_sel
HI541WriteCmosSensor(0x147A, 0x0081); // 0x7404 : imp_ctl,		0x7405 : nr_lpf_sel_01
HI541WriteCmosSensor(0x147C, 0x0458); // 0x7406 : nr_lpf_sel_02,	0x7407 : nr_lpf_std_01 		, Rev 04 : r04_lpf median
HI541WriteCmosSensor(0x147E, 0x3012); // 0x7408 : nr_lpf_std_02,	0x7409 : nr_lpf_std_03 
HI541WriteCmosSensor(0x1480, 0x1200); // 0x740a : org_std_ctl,	0x740b : org_std_oft
HI541WriteCmosSensor(0x1482, 0x1310); // 0x743c : nr_rate,		0x743d : line_rate 
HI541WriteCmosSensor(0x1484, 0x8631); // 0x7464 : byr_sp_onoff,	0x7467 : sf_sel_std_sel 
HI541WriteCmosSensor(0x1486, 0x0115); // 0x7c28 : ci_cnr_ctl,	0x7c29 : ci_y_nr_ctl		, 01 <- Rev 77: [6:4] Morie Filter Rate Control, 7: median filter value max rate, [1] : median filter on
HI541WriteCmosSensor(0x1488, 0x4700); // 0x7c32 : ci_nrgain_std_ctl,0x7c33 : ci_nrgain_std_ofst
HI541WriteCmosSensor(0x148A, 0x0002); // 0x7c34 : ci_cbcr_gain,	0x7c35 : ci_dy_gain 
HI541WriteCmosSensor(0x148C, 0x9540); // 0x7c36 : ci_filt_sel_ctl,	0x7c37 : ci_yfilt_sel_h 
HI541WriteCmosSensor(0x148E, 0x2010); // 0x7c38 : ci_yfilt_sel_m,	0x7c39 : ci_yfilt_sel_l 
HI541WriteCmosSensor(0x1490, 0x9F00); // 0x7c3b : ci_spstd_ctl,	0x7c4f : ci_sp_ln_ctl 
HI541WriteCmosSensor(0x1492, 0x1F0F); // 0x7caa : ci_otp1,		0x7cac : ci_weak_hv_gain 	, Rev 1F : [2] str_hv_en, 0F : [3:2] Intra HV Gain [1:0] Inter HV Gain // moire stripe diff thr (diff_Esum > th)
HI541WriteCmosSensor(0x1494, 0x2C13); // 0x7cad : ci_moire_th1,	0x7cae : ci_moire_th2 		, 13 : gbgr lind diff thr    (diff_gbgr > th)
HI541WriteCmosSensor(0x1496, 0x3E90); // 0x8000 : cmc_ctl1,		0x8002 : cmc_ps_ln_cmc_gain
HI541WriteCmosSensor(0x1498, 0x0090); // 0x8800 : gma_ctl1,		0x8802 : gma_ps_ln_gain 
HI541WriteCmosSensor(0x149A, 0x0EDA); // 0x9401 : ynr_ctl2,		0x9402 : ynr_ctl3 
HI541WriteCmosSensor(0x149C, 0x2530); // 0x9403 : ynr_ctl4,		0x9404 : ynr_std_l_th 
HI541WriteCmosSensor(0x149E, 0x60A0); // 0x9405 : ynr_std_m_th,	0x9406 : ynr_std_h_th 
HI541WriteCmosSensor(0x14A0, 0x3F3F); // 0x7c86 : ci_splum_max_n1,	0x7c87 : ci_splum_max_n2
HI541WriteCmosSensor(0x14A2, 0x3F3F); // 0x7c88 : ci_splum_max_n3,	0x7c89 : ci_splum_max_n4
HI541WriteCmosSensor(0x14A4, 0x3F3F); // 0x7c8a : ci_splum_max_n5,	0x7c8b : ci_splum_max_p1
HI541WriteCmosSensor(0x14A6, 0x3F3F); // 0x7c8c : ci_splum_max_p2,	0x7c8d : ci_splum_max_p3
HI541WriteCmosSensor(0x14A8, 0x3F3F); // 0x7c8e : ci_splum_max_p4,	0x7c8f : ci_splum_max_p5
HI541WriteCmosSensor(0x14AA, 0x1313); // 0x7ca5 : ci_dy_sel_ctl,	0x7ca6 : ci_lum_sel_ctl
HI541WriteCmosSensor(0x1AFE, 0x1010); // 0x7c9b : bi_dy_th,         0x7c9a : bi_std_th // 20140520 for indoor diag noise (patch rev)

// Indoor
HI541WriteCmosSensor(0x14AC, 0x030D); // 0x9000 : Saturation ENB Transfer Function , 0x7034 : pre_flat_ctl 	
HI541WriteCmosSensor(0x14AE, 0x00FF); // 0x7035 : flat_max_luml1, 	0x7036 : flat_max_lumh1,
HI541WriteCmosSensor(0x14B0, 0x0000); // 0x7037 : flat_max_luml2, 	0x7038 : flat_max_lumh2, 	
HI541WriteCmosSensor(0x14B2, 0x0024); // 0x7039 : flat_max_dyl1,	0x703a : flat_max_dyh1,	
HI541WriteCmosSensor(0x14B4, 0x0000); // 0x703b : flat_max_dyl2,	0x703c : flat_max_dyh2, 	
HI541WriteCmosSensor(0x14B6, 0x001a); // 0x703d : flat_max_stdl1,  0x703e : flat_max_stdh1,  	
HI541WriteCmosSensor(0x14B8, 0x0000); // 0x703f : flat_max_stdl2,	0x7040 : flat_max_stdh2, 
HI541WriteCmosSensor(0x14BA, 0x3F3F); // 0x742c : nr_lum_ofs_1,	0x742d : nr_lum_ofs_2,	
HI541WriteCmosSensor(0x14BC, 0x3F3D); // 0x742e : nr_lum_ofs_3,  	0x742f : nr_lum_ofs_4,	
HI541WriteCmosSensor(0x14BE, 0x3C38); // 0x7430 : nr_lum_ofs_5,	0x7431 : nr_lum_ofs_6,	
HI541WriteCmosSensor(0x14C0, 0x3634); // 0x7432 : nr_lum_ofs_7,	0x7433 : nr_lum_ofs_8,	
HI541WriteCmosSensor(0x14C2, 0x3F3F); // 0x7434 : nr_imp_th_1,	0x7435 : nr_imp_th_2,	
HI541WriteCmosSensor(0x14C4, 0x3E38); // 0x7436 : nr_imp_th_3,	0x7437 : nr_imp_th_4,	
HI541WriteCmosSensor(0x14C6, 0x3331); // 0x7438 : nr_imp_th_5,	0x7439 : nr_imp_th_6, 	
HI541WriteCmosSensor(0x14C8, 0x3030); // 0x743a : nr_imp_th_7,	0x743b : nr_imp_th_8,	
HI541WriteCmosSensor(0x14CA, 0x2018); // error 0x14CA, 0x14CB 0x743c : nr_rate,		0x743d : line_rate,	
HI541WriteCmosSensor(0x14CC, 0x1C1C); // 0x7800 : splum_gain_pos_1,0x7801 : splum_gain_pos_2,
HI541WriteCmosSensor(0x14CE, 0x1A1A); // 0x7802 : splum_gain_pos_3,0x7803 : splum_gain_pos_4, 
HI541WriteCmosSensor(0x14D0, 0x1A19); // 0x7804 : splum_gain_pos_5,0x7805 : splum_gain_pos_6, 
HI541WriteCmosSensor(0x14D2, 0x1818); // 0x7806 : splum_gain_pos_7,0x7807 : splum_gain_pos_8, 
HI541WriteCmosSensor(0x14D4, 0x1E1E); // 0x7808 : splum_gain_neg_1,0x7809 : splum_gain_neg_2, 
HI541WriteCmosSensor(0x14D6, 0x1A1A); // 0x780a : splum_gain_neg_3,0x780b : splum_gain_neg_4, 
HI541WriteCmosSensor(0x14D8, 0x1A1A); // 0x780c : splum_gain_neg_5,0x780d : splum_gain_neg_6, 
HI541WriteCmosSensor(0x14DA, 0x1A1A); // 0x780e : splum_gain_neg_7,0x780f : splum_gain_neg_8, 
HI541WriteCmosSensor(0x14DC, 0x201C); // 0x7810 : spdy_gain_pos_1,	0x7811 : spdy_gain_pos_2,	
HI541WriteCmosSensor(0x14DE, 0x1C1B); // 0x7812 : spdy_gain_pos_3, 0x7813 : spdy_gain_pos_4,	                
HI541WriteCmosSensor(0x14E0, 0x1A1A); // 0x7814 : spdy_gain_pos_5, 0x7815 : spdy_gain_pos_6,	
HI541WriteCmosSensor(0x14E2, 0x1A1A); // 0x7816 : spdy_gain_pos_7, 0x7817 : spdy_gain_pos_8,	
HI541WriteCmosSensor(0x14E4, 0x1E1E); // 0x7818 : spdy_gain_neg_1, 0x7819 : spdy_gain_neg_2,	                
HI541WriteCmosSensor(0x14E6, 0x1E1F); // 0x781a : spdy_gain_neg_3, 0x781b : spdy_gain_neg_4,	
HI541WriteCmosSensor(0x14E8, 0x1F1F); // 0x781c : spdy_gain_neg_5, 0x781d : spdy_gain_neg_6,	
HI541WriteCmosSensor(0x14EA, 0x1F1E); // 0x781e : spdy_gain_neg_7, 0x781f : spdy_gain_neg_8,	
		HI541WriteCmosSensor(0x14EC, 0x1820); // 0x7820 : spedge_gain_1,   0x7821 : spedge_gain_2,	
		HI541WriteCmosSensor(0x14EE, 0x2428); // 0x7822 : spedge_gain_3,   0x7823 : spedge_gain_4, 	
		HI541WriteCmosSensor(0x14F0, 0x2828); // 0x7824 : spedge_gain_5,   0x7825 : spedge_gain_6,	
		HI541WriteCmosSensor(0x14F2, 0x2828); // 0x7826 : spedge_gain_7,   0x7827 : spedge_gain_8,	
HI541WriteCmosSensor(0x14F4, 0x1011); // 0x7830 : spstd_gain_1,    0x7831 : spstd_gain_2,	
HI541WriteCmosSensor(0x14F6, 0x1315); // 0x7832 : spstd_gain_3,    0x7833 : spstd_gain_4,	
HI541WriteCmosSensor(0x14F8, 0x1517); // 0x7834 : spstd_gain_5,    0x7835 : spstd_gain_6,                          
HI541WriteCmosSensor(0x14FA, 0x1411); // 0x7836 : spstd_gain_7,    0x7837 : spstd_gain_8,                          
HI541WriteCmosSensor(0x14FC, 0xcb16); // 0x7838 : post_std_sel,	0x7440 : flt_luml1, //20140319 indoor Bayer flat
HI541WriteCmosSensor(0x14FE, 0x3F40); // 0x7441 : flt_lumh1,	0x7442 : flt_luml2,                             
HI541WriteCmosSensor(0x1500, 0xFF02); // 0x7443 : flt_lumh2,	0x7444 : flt_stdl1,	
HI541WriteCmosSensor(0x1502, 0x0C01); // 0x7445 : flt_stdh1,	0x7446 : flt_stdl2,	
HI541WriteCmosSensor(0x1504, 0x0701); // 0x7447 : flt_stdh2, 	0x7448 : flt_dyl1,
HI541WriteCmosSensor(0x1506, 0x0E01); // 0x7449 : flt_dyh1,	0x744a : flt_dyl2,	                    
HI541WriteCmosSensor(0x1508, 0x0777); // 0x744b : flt_dyh2,	0x7458 : flt_rate1,  //20140319 indoor Bayer flat			        
HI541WriteCmosSensor(0x150A, 0x327A); // 0x744c : flt_lumll1,	0x744d : flt_lumhh1,	
HI541WriteCmosSensor(0x150C, 0x0000); // 0x744e : flt_lumll2, 	0x744f : flt_lumhh2,	
HI541WriteCmosSensor(0x150E, 0x012f); // 0x7450 : flt_stdll1,	0x7451 : flt_stdhh1,	//20140319 indoor
HI541WriteCmosSensor(0x1510, 0x0000); // 0x7452 : flt_stdll2, 	0x7453 : flt_stdhh2,	
HI541WriteCmosSensor(0x1512, 0x011D); // 0x7454 : flt_dyll1,       0x7455 : flt_dyhh1,	
HI541WriteCmosSensor(0x1514, 0x0000); // 0x7456 : flt_dyll2, 	0x7457 : flt_dyhh2, 	
HI541WriteCmosSensor(0x1516, 0x771A); // 0x7459 : flt_rate2,       0x7465 : highgain,	//20140319 indoor Bayer flat
HI541WriteCmosSensor(0x1518, 0x1636); // 0x7466 : lowgain,         0x746b : cent_flt_sel,	
HI541WriteCmosSensor(0x151A, 0x2FFF); // 0x746c : sharp_sel,       0x746d : resol_ctl,	
HI541WriteCmosSensor(0x151C, 0x5010); // 0x746e : flat_sharpgainh, 0x746f : flat_sharpgainl,
HI541WriteCmosSensor(0x151E, 0x5010); // 0x7470 : skin_sharpgainh, 0x7471 : skin_sharpgainl,
HI541WriteCmosSensor(0x1520, 0x507F); // 0x7472 : byr_sp_line,	0x7488 : skin_cyan_ctl,	
HI541WriteCmosSensor(0x1522, 0x8091); // 0x7489 : flat_max_th,     0x7c04 : ci_lpf_ctl_01,	
HI541WriteCmosSensor(0x1524, 0xA43C); // 0x7c05 : ci_lpf_ctl_02,   0x7c06 : ci_lpf_ctl_03,	
HI541WriteCmosSensor(0x1526, 0x3830); // 0x7c0e : ci_lum_ofs_1,    0x7c0f : ci_lum_ofs_2,  	
HI541WriteCmosSensor(0x1528, 0x2A2A); // 0x7c10 : ci_lum_ofs_3,    0x7c11 : ci_lum_ofs_4,  	
HI541WriteCmosSensor(0x152A, 0x2420); // 0x7c12 : ci_lum_ofs_5,    0x7c13 : ci_lum_ofs_6, 	
HI541WriteCmosSensor(0x152C, 0x1A16); // 0x7c14 : ci_lum_ofs_7,    0x7c15 : ci_lum_ofs_8,  	
HI541WriteCmosSensor(0x152E, 0x63FC); // 0x7c16 : ci_ps_dy_ctl,    0x7c17 : ci_ps_max_th,	
HI541WriteCmosSensor(0x1530, 0x0210); // 0x7c18 : ci_ps_min_th,    0x7c19 : ci_ln_ps_gain_1,	
HI541WriteCmosSensor(0x1532, 0x121A); // 0x7c1a : ci_ln_ps_gain_2, 0x7c1b : ci_ln_ps_gain_3,	
HI541WriteCmosSensor(0x1534, 0x1D1E); // 0x7c1c : ci_ln_ps_gain_4, 0x7c1d : ci_ln_ps_gain_5,	
HI541WriteCmosSensor(0x1536, 0x1F10); // 0x7c1e : ci_ln_ps_gain_6, 0x7c1f : ci_dy_ps_gain_1,	
HI541WriteCmosSensor(0x1538, 0x1218); // 0x7c20 : ci_dy_ps_gain_2, 0x7c21 : ci_dy_ps_gain_3,	                
HI541WriteCmosSensor(0x153A, 0x1C1E); // 0x7c22 : ci_dy_ps_gain_4, 0x7c23 : ci_dy_ps_gain_5,	
HI541WriteCmosSensor(0x153C, 0x1F00); // 0x7c24 : ci_dy_ps_gain_6, 0x7c25 : ci_ps_type_ctl, 	
HI541WriteCmosSensor(0x153E, 0x2A32); // 0x7c26 : ci_ln_min_th,    0x7c27 : ci_dy_min_th,	
HI541WriteCmosSensor(0x1540, 0x2020); // 0x940c : ynr_lum_gain_1,  0x940d : ynr_lum_gain_2, 
HI541WriteCmosSensor(0x1542, 0x2020); // 0x940e : ynr_lum_gain_3,  0x940f : ynr_lum_gain_4,	
HI541WriteCmosSensor(0x1544, 0x2020); // 0x9410 : ynr_lum_gain_5,  0x9411 : ynr_lum_gain_6, 
HI541WriteCmosSensor(0x1546, 0x2020); // 0x9412 : ynr_lum_gain_7,  0x9413 : ynr_lum_gain_8,	
HI541WriteCmosSensor(0x1548, 0x0218); // 0x940a : ynr_line_rate,   0x940b : ynr_flat_ctl,	
HI541WriteCmosSensor(0x154A, 0x2562); // 0x9414 : ynr_flat_luml1,  0x9415 : ynr_flat_lumh1, 
HI541WriteCmosSensor(0x154C, 0x63FF); // 0x9416 : ynr_flat_luml2,	0x9417 : ynr_flat_lumh2,	
HI541WriteCmosSensor(0x154E, 0x0445); // 0x9418 : ynr_flat_stdl1,  0x9419 : ynr_flat_stdh1,	
HI541WriteCmosSensor(0x1550, 0x0565); // 0x941a : ynr_flat_stdl2,  0x941b : ynr_flat_stdh2,	
HI541WriteCmosSensor(0x1552, 0x0222); // 0x941c : ynr_flat_dyl1,   0x941d : ynr_flat_dyh1,	
HI541WriteCmosSensor(0x1554, 0x0630); // 0x941e : ynr_flat_dyl2,   0x941f : ynr_flat_dyh2,	
HI541WriteCmosSensor(0x1556, 0x080A); // 0x9426 : cnr_cf_cb_awm_th,0x9427 : cnr_cf_cr_awm_th,
HI541WriteCmosSensor(0x1558, 0xFA77); // 0x9448 : ac_mul_ctl12_cb, 0x9449 : ac_mul_ctl34_cb,	
HI541WriteCmosSensor(0x155A, 0xFDA7); // 0x944a : ac_mul_ctl12_cr, 0x944b : ac_mul_ctl34_cr,	
HI541WriteCmosSensor(0x155C, 0x5401); // 0x944e : cnr_lum_min_ctl, 0x9455 : cnr_dc_stdm12,	                        
HI541WriteCmosSensor(0x155E, 0x2344); // 0x9456 : cnr_dc_stdm34,   0x9457 : cnr_dc_stdm56,	
HI541WriteCmosSensor(0x1560, 0xB398); // 0x9800 : yee_ctl,         0x9801 : yee_index_gain,	                
HI541WriteCmosSensor(0x1562, 0x1003); // 0x9802 : yee_status_gain, 0x9803 : yee_lo_filter,	
HI541WriteCmosSensor(0x1564, 0x107B); // 0x9804 : yee_lo_gain,     0x9805 : yee_hi_filter,	
HI541WriteCmosSensor(0x1566, 0x1040); // 0x9806 : yee_hi_gain,     0x9807 : yee_flat_ctl,	
HI541WriteCmosSensor(0x1568, 0x0C0C); // 0x9808 : yee_skin_low_gain,    0x9809 : yee_skin_higgain,	
HI541WriteCmosSensor(0x156A, 0x1718); // 0x980a : yee_std_post_gain_pos,0x980b : yee_std_gain,	
HI541WriteCmosSensor(0x156C, 0x2424); // 0x980c : yee_lum_gain_p1, 0x980d : yee_lum_gain_p2,	
HI541WriteCmosSensor(0x156E, 0x2322); // 0x980e : yee_lum_gain_p3, 0x980f : yee_lum_gain_p4,  
HI541WriteCmosSensor(0x1570, 0x2121); // 0x9810 : yee_lum_gain_p5, 0x9811 : yee_lum_gain_p6, 
HI541WriteCmosSensor(0x1572, 0x1F1E); // 0x9812 : yee_lum_gain_p7, 0x9813 : yee_lum_gain_p8,
HI541WriteCmosSensor(0x1574, 0x2323); // 0x9814 : yee_lum_gain_n1, 0x9815 : yee_lum_gain_n2,	
HI541WriteCmosSensor(0x1576, 0x2222); // 0x9816 : yee_lum_gain_n3, 0x9817 : yee_lum_gain_n4,	
HI541WriteCmosSensor(0x1578, 0x2222); // 0x9818 : yee_lum_gain_n5, 0x9819 : yee_lum_gain_n6,	
HI541WriteCmosSensor(0x157A, 0x2120); // 0x981a : yee_lum_gain_n7, 0x981b : yee_lum_gain_n8,	
HI541WriteCmosSensor(0x157C, 0x2828); // 0x981c : yee_dy_gain_p1,  0x981d : yee_dy_gain_p2, //20140714 CKT	
HI541WriteCmosSensor(0x157E, 0x2020); // 0x981e : yee_dy_gain_p3,  0x981f : yee_dy_gain_p4, //20140714 CKT	
HI541WriteCmosSensor(0x1580, 0x2020); // 0x9820 : yee_dy_gain_p5,  0x9821 : yee_dy_gain_p6, //20140714 CKT	
HI541WriteCmosSensor(0x1582, 0x2828); // 0x9822 : yee_dy_gain_p7,  0x9823 : yee_dy_gain_p8, //20140714 CKT	
HI541WriteCmosSensor(0x1584, 0x2020); // 0x9824 : yee_dy_gain_n1,  0x9825 : yee_dy_gain_n2, //20140714 CKT	
HI541WriteCmosSensor(0x1586, 0x2020); // 0x9826 : yee_dy_gain_n3,  0x9827 : yee_dy_gain_n4, //20140714 CKT	
HI541WriteCmosSensor(0x1588, 0x2020); // 0x9828 : yee_dy_gain_n5,  0x9829 : yee_dy_gain_n6, //20140714 CKT	
HI541WriteCmosSensor(0x158A, 0x2020); // 0x982a : yee_dy_gain_n7,  0x982b : yee_dy_gain_n8, //20140714 CKT	
HI541WriteCmosSensor(0x158C, 0x2828); // 0x982c : yee_edge_gain_1, 0x982d : yee_edge_gain_2,	
HI541WriteCmosSensor(0x158E, 0x2828); // 0x982e : yee_edge_gain_3, 0x982f : yee_edge_gain_4,	
HI541WriteCmosSensor(0x1590, 0x2828); // 0x9830 : yee_edge_gain_5, 0x9831 : yee_edge_gain_6,	
HI541WriteCmosSensor(0x1592, 0x2020); // 0x9832 : yee_edge_gain_7, 0x9833 : yee_edge_gain_8,
HI541WriteCmosSensor(0x1594, 0x1213); // 0x983c : yee_std_gain_1,  0x983d : yee_std_gain_2,	
HI541WriteCmosSensor(0x1596, 0x1418); // 0x983e : yee_std_gain_3,  0x983f : yee_std_gain_4, 	
HI541WriteCmosSensor(0x1598, 0x1A1A); // 0x9840 : yee_std_gain_5,  0x9841 : yee_std_gain_6,	
HI541WriteCmosSensor(0x159A, 0x1713); // 0x9842 : yee_std_gain_7,  0x9843 : yee_std_gain_8,	
HI541WriteCmosSensor(0x159C, 0x1327); // 0x9865 : cnr_color_region_ctl, 0x9866 : cnr_color_sat_th, 
HI541WriteCmosSensor(0x159E, 0x3F35); // 0x7c2a : ci_nrlum_ofst_1, 0x7c2b : ci_nrlum_ofst_2,
HI541WriteCmosSensor(0x15A0, 0x2A27); // 0x7c2c : ci_nrlum_ofst_3, 0x7c2d : ci_nrlum_ofst_4,
HI541WriteCmosSensor(0x15A2, 0x2323); // 0x7c2e : ci_nrlum_ofst_5, 0x7c2f : ci_nrlum_ofst_6, 
HI541WriteCmosSensor(0x15A4, 0x2323); // 0x7c30 : ci_nrlum_ofst_7, 0x7c31 : ci_nrlum_ofst_8,
HI541WriteCmosSensor(0x15A6, 0xD011); // 0x7c3f : ci_highgain,     0x7c40 : ci_lowgain,	
HI541WriteCmosSensor(0x15A8, 0x0040); // 0x7c41 : ci_sp_pre_dif_ctl, 0x7c42 : ci_flt_luml1,	
HI541WriteCmosSensor(0x15AA, 0xFF0F); // 0x7c43 : ci_flt_lumh1,    0x7c44 : ci_flt_luml2,	
HI541WriteCmosSensor(0x15AC, 0x3F00); // 0x7c45 : ci_flt_lumh2,    0x7c46 : ci_flt_stdl1,	
HI541WriteCmosSensor(0x15AE, 0x4C00); // 0x7c47 : ci_flt_stdh1,    0x7c48 : ci_flt_stdl2,	
HI541WriteCmosSensor(0x15B0, 0x1300); // 0x7c49 : ci_flt_stdh2,    0x7c4a : ci_flt_dyl1,	
HI541WriteCmosSensor(0x15B2, 0x2A00); // 0x7c4b : ci_flt_dyh1,     0x7c4c : ci_flt_dyl2,	
HI541WriteCmosSensor(0x15B4, 0x172C); // 0x7c4d : ci_flt_dyh2,     0x7c4e : ci_flat_ctl,	
HI541WriteCmosSensor(0x15B6, 0x1616); // 0x7c50 : ci_splum_n1,     0x7c51 : ci_splum_n2,	
HI541WriteCmosSensor(0x15B8, 0x1514); // 0x7c52 : ci_splum_n3,     0x7c53 : ci_splum_n4,	
HI541WriteCmosSensor(0x15BA, 0x1313); // 0x7c54 : ci_splum_n5,     0x7c55 : ci_splum_n6,	
HI541WriteCmosSensor(0x15BC, 0x1313); // 0x7c56 : ci_splum_n7,     0x7c57 : ci_splum_n8,	
HI541WriteCmosSensor(0x15BE, 0x1616); // 0x7c58 : ci_splum_p1,     0x7c59 : ci_splum_p2,	
HI541WriteCmosSensor(0x15C0, 0x1513); // 0x7c5a : ci_splum_p3,     0x7c5b : ci_splum_p4,	
HI541WriteCmosSensor(0x15C2, 0x1212); // 0x7c5c : ci_splum_p5,     0x7c5d : ci_splum_p6,	
HI541WriteCmosSensor(0x15C4, 0x1212); // 0x7c5e : ci_splum_p7,     0x7c5f : ci_splum_p8,	
HI541WriteCmosSensor(0x15C6, 0x1414); // 0x7c60 : ci_spdy_n1,      0x7c61 : ci_spdy_n2,	
HI541WriteCmosSensor(0x15C8, 0x1516); // 0x7c62 : ci_spdy_n3,      0x7c63 : ci_spdy_n4,	
HI541WriteCmosSensor(0x15CA, 0x1615); // 0x7c64 : ci_spdy_n5,      0x7c65 : ci_spdy_n6,	
HI541WriteCmosSensor(0x15CC, 0x1414); // 0x7c66 : ci_spdy_n7,      0x7c67 : ci_spdy_n8,	
HI541WriteCmosSensor(0x15CE, 0x1414); // 0x7c68 : ci_spdy_p1,      0x7c69 : ci_spdy_p2,	
HI541WriteCmosSensor(0x15D0, 0x1415); // 0x7c6a : ci_spdy_p3,      0x7c6b : ci_spdy_p4,	
HI541WriteCmosSensor(0x15D2, 0x1513); // 0x7c6c : ci_spdy_p5,      0x7c6d : ci_spdy_p6,	                        
HI541WriteCmosSensor(0x15D4, 0x1212); // 0x7c6e : ci_spdy_p7,      0x7c6f : ci_spdy_p8,	
HI541WriteCmosSensor(0x15D6, 0x1B1B); // 0x7c70 : ci_spedge_1,     0x7c71 : ci_spedge_2,	
HI541WriteCmosSensor(0x15D8, 0x1C1D); // 0x7c72 : ci_spedge_3, 	0x7c73 : ci_spedge_4,	
HI541WriteCmosSensor(0x15DA, 0x1D1C); // 0x7c74 : ci_spedge_5,	0x7c75 : ci_spedge_6,	
HI541WriteCmosSensor(0x15DC, 0x1C1C); // 0x7c76 : ci_spedge_7,	0x7c77 : ci_spedge_8,	
HI541WriteCmosSensor(0x15DE, 0x1013); // 0x7c7d : ci_spstd_1,	0x7c7e : ci_spstd_2,	
HI541WriteCmosSensor(0x15E0, 0x1614); // 0x7c7f : ci_spstd_3,	0x7c80 : ci_spstd_4,	
HI541WriteCmosSensor(0x15E2, 0x1312); // 0x7c81 : ci_spstd_5,	0x7c82 : ci_spstd_6,	
HI541WriteCmosSensor(0x15E4, 0x1010); // 0x7c83 : ci_spstd_7,	0x7c84 : ci_spstd_8,	
HI541WriteCmosSensor(0x15E6, 0x3400); // 0x7c85 : ci_post_std_ctl, 0x7c97 : ci_skin_blue_ctl,
HI541WriteCmosSensor(0x15E8, 0x0001); // 0x9004 : sat_tra_sign_dk, 0x9005 : sat_tra_deltah_dk,
HI541WriteCmosSensor(0x15EA, 0x747c); // 0x9008 : sat_tra_delta1_dk,0x9009 : sat_tra_delta2_dk,
HI541WriteCmosSensor(0x15EC, 0x5805); // 0x900a : sat_tra_delta3_dk,0x900b : sat_tra_delta4_dk,
HI541WriteCmosSensor(0x15EE, 0x2933); // 0x900c : sat_tra_yb1_dk,	0x900e : sat_tra_yb2_dk_h,	
HI541WriteCmosSensor(0x15F0, 0x0036); // 0x900f : sat_tra_yb2_dk_l,0x9010 : sat_tra_yb3_dk_h,	
HI541WriteCmosSensor(0x15F2, 0xb03c); // 0x9011 : sat_tra_yb3_dk_l,0x9012 : sat_tra_yb4_dk_h,	
HI541WriteCmosSensor(0x15F4, 0x0007); // 0x9013 : sat_tra_yb4_dk_l,0x9014 : sat_tra_th1_dk,	
HI541WriteCmosSensor(0x15F6, 0x0f1e); // 0x9015 : sat_tra_th2_dk,	0x9016 : sat_tra_th3_dk,

HI541WriteCmosSensor(0x166C, 0x0D00); // 0x9ca4 : mcmc_allgain_y11, 0x7c3c : ci_gain_std_ofst, //seperating MCMC Adaptive register

HI541WriteCmosSensor(0x166E, 0x871E); // 0x7c3d : ci_spflt_std_ctl,0x7400 : lensd_lpf_ctl1,	
HI541WriteCmosSensor(0x1670, 0x7700); // 0x7401 : lpf_flt_sel,	0x7404 : imp_ctl,		
HI541WriteCmosSensor(0x1672, 0x8104); // 0x7405 : nr_lpf_sel_01,	0x7406 : nr_lpf_sel_02,	
HI541WriteCmosSensor(0x1674, 0x5830); // 0x7407 : nr_lpf_std_01,	0x7408 : nr_lpf_std_02,	
HI541WriteCmosSensor(0x1676, 0x1212); // 0x7409 : nr_lpf_std_03,	0x740a : org_std_ctl,	
HI541WriteCmosSensor(0x1678, 0xC020); // 0x740b : org_std_oft,	0x743c : nr_rate,			
HI541WriteCmosSensor(0x167A, 0x189F); // 0x743d : line_rate, 	0x7464 : byr_sp_onoff,	
HI541WriteCmosSensor(0x167C, 0xF101); // 0x7467 : sf_sel_std_sel, 	0x7c28 : ci_cnr_ctl,	
HI541WriteCmosSensor(0x167E, 0x15E7); // 0x7c29 : ci_y_nr_ctl,	0x7c32 : ci_nrgain_std_ctl,
HI541WriteCmosSensor(0x1680, 0x0000); // 0x7c33 : ci_nrgain_std_ofst,0x7c34 : ci_cbcr_gain,	
HI541WriteCmosSensor(0x1682, 0x0295); // 0x7c35 : ci_dy_gain, 	0x7c36 : ci_filt_sel_ctl,	
HI541WriteCmosSensor(0x1684, 0x4020); // 0x7c37 : ci_yfilt_sel_h,  0x7c38 : ci_yfilt_sel_m,	
HI541WriteCmosSensor(0x1686, 0x109F); // 0x7c39 : ci_yfilt_sel_l,  0x7c3b : ci_spstd_ctl,	
HI541WriteCmosSensor(0x1688, 0x100D); // 0x7c4f : ci_sp_ln_ctl, 	0x7caa : ci_otp1,		
HI541WriteCmosSensor(0x168A, 0x0F2C); // 0x7cac : ci_weak_hv_gain, 0x7cad : ci_moire_th1,	
HI541WriteCmosSensor(0x168C, 0x1307); // 0x7cae : ci_moire_th2, 	0x8000 : cmc_ctl1,		
HI541WriteCmosSensor(0x168E, 0x3400); // 0x8002 : cmc_ps_ln_cmc_gain,0x8800 : gma_ctl1,		
HI541WriteCmosSensor(0x1690, 0x430E); // 0x8802 : gma_ps_ln_gain, 	0x9401 : ynr_ctl2,		
HI541WriteCmosSensor(0x1692, 0xDA25); // 0x9402 : ynr_ctl3, 	0x9403 : ynr_ctl4,		
HI541WriteCmosSensor(0x1694, 0x3060); // 0x9404 : ynr_std_l_th, 	0x9405 : ynr_std_m_th,	
HI541WriteCmosSensor(0x1696, 0xA03F); // 0x9406 : ynr_std_h_th, 	0x7c86 : ci_splum_max_n1,	
HI541WriteCmosSensor(0x1698, 0x3F3F); // 0x7c87 : ci_splum_max_n2,	0x7c88 : ci_splum_max_n3,	
HI541WriteCmosSensor(0x169A, 0x3F3F); // 0x7c89 : ci_splum_max_n4,	0x7c8a : ci_splum_max_n5,	
HI541WriteCmosSensor(0x169C, 0x3F3F); // 0x7c8b : ci_splum_max_p1,	0x7c8c : ci_splum_max_p2,	
HI541WriteCmosSensor(0x169E, 0x3F3F); // 0x7c8d : ci_splum_max_p3,	0x7c8e : ci_splum_max_p4,	
HI541WriteCmosSensor(0x16A0, 0x3F13); // 0x7c8f : ci_splum_max_p5, 0x7ca5 : ci_dy_sel_ctl,	
HI541WriteCmosSensor(0x16A2, 0x1300); // 0x7ca6 : ci_lum_sel_ctl

//HI541WriteCmosSensor(0x1a90, 0x1320); // INDOOR 0x7c9a : bi_std_th // 0x1a91 // 20140520 for indoor diag noise (patch rev)
HI541WriteCmosSensor(0x1afa, 0x0030); // INDOOR 0x7c9b : bi_dy_th  // 0x1afb // 20140628 for indoor diag noise (patch rev)

// Dark1
HI541WriteCmosSensor(0x16A2, 0x0000); // 				0x9000 : Saturation ENB Transfer Function	
HI541WriteCmosSensor(0x16A4, 0x1F00); // 0x7034 : pre_flat_ctl, 	0x7035 : flat_max_luml1
HI541WriteCmosSensor(0x16A6, 0xFF00); // 0x7036 : flat_max_lumh1, 	0x7037 : flat_max_luml2
HI541WriteCmosSensor(0x16A8, 0x0000); // 0x7038 : flat_max_lumh2, 	0x7039 : flat_max_dyl1
HI541WriteCmosSensor(0x16AA, 0x5300); // 0x703a : flat_max_dyh1,	0x703b : flat_max_dyl2
HI541WriteCmosSensor(0x16AC, 0x0000); // 0x703c : flat_max_dyh2, 	0x703d : flat_max_stdl1
HI541WriteCmosSensor(0x16AE, 0x2C00); // 0x703e : flat_max_stdh1,  	0x703f : flat_max_stdl2
HI541WriteCmosSensor(0x16B0, 0x003F); // 0x7040 : flat_max_stdh2, 	0x742c : nr_lum_ofs_1
HI541WriteCmosSensor(0x16B2, 0x3F3F); // 0x742d : nr_lum_ofs_2,	0x742e : nr_lum_ofs_3
HI541WriteCmosSensor(0x16B4, 0x3D3C); // 0x742f : nr_lum_ofs_4,	0x7430 : nr_lum_ofs_5
HI541WriteCmosSensor(0x16B6, 0x3836); // 0x7431 : nr_lum_ofs_6,	0x7432 : nr_lum_ofs_7
HI541WriteCmosSensor(0x16B8, 0x343F); // 0x7433 : nr_lum_ofs_8,	0x7434 : nr_imp_th_1
HI541WriteCmosSensor(0x16BA, 0x3F3E); // 0x7435 : nr_imp_th_2,	0x7436 : nr_imp_th_3
HI541WriteCmosSensor(0x16BC, 0x3833); // 0x7437 : nr_imp_th_4,	0x7438 : nr_imp_th_5
HI541WriteCmosSensor(0x16BE, 0x3130); // 0x7439 : nr_imp_th_6, 	0x743a : nr_imp_th_7
HI541WriteCmosSensor(0x16C0, 0x3020); // error 0x16c1 0x743b : nr_imp_th_8,	0x743c : nr_rate
HI541WriteCmosSensor(0x16C2, 0x1812); // error 0x16c2 0x743d : line_rate,	0x7800 : splum_gain_pos_1
HI541WriteCmosSensor(0x16C4, 0x0F0E); // 0x7801 : splum_gain_pos_2, 0x7802 : splum_gain_pos_3
HI541WriteCmosSensor(0x16C6, 0x0E0E); // 0x7803 : splum_gain_pos_4, 0x7804 : splum_gain_pos_5
HI541WriteCmosSensor(0x16C8, 0x1012); // 0x7805 : splum_gain_pos_6, 0x7806 : splum_gain_pos_7
HI541WriteCmosSensor(0x16CA, 0x1212); // 0x7807 : splum_gain_pos_8, 0x7808 : splum_gain_neg_1
HI541WriteCmosSensor(0x16CC, 0x0F0F); // 0x7809 : splum_gain_neg_2, 0x780a : splum_gain_neg_3
HI541WriteCmosSensor(0x16CE, 0x1212); // 0x780b : splum_gain_neg_4, 0x780c : splum_gain_neg_5
HI541WriteCmosSensor(0x16D0, 0x1212); // 0x780d : splum_gain_neg_6, 0x780e : splum_gain_neg_7
HI541WriteCmosSensor(0x16D2, 0x120F); // 0x780f : splum_gain_neg_8, 0x7810 : spdy_gain_pos_1
HI541WriteCmosSensor(0x16D4, 0x0F12); // 0x7811 : spdy_gain_pos_2,	0x7812 : spdy_gain_pos_3
HI541WriteCmosSensor(0x16D6, 0x1212); // 0x7813 : spdy_gain_pos_4,	0x7814 : spdy_gain_pos_5
HI541WriteCmosSensor(0x16D8, 0x0404); // 0x7815 : spdy_gain_pos_6,	0x7816 : spdy_gain_pos_7 //20140627
HI541WriteCmosSensor(0x16DA, 0x040F); // 0x7817 : spdy_gain_pos_8,	0x7818 : spdy_gain_neg_1 //20140627
HI541WriteCmosSensor(0x16DC, 0x0F12); // 0x7819 : spdy_gain_neg_2,	0x781a : spdy_gain_neg_3
HI541WriteCmosSensor(0x16DE, 0x1212); // 0x781b : spdy_gain_neg_4,	0x781c : spdy_gain_neg_5
HI541WriteCmosSensor(0x16E0, 0x0404); // 0x781d : spdy_gain_neg_6,	0x781e : spdy_gain_neg_7 //20140627 
HI541WriteCmosSensor(0x16E2, 0x041C); // 0x781f : spdy_gain_neg_8,	0x7820 : spedge_gain_1   //20140627
HI541WriteCmosSensor(0x16E4, 0x1818); // 0x7821 : spedge_gain_2,	0x7822 : spedge_gain_3
HI541WriteCmosSensor(0x16E6, 0x181D); // 0x7823 : spedge_gain_4, 	0x7824 : spedge_gain_5
HI541WriteCmosSensor(0x16E8, 0x2020); // 0x7825 : spedge_gain_6,	0x7826 : spedge_gain_7
HI541WriteCmosSensor(0x16EA, 0x2012); // 0x7827 : spedge_gain_8,	0x7830 : spstd_gain_1
HI541WriteCmosSensor(0x16EC, 0x1213); // 0x7831 : spstd_gain_2,	0x7832 : spstd_gain_3
HI541WriteCmosSensor(0x16EE, 0x1818); // 0x7833 : spstd_gain_4,	0x7834 : spstd_gain_5
HI541WriteCmosSensor(0x16F0, 0x1818); // 0x7835 : spstd_gain_6, 	0x7836 : spstd_gain_7
HI541WriteCmosSensor(0x16F2, 0x1898); // 0x7837 : spstd_gain_8,     0x7838 : post_std_sel	
HI541WriteCmosSensor(0x16F4, 0x00FF); // 0x7440 : flt_luml1,	0x7441 : flt_lumh1	
HI541WriteCmosSensor(0x16F6, 0x00FF); // 0x7442 : flt_luml2,	0x7443 : flt_lumh2	
HI541WriteCmosSensor(0x16F8, 0x0040); // 0x7444 : flt_stdl1,	0x7445 : flt_stdh1	
HI541WriteCmosSensor(0x16FA, 0x002A); // 0x7446 : flt_stdl2,	0x7447 : flt_stdh2 	
HI541WriteCmosSensor(0x16FC, 0x0027); // 0x7448 : flt_dyl1,		0x7449 : flt_dyh1		
HI541WriteCmosSensor(0x16FE, 0x0020); // 0x744a : flt_dyl2,		0x744b : flt_dyh2		
HI541WriteCmosSensor(0x1700, 0xFF32); // 0x7458 : flt_rate1,	0x744c : flt_lumll1	
HI541WriteCmosSensor(0x1702, 0x7A00); // 0x744d : flt_lumhh1,	0x744e : flt_lumll2 	
HI541WriteCmosSensor(0x1704, 0x0001); // 0x744f : flt_lumhh2,	0x7450 : flt_stdll1	
HI541WriteCmosSensor(0x1706, 0x2100); // 0x7451 : flt_stdhh1,	0x7452 : flt_stdll2 	
HI541WriteCmosSensor(0x1708, 0x0001); // 0x7453 : flt_stdhh2,	0x7454 : flt_dyll1
HI541WriteCmosSensor(0x170A, 0x1D00); // 0x7455 : flt_dyhh1,	0x7456 : flt_dyll2 	
HI541WriteCmosSensor(0x170C, 0x00FF); // 0x7457 : flt_dyhh2, 	0x7459 : flt_rate2
HI541WriteCmosSensor(0x170E, 0x1A16); // 0x7465 : highgain,		0x7466 : lowgain
HI541WriteCmosSensor(0x1710, 0x362F); // 0x746b : cent_flt_sel,	0x746c : sharp_sel
HI541WriteCmosSensor(0x1712, 0xFF50); // 0x746d : resol_ctl,	0x746e : flat_sharpgainh
HI541WriteCmosSensor(0x1714, 0x104C); // 0x746f : flat_sharpgainl,  0x7470 : skin_sharpgainh //20140702 for skin
HI541WriteCmosSensor(0x1716, 0x2450); // 0x7471 : skin_sharpgainl, 	0x7472 : byr_sp_line	
HI541WriteCmosSensor(0x1718, 0x7F00); // 0x7488 : skin_cyan_ctl,	0x7489 : flat_max_th
HI541WriteCmosSensor(0x171A, 0x91A4); // 0x7c04 : ci_lpf_ctl_01,	0x7c05 : ci_lpf_ctl_02
HI541WriteCmosSensor(0x171C, 0x3C38); // 0x7c06 : ci_lpf_ctl_03,	0x7c0e : ci_lum_ofs_1
HI541WriteCmosSensor(0x171E, 0x302A); // 0x7c0f : ci_lum_ofs_2,  	0x7c10 : ci_lum_ofs_3
HI541WriteCmosSensor(0x1720, 0x2A24); // 0x7c11 : ci_lum_ofs_4,  	0x7c12 : ci_lum_ofs_5
HI541WriteCmosSensor(0x1722, 0x201A); // 0x7c13 : ci_lum_ofs_6, 	0x7c14 : ci_lum_ofs_7
HI541WriteCmosSensor(0x1724, 0x1662); // 0x7c15 : ci_lum_ofs_8,  	0x7c16 : ci_ps_dy_ctl //20140627 for ps off
HI541WriteCmosSensor(0x1726, 0xFC02); // 0x7c17 : ci_ps_max_th,	0x7c18 : ci_ps_min_th
HI541WriteCmosSensor(0x1728, 0x1012); // 0x7c19 : ci_ln_ps_gain_1,	0x7c1a : ci_ln_ps_gain_2
HI541WriteCmosSensor(0x172A, 0x1A1D); // 0x7c1b : ci_ln_ps_gain_3,	0x7c1c : ci_ln_ps_gain_4
HI541WriteCmosSensor(0x172C, 0x1E1F); // 0x7c1d : ci_ln_ps_gain_5,	0x7c1e : ci_ln_ps_gain_6
HI541WriteCmosSensor(0x172E, 0x1012); // 0x7c1f : ci_dy_ps_gain_1,	0x7c20 : ci_dy_ps_gain_2
HI541WriteCmosSensor(0x1730, 0x181C); // 0x7c21 : ci_dy_ps_gain_3,	0x7c22 : ci_dy_ps_gain_4
HI541WriteCmosSensor(0x1732, 0x1E1F); // 0x7c23 : ci_dy_ps_gain_5,	0x7c24 : ci_dy_ps_gain_6
HI541WriteCmosSensor(0x1734, 0x002A); // 0x7c25 : ci_ps_type_ctl, 	0x7c26 : ci_ln_min_th
HI541WriteCmosSensor(0x1736, 0x3220); // 0x7c27 : ci_dy_min_th,	0x940c : ynr_lum_gain_1
HI541WriteCmosSensor(0x1738, 0x2020); // 0x940d : ynr_lum_gain_2, 	0x940e : ynr_lum_gain_3 
HI541WriteCmosSensor(0x173A, 0x2020); // 0x940f : ynr_lum_gain_4,	0x9410 : ynr_lum_gain_5
HI541WriteCmosSensor(0x173C, 0x2020); // 0x9411 : ynr_lum_gain_6, 	0x9412 : ynr_lum_gain_7
HI541WriteCmosSensor(0x173E, 0x2002); // 0x9413 : ynr_lum_gain_8,	0x940a : ynr_line_rate
HI541WriteCmosSensor(0x1740, 0x1836); // 0x940b : ynr_flat_ctl,	0x9414 : ynr_flat_luml1
HI541WriteCmosSensor(0x1742, 0x6263); // 0x9415 : ynr_flat_lumh1,  	0x9416 : ynr_flat_luml2 
HI541WriteCmosSensor(0x1744, 0xFF04); // 0x9417 : ynr_flat_lumh2,	0x9418 : ynr_flat_stdl1
HI541WriteCmosSensor(0x1746, 0x4505); // 0x9419 : ynr_flat_stdh1,	0x941a : ynr_flat_stdl2
HI541WriteCmosSensor(0x1748, 0x6502); // 0x941b : ynr_flat_stdh2,	0x941c : ynr_flat_dyl1
HI541WriteCmosSensor(0x174A, 0x2206); // 0x941d : ynr_flat_dyh1,	0x941e : ynr_flat_dyl2
HI541WriteCmosSensor(0x174C, 0x3008); // 0x941f : ynr_flat_dyh2,	0x9426 : cnr_cf_cb_awm_th
HI541WriteCmosSensor(0x174E, 0x0AFA); // 0x9427 : cnr_cf_cr_awm_th,	0x9448 : ac_mul_ctl12_cb
HI541WriteCmosSensor(0x1750, 0x77FE); // 0x9449 : ac_mul_ctl34_cb,	0x944a : ac_mul_ctl12_cr
HI541WriteCmosSensor(0x1752, 0xCA78); // 0x944b : ac_mul_ctl34_cr,	0x944e : cnr_lum_min_ctl
HI541WriteCmosSensor(0x1754, 0x0122); // 0x9455 : cnr_dc_stdm12,	0x9456 : cnr_dc_stdm34
HI541WriteCmosSensor(0x1756, 0x33B3); // 0x9457 : cnr_dc_stdm56,	0x9800 : yee_ctl
HI541WriteCmosSensor(0x1758, 0x9810); // 0x9801 : yee_index_gain,	0x9802 : yee_status_gain
HI541WriteCmosSensor(0x175A, 0x0323); // 0x9803 : yee_lo_filter,	0x9804 : yee_lo_gain
HI541WriteCmosSensor(0x175C, 0x7B10); // 0x9805 : yee_hi_filter,	0x9806 : yee_hi_gain
HI541WriteCmosSensor(0x175E, 0x400C); // 0x9807 : yee_flat_ctl,	0x9808 : yee_skin_low_gain
HI541WriteCmosSensor(0x1760, 0x0C12); // 0x9809 : yee_skin_higgain,	0x980a : yee_std_post_gain_pos
HI541WriteCmosSensor(0x1762, 0x1214); // 0x980b : yee_std_gain,	0x980c : yee_lum_gain_p1
HI541WriteCmosSensor(0x1764, 0x1414); // 0x980d : yee_lum_gain_p2,	0x980e : yee_lum_gain_p3
HI541WriteCmosSensor(0x1766, 0x1414); // 0x980f : yee_lum_gain_p4,  0x9810 : yee_lum_gain_p5 
HI541WriteCmosSensor(0x1768, 0x1414); // 0x9811 : yee_lum_gain_p6,  0x9812 : yee_lum_gain_p7
HI541WriteCmosSensor(0x176A, 0x1412); // 0x9813 : yee_lum_gain_p8,  0x9814 : yee_lum_gain_n1
HI541WriteCmosSensor(0x176C, 0x1212); // 0x9815 : yee_lum_gain_n2,	0x9816 : yee_lum_gain_n3
HI541WriteCmosSensor(0x176E, 0x1109); // 0x9817 : yee_lum_gain_n4,	0x9818 : yee_lum_gain_n5
HI541WriteCmosSensor(0x1770, 0x0810); // 0x9819 : yee_lum_gain_n6,	0x981a : yee_lum_gain_n7
HI541WriteCmosSensor(0x1772, 0x1214); // 0x981b : yee_lum_gain_n8,	0x981c : yee_dy_gain_p1
HI541WriteCmosSensor(0x1774, 0x1414); // 0x981d : yee_dy_gain_p2,	0x981e : yee_dy_gain_p3
HI541WriteCmosSensor(0x1776, 0x1414); // 0x981f : yee_dy_gain_p4,	0x9820 : yee_dy_gain_p5
HI541WriteCmosSensor(0x1778, 0x0606); // 0x9821 : yee_dy_gain_p6,	0x9822 : yee_dy_gain_p7 //20140627
HI541WriteCmosSensor(0x177A, 0x0612); // 0x9823 : yee_dy_gain_p8,	0x9824 : yee_dy_gain_n1 //20140627
HI541WriteCmosSensor(0x177C, 0x1212); // 0x9825 : yee_dy_gain_n2,	0x9826 : yee_dy_gain_n3
HI541WriteCmosSensor(0x177E, 0x1212); // 0x9827 : yee_dy_gain_n4,	0x9828 : yee_dy_gain_n5
HI541WriteCmosSensor(0x1780, 0x0404); // 0x9829 : yee_dy_gain_n6,	0x982a : yee_dy_gain_n7	//20140627
HI541WriteCmosSensor(0x1782, 0x0420); // 0x982b : yee_dy_gain_n8,	0x982c : yee_edge_gain_1//20140627
HI541WriteCmosSensor(0x1784, 0x2020); // 0x982d : yee_edge_gain_2,	0x982e : yee_edge_gain_3
HI541WriteCmosSensor(0x1786, 0x2020); // 0x982f : yee_edge_gain_4,	0x9830 : yee_edge_gain_5
HI541WriteCmosSensor(0x1788, 0x2020); // 0x9831 : yee_edge_gain_6,	0x9832 : yee_edge_gain_7
HI541WriteCmosSensor(0x178A, 0x202A); // 0x9833 : yee_edge_gain_8,  0x983c : yee_std_gain_1
HI541WriteCmosSensor(0x178C, 0x2A2A); // 0x983d : yee_std_gain_2,	0x983e : yee_std_gain_3
HI541WriteCmosSensor(0x178E, 0x2A2A); // 0x983f : yee_std_gain_4, 	0x9840 : yee_std_gain_5
HI541WriteCmosSensor(0x1790, 0x1A1A); // 0x9841 : yee_std_gain_6,	0x9842 : yee_std_gain_7
HI541WriteCmosSensor(0x1792, 0x1A13); // 0x9843 : yee_std_gain_8,	0x9865 : cnr_color_region_ctl
HI541WriteCmosSensor(0x1794, 0x273F); // 0x9866 : cnr_color_sat_th, 0x7c2a : ci_nrlum_ofst_1
HI541WriteCmosSensor(0x1796, 0x3F2A); // 0x7c2b : ci_nrlum_ofst_2, 	0x7c2c : ci_nrlum_ofst_3
HI541WriteCmosSensor(0x1798, 0x2723); // 0x7c2d : ci_nrlum_ofst_4, 	0x7c2e : ci_nrlum_ofst_5
HI541WriteCmosSensor(0x179A, 0x2323); // 0x7c2f : ci_nrlum_ofst_6, 	0x7c30 : ci_nrlum_ofst_7
HI541WriteCmosSensor(0x179C, 0x23D0); // 0x7c31 : ci_nrlum_ofst_8,	0x7c3f : ci_highgain
HI541WriteCmosSensor(0x179E, 0x1100); // 0x7c40 : ci_lowgain,	0x7c41 : ci_sp_pre_dif_ctl
HI541WriteCmosSensor(0x17A0, 0x40FF); // 0x7c42 : ci_flt_luml1,	0x7c43 : ci_flt_lumh1	
HI541WriteCmosSensor(0x17A2, 0x113F); // 0x7c44 : ci_flt_luml2,	0x7c45 : ci_flt_lumh2
HI541WriteCmosSensor(0x17A4, 0x004C); // 0x7c46 : ci_flt_stdl1,	0x7c47 : ci_flt_stdh1
HI541WriteCmosSensor(0x17A6, 0x0013); // 0x7c48 : ci_flt_stdl2,	0x7c49 : ci_flt_stdh2
HI541WriteCmosSensor(0x17A8, 0x002A); // 0x7c4a : ci_flt_dyl1,	0x7c4b : ci_flt_dyh1
HI541WriteCmosSensor(0x17AA, 0x0017); // 0x7c4c : ci_flt_dyl2,	0x7c4d : ci_flt_dyh2
HI541WriteCmosSensor(0x17AC, 0x2C08); // 0x7c4e : ci_flat_ctl,	0x7c50 : ci_splum_n1
HI541WriteCmosSensor(0x17AE, 0x0A10); // 0x7c51 : ci_splum_n2,	0x7c52 : ci_splum_n3
HI541WriteCmosSensor(0x17B0, 0x1212); // 0x7c53 : ci_splum_n4,	0x7c54 : ci_splum_n5
HI541WriteCmosSensor(0x17B2, 0x1212); // 0x7c55 : ci_splum_n6,	0x7c56 : ci_splum_n7
HI541WriteCmosSensor(0x17B4, 0x1212); // 0x7c57 : ci_splum_n8,	0x7c58 : ci_splum_p1
HI541WriteCmosSensor(0x17B6, 0x1212); // 0x7c59 : ci_splum_p2,	0x7c5a : ci_splum_p3
HI541WriteCmosSensor(0x17B8, 0x1212); // 0x7c5b : ci_splum_p4,	0x7c5c : ci_splum_p5
HI541WriteCmosSensor(0x17BA, 0x1212); // 0x7c5d : ci_splum_p6,	0x7c5e : ci_splum_p7
HI541WriteCmosSensor(0x17BC, 0x1212); // 0x7c5f : ci_splum_p8,	0x7c60 : ci_spdy_n1
HI541WriteCmosSensor(0x17BE, 0x1212); // 0x7c61 : ci_spdy_n2,	0x7c62 : ci_spdy_n3
HI541WriteCmosSensor(0x17C0, 0x1212); // 0x7c63 : ci_spdy_n4,	0x7c64 : ci_spdy_n5
HI541WriteCmosSensor(0x17C2, 0x0404); // 0x7c65 : ci_spdy_n6,	0x7c66 : ci_spdy_n7
HI541WriteCmosSensor(0x17C4, 0x0412); // 0x7c67 : ci_spdy_n8,	0x7c68 : ci_spdy_p1
HI541WriteCmosSensor(0x17C6, 0x1212); // 0x7c69 : ci_spdy_p2,	0x7c6a : ci_spdy_p3
HI541WriteCmosSensor(0x17C8, 0x1212); // 0x7c6b : ci_spdy_p4,	0x7c6c : ci_spdy_p5
HI541WriteCmosSensor(0x17CA, 0x0404); // 0x7c6d : ci_spdy_p6,	0x7c6e : ci_spdy_p7	 //20140627
HI541WriteCmosSensor(0x17CC, 0x0420); // 0x7c6f : ci_spdy_p8,	0x7c70 : ci_spedge_1 //20140627
HI541WriteCmosSensor(0x17D0, 0x2020); // 0x7c73 : ci_spedge_4,	0x7c74 : ci_spedge_5
HI541WriteCmosSensor(0x17D2, 0x2020); // 0x7c75 : ci_spedge_6,	0x7c76 : ci_spedge_7
HI541WriteCmosSensor(0x17D4, 0x201A); // 0x7c77 : ci_spedge_8,	0x7c7d : ci_spstd_1
HI541WriteCmosSensor(0x17D6, 0x1A1A); // 0x7c7e : ci_spstd_2,	0x7c7f : ci_spstd_3
HI541WriteCmosSensor(0x17D8, 0x1A1A); // 0x7c80 : ci_spstd_4,	0x7c81 : ci_spstd_5
HI541WriteCmosSensor(0x17DA, 0x1A1A); // 0x7c82 : ci_spstd_6,	0x7c83 : ci_spstd_7
HI541WriteCmosSensor(0x17DC, 0x1A36); // 0x7c84 : ci_spstd_8,	0x7c85 : ci_post_std_ctl
HI541WriteCmosSensor(0x17DE, 0x0000); // 0x7c97 : ci_skin_blue_ctl,	0x9004 : sat_tra_sign_dk
HI541WriteCmosSensor(0x17E0, 0x0120); // 0x9005 : sat_tra_deltah_dk,0x9008 : sat_tra_delta1_dk
HI541WriteCmosSensor(0x17E2, 0xffe0); // 0x9009 : sat_tra_delta2_dk,0x900a : sat_tra_delta3_dk
HI541WriteCmosSensor(0x17E4, 0x0a1a); // 0x900b : sat_tra_delta4_dk,0x900c : sat_tra_yb1_dk
HI541WriteCmosSensor(0x17E6, 0x21e0); // 0x900e : sat_tra_yb2_dk_h,	0x900f : sat_tra_yb2_dk_l
HI541WriteCmosSensor(0x17E8, 0x2aa0); // 0x9010 : sat_tra_yb3_dk_h,	0x9011 : sat_tra_yb3_dk_l
HI541WriteCmosSensor(0x17EA, 0x37b0); // 0x9012 : sat_tra_yb4_dk_h,	0x9013 : sat_tra_yb4_dk_l
HI541WriteCmosSensor(0x17EC, 0x070F); // 0x9014 : sat_tra_th1_dk,	0x9015 : sat_tra_th2_dk
HI541WriteCmosSensor(0x17EE, 0x1E17); // 0x9016 : sat_tra_th3_dk,	0x9c02 : mcmc_ang_dlta1 //seperating MCMC Adaptive register

HI541WriteCmosSensor(0x1864, 0x0087); // 0x7c3c : ci_gain_std_ofst,	0x7c3d : ci_spflt_std_ctl
HI541WriteCmosSensor(0x1866, 0x1E77); // 0x7400 : lensd_lpf_ctl1,	0x7401 : lpf_flt_sel
HI541WriteCmosSensor(0x1868, 0x0081); // 0x7404 : imp_ctl,		0x7405 : nr_lpf_sel_01
HI541WriteCmosSensor(0x186A, 0x0458); // 0x7406 : nr_lpf_sel_02,	0x7407 : nr_lpf_std_01
HI541WriteCmosSensor(0x186C, 0x3012); // 0x7408 : nr_lpf_std_02,	0x7409 : nr_lpf_std_03 
HI541WriteCmosSensor(0x186E, 0x12C0); // 0x740a : org_std_ctl,	0x740b : org_std_oft
HI541WriteCmosSensor(0x1870, 0x2018); // 0x743c : nr_rate,		0x743d : line_rate 
HI541WriteCmosSensor(0x1872, 0x9FF1); // 0x7464 : byr_sp_onoff,	0x7467 : sf_sel_std_sel 
HI541WriteCmosSensor(0x1874, 0x0115); // 0x7c28 : ci_cnr_ctl,	0x7c29 : ci_y_nr_ctl
HI541WriteCmosSensor(0x1876, 0xE700); // 0x7c32 : ci_nrgain_std_ctl,0x7c33 : ci_nrgain_std_ofst
HI541WriteCmosSensor(0x1878, 0x0002); // 0x7c34 : ci_cbcr_gain,	0x7c35 : ci_dy_gain 
HI541WriteCmosSensor(0x187A, 0x9540); // 0x7c36 : ci_filt_sel_ctl,	0x7c37 : ci_yfilt_sel_h 
HI541WriteCmosSensor(0x187C, 0x2010); // 0x7c38 : ci_yfilt_sel_m,	0x7c39 : ci_yfilt_sel_l 
HI541WriteCmosSensor(0x187E, 0x9F10); // 0x7c3b : ci_spstd_ctl,	0x7c4f : ci_sp_ln_ctl 
HI541WriteCmosSensor(0x1880, 0x0D0F); // 0x7caa : ci_otp1,		0x7cac : ci_weak_hv_gain 
HI541WriteCmosSensor(0x1882, 0x2C13); // 0x7cad : ci_moire_th1,	0x7cae : ci_moire_th2 
HI541WriteCmosSensor(0x1884, 0x0700); // 0x8000 : cmc_ctl1,		0x8002 : cmc_ps_ln_cmc_gain //20140627 for ps
HI541WriteCmosSensor(0x1886, 0x0000); // 0x8800 : gma_ctl1,		0x8802 : gma_ps_ln_gain //20140627 for ps
HI541WriteCmosSensor(0x1888, 0x0EDA); // 0x9401 : ynr_ctl2,		0x9402 : ynr_ctl3 
HI541WriteCmosSensor(0x188A, 0x2530); // 0x9403 : ynr_ctl4,		0x9404 : ynr_std_l_th 
HI541WriteCmosSensor(0x188C, 0x60A0); // 0x9405 : ynr_std_m_th,	0x9406 : ynr_std_h_th 
HI541WriteCmosSensor(0x188E, 0x3F3F); // 0x7c86 : ci_splum_max_n1,	0x7c87 : ci_splum_max_n2
HI541WriteCmosSensor(0x1890, 0x3F3F); // 0x7c88 : ci_splum_max_n3,	0x7c89 : ci_splum_max_n4
HI541WriteCmosSensor(0x1892, 0x3F3F); // 0x7c8a : ci_splum_max_n5,	0x7c8b : ci_splum_max_p1
HI541WriteCmosSensor(0x1894, 0x3F3F); // 0x7c8c : ci_splum_max_p2,	0x7c8d : ci_splum_max_p3
HI541WriteCmosSensor(0x1896, 0x3F3F); // 0x7c8e : ci_splum_max_p4,	0x7c8f : ci_splum_max_p5
HI541WriteCmosSensor(0x1898, 0x1313); // 0x7ca5 : ci_dy_sel_ctl,	0x7ca6 : ci_lum_sel_ctl

HI541WriteCmosSensor(0x1292, 0x3001); //DARK1 0x7c9a : bi_std_th // 0x1292 // 20140520 for diag noise (patch rev)
HI541WriteCmosSensor(0x12b0, 0x0030); //DARK1 0x7c9b : bi_dy_th  // 0x12b1 // 20140520 for diag noise (patch rev)

// Dark2
HI541WriteCmosSensor(0x189A, 0x001F);  // 0x9000 : Saturation ENB Transfer Function , 0x7034 : pre_flat_ctl 	
HI541WriteCmosSensor(0x189C, 0x00FF);  // 0x7035 : flat_max_luml1, 	0x7036 : flat_max_lumh1,
HI541WriteCmosSensor(0x189E, 0x0000);  // 0x7037 : flat_max_luml2, 	0x7038 : flat_max_lumh2, 	
HI541WriteCmosSensor(0x18A0, 0x0053);  // 0x7039 : flat_max_dyl1,	0x703a : flat_max_dyh1,	
HI541WriteCmosSensor(0x18A2, 0x0000);  // 0x703b : flat_max_dyl2,	0x703c : flat_max_dyh2, 	
HI541WriteCmosSensor(0x18A4, 0x002C);  // 0x703d : flat_max_stdl1,  0x703e : flat_max_stdh1,  	
HI541WriteCmosSensor(0x18A6, 0x0000);  // 0x703f : flat_max_stdl2,	0x7040 : flat_max_stdh2, 
HI541WriteCmosSensor(0x18A8, 0x3F3F);  // 0x742c : nr_lum_ofs_1,	0x742d : nr_lum_ofs_2,	
HI541WriteCmosSensor(0x18AA, 0x3F3D);  // 0x742e : nr_lum_ofs_3,  	0x742f : nr_lum_ofs_4,	
HI541WriteCmosSensor(0x18AC, 0x3C38);  // 0x7430 : nr_lum_ofs_5,	0x7431 : nr_lum_ofs_6,	
HI541WriteCmosSensor(0x18AE, 0x3634);  // 0x7432 : nr_lum_ofs_7,	0x7433 : nr_lum_ofs_8,	
HI541WriteCmosSensor(0x18B0, 0x3F3F);  // 0x7434 : nr_imp_th_1,	0x7435 : nr_imp_th_2,	
HI541WriteCmosSensor(0x18B2, 0x3E38);  // 0x7436 : nr_imp_th_3,	0x7437 : nr_imp_th_4,	
HI541WriteCmosSensor(0x18B4, 0x3331);  // 0x7438 : nr_imp_th_5,	0x7439 : nr_imp_th_6, 	
HI541WriteCmosSensor(0x18B6, 0x3030);  // 0x743a : nr_imp_th_7,	0x743b : nr_imp_th_8,	
HI541WriteCmosSensor(0x18B8, 0x2018);  // error 0x18B8 0x18B9 0x743c : nr_rate,		0x743d : line_rate,	
HI541WriteCmosSensor(0x18BA, 0x120F);  // 0x7800 : splum_gain_pos_1,0x7801 : splum_gain_pos_2,
HI541WriteCmosSensor(0x18BC, 0x0E0E);  // 0x7802 : splum_gain_pos_3,0x7803 : splum_gain_pos_4, 
HI541WriteCmosSensor(0x18BE, 0x0E10);  // 0x7804 : splum_gain_pos_5,0x7805 : splum_gain_pos_6, 
HI541WriteCmosSensor(0x18C0, 0x1212);  // 0x7806 : splum_gain_pos_7,0x7807 : splum_gain_pos_8, 
HI541WriteCmosSensor(0x18C2, 0x120F);  // 0x7808 : splum_gain_neg_1,0x7809 : splum_gain_neg_2, 
HI541WriteCmosSensor(0x18C4, 0x0F12);  // 0x780a : splum_gain_neg_3,0x780b : splum_gain_neg_4, 
HI541WriteCmosSensor(0x18C6, 0x1212);  // 0x780c : splum_gain_neg_5,0x780d : splum_gain_neg_6, 
HI541WriteCmosSensor(0x18C8, 0x1212);  // 0x780e : splum_gain_neg_7,0x780f : splum_gain_neg_8, 
HI541WriteCmosSensor(0x18CA, 0x0F0F);  // 0x7810 : spdy_gain_pos_1,	0x7811 : spdy_gain_pos_2,	
HI541WriteCmosSensor(0x18CC, 0x1212);  // 0x7812 : spdy_gain_pos_3, 0x7813 : spdy_gain_pos_4,	                              
HI541WriteCmosSensor(0x18CE, 0x1204);  // 0x7814 : spdy_gain_pos_5, 0x7815 : spdy_gain_pos_6, //20140627	
HI541WriteCmosSensor(0x18D0, 0x0404);  // 0x7816 : spdy_gain_pos_7, 0x7817 : spdy_gain_pos_8, //20140627	
HI541WriteCmosSensor(0x18D2, 0x0F0F);  // 0x7818 : spdy_gain_neg_1, 0x7819 : spdy_gain_neg_2,	                             
HI541WriteCmosSensor(0x18D4, 0x1212);  // 0x781a : spdy_gain_neg_3, 0x781b : spdy_gain_neg_4,	
HI541WriteCmosSensor(0x18D6, 0x1204);  // 0x781c : spdy_gain_neg_5, 0x781d : spdy_gain_neg_6, //20140627	
HI541WriteCmosSensor(0x18D8, 0x0404);  // 0x781e : spdy_gain_neg_7, 0x781f : spdy_gain_neg_8, //20140627	
HI541WriteCmosSensor(0x18DA, 0x1C18);  // 0x7820 : spedge_gain_1,   0x7821 : spedge_gain_2,	
HI541WriteCmosSensor(0x18DC, 0x1818);  // 0x7822 : spedge_gain_3,   0x7823 : spedge_gain_4, 	
HI541WriteCmosSensor(0x18DE, 0x1D20);  // 0x7824 : spedge_gain_5,   0x7825 : spedge_gain_6,	
HI541WriteCmosSensor(0x18E0, 0x2020);  // 0x7826 : spedge_gain_7,   0x7827 : spedge_gain_8,	
HI541WriteCmosSensor(0x18E2, 0x1212);  // 0x7830 : spstd_gain_1,    0x7831 : spstd_gain_2,	
HI541WriteCmosSensor(0x18E4, 0x1318);  // 0x7832 : spstd_gain_3,    0x7833 : spstd_gain_4,	
HI541WriteCmosSensor(0x18E6, 0x1818);  // 0x7834 : spstd_gain_5,    0x7835 : spstd_gain_6,                               
HI541WriteCmosSensor(0x18E8, 0x1818);  // 0x7836 : spstd_gain_7,    0x7837 : spstd_gain_8,                                   
HI541WriteCmosSensor(0x18EA, 0x9800);  // 0x7838 : post_std_sel,	0x7440 : flt_luml1,
HI541WriteCmosSensor(0x18EC, 0xFF00);  // 0x7441 : flt_lumh1,	0x7442 : flt_luml2,                                  
HI541WriteCmosSensor(0x18EE, 0xFF00);  // 0x7443 : flt_lumh2,	0x7444 : flt_stdl1,	
HI541WriteCmosSensor(0x18F0, 0x4000);  // 0x7445 : flt_stdh1,	0x7446 : flt_stdl2,	
HI541WriteCmosSensor(0x18F2, 0x2A00);  // 0x7447 : flt_stdh2, 	0x7448 : flt_dyl1,
HI541WriteCmosSensor(0x18F4, 0x2700);  // 0x7449 : flt_dyh1,	0x744a : flt_dyl2,	                    
HI541WriteCmosSensor(0x18F6, 0x20FF);  // 0x744b : flt_dyh2,	0x7458 : flt_rate1,			                    
HI541WriteCmosSensor(0x18F8, 0x327A);  // 0x744c : flt_lumll1,	0x744d : flt_lumhh1,	
HI541WriteCmosSensor(0x18FA, 0x0000);  // 0x744e : flt_lumll2, 	0x744f : flt_lumhh2,	
HI541WriteCmosSensor(0x18FC, 0x0121);  // 0x7450 : flt_stdll1,	0x7451 : flt_stdhh1,	
HI541WriteCmosSensor(0x18FE, 0x0000);  // 0x7452 : flt_stdll2, 	0x7453 : flt_stdhh2,	
HI541WriteCmosSensor(0x1900, 0x011D);  // 0x7454 : flt_dyll1,       0x7455 : flt_dyhh1,	
HI541WriteCmosSensor(0x1902, 0x0000);  // 0x7456 : flt_dyll2, 	0x7457 : flt_dyhh2, 	
HI541WriteCmosSensor(0x1904, 0xFF1A);  // 0x7459 : flt_rate2,       0x7465 : highgain,	
HI541WriteCmosSensor(0x1906, 0x1636);  // 0x7466 : lowgain,         0x746b : cent_flt_sel,	
HI541WriteCmosSensor(0x1908, 0x2FFF);  // 0x746c : sharp_sel,       0x746d : resol_ctl,	
HI541WriteCmosSensor(0x190A, 0x5010);  // 0x746e : flat_sharpgainh, 0x746f : flat_sharpgainl,
HI541WriteCmosSensor(0x190C, 0x4C24);  // 0x7470 : skin_sharpgainh, 0x7471 : skin_sharpgainl, //20140702 8 -> 12
HI541WriteCmosSensor(0x190E, 0x507F);  // 0x7472 : byr_sp_line,	0x7488 : skin_cyan_ctl,	
HI541WriteCmosSensor(0x1910, 0x0091);  // 0x7489 : flat_max_th,     0x7c04 : ci_lpf_ctl_01,	
HI541WriteCmosSensor(0x1912, 0xA43C);  // 0x7c05 : ci_lpf_ctl_02,   0x7c06 : ci_lpf_ctl_03,	
HI541WriteCmosSensor(0x1914, 0x3830);  // 0x7c0e : ci_lum_ofs_1,    0x7c0f : ci_lum_ofs_2,  	
HI541WriteCmosSensor(0x1916, 0x2A2A);  // 0x7c10 : ci_lum_ofs_3,    0x7c11 : ci_lum_ofs_4,  	
HI541WriteCmosSensor(0x1918, 0x2420);  // 0x7c12 : ci_lum_ofs_5,    0x7c13 : ci_lum_ofs_6, 	
HI541WriteCmosSensor(0x191A, 0x1A16);  // 0x7c14 : ci_lum_ofs_7,    0x7c15 : ci_lum_ofs_8,  	
HI541WriteCmosSensor(0x191C, 0x62FC);  // 0x7c16 : ci_ps_dy_ctl,    0x7c17 : ci_ps_max_th,	//20140627 for ps off
HI541WriteCmosSensor(0x191E, 0x0210);  // 0x7c18 : ci_ps_min_th,    0x7c19 : ci_ln_ps_gain_1,	
HI541WriteCmosSensor(0x1920, 0x121A);  // 0x7c1a : ci_ln_ps_gain_2, 0x7c1b : ci_ln_ps_gain_3,	
HI541WriteCmosSensor(0x1922, 0x1D1E);  // 0x7c1c : ci_ln_ps_gain_4, 0x7c1d : ci_ln_ps_gain_5,	
HI541WriteCmosSensor(0x1924, 0x1F10);  // 0x7c1e : ci_ln_ps_gain_6, 0x7c1f : ci_dy_ps_gain_1,	
HI541WriteCmosSensor(0x1926, 0x1218);  // 0x7c20 : ci_dy_ps_gain_2, 0x7c21 : ci_dy_ps_gain_3,	                             
HI541WriteCmosSensor(0x1928, 0x1C1E);  // 0x7c22 : ci_dy_ps_gain_4, 0x7c23 : ci_dy_ps_gain_5,	
HI541WriteCmosSensor(0x192A, 0x1F00);  // 0x7c24 : ci_dy_ps_gain_6, 0x7c25 : ci_ps_type_ctl, 	
HI541WriteCmosSensor(0x192C, 0x2A32);  // 0x7c26 : ci_ln_min_th,    0x7c27 : ci_dy_min_th,	
HI541WriteCmosSensor(0x192E, 0x2020);  // 0x940c : ynr_lum_gain_1,  0x940d : ynr_lum_gain_2, 
HI541WriteCmosSensor(0x1930, 0x2020);  // 0x940e : ynr_lum_gain_3,  0x940f : ynr_lum_gain_4,	
HI541WriteCmosSensor(0x1932, 0x2020);  // 0x9410 : ynr_lum_gain_5,  0x9411 : ynr_lum_gain_6, 
HI541WriteCmosSensor(0x1934, 0x2020);  // 0x9412 : ynr_lum_gain_7,  0x9413 : ynr_lum_gain_8,	
HI541WriteCmosSensor(0x1936, 0x0218);  // 0x940a : ynr_line_rate,   0x940b : ynr_flat_ctl,	
HI541WriteCmosSensor(0x1938, 0x3662);  // 0x9414 : ynr_flat_luml1,  0x9415 : ynr_flat_lumh1, 
HI541WriteCmosSensor(0x193A, 0x63FF);  // 0x9416 : ynr_flat_luml2,	0x9417 : ynr_flat_lumh2,	
HI541WriteCmosSensor(0x193C, 0x0445);  // 0x9418 : ynr_flat_stdl1,  0x9419 : ynr_flat_stdh1,	
HI541WriteCmosSensor(0x193E, 0x0565);  // 0x941a : ynr_flat_stdl2,  0x941b : ynr_flat_stdh2,	
HI541WriteCmosSensor(0x1940, 0x0222);  // 0x941c : ynr_flat_dyl1,   0x941d : ynr_flat_dyh1,	
HI541WriteCmosSensor(0x1942, 0x0630);  // 0x941e : ynr_flat_dyl2,   0x941f : ynr_flat_dyh2,	
HI541WriteCmosSensor(0x1944, 0x080A);  // 0x9426 : cnr_cf_cb_awm_th,0x9427 : cnr_cf_cr_awm_th,
HI541WriteCmosSensor(0x1946, 0xFA77);  // 0x9448 : ac_mul_ctl12_cb, 0x9449 : ac_mul_ctl34_cb,	
HI541WriteCmosSensor(0x1948, 0xFECA);  // 0x944a : ac_mul_ctl12_cr, 0x944b : ac_mul_ctl34_cr,	
HI541WriteCmosSensor(0x194A, 0x7801);  // 0x944e : cnr_lum_min_ctl, 0x9455 : cnr_dc_stdm12,	                            
HI541WriteCmosSensor(0x194C, 0x2233);  // 0x9456 : cnr_dc_stdm34,   0x9457 : cnr_dc_stdm56,	
HI541WriteCmosSensor(0x194E, 0xB398);  // 0x9800 : yee_ctl,         0x9801 : yee_index_gain,	                              
HI541WriteCmosSensor(0x1950, 0x1003);  // 0x9802 : yee_status_gain, 0x9803 : yee_lo_filter,	
HI541WriteCmosSensor(0x1952, 0x237B);  // 0x9804 : yee_lo_gain,     0x9805 : yee_hi_filter,	
HI541WriteCmosSensor(0x1954, 0x1040);  // 0x9806 : yee_hi_gain,     0x9807 : yee_flat_ctl,	
HI541WriteCmosSensor(0x1956, 0x0C0C);  // 0x9808 : yee_skin_low_gain,    0x9809 : yee_skin_higgain,	
HI541WriteCmosSensor(0x1958, 0x1212);  // 0x980a : yee_std_post_gain_pos,0x980b : yee_std_gain,	//20140628
HI541WriteCmosSensor(0x195A, 0x1414);  // 0x980c : yee_lum_gain_p1, 0x980d : yee_lum_gain_p2,	
HI541WriteCmosSensor(0x195C, 0x1414);  // 0x980e : yee_lum_gain_p3, 0x980f : yee_lum_gain_p4,  
HI541WriteCmosSensor(0x195E, 0x1414);  // 0x9810 : yee_lum_gain_p5, 0x9811 : yee_lum_gain_p6, 
HI541WriteCmosSensor(0x1960, 0x1414);  // 0x9812 : yee_lum_gain_p7, 0x9813 : yee_lum_gain_p8,
HI541WriteCmosSensor(0x1962, 0x1212);  // 0x9814 : yee_lum_gain_n1, 0x9815 : yee_lum_gain_n2,	
HI541WriteCmosSensor(0x1964, 0x1211);  // 0x9816 : yee_lum_gain_n3, 0x9817 : yee_lum_gain_n4,	
HI541WriteCmosSensor(0x1966, 0x0908);  // 0x9818 : yee_lum_gain_n5, 0x9819 : yee_lum_gain_n6,	
HI541WriteCmosSensor(0x1968, 0x1012);  // 0x981a : yee_lum_gain_n7, 0x981b : yee_lum_gain_n8,	
HI541WriteCmosSensor(0x196A, 0x1414);  // 0x981c : yee_dy_gain_p1,  0x981d : yee_dy_gain_p2,	
HI541WriteCmosSensor(0x196C, 0x1414);  // 0x981e : yee_dy_gain_p3,  0x981f : yee_dy_gain_p4,	
HI541WriteCmosSensor(0x196E, 0x1406);  // 0x9820 : yee_dy_gain_p5,  0x9821 : yee_dy_gain_p6, //20140627	
HI541WriteCmosSensor(0x1970, 0x0606);  // 0x9822 : yee_dy_gain_p7,  0x9823 : yee_dy_gain_p8, //20140627	
HI541WriteCmosSensor(0x1972, 0x1212);  // 0x9824 : yee_dy_gain_n1,  0x9825 : yee_dy_gain_n2,	
HI541WriteCmosSensor(0x1974, 0x1212);  // 0x9826 : yee_dy_gain_n3,  0x9827 : yee_dy_gain_n4,	
HI541WriteCmosSensor(0x1976, 0x1204);  // 0x9828 : yee_dy_gain_n5,  0x9829 : yee_dy_gain_n6, //20140627	
HI541WriteCmosSensor(0x1978, 0x0404);  // 0x982a : yee_dy_gain_n7,  0x982b : yee_dy_gain_n8, //20140627	
HI541WriteCmosSensor(0x197A, 0x2020);  // 0x982c : yee_edge_gain_1, 0x982d : yee_edge_gain_2,	
HI541WriteCmosSensor(0x197C, 0x2020);  // 0x982e : yee_edge_gain_3, 0x982f : yee_edge_gain_4,	
HI541WriteCmosSensor(0x197E, 0x2020);  // 0x9830 : yee_edge_gain_5, 0x9831 : yee_edge_gain_6,	
HI541WriteCmosSensor(0x1980, 0x2020);  // 0x9832 : yee_edge_gain_7, 0x9833 : yee_edge_gain_8,
HI541WriteCmosSensor(0x1982, 0x2A2A);  // 0x983c : yee_std_gain_1,  0x983d : yee_std_gain_2,	
HI541WriteCmosSensor(0x1984, 0x2A2A);  // 0x983e : yee_std_gain_3,  0x983f : yee_std_gain_4, 	
HI541WriteCmosSensor(0x1986, 0x2A1A);  // 0x9840 : yee_std_gain_5,  0x9841 : yee_std_gain_6,	
HI541WriteCmosSensor(0x1988, 0x1A1A);  // 0x9842 : yee_std_gain_7,  0x9843 : yee_std_gain_8,	
HI541WriteCmosSensor(0x198A, 0x1327);  // 0x9865 : cnr_color_region_ctl, 0x9866 : cnr_color_sat_th, 
HI541WriteCmosSensor(0x198C, 0x3F3F);  // 0x7c2a : ci_nrlum_ofst_1, 0x7c2b : ci_nrlum_ofst_2,
HI541WriteCmosSensor(0x198E, 0x2A27);  // 0x7c2c : ci_nrlum_ofst_3, 0x7c2d : ci_nrlum_ofst_4,
HI541WriteCmosSensor(0x1990, 0x2323);  // 0x7c2e : ci_nrlum_ofst_5, 0x7c2f : ci_nrlum_ofst_6, 
HI541WriteCmosSensor(0x1992, 0x2323);  // 0x7c30 : ci_nrlum_ofst_7, 0x7c31 : ci_nrlum_ofst_8,
HI541WriteCmosSensor(0x1994, 0xD011);  // 0x7c3f : ci_highgain,     0x7c40 : ci_lowgain,	
HI541WriteCmosSensor(0x1996, 0x0040);  // 0x7c41 : ci_sp_pre_dif_ctl, 0x7c42 : ci_flt_luml1,	
HI541WriteCmosSensor(0x1998, 0xFF11);  // 0x7c43 : ci_flt_lumh1,    0x7c44 : ci_flt_luml2,	
HI541WriteCmosSensor(0x199A, 0x3F00);  // 0x7c45 : ci_flt_lumh2,    0x7c46 : ci_flt_stdl1,	
HI541WriteCmosSensor(0x199C, 0x4C00);  // 0x7c47 : ci_flt_stdh1,    0x7c48 : ci_flt_stdl2,	
HI541WriteCmosSensor(0x199E, 0x1300);  // 0x7c49 : ci_flt_stdh2,    0x7c4a : ci_flt_dyl1,	
HI541WriteCmosSensor(0x19A0, 0x2A00);  // 0x7c4b : ci_flt_dyh1,     0x7c4c : ci_flt_dyl2,	
HI541WriteCmosSensor(0x19A2, 0x172C);  // 0x7c4d : ci_flt_dyh2,     0x7c4e : ci_flat_ctl,	
HI541WriteCmosSensor(0x19A4, 0x080A);  // 0x7c50 : ci_splum_n1,     0x7c51 : ci_splum_n2,	
HI541WriteCmosSensor(0x19A6, 0x1012);  // 0x7c52 : ci_splum_n3,     0x7c53 : ci_splum_n4,	
HI541WriteCmosSensor(0x19A8, 0x1212);  // 0x7c54 : ci_splum_n5,     0x7c55 : ci_splum_n6,	
HI541WriteCmosSensor(0x19AA, 0x1212);  // 0x7c56 : ci_splum_n7,     0x7c57 : ci_splum_n8,	
HI541WriteCmosSensor(0x19AC, 0x1212);  // 0x7c58 : ci_splum_p1,     0x7c59 : ci_splum_p2,	
HI541WriteCmosSensor(0x19AE, 0x1212);  // 0x7c5a : ci_splum_p3,     0x7c5b : ci_splum_p4,	
HI541WriteCmosSensor(0x19B0, 0x1212);  // 0x7c5c : ci_splum_p5,     0x7c5d : ci_splum_p6,	
HI541WriteCmosSensor(0x19B2, 0x1212);  // 0x7c5e : ci_splum_p7,     0x7c5f : ci_splum_p8,	
HI541WriteCmosSensor(0x19B4, 0x1212);  // 0x7c60 : ci_spdy_n1,      0x7c61 : ci_spdy_n2,	
HI541WriteCmosSensor(0x19B6, 0x1212);  // 0x7c62 : ci_spdy_n3,      0x7c63 : ci_spdy_n4,	
HI541WriteCmosSensor(0x19B8, 0x1204);  // 0x7c64 : ci_spdy_n5,      0x7c65 : ci_spdy_n6, //20140627	
HI541WriteCmosSensor(0x19BA, 0x0404);  // 0x7c66 : ci_spdy_n7,      0x7c67 : ci_spdy_n8, //20140627	
HI541WriteCmosSensor(0x19BC, 0x1212);  // 0x7c68 : ci_spdy_p1,      0x7c69 : ci_spdy_p2,	
HI541WriteCmosSensor(0x19BE, 0x1212);  // 0x7c6a : ci_spdy_p3,      0x7c6b : ci_spdy_p4,	
HI541WriteCmosSensor(0x19C0, 0x1204);  // 0x7c6c : ci_spdy_p5,      0x7c6d : ci_spdy_p6, //20140627	                                   
HI541WriteCmosSensor(0x19C2, 0x0404);  // 0x7c6e : ci_spdy_p7,      0x7c6f : ci_spdy_p8, //20140627	
HI541WriteCmosSensor(0x19C4, 0x2020);  // 0x7c70 : ci_spedge_1,     0x7c71 : ci_spedge_2,	
HI541WriteCmosSensor(0x19C6, 0x2020);  // 0x7c72 : ci_spedge_3, 	0x7c73 : ci_spedge_4,	
HI541WriteCmosSensor(0x19C8, 0x2020);  // 0x7c74 : ci_spedge_5,	0x7c75 : ci_spedge_6,	
HI541WriteCmosSensor(0x19CA, 0x2020);  // 0x7c76 : ci_spedge_7,	0x7c77 : ci_spedge_8,	
HI541WriteCmosSensor(0x19CC, 0x1A1A);  // 0x7c7d : ci_spstd_1,	0x7c7e : ci_spstd_2,	
HI541WriteCmosSensor(0x19CE, 0x1A1A);  // 0x7c7f : ci_spstd_3,	0x7c80 : ci_spstd_4,	
HI541WriteCmosSensor(0x19D0, 0x1A1A);  // 0x7c81 : ci_spstd_5,	0x7c82 : ci_spstd_6,	
HI541WriteCmosSensor(0x19D2, 0x1A1A);  // 0x7c83 : ci_spstd_7,	0x7c84 : ci_spstd_8,	
HI541WriteCmosSensor(0x19D4, 0x3600);  // 0x7c85 : ci_post_std_ctl, 0x7c97 : ci_skin_blue_ctl,
HI541WriteCmosSensor(0x19D6, 0x0001);  // 0x9004 : sat_tra_sign_dk, 0x9005 : sat_tra_deltah_dk,
HI541WriteCmosSensor(0x19D8, 0x20ff);  // 0x9008 : sat_tra_delta1_dk,0x9009 : sat_tra_delta2_dk,
HI541WriteCmosSensor(0x19DA, 0xe00a);  // 0x900a : sat_tra_delta3_dk,0x900b : sat_tra_delta4_dk,
HI541WriteCmosSensor(0x19DC, 0x1a21);  // 0x900c : sat_tra_yb1_dk,	0x900e : sat_tra_yb2_dk_h,	
HI541WriteCmosSensor(0x19DE, 0xe02a);  // 0x900f : sat_tra_yb2_dk_l,0x9010 : sat_tra_yb3_dk_h,	
HI541WriteCmosSensor(0x19E0, 0xa037);  // 0x9011 : sat_tra_yb3_dk_l,0x9012 : sat_tra_yb4_dk_h,	
HI541WriteCmosSensor(0x19E2, 0xb007);  // 0x9013 : sat_tra_yb4_dk_l,0x9014 : sat_tra_th1_dk,	
HI541WriteCmosSensor(0x19E4, 0x0F1E);  // 0x9015 : sat_tra_th2_dk,	0x9016 : sat_tra_th3_dk,	

HI541WriteCmosSensor(0x1A5A, 0x0F00);  // 0x9ca4 : mcmc_allgain_y11,0x7c3c : ci_gain_std_ofst, //seperating MCMC Adaptive register

HI541WriteCmosSensor(0x1A5C, 0x871E);  // 0x7c3d : ci_spflt_std_ctl,0x7400 : lensd_lpf_ctl1,	
HI541WriteCmosSensor(0x1A5E, 0x7700);  // 0x7401 : lpf_flt_sel,	0x7404 : imp_ctl,		
HI541WriteCmosSensor(0x1A60, 0x8104);  // 0x7405 : nr_lpf_sel_01,	0x7406 : nr_lpf_sel_02,	
HI541WriteCmosSensor(0x1A62, 0x5830);  // 0x7407 : nr_lpf_std_01,	0x7408 : nr_lpf_std_02,	
HI541WriteCmosSensor(0x1A64, 0x1212);  // 0x7409 : nr_lpf_std_03,	0x740a : org_std_ctl,	
HI541WriteCmosSensor(0x1A66, 0xC020);  // 0x740b : org_std_oft,	0x743c : nr_rate,			
HI541WriteCmosSensor(0x1A68, 0x189F);  // 0x743d : line_rate, 	0x7464 : byr_sp_onoff,	
HI541WriteCmosSensor(0x1A6A, 0xF101);  // 0x7467 : sf_sel_std_sel, 	0x7c28 : ci_cnr_ctl,	
HI541WriteCmosSensor(0x1A6C, 0x15E7);  // 0x7c29 : ci_y_nr_ctl,	0x7c32 : ci_nrgain_std_ctl,
HI541WriteCmosSensor(0x1A6E, 0x0000);  // 0x7c33 : ci_nrgain_std_ofst,0x7c34 : ci_cbcr_gain,	
HI541WriteCmosSensor(0x1A70, 0x0295);  // 0x7c35 : ci_dy_gain, 	0x7c36 : ci_filt_sel_ctl,	
HI541WriteCmosSensor(0x1A72, 0x4020);  // 0x7c37 : ci_yfilt_sel_h,  0x7c38 : ci_yfilt_sel_m,	
HI541WriteCmosSensor(0x1A74, 0x109F);  // 0x7c39 : ci_yfilt_sel_l,  0x7c3b : ci_spstd_ctl,	
HI541WriteCmosSensor(0x1A76, 0x100D);  // 0x7c4f : ci_sp_ln_ctl, 	0x7caa : ci_otp1,		
HI541WriteCmosSensor(0x1A78, 0x0F2C);  // 0x7cac : ci_weak_hv_gain, 0x7cad : ci_moire_th1,	
HI541WriteCmosSensor(0x1A7A, 0x1307);  // 0x7cae : ci_moire_th2, 	0x8000 : cmc_ctl1,		
HI541WriteCmosSensor(0x1A7C, 0x0000);  // 0x8002 : cmc_ps_ln_cmc_gain,0x8800 : gma_ctl1,	//20140627 for ps	
HI541WriteCmosSensor(0x1A7E, 0x000E);  // 0x8802 : gma_ps_ln_gain, 	0x9401 : ynr_ctl2,	//20140627 for ps	
HI541WriteCmosSensor(0x1A80, 0xDA25);  // 0x9402 : ynr_ctl3, 	0x9403 : ynr_ctl4,		
HI541WriteCmosSensor(0x1A82, 0x3060);  // 0x9404 : ynr_std_l_th, 	0x9405 : ynr_std_m_th,	
HI541WriteCmosSensor(0x1A84, 0xA03F);  // 0x9406 : ynr_std_h_th, 	0x7c86 : ci_splum_max_n1,	
HI541WriteCmosSensor(0x1A86, 0x3F3F);  // 0x7c87 : ci_splum_max_n2,	0x7c88 : ci_splum_max_n3,	
HI541WriteCmosSensor(0x1A88, 0x3F3F);  // 0x7c89 : ci_splum_max_n4,	0x7c8a : ci_splum_max_n5,	
HI541WriteCmosSensor(0x1A8A, 0x3F3F);  // 0x7c8b : ci_splum_max_p1,	0x7c8c : ci_splum_max_p2,	
HI541WriteCmosSensor(0x1A8C, 0x3F3F);  // 0x7c8d : ci_splum_max_p3,	0x7c8e : ci_splum_max_p4,	
HI541WriteCmosSensor(0x1A8E, 0x3F13);  // 0x7c8f : ci_splum_max_p5, 0x7ca5 : ci_dy_sel_ctl,	
HI541WriteCmosSensor(0x1A90, 0x1330);  // 0x7ca6 : ci_lum_sel_ctl  // INDOOR 0x7c9a : bi_std_th // 2014628 for indoor diag noise (patch rev)

HI541WriteCmosSensor(0x0dde, 0x0230);  //DARK2 0x7c9a : bi_std_th // 0x0ddf // 20140714 for diag noise (patch rev)
HI541WriteCmosSensor(0x0de8, 0x0030);  //DARK2 0x7c9b : bi_dy_th  // 0x0de9 // 20140714 for diag noise (patch rev)

//===================================================
//========= Luminance Adaptive ISP END    ===========
//===================================================

//===================================================
//=================== SYSTEM Start ==================
//===================================================
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x3824,0x0000);
HI541WriteCmosSensor(0x3826,0x5a10);//MCU preview hif cmd

//===================================================
//======= MIPI Timing Speed 352Mbps START ===========
//===================================================
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0xbc0e, 0x0302); // LPX, CLK Prepare
HI541WriteCmosSensor(0xbc10, 0x1604); // CLK Zero, CLK Pre
HI541WriteCmosSensor(0xbc12, 0x030c); // HS Prepare, HS Zero
HI541WriteCmosSensor(0xbc14, 0x060e); // HS Zero, CLK Post
HI541WriteCmosSensor(0xbc16, 0x050a); // HS Prepare, HS Zero
//===================================================
//=======  MIPI Timing Speed 672Mbps END  ===========
//===================================================

//===================================================
//========= WDR Start    ===========
//===================================================
HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x2076,0x3040);//WDR enable
//===================================================
//========= WDR END    ===========
//===================================================

HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x0100,0x0100); //Sleep Off

///END
//[END]

#endif



}
    
}


void HI541_FOCUS_AFC_Init(void)

{

//DISP_NAME = "AF_InitParaList"
//BEGIN
//=====================================================
//=========== AF Filter Start - preview    ============
//===================================================== 

HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0xC800, 0x4840);	// sobel& spmd  
HI541WriteCmosSensor(0xc802, 0x0044);
HI541WriteCmosSensor(0xc804, 0x0044);
HI541WriteCmosSensor(0xC806, 0xffC3); // af_ctl8 // manual window setting + x_half[6] 
HI541WriteCmosSensor(0xC808, 0x8000); // af_ctl8 // manual window setting + x_half[6] 

//========================================
//=========== F/W AF Start ===============
//========================================
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x003E, 0x8080);			// AWB Lock
HI541WriteCmosSensor(0x0020, 0xff00); 			// IICM Delay

HI541WriteCmosSensor(0x05B4, 0x0af6); 			//0x05b5 : f8(8), f7(9), f6(10),  EC(20) macro shift 2014.05.27
HI541WriteCmosSensor(0x05B6, 0x3219);			// Intelligent Step Size = 20

HI541WriteCmosSensor(0x05B8, 0x0819);			// Fine Scan Step Size
HI541WriteCmosSensor(0x05Ba, 0x0003);			// Falling Threshold Number
	HI541WriteCmosSensor(0x05C0, 0x1100);			// Skip Frame	 
HI541WriteCmosSensor(0x05C8, 0x6414);			// softlanding step / first scan outdoor/ 	 
HI541WriteCmosSensor(0x05CA, 0x1414);			// first scan indoor / first scan dark

HI541WriteCmosSensor(0x05F4, 0x0200);			// Ae STD thresholg
	HI541WriteCmosSensor(0x0628, 0x1e15);
HI541WriteCmosSensor(0x062A, 0x8000);			// min duty		// 250
HI541WriteCmosSensor(0x062C, 0xBC02);			// max duty
HI541WriteCmosSensor(0x062e, 0x9001);			// marco min duty

HI541WriteCmosSensor(0x0668, 0x4b4b);			// Uniform Th (rev BE)
HI541WriteCmosSensor(0x066a, 0x4001);			// Hyper Position (rev BE)

HI541WriteCmosSensor(0x0680, 0x1111);			// weight
HI541WriteCmosSensor(0x0682, 0x1111);			// weight
HI541WriteCmosSensor(0x0684, 0x2828);			// center weight  

//20140805
	//HI541WriteCmosSensor(0x05CE, 0x0003);        // MSB Masking set
//HI541WriteCmosSensor(0x069C, 0x0008);        // MSB / LSB shift set


	HI541WriteCmosSensor(0xffff, 0x0020);
	if (0x00AE == HI541ReadCmosSensor(0x0002))
	{
HI541WriteCmosSensor(0x0714, 0xC000);			// outdoor step LUT 1
HI541WriteCmosSensor(0x0716, 0xD000);			// outdoor step LUT 2	
HI541WriteCmosSensor(0x0718, 0xE000);			// outdoor step LUT 3	
HI541WriteCmosSensor(0x071A, 0xF000);			// outdoor step LUT 4	
HI541WriteCmosSensor(0x071C, 0x0001);			// outdoor step LUT 5	
HI541WriteCmosSensor(0x071E, 0x1001);			// outdoor step LUT 6	
HI541WriteCmosSensor(0x0720, 0x2001);			// outdoor step LUT 7	
HI541WriteCmosSensor(0x0722, 0x3001);			// outdoor step LUT 8	
HI541WriteCmosSensor(0x0724, 0x4001);			// outdoor step LUT 9	
HI541WriteCmosSensor(0x0726, 0x5001);			// outdoor step LUT 10	
HI541WriteCmosSensor(0x0728, 0x6001);			// outdoor step LUT 11	   
HI541WriteCmosSensor(0x072a, 0x7001);			// outdoor step LUT 12
HI541WriteCmosSensor(0x072c, 0x7801);			// outdoor step LUT 13
HI541WriteCmosSensor(0x072e, 0x8001);			// outdoor step LUT 14
HI541WriteCmosSensor(0x0730, 0x8801);			// outdoor step LUT 15
HI541WriteCmosSensor(0x0732, 0x9001);			// outdoor step LUT 16
HI541WriteCmosSensor(0x0734, 0xA001);			// outdoor step LUT 17
HI541WriteCmosSensor(0x0736, 0xB001);			// outdoor step LUT 18
HI541WriteCmosSensor(0x0738, 0xC001);			// outdoor step LUT 19
HI541WriteCmosSensor(0x073A, 0xC001);			// outdoor step LUT 20
HI541WriteCmosSensor(0x073C, 0xC001);			// outdoor step LUT 20


HI541WriteCmosSensor(0x0748, 0xC000);			// indoor step LUT 1
HI541WriteCmosSensor(0x074A, 0xD000);			// indoor step LUT 2	
HI541WriteCmosSensor(0x074C, 0xE000);			// indoor step LUT 3	
HI541WriteCmosSensor(0x074E, 0xF000);			// indoor step LUT 4	
HI541WriteCmosSensor(0x0750, 0x0001);			// indoor step LUT 5	
HI541WriteCmosSensor(0x0752, 0x1001);			// indoor step LUT 6	
HI541WriteCmosSensor(0x0754, 0x2001);			// indoor step LUT 7	
HI541WriteCmosSensor(0x0756, 0x3001);			// indoor step LUT 8	
HI541WriteCmosSensor(0x0758, 0x4001);			// indoor step LUT 9	
HI541WriteCmosSensor(0x075A, 0x5001);			// indoor step LUT 10	
HI541WriteCmosSensor(0x075C, 0x6001);
HI541WriteCmosSensor(0x075E, 0x7001);			// 
HI541WriteCmosSensor(0x0760, 0x7801);			//
HI541WriteCmosSensor(0x0762, 0x8001);			//
HI541WriteCmosSensor(0x0764, 0x8801);			//
HI541WriteCmosSensor(0x0766, 0x9001);			//
HI541WriteCmosSensor(0x0768, 0xA001);			//
HI541WriteCmosSensor(0x076A, 0xB001);			//
HI541WriteCmosSensor(0x076C, 0xC001);			//
HI541WriteCmosSensor(0x076E, 0xC001);			//
HI541WriteCmosSensor(0x0770, 0xC001);			//


HI541WriteCmosSensor(0x077C, 0xC000);			// dark step LUT 1
HI541WriteCmosSensor(0x077E, 0xD000);			// dark step LUT 2	
HI541WriteCmosSensor(0x0780, 0xE000);			// dark step LUT 3	
HI541WriteCmosSensor(0x0782, 0xF000);			// dark step LUT 4	
HI541WriteCmosSensor(0x0784, 0x0001);			// dark step LUT 5	
HI541WriteCmosSensor(0x0786, 0x1001);			// dark step LUT 6	
HI541WriteCmosSensor(0x0788, 0x2001);			// dark step LUT 7	
HI541WriteCmosSensor(0x078A, 0x3001);			// dark step LUT 8	
HI541WriteCmosSensor(0x078C, 0x4001);			// dark step LUT 9	
HI541WriteCmosSensor(0x078E, 0x5001);			// dark step LUT 10	
HI541WriteCmosSensor(0x0790, 0x6001);
HI541WriteCmosSensor(0x0792, 0x7001);
HI541WriteCmosSensor(0x0794, 0x7801);
HI541WriteCmosSensor(0x0796, 0x8001);
HI541WriteCmosSensor(0x0798, 0x8801);
HI541WriteCmosSensor(0x079A, 0x9001);
HI541WriteCmosSensor(0x079C, 0xA001);
HI541WriteCmosSensor(0x079E, 0xB001);
HI541WriteCmosSensor(0x07A0, 0xC001);
HI541WriteCmosSensor(0x07A2, 0xC001);
HI541WriteCmosSensor(0x07A4, 0xC001);



// added to Improve speed continuous AF
		HI541WriteCmosSensor(0x406E, 0x0014);			// Step Size for Cont.
		HI541WriteCmosSensor(0x4070, 0x1414);
//{0x05F4, 0x0200},                
//{0x05F8, 0x0008},                

	}
	else
//DISP_NAME = "AF_InitParaList"
//BEGIN
//=====================================================
//=========== AF Filter Start - preview    ============
//=====================================================
	{
//========================================
//=========== F/W AF Start ===============
//========================================
		HI541WriteCmosSensor(0x0728, 0xC000);			// outdoor step LUT 1
		HI541WriteCmosSensor(0x072A, 0xD000);			// outdoor step LUT 2	
		HI541WriteCmosSensor(0x072C, 0xE000);			// outdoor step LUT 3	

		HI541WriteCmosSensor(0x072E, 0xF000);			// outdoor step LUT 4	
		HI541WriteCmosSensor(0x0730, 0x0001);			// outdoor step LUT 5	

		HI541WriteCmosSensor(0x0732, 0x1001);			// outdoor step LUT 6	
		HI541WriteCmosSensor(0x0734, 0x2001);			// outdoor step LUT 7	
		HI541WriteCmosSensor(0x0736, 0x3001);			// outdoor step LUT 8	
		HI541WriteCmosSensor(0x0738, 0x4001);			// outdoor step LUT 9	
		HI541WriteCmosSensor(0x073A, 0x5001);			// outdoor step LUT 10	

		HI541WriteCmosSensor(0x073C, 0x6001);			// outdoor step LUT 11	   
		HI541WriteCmosSensor(0x073E, 0x7001);			// outdoor step LUT 12
		HI541WriteCmosSensor(0x0740, 0x7801);			// outdoor step LUT 13
		HI541WriteCmosSensor(0x0742, 0x8001);			// outdoor step LUT 14

		HI541WriteCmosSensor(0x0744, 0x8801);			// outdoor step LUT 15
		HI541WriteCmosSensor(0x0746, 0x9001);			// outdoor step LUT 16

		HI541WriteCmosSensor(0x0748, 0xA001);			// outdoor step LUT 17
		HI541WriteCmosSensor(0x074A, 0xB001);			// outdoor step LUT 18
		HI541WriteCmosSensor(0x074C, 0xC001);			// outdoor step LUT 19

//20140805
		HI541WriteCmosSensor(0x074E, 0xC001);			// outdoor step LUT 20
		HI541WriteCmosSensor(0x0750, 0xC001);			// outdoor step LUT 20
//HI541WriteCmosSensor(0x069C, 0x0008);        // MSB / LSB shift set


		HI541WriteCmosSensor(0x075C, 0xC000);			// indoor step LUT 1
		HI541WriteCmosSensor(0x075E, 0xD000);			// indoor step LUT 2	
		HI541WriteCmosSensor(0x0760, 0xE000);			// indoor step LUT 3	
		HI541WriteCmosSensor(0x0762, 0xF000);			// indoor step LUT 4	
		HI541WriteCmosSensor(0x0764, 0x0001);			// indoor step LUT 5	
		HI541WriteCmosSensor(0x0766, 0x1001);			// indoor step LUT 6	
		HI541WriteCmosSensor(0x0768, 0x2001);			// indoor step LUT 7	
		HI541WriteCmosSensor(0x076A, 0x3001);			// indoor step LUT 8	
		HI541WriteCmosSensor(0x076C, 0x4001);			// indoor step LUT 9	
		HI541WriteCmosSensor(0x076E, 0x5001);			// indoor step LUT 10	
		HI541WriteCmosSensor(0x0770, 0x6001);
		HI541WriteCmosSensor(0x0772, 0x7001);			// 
		HI541WriteCmosSensor(0x0774, 0x7801);			//
		HI541WriteCmosSensor(0x0776, 0x8001);			//
		HI541WriteCmosSensor(0x0778, 0x8801);			//
		HI541WriteCmosSensor(0x077A, 0x9001);			//
		HI541WriteCmosSensor(0x077C, 0xA001);			//
		HI541WriteCmosSensor(0x077E, 0xB001);			//
		HI541WriteCmosSensor(0x0780, 0xC001);			//
		HI541WriteCmosSensor(0x0782, 0xC001);			//
		HI541WriteCmosSensor(0x0784, 0xC001);			//


		HI541WriteCmosSensor(0x0790, 0xC000);			// dark step LUT 1
		HI541WriteCmosSensor(0x0792, 0xD000);			// dark step LUT 2	
		HI541WriteCmosSensor(0x0794, 0xE000);			// dark step LUT 3	
		HI541WriteCmosSensor(0x0796, 0xF000);			// dark step LUT 4	
		HI541WriteCmosSensor(0x0798, 0x0001);			// dark step LUT 5	
		HI541WriteCmosSensor(0x079A, 0x1001);			// dark step LUT 6	
		HI541WriteCmosSensor(0x079C, 0x2001);			// dark step LUT 7	
		HI541WriteCmosSensor(0x079E, 0x3001);			// dark step LUT 8	
		HI541WriteCmosSensor(0x07A0, 0x4001);			// dark step LUT 9	
		HI541WriteCmosSensor(0x07A2, 0x5001);			// dark step LUT 10	
		HI541WriteCmosSensor(0x07A4, 0x6001);
		HI541WriteCmosSensor(0x07A6, 0x7001);
		HI541WriteCmosSensor(0x07A8, 0x7801);
		HI541WriteCmosSensor(0x07AA, 0x8001);
		HI541WriteCmosSensor(0x07AC, 0x8801);
		HI541WriteCmosSensor(0x07AE, 0x9001);
		HI541WriteCmosSensor(0x07B0, 0xA001);
		HI541WriteCmosSensor(0x07B2, 0xB001);
		HI541WriteCmosSensor(0x07B4, 0xC001);
		HI541WriteCmosSensor(0x07B6, 0xC001);
		HI541WriteCmosSensor(0x07B8, 0xC001);


		HI541WriteCmosSensor(0x2C0A, 0x0014);			// Step Size for Cont. AF
		HI541WriteCmosSensor(0x2C0C, 0x1414);

	}

// added to Improve speed continuous AF
HI541WriteCmosSensor(0x05EA, 0x0008);  
HI541WriteCmosSensor(0x05EC, 0x0800);  
HI541WriteCmosSensor(0x05F0, 0x0408);  
HI541WriteCmosSensor(0x05F2, 0x0800);                  
HI541WriteCmosSensor(0x05F6, 0x0828);                  

HI541WriteCmosSensor(0x05C0, 0x1100);			// Skip Frame	
HI541WriteCmosSensor(0x0628, 0x1E15);  			// Slpope Step / Delay 

	

//========================================
//=========== F/W AF End  ================
//========================================


// no need SeongYW
//========================================
//========  Constant_Focus  Star add Rampart ========
//========================================
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x05b0, 0x8616);	//AF Mode Select
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0x3824, 0x0500);	
HI541WriteCmosSensor(0x3826, 0xAF10);	//Do AF
//========================================
//========  Constant_Focus  End  Rampart   ==========
//========================================


//========================================
//=========== F/W AF End  ================
//========================================
}

void HI541_FOCUS_AFC_Constant_Focus(void)

{
SENSORDB("HI541_FOCUS_AFC_Constant_Focus_start\n");
//DISP_NAME = "AF_AutoParaList"
//BEGIN
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x05b0, 0x8616);	//AF Mode Select
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0x3824, 0x0500);	
HI541WriteCmosSensor(0x3826, 0xAF10);	//Do AF
SENSORDB("HI541_FOCUS_AFC_Constant_Focus_end\n");

}

void HI541_FOCUS_AFC_Single_Focus(void)

{
//DISP_NAME = "AF_AutoParaList"
//BEGIN
SENSORDB("HI541_FOCUS_AFC_Single_Focus_start-----\n");

HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x05b0, 0x9E16);	//AF Mode Select
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0x3824, 0x0200);	//AF Mode Select
HI541WriteCmosSensor(0x3826, 0xAF10);	//Do AF

}
static void printAE_1_ARRAY(void)
{
    UINT32 i;
    for(i=0; i<AE_SECTION_INDEX_MAX; i++)
    {
        SENSORDB("AE_1_ARRAY[%2d]=%d\n", i, AE_1_ARRAY[i]);
    }
}

static void printAE_2_ARRAY(void)
{
    UINT32 i, j;
    SENSORDB("\t\t");
    for(i=0; i<AE_VERTICAL_BLOCKS; i++)
    {
        SENSORDB("      line[%2d]", i);
    }
    printk("\n");
    for(j=0; j<AE_HORIZONTAL_BLOCKS; j++)
    {
        SENSORDB("\trow[%2d]", j);
        for(i=0; i<AE_VERTICAL_BLOCKS; i++)
        {
            //SENSORDB("AE_2_ARRAY[%2d][%2d]=%d\n", j,i,AE_2_ARRAY[j][i]);
            SENSORDB("  %7d", AE_2_ARRAY[j][i]);
        }
        SENSORDB("\n");
    }
}

static void clearAE_2_ARRAY(void)
{
    UINT32 i, j;
    for(j=0; j<AE_HORIZONTAL_BLOCKS; j++)
    {
        for(i=0; i<AE_VERTICAL_BLOCKS; i++)
        {AE_2_ARRAY[j][i]=FALSE;}
    }
}
static void mapAE_2_ARRAY_To_AE_1_ARRAY(void)
{
    UINT32 i, j;
    for(j=0; j<AE_HORIZONTAL_BLOCKS; j++)
    {
        for(i=0; i<AE_VERTICAL_BLOCKS; i++)
        { AE_1_ARRAY[j*AE_VERTICAL_BLOCKS+i] = AE_2_ARRAY[j][i];}
    }
}
static void mapMiddlewaresizePointToPreviewsizePoint(
    UINT32 mx,
    UINT32 my,
    UINT32 mw,
    UINT32 mh,
    UINT32 * pvx,
    UINT32 * pvy,
    UINT32 pvw,
    UINT32 pvh
)
{
    *pvx = pvw * mx / mw;
    *pvy = pvh * my / mh;
    SENSORDB("mapping middlware x[%d],y[%d], [%d X %d]\n\t\tto x[%d],y[%d],[%d X %d]\n ",
        mx, my, mw, mh, *pvx, *pvy, pvw, pvh);
}
static void calcLine(void)
{//line[5]
    UINT32 i;
    UINT32 step = PRV_W / AE_VERTICAL_BLOCKS;
    for(i=0; i<=AE_VERTICAL_BLOCKS; i++)
    {
        *(&line_coordinate[0]+i) = step*i;
        SENSORDB("line[%d]=%d\t",i, *(&line_coordinate[0]+i));
    }
    SENSORDB("\n");
}

static void calcRow(void)
{//row[5]
    UINT32 i;
    UINT32 step = PRV_H / AE_HORIZONTAL_BLOCKS;
    for(i=0; i<=AE_HORIZONTAL_BLOCKS; i++)
    {
        *(&row_coordinate[0]+i) = step*i;
        SENSORDB("row[%d]=%d\t",i,*(&row_coordinate[0]+i));
    }
    SENSORDB("\n");
}
static void calcPointsAELineRowCoordinate(UINT32 x, UINT32 y, UINT32 * linenum, UINT32 * rownum)
{
    UINT32 i;
    i = 1;
    while(i<=AE_VERTICAL_BLOCKS)
    {
        if(x<line_coordinate[i])
        {
            *linenum = i;
            break;
        }
        *linenum = i++;
    }
    i = 1;
    while(i<=AE_HORIZONTAL_BLOCKS)
    {
        if(y<row_coordinate[i])
        {
            *rownum = i;
            break;
        }
        *rownum = i++;
    }
    SENSORDB("PV point [%d, %d] to section line coordinate[%d] row[%d]\n",x,y,*linenum,*rownum);
}



static MINT32 clampSection(UINT32 x, UINT32 min, UINT32 max)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

static void mapCoordinate(UINT32 linenum, UINT32 rownum, UINT32 * sectionlinenum, UINT32 * sectionrownum)
{
    *sectionlinenum = clampSection(linenum-1,0,AE_VERTICAL_BLOCKS-1);
    *sectionrownum = clampSection(rownum-1,0,AE_HORIZONTAL_BLOCKS-1);	
    SENSORDB("mapCoordinate from[%d][%d] to[%d][%d]\n",
		linenum, rownum,*sectionlinenum,*sectionrownum);
}

static void mapRectToAE_2_ARRAY(UINT32 x0, UINT32 y0, UINT32 x1, UINT32 y1)
{
    UINT32 i, j;
    SENSORDB("([%d][%d]),([%d][%d])\n", x0,y0,x1,y1);
    clearAE_2_ARRAY();
    x0=clampSection(x0,0,AE_VERTICAL_BLOCKS-1);
    y0=clampSection(y0,0,AE_HORIZONTAL_BLOCKS-1);
    x1=clampSection(x1,0,AE_VERTICAL_BLOCKS-1);
    y1=clampSection(y1,0,AE_HORIZONTAL_BLOCKS-1);

    for(j=y0; j<=y1; j++)
    {
        for(i=x0; i<=x1; i++)
        {
            AE_2_ARRAY[j][i]=TRUE;
        }
    }
}

static void resetPVAE_2_ARRAY(void)
{
    mapRectToAE_2_ARRAY(1,1,2,2);
}
static void HI541_FOCUS_Get_AF_Max_Num_Focus_Areas(UINT32 *pFeatureReturnPara32)
{ 	  
    *pFeatureReturnPara32 = 1;    
    SENSORDB(" *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);	
}

static void HI541_FOCUS_Get_AE_Max_Num_Metering_Areas(UINT32 *pFeatureReturnPara32)
{ 	
    SENSORDB("[HI541]enter HI541_FOCUS_Get_AE_Max_Num_Metering_Areas function:\n ");
    *pFeatureReturnPara32 = 1;    
    SENSORDB(" *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);
	SENSORDB("[HI541]exit HI541_FOCUS_Get_AE_Max_Num_Metering_Areas function:\n ");
}



/* 
********************************************
*	Modified Touch AF function by SK hynix *
********************************************
*/

#define X_OUTPUT_SIZE	2592
#define Y_OUTPUT_SIZE	1944
#define X_LCD_SIZE 	80
#define Y_LCD_SIZE	60

static void HI541_FOCUS_OVT_AFC_Touch_AF(UINT32 x,UINT32 y)
{
#if 0
	 SENSORDB("[HI541]enter HI541_FOCUS_OVT_AFC_Touch_AF function:\n ");
	 int x_view,y_view;
     int x_tmp,y_tmp;
     if(x<1)
     {
               x_view=1;
     }
     else if(x>79)
     {
               x_view=79;
     }
     else
     {
               x_view= x;
     }
     if(y<1)
     {
               y_view=1;
     }
     else if(y>59)
     {
               y_view=59;
     }
     else
     {
               y_view= y;
     }
	 SENSORDB("[HI541]AF x_view=%d,y_view=%d\n",x_view, y_view);
	  HI541WriteCmosSensor(0xffff,0x2000);
    HI541WriteCmosSensor(0x07b0,x_view);
   HI541WriteCmosSensor(0x07b4,y_view);   
     x_tmp = HI541ReadCmosSensor(0x07b0);
	 y_tmp = HI541ReadCmosSensor(0x07b4);
	 af_xcoordinate = x_tmp;
	 af_ycoordinate = y_tmp;
#else
	 
	SENSORDB("[HI541]enter HI541_FOCUS_OVT_AFC_Touch_AF function:\n ");

	UINT8 x_view,y_view;
	UINT8 x_start_h, x_start_l, x_end_h, x_end_l;
	UINT8 y_start_h, y_start_l, y_end_h, y_end_l;

	UINT16     x_start_backup, x_end_backup	, y_start_backup, y_end_backup;

	int x_tmp,y_tmp;
	int x_tmp_size,y_tmp_size;
	int x_ratio, y_ratio;

	TOUCHAF_FLAG = 1;					// touch flag enable

	x_ratio = (int)X_OUTPUT_SIZE/X_LCD_SIZE;  //2592/80 = 32
	y_ratio = (int)Y_OUTPUT_SIZE/Y_LCD_SIZE;  //1944/60 = 32

	// current window backup
    HI541WriteCmosSensor(0xffff,0x0040);
	x_start_backup = (HI541ReadCmosSensor(0xC84E)	<< 8) | (HI541ReadCmosSensor(0xC84F));
	x_end_backup = (HI541ReadCmosSensor(0xC862) 	<< 8) | (HI541ReadCmosSensor(0xC863));	 
	y_start_backup = (HI541ReadCmosSensor(0xC826)	<< 8) | (HI541ReadCmosSensor(0xC827));
	y_end_backup = (HI541ReadCmosSensor(0xC83A) 	<< 8) | (HI541ReadCmosSensor(0xC83B));

	// save to sensor register
     HI541WriteCmosSensor(0xffff, 0x0020);
     HI541WriteCmosSensor(0x0638, x_start_backup);
     HI541WriteCmosSensor(0x063A, x_end_backup);
     HI541WriteCmosSensor(0x063C, y_start_backup);
     HI541WriteCmosSensor(0x063E, y_end_backup);

	// re-mapping SK hynix
     if(x<4)
     {
               x_view=4;
     }
     else if(x>76)
     {
               x_view=76;
     }
     else
     {
               x_view= x;
     }
     if(y<4)
     {
               y_view=4;
     }
     else if(y>56)
     {
               y_view=56;
     }
     else
     {
               y_view= y;
     }
	 
    // calc Window Ratio
    x_tmp_size = x_view * x_ratio;
    y_tmp_size = y_view * y_ratio;
     
    x_start_h = (x_tmp_size - (x_ratio*4)) >> 8;
    x_start_l = (x_tmp_size - (x_ratio*4)) & 0xff;
    x_end_h 	= (x_tmp_size + (x_ratio*4)) >> 8;
    x_end_l	= (x_tmp_size + (x_ratio*4)) & 0xff;

    y_start_h = (y_tmp_size - (y_ratio*4)) >> 8;
    y_start_l = (y_tmp_size - (y_ratio*4)) & 0xff;
    y_end_h 	= (y_tmp_size + (y_ratio*4)) >> 8;
    y_end_l	= (y_tmp_size + (y_ratio*4)) & 0xff;

    HI541WriteCmosSensor(0xffff,0x0040);
    HI541WriteCmosSensor(0xC826, (y_start_l << 8)	| y_start_h );
    HI541WriteCmosSensor(0xC83A, (y_end_l << 8)	| y_end_h );
    HI541WriteCmosSensor(0xC84E, (x_start_l << 8)	| x_start_h );
    HI541WriteCmosSensor(0xC862, (x_end_l << 8)	| x_end_h );

	SENSORDB("[HI541]AF x_view=%d,y_view=%d\n",x_view, y_view);

	// AF command
	HI541_FOCUS_AFC_Constant_Focus();

	SENSORDB("[HI541]exit HI541_FOCUS_OVT_AFC_Touch_AF function:\n ");
	


	 //SENSORDB("[HI541]AF x_tmp1=%d,y_tmp1=%d\n",x_tmp, y_tmp);
    // SENSORDB("[HI541]exit HI541_FOCUS_OVT_AFC_Touch_AF function:\n ");
	 #endif
	 //=====================================
	 //============Fail AF   add Rampart============
#if 0  // no need 
	SENSORDB("HI541_FOCUS_Touch AF Fail is Constant star Rampart\n");
	//DISP_NAME = "AF_AutoParaList"
	//BEGIN
	HI541WriteCmosSensor(0xffff, 0x0020);
	HI541WriteCmosSensor(0x05b0, 0x8616);	//AF Mode Select
	HI541WriteCmosSensor(0xffff, 0x0040);
	HI541WriteCmosSensor(0x3824, 0x0500);	
	HI541WriteCmosSensor(0x3826, 0xAF10);	//Do AF
	SENSORDB("HI541_FOCUS_Touch AF Fail is Constant end\n");
	 //=====================================
	 
	 //SENSORDB("[HI541]AF x_tmp1=%d,y_tmp1=%d\n",x_tmp, y_tmp);
    // SENSORDB("[HI541]exit HI541_FOCUS_OVT_AFC_Touch_AF function:\n ");
#endif
}

static void Hi541_FOCUS_Set_AF_Window(UINT32 zone_addr)
{//update global zone
	  SENSORDB("[HI541]enter HI541_FOCUS_Set_AF_Window function:\n ");
	  UINT32 FD_XS;
	  UINT32 FD_YS;   
	  UINT32 x0, y0, x1, y1;
	  UINT32 pvx0, pvy0, pvx1, pvy1;
	  UINT32 linenum, rownum;
	  UINT32 AF_pvx, AF_pvy;
	  UINT32* zone = (UINT32*)zone_addr;
	  x0 = *zone;
	  y0 = *(zone + 1);
	  x1 = *(zone + 2);
	  y1 = *(zone + 3);   
	  FD_XS = *(zone + 4);
	  FD_YS = *(zone + 5);
	  
	  SENSORDB("AE x0=%d,y0=%d,x1=%d,y1=%d,FD_XS=%d,FD_YS=%d\n",
	  x0, y0, x1, y1, FD_XS, FD_YS);  
	  mapMiddlewaresizePointToPreviewsizePoint(x0,y0,FD_XS,FD_YS,&pvx0, &pvy0, PRV_W, PRV_H);
	  mapMiddlewaresizePointToPreviewsizePoint(x1,y1,FD_XS,FD_YS,&pvx1, &pvy1, PRV_W, PRV_H);  
	  SENSORDB("[HI541]AF pvx0=%d,pvy0=%d\n",pvx0, pvy0);
	  SENSORDB("[HI541]AF pvx0=%d,pvy0=%d\n",pvx1, pvy1);
	  AF_pvx =(pvx0+pvx1)/32;
	  AF_pvy =(pvy0+pvy1)/32;
	  SENSORDB("[HI541]AF AF_pvx=%d,AF_pvy=%d\n",AF_pvx, AF_pvy);
	 HI541_FOCUS_OVT_AFC_Touch_AF(AF_pvx ,AF_pvy);
	  
}
static void HI541_FOCUS_Get_AF_Macro(UINT32 *pFeatureReturnPara32)
{
	HI541WriteCmosSensor(0xffff,0x0040);
	HI541WriteCmosSensor(0x3824,0x1400);				// Manual AF Enable
	HI541WriteCmosSensor(0x3826,0x0100);
	HI541WriteCmosSensor(0x3824, 0xc02f);
	HI541WriteCmosSensor(0x3826, 0x0200);
    *pFeatureReturnPara32 = 0;
}
static void HI541_FOCUS_Get_AF_Inf(UINT32 * pFeatureReturnPara32)
{
	HI541WriteCmosSensor(0xffff,0x0040);
	HI541WriteCmosSensor(0x3824,0x1400);				// Manual AF Enable
	HI541WriteCmosSensor(0x3826,0x0100);
	HI541WriteCmosSensor(0x3824, 0xa00f);
	HI541WriteCmosSensor(0x3826, 0x0200);
    *pFeatureReturnPara32 = 0;
}

static void HI541_FOCUS_OVT_AFC_Get_AF_Status(UINT32 * pFeatureReturnPara32)
{
    UINT32 af_status = 0;
	UINT32 caf_status = 0;
	UINT16 x_start, x_end, y_start, y_end;
	
	*pFeatureReturnPara32 = SENSOR_AF_IDLE;
	
	HI541WriteCmosSensor(0xffff,0x0020);
	af_status = HI541ReadCmosSensor(0x05d4);
	af_status &=0x1;
	if(af_status == 0x1)
	{
        *pFeatureReturnPara32 = SENSOR_AF_FOCUSED;

		// added AF status by SK hynix

		if(TOUCHAF_FLAG == 1)
		{
			caf_status = HI541ReadCmosSensor(0x05D6);

			if (caf_status & 0x40)
			{
				// load origianl window region
			    HI541WriteCmosSensor(0xffff, 0x0020);
				x_start = (HI541ReadCmosSensor(0x0638)	<< 8) | (HI541ReadCmosSensor(0x0639));
				x_end	= (HI541ReadCmosSensor(0x063A) 	<< 8) | (HI541ReadCmosSensor(0x063B));	 
				y_start = (HI541ReadCmosSensor(0x063C)	<< 8) | (HI541ReadCmosSensor(0x063D));
				y_end	= (HI541ReadCmosSensor(0x063E) 	<< 8) | (HI541ReadCmosSensor(0x063F));

				// write to sensor register
			     HI541WriteCmosSensor(0xffff, 0x0040);
			     HI541WriteCmosSensor(0xC84E, x_start);
			     HI541WriteCmosSensor(0xC862, x_end);
			     HI541WriteCmosSensor(0xC826, y_start);
			     HI541WriteCmosSensor(0xC83A, y_end);

				 TOUCHAF_FLAG = 0;
			}
		}

	}
	else
	{
          *pFeatureReturnPara32 = SENSOR_AF_FOCUSING;
	}

    SENSORDB("[HI541]HI541_FOCUS_OVT_AFC_Get_AF_Status af_status = %d\n",af_status);
}


#if 1
#define Ratio_Hi541 4
kal_uint16 HI541_TouchAE_RegtoSensor(UINT32 *startX, UINT32 *startY,UINT32  *endX, UINT32  *endY)
{
	UINT32 target_X, target_Y;
	UINT32 tmpX, tmpY;
	kal_uint16 tmp_PreYmean;
	target_X = (((*startX)/80)<<8)|((*endX)/80);
	target_Y = (((*startY)/60)<<8)|((*endY)/60);
	HI541WriteCmosSensor(0xffff, 0x0020);
	HI541WriteCmosSensor(0x039C, 0xff11);
	HI541WriteCmosSensor(0x03AA, 0x1f0a);
	HI541WriteCmosSensor(0x03A6, (unsigned short)target_X);
	HI541WriteCmosSensor(0x03A8, (unsigned short)target_Y);
	HI541WriteCmosSensor(0xffff, 0x0040);
	tmp_PreYmean=HI541ReadCmosSensor(0x032a);
	SENSORDB("SENONG: %d,%d,%d,%d,%d,%d\n",target_X,target_Y,*startX,*endX,*startY,*endY);
    return tmp_PreYmean;
}
void HI541_TouchAE_TurnOff(kal_uint16 PreYmean)
{
	if(abs(PreYmean-HI541ReadCmosSensor(0x032a))>10)
	{
		HI541WriteCmosSensor(0xffff, 0x0020);
		HI541WriteCmosSensor(0x039C, 0xff01);
	}
}
#endif

void HI541_FOCUS_Set_AE_Window(UINT32 zone_addr)
{//update global zone
   SENSORDB("[HI541]enter HI541_FOCUS_Set_AE_Window function\n");
    //input:
    UINT32 FD_XS;
    UINT32 FD_YS;	
    UINT32 x0, y0, x1, y1;
    UINT32 pvx0, pvy0, pvx1, pvy1;
    UINT32 linenum, rownum;
    UINT32 rightbottomlinenum,rightbottomrownum;
    UINT32 leftuplinenum,leftuprownum;
    UINT32* zone = (UINT32*)zone_addr;
    kal_uint16 PreYmean;
	
    x0 = *zone;
    y0 = *(zone + 1);
    x1 = *(zone + 2);
    y1 = *(zone + 3);	
    FD_XS = *(zone + 4);
    FD_YS = *(zone + 5);

    SENSORDB("AE x0=%d,y0=%d,x1=%d,y1=%d,FD_XS=%d,FD_YS=%d\n",
    x0, y0, x1, y1, FD_XS, FD_YS);	
    
    //print_sensor_ae_section();
    //print_AE_section();	

    //1.transfer points to preview size
    //UINT32 pvx0, pvy0, pvx1, pvy1;
    mapMiddlewaresizePointToPreviewsizePoint(x0,y0,FD_XS,FD_YS,&pvx0, &pvy0, PRV_W, PRV_H);
    mapMiddlewaresizePointToPreviewsizePoint(x1,y1,FD_XS,FD_YS,&pvx1, &pvy1, PRV_W, PRV_H);
    
    //2.sensor AE line and row coordinate
    calcLine();
    calcRow();

    //3.calc left up point to section
    //UINT32 linenum, rownum;
    calcPointsAELineRowCoordinate(pvx0,pvy0,&linenum,&rownum);    
    //UINT32 leftuplinenum,leftuprownum;
    mapCoordinate(linenum, rownum, &leftuplinenum, &leftuprownum);
    //SENSORDB("leftuplinenum=%d,leftuprownum=%d\n",leftuplinenum,leftuprownum);

    //4.calc right bottom point to section
    calcPointsAELineRowCoordinate(pvx1,pvy1,&linenum,&rownum);    
    //UINT32 rightbottomlinenum,rightbottomrownum;
    mapCoordinate(linenum, rownum, &rightbottomlinenum, &rightbottomrownum);
    //SENSORDB("rightbottomlinenum=%d,rightbottomrownum=%d\n",rightbottomlinenum,rightbottomrownum);

    //5.update global section array
    mapRectToAE_2_ARRAY(leftuplinenum, leftuprownum, rightbottomlinenum, rightbottomrownum);
    //print_AE_section();

    //6.write to reg
    mapAE_2_ARRAY_To_AE_1_ARRAY();
    //printAE_1_ARRAY();
    printAE_2_ARRAY();
	
#if 1 //need open Touch_AE if 1
	PreYmean = HI541_TouchAE_RegtoSensor(&pvx0, &pvy0, &pvx1, &pvy1);
	HI541_TouchAE_TurnOff(PreYmean);
#endif
	//HI541_FOCUS_AFC_Single_Focus();//rampart
//	HI541_FOCUS_AFC_Constant_Focus();//rampart test
//    writeAEReg();//just to test 20140813 liuzhiyou
	// SENSORDB("[OV5645MIPI]exit OV5645_FOCUS_Set_AE_Window function\n");
}
static void HI541_FOCUS_AFC_Cancel_Focus()
{
#if 0 			// modified SeongYW
	HI541WriteCmosSensor(0xffff, 0x0040);
	HI541WriteCmosSensor(0x3824, 0x0100);	//AF Mode Select
	HI541WriteCmosSensor(0x3826, 0xAF10);	//Do AF
#else
	//HI541WriteCmosSensor(0xffff, 0x0020);
	//HI541WriteCmosSensor(0x001c, 0x1000);	//AF Kill
#endif
}
void HI541_get_AEAWB_lock(UINT32 *pAElockRet32, UINT32 *pAWBlockRet32)
{
	SENSORDB("[OV5645MIPI]enter OV5645MIPI_get_AEAWB_lock function:\n ");
	*pAElockRet32 =1;
	*pAWBlockRet32=1;
	SENSORDB("[OV5645MIPI]OV5645MIPI_get_AEAWB_lock,AE=%d,AWB=%d\n",*pAElockRet32,*pAWBlockRet32);
	SENSORDB("[OV5645MIPI]exit OV5645MIPI_get_AEAWB_lock function:\n ");
}


UINT32 HI541_get_Shutterspeed(void)
{
    kal_uint32 exposuretime;
	kal_uint16 a,b,c,d;

	HI541WriteCmosSensor(0xffff,0x0020); //page mode 0x2000
	a = HI541ReadCmosSensor(0x1C48); //Exp_L
	b = HI541ReadCmosSensor(0x1C49); //Exp_ML
	c = HI541ReadCmosSensor(0x1C4A); //Exp_M
	d = HI541ReadCmosSensor(0x1C4B); //Exp_H

	exposuretime = ((d<<24)|(c<<16)|(b<<8)|a)/21;
	
    SENSORDB("[HI541MIPI] exposuretime = %d,d=0x%x,c=0x%x,b=0x%x,a=0x%x,\n",exposuretime,d,c,b,a);
	return exposuretime;
		
}

kal_uint8 HI541_get_iso(void)
{
    kal_uint8 iso_temp = 0;
	kal_uint16 regData = 0; 

	HI541WriteCmosSensor(0xffff,0x0020); //page mode 0x2000
	regData = HI541ReadCmosSensor(0x1C45);

	if(regData < 0x40)
	{
        iso_temp = AE_ISO_100;
	}else if (regData < 0x80)
	{
        iso_temp = AE_ISO_200;
	}else
	{
        iso_temp = AE_ISO_400;
	}
		
    return iso_temp;
}


void HI541InitPara(void)
{
	//SENSORDB("enter the initPara function");
	spin_lock(&HI541_drv_lock);
  HI541Status.NightMode = KAL_FALSE;
  HI541Status.ZoomFactor = 0;
  HI541Status.Banding = AE_FLICKER_MODE_50HZ;
  HI541Status.PvShutter = 0x9eb10;
  HI541Status.MaxFrameRate = HI541_MAX_FPS;
  HI541Status.MiniFrameRate = HI541_FPS(10);
  HI541Status.PvDummyPixels = 376;
  HI541Status.PvDummyLines = 20;
  HI541Status.CapDummyPixels = 376;
  HI541Status.CapDummyLines = 20; /* 10 FPS, 104 for 9.6 FPS*/
  HI541Status.PvOpClk = 26;
  HI541Status.CapOpClk = 26;  
  HI541Status.VDOCTL2 = 0x90;
  HI541Status.ISPCTL3 = 0x30;
  HI541Status.AECTL1 = 0x9c;
  HI541Status.AWBCTL1 = 0xe9;

  HI541CurrentStatus.iNightMode = 0xFFFF;
  HI541CurrentStatus.iWB = AWB_MODE_AUTO;
  HI541CurrentStatus.iEffect = MEFFECT_OFF;
  HI541CurrentStatus.iBanding = AE_FLICKER_MODE_50HZ;
  HI541CurrentStatus.iEV = AE_EV_COMP_00;
  HI541CurrentStatus.iMirror = IMAGE_NORMAL;
  HI541CurrentStatus.iFrameRate = 0;//No Fix FrameRate
  HI541Status.VideoMode=KAL_FALSE;
  	spin_unlock(&HI541_drv_lock);
}

/*************************************************************************
* FUNCTION
*  HI541SetMirror
*
* DESCRIPTION
*  This function mirror, flip or mirror & flip the sensor output image.
*
*  IMPORTANT NOTICE: For some sensor, it need re-set the output order Y1CbY2Cr after
*  mirror or flip.
*
* PARAMETERS
*  1. kal_uint16 : horizontal mirror or vertical flip direction.
*
* RETURNS
*  None
*
*************************************************************************/
static void HI541SetMirror(kal_uint16 ImageMirror)
{
	spin_lock(&HI541_drv_lock);
  HI541Status.VDOCTL2 &= 0xfc;   //10010000	11111100=> 10010000
  spin_unlock(&HI541_drv_lock);
  switch (ImageMirror)
  {
    case IMAGE_H_MIRROR:
		spin_lock(&HI541_drv_lock);
      HI541Status.VDOCTL2 |= 0x01;
	  spin_unlock(&HI541_drv_lock);
      break;
    case IMAGE_V_MIRROR:
		spin_lock(&HI541_drv_lock);
      HI541Status.VDOCTL2 |= 0x02; 
	  spin_unlock(&HI541_drv_lock);
      break;
    case IMAGE_HV_MIRROR:
		spin_lock(&HI541_drv_lock);
      HI541Status.VDOCTL2 |= 0x03;
	  spin_unlock(&HI541_drv_lock);
      break;
    case IMAGE_NORMAL:
    default:
		spin_lock(&HI541_drv_lock);
      HI541Status.VDOCTL2 |= 0x00; 
	  spin_unlock(&HI541_drv_lock);
  }

}

static void HI541SetAeMode(kal_bool AeEnable)
{
  //SENSORDB("[HI541]HI541SetAeMode AeEnable:%d;\n",AeEnable);

  if (AeEnable == KAL_TRUE)
  {
  	spin_lock(&HI541_drv_lock);
    HI541Status.AECTL1 |= 0x80;
	spin_unlock(&HI541_drv_lock);
  }
  else
  {
  	spin_lock(&HI541_drv_lock);
    HI541Status.AECTL1 &= (~0x80);
	spin_unlock(&HI541_drv_lock);
  }
 
}


static void HI541SetAwbMode(kal_bool AwbEnable)
{
  //SENSORDB("[HI541]HI541SetAwbMode AwbEnable:%d;\n",AwbEnable);
  if (AwbEnable == KAL_TRUE)
  {
  	spin_lock(&HI541_drv_lock);
    HI541Status.AWBCTL1 |= 0x80;
	spin_unlock(&HI541_drv_lock);
  }
  else
  {
  	spin_lock(&HI541_drv_lock);
    HI541Status.AWBCTL1 &= (~0x80);
	spin_unlock(&HI541_drv_lock);
  }

}


static UINT32 ISDarkMode()
{
     kal_uint32 exptime,expmax;
	
	 //SENSORDB("[HI541] exptime:%d;expmax:%d\n",exptime,expmax);

	 if(exptime<expmax)
	 	return FALSE;
	 else 
	 	return TRUE;
	 
}


BOOL HI541SetWb(UINT16 Para)
{
	

	//SENSORDB("[HI541_Debug]HI541SetWb Para:%d;\n",Para);
	switch (Para)
	{
	case AWB_MODE_OFF:
		//HI541SetAwbMode(KAL_FALSE);
		break;                     
	case AWB_MODE_AUTO:
		
//AWB_Auto
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0a0c, 0x1010); //Rmin,Bmin
HI541WriteCmosSensor(0x0a0e, 0xc0c0); //Rmax,Bmax 
HI541WriteCmosSensor(0xffff, 0x0040);
			   

		break;

	case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy

//D65
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0a0c, 0x6650); //Rmin,Bmin
HI541WriteCmosSensor(0x0a0e, 0x705A); //Rmax,Bmax 
HI541WriteCmosSensor(0xffff, 0x0040);
		
		break;

	case AWB_MODE_DAYLIGHT: //sunny

//D55
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0a0c, 0x6359); //Rmin,Bmin
HI541WriteCmosSensor(0x0a0e, 0x6d64); //Rmax,Bmax 
HI541WriteCmosSensor(0xffff, 0x0040);

		
		break;
		case AWB_MODE_INCANDESCENT: //office
		
//A
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0a0c, 0x3b9b); //Rmin,Bmin
HI541WriteCmosSensor(0x0a0e, 0x45a5); //Rmax,Bmax 
HI541WriteCmosSensor(0xffff, 0x0040);
		
		break;
		case AWB_MODE_TUNGSTEN: //home
		
//A2
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0a0c, 0x3b9b); //Rmin,Bmin
HI541WriteCmosSensor(0x0a0e, 0x45a5); //Rmax,Bmax 
HI541WriteCmosSensor(0xffff, 0x0040);
		
		break;
		case AWB_MODE_FLUORESCENT:
			
//TL84
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0a0c, 0x457b); //Rmin,Bmin
HI541WriteCmosSensor(0x0a0e, 0x4f85); //Rmax,Bmax 
HI541WriteCmosSensor(0xffff, 0x0040);
	
	
	default:
		return KAL_FALSE;
	}

    spin_lock(&HI541_drv_lock);
    HI541CurrentStatus.iWB = Para;
    spin_unlock(&HI541_drv_lock);
    
	return KAL_TRUE;      
} /* HI541SetWb */


/*************************************************************************
* FUNCTION
* HI541NightMode
*
* DESCRIPTION
* This function night mode of HI541.
*
* PARAMETERS
* none
*
* RETURNS
* None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void HI541NightMode(kal_bool Enable)
{
	kal_uint32 EXPMAX, EXPTIME, BLC_TIME_TH_ONOFF;
	kal_uint32 LineLength,BandingValue;
	//SENSORDB("[HI541]HI541NightMode Enable:%d;\n",Enable);
	
	if (zsd==1){
		return ;
		}else{

	HI541SetAeMode(KAL_TRUE);
	
	if (Enable)
	{
		HI541Status.NightMode=TRUE;

		if(ISDarkMode())//dark condition
		{
			
    
 		
 
 
 

		}
		else //Non-Dark Condition
		{
			
		}

		
//DISP_NAME = "Pre_Night 15~5"
//DISP_WIDTH = 1280	
//DISP_HEIGHT = 960

//BEGIN

///////////////////////////////////////////
// Preview SXGA SUB Setting
// 1280x960@15fps, MIPI_YUV422, 1/2 Digital Scalex1/2 Analog Sub-sampling
///////////////////////////////////////////
//I2C_ID = 0x40
//I2C_BYTE  = 0x22


//================================
//========= SYSTEM Start =========
//================================
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x0100,0x0000);	//streaming disable
HI541WriteCmosSensor(0x5402,0x0004);	//tg_ctl3
HI541WriteCmosSensor(0x0900,0x0002);	//binning
HI541WriteCmosSensor(0x0340,0xe907);   //frame_length_lines  //15 fps
HI541WriteCmosSensor(0x0342,0xca0a);//line_length_pck
HI541WriteCmosSensor(0x0344,0x1000);//x_addr_start
HI541WriteCmosSensor(0x0346,0x0e00);//y_addr_start
HI541WriteCmosSensor(0x0348,0x570a);//x_addr_end
HI541WriteCmosSensor(0x034a,0xc907);//y_addr_end
HI541WriteCmosSensor(0x034c,0x0005);//x_output_size
HI541WriteCmosSensor(0x034e,0xc003);//y_output_size
HI541WriteCmosSensor(0x0380,0x0100);//x_even_inc
HI541WriteCmosSensor(0x0382,0x0100);//x_odd_inc
HI541WriteCmosSensor(0x0384,0x0100);//y_even_inc
HI541WriteCmosSensor(0x0386,0x0300);//y_odd_inc
HI541WriteCmosSensor(0x540e,0x3677);//JH.LEE_140904
HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x0384,0x802c);//L
HI541WriteCmosSensor(0x0386,0x8000);//H exposure max 100 5fps
HI541WriteCmosSensor(0x0388,0x802c);//L
HI541WriteCmosSensor(0x038a,0x8000);//H exposure max 120 5fps
//================================
//========= SYSTEM End ===========
//================================


		}
		else //Normal Mode
		{

//================================
//========= SYSTEM Start =========
//================================
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x0100,0x0000);	//streaming disable
HI541WriteCmosSensor(0x5402,0x0004);	//tg_ctl3
HI541WriteCmosSensor(0x0900,0x0002);	//binning
HI541WriteCmosSensor(0x0340,0xf503);//frame_length_lines  //16 fps
HI541WriteCmosSensor(0x0342,0xca0a);//line_length_pck
HI541WriteCmosSensor(0x0344,0x1000);//x_addr_start
HI541WriteCmosSensor(0x0346,0x0e00);//y_addr_start
HI541WriteCmosSensor(0x0348,0x570a);//x_addr_end
HI541WriteCmosSensor(0x034a,0xc907);//y_addr_end
HI541WriteCmosSensor(0x034c,0x0005);//x_output_size
HI541WriteCmosSensor(0x034e,0xc003);//y_output_size
HI541WriteCmosSensor(0x0380,0x0100);//x_even_inc
HI541WriteCmosSensor(0x0382,0x0100);//x_odd_inc
HI541WriteCmosSensor(0x0384,0x0100);//y_even_inc
HI541WriteCmosSensor(0x0386,0x0300);//y_odd_inc

HI541WriteCmosSensor(0x540e,0x3677);//JH.LEE_140904
HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x0384,0x80e7);//L
HI541WriteCmosSensor(0x0386,0x4c00);//H exposure max 100 8.33 fps
HI541WriteCmosSensor(0x0388,0x80e7);//L
HI541WriteCmosSensor(0x038a,0x4c00);//H exposure max 120 8.57 fps
//================================
//========= SYSTEM End ===========
//================================

		};
//================================
//======= ISP HW Start ===========
//================================
HI541WriteCmosSensor(0xffff,0x0040);

// ISP enable
HI541WriteCmosSensor(0x4830,0xfeef);
HI541WriteCmosSensor(0x4832,0x7f7a); //AF enable
HI541WriteCmosSensor(0x4834,0x0401);
  
// BScaler
HI541WriteCmosSensor(0x6000,0x2800);//mode_byrsc1
HI541WriteCmosSensor(0x6002,0x2000);//byrsc_fifo_delay

//Yscaler 1280x960
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0xa800,0x2000); //mode_zoom1
HI541WriteCmosSensor(0xa804,0x0005); //zoom_dst_width
HI541WriteCmosSensor(0xa806,0xc003); //zoom_dst_height
		HI541WriteCmosSensor(0xa810,0x1308); //zoom_hor_step
HI541WriteCmosSensor(0xa812,0x1908); //zoom_ver_step
		HI541WriteCmosSensor(0xa814,0x3E02); //zoom_hor_step_remain
HI541WriteCmosSensor(0xa816,0x9909); //zoom_ver_step_remain
		HI541WriteCmosSensor(0xa818,0x1000); //zoom_fifo_delay
HI541WriteCmosSensor(0xa824,0x0f00); //zoom_intpol1
HI541WriteCmosSensor(0xa826,0x0000); //zoom_intpol3

// Iridix
HI541WriteCmosSensor(0x8404,0x1405);//hdr_frame_width
HI541WriteCmosSensor(0x8406,0xd003);//hdr_frame_height
//================================
//======= ISP HW End   ===========
//================================

//================================
//=========== SSD Start ==========
//================================
//Hardware SSD SET
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0xc414,0x0000);//ae_patch_xy_offset
HI541WriteCmosSensor(0xc416,0xa03c);//ae_patch_xy_w_h x 16
HI541WriteCmosSensor(0xc418,0x0404);//awb_size_xy_offset
HI541WriteCmosSensor(0xc41a,0xa034);//awb_size_xy_w_h x 16

//Firmware SSD SET
HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x1fe4,0x0404);//awb_size_y_offset
HI541WriteCmosSensor(0x1fe6,0x200a);//awb_size_xy_w_h x 16
HI541WriteCmosSensor(0x1fe8,0x0404);//ae_size_xy_offset
HI541WriteCmosSensor(0x1fea,0xa034);//ae_size_xy_w_h x 16

//================================
//=========== SSD End   ==========
//================================

//=====================================================
//=========== AF Filter Start - preview    ============
//=====================================================


//DISP_NAME = "AF_InitParaList"
//BEGIN
//=====================================================
//=========== AF Filter Start - preview    ============
//=====================================================
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0xC800, 0x4840);	// sobel& spmd  
HI541WriteCmosSensor(0xc802, 0x0044);
HI541WriteCmosSensor(0xc804, 0x0044);
HI541WriteCmosSensor(0xC806, 0xffC3); // af_ctl8 // manual window setting + x_half[6] 
HI541WriteCmosSensor(0xC808, 0x8000); // af_ctl8 // manual window setting + x_half[6] 

#if 0
//========================================
//=========== F/W AF Start ===============
//========================================
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x003E, 0x8080);			// AWB Lock
HI541WriteCmosSensor(0x0020, 0xff00); 			// IICM Delay

HI541WriteCmosSensor(0x05B4, 0x0af6); 			//0x05b5 : f8(8), f7(9), f6(10),  EC(20) macro shift 2014.05.27
HI541WriteCmosSensor(0x05B6, 0x3219);			// Intelligent Step Size = 20

HI541WriteCmosSensor(0x05B8, 0x0819);			// Fine Scan Step Size
HI541WriteCmosSensor(0x05Ba, 0x0003);			// Falling Threshold Number
HI541WriteCmosSensor(0x05C0, 0x1100);			// Skip Frame	 
HI541WriteCmosSensor(0x05C8, 0x6414);			// softlanding step / first scan outdoor/ 	 
HI541WriteCmosSensor(0x05CA, 0x1414);			// first scan indoor / first scan dark

HI541WriteCmosSensor(0x05F4, 0x0200);			// Ae STD thresholg
HI541WriteCmosSensor(0x062A, 0x8000);			// min duty		// 250
HI541WriteCmosSensor(0x062C, 0xBC02);			// max duty
HI541WriteCmosSensor(0x062e, 0x9001);			// marco min duty

HI541WriteCmosSensor(0x0668, 0x4b4b);			// Uniform Th (rev BE)
HI541WriteCmosSensor(0x066a, 0x4001);			// Hyper Position (rev BE)

HI541WriteCmosSensor(0x0680, 0x1111);			// weight
HI541WriteCmosSensor(0x0682, 0x1111);			// weight
HI541WriteCmosSensor(0x0684, 0x2828);			// center weight  

//20140805
HI541WriteCmosSensor(0x05CE, 0x0003);        // MSB Masking set
HI541WriteCmosSensor(0x05D0, 0xFF00);        // LSB Masking set / SlewRate set
//HI541WriteCmosSensor(0x069C, 0x0008);        // MSB / LSB shift set


HI541WriteCmosSensor(0x0714, 0xC000);			// outdoor step LUT 1
HI541WriteCmosSensor(0x0716, 0xD000);			// outdoor step LUT 2	
HI541WriteCmosSensor(0x0718, 0xE000);			// outdoor step LUT 3	
HI541WriteCmosSensor(0x071A, 0xF000);			// outdoor step LUT 4	
HI541WriteCmosSensor(0x071C, 0x0001);			// outdoor step LUT 5	
HI541WriteCmosSensor(0x071E, 0x1001);			// outdoor step LUT 6	
HI541WriteCmosSensor(0x0720, 0x2001);			// outdoor step LUT 7	
HI541WriteCmosSensor(0x0722, 0x3001);			// outdoor step LUT 8	
HI541WriteCmosSensor(0x0724, 0x4001);			// outdoor step LUT 9	
HI541WriteCmosSensor(0x0726, 0x5001);			// outdoor step LUT 10	
HI541WriteCmosSensor(0x0728, 0x6001);			// outdoor step LUT 11	   
HI541WriteCmosSensor(0x072a, 0x7001);			// outdoor step LUT 12
HI541WriteCmosSensor(0x072c, 0x7801);			// outdoor step LUT 13
HI541WriteCmosSensor(0x072e, 0x8001);			// outdoor step LUT 14
HI541WriteCmosSensor(0x0730, 0x8801);			// outdoor step LUT 15
HI541WriteCmosSensor(0x0732, 0x9001);			// outdoor step LUT 16
HI541WriteCmosSensor(0x0734, 0xA001);			// outdoor step LUT 17
HI541WriteCmosSensor(0x0736, 0xB001);			// outdoor step LUT 18
HI541WriteCmosSensor(0x0738, 0xC001);			// outdoor step LUT 19
HI541WriteCmosSensor(0x073A, 0xC001);			// outdoor step LUT 20
HI541WriteCmosSensor(0x073C, 0xC001);			// outdoor step LUT 20


HI541WriteCmosSensor(0x0748, 0xC000);			// indoor step LUT 1
HI541WriteCmosSensor(0x074A, 0xD000);			// indoor step LUT 2	
HI541WriteCmosSensor(0x074C, 0xE000);			// indoor step LUT 3	
HI541WriteCmosSensor(0x074E, 0xF000);			// indoor step LUT 4	
HI541WriteCmosSensor(0x0750, 0x0001);			// indoor step LUT 5	
HI541WriteCmosSensor(0x0752, 0x1001);			// indoor step LUT 6	
HI541WriteCmosSensor(0x0754, 0x2001);			// indoor step LUT 7	
HI541WriteCmosSensor(0x0756, 0x3001);			// indoor step LUT 8	
HI541WriteCmosSensor(0x0758, 0x4001);			// indoor step LUT 9	
HI541WriteCmosSensor(0x075A, 0x5001);			// indoor step LUT 10	
HI541WriteCmosSensor(0x075C, 0x6001);
HI541WriteCmosSensor(0x075E, 0x7001);			// 
HI541WriteCmosSensor(0x0760, 0x7801);			//
HI541WriteCmosSensor(0x0762, 0x8001);			//
HI541WriteCmosSensor(0x0764, 0x8801);			//
HI541WriteCmosSensor(0x0766, 0x9001);			//
HI541WriteCmosSensor(0x0768, 0xA001);			//
HI541WriteCmosSensor(0x076A, 0xB001);			//
HI541WriteCmosSensor(0x076C, 0xC001);			//
HI541WriteCmosSensor(0x076E, 0xC001);			//
HI541WriteCmosSensor(0x0770, 0xC001);			//


HI541WriteCmosSensor(0x077C, 0xC000);			// dark step LUT 1
HI541WriteCmosSensor(0x077E, 0xD000);			// dark step LUT 2	
HI541WriteCmosSensor(0x0780, 0xE000);			// dark step LUT 3	
HI541WriteCmosSensor(0x0782, 0xF000);			// dark step LUT 4	
HI541WriteCmosSensor(0x0784, 0x0001);			// dark step LUT 5	
HI541WriteCmosSensor(0x0786, 0x1001);			// dark step LUT 6	
HI541WriteCmosSensor(0x0788, 0x2001);			// dark step LUT 7	
HI541WriteCmosSensor(0x078A, 0x3001);			// dark step LUT 8	
HI541WriteCmosSensor(0x078C, 0x4001);			// dark step LUT 9	
HI541WriteCmosSensor(0x078E, 0x5001);			// dark step LUT 10	
HI541WriteCmosSensor(0x0790, 0x6001);
HI541WriteCmosSensor(0x0792, 0x7001);
HI541WriteCmosSensor(0x0794, 0x7801);
HI541WriteCmosSensor(0x0796, 0x8001);
HI541WriteCmosSensor(0x0798, 0x8801);
HI541WriteCmosSensor(0x079A, 0x9001);
HI541WriteCmosSensor(0x079C, 0xA001);
HI541WriteCmosSensor(0x079E, 0xB001);
HI541WriteCmosSensor(0x07A0, 0xC001);
HI541WriteCmosSensor(0x07A2, 0xC001);
HI541WriteCmosSensor(0x07A4, 0xC001);



// added to Improve speed continuous AF
HI541WriteCmosSensor(0x05EA, 0x0008);  
HI541WriteCmosSensor(0x05EC, 0x0800);  
HI541WriteCmosSensor(0x05F0, 0x0408);  
HI541WriteCmosSensor(0x05F2, 0x0800);                  
HI541WriteCmosSensor(0x05F6, 0x0828);                  

HI541WriteCmosSensor(0x05C0, 0x1100);			// Skip Frame	
HI541WriteCmosSensor(0x0628, 0x1E15);  			// Slpope Step / Delay 

//========================================
//=========== F/W AF End  ================
//========================================
#endif
		HI541_FOCUS_AFC_Init();
//========================================
//========  Constant_Focus  Star add Rampart ========
//========================================
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x05b0, 0x8616);	//AF Mode Select
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0x3824, 0x0500);	
HI541WriteCmosSensor(0x3826, 0xAF10);	//Do AF
//========================================
//========  Constant_Focus  End  Rampart   ==========
//========================================

//================================
//======== FINDBAND Start ========
//================================
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0xC002,0x4608);
HI541WriteCmosSensor(0xC004,0x4005);

//================================
//======== FINDBAND End   ========
//================================

//================================
//========= SYSTEM Start==========
//================================
// (4) Skhynix JHKIM - duplicated command
//HI541WriteCmosSensor(0xffff,0x0040);
//HI541WriteCmosSensor(0x3824,0x0000);
//HI541WriteCmosSensor(0x3826,0x5a10);//MCU preview hif cmd
//================================
//========= SYSTEM End  ==========
//================================

//================================
//========= AE min Start==========
//================================
HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x0310,0x282b);
HI541WriteCmosSensor(0x0312,0x0000);//

HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x548a,0x0004);//

//================================
//========= AE min End  ==========
//================================

//================================
//====Return to Preview Start=====
//================================
// Para
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x3824,0x0000);
// Command
HI541WriteCmosSensor(0x3826,0x5a10);
//================================
//====Return to Preview End=======
//================================

HI541WriteCmosSensor(0x0100,0x0100);	//streaming enable

//END
//[END]


		

	LineLength = HI541_PV_PERIOD_PIXEL_NUMS + HI541Status.PvDummyPixels;

			}

} /* HI541NightMode */
 /* HI541NightMode */


/*************************************************************************
* FUNCTION
* HI541Open
*
* DESCRIPTION
* this function initialize the registers of CMOS sensor
*
* PARAMETERS
* none
*
* RETURNS
*  none
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 HI541Open(void)
{
	kal_uint16 SensorId = 0;
	kal_uint8 i;
	////1 software reset sensor and wait (to sensor)
	
    kal_uint16 ckt_sensorID = (HI541ReadCmosSensor(0x4865)<<8)|HI541ReadCmosSensor(0x4864);
    
	SENSORDB("HHL_enter the open function 0x%04x\n", ckt_sensorID);

    af_power();
	HI541InitSetting();
	HI541InitPara();
	
	HI541_FOCUS_AFC_Init();
	return ERROR_NONE;

}
/* HI541Open() */

/*************************************************************************
* FUNCTION
*   HI541GetSensorID
*
* DESCRIPTION
*   This function get the sensor ID 
*
* PARAMETERS
*   *sensorID : return the sensor ID 
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 HI541GetSensorID(UINT32 *sensorID) 

{
	int  retry = 3; 
	
	HI541WriteCmosSensor(0xffff,0x0040);

	
    *sensorID = HI541ReadCmosSensor(0x4864);
     SENSORDB("HI541 0X4864 Sensor ID = 0x%04x\n", *sensorID);
	
    *sensorID = HI541ReadCmosSensor(0x4865);
    SENSORDB("HI541 0X4865 Sensor ID = 0x%04x\n", *sensorID);

   
    do {
        *sensorID = (HI541ReadCmosSensor(0x4865)<<8)|HI541ReadCmosSensor(0x4864);
        if (*sensorID == HI541MIPI_SENSOR_ID)
            {
                SENSORDB("Sensor ID = 0x%04x\n", *sensorID);
                break;
            }
        SENSORDB("Read Sensor ID Fail = 0x%04x\n", *sensorID);

        retry--;
    } while (retry > 0);



	if (*sensorID != HI541MIPI_SENSOR_ID) {
		*sensorID = 0xFFFFFFFF; 
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
* HI541Close
*
* DESCRIPTION
* This HI541SetMaxFramerateByScenario is to turn off sensor module power.
*
* PARAMETERS
* None
*
* RETURNS
* None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 HI541SetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) {
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;

	//SENSORDB("HI541SetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);

	return ERROR_NONE;
}
/*************************************************************************
* FUNCTION
* HI541GetDefaultFramerateByScenario
*
* DESCRIPTION
* This function is to turn off sensor module power.
*
* PARAMETERS
* None
*
* RETURNS
* None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 HI541GetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*pframeRate = 300;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			*pframeRate = 220;
			break;		//hhl 2-28

		default:
			*pframeRate = 300;
			break;
	}
}

/*************************************************************************
* FUNCTION
* HI541Close
*
* DESCRIPTION
* This function is to turn off sensor module power.
*
* PARAMETERS
* None
*
* RETURNS
* None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 HI541Close(void)
{

     zsd =0;
	return ERROR_NONE;
} /* HI541Close() */

/*************************************************************************
* FUNCTION
* HI541Preview
*
* DESCRIPTION
* This function start the sensor preview.
*
* PARAMETERS
* *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
* None
*
* GLOBALS AFFECTED
*
*************************************************************************/



/*************************************************************************
* FUNCTION
* HI541Preview
*
* DESCRIPTION
* This function start the sensor preview.
*
* PARAMETERS
* *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
* None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 HI541Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint32 LineLength, EXP100, EXP120, EXPMIN, EXPUNIT; 

	//SENSORDB("\n\n\n\n\n\n");
	//SENSORDB("HHL[HI541]HI541Preview\n");
	/* For change max frame rate only need modify HI541Status.MaxFrameRate */
	SENSORDB("HHL[HI541]SensorOperationMode=%d\n",sensor_config_data->SensorOperationMode);
	spin_lock(&HI541_drv_lock);
	HI541Status.MaxFrameRate = HI541_MAX_FPS;
	HI541Status.SensorMode=SENSOR_MODE_PREVIEW;
	spin_unlock(&HI541_drv_lock);

zsd =0;

	if(sensor_config_data->SensorOperationMode== ACDK_SENSOR_OPERATION_MODE_VIDEO)
	{
		//SENSORDB("[HI541]HI541Video\n");
		/* For change max frame rate only need modify HI541Status.MaxFrameRate */
		spin_lock(&HI541_drv_lock);
		HI541Status.VideoMode=KAL_TRUE;
		HI541Status.SensorMode=SENSOR_MODE_VIDEO;
		spin_unlock(&HI541_drv_lock);

 #if 0               

HI541WriteCmosSensor(0xffff, 0x0040);

#else

	
		
	
//[USERSET_1]
//DISP_NAME = "Video_30"
//DISP_WIDTH = 1280	
//DISP_HEIGHT = 960
//MCLK = 84.00
//PLL = 1.00

//BEGIN

///////////////////////////////////////////
// Preview SXGA SUB Setting
// 1280x960@15fps, MIPI_YUV422, 1/2 Digital Scalex1/2 Analog Sub-sampling
///////////////////////////////////////////
//I2C_ID = 0x40
//I2C_BYTE  = 0x22

//================================
//========= SYSTEM Start =========
//================================
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x0100,0x0000);	//streaming disable
HI541WriteCmosSensor(0x5402,0x0004);	//tg_ctl3
HI541WriteCmosSensor(0x0900,0x0002);	//binning
HI541WriteCmosSensor(0x0340,0xf503);//frame_length_lines  //15 fps
HI541WriteCmosSensor(0x0342,0xca0a);//line_length_pck
HI541WriteCmosSensor(0x0344,0x1000);//x_addr_start
HI541WriteCmosSensor(0x0346,0x0e00);//y_addr_start
HI541WriteCmosSensor(0x0348,0x570a);//x_addr_end
HI541WriteCmosSensor(0x034a,0xc907);//y_addr_end
HI541WriteCmosSensor(0x034c,0x0005);//x_output_size
HI541WriteCmosSensor(0x034e,0xc003);//y_output_size
HI541WriteCmosSensor(0x0380,0x0100);//x_even_inc
HI541WriteCmosSensor(0x0382,0x0100);//x_odd_inc
HI541WriteCmosSensor(0x0384,0x0100);//y_even_inc
HI541WriteCmosSensor(0x0386,0x0300);//y_odd_inc

HI541WriteCmosSensor(0x540e,0x3677);//JH.LEE_140904
HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x0384,0xc073);//L
HI541WriteCmosSensor(0x0386,0x2600);//H exposure max 100 16.67fps
HI541WriteCmosSensor(0x0388,0x80b9);//L
HI541WriteCmosSensor(0x038a,0x2a00);//H exposure max 120 15fps
//================================
//========= SYSTEM End ===========
//================================

//================================
//======= ISP HW Start ===========
//================================
HI541WriteCmosSensor(0xffff,0x0040);

// ISP enable
HI541WriteCmosSensor(0x4830,0xfeef);
HI541WriteCmosSensor(0x4832,0x7f7a);//AF enable
HI541WriteCmosSensor(0x4834,0x0401);
  
// BScaler
HI541WriteCmosSensor(0x6000,0x2800);//mode_byrsc1
HI541WriteCmosSensor(0x6002,0x2000);//byrsc_fifo_delay

//Yscaler 1280x960
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0xa800,0x2000); //mode_zoom1
HI541WriteCmosSensor(0xa804,0x0005); //zoom_dst_width
HI541WriteCmosSensor(0xa806,0xc003); //zoom_dst_height
		HI541WriteCmosSensor(0xa810,0x1308); //zoom_hor_step
HI541WriteCmosSensor(0xa812,0x1908); //zoom_ver_step
		HI541WriteCmosSensor(0xa814,0x3E02); //zoom_hor_step_remain
HI541WriteCmosSensor(0xa816,0x9909); //zoom_ver_step_remain
		HI541WriteCmosSensor(0xa818,0x1000); //zoom_fifo_delay
HI541WriteCmosSensor(0xa824,0x0f00); //zoom_intpol1
HI541WriteCmosSensor(0xa826,0x0000); //zoom_intpol3

// Iridix
HI541WriteCmosSensor(0x8404,0x1405);//hdr_frame_width
HI541WriteCmosSensor(0x8406,0xd003);//hdr_frame_height
//================================
//======= ISP HW End   ===========
//================================


//================================
//=========== SSD Start ==========
//================================
//Hardware SSD SET
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0xc414,0x0000);//ae_patch_xy_offset
HI541WriteCmosSensor(0xc416,0xa03c);//ae_patch_xy_w_h x 16
HI541WriteCmosSensor(0xc418,0x0404);//awb_size_xy_offset
HI541WriteCmosSensor(0xc41a,0xa034);//awb_size_xy_w_h x 16

//Firmware SSD SET
HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x1fe4,0x0404);//awb_size_y_offset
HI541WriteCmosSensor(0x1fe6,0x200a);//awb_size_xy_w_h x 16
HI541WriteCmosSensor(0x1fe8,0x0404);//ae_size_xy_offset
HI541WriteCmosSensor(0x1fea,0xa034);//ae_size_xy_w_h x 16

//================================
//=========== SSD End   ==========
//================================

#if 1
//=====================================================
//=========== AF Filter Start - preview    ============
//=====================================================
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0xC806, 0xffC3); // af_ctl8 // manual window setting + x_half[6] 
HI541WriteCmosSensor(0xC808, 0x8000); // af_ctl8 // manual window setting + x_half[6] 

HI541WriteCmosSensor(0xC814, 0x1303); // AF window Region #1       	// Y Start
HI541WriteCmosSensor(0xC828, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC83C, 0xa601); 					// X Start	
HI541WriteCmosSensor(0xC850, 0x4a03); 					// X end	
HI541WriteCmosSensor(0xC816, 0x9f02); // AF window Region #2       	// Y Start
HI541WriteCmosSensor(0xC82A, 0xe504); 					// Y end	
HI541WriteCmosSensor(0xC83E, 0x3204); 					// X Start	
HI541WriteCmosSensor(0xC852, 0x5206); 					// X end	
HI541WriteCmosSensor(0xC818, 0x1303); // AF window Region #3       	// Y Start
HI541WriteCmosSensor(0xC82C, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC840, 0x3107); 					// X Start	
HI541WriteCmosSensor(0xC854, 0xd508); 					// X end	
HI541WriteCmosSensor(0xC81A, 0x5b05); // AF window Region #4       	// Y Start
HI541WriteCmosSensor(0xC82E, 0x8206); 					// Y end	
HI541WriteCmosSensor(0xC842, 0x6e04); 					// X Start	
HI541WriteCmosSensor(0xC856, 0x1206); 					// X end	
HI541WriteCmosSensor(0xC81C, 0x4803); // AF window Region #5       	// Y Start
HI541WriteCmosSensor(0xC830, 0x3804); 					// Y end	
HI541WriteCmosSensor(0xC844, 0xEA04); 					// X Start	
HI541WriteCmosSensor(0xC858, 0x9E05); 					// X end	
HI541WriteCmosSensor(0xC81E, 0x1303); // AF window Region #6       	// Y Start
HI541WriteCmosSensor(0xC832, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC846, 0xa601); 					// X Start	
HI541WriteCmosSensor(0xC85A, 0x4a03); 					// X end	
HI541WriteCmosSensor(0xC820, 0x9f02); // AF window Region #7       	// Y Start
HI541WriteCmosSensor(0xC834, 0xe504); 					// Y end	
HI541WriteCmosSensor(0xC848, 0x3204); 					// X Start	
HI541WriteCmosSensor(0xC85C, 0x5206); 					// X end	
HI541WriteCmosSensor(0xC822, 0x1303); // AF window Region #8       	// Y Start
HI541WriteCmosSensor(0xC836, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC84A, 0x3107); 					// X Start	
HI541WriteCmosSensor(0xC85E, 0xd508); 					// X end	
HI541WriteCmosSensor(0xC824, 0x5b05); // AF window Region #9       	// Y Start
HI541WriteCmosSensor(0xC838, 0x8206); 					// Y end	
HI541WriteCmosSensor(0xC84C, 0x6e04); 					// X Start	
HI541WriteCmosSensor(0xC860, 0x1206); 					// X end	
HI541WriteCmosSensor(0xC826, 0x4803); // AF window Region #10       	// Y Start
HI541WriteCmosSensor(0xC83A, 0x3804); 					// Y end	
HI541WriteCmosSensor(0xC84E, 0xEA04); 					// X Start	
HI541WriteCmosSensor(0xC862, 0x9E05); 					// X end

#else
//DISP_NAME = "AF_InitParaList"
//BEGIN
//=====================================================
//=========== AF Filter Start - preview    ============
//=====================================================
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0xC800, 0x4840);	// sobel& spmd  
HI541WriteCmosSensor(0xc802, 0x0044);
HI541WriteCmosSensor(0xc804, 0x0044);
HI541WriteCmosSensor(0xC806, 0xffC3); // af_ctl8 // manual window setting + x_half[6] 
HI541WriteCmosSensor(0xC808, 0x8000); // af_ctl8 // manual window setting + x_half[6] 

#if 0		// deleted by SeongYW
//========================================
//=========== F/W AF Start ===============
//========================================
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x003E, 0x8080);			// AWB Lock
HI541WriteCmosSensor(0x0020, 0xff00); 			// IICM Delay

HI541WriteCmosSensor(0x05B4, 0x0af6); 			//0x05b5 : f8(8), f7(9), f6(10),  EC(20) macro shift 2014.05.27
HI541WriteCmosSensor(0x05B6, 0x3219);			// Intelligent Step Size = 20

HI541WriteCmosSensor(0x05B8, 0x0819);			// Fine Scan Step Size
HI541WriteCmosSensor(0x05Ba, 0x0003);			// Falling Threshold Number
HI541WriteCmosSensor(0x05C0, 0x1100);			// Skip Frame	 
HI541WriteCmosSensor(0x05C8, 0x6414);			// softlanding step / first scan outdoor/ 	 
HI541WriteCmosSensor(0x05CA, 0x1414);			// first scan indoor / first scan dark

HI541WriteCmosSensor(0x05F4, 0x0200);			// Ae STD thresholg
HI541WriteCmosSensor(0x062A, 0x8000);			// min duty		// 250
HI541WriteCmosSensor(0x062C, 0xBC02);			// max duty
HI541WriteCmosSensor(0x062e, 0x9001);			// marco min duty

HI541WriteCmosSensor(0x0668, 0x4b4b);			// Uniform Th (rev BE)
HI541WriteCmosSensor(0x066a, 0x4001);			// Hyper Position (rev BE)

HI541WriteCmosSensor(0x0680, 0x1111);			// weight
HI541WriteCmosSensor(0x0682, 0x1111);			// weight
HI541WriteCmosSensor(0x0684, 0x2828);			// center weight  

//20140805
HI541WriteCmosSensor(0x05CE, 0x0003);        // MSB Masking set
HI541WriteCmosSensor(0x05D0, 0xFF00);        // LSB Masking set / SlewRate set
//HI541WriteCmosSensor(0x069C, 0x0008);        // MSB / LSB shift set


HI541WriteCmosSensor(0x0714, 0xC000);			// outdoor step LUT 1
HI541WriteCmosSensor(0x0716, 0xD000);			// outdoor step LUT 2	
HI541WriteCmosSensor(0x0718, 0xE000);			// outdoor step LUT 3	
HI541WriteCmosSensor(0x071A, 0xF000);			// outdoor step LUT 4	
HI541WriteCmosSensor(0x071C, 0x0001);			// outdoor step LUT 5	
HI541WriteCmosSensor(0x071E, 0x1001);			// outdoor step LUT 6	
HI541WriteCmosSensor(0x0720, 0x2001);			// outdoor step LUT 7	
HI541WriteCmosSensor(0x0722, 0x3001);			// outdoor step LUT 8	
HI541WriteCmosSensor(0x0724, 0x4001);			// outdoor step LUT 9	
HI541WriteCmosSensor(0x0726, 0x5001);			// outdoor step LUT 10	
HI541WriteCmosSensor(0x0728, 0x6001);			// outdoor step LUT 11	   
HI541WriteCmosSensor(0x072a, 0x7001);			// outdoor step LUT 12
HI541WriteCmosSensor(0x072c, 0x7801);			// outdoor step LUT 13
HI541WriteCmosSensor(0x072e, 0x8001);			// outdoor step LUT 14
HI541WriteCmosSensor(0x0730, 0x8801);			// outdoor step LUT 15
HI541WriteCmosSensor(0x0732, 0x9001);			// outdoor step LUT 16
HI541WriteCmosSensor(0x0734, 0xA001);			// outdoor step LUT 17
HI541WriteCmosSensor(0x0736, 0xB001);			// outdoor step LUT 18
HI541WriteCmosSensor(0x0738, 0xC001);			// outdoor step LUT 19
HI541WriteCmosSensor(0x073A, 0xC001);			// outdoor step LUT 20
HI541WriteCmosSensor(0x073C, 0xC001);			// outdoor step LUT 20


HI541WriteCmosSensor(0x0748, 0xC000);			// indoor step LUT 1
HI541WriteCmosSensor(0x074A, 0xD000);			// indoor step LUT 2	
HI541WriteCmosSensor(0x074C, 0xE000);			// indoor step LUT 3	
HI541WriteCmosSensor(0x074E, 0xF000);			// indoor step LUT 4	
HI541WriteCmosSensor(0x0750, 0x0001);			// indoor step LUT 5	
HI541WriteCmosSensor(0x0752, 0x1001);			// indoor step LUT 6	
HI541WriteCmosSensor(0x0754, 0x2001);			// indoor step LUT 7	
HI541WriteCmosSensor(0x0756, 0x3001);			// indoor step LUT 8	
HI541WriteCmosSensor(0x0758, 0x4001);			// indoor step LUT 9	
HI541WriteCmosSensor(0x075A, 0x5001);			// indoor step LUT 10	
HI541WriteCmosSensor(0x075C, 0x6001);
HI541WriteCmosSensor(0x075E, 0x7001);			// 
HI541WriteCmosSensor(0x0760, 0x7801);			//
HI541WriteCmosSensor(0x0762, 0x8001);			//
HI541WriteCmosSensor(0x0764, 0x8801);			//
HI541WriteCmosSensor(0x0766, 0x9001);			//
HI541WriteCmosSensor(0x0768, 0xA001);			//
HI541WriteCmosSensor(0x076A, 0xB001);			//
HI541WriteCmosSensor(0x076C, 0xC001);			//
HI541WriteCmosSensor(0x076E, 0xC001);			//
HI541WriteCmosSensor(0x0770, 0xC001);			//


HI541WriteCmosSensor(0x077C, 0xC000);			// dark step LUT 1
HI541WriteCmosSensor(0x077E, 0xD000);			// dark step LUT 2	
HI541WriteCmosSensor(0x0780, 0xE000);			// dark step LUT 3	
HI541WriteCmosSensor(0x0782, 0xF000);			// dark step LUT 4	
HI541WriteCmosSensor(0x0784, 0x0001);			// dark step LUT 5	
HI541WriteCmosSensor(0x0786, 0x1001);			// dark step LUT 6	
HI541WriteCmosSensor(0x0788, 0x2001);			// dark step LUT 7	
HI541WriteCmosSensor(0x078A, 0x3001);			// dark step LUT 8	
HI541WriteCmosSensor(0x078C, 0x4001);			// dark step LUT 9	
HI541WriteCmosSensor(0x078E, 0x5001);			// dark step LUT 10	
HI541WriteCmosSensor(0x0790, 0x6001);
HI541WriteCmosSensor(0x0792, 0x7001);
HI541WriteCmosSensor(0x0794, 0x7801);
HI541WriteCmosSensor(0x0796, 0x8001);
HI541WriteCmosSensor(0x0798, 0x8801);
HI541WriteCmosSensor(0x079A, 0x9001);
HI541WriteCmosSensor(0x079C, 0xA001);
HI541WriteCmosSensor(0x079E, 0xB001);
HI541WriteCmosSensor(0x07A0, 0xC001);
HI541WriteCmosSensor(0x07A2, 0xC001);
HI541WriteCmosSensor(0x07A4, 0xC001);



// added to Improve speed continuous AF
HI541WriteCmosSensor(0x05EA, 0x0008);  
HI541WriteCmosSensor(0x05EC, 0x0800);  
HI541WriteCmosSensor(0x05F0, 0x0408);  
HI541WriteCmosSensor(0x05F2, 0x0800);                  
HI541WriteCmosSensor(0x05F6, 0x0828);                  

HI541WriteCmosSensor(0x05C0, 0x1100);			// Skip Frame	
HI541WriteCmosSensor(0x0628, 0x1E15);  			// Slpope Step / Delay 

//========================================
//=========== F/W AF End  ================
//========================================
#endif
		HI541_FOCUS_AFC_Init();
//========================================
//========  Constant_Focus  Star add Rampart ========
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x05b0, 0x8616);	//AF Mode Select
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0x3824, 0x0500);	
HI541WriteCmosSensor(0x3826, 0xAF10);	//Do AF
//========================================
//========  Constant_Focus  End  Rampart   ==========
//========================================
#endif
//================================
//======== FINDBAND Start ========
//================================
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0xC002,0x4608);
HI541WriteCmosSensor(0xC004,0x4005);

//================================
//======== FINDBAND End   ========
//================================

//================================
//========= SYSTEM Start==========
//================================
//HI541WriteCmosSensor(0xffff,0x0040);
//HI541WriteCmosSensor(0x3824,0x0000);
//HI541WriteCmosSensor(0x3826,0x5a10);//MCU preview hif cmd

//HI541WriteCmosSensor(0x0100,0x0100);	//streaming enable
//================================
//========= SYSTEM End  ==========
//================================

//================================
//========= AE min Start==========
//================================
HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x0310,0x282b);
HI541WriteCmosSensor(0x0312,0x0000);//

HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x548a,0x0004);//

//================================
//========= AE min End  ==========
//================================

//================================
//====Return to Preview Start=====
//================================
// Para
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x3824,0x0000);
// Command
HI541WriteCmosSensor(0x3826,0x5a10);
//================================
//====Return to Preview End=======
//================================

HI541WriteCmosSensor(0x0100,0x0100);	//streaming eble

//END
//[END]

			

				
	
	



#endif

HI541_FOCUS_AFC_Constant_Focus();
		SENSORDB("HI541_FOCUS_AFC_Constant_Focus+++");
	

		spin_lock(&HI541_drv_lock);
		HI541Status.PvDummyPixels = 376;
		spin_unlock(&HI541_drv_lock);
		LineLength = HI541_PV_PERIOD_PIXEL_NUMS + HI541Status.PvDummyPixels;
		spin_lock(&HI541_drv_lock);
		HI541Status.MiniFrameRate = HI541_FPS(10);  
		HI541Status.PvDummyLines = HI541Status.PvOpClk * 1000000 * HI541_FRAME_RATE_UNIT / LineLength / HI541Status.MaxFrameRate -  HI541_PV_PERIOD_LINE_NUMS; 
		spin_unlock(&HI541_drv_lock);
		HI541SetAeMode(KAL_TRUE);
		HI541SetAwbMode(KAL_TRUE);
		// HI541NightMode(HI541Status.NightMode);
		SENSORDB("HHL[HI541]HI541Status.VideoMode=%d\n",HI541Status.VideoMode);
	}

	else if(sensor_config_data->SensorOperationMode== ACDK_SENSOR_OPERATION_MODE_CAMERA_PREVIEW)
	{          
                 
	#if 0

HI541WriteCmosSensor(0xffff, 0x0040);


#else

//[USERSET_3]
//DISP_NAME = "Pre_Normal 30~15"
//DISP_WIDTH = 1280	
//DISP_HEIGHT = 960

//BEGIN

///////////////////////////////////////////
// Preview SXGA SUB Setting
// 1280x960@30fps, MIPI_YUV422, 1/2 Digital Scalex1/2 Analog Sub-sampling
///////////////////////////////////////////
//I2C_ID = 0x40
//I2C_BYTE  = 0x22

//================================
//========= SYSTEM Start =========
//================================
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x0100,0x0000);	//streaming disable
HI541WriteCmosSensor(0x5402,0x0004);	//tg_ctl3
HI541WriteCmosSensor(0x0900,0x0002);	//binning
HI541WriteCmosSensor(0x0340,0xf503);//frame_length_lines  //16 fps
HI541WriteCmosSensor(0x0342,0xca0a);//line_length_pck
HI541WriteCmosSensor(0x0344,0x1000);//x_addr_start
HI541WriteCmosSensor(0x0346,0x0e00);//y_addr_start
HI541WriteCmosSensor(0x0348,0x570a);//x_addr_end
HI541WriteCmosSensor(0x034a,0xc907);//y_addr_end
HI541WriteCmosSensor(0x034c,0x0005);//x_output_size
HI541WriteCmosSensor(0x034e,0xc003);//y_output_size
HI541WriteCmosSensor(0x0380,0x0100);//x_even_inc
HI541WriteCmosSensor(0x0382,0x0100);//x_odd_inc
HI541WriteCmosSensor(0x0384,0x0100);//y_even_inc
HI541WriteCmosSensor(0x0386,0x0300);//y_odd_inc

HI541WriteCmosSensor(0x540e,0x3677);//JH.LEE_140904
HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x0384,0x80e7);//L
HI541WriteCmosSensor(0x0386,0x4c00);//H exposure max 100 8.33 fps
HI541WriteCmosSensor(0x0388,0x80e7);//L
HI541WriteCmosSensor(0x038a,0x4c00);//H exposure max 120 8.57 fps
//================================
//========= SYSTEM End ===========
//================================

//================================
//======= ISP HW Start ===========
//================================
HI541WriteCmosSensor(0xffff,0x0040);

// ISP enable
HI541WriteCmosSensor(0x4830,0xfeef);
HI541WriteCmosSensor(0x4832,0x7f7a);//AF enable
HI541WriteCmosSensor(0x4834,0x0401);
  
// BScaler
HI541WriteCmosSensor(0x6000,0x2800);//mode_byrsc1
HI541WriteCmosSensor(0x6002,0x2000);//byrsc_fifo_delay

//Yscaler 1280x960
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0xa800,0x2000); //mode_zoom1
HI541WriteCmosSensor(0xa804,0x0005); //zoom_dst_width
HI541WriteCmosSensor(0xa806,0xc003); //zoom_dst_height
		HI541WriteCmosSensor(0xa810,0x1308); //zoom_hor_step
HI541WriteCmosSensor(0xa812,0x1908); //zoom_ver_step
		HI541WriteCmosSensor(0xa814,0x3E02); //zoom_hor_step_remain
HI541WriteCmosSensor(0xa816,0x9909); //zoom_ver_step_remain
		HI541WriteCmosSensor(0xa818,0x1000); //zoom_fifo_delay
HI541WriteCmosSensor(0xa824,0x0f00); //zoom_intpol1
HI541WriteCmosSensor(0xa826,0x0000); //zoom_intpol3

// Iridix
HI541WriteCmosSensor(0x8404,0x1405);//hdr_frame_width
HI541WriteCmosSensor(0x8406,0xd003);//hdr_frame_height
//================================
//======= ISP HW End   ===========
//================================

//================================
//=========== SSD Start ==========
//================================
//Hardware SSD SET
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0xc414,0x0000);//ae_patch_xy_offset
HI541WriteCmosSensor(0xc416,0xa03c);//ae_patch_xy_w_h x 16
HI541WriteCmosSensor(0xc418,0x0404);//awb_size_xy_offset
HI541WriteCmosSensor(0xc41a,0xa034);//awb_size_xy_w_h x 16

//Firmware SSD SET
HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x1fe4,0x0404);//awb_size_y_offset
HI541WriteCmosSensor(0x1fe6,0x200a);//awb_size_xy_w_h x 16
HI541WriteCmosSensor(0x1fe8,0x0404);//ae_size_xy_offset
HI541WriteCmosSensor(0x1fea,0xa034);//ae_size_xy_w_h x 16

//================================
//=========== SSD End   ==========
//================================

#if 0
//=====================================================
//=========== AF Filter Start - preview    ============
//=====================================================
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0xC806, 0xffC3); // af_ctl8 // manual window setting + x_half[6] 
HI541WriteCmosSensor(0xC808, 0x8000); // af_ctl8 // manual window setting + x_half[6] 

HI541WriteCmosSensor(0xC814, 0x1303); // AF window Region #1       	// Y Start
HI541WriteCmosSensor(0xC828, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC83C, 0xa601); 					// X Start	
HI541WriteCmosSensor(0xC850, 0x4a03); 					// X end	
HI541WriteCmosSensor(0xC816, 0x9f02); // AF window Region #2       	// Y Start
HI541WriteCmosSensor(0xC82A, 0xe504); 					// Y end	
HI541WriteCmosSensor(0xC83E, 0x3204); 					// X Start	
HI541WriteCmosSensor(0xC852, 0x5206); 					// X end	
HI541WriteCmosSensor(0xC818, 0x1303); // AF window Region #3       	// Y Start
HI541WriteCmosSensor(0xC82C, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC840, 0x3107); 					// X Start	
HI541WriteCmosSensor(0xC854, 0xd508); 					// X end	
HI541WriteCmosSensor(0xC81A, 0x5b05); // AF window Region #4       	// Y Start
HI541WriteCmosSensor(0xC82E, 0x8206); 					// Y end	
HI541WriteCmosSensor(0xC842, 0x6e04); 					// X Start	
HI541WriteCmosSensor(0xC856, 0x1206); 					// X end	
HI541WriteCmosSensor(0xC81C, 0x4803); // AF window Region #5       	// Y Start
HI541WriteCmosSensor(0xC830, 0x3804); 					// Y end	
HI541WriteCmosSensor(0xC844, 0xEA04); 					// X Start	
HI541WriteCmosSensor(0xC858, 0x9E05); 					// X end	
HI541WriteCmosSensor(0xC81E, 0x1303); // AF window Region #6       	// Y Start
HI541WriteCmosSensor(0xC832, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC846, 0xa601); 					// X Start	
HI541WriteCmosSensor(0xC85A, 0x4a03); 					// X end	
HI541WriteCmosSensor(0xC820, 0x9f02); // AF window Region #7       	// Y Start
HI541WriteCmosSensor(0xC834, 0xe504); 					// Y end	
HI541WriteCmosSensor(0xC848, 0x3204); 					// X Start	
HI541WriteCmosSensor(0xC85C, 0x5206); 					// X end	
HI541WriteCmosSensor(0xC822, 0x1303); // AF window Region #8       	// Y Start
HI541WriteCmosSensor(0xC836, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC84A, 0x3107); 					// X Start	
HI541WriteCmosSensor(0xC85E, 0xd508); 					// X end	
HI541WriteCmosSensor(0xC824, 0x5b05); // AF window Region #9       	// Y Start
HI541WriteCmosSensor(0xC838, 0x8206); 					// Y end	
HI541WriteCmosSensor(0xC84C, 0x6e04); 					// X Start	
HI541WriteCmosSensor(0xC860, 0x1206); 					// X end	
HI541WriteCmosSensor(0xC826, 0x4803); // AF window Region #10       	// Y Start
HI541WriteCmosSensor(0xC83A, 0x3804); 					// Y end	
HI541WriteCmosSensor(0xC84E, 0xEA04); 					// X Start	
HI541WriteCmosSensor(0xC862, 0x9E05); 					// X end

#else
    #if 0
//DISP_NAME = "AF_InitParaList"
//BEGIN
//=====================================================
//=========== AF Filter Start - preview    ============
//=====================================================
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0xC800, 0x4840);	// sobel& spmd  
HI541WriteCmosSensor(0xc802, 0x0044);
HI541WriteCmosSensor(0xc804, 0x0044);
HI541WriteCmosSensor(0xC806, 0xffC3); // af_ctl8 // manual window setting + x_half[6] 
HI541WriteCmosSensor(0xC808, 0x8000); // af_ctl8 // manual window setting + x_half[6] 

//========================================
//=========== F/W AF Start ===============
//========================================
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x003E, 0x8080);			// AWB Lock
HI541WriteCmosSensor(0x0020, 0xff00); 			// IICM Delay

HI541WriteCmosSensor(0x05B4, 0x0af6); 			//0x05b5 : f8(8), f7(9), f6(10),  EC(20) macro shift 2014.05.27
HI541WriteCmosSensor(0x05B6, 0x3219);			// Intelligent Step Size = 20

HI541WriteCmosSensor(0x05B8, 0x0819);			// Fine Scan Step Size
HI541WriteCmosSensor(0x05Ba, 0x0003);			// Falling Threshold Number
HI541WriteCmosSensor(0x05C0, 0x1100);			// Skip Frame	 
HI541WriteCmosSensor(0x05C8, 0x6414);			// softlanding step / first scan outdoor/ 	 
HI541WriteCmosSensor(0x05CA, 0x1414);			// first scan indoor / first scan dark

HI541WriteCmosSensor(0x05F4, 0x0200);			// Ae STD thresholg
HI541WriteCmosSensor(0x062A, 0x8000);			// min duty		// 250
HI541WriteCmosSensor(0x062C, 0xBC02);			// max duty
HI541WriteCmosSensor(0x062e, 0x9001);			// marco min duty

HI541WriteCmosSensor(0x0668, 0x4b4b);			// Uniform Th (rev BE)
HI541WriteCmosSensor(0x066a, 0x4001);			// Hyper Position (rev BE)

HI541WriteCmosSensor(0x0680, 0x1111);			// weight
HI541WriteCmosSensor(0x0682, 0x1111);			// weight
HI541WriteCmosSensor(0x0684, 0x2828);			// center weight  

//20140805
HI541WriteCmosSensor(0x05CE, 0x0003);        // MSB Masking set
HI541WriteCmosSensor(0x05D0, 0xFF00);        // LSB Masking set / SlewRate set
//HI541WriteCmosSensor(0x069C, 0x0008);        // MSB / LSB shift set


HI541WriteCmosSensor(0x0714, 0xC000);			// outdoor step LUT 1
HI541WriteCmosSensor(0x0716, 0xD000);			// outdoor step LUT 2	
HI541WriteCmosSensor(0x0718, 0xE000);			// outdoor step LUT 3	
HI541WriteCmosSensor(0x071A, 0xF000);			// outdoor step LUT 4	
HI541WriteCmosSensor(0x071C, 0x0001);			// outdoor step LUT 5	
HI541WriteCmosSensor(0x071E, 0x1001);			// outdoor step LUT 6	
HI541WriteCmosSensor(0x0720, 0x2001);			// outdoor step LUT 7	
HI541WriteCmosSensor(0x0722, 0x3001);			// outdoor step LUT 8	
HI541WriteCmosSensor(0x0724, 0x4001);			// outdoor step LUT 9	
HI541WriteCmosSensor(0x0726, 0x5001);			// outdoor step LUT 10	
HI541WriteCmosSensor(0x0728, 0x6001);			// outdoor step LUT 11	   
HI541WriteCmosSensor(0x072a, 0x7001);			// outdoor step LUT 12
HI541WriteCmosSensor(0x072c, 0x7801);			// outdoor step LUT 13
HI541WriteCmosSensor(0x072e, 0x8001);			// outdoor step LUT 14
HI541WriteCmosSensor(0x0730, 0x8801);			// outdoor step LUT 15
HI541WriteCmosSensor(0x0732, 0x9001);			// outdoor step LUT 16
HI541WriteCmosSensor(0x0734, 0xA001);			// outdoor step LUT 17
HI541WriteCmosSensor(0x0736, 0xB001);			// outdoor step LUT 18
HI541WriteCmosSensor(0x0738, 0xC001);			// outdoor step LUT 19
HI541WriteCmosSensor(0x073A, 0xC001);			// outdoor step LUT 20
HI541WriteCmosSensor(0x073C, 0xC001);			// outdoor step LUT 20


HI541WriteCmosSensor(0x0748, 0xC000);			// indoor step LUT 1
HI541WriteCmosSensor(0x074A, 0xD000);			// indoor step LUT 2	
HI541WriteCmosSensor(0x074C, 0xE000);			// indoor step LUT 3	
HI541WriteCmosSensor(0x074E, 0xF000);			// indoor step LUT 4	
HI541WriteCmosSensor(0x0750, 0x0001);			// indoor step LUT 5	
HI541WriteCmosSensor(0x0752, 0x1001);			// indoor step LUT 6	
HI541WriteCmosSensor(0x0754, 0x2001);			// indoor step LUT 7	
HI541WriteCmosSensor(0x0756, 0x3001);			// indoor step LUT 8	
HI541WriteCmosSensor(0x0758, 0x4001);			// indoor step LUT 9	
HI541WriteCmosSensor(0x075A, 0x5001);			// indoor step LUT 10	
HI541WriteCmosSensor(0x075C, 0x6001);
HI541WriteCmosSensor(0x075E, 0x7001);			// 
HI541WriteCmosSensor(0x0760, 0x7801);			//
HI541WriteCmosSensor(0x0762, 0x8001);			//
HI541WriteCmosSensor(0x0764, 0x8801);			//
HI541WriteCmosSensor(0x0766, 0x9001);			//
HI541WriteCmosSensor(0x0768, 0xA001);			//
HI541WriteCmosSensor(0x076A, 0xB001);			//
HI541WriteCmosSensor(0x076C, 0xC001);			//
HI541WriteCmosSensor(0x076E, 0xC001);			//
HI541WriteCmosSensor(0x0770, 0xC001);			//


HI541WriteCmosSensor(0x077C, 0xC000);			// dark step LUT 1
HI541WriteCmosSensor(0x077E, 0xD000);			// dark step LUT 2	
HI541WriteCmosSensor(0x0780, 0xE000);			// dark step LUT 3	
HI541WriteCmosSensor(0x0782, 0xF000);			// dark step LUT 4	
HI541WriteCmosSensor(0x0784, 0x0001);			// dark step LUT 5	
HI541WriteCmosSensor(0x0786, 0x1001);			// dark step LUT 6	
HI541WriteCmosSensor(0x0788, 0x2001);			// dark step LUT 7	
HI541WriteCmosSensor(0x078A, 0x3001);			// dark step LUT 8	
HI541WriteCmosSensor(0x078C, 0x4001);			// dark step LUT 9	
HI541WriteCmosSensor(0x078E, 0x5001);			// dark step LUT 10	
HI541WriteCmosSensor(0x0790, 0x6001);
HI541WriteCmosSensor(0x0792, 0x7001);
HI541WriteCmosSensor(0x0794, 0x7801);
HI541WriteCmosSensor(0x0796, 0x8001);
HI541WriteCmosSensor(0x0798, 0x8801);
HI541WriteCmosSensor(0x079A, 0x9001);
HI541WriteCmosSensor(0x079C, 0xA001);
HI541WriteCmosSensor(0x079E, 0xB001);
HI541WriteCmosSensor(0x07A0, 0xC001);
HI541WriteCmosSensor(0x07A2, 0xC001);
HI541WriteCmosSensor(0x07A4, 0xC001);



// added to Improve speed continuous AF
HI541WriteCmosSensor(0x05EA, 0x0008);  
HI541WriteCmosSensor(0x05EC, 0x0800);  
HI541WriteCmosSensor(0x05F0, 0x0408);  
HI541WriteCmosSensor(0x05F2, 0x0800);                  
HI541WriteCmosSensor(0x05F6, 0x0828);                  

HI541WriteCmosSensor(0x05C0, 0x1100);			// Skip Frame	
HI541WriteCmosSensor(0x0628, 0x1E15);  			// Slpope Step / Delay 

//========================================
//=========== F/W AF End  ================
//========================================
//========================================
//========  Constant_Focus  Star add Rampart ========
//========================================
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x05b0, 0x8616);	//AF Mode Select
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0x3824, 0x0500);	
HI541WriteCmosSensor(0x3826, 0xAF10);	//Do AF

//========================================
//========  Constant_Focus  End  Rampart   ==========
//========================================
    #endif
#endif
//================================
//======== FINDBAND Start ========
//================================
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0xC002,0x4608);
HI541WriteCmosSensor(0xC004,0x4005);

//================================
//======== FINDBAND End   ========
//================================

//================================
//========= SYSTEM Start==========
//================================
//HI541WriteCmosSensor(0xffff,0x0040);
//HI541WriteCmosSensor(0x3824,0x0000);
//HI541WriteCmosSensor(0x3826,0x5a10);//MCU preview hif cmd
//================================
//========= SYSTEM End  ==========
//================================

//================================
//========= AE min Start==========
//================================
HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x0310,0x282b);
HI541WriteCmosSensor(0x0312,0x0000);//

HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x548a,0x0004);//

//================================
//========= AE min End  ==========
//================================

//================================
//====Return to Preview Start=====
//================================
// Para
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x3824,0x0000);
// Command
HI541WriteCmosSensor(0x3826,0x5a10);
//================================
//====Return to Preview End=======
//================================

HI541WriteCmosSensor(0x0100,0x0100);	//streaming eble

//END
//[END]

#endif

		

		

		spin_lock(&HI541_drv_lock);
		HI541Status.PvDummyPixels = 388;
		spin_unlock(&HI541_drv_lock);
		LineLength = HI541_PV_PERIOD_PIXEL_NUMS + HI541Status.PvDummyPixels;
		spin_lock(&HI541_drv_lock);
		HI541Status.MiniFrameRate = HI541_FPS(10);
		HI541Status.PvDummyLines = HI541Status.PvOpClk * 1000000 * HI541_FRAME_RATE_UNIT / LineLength / HI541Status.MaxFrameRate -  HI541_PV_PERIOD_LINE_NUMS;
		spin_unlock(&HI541_drv_lock);

		HI541SetAeMode(KAL_TRUE);
		HI541SetAwbMode(KAL_TRUE);

		HI541NightMode(HI541Status.NightMode);
	}
	//added for test
	return ERROR_NONE;
}/* HI541Preview() */


UINT32 HI541ZSD(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{										 
	kal_uint32 LineLength, EXP100, EXP120, EXPMIN, EXPUNIT, CapShutter;
	kal_uint8 ClockDivider;
	kal_uint32 temp;
	//SENSORDB("\n\n\n\n\n\n");
	//SENSORDB("HHL[HI541]HI541ZSD Preview!!!!!!!!!!!!!\n");
	//SENSORDB("[HI541]Image Target Width: %d; Height: %d\n",image_window->ImageTargetWidth, image_window->ImageTargetHeight);
	zsd =1 ;
	

	spin_lock(&HI541_drv_lock);
	HI541Status.SensorMode=SENSOR_MODE_ZSD;
	spin_unlock(&HI541_drv_lock);
	
	
//DISP_WIDTH = 2592	
//DISP_HEIGHT = 1944
///////////////////////////////////////////
// Capture Setting
// 2592x1944@15fps, MIPI_YUV422, Normal
///////////////////////////////////////////


///////////////////////////////////////////
// Capture Setting
// 2592x1944@15fps, MIPI_YUV422, Normal
///////////////////////////////////////////


//================================
//========= SYSTEM Start =========
//================================
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0x0100, 0x0000);	//streaming disable

HI541WriteCmosSensor(0x5402, 0x0004);	//tg_ctl3
HI541WriteCmosSensor(0x0900, 0x0002);	//binning
HI541WriteCmosSensor(0x0340, 0xc207);	//frame_length_lines //8fps
HI541WriteCmosSensor(0x0342, 0xca0a); //line_length_pck
HI541WriteCmosSensor(0x0344, 0x1a00);	//x_addr_start
HI541WriteCmosSensor(0x0346, 0x1800);	//y_addr_start
	HI541WriteCmosSensor(0x0348, 0x510a);	//x_addr_end
HI541WriteCmosSensor(0x034a, 0xc107);	//y_addr_end
HI541WriteCmosSensor(0x034c, 0x200a);	//x_output_size
HI541WriteCmosSensor(0x034e, 0x9807);	//y_output_size
HI541WriteCmosSensor(0x0380, 0x0100);	//x_even_inc
HI541WriteCmosSensor(0x0382, 0x0100);	//x_odd_inc
HI541WriteCmosSensor(0x0384, 0x0100);	//y_even_inc
HI541WriteCmosSensor(0x0386, 0x0100);	//y_odd_inc
HI541WriteCmosSensor(0x540e, 0x3777);//JH.LEE_140904 
//================================
//========= SYSTEM End ===========
//================================

//================================
//======= ISP HW Start ===========
//================================
HI541WriteCmosSensor(0xffff, 0x0040);

// ISP enable
HI541WriteCmosSensor(0x4830, 0xfeef);
HI541WriteCmosSensor(0x4832, 0x7f7a);//AF enable
HI541WriteCmosSensor(0x4834, 0x0401);	

// BScaler
HI541WriteCmosSensor(0x6000, 0x0000); //mode_byrsc1
HI541WriteCmosSensor(0x6002, 0x2000); //byrsc_fifo_delay

//Yscaler 2592x1944
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0xa800,0x2000); //mode_zoom1
HI541WriteCmosSensor(0xa804,0x200a); //zoom_dst_width
HI541WriteCmosSensor(0xa806,0x9807); //zoom_dst_height
HI541WriteCmosSensor(0xa810,0x0008); //zoom_hor_step
HI541WriteCmosSensor(0xa812,0x0008); //zoom_ver_step
HI541WriteCmosSensor(0xa814,0x0000); //zoom_hor_step_remain
HI541WriteCmosSensor(0xa816,0x0000); //zoom_ver_step_remain
HI541WriteCmosSensor(0xa818,0x6400); //zoom_fifo_delay
HI541WriteCmosSensor(0xa824,0x0000); //zoom_intpol1
HI541WriteCmosSensor(0xa826,0x0000); //zoom_intpol3

// Iridix
HI541WriteCmosSensor(0x8404, 0x240a); //hdr_frame_width
HI541WriteCmosSensor(0x8406, 0x9c07); //hdr_frame_height
//================================
//======= ISP HW End   ===========
//================================

//================================
//=========== SSD Start ==========
//================================
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0xc416, 0xa078); //ae_patch_xy_w_h x 16

HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x1fe4, 0x0404); //awb_size_xy_offset
HI541WriteCmosSensor(0x1fea, 0xa070); //Full_size_height
//================================
//=========== SSD End   ==========
//================================

//==================================================================
//=========== AF Filter Start - Capture Size (2592x1944)============
//==================================================================
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0xC806, 0xff03); // af_ctl8 // manual window setting + x_half[6] 
HI541WriteCmosSensor(0xC808, 0x0000); // af_ctl8 // manual window setting + x_half[6]


HI541WriteCmosSensor(0xC814, 0x1303); // AF window Region #1       	// Y Start
HI541WriteCmosSensor(0xC828, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC83C, 0xa601); 					// X Start	
HI541WriteCmosSensor(0xC850, 0x4a03); 					// X end	
HI541WriteCmosSensor(0xC816, 0x9f02); // AF window Region #2       	// Y Start
HI541WriteCmosSensor(0xC82A, 0xe504); 					// Y end	
HI541WriteCmosSensor(0xC83E, 0x3204); 					// X Start	
HI541WriteCmosSensor(0xC852, 0x5206); 					// X end	
HI541WriteCmosSensor(0xC818, 0x1303); // AF window Region #3       	// Y Start
HI541WriteCmosSensor(0xC82C, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC840, 0x3107); 					// X Start	
HI541WriteCmosSensor(0xC854, 0xd508); 					// X end	
HI541WriteCmosSensor(0xC81A, 0x5b05); // AF window Region #4       	// Y Start
HI541WriteCmosSensor(0xC82E, 0x8206); 					// Y end	
HI541WriteCmosSensor(0xC842, 0x6e04); 					// X Start	
HI541WriteCmosSensor(0xC856, 0x1206); 					// X end	
HI541WriteCmosSensor(0xC81C, 0x4803); // AF window Region #5       	// Y Start
HI541WriteCmosSensor(0xC830, 0x3804); 					// Y end	
HI541WriteCmosSensor(0xC844, 0xEA04); 					// X Start	
HI541WriteCmosSensor(0xC858, 0x9E05); 					// X end	
HI541WriteCmosSensor(0xC81E, 0x1303); // AF window Region #6       	// Y Start
HI541WriteCmosSensor(0xC832, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC846, 0xa601); 					// X Start	
HI541WriteCmosSensor(0xC85A, 0x4a03); 					// X end	
HI541WriteCmosSensor(0xC820, 0x9f02); // AF window Region #7       	// Y Start
HI541WriteCmosSensor(0xC834, 0xe504); 					// Y end	
HI541WriteCmosSensor(0xC848, 0x3204); 					// X Start	
HI541WriteCmosSensor(0xC85C, 0x5206); 					// X end	
HI541WriteCmosSensor(0xC822, 0x1303); // AF window Region #8       	// Y Start
HI541WriteCmosSensor(0xC836, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC84A, 0x3107); 					// X Start	
HI541WriteCmosSensor(0xC85E, 0xd508); 					// X end	
HI541WriteCmosSensor(0xC824, 0x5b05); // AF window Region #9       	// Y Start
HI541WriteCmosSensor(0xC838, 0x8206); 					// Y end	
HI541WriteCmosSensor(0xC84C, 0x6e04); 					// X Start	
HI541WriteCmosSensor(0xC860, 0x1206); 					// X end	
HI541WriteCmosSensor(0xC826, 0x4803); // AF window Region #10       	// Y Start
HI541WriteCmosSensor(0xC83A, 0x3804); 					// Y end	
HI541WriteCmosSensor(0xC84E, 0xEA04); 					// X Start	
HI541WriteCmosSensor(0xC862, 0x9E05); 					// X end	
//================================
//======== FINDBAND Start ========
//================================
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0xC002, 0x4608);
HI541WriteCmosSensor(0xC004, 0x700a);

//================================
//======== FINDBAND End   ========
//================================

//================================
//========= SYSTEM Start==========
//================================
//HI541WriteCmosSensor(0xffff, 0x0040);
//HI541WriteCmosSensor(0x3824, 0x0000);
//HI541WriteCmosSensor(0x3826, 0x5a10); //MCU preview hif cmd
//================================
//========= SYSTEM End  ==========
//================================
//================================
//========= AE min Start==========
//================================
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0310, 0x282b);
HI541WriteCmosSensor(0x0312, 0x0000); //

HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0x548a, 0x0004); //

//================================
//========= AE min End  ==========
//================================	
// (7) Skhynix JHKIM - change sequence
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0x3824, 0x0000);
HI541WriteCmosSensor(0x3826, 0x5a10); //MCU preview hif cmd
	
////================================
////========= Capture Start=========
////================================
//// Para
//HI541WriteCmosSensor(0x3824, 0x0000);
//// Command
//HI541WriteCmosSensor(0x3826, 0x5b10);
////================================
////========= Capture End  =========
////================================	
HI541WriteCmosSensor(0x0100, 0x0100);	//streaming eble


	//HI541SetAeMode(KAL_TRUE);
	//HI541SetAwbMode(KAL_TRUE);
	//HI541NightMode(HI541Status.NightMode);
	//CAPTURE_FLAG = 1;
	//CAPTURE_FLAG1 = 1;  //xianliang sign

	hi541_exposuretime = HI541_get_Shutterspeed();
	hi541_isospeed = HI541_get_iso();

	
	return ERROR_NONE;
} /* HI541Capture() */
/* HI541Capture() */


UINT32 HI541Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{										 
	kal_uint32 LineLength, EXP100, EXP120, EXPMIN, EXPUNIT, CapShutter;
	kal_uint8 ClockDivider;
	kal_uint32 temp;
//	SENSORDB("\n\n\n\n\n\n");
	//SENSORDB("HHL[HI541]HI541Capture!!!!!!!!!!!!!\n");
	//SENSORDB("[HI541]Image Target Width: %d; Height: %d\n",image_window->ImageTargetWidth, image_window->ImageTargetHeight);

#if 0	


HI541WriteCmosSensor(0xffff, 0x0040);

#else

zsd =0;
//[USERSET_5]
//DISP_NAME = "Capture"
//DISP_WIDTH = 2592
//DISP_HEIGHT = 1944

//BEGIN	

//================================
//========= SYSTEM Start =========
//================================
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x0100,0x0000);	//streaming disable
HI541WriteCmosSensor(0x5402,0x0004);	//tg_ctl3
HI541WriteCmosSensor(0x0900,0x0002);	//binning
HI541WriteCmosSensor(0x0340,0xc207);	//frame_length_lines //8fps
HI541WriteCmosSensor(0x0342,0xca0a);	//line_length_pck
HI541WriteCmosSensor(0x0344,0x1a00);	//x_addr_start
HI541WriteCmosSensor(0x0346,0x1800);	//y_addr_start
	HI541WriteCmosSensor(0x0348,0x510a);	//x_addr_end
HI541WriteCmosSensor(0x034a,0xc107);	//y_addr_end
HI541WriteCmosSensor(0x034c,0x200a);	//x_output_size
HI541WriteCmosSensor(0x034e,0x9807);	//y_output_size
HI541WriteCmosSensor(0x0380,0x0100);	//x_even_inc
HI541WriteCmosSensor(0x0382,0x0100);	//x_odd_inc
HI541WriteCmosSensor(0x0384,0x0100);	//y_even_inc
HI541WriteCmosSensor(0x0386,0x0100);	//y_odd_inc
HI541WriteCmosSensor(0x540e,0x3777);//JH.LEE_140904
//================================
//========= SYSTEM End ===========
//================================

//================================
//======= ISP HW Start ===========
//================================
HI541WriteCmosSensor(0xffff,0x0040);

// ISP enable
HI541WriteCmosSensor(0x4830,0xfeef);
HI541WriteCmosSensor(0x4832,0x7f7a);//AF enable
HI541WriteCmosSensor(0x4834,0x0401);	

// BScaler
HI541WriteCmosSensor(0x6000,0x0000);//mode_byrsc1
HI541WriteCmosSensor(0x6002,0x2000);//byrsc_fifo_delay

//Yscaler 2592x1944
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0xa800,0x2000); //mode_zoom1
HI541WriteCmosSensor(0xa804,0x200a); //zoom_dst_width
HI541WriteCmosSensor(0xa806,0x9807); //zoom_dst_height
HI541WriteCmosSensor(0xa810,0x0008); //zoom_hor_step
HI541WriteCmosSensor(0xa812,0x0008); //zoom_ver_step
HI541WriteCmosSensor(0xa814,0x0000); //zoom_hor_step_remain
HI541WriteCmosSensor(0xa816,0x0000); //zoom_ver_step_remain
HI541WriteCmosSensor(0xa818,0x6400); //zoom_fifo_delay
HI541WriteCmosSensor(0xa824,0x0000); //zoom_intpol1
HI541WriteCmosSensor(0xa826,0x0000); //zoom_intpol3

// Iridix
HI541WriteCmosSensor(0x8404,0x240a);//hdr_frame_width
HI541WriteCmosSensor(0x8406,0x9c07);//hdr_frame_height
//================================
//======= ISP HW End   ===========
//================================

//================================
//=========== SSD Start ==========
//================================
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0xc416,0xa078);//ae_patch_xy_w_h x 16

HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x1fe4,0x0404);//awb_size_xy_offset
HI541WriteCmosSensor(0x1fe6,0x200a);//awb_size_xy_w_h x 16  _added_0428
HI541WriteCmosSensor(0x1fea,0xa070);//Full_size_height
//================================
//=========== SSD End   ==========
//================================

//================================
//======== FINDBAND Start ========
//================================
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0xC002,0x4608);
HI541WriteCmosSensor(0xC004,0x7005);

//================================
//======== FINDBAND End   ========
//================================

//================================
//========= SYSTEM Start==========
//================================
// (8) Skhynix JHKIM - no need
//HI541WriteCmosSensor(0xffff,0x0040);
//HI541WriteCmosSensor(0x3824,0x0000);
//HI541WriteCmosSensor(0x3826,0x5a10);//MCU preview hif cmd
//================================
//========= SYSTEM End  ==========
//================================
//================================
//========= AE min Start==========
//================================
HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x0310,0x282b);
HI541WriteCmosSensor(0x0312,0x0000);//

HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x548a,0x0004);//

//================================
//========= AE min End  ==========
//================================	
	
//================================
//========= Capture Start=========
//================================
HI541WriteCmosSensor(0xffff,0x0020);
// (Exp time*2, AG/2) Settng 
HI541WriteCmosSensor(0x1BA0, 0x0774); 
HI541WriteCmosSensor(0x1BA2, 0xa000);
HI541WriteCmosSensor(0x1BB6, 0x0001); // = ((1BB7<<8)+1BB8)/256
HI541WriteCmosSensor(0x1BB8, 0x00FF); 
HI541WriteCmosSensor(0x1BC0, 0x0104); 
HI541WriteCmosSensor(0xffff,0x0040);
// Para
HI541WriteCmosSensor(0x3824,0x0000);
// Command
HI541WriteCmosSensor(0x3826,0x5b10);
//================================
//========= Capture End  =========
//================================
HI541WriteCmosSensor(0x0100,0x0100);	//streaming eble

//END
//[END]


#endif

#if 0
//add for flash 
    if (hi541_strobe == TRUE)
    {
		HI541WriteCmosSensor(0xffff,0x0020);  
		HI541WriteCmosSensor(0x0E00,0x8080);// 0?? ????
		HI541WriteCmosSensor(0x0E02,0x8080);// 2?? ????
		HI541WriteCmosSensor(0x0E04,0x8080);// 4?? ????
		HI541WriteCmosSensor(0x0E06,0x8080);// 6?? ???? // ????
		HI541WriteCmosSensor(0x0E08,0x8080);// 8?? ????
		HI541WriteCmosSensor(0x0E0A,0x8080);// 10?? ????
    }else
    { 
        HI541WriteCmosSensor(0xffff,0x0020);  
        HI541WriteCmosSensor(0x0E00,0x4040);// 0?? ????
        HI541WriteCmosSensor(0x0E02,0x4040);// 2?? ????
        HI541WriteCmosSensor(0x0E04,0x4040);// 4?? ????
        HI541WriteCmosSensor(0x0E06,0x4040);// 6?? ???? // ????
        HI541WriteCmosSensor(0x0E08,0x4040);// 8?? ????
        HI541WriteCmosSensor(0x0E0A,0x4040);// 10?? ????
	}

#endif
    


	//HI541SetAeMode(KAL_FALSE);
	HI541SetAwbMode(KAL_FALSE);
	spin_lock(&HI541_drv_lock);
	CAPTURE_FLAG = 1;
	CAPTURE_FLAG1= 1;
	HI541Status.SensorMode=SENSOR_MODE_CAPTURE;
	spin_unlock(&HI541_drv_lock);

	hi541_exposuretime = HI541_get_Shutterspeed();
	hi541_isospeed = HI541_get_iso();
	
	return ERROR_NONE;
} /* HI541Capture() */
/* HI541Capture() */

UINT32 HI541GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	pSensorResolution->SensorFullWidth = HI541_FULL_WIDTH;
	pSensorResolution->SensorFullHeight = HI541_FULL_HEIGHT;
	pSensorResolution->SensorPreviewWidth = HI541_PV_WIDTH;
	pSensorResolution->SensorPreviewHeight = HI541_PV_HEIGHT;
	pSensorResolution->SensorVideoWidth=HI541_PV_WIDTH;
	pSensorResolution->SensorVideoHeight = HI541_PV_HEIGHT;
	return ERROR_NONE;
} /* HI541GetResolution() */
void HI541GetDelayInfo(UINT32 delayAddr)
{
	SENSOR_DELAY_INFO_STRUCT* pDelayInfo = (SENSOR_DELAY_INFO_STRUCT*)delayAddr;
	pDelayInfo->InitDelay = 1;
	pDelayInfo->EffectDelay = 1;
	pDelayInfo->AwbDelay = 1;
	pDelayInfo->AFSwitchDelayFrame=50;
}
void HI541GetAEAWBLock(UINT32 *pAElockRet32,UINT32 *pAWBlockRet32)
{
	*pAElockRet32 = 1;
	*pAWBlockRet32 = 1;
	//SENSORDB("HI541_MIPIGetAEAWBLock,AE=%d ,AWB=%d\n,",*pAElockRet32,*pAWBlockRet32);
}

void HI541AutoTestCmd(UINT32 *cmd, UINT32 *para)
{
	switch(*cmd){
		 case YUV_AUTOTEST_SET_SHADDING:
			 //SENSORDB("YUV_AUTOTEST_SET_SHADDING:para = %d\n",*para);
			 break;
		 case YUV_AUTOTEST_SET_GAMMA:
			 //SENSORDB("YUV_AUTOTEST_SET_GAMMA:para = %d\n",*para);
			 break;
		 case YUV_AUTOTEST_SET_AE:
			 //SENSORDB("YUV_AUTOTEST_SET_AE:para = %d\n",*para);
			 break;
		 case YUV_AUTOTEST_SET_SHUTTER:
			 //SENSORDB("YUV_AUTOTEST_SET_SHUTTER:para = %d\n",*para);
			 break;
		 case YUV_AUTOTEST_SET_GAIN:
			 //SENSORDB("YUV_AUTOTEST_SET_GAIN:para = %d\n",*para);
			 break;
		 case YUV_AUTOTEST_GET_SHUTTER_RANGE:
			 *para = 8228;
			 break;
		 default:	 
			 //SENSORDB("YUV AUTOTEST NOT SUPPORT CMD:%d\n",*cmd);
			 break;
	}
}

UINT32 HI541GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	pSensorInfo->SensorPreviewResolutionX=HI541_PV_WIDTH;
	pSensorInfo->SensorPreviewResolutionY=HI541_PV_HEIGHT;
	pSensorInfo->SensorFullResolutionX=HI541_FULL_WIDTH;
	pSensorInfo->SensorFullResolutionY=HI541_FULL_HEIGHT;
	pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=20;
	pSensorInfo->SensorStillCaptureFrameRate=10;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=1;
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV; // back for 16 SENSOR_OUTPUT_FORMAT_UYVY;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorInterruptDelayLines = 1;
	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;

	pSensorInfo->CaptureDelayFrame = 1; 
	pSensorInfo->PreviewDelayFrame = 1; 
	pSensorInfo->VideoDelayFrame = 1; 
	pSensorInfo->SensorMasterClockSwitch = 0; 
	pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_2MA; 

	switch (ScenarioId)
	{
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

                 pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=	5;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = HI541_GRAB_START_X;
			pSensorInfo->SensorGrabStartY = HI541_GRAB_START_Y; 
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0; 
			pSensorInfo->SensorHightSampling = 0;  	
			pSensorInfo->SensorPacketECCOrder = 1;	
	

                  case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		   case MSDK_SCENARIO_ID_CAMERA_ZSD:

              pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=	5;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = HI541_GRAB_START_X;
			pSensorInfo->SensorGrabStartY = HI541_GRAB_START_Y;            
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount =14; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->SensorWidthSampling = 0; 
			pSensorInfo->SensorHightSampling = 0;
			pSensorInfo->SensorPacketECCOrder = 1;
	default:
                        pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=5;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
			pSensorInfo->SensorGrabStartX = HI541_GRAB_START_X; 
			pSensorInfo->SensorGrabStartY = HI541_GRAB_START_Y;  			
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;
			pSensorInfo->SensorHightSampling = 0;	
			pSensorInfo->SensorPacketECCOrder = 1;
		break;
	}
	return ERROR_NONE;
} /* HI541GetInfo() */


UINT32 HI541Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

	SENSORDB("HI541_DebugHHL_enter the control function the ScenarioId is %d\n",ScenarioId);
	switch (ScenarioId)
	{
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		HI541Preview(pImageWindow, pSensorConfigData);

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		HI541Capture(pImageWindow, pSensorConfigData);
		break;
	case MSDK_SCENARIO_ID_CAMERA_ZSD:
		HI541ZSD(pImageWindow, pSensorConfigData);
		break;
	default:
		break; 
	}
	return TRUE;
} /* HI541Control() */

void HI541_set_scene_mode(UINT16 para)
{

        switch (para)
    { 
		    case SCENE_MODE_NIGHTSCENE:
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0DB0, 0x0000);// ADP B[0]On/Off // B[1] DMA  On/Off // ADP Std Copy

HI541WriteCmosSensor(0x0DCC, 0x0064);// EV TH (Dark2) 5fps AGx8
HI541WriteCmosSensor(0x0DCE, 0x0104);

HI541WriteCmosSensor(0x0DD0, 0x00b2);// EV TH (Dark1) 10fps AGx4
HI541WriteCmosSensor(0x0DD2, 0x0002);

HI541WriteCmosSensor(0x0DD4, 0xf508);// EV TH (Indoor) 33.33fps AGx1.25
HI541WriteCmosSensor(0x0DD6, 0x1800);

HI541WriteCmosSensor(0x0DD8, 0x10a4);// EV TH (Outdoor) 1000fps AGx1
HI541WriteCmosSensor(0x0DDA, 0x0000);
//===============================================
//5fps

HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0384, 0x802c);//L
HI541WriteCmosSensor(0x0386, 0x8000);//H exposure max 100 5fps
HI541WriteCmosSensor(0x0388, 0x802c);//L
HI541WriteCmosSensor(0x038a, 0x8000);//H exposure max 120 5fps
//================================================

//===============================================
//ISO AUTO
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x02ee, 0x2020);//AG th0,1
HI541WriteCmosSensor(0x02f0, 0x2020);//AG th2,3
HI541WriteCmosSensor(0x02f2, 0xff00);//AG th4,

//===============================================

//===============================================
//AE weight table_Center
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0448, 0x1111);
HI541WriteCmosSensor(0x044a, 0x1111);
HI541WriteCmosSensor(0x044c, 0x1111);
HI541WriteCmosSensor(0x044e, 0x1111);
HI541WriteCmosSensor(0x0450, 0x1111);
HI541WriteCmosSensor(0x0452, 0x1111);
HI541WriteCmosSensor(0x0454, 0x1111);
HI541WriteCmosSensor(0x0456, 0x1111);
HI541WriteCmosSensor(0x0458, 0x1111);
HI541WriteCmosSensor(0x045a, 0x1111);
HI541WriteCmosSensor(0x045c, 0x1111);
HI541WriteCmosSensor(0x045e, 0x1111);
HI541WriteCmosSensor(0x0460, 0x1111);
HI541WriteCmosSensor(0x0462, 0x1111);
HI541WriteCmosSensor(0x0464, 0x1111);
HI541WriteCmosSensor(0x0466, 0x1111);
HI541WriteCmosSensor(0x0468, 0x1111);
HI541WriteCmosSensor(0x046a, 0x4444);
HI541WriteCmosSensor(0x046c, 0x4444);
HI541WriteCmosSensor(0x046e, 0x1111);
HI541WriteCmosSensor(0x0470, 0x1111);
HI541WriteCmosSensor(0x0472, 0x4444);
HI541WriteCmosSensor(0x0474, 0x4444);
HI541WriteCmosSensor(0x0476, 0x1111);
HI541WriteCmosSensor(0x0478, 0x1111);
HI541WriteCmosSensor(0x047a, 0x4444);
HI541WriteCmosSensor(0x047c, 0x4444);
HI541WriteCmosSensor(0x047e, 0x1111);
HI541WriteCmosSensor(0x0480, 0x1111);
HI541WriteCmosSensor(0x0482, 0x4455);
HI541WriteCmosSensor(0x0484, 0x5544);
HI541WriteCmosSensor(0x0486, 0x1111);
HI541WriteCmosSensor(0x0488, 0x1111);
HI541WriteCmosSensor(0x048a, 0x4455);
HI541WriteCmosSensor(0x048c, 0x5544);
HI541WriteCmosSensor(0x048e, 0x1111);
HI541WriteCmosSensor(0x0490, 0x1111);
HI541WriteCmosSensor(0x0492, 0x4444);
HI541WriteCmosSensor(0x0494, 0x4444);
HI541WriteCmosSensor(0x0496, 0x1111);
HI541WriteCmosSensor(0x0498, 0x1111);
HI541WriteCmosSensor(0x049a, 0x4444);
HI541WriteCmosSensor(0x049c, 0x4444);
HI541WriteCmosSensor(0x049e, 0x1111);
HI541WriteCmosSensor(0x04a0, 0x1111);
HI541WriteCmosSensor(0x04a2, 0x4444);
HI541WriteCmosSensor(0x04a4, 0x4444);
HI541WriteCmosSensor(0x04a6, 0x1111);
HI541WriteCmosSensor(0x04a8, 0x1111);
HI541WriteCmosSensor(0x04aa, 0x4444);
HI541WriteCmosSensor(0x04ac, 0x4444);
HI541WriteCmosSensor(0x04ae, 0x1111);
HI541WriteCmosSensor(0x04b0, 0x3333);
HI541WriteCmosSensor(0x04b2, 0x3333);
HI541WriteCmosSensor(0x04b4, 0x3333);
HI541WriteCmosSensor(0x04b6, 0x3333);
HI541WriteCmosSensor(0x04b8, 0x3333);
HI541WriteCmosSensor(0x04ba, 0x3333);
HI541WriteCmosSensor(0x04bc, 0x3333);
HI541WriteCmosSensor(0x04be, 0x3333);
HI541WriteCmosSensor(0x04c0, 0x3333);
HI541WriteCmosSensor(0x04c2, 0x3333);
HI541WriteCmosSensor(0x04c4, 0x3333);
HI541WriteCmosSensor(0x04c6, 0x3333);
//================================================

//===============================================
//AWB_Auto
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0a0c, 0x1010); //Rmin,Bmin
HI541WriteCmosSensor(0x0a0e, 0xc0c0); //Rmax,Bmax 
HI541WriteCmosSensor(0xffff, 0x0040);
//===============================================

//===============================================
// SAT ORI
HI541WriteCmosSensor(0xffff, 0x0020);
// CB Saturation
HI541WriteCmosSensor(0x0FC8, 0x8080);// 0?? ???? CB Saturation
		HI541WriteCmosSensor(0x0FCA,0x8088);// 2?? ???? CB Saturation
		HI541WriteCmosSensor(0x0FCC,0x9090);// 4?? ???? CB Saturation
HI541WriteCmosSensor(0x0FCE, 0x9a9a);// 6?? ???? CB Saturation
HI541WriteCmosSensor(0x0FD0, 0x9ca0);// 8?? ???? CB Saturation
HI541WriteCmosSensor(0x0FD2, 0xa0a0);// 10?? ???? CB Saturation

// CR Saturation
HI541WriteCmosSensor(0x0FD4, 0x8080);// 0?? ???? CR Saturation
		HI541WriteCmosSensor(0x0FD6,0x8088);// 2?? ???? CR Saturation
		HI541WriteCmosSensor(0x0FD8,0x9090);// 4?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDA, 0x9a98);// 6?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDC, 0x8ca0);// 8?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDE, 0xa0a0);// 10?? ???? CR Saturation
//================================================

// AUTO
//================================
//=========== ADP Start ==========
//================================

HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0DB0, 0x0000); // ADP B[0]On/Off // B[1] DMA  On/Off // ADP Std Copy

//===================================================
//========= Luminance Adaptive ISP Start  ===========
//===================================================
HI541WriteCmosSensor(0xffff, 0x0020);

// Outdoor            
HI541WriteCmosSensor(0x12B6, 0x0000); // 0x7034 : pre_flat_ctl, 	0x7035 : flat_max_luml1
HI541WriteCmosSensor(0x1304, 0x1B68); // 0x7837 : spstd_gain_8,	0x7838 : post_std_sel //20140522 skin detail 79 -> 68 -> 46             
HI541WriteCmosSensor(0x1352, 0x1a25); // 0x940b : ynr_flat_ctl,	0x9414 : ynr_flat_luml1
HI541WriteCmosSensor(0x1372, 0x0C18); // 0x9809 : yee_skin_higgain,	0x980a : yee_std_post_gain_pos //22->14->16->14    
HI541WriteCmosSensor(0x1374, 0x1626); // 0x980b : yee_std_gain,	0x980c : yee_lum_gain_p1 //24->0c->14                  
HI541WriteCmosSensor(0x13EE, 0x1812); // 0x7c84 : ci_spstd_8,		0x7c85 : ci_post_std_ctl

// Indoor
HI541WriteCmosSensor(0x14AC, 0x030D); // INDOOR 0x9000 : Saturation ENB Transfer Function , 0x7034 : pre_flat_ctl //20140626
HI541WriteCmosSensor(0x14FC, 0xcb16); // INDOOR 0x7838 : post_std_sel,	0x7440 : flt_luml1, //20140627 post neg/pos 87 -> 66
HI541WriteCmosSensor(0x1548, 0x0218); // INDOOR 0x940a : ynr_line_rate,   0x940b : ynr_flat_ctl, //20140626 rate 00
HI541WriteCmosSensor(0x156A, 0x1718); // INDOOR 0x980a : yee_std_post_gain_pos, 0x980b : yee_std_gain,	
HI541WriteCmosSensor(0x15E6, 0x3400); // INDOOR 0x7c85 : ci_post_std_ctl, 0x7c97 : ci_skin_blue_ctl,

// Dark1
HI541WriteCmosSensor(0x16A4, 0x1F00); // Dark1 0x7034 : pre_flat_ctl, 	0x7035 : flat_max_luml1
HI541WriteCmosSensor(0x16F2, 0x1898); // Dark1 0x7837 : spstd_gain_8,     0x7838 : post_std_sel
HI541WriteCmosSensor(0x1760, 0x0C12); // Dark1 0x9809 : yee_skin_higgain,	0x980a : yee_std_post_gain_pos
HI541WriteCmosSensor(0x1762, 0x1214); // Dark1 0x980b : yee_std_gain,	0x980c : yee_lum_gain_p1
HI541WriteCmosSensor(0x1740, 0x1836); // Dark1 0x940b : ynr_flat_ctl,	0x9414 : ynr_flat_luml1
HI541WriteCmosSensor(0x17DC, 0x1A36); // Dark1 0x7c84 : ci_spstd_8,	0x7c85 : ci_post_std_ctl

// Dark2
HI541WriteCmosSensor(0x189A, 0x001F);  // Dark2 0x9000 : Saturation ENB Transfer Function , 0x7034 : pre_flat_ctl 	
HI541WriteCmosSensor(0x18EA, 0x9800);  // Dark2 0x7838 : post_std_sel,	0x7440 : flt_luml1,
HI541WriteCmosSensor(0x1936, 0x0218);  // Dark2 0x940a : ynr_line_rate,   0x940b : ynr_flat_ctl,	 
HI541WriteCmosSensor(0x1958, 0x1212);  // Dark2 0x980a : yee_std_post_gain_pos,0x980b : yee_std_gain,	//20140628
HI541WriteCmosSensor(0x19D4, 0x3600);  // Dark2 0x7c85 : ci_post_std_ctl, 0x7c97 : ci_skin_blue_ctl,

HI541WriteCmosSensor(0x0DB0, 0x0300); // ADP B[0]On/Off // B[1] DMA  On/Off // ADP Std Copy
HI541WriteCmosSensor(0x0DB0, 0x0300); // ADP B[0]On/Off // B[1] DMA  On/Off // ADP Std Copy
HI541WriteCmosSensor(0x12b0, 0x0520);
HI541WriteCmosSensor(0x1b1a, 0x0f00);
HI541WriteCmosSensor(0xffff, 0x0040);
			    
			         break;
        case SCENE_MODE_PORTRAIT:
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0DB0, 0x0000);// ADP B[0]On/Off // B[1] DMAOn/Off // ADP Std Copy
 

HI541WriteCmosSensor(0x0DCC, 0x00b2);// EV TH (Dark2) 10fps AGx8
HI541WriteCmosSensor(0x0DCE, 0x0002);

HI541WriteCmosSensor(0x0DD0, 0x0059);// EV TH (Dark1) 10fps AGx4
HI541WriteCmosSensor(0x0DD2, 0x0001);

HI541WriteCmosSensor(0x0DD4, 0xf508);// EV TH (Indoor) 33.33fps AGx1.25
HI541WriteCmosSensor(0x0DD6, 0x1800);

HI541WriteCmosSensor(0x0DD8, 0x10a4);// EV TH (Outdoor) 1000fps AGx1
HI541WriteCmosSensor(0x0DDA, 0x0000);
//===============================================
//10fps

HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0384, 0x4016);//L
HI541WriteCmosSensor(0x0386, 0x4000);//H exposure max 100 10fps
HI541WriteCmosSensor(0x0388, 0x4016);//L
HI541WriteCmosSensor(0x038a, 0x4000);//H exposure max 120 10fps
//================================================

//===============================================
//ISO AUTO 
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x02ee, 0x2020);//AG th0,1
HI541WriteCmosSensor(0x02f0, 0x2020);//AG th2,3
HI541WriteCmosSensor(0x02f2, 0xff00);//AG th4,

//===============================================
 
//===============================================
//AE weight table_Center
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0448, 0x1111);
HI541WriteCmosSensor(0x044a, 0x1111);
HI541WriteCmosSensor(0x044c, 0x1111);
HI541WriteCmosSensor(0x044e, 0x1111);
HI541WriteCmosSensor(0x0450, 0x1111);
HI541WriteCmosSensor(0x0452, 0x1111);
HI541WriteCmosSensor(0x0454, 0x1111);
HI541WriteCmosSensor(0x0456, 0x1111);
HI541WriteCmosSensor(0x0458, 0x1111);
HI541WriteCmosSensor(0x045a, 0x1111);
HI541WriteCmosSensor(0x045c, 0x1111);
HI541WriteCmosSensor(0x045e, 0x1111);
HI541WriteCmosSensor(0x0460, 0x1111);
HI541WriteCmosSensor(0x0462, 0x1111);
HI541WriteCmosSensor(0x0464, 0x1111);
HI541WriteCmosSensor(0x0466, 0x1111);
HI541WriteCmosSensor(0x0468, 0x1111);
HI541WriteCmosSensor(0x046a, 0x4444);
HI541WriteCmosSensor(0x046c, 0x4444);
HI541WriteCmosSensor(0x046e, 0x1111);
HI541WriteCmosSensor(0x0470, 0x1111);
HI541WriteCmosSensor(0x0472, 0x4444);
HI541WriteCmosSensor(0x0474, 0x4444);
HI541WriteCmosSensor(0x0476, 0x1111);
HI541WriteCmosSensor(0x0478, 0x1111);
HI541WriteCmosSensor(0x047a, 0x4444);
HI541WriteCmosSensor(0x047c, 0x4444);
HI541WriteCmosSensor(0x047e, 0x1111);
HI541WriteCmosSensor(0x0480, 0x1111);
HI541WriteCmosSensor(0x0482, 0x4455);
HI541WriteCmosSensor(0x0484, 0x5544);
HI541WriteCmosSensor(0x0486, 0x1111);
HI541WriteCmosSensor(0x0488, 0x1111);
HI541WriteCmosSensor(0x048a, 0x4455);
HI541WriteCmosSensor(0x048c, 0x5544);
HI541WriteCmosSensor(0x048e, 0x1111);
HI541WriteCmosSensor(0x0490, 0x1111);
HI541WriteCmosSensor(0x0492, 0x4444);
HI541WriteCmosSensor(0x0494, 0x4444);
HI541WriteCmosSensor(0x0496, 0x1111);
HI541WriteCmosSensor(0x0498, 0x1111);
HI541WriteCmosSensor(0x049a, 0x4444);
HI541WriteCmosSensor(0x049c, 0x4444);
HI541WriteCmosSensor(0x049e, 0x1111);
HI541WriteCmosSensor(0x04a0, 0x1111);
HI541WriteCmosSensor(0x04a2, 0x4444);
HI541WriteCmosSensor(0x04a4, 0x4444);
HI541WriteCmosSensor(0x04a6, 0x1111);
HI541WriteCmosSensor(0x04a8, 0x1111);
HI541WriteCmosSensor(0x04aa, 0x4444);
HI541WriteCmosSensor(0x04ac, 0x4444);
HI541WriteCmosSensor(0x04ae, 0x1111);
HI541WriteCmosSensor(0x04b0, 0x3333);
HI541WriteCmosSensor(0x04b2, 0x3333);
HI541WriteCmosSensor(0x04b4, 0x3333);
HI541WriteCmosSensor(0x04b6, 0x3333);
HI541WriteCmosSensor(0x04b8, 0x3333);
HI541WriteCmosSensor(0x04ba, 0x3333);
HI541WriteCmosSensor(0x04bc, 0x3333);
HI541WriteCmosSensor(0x04be, 0x3333);
HI541WriteCmosSensor(0x04c0, 0x3333);
HI541WriteCmosSensor(0x04c2, 0x3333);
HI541WriteCmosSensor(0x04c4, 0x3333);
HI541WriteCmosSensor(0x04c6, 0x3333);
//================================================ 
 
 
//===============================================
//AWB_Auto
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0a0c, 0x1010); //Rmin,Bmin
HI541WriteCmosSensor(0x0a0e, 0xc0c0); //Rmax,Bmax 
HI541WriteCmosSensor(0xffff, 0x0040);
//===============================================
 
 
//===============================================
// SAT ORI 
HI541WriteCmosSensor(0xffff, 0x0020);
// CB Saturation
HI541WriteCmosSensor(0x0FC8, 0x8080);// 0?? ???? CB Saturation
		HI541WriteCmosSensor(0x0FCA,0x8088);// 2?? ???? CB Saturation
		HI541WriteCmosSensor(0x0FCC,0x9090);// 4?? ???? CB Saturation
HI541WriteCmosSensor(0x0FCE, 0x9a9a);// 6?? ???? CB Saturation
HI541WriteCmosSensor(0x0FD0, 0x9ca0);// 8?? ???? CB Saturation
HI541WriteCmosSensor(0x0FD2, 0xa0a0);// 10?? ???? CB Saturation

// CR Saturation
HI541WriteCmosSensor(0x0FD4, 0x8080);// 0?? ???? CR Saturation
		HI541WriteCmosSensor(0x0FD6,0x8088);// 2?? ???? CR Saturation
		HI541WriteCmosSensor(0x0FD8,0x9090);// 4?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDA, 0x9a98);// 6?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDC, 0x8ca0);// 8?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDE, 0xa0a0);// 10?? ???? CR Saturation
//================================================ 
 
// SHARP DOWN
HI541WriteCmosSensor(0xffff, 0x0020);
//===================================================
//========= Luminance Adaptive ISP Start  ===========
//===================================================
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0DB0, 0x0000); // ADP B[0]On/Off // B[1] DMA  On/Off // ADP Std Copy


// Outdoor            
HI541WriteCmosSensor(0x12B6, 0x1f00); // 0x7034 : pre_flat_ctl, 	0x7035 : flat_max_luml1
HI541WriteCmosSensor(0x1304, 0x1B00); // 0x7837 : spstd_gain_8,	0x7838 : post_std_sel //20140522 skin detail 79 -> 68 -> 46             
HI541WriteCmosSensor(0x1352, 0x1f25); // 0x940b : ynr_flat_ctl,	0x9414 : ynr_flat_luml1
HI541WriteCmosSensor(0x1372, 0x0C00); // 0x9809 : yee_skin_higgain,	0x980a : yee_std_post_gain_pos //22->14->16->14    
HI541WriteCmosSensor(0x1374, 0x0026); // 0x980b : yee_std_gain,	0x980c : yee_lum_gain_p1 //24->0c->14                  
HI541WriteCmosSensor(0x13EE, 0x1800); // 0x7c84 : ci_spstd_8,		0x7c85 : ci_post_std_ctl

// Indoor
HI541WriteCmosSensor(0x14AC, 0x031f); // INDOOR 0x9000 : Saturation ENB Transfer Function , 0x7034 : pre_flat_ctl //20140626
HI541WriteCmosSensor(0x14FC, 0x0016); // INDOOR 0x7838 : post_std_sel,	0x7440 : flt_luml1, //20140627 post neg/pos 87 -> 66
HI541WriteCmosSensor(0x1548, 0x021f); // INDOOR 0x940a : ynr_line_rate,   0x940b : ynr_flat_ctl, //20140626 rate 00
HI541WriteCmosSensor(0x156A, 0x0000); // INDOOR 0x980a : yee_std_post_gain_pos, 0x980b : yee_std_gain,	
HI541WriteCmosSensor(0x15E6, 0x0000); // INDOOR 0x7c85 : ci_post_std_ctl, 0x7c97 : ci_skin_blue_ctl,

// Dark1
HI541WriteCmosSensor(0x16A4, 0x1F00); // Dark1 0x7034 : pre_flat_ctl, 	0x7035 : flat_max_luml1
HI541WriteCmosSensor(0x16F2, 0x1800); // Dark1 0x7837 : spstd_gain_8,     0x7838 : post_std_sel
HI541WriteCmosSensor(0x1760, 0x0C00); // Dark1 0x9809 : yee_skin_higgain,	0x980a : yee_std_post_gain_pos
HI541WriteCmosSensor(0x1762, 0x0014); // Dark1 0x980b : yee_std_gain,	0x980c : yee_lum_gain_p1
HI541WriteCmosSensor(0x1740, 0x1f36); // Dark1 0x940b : ynr_flat_ctl,	0x9414 : ynr_flat_luml1
HI541WriteCmosSensor(0x17DC, 0x1A00); // Dark1 0x7c84 : ci_spstd_8,	0x7c85 : ci_post_std_ctl

// Dark2
HI541WriteCmosSensor(0x189A, 0x001F);  // Dark2 0x9000 : Saturation ENB Transfer Function , 0x7034 : pre_flat_ctl 	
HI541WriteCmosSensor(0x18EA, 0x0000);  // Dark2 0x7838 : post_std_sel,	0x7440 : flt_luml1,
HI541WriteCmosSensor(0x1936, 0x021f);  // Dark2 0x940a : ynr_line_rate,   0x940b : ynr_flat_ctl,	 
HI541WriteCmosSensor(0x1958, 0x0000);  // Dark2 0x980a : yee_std_post_gain_pos,0x980b : yee_std_gain,	//20140628
HI541WriteCmosSensor(0x19D4, 0x0000);  // Dark2 0x7c85 : ci_post_std_ctl, 0x7c97 : ci_skin_blue_ctl,

HI541WriteCmosSensor(0x0DB0, 0x0300); // ADP B[0]On/Off // B[1] DMA  On/Off // ADP Std Copy

HI541WriteCmosSensor(0x12b0, 0x0520);
HI541WriteCmosSensor(0x1b1a, 0x0f00);
HI541WriteCmosSensor(0xffff, 0x0040);
			        
               break;
        case SCENE_MODE_LANDSCAPE:
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0DB0, 0x0000);// ADP B[0]On/Off // B[1] DMAOn/Off // ADP Std Copy
 

HI541WriteCmosSensor(0x0DCC, 0x00b2);// EV TH (Dark2) 10fps AGx8
HI541WriteCmosSensor(0x0DCE, 0x0002);

HI541WriteCmosSensor(0x0DD0, 0x0059);// EV TH (Dark1) 10fps AGx4
HI541WriteCmosSensor(0x0DD2, 0x0001);

HI541WriteCmosSensor(0x0DD4, 0xf508);// EV TH (Indoor) 33.33fps AGx1.25
HI541WriteCmosSensor(0x0DD6, 0x1800);

HI541WriteCmosSensor(0x0DD8, 0x10a4);// EV TH (Outdoor) 1000fps AGx1
HI541WriteCmosSensor(0x0DDA, 0x0000);
//===============================================
//10fps

HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0384, 0x4016);//L
HI541WriteCmosSensor(0x0386, 0x4000);//H exposure max 100 10fps
HI541WriteCmosSensor(0x0388, 0x4016);//L
HI541WriteCmosSensor(0x038a, 0x4000);//H exposure max 120 10fps
//================================================

 
//===============================================
//ISO AUTO 
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x02ee, 0x2020);//AG th0,1
HI541WriteCmosSensor(0x02f0, 0x2020);//AG th2,3
HI541WriteCmosSensor(0x02f2, 0xff00);//AG th4, 

//=============================================== 

//=============================================== 
//AE Weight_matrix
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0448, 0x1111);
HI541WriteCmosSensor(0x044a, 0x1111);
HI541WriteCmosSensor(0x044c, 0x1111);
HI541WriteCmosSensor(0x044e, 0x1111);
HI541WriteCmosSensor(0x0450, 0x1111);
HI541WriteCmosSensor(0x0452, 0x1111);
HI541WriteCmosSensor(0x0454, 0x1111);
HI541WriteCmosSensor(0x0456, 0x1111);
HI541WriteCmosSensor(0x0458, 0x1111);
HI541WriteCmosSensor(0x045a, 0x1111);
HI541WriteCmosSensor(0x045c, 0x1111);
HI541WriteCmosSensor(0x045e, 0x1111);
HI541WriteCmosSensor(0x0460, 0x1111);
HI541WriteCmosSensor(0x0462, 0x1111);
HI541WriteCmosSensor(0x0464, 0x1111);
HI541WriteCmosSensor(0x0466, 0x1111);
HI541WriteCmosSensor(0x0468, 0x1111);
HI541WriteCmosSensor(0x046a, 0x1111);
HI541WriteCmosSensor(0x046c, 0x1111);
HI541WriteCmosSensor(0x046e, 0x1111);
HI541WriteCmosSensor(0x0470, 0x1111);
HI541WriteCmosSensor(0x0472, 0x1111);
HI541WriteCmosSensor(0x0474, 0x1111);
HI541WriteCmosSensor(0x0476, 0x1111);
HI541WriteCmosSensor(0x0478, 0x1111);
HI541WriteCmosSensor(0x047a, 0x1111);
HI541WriteCmosSensor(0x047c, 0x1111);
HI541WriteCmosSensor(0x047e, 0x1111);
HI541WriteCmosSensor(0x0480, 0x1111);
HI541WriteCmosSensor(0x0482, 0x1111);
HI541WriteCmosSensor(0x0484, 0x1111);
HI541WriteCmosSensor(0x0486, 0x1111);
HI541WriteCmosSensor(0x0488, 0x1111);
HI541WriteCmosSensor(0x048a, 0x1111);
HI541WriteCmosSensor(0x048c, 0x1111);
HI541WriteCmosSensor(0x048e, 0x1111);
HI541WriteCmosSensor(0x0490, 0x1111);
HI541WriteCmosSensor(0x0492, 0x1111);
HI541WriteCmosSensor(0x0494, 0x1111);
HI541WriteCmosSensor(0x0496, 0x1111);
HI541WriteCmosSensor(0x0498, 0x1111);
HI541WriteCmosSensor(0x049a, 0x1111);
HI541WriteCmosSensor(0x049c, 0x1111);
HI541WriteCmosSensor(0x049e, 0x1111);
HI541WriteCmosSensor(0x04a0, 0x1111);
HI541WriteCmosSensor(0x04a2, 0x1111);
HI541WriteCmosSensor(0x04a4, 0x1111);
HI541WriteCmosSensor(0x04a6, 0x1111);
HI541WriteCmosSensor(0x04a8, 0x1111);
HI541WriteCmosSensor(0x04aa, 0x1111);
HI541WriteCmosSensor(0x04ac, 0x1111);
HI541WriteCmosSensor(0x04ae, 0x1111);
HI541WriteCmosSensor(0x04b0, 0x1111);
HI541WriteCmosSensor(0x04b2, 0x1111);
HI541WriteCmosSensor(0x04b4, 0x1111);
HI541WriteCmosSensor(0x04b6, 0x1111);
HI541WriteCmosSensor(0x04b8, 0x1111);
HI541WriteCmosSensor(0x04ba, 0x1111);
HI541WriteCmosSensor(0x04bc, 0x1111);
HI541WriteCmosSensor(0x04be, 0x1111);
HI541WriteCmosSensor(0x04c0, 0x1111);
HI541WriteCmosSensor(0x04c2, 0x1111);
HI541WriteCmosSensor(0x04c4, 0x1111);
HI541WriteCmosSensor(0x04c6, 0x1111);
//================================================


//=============================================== 
//AWB_Auto
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0a0c, 0x1010); //Rmin,Bmin
HI541WriteCmosSensor(0x0a0e, 0xc0c0); //Rmax,Bmax 
HI541WriteCmosSensor(0xffff, 0x0040);
//=============================================== 


//=============================================== 
// SAT Enhance
HI541WriteCmosSensor(0xffff, 0x0020);
// CB Saturation
HI541WriteCmosSensor(0x0FC8, 0x8888);// 0?? ???? CB Saturation
		HI541WriteCmosSensor(0x0FCA,0x888a);// 2?? ???? CB Saturation
HI541WriteCmosSensor(0x0FCC, 0xa0a0);// 4?? ???? CB Saturation
HI541WriteCmosSensor(0x0FCE, 0xa8a8);// 6?? ???? CB Saturation
HI541WriteCmosSensor(0x0FD0, 0xb8b8);// 8?? ???? CB Saturation
HI541WriteCmosSensor(0x0FD2, 0xb8b8);// 10?? ???? CB Saturation

// CR Saturation
HI541WriteCmosSensor(0x0FD4, 0x8888);// 0?? ???? CR Saturation
HI541WriteCmosSensor(0x0FD6, 0x8880);// 2?? ???? CR Saturation
HI541WriteCmosSensor(0x0FD8, 0xa0a0);// 4?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDA, 0xb8b8);// 6?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDC, 0xb8b8);// 8?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDE, 0xb8b8);// 10?? ???? CR Saturation
//================================================

// SHARP UP 
//===================================================
//========= Luminance Adaptive ISP Start  ===========
//===================================================
HI541WriteCmosSensor(0xffff, 0x0020);

HI541WriteCmosSensor(0x0DB0, 0x0000); // ADP B[0]On/Off // B[1] DMA  On/Off // ADP Std Copy


// Outdoor            
HI541WriteCmosSensor(0x12B6, 0x0300); // 0x7034 : pre_flat_ctl, 	0x7035 : flat_max_luml1
HI541WriteCmosSensor(0x1304, 0x1Bff); // 0x7837 : spstd_gain_8,	0x7838 : post_std_sel //20140522 skin detail 79 -> 68 -> 46             
HI541WriteCmosSensor(0x1352, 0x1325); // 0x940b : ynr_flat_ctl,	0x9414 : ynr_flat_luml1
HI541WriteCmosSensor(0x1372, 0x0Cff); // 0x9809 : yee_skin_higgain,	0x980a : yee_std_post_gain_pos //22->14->16->14    
HI541WriteCmosSensor(0x1374, 0xff26); // 0x980b : yee_std_gain,	0x980c : yee_lum_gain_p1 //24->0c->14                  
HI541WriteCmosSensor(0x13EE, 0x18ff); // 0x7c84 : ci_spstd_8,		0x7c85 : ci_post_std_ctl

// Indoor
HI541WriteCmosSensor(0x14AC, 0x0303); // INDOOR 0x9000 : Saturation ENB Transfer Function , 0x7034 : pre_flat_ctl //20140626
HI541WriteCmosSensor(0x14FC, 0xff16); // INDOOR 0x7838 : post_std_sel,	0x7440 : flt_luml1, //20140627 post neg/pos 87 -> 66
HI541WriteCmosSensor(0x1548, 0x0213); // INDOOR 0x940a : ynr_line_rate,   0x940b : ynr_flat_ctl, //20140626 rate 00
HI541WriteCmosSensor(0x156A, 0xffff); // INDOOR 0x980a : yee_std_post_gain_pos, 0x980b : yee_std_gain,	
HI541WriteCmosSensor(0x15E6, 0xff00); // INDOOR 0x7c85 : ci_post_std_ctl, 0x7c97 : ci_skin_blue_ctl,

// Dark1
HI541WriteCmosSensor(0x16A4, 0x0300); // Dark1 0x7034 : pre_flat_ctl, 	0x7035 : flat_max_luml1
HI541WriteCmosSensor(0x16F2, 0x18ff); // Dark1 0x7837 : spstd_gain_8,     0x7838 : post_std_sel
HI541WriteCmosSensor(0x1760, 0x0C80); // Dark1 0x9809 : yee_skin_higgain,	0x980a : yee_std_post_gain_pos
HI541WriteCmosSensor(0x1762, 0x8014); // Dark1 0x980b : yee_std_gain,	0x980c : yee_lum_gain_p1
HI541WriteCmosSensor(0x1740, 0x1836); // Dark1 0x940b : ynr_flat_ctl,	0x9414 : ynr_flat_luml1
HI541WriteCmosSensor(0x17DC, 0x8036); // Dark1 0x7c84 : ci_spstd_8,	0x7c85 : ci_post_std_ctl

// Dark2
HI541WriteCmosSensor(0x189A, 0x001F);  // Dark2 0x9000 : Saturation ENB Transfer Function , 0x7034 : pre_flat_ctl 	
HI541WriteCmosSensor(0x18EA, 0x9800);  // Dark2 0x7838 : post_std_sel,	0x7440 : flt_luml1,
HI541WriteCmosSensor(0x1936, 0x0218);  // Dark2 0x940a : ynr_line_rate,   0x940b : ynr_flat_ctl,	 
HI541WriteCmosSensor(0x1958, 0x1212);  // Dark2 0x980a : yee_std_post_gain_pos,0x980b : yee_std_gain,	//20140628
HI541WriteCmosSensor(0x19D4, 0x3600);  // Dark2 0x7c85 : ci_post_std_ctl, 0x7c97 : ci_skin_blue_ctl,

HI541WriteCmosSensor(0x0DB0, 0x0300); // ADP B[0]On/Off // B[1] DMA  On/Off // ADP Std Copy
HI541WriteCmosSensor(0x12b0, 0x0520);
HI541WriteCmosSensor(0x1b1a, 0x0f00);
HI541WriteCmosSensor(0xffff, 0x0040);

			    
               break;
        case SCENE_MODE_SUNSET:
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0DB0, 0x0000);// ADP B[0]On/Off // B[1] DMAOn/Off // ADP Std Copy 


HI541WriteCmosSensor(0x0DCC, 0x00b2);// EV TH (Dark2) 10fps AGx8
HI541WriteCmosSensor(0x0DCE, 0x0002);

HI541WriteCmosSensor(0x0DD0, 0x0059);// EV TH (Dark1) 10fps AGx4
HI541WriteCmosSensor(0x0DD2, 0x0001);

HI541WriteCmosSensor(0x0DD4, 0xf508);// EV TH (Indoor) 33.33fps AGx1.25
HI541WriteCmosSensor(0x0DD6, 0x1800);

HI541WriteCmosSensor(0x0DD8, 0x10a4);// EV TH (Outdoor) 1000fps AGx1
HI541WriteCmosSensor(0x0DDA, 0x0000);
//===============================================
//10fps

HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0384, 0x4016);//L
HI541WriteCmosSensor(0x0386, 0x4000);//H exposure max 100 10fps
HI541WriteCmosSensor(0x0388, 0x4016);//L
HI541WriteCmosSensor(0x038a, 0x4000);//H exposure max 120 10fps
//================================================


//=============================================== 
//ISO AUTO
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x02ee, 0x2020);//AG th0,1
HI541WriteCmosSensor(0x02f0, 0x2020);//AG th2,3
HI541WriteCmosSensor(0x02f2, 0xff00);//AG th4,
//===============================================
 
//===============================================
//AE weight table_Center
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0448, 0x1111);
HI541WriteCmosSensor(0x044a, 0x1111);
HI541WriteCmosSensor(0x044c, 0x1111);
HI541WriteCmosSensor(0x044e, 0x1111);
HI541WriteCmosSensor(0x0450, 0x1111);
HI541WriteCmosSensor(0x0452, 0x1111);
HI541WriteCmosSensor(0x0454, 0x1111);
HI541WriteCmosSensor(0x0456, 0x1111);
HI541WriteCmosSensor(0x0458, 0x1111);
HI541WriteCmosSensor(0x045a, 0x1111);
HI541WriteCmosSensor(0x045c, 0x1111);
HI541WriteCmosSensor(0x045e, 0x1111);
HI541WriteCmosSensor(0x0460, 0x1111);
HI541WriteCmosSensor(0x0462, 0x1111);
HI541WriteCmosSensor(0x0464, 0x1111);
HI541WriteCmosSensor(0x0466, 0x1111);
HI541WriteCmosSensor(0x0468, 0x1111);
HI541WriteCmosSensor(0x046a, 0x4444);
HI541WriteCmosSensor(0x046c, 0x4444);
HI541WriteCmosSensor(0x046e, 0x1111);
HI541WriteCmosSensor(0x0470, 0x1111);
HI541WriteCmosSensor(0x0472, 0x4444);
HI541WriteCmosSensor(0x0474, 0x4444);
HI541WriteCmosSensor(0x0476, 0x1111);
HI541WriteCmosSensor(0x0478, 0x1111);
HI541WriteCmosSensor(0x047a, 0x4444);
HI541WriteCmosSensor(0x047c, 0x4444);
HI541WriteCmosSensor(0x047e, 0x1111);
HI541WriteCmosSensor(0x0480, 0x1111);
HI541WriteCmosSensor(0x0482, 0x4455);
HI541WriteCmosSensor(0x0484, 0x5544);
HI541WriteCmosSensor(0x0486, 0x1111);
HI541WriteCmosSensor(0x0488, 0x1111);
HI541WriteCmosSensor(0x048a, 0x4455);
HI541WriteCmosSensor(0x048c, 0x5544);
HI541WriteCmosSensor(0x048e, 0x1111);
HI541WriteCmosSensor(0x0490, 0x1111);
HI541WriteCmosSensor(0x0492, 0x4444);
HI541WriteCmosSensor(0x0494, 0x4444);
HI541WriteCmosSensor(0x0496, 0x1111);
HI541WriteCmosSensor(0x0498, 0x1111);
HI541WriteCmosSensor(0x049a, 0x4444);
HI541WriteCmosSensor(0x049c, 0x4444);
HI541WriteCmosSensor(0x049e, 0x1111);
HI541WriteCmosSensor(0x04a0, 0x1111);
HI541WriteCmosSensor(0x04a2, 0x4444);
HI541WriteCmosSensor(0x04a4, 0x4444);
HI541WriteCmosSensor(0x04a6, 0x1111);
HI541WriteCmosSensor(0x04a8, 0x1111);
HI541WriteCmosSensor(0x04aa, 0x4444);
HI541WriteCmosSensor(0x04ac, 0x4444);
HI541WriteCmosSensor(0x04ae, 0x1111);
HI541WriteCmosSensor(0x04b0, 0x3333);
HI541WriteCmosSensor(0x04b2, 0x3333);
HI541WriteCmosSensor(0x04b4, 0x3333);
HI541WriteCmosSensor(0x04b6, 0x3333);
HI541WriteCmosSensor(0x04b8, 0x3333);
HI541WriteCmosSensor(0x04ba, 0x3333);
HI541WriteCmosSensor(0x04bc, 0x3333);
HI541WriteCmosSensor(0x04be, 0x3333);
HI541WriteCmosSensor(0x04c0, 0x3333);
HI541WriteCmosSensor(0x04c2, 0x3333);
HI541WriteCmosSensor(0x04c4, 0x3333);
HI541WriteCmosSensor(0x04c6, 0x3333);
//================================================ 
 
 
//===============================================
//TL84
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0a0c, 0x457b); //Rmin,Bmin
HI541WriteCmosSensor(0x0a0e, 0x4f85); //Rmax,Bmax 
HI541WriteCmosSensor(0xffff, 0x0040);
//===============================================
 
 
//===============================================
// SAT ORI 
HI541WriteCmosSensor(0xffff, 0x0020);
// CB Saturation
HI541WriteCmosSensor(0x0FC8, 0x8080);// 0?? ???? CB Saturation
		HI541WriteCmosSensor(0x0FCA,0x8088);// 2?? ???? CB Saturation
		HI541WriteCmosSensor(0x0FCC,0x9090);// 4?? ???? CB Saturation
HI541WriteCmosSensor(0x0FCE, 0x9a9a);// 6?? ???? CB Saturation
HI541WriteCmosSensor(0x0FD0, 0x9ca0);// 8?? ???? CB Saturation
HI541WriteCmosSensor(0x0FD2, 0xa0a0);// 10?? ???? CB Saturation

// CR Saturation
HI541WriteCmosSensor(0x0FD4, 0x8080);// 0?? ???? CR Saturation
		HI541WriteCmosSensor(0x0FD6,0x8088);// 2?? ???? CR Saturation
		HI541WriteCmosSensor(0x0FD8,0x9090);// 4?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDA, 0x9a98);// 6?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDC, 0x8ca0);// 8?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDE, 0xa0a0);// 10?? ???? CR Saturation
//================================================ 
 
// AUTO
//================================
//=========== ADP Start ==========
//================================

HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0DB0, 0x0000); // ADP B[0]On/Off // B[1] DMA  On/Off // ADP Std Copy

//===================================================
//========= Luminance Adaptive ISP Start  ===========
//===================================================
HI541WriteCmosSensor(0xffff, 0x0020);

// Outdoor            
HI541WriteCmosSensor(0x12B6, 0x0000); // 0x7034 : pre_flat_ctl, 	0x7035 : flat_max_luml1
HI541WriteCmosSensor(0x1304, 0x1B68); // 0x7837 : spstd_gain_8,	0x7838 : post_std_sel //20140522 skin detail 79 -> 68 -> 46             
HI541WriteCmosSensor(0x1352, 0x1a25); // 0x940b : ynr_flat_ctl,	0x9414 : ynr_flat_luml1
HI541WriteCmosSensor(0x1372, 0x0C18); // 0x9809 : yee_skin_higgain,	0x980a : yee_std_post_gain_pos //22->14->16->14    
HI541WriteCmosSensor(0x1374, 0x1626); // 0x980b : yee_std_gain,	0x980c : yee_lum_gain_p1 //24->0c->14                  
HI541WriteCmosSensor(0x13EE, 0x1812); // 0x7c84 : ci_spstd_8,		0x7c85 : ci_post_std_ctl

// Indoor
HI541WriteCmosSensor(0x14AC, 0x030D); // INDOOR 0x9000 : Saturation ENB Transfer Function , 0x7034 : pre_flat_ctl //20140626
HI541WriteCmosSensor(0x14FC, 0xcb16); // INDOOR 0x7838 : post_std_sel,	0x7440 : flt_luml1, //20140627 post neg/pos 87 -> 66
HI541WriteCmosSensor(0x1548, 0x0218); // INDOOR 0x940a : ynr_line_rate,   0x940b : ynr_flat_ctl, //20140626 rate 00
HI541WriteCmosSensor(0x156A, 0x1718); // INDOOR 0x980a : yee_std_post_gain_pos, 0x980b : yee_std_gain,	
HI541WriteCmosSensor(0x15E6, 0x3400); // INDOOR 0x7c85 : ci_post_std_ctl, 0x7c97 : ci_skin_blue_ctl,

// Dark1
HI541WriteCmosSensor(0x16A4, 0x1F00); // Dark1 0x7034 : pre_flat_ctl, 	0x7035 : flat_max_luml1
HI541WriteCmosSensor(0x16F2, 0x1898); // Dark1 0x7837 : spstd_gain_8,     0x7838 : post_std_sel
HI541WriteCmosSensor(0x1760, 0x0C12); // Dark1 0x9809 : yee_skin_higgain,	0x980a : yee_std_post_gain_pos
HI541WriteCmosSensor(0x1762, 0x1214); // Dark1 0x980b : yee_std_gain,	0x980c : yee_lum_gain_p1
HI541WriteCmosSensor(0x1740, 0x1836); // Dark1 0x940b : ynr_flat_ctl,	0x9414 : ynr_flat_luml1
HI541WriteCmosSensor(0x17DC, 0x1A36); // Dark1 0x7c84 : ci_spstd_8,	0x7c85 : ci_post_std_ctl

// Dark2
HI541WriteCmosSensor(0x189A, 0x001F);  // Dark2 0x9000 : Saturation ENB Transfer Function , 0x7034 : pre_flat_ctl 	
HI541WriteCmosSensor(0x18EA, 0x9800);  // Dark2 0x7838 : post_std_sel,	0x7440 : flt_luml1,
HI541WriteCmosSensor(0x1936, 0x0218);  // Dark2 0x940a : ynr_line_rate,   0x940b : ynr_flat_ctl,	 
HI541WriteCmosSensor(0x1958, 0x1212);  // Dark2 0x980a : yee_std_post_gain_pos,0x980b : yee_std_gain,	//20140628
HI541WriteCmosSensor(0x19D4, 0x3600);  // Dark2 0x7c85 : ci_post_std_ctl, 0x7c97 : ci_skin_blue_ctl,

HI541WriteCmosSensor(0x0DB0, 0x0300); // ADP B[0]On/Off // B[1] DMA  On/Off // ADP Std Copy
HI541WriteCmosSensor(0x0DB0, 0x0300); // ADP B[0]On/Off // B[1] DMA  On/Off // ADP Std Copy
HI541WriteCmosSensor(0x12b0, 0x0520);
HI541WriteCmosSensor(0x1b1a, 0x0f00);
HI541WriteCmosSensor(0xffff, 0x0040);
			        
               break;
        case SCENE_MODE_SPORTS:
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0DB0, 0x0000);// ADP B[0]On/Off // B[1] DMAOn/Off // ADP Std Copy
 

HI541WriteCmosSensor(0x0DCC, 0x00b2);// EV TH (Dark2) 10fps AGx8
HI541WriteCmosSensor(0x0DCE, 0x0002);

HI541WriteCmosSensor(0x0DD0, 0x0059);// EV TH (Dark1) 10fps AGx4
HI541WriteCmosSensor(0x0DD2, 0x0001);

HI541WriteCmosSensor(0x0DD4, 0xf508);// EV TH (Indoor) 33fps AGx1.25
HI541WriteCmosSensor(0x0DD6, 0x1800);
//0A 27 39
HI541WriteCmosSensor(0x0DD8, 0x10a4);// EV TH (Outdoor) 1000fps AGx1
HI541WriteCmosSensor(0x0DDA, 0x0000);
//===============================================
//15fps

HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0384, 0xc073);//L
HI541WriteCmosSensor(0x0386, 0x2600);//H exposure max 100 16.67fps
HI541WriteCmosSensor(0x0388, 0x80b9);//L
HI541WriteCmosSensor(0x038a, 0x2a00);//H exposure max 120 15fps
//================================================

 
//===============================================
//ISO400 
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x02ee, 0x8080);//AG th0,1
HI541WriteCmosSensor(0x02f0, 0x8080);//AG th2,3
HI541WriteCmosSensor(0x02f2, 0xff00);//AG th4, 
//================================================ 
 
//===============================================
//AE weight table_Center
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0448, 0x1111);
HI541WriteCmosSensor(0x044a, 0x1111);
HI541WriteCmosSensor(0x044c, 0x1111);
HI541WriteCmosSensor(0x044e, 0x1111);
HI541WriteCmosSensor(0x0450, 0x1111);
HI541WriteCmosSensor(0x0452, 0x1111);
HI541WriteCmosSensor(0x0454, 0x1111);
HI541WriteCmosSensor(0x0456, 0x1111);
HI541WriteCmosSensor(0x0458, 0x1111);
HI541WriteCmosSensor(0x045a, 0x1111);
HI541WriteCmosSensor(0x045c, 0x1111);
HI541WriteCmosSensor(0x045e, 0x1111);
HI541WriteCmosSensor(0x0460, 0x1111);
HI541WriteCmosSensor(0x0462, 0x1111);
HI541WriteCmosSensor(0x0464, 0x1111);
HI541WriteCmosSensor(0x0466, 0x1111);
HI541WriteCmosSensor(0x0468, 0x1111);
HI541WriteCmosSensor(0x046a, 0x4444);
HI541WriteCmosSensor(0x046c, 0x4444);
HI541WriteCmosSensor(0x046e, 0x1111);
HI541WriteCmosSensor(0x0470, 0x1111);
HI541WriteCmosSensor(0x0472, 0x4444);
HI541WriteCmosSensor(0x0474, 0x4444);
HI541WriteCmosSensor(0x0476, 0x1111);
HI541WriteCmosSensor(0x0478, 0x1111);
HI541WriteCmosSensor(0x047a, 0x4444);
HI541WriteCmosSensor(0x047c, 0x4444);
HI541WriteCmosSensor(0x047e, 0x1111);
HI541WriteCmosSensor(0x0480, 0x1111);
HI541WriteCmosSensor(0x0482, 0x4455);
HI541WriteCmosSensor(0x0484, 0x5544);
HI541WriteCmosSensor(0x0486, 0x1111);
HI541WriteCmosSensor(0x0488, 0x1111);
HI541WriteCmosSensor(0x048a, 0x4455);
HI541WriteCmosSensor(0x048c, 0x5544);
HI541WriteCmosSensor(0x048e, 0x1111);
HI541WriteCmosSensor(0x0490, 0x1111);
HI541WriteCmosSensor(0x0492, 0x4444);
HI541WriteCmosSensor(0x0494, 0x4444);
HI541WriteCmosSensor(0x0496, 0x1111);
HI541WriteCmosSensor(0x0498, 0x1111);
HI541WriteCmosSensor(0x049a, 0x4444);
HI541WriteCmosSensor(0x049c, 0x4444);
HI541WriteCmosSensor(0x049e, 0x1111);
HI541WriteCmosSensor(0x04a0, 0x1111);
HI541WriteCmosSensor(0x04a2, 0x4444);
HI541WriteCmosSensor(0x04a4, 0x4444);
HI541WriteCmosSensor(0x04a6, 0x1111);
HI541WriteCmosSensor(0x04a8, 0x1111);
HI541WriteCmosSensor(0x04aa, 0x4444);
HI541WriteCmosSensor(0x04ac, 0x4444);
HI541WriteCmosSensor(0x04ae, 0x1111);
HI541WriteCmosSensor(0x04b0, 0x3333);
HI541WriteCmosSensor(0x04b2, 0x3333);
HI541WriteCmosSensor(0x04b4, 0x3333);
HI541WriteCmosSensor(0x04b6, 0x3333);
HI541WriteCmosSensor(0x04b8, 0x3333);
HI541WriteCmosSensor(0x04ba, 0x3333);
HI541WriteCmosSensor(0x04bc, 0x3333);
HI541WriteCmosSensor(0x04be, 0x3333);
HI541WriteCmosSensor(0x04c0, 0x3333);
HI541WriteCmosSensor(0x04c2, 0x3333);
HI541WriteCmosSensor(0x04c4, 0x3333);
HI541WriteCmosSensor(0x04c6, 0x3333);
//================================================ 
 
 
//===============================================
//AWB_Auto
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0a0c, 0x1010); //Rmin,Bmin
HI541WriteCmosSensor(0x0a0e, 0xc0c0); //Rmax,Bmax 
HI541WriteCmosSensor(0xffff, 0x0040);
//===============================================
 
 
//===============================================
// SAT ORI 
HI541WriteCmosSensor(0xffff, 0x0020);
// CB Saturation
HI541WriteCmosSensor(0x0FC8, 0x8080);// 0?? ???? CB Saturation
		HI541WriteCmosSensor(0x0FCA,0x8088);// 2?? ???? CB Saturation
		HI541WriteCmosSensor(0x0FCC,0x9090);// 4?? ???? CB Saturation
HI541WriteCmosSensor(0x0FCE, 0x9a9a);// 6?? ???? CB Saturation
HI541WriteCmosSensor(0x0FD0, 0x9ca0);// 8?? ???? CB Saturation
HI541WriteCmosSensor(0x0FD2, 0xa0a0);// 10?? ???? CB Saturation

// CR Saturation
HI541WriteCmosSensor(0x0FD4, 0x8080);// 0?? ???? CR Saturation
		HI541WriteCmosSensor(0x0FD6,0x8088);// 2?? ???? CR Saturation
		HI541WriteCmosSensor(0x0FD8,0x9090);// 4?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDA, 0x9a98);// 6?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDC, 0x8ca0);// 8?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDE, 0xa0a0);// 10?? ???? CR Saturation
//================================================ 
 
// AUTO
//================================
//=========== ADP Start ==========
//================================

HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0DB0, 0x0000); // ADP B[0]On/Off // B[1] DMA  On/Off // ADP Std Copy

//===================================================
//========= Luminance Adaptive ISP Start  ===========
//===================================================
HI541WriteCmosSensor(0xffff, 0x0020);

// Outdoor            
HI541WriteCmosSensor(0x12B6, 0x0000); // 0x7034 : pre_flat_ctl, 	0x7035 : flat_max_luml1
HI541WriteCmosSensor(0x1304, 0x1B68); // 0x7837 : spstd_gain_8,	0x7838 : post_std_sel //20140522 skin detail 79 -> 68 -> 46             
HI541WriteCmosSensor(0x1352, 0x1a25); // 0x940b : ynr_flat_ctl,	0x9414 : ynr_flat_luml1
HI541WriteCmosSensor(0x1372, 0x0C18); // 0x9809 : yee_skin_higgain,	0x980a : yee_std_post_gain_pos //22->14->16->14    
HI541WriteCmosSensor(0x1374, 0x1626); // 0x980b : yee_std_gain,	0x980c : yee_lum_gain_p1 //24->0c->14                  
HI541WriteCmosSensor(0x13EE, 0x1812); // 0x7c84 : ci_spstd_8,		0x7c85 : ci_post_std_ctl

// Indoor
HI541WriteCmosSensor(0x14AC, 0x030D); // INDOOR 0x9000 : Saturation ENB Transfer Function , 0x7034 : pre_flat_ctl //20140626
HI541WriteCmosSensor(0x14FC, 0xcb16); // INDOOR 0x7838 : post_std_sel,	0x7440 : flt_luml1, //20140627 post neg/pos 87 -> 66
HI541WriteCmosSensor(0x1548, 0x0218); // INDOOR 0x940a : ynr_line_rate,   0x940b : ynr_flat_ctl, //20140626 rate 00
HI541WriteCmosSensor(0x156A, 0x1718); // INDOOR 0x980a : yee_std_post_gain_pos, 0x980b : yee_std_gain,	
HI541WriteCmosSensor(0x15E6, 0x3400); // INDOOR 0x7c85 : ci_post_std_ctl, 0x7c97 : ci_skin_blue_ctl,

// Dark1
HI541WriteCmosSensor(0x16A4, 0x1F00); // Dark1 0x7034 : pre_flat_ctl, 	0x7035 : flat_max_luml1
HI541WriteCmosSensor(0x16F2, 0x1898); // Dark1 0x7837 : spstd_gain_8,     0x7838 : post_std_sel
HI541WriteCmosSensor(0x1760, 0x0C12); // Dark1 0x9809 : yee_skin_higgain,	0x980a : yee_std_post_gain_pos
HI541WriteCmosSensor(0x1762, 0x1214); // Dark1 0x980b : yee_std_gain,	0x980c : yee_lum_gain_p1
HI541WriteCmosSensor(0x1740, 0x1836); // Dark1 0x940b : ynr_flat_ctl,	0x9414 : ynr_flat_luml1
HI541WriteCmosSensor(0x17DC, 0x1A36); // Dark1 0x7c84 : ci_spstd_8,	0x7c85 : ci_post_std_ctl

// Dark2
HI541WriteCmosSensor(0x189A, 0x001F);  // Dark2 0x9000 : Saturation ENB Transfer Function , 0x7034 : pre_flat_ctl 	
HI541WriteCmosSensor(0x18EA, 0x9800);  // Dark2 0x7838 : post_std_sel,	0x7440 : flt_luml1,
HI541WriteCmosSensor(0x1936, 0x0218);  // Dark2 0x940a : ynr_line_rate,   0x940b : ynr_flat_ctl,	 
HI541WriteCmosSensor(0x1958, 0x1212);  // Dark2 0x980a : yee_std_post_gain_pos,0x980b : yee_std_gain,	//20140628
HI541WriteCmosSensor(0x19D4, 0x3600);  // Dark2 0x7c85 : ci_post_std_ctl, 0x7c97 : ci_skin_blue_ctl,

HI541WriteCmosSensor(0x0DB0, 0x0300); // ADP B[0]On/Off // B[1] DMA  On/Off // ADP Std Copy
HI541WriteCmosSensor(0x0DB0, 0x0300); // ADP B[0]On/Off // B[1] DMA  On/Off // ADP Std Copy
HI541WriteCmosSensor(0x12b0, 0x0520);
HI541WriteCmosSensor(0x1b1a, 0x0f00);
HI541WriteCmosSensor(0xffff, 0x0040);
                         
               break;
        case SCENE_MODE_HDR:
             
               break;
        case SCENE_MODE_OFF:
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0DB0, 0x0000);// ADP B[0]On/Off // B[1] DMA  On/Off // ADP Std Copy


HI541WriteCmosSensor(0x0DCC, 0x9CA8);// EV TH (Dark2) 6.67fps AGx8
HI541WriteCmosSensor(0x0DCE, 0x0003);

HI541WriteCmosSensor(0x0DD0, 0x4E54);// EV TH (Dark1) 6.67fps AGx4
HI541WriteCmosSensor(0x0DD2, 0x8001);

HI541WriteCmosSensor(0x0DD4, 0xf508);// EV TH (Indoor) 33.33fps AGx1.25
HI541WriteCmosSensor(0x0DD6, 0x1800);

HI541WriteCmosSensor(0x0DD8, 0x10a4);// EV TH (Outdoor) 1000fps AGx1
HI541WriteCmosSensor(0x0DDA, 0x0000);
//===============================================
//10fps

HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0384,0x80e7);//L
HI541WriteCmosSensor(0x0386,0x4c00);//H exposure max 100 8.33 fps
HI541WriteCmosSensor(0x0388,0x80e7);//L
HI541WriteCmosSensor(0x038a,0x4c00);//H exposure max 120 8.57 fps
//================================================


//===============================================
//ISO AUTO
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x02ee, 0x2020);//AG th0,1
HI541WriteCmosSensor(0x02f0, 0x2020);//AG th2,3

HI541WriteCmosSensor(0x02f2, 0xff00);//AG th4, 

//===============================================

//===============================================
//AE weight table_Center
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0448, 0x1111);
HI541WriteCmosSensor(0x044a, 0x1111);
HI541WriteCmosSensor(0x044c, 0x1111);
HI541WriteCmosSensor(0x044e, 0x1111);
HI541WriteCmosSensor(0x0450, 0x1111);
HI541WriteCmosSensor(0x0452, 0x1111);
HI541WriteCmosSensor(0x0454, 0x1111);
HI541WriteCmosSensor(0x0456, 0x1111);
HI541WriteCmosSensor(0x0458, 0x1111);
HI541WriteCmosSensor(0x045a, 0x1111);
HI541WriteCmosSensor(0x045c, 0x1111);
HI541WriteCmosSensor(0x045e, 0x1111);
HI541WriteCmosSensor(0x0460, 0x1111);
HI541WriteCmosSensor(0x0462, 0x1111);
HI541WriteCmosSensor(0x0464, 0x1111);
HI541WriteCmosSensor(0x0466, 0x1111);
HI541WriteCmosSensor(0x0468, 0x1111);
HI541WriteCmosSensor(0x046a, 0x4444);
HI541WriteCmosSensor(0x046c, 0x4444);
HI541WriteCmosSensor(0x046e, 0x1111);
HI541WriteCmosSensor(0x0470, 0x1111);
HI541WriteCmosSensor(0x0472, 0x4444);
HI541WriteCmosSensor(0x0474, 0x4444);
HI541WriteCmosSensor(0x0476, 0x1111);
HI541WriteCmosSensor(0x0478, 0x1111);
HI541WriteCmosSensor(0x047a, 0x4444);
HI541WriteCmosSensor(0x047c, 0x4444);
HI541WriteCmosSensor(0x047e, 0x1111);
HI541WriteCmosSensor(0x0480, 0x1111);
HI541WriteCmosSensor(0x0482, 0x4455);
HI541WriteCmosSensor(0x0484, 0x5544);
HI541WriteCmosSensor(0x0486, 0x1111);
HI541WriteCmosSensor(0x0488, 0x1111);
HI541WriteCmosSensor(0x048a, 0x4455);
HI541WriteCmosSensor(0x048c, 0x5544);
HI541WriteCmosSensor(0x048e, 0x1111);
HI541WriteCmosSensor(0x0490, 0x1111);
HI541WriteCmosSensor(0x0492, 0x4444);
HI541WriteCmosSensor(0x0494, 0x4444);
HI541WriteCmosSensor(0x0496, 0x1111);
HI541WriteCmosSensor(0x0498, 0x1111);
HI541WriteCmosSensor(0x049a, 0x4444);
HI541WriteCmosSensor(0x049c, 0x4444);
HI541WriteCmosSensor(0x049e, 0x1111);
HI541WriteCmosSensor(0x04a0, 0x1111);
HI541WriteCmosSensor(0x04a2, 0x4444);
HI541WriteCmosSensor(0x04a4, 0x4444);
HI541WriteCmosSensor(0x04a6, 0x1111);
HI541WriteCmosSensor(0x04a8, 0x1111);
HI541WriteCmosSensor(0x04aa, 0x4444);
HI541WriteCmosSensor(0x04ac, 0x4444);
HI541WriteCmosSensor(0x04ae, 0x1111);
HI541WriteCmosSensor(0x04b0, 0x3333);
HI541WriteCmosSensor(0x04b2, 0x3333);
HI541WriteCmosSensor(0x04b4, 0x3333);
HI541WriteCmosSensor(0x04b6, 0x3333);
HI541WriteCmosSensor(0x04b8, 0x3333);
HI541WriteCmosSensor(0x04ba, 0x3333);
HI541WriteCmosSensor(0x04bc, 0x3333);
HI541WriteCmosSensor(0x04be, 0x3333);
HI541WriteCmosSensor(0x04c0, 0x3333);
HI541WriteCmosSensor(0x04c2, 0x3333);
HI541WriteCmosSensor(0x04c4, 0x3333);
HI541WriteCmosSensor(0x04c6, 0x3333);
//================================================


//===============================================
//AWB_Auto
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0a0c, 0x1010); //Rmin,Bmin
HI541WriteCmosSensor(0x0a0e, 0xc0c0); //Rmax,Bmax 
HI541WriteCmosSensor(0xffff, 0x0040);
//===============================================

 
//===============================================
// SAT ORI
HI541WriteCmosSensor(0xffff, 0x0020);
// CB Saturation
HI541WriteCmosSensor(0x0FC8, 0x8080);// 0?? ???? CB Saturation
		HI541WriteCmosSensor(0x0FCA,0x8088);// 2?? ???? CB Saturation
		HI541WriteCmosSensor(0x0FCC,0x9090);// 4?? ???? CB Saturation
HI541WriteCmosSensor(0x0FCE,0x949a);// 6?? ???? CB Saturation
HI541WriteCmosSensor(0x0FD0, 0x9ca0);// 8?? ???? CB Saturation
HI541WriteCmosSensor(0x0FD2, 0xa0a0);// 10?? ???? CB Saturation

// CR Saturation
HI541WriteCmosSensor(0x0FD4, 0x8080);// 0?? ???? CR Saturation
		HI541WriteCmosSensor(0x0FD6,0x8088);// 2?? ???? CR Saturation
		HI541WriteCmosSensor(0x0FD8,0x9090);// 4?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDA,0x9498);// 6?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDC, 0x8ca0);// 8?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDE, 0xa0a0);// 10?? ???? CR Saturation
//================================================

// AUTO
//================================
//=========== ADP Start ==========
//================================

HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x0DB0, 0x0000); // ADP B[0]On/Off // B[1] DMA  On/Off // ADP Std Copy

//===================================================
//========= Luminance Adaptive ISP Start  ===========
//===================================================
HI541WriteCmosSensor(0xffff, 0x0020);

// Outdoor            
HI541WriteCmosSensor(0x12B6, 0x0000); // 0x7034 : pre_flat_ctl, 	0x7035 : flat_max_luml1
HI541WriteCmosSensor(0x1304, 0x1B68); // 0x7837 : spstd_gain_8,	0x7838 : post_std_sel //20140522 skin detail 79 -> 68 -> 46             
HI541WriteCmosSensor(0x1352, 0x1a25); // 0x940b : ynr_flat_ctl,	0x9414 : ynr_flat_luml1
HI541WriteCmosSensor(0x1372, 0x0C18); // 0x9809 : yee_skin_higgain,	0x980a : yee_std_post_gain_pos //22->14->16->14    
HI541WriteCmosSensor(0x1374, 0x1626); // 0x980b : yee_std_gain,	0x980c : yee_lum_gain_p1 //24->0c->14                  
HI541WriteCmosSensor(0x13EE, 0x1812); // 0x7c84 : ci_spstd_8,		0x7c85 : ci_post_std_ctl

// Indoor
HI541WriteCmosSensor(0x14AC, 0x030D); // INDOOR 0x9000 : Saturation ENB Transfer Function , 0x7034 : pre_flat_ctl //20140626
HI541WriteCmosSensor(0x14FC, 0xcb16); // INDOOR 0x7838 : post_std_sel,	0x7440 : flt_luml1, //20140627 post neg/pos 87 -> 66
HI541WriteCmosSensor(0x1548, 0x0218); // INDOOR 0x940a : ynr_line_rate,   0x940b : ynr_flat_ctl, //20140626 rate 00
HI541WriteCmosSensor(0x156A, 0x1718); // INDOOR 0x980a : yee_std_post_gain_pos, 0x980b : yee_std_gain,	
HI541WriteCmosSensor(0x15E6, 0x3400); // INDOOR 0x7c85 : ci_post_std_ctl, 0x7c97 : ci_skin_blue_ctl,

// Dark1
HI541WriteCmosSensor(0x16A4, 0x1F00); // Dark1 0x7034 : pre_flat_ctl, 	0x7035 : flat_max_luml1
HI541WriteCmosSensor(0x16F2, 0x1898); // Dark1 0x7837 : spstd_gain_8,     0x7838 : post_std_sel
HI541WriteCmosSensor(0x1760, 0x0C12); // Dark1 0x9809 : yee_skin_higgain,	0x980a : yee_std_post_gain_pos
HI541WriteCmosSensor(0x1762, 0x1214); // Dark1 0x980b : yee_std_gain,	0x980c : yee_lum_gain_p1
HI541WriteCmosSensor(0x1740, 0x1836); // Dark1 0x940b : ynr_flat_ctl,	0x9414 : ynr_flat_luml1
HI541WriteCmosSensor(0x17DC, 0x1A36); // Dark1 0x7c84 : ci_spstd_8,	0x7c85 : ci_post_std_ctl

// Dark2
HI541WriteCmosSensor(0x189A, 0x001F);  // Dark2 0x9000 : Saturation ENB Transfer Function , 0x7034 : pre_flat_ctl 	
HI541WriteCmosSensor(0x18EA, 0x9800);  // Dark2 0x7838 : post_std_sel,	0x7440 : flt_luml1,
HI541WriteCmosSensor(0x1936, 0x0218);  // Dark2 0x940a : ynr_line_rate,   0x940b : ynr_flat_ctl,	 
HI541WriteCmosSensor(0x1958, 0x1212);  // Dark2 0x980a : yee_std_post_gain_pos,0x980b : yee_std_gain,	//20140628
HI541WriteCmosSensor(0x19D4, 0x3600);  // Dark2 0x7c85 : ci_post_std_ctl, 0x7c97 : ci_skin_blue_ctl,

HI541WriteCmosSensor(0x0DB0, 0x0300); // ADP B[0]On/Off // B[1] DMA  On/Off // ADP Std Copy
HI541WriteCmosSensor(0x0DB0, 0x0300); // ADP B[0]On/Off // B[1] DMA  On/Off // ADP Std Copy
HI541WriteCmosSensor(0x12b0, 0x0520);
HI541WriteCmosSensor(0x1b1a, 0x0f00);
HI541WriteCmosSensor(0xffff, 0x0040);

			
           break;
        default:
	      return KAL_FALSE;
               break;
    }

}

BOOL HI541SetEffect(UINT16 Para)
{
	//SENSORDB("[HI541_Debug]HI541SetEffect Para:%d;\n",Para);

	switch (Para)
	{
	case MEFFECT_OFF:

HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0xa400, 0x0000);// Effect off
HI541WriteCmosSensor(0xa404, 0x8080);// Cb / Cr
HI541WriteCmosSensor(0xffff, 0x0040);

		

		break;
	case MEFFECT_SEPIA:
		
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0xa400, 0x6000);//Effect off
HI541WriteCmosSensor(0xa404, 0x60a0);// Cb / Cr
HI541WriteCmosSensor(0xffff, 0x0040);
		
		break;
	case MEFFECT_NEGATIVE://----datasheet
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0xa400, 0x0400);//Effect off
HI541WriteCmosSensor(0xa404, 0x8080);// Cb / Cr
HI541WriteCmosSensor(0xffff, 0x0040);
		
		break;
	case MEFFECT_SEPIAGREEN://----datasheet aqua
		
		break;
	case MEFFECT_SEPIABLUE:
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0xa400, 0x6000);//Effect off
HI541WriteCmosSensor(0xa404, 0xa060);// Cb / Cr
HI541WriteCmosSensor(0xffff, 0x0040);
		
		break;
	case MEFFECT_MONO: //----datasheet black & white
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0xa400, 0x6000);//Effect off
HI541WriteCmosSensor(0xa404, 0x8080);// Cb / Cr
HI541WriteCmosSensor(0xffff, 0x0040);
		break;
	default:
		return KAL_FALSE;
	}
	return KAL_TRUE;

} /* HI541SetEffect */

BOOL HI541SetBanding(UINT16 Para)
{



	//SENSORDB("[HI541_Debug]HI541SetBanding Para:%d;\n",Para);
	spin_lock(&HI541_drv_lock);
	HI541Status.Banding = Para;
	spin_unlock(&HI541_drv_lock);


     if(HI541Status.SensorMode==SENSOR_MODE_VIDEO)
       {
              if (HI541Status.Banding == AE_FLICKER_MODE_60HZ) 
              {
              spin_lock(&HI541_drv_lock);
           
              spin_unlock(&HI541_drv_lock);
HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x02e0,0xc940); //Banding 60Hz
HI541WriteCmosSensor(0xffff,0x0040);
              
            
              }
              else
              {
              spin_lock(&HI541_drv_lock);
            
              spin_unlock(&HI541_drv_lock);
                HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x02e0,0xc540); //Banding 50Hz
HI541WriteCmosSensor(0xffff,0x0040);
              
            
              }
     	}
      else
       {
              if (HI541Status.Banding == AE_FLICKER_MODE_60HZ) 
              {
              spin_lock(&HI541_drv_lock);
             
              spin_unlock(&HI541_drv_lock);
HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x02e0,0xc940); //Banding 60Hz
HI541WriteCmosSensor(0xffff,0x0040);
              
              }
              else
              {
              spin_lock(&HI541_drv_lock);
              
              spin_unlock(&HI541_drv_lock);
               HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x02e0,0xc540); //Banding 50Hz
HI541WriteCmosSensor(0xffff,0x0040);
              
           
             
              }
     	}


	return TRUE;
} /* HI541SetBanding */

BOOL HI541SetExposure(UINT16 Para)
{
	//SENSORDB("[HI541_Debug]HI541SetExposure Para:%d;\n",Para);

if(zsd==1)
{
return;
}else{
	
	spin_lock(&HI541_drv_lock);
	HI541Status.ISPCTL3 |= 0x10;
	spin_unlock(&HI541_drv_lock);
	
	if (SCENE_MODE_HDR == HI541Status.ScenMode && SENSOR_MODE_CAPTURE == HI541Status.SensorMode)
	{

		switch (Para)
		{
		case AE_EV_COMP_n20:	
			/* EV -2 */
			//SENSORDB("[HI541_Debug]AE_EV_COMP_n20 Para:%d;\n",Para);	 
			

		case AE_EV_COMP_20:			   /* EV +2 */
			//SENSORDB("[HI541_Debug]AE_EV_COMP_20 Para:%d;\n",Para);
			
			break;
		case AE_EV_COMP_00: 			 /* EV +2 */
			//SENSORDB("[HI541_Debug]ISP_BRIGHT_MIDDLE Para:%d;\n",Para);
			
			break;
		default:
			return KAL_FALSE;
		}
	}
	else{
		switch (Para)
		{
		case AE_EV_COMP_n20:  
			/* EV -2 */
			//SENSORDB("[HI541_Debug]AE_EV_COMP_n20 Para:%d;\n",Para);
            		
			break;
		
		case AE_EV_COMP_n10:              /* EV -1 */

HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0x4832, 0x7f7a);//EV EN
				HI541WriteCmosSensor(0xa002, 0x60f0);//EV -1
HI541WriteCmosSensor(0xffff, 0x0040);
            		
			break;
		
		case AE_EV_COMP_00:                /* EV 0 */

HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0x4832, 0x7f7a);//EV EN
HI541WriteCmosSensor(0xa002, 0x8000);//Effect off
HI541WriteCmosSensor(0xffff, 0x0040);
            		
			break;

		case AE_EV_COMP_10:              /* EV +1 */

HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0x4832, 0x7f7a);//EV EN
				HI541WriteCmosSensor(0xa002, 0xa070);//EV +1
HI541WriteCmosSensor(0xffff, 0x0040);
            		
			break;
		case AE_EV_COMP_20:              /* EV +2 */
			//SENSORDB("[HI541_Debug]AE_EV_COMP_20 Para:%d;\n",Para);
            		
			break;
		default:
			return KAL_FALSE;
		}
	}
	return KAL_TRUE;
}
} /* HI541SetExposure */

void HI541SetBrightness(UINT16 para)
{


        switch (para)
    {
        case ISP_BRIGHT_LOW:
		HI541WriteCmosSensor(0xffff, 0x0040);
		HI541WriteCmosSensor(0x4832, 0x7f7a);//Brightness EN
		HI541WriteCmosSensor(0xa000, 0xc080);//Brightness -1
		HI541WriteCmosSensor(0xffff, 0x0040);

      
             break;
        case ISP_BRIGHT_HIGH:
		HI541WriteCmosSensor(0xffff, 0x0040);
		HI541WriteCmosSensor(0x4832, 0x7f7a);//Brightness EN
		HI541WriteCmosSensor(0xa000, 0x4080);//Brightness +1
		HI541WriteCmosSensor(0xffff, 0x0040);

    
             break;
        case ISP_BRIGHT_MIDDLE:
		HI541WriteCmosSensor(0xffff, 0x0040);
		HI541WriteCmosSensor(0x4832, 0x7f7a);//Brightness EN
		HI541WriteCmosSensor(0xa000, 0x8080);//Brightness off
		HI541WriteCmosSensor(0xffff, 0x0040);
		
			 break;
        default:
             return KAL_FALSE;
             break;
    }

}

void HI541SetContrast(UINT16 para)
{
	kal_uint32 contrast;

    contrast = (HI541ReadCmosSensor(0xa000)<<8)|HI541ReadCmosSensor(0xa001);//멕貫角좋똑，됴貫角뚤궐똑
	contrast =(contrast&0xff00);

	//spin_lock(&HI541gx_mipi_rw_lock);

	switch (para)
	{
	case ISP_CONTRAST_LOW:
		//SENSORDB("[HI541_Debug]Contrast:LOW Para:%d;\n",para);
		HI541WriteCmosSensor(0xffff, 0x0040);
		HI541WriteCmosSensor(0x4832, 0x7f7a);//EV EN
        HI541WriteCmosSensor(0xa000, (contrast|0x40));//Contrast -1
		HI541WriteCmosSensor(0xffff, 0x0040);
		
		break; 
	case ISP_CONTRAST_HIGH:
		//SENSORDB("[HI541_Debug]Contrast:HIGH Para:%d;\n",para);
		
		HI541WriteCmosSensor(0xffff, 0x0040);
		HI541WriteCmosSensor(0x4832, 0x7f7a);//EV EN
        HI541WriteCmosSensor(0xa000, (contrast|0xc0));//Contrast +1
		HI541WriteCmosSensor(0xffff, 0x0040);
		
		break; 
	case ISP_CONTRAST_MIDDLE:
		//SENSORDB("[HI541_Debug]Contrast:MIDDLE Para:%d;\n",para);
		HI541WriteCmosSensor(0xffff, 0x0040);
		HI541WriteCmosSensor(0x4832, 0x7f7a);//EV EN
        HI541WriteCmosSensor(0xa000, (contrast|0x80));//Effect off
		HI541WriteCmosSensor(0xffff, 0x0040);

		
	default:
		break; 
	}
//	spin_unlock(&HI541gx_mipi_rw_lock); 
	return;
}

void HI541SetSetIso(UINT16 para)
{


     switch (para) 
    {
        case AE_ISO_100:
             //ISO 100
             
//================================
//========= SYSTEM Start =========
//================================


//===============================================
//ISO100
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x02ee, 0x2020); //AG th0,1 
HI541WriteCmosSensor(0x02f0, 0x2020); //AG th2,3 
HI541WriteCmosSensor(0x02f2, 0x4000); //AG th4, 
HI541WriteCmosSensor(0xffff, 0x0040);
             break;

			 
		case AE_ISO_AUTO:

			//===============================================
//ISO AUTO
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x02ee, 0x2040);//AG th0,1 
HI541WriteCmosSensor(0x02f0, 0x5064);//AG th2,3 
HI541WriteCmosSensor(0x02f2, 0xff00);//AG th4, 
HI541WriteCmosSensor(0xffff, 0x0040);
 break;


        case AE_ISO_200:
             //ISO 200
            
//===============================================
//ISO200
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x02ee, 0x4040); //AG th0,1 
HI541WriteCmosSensor(0x02f0, 0x4040); //AG th2,3
HI541WriteCmosSensor(0x02f2, 0x8000); //AG th4, 
HI541WriteCmosSensor(0xffff, 0x0040);
             break;

			 
        case AE_ISO_400:
             //ISO 400
         
//===============================================
//ISO400
HI541WriteCmosSensor(0xffff, 0x0020);
HI541WriteCmosSensor(0x02ee, 0x8080); //AG th0,1 
HI541WriteCmosSensor(0x02f0, 0x8080); //AG th2,3 
HI541WriteCmosSensor(0x02f2, 0xff00); //AG th4, 
HI541WriteCmosSensor(0xffff, 0x0040);
             break;
			 
        default:
             break;
    }
    return;


}


void HI541SetSaturation(UINT16 para)
{


	switch (para)
	{
	case ISP_SAT_HIGH:
		//SENSORDB("[HI541_Debug]Saturation:High Para:%d;\n",para);
		//===============================================

HI541WriteCmosSensor(0xffff, 0x0020);
// CB Saturation
HI541WriteCmosSensor(0x0FC8, 0x8888);// 0?? ???? CB Saturation
HI541WriteCmosSensor(0x0FCA, 0x8888);// 2?? ???? CB Saturation
HI541WriteCmosSensor(0x0FCC, 0xa0a0);// 4?? ???? CB Saturation
HI541WriteCmosSensor(0x0FCE, 0xa8a8);// 6?? ???? CB Saturation
HI541WriteCmosSensor(0x0FD0, 0xb8b8);// 8?? ???? CB Saturation
HI541WriteCmosSensor(0x0FD2, 0xb8b8);// 10?? ???? CB Saturation

// CR Saturation
HI541WriteCmosSensor(0x0FD4, 0x8888);// 0?? ???? CR Saturation
HI541WriteCmosSensor(0x0FD6, 0x8880);// 2?? ???? CR Saturation
HI541WriteCmosSensor(0x0FD8, 0xa0a0);// 4?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDA, 0xb8b8);// 6?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDC, 0xb8b8);// 8?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDE, 0xb8b8);// 10?? ???? CR Saturation

//================================================
HI541WriteCmosSensor(0xffff, 0x0040);
		
		break; 
	case ISP_SAT_LOW:
		//SENSORDB("[HI541_Debug]Saturation:LOW Para:%d;\n",para);
		//===============================================

HI541WriteCmosSensor(0xffff, 0x0020);
// CB Saturation
HI541WriteCmosSensor(0x0FC8, 0x6868);// 0?? ???? CB Saturation
HI541WriteCmosSensor(0x0FCA, 0x6868);// 2?? ???? CB Saturation
HI541WriteCmosSensor(0x0FCC, 0x7070);// 4?? ???? CB Saturation
HI541WriteCmosSensor(0x0FCE, 0x8282);// 6?? ???? CB Saturation
HI541WriteCmosSensor(0x0FD0, 0x8488);// 8?? ???? CB Saturation
HI541WriteCmosSensor(0x0FD2, 0x8888);// 10?? ???? CB Saturation

// CR Saturation
HI541WriteCmosSensor(0x0FD4, 0x6868);// 0?? ???? CR Saturation
HI541WriteCmosSensor(0x0FD6, 0x6868);// 2?? ???? CR Saturation
HI541WriteCmosSensor(0x0FD8, 0x7070);// 4?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDA, 0x8280);// 6?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDC, 0x7488);// 8?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDE, 0x8888);// 10?? ???? CR Saturation

//================================================
HI541WriteCmosSensor(0xffff, 0x0040);
		
		
		break; 
	case ISP_SAT_MIDDLE:
		//SENSORDB("[HI541_Debug]Saturation:Middle Para:%d;\n",para);
		//===============================================

HI541WriteCmosSensor(0xffff, 0x0020);
// CB Saturation
HI541WriteCmosSensor(0x0FC8, 0x8080);// 0?? ???? CB Saturation
		HI541WriteCmosSensor(0x0FCA,0x8088);// 2?? ???? CB Saturation
		HI541WriteCmosSensor(0x0FCC,0x9090);// 4?? ???? CB Saturation
HI541WriteCmosSensor(0x0FCE,0x949a);// 6?? ???? CB Saturation
HI541WriteCmosSensor(0x0FD0, 0x9ca0);// 8?? ???? CB Saturation
HI541WriteCmosSensor(0x0FD2, 0xa0a0);// 10?? ???? CB Saturation

// CR Saturation
HI541WriteCmosSensor(0x0FD4, 0x8080);// 0?? ???? CR Saturation
		HI541WriteCmosSensor(0x0FD6,0x8088);// 2?? ???? CR Saturation
		HI541WriteCmosSensor(0x0FD8,0x9090);// 4?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDA,0x9498);// 6?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDC, 0x8ca0);// 8?? ???? CR Saturation
HI541WriteCmosSensor(0x0FDE, 0xa0a0);// 10?? ???? CR Saturation

//================================================
HI541WriteCmosSensor(0xffff, 0x0040);
		
		break; 
	default:
		break; 
	} 
}

void HI541SetHue(UINT16 para)
{

}

void HI541SetEdge(UINT16 para)
{

	
	

}

UINT32 HI541YUVSensorSetting(FEATURE_ID Cmd, UINT32 Para)
{
	switch (Cmd)
	{
		case FID_SCENE_MODE:
			HI541_set_scene_mode(Para);            
    if (Para == SCENE_MODE_OFF)         
        {         
   
            HI541NightMode(KAL_FALSE);         
        }
    else if (Para == SCENE_MODE_NIGHTSCENE)         
        {
            HI541NightMode(KAL_TRUE);           
        }         

            break; 
		case FID_AWB_MODE:
			HI541SetWb(Para);
			break;
		case FID_COLOR_EFFECT:
			HI541SetEffect(Para);
			break;
		case FID_AE_EV:
			HI541SetExposure(Para);
			break;
		case FID_AE_FLICKER:
            if(CAPTURE_FLAG1== 0)
            {
                HI541SetBanding(Para);
            }
            else
            {      
                CAPTURE_FLAG1= 0;
            }
			break;
		case FID_AE_SCENE_MODE: 
			if (Para == AE_MODE_OFF) 
			{
				HI541SetAeMode(KAL_FALSE);
			}
			else 
			{
				HI541SetAeMode(KAL_TRUE);
			}
			break; 
		case FID_ZOOM_FACTOR:
			//SENSORDB("[HI541]ZoomFactor :%d;\n",Para);
			spin_lock(&HI541_drv_lock);
			HI541Status.ZoomFactor = Para;
			spin_unlock(&HI541_drv_lock);
			break;

		case FID_ISP_CONTRAST:
			//SENSORDB("HI541GX_MIPISensorSetting func:FID_ISP_CONTRAST:%d\n",Para);
			HI541SetContrast(Para);
			break;
		case FID_ISP_BRIGHT:
			//SENSORDB("HI541GX_MIPISensorSetting func:FID_ISP_BRIGHT:%d\n",Para);
			HI541SetBrightness(Para);
			break;
		case FID_ISP_SAT:
			//SENSORDB("HI541GX_MIPISensorSetting func:FID_ISP_SAT:%d\n",Para);
			HI541SetSaturation(Para);
			break;
		case FID_AE_ISO:
			//SENSORDB("HI541GX_MIPISensorSetting func:FID_AE_ISO:%d\n",Para);
			HI541SetSetIso(Para);

			break;
		case FID_ISP_HUE :

			//SENSORDB("HI541GX_MIPISensorSetting func:FID_AE_ISO:%d\n",Para);
			HI541SetHue(Para);
			break;
		case FID_ISP_EDGE:

			HI541SetEdge(Para);
			break;
		default:
			break;
	}
	return TRUE;
}   /* HI541YUVSensorSetting */

UINT32 HI541YUVSetVideoMode(UINT16 FrameRate)

{
	kal_uint32 EXPFIX, BLC_TIME_TH_ONOFF;
	kal_uint32 LineLength,BandingValue;


	//  return TRUE;
	spin_lock(&HI541_drv_lock);
	HI541Status.VideoMode=KAL_TRUE;
	HI541Status.SensorMode=SENSOR_MODE_VIDEO;
	spin_unlock(&HI541_drv_lock);

	//SENSORDB("[HI541]HI541YUVSetVideoMode FrameRate:%d;\n",FrameRate);

	if(FrameRate>=30)
	
	{
	
		FrameRate=30;
				//Video mode30
		//SENSORDB("[HI541]HI541YUVSetVideoMode FrameRate:%d;\n",FrameRate);
//[USERSET_1]
//DISP_NAME = "Video_30"
//DISP_WIDTH = 1280	
//DISP_HEIGHT = 960
//MCLK = 84.00
//PLL = 1.00

//BEGIN

///////////////////////////////////////////
// Preview SXGA SUB Setting
// 1280x960@30fps, MIPI_YUV422, 1/2 Digital Scalex1/2 Analog Sub-sampling
///////////////////////////////////////////
//I2C_ID = 0x40
//I2C_BYTE  = 0x22

//================================
//========= SYSTEM Start =========
//================================
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x0100,0x0000);	//streaming disable
HI541WriteCmosSensor(0x5402,0x0004);	//tg_ctl3
HI541WriteCmosSensor(0x0900,0x0002);	//binning
HI541WriteCmosSensor(0x0340,0xf503);//frame_length_lines  //15 fps
HI541WriteCmosSensor(0x0342,0xca0a);//line_length_pck
HI541WriteCmosSensor(0x0344,0x1000);//x_addr_start
HI541WriteCmosSensor(0x0346,0x0e00);//y_addr_start
HI541WriteCmosSensor(0x0348,0x570a);//x_addr_end
HI541WriteCmosSensor(0x034a,0xc907);//y_addr_end
HI541WriteCmosSensor(0x034c,0x0005);//x_output_size
HI541WriteCmosSensor(0x034e,0xc003);//y_output_size
HI541WriteCmosSensor(0x0380,0x0100);//x_even_inc
HI541WriteCmosSensor(0x0382,0x0100);//x_odd_inc
HI541WriteCmosSensor(0x0384,0x0100);//y_even_inc
HI541WriteCmosSensor(0x0386,0x0300);//y_odd_inc

HI541WriteCmosSensor(0x540e,0x3677);//JH.LEE_140904
HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x0384,0xc073);//L
HI541WriteCmosSensor(0x0386,0x2600);//H exposure max 100 16.67fps
HI541WriteCmosSensor(0x0388,0x80b9);//L
HI541WriteCmosSensor(0x038a,0x2a00);//H exposure max 120 15fps
//================================
//========= SYSTEM End ===========
//================================

//================================
//======= ISP HW Start ===========
//================================
HI541WriteCmosSensor(0xffff,0x0040);

// ISP enable
HI541WriteCmosSensor(0x4830,0xfeef);
HI541WriteCmosSensor(0x4832,0x7f7a);
HI541WriteCmosSensor(0x4834,0x0401);
  
// BScaler
HI541WriteCmosSensor(0x6000,0x2800);//mode_byrsc1
HI541WriteCmosSensor(0x6002,0x2000);//byrsc_fifo_delay

//Yscaler 1280x960
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0xa800,0x2000); //mode_zoom1
HI541WriteCmosSensor(0xa804,0x0005); //zoom_dst_width
HI541WriteCmosSensor(0xa806,0xc003); //zoom_dst_height
		HI541WriteCmosSensor(0xa810,0x1308); //zoom_hor_step
HI541WriteCmosSensor(0xa812,0x1908); //zoom_ver_step
		HI541WriteCmosSensor(0xa814,0x3E02); //zoom_hor_step_remain
HI541WriteCmosSensor(0xa816,0x9909); //zoom_ver_step_remain
		HI541WriteCmosSensor(0xa818,0x1000); //zoom_fifo_delay
HI541WriteCmosSensor(0xa824,0x0f00); //zoom_intpol1
HI541WriteCmosSensor(0xa826,0x0000); //zoom_intpol3

// Iridix
HI541WriteCmosSensor(0x8404,0x1405);//hdr_frame_width
HI541WriteCmosSensor(0x8406,0xd003);//hdr_frame_height
//================================
//======= ISP HW End   ===========
//================================


//================================
//=========== SSD Start ==========
//================================
//Hardware SSD SET
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0xc414,0x0000);//ae_patch_xy_offset
HI541WriteCmosSensor(0xc416,0xa03c);//ae_patch_xy_w_h x 16
HI541WriteCmosSensor(0xc418,0x0404);//awb_size_xy_offset
HI541WriteCmosSensor(0xc41a,0xa034);//awb_size_xy_w_h x 16

//Firmware SSD SET
HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x1fe4,0x0404);//awb_size_y_offset
HI541WriteCmosSensor(0x1fe6,0x200a);//awb_size_xy_w_h x 16
HI541WriteCmosSensor(0x1fe8,0x0404);//ae_size_xy_offset
HI541WriteCmosSensor(0x1fea,0xa034);//ae_size_xy_w_h x 16

//================================
//=========== SSD End   ==========
//================================

HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0xC806, 0xffC3); // af_ctl8 // manual window setting + x_half[6] 
HI541WriteCmosSensor(0xC808, 0x8000); // af_ctl8 // manual window setting + x_half[6] 

HI541WriteCmosSensor(0xC814, 0x1303); // AF window Region #1       	// Y Start
HI541WriteCmosSensor(0xC828, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC83C, 0xa601); 					// X Start	
HI541WriteCmosSensor(0xC850, 0x4a03); 					// X end	
HI541WriteCmosSensor(0xC816, 0x9f02); // AF window Region #2       	// Y Start
HI541WriteCmosSensor(0xC82A, 0xe504); 					// Y end	
HI541WriteCmosSensor(0xC83E, 0x3204); 					// X Start	
HI541WriteCmosSensor(0xC852, 0x5206); 					// X end	
HI541WriteCmosSensor(0xC818, 0x1303); // AF window Region #3       	// Y Start
HI541WriteCmosSensor(0xC82C, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC840, 0x3107); 					// X Start	
HI541WriteCmosSensor(0xC854, 0xd508); 					// X end	
HI541WriteCmosSensor(0xC81A, 0x5b05); // AF window Region #4       	// Y Start
HI541WriteCmosSensor(0xC82E, 0x8206); 					// Y end	
HI541WriteCmosSensor(0xC842, 0x6e04); 					// X Start	
HI541WriteCmosSensor(0xC856, 0x1206); 					// X end	
HI541WriteCmosSensor(0xC81C, 0x4803); // AF window Region #5       	// Y Start
HI541WriteCmosSensor(0xC830, 0x3804); 					// Y end	
HI541WriteCmosSensor(0xC844, 0xEA04); 					// X Start	
HI541WriteCmosSensor(0xC858, 0x9E05); 					// X end	
HI541WriteCmosSensor(0xC81E, 0x1303); // AF window Region #6       	// Y Start
HI541WriteCmosSensor(0xC832, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC846, 0xa601); 					// X Start	
HI541WriteCmosSensor(0xC85A, 0x4a03); 					// X end	
HI541WriteCmosSensor(0xC820, 0x9f02); // AF window Region #7       	// Y Start
HI541WriteCmosSensor(0xC834, 0xe504); 					// Y end	
HI541WriteCmosSensor(0xC848, 0x3204); 					// X Start	
HI541WriteCmosSensor(0xC85C, 0x5206); 					// X end	
HI541WriteCmosSensor(0xC822, 0x1303); // AF window Region #8       	// Y Start
HI541WriteCmosSensor(0xC836, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC84A, 0x3107); 					// X Start	
HI541WriteCmosSensor(0xC85E, 0xd508); 					// X end	
HI541WriteCmosSensor(0xC824, 0x5b05); // AF window Region #9       	// Y Start
HI541WriteCmosSensor(0xC838, 0x8206); 					// Y end	
HI541WriteCmosSensor(0xC84C, 0x6e04); 					// X Start	
HI541WriteCmosSensor(0xC860, 0x1206); 					// X end	
HI541WriteCmosSensor(0xC826, 0x4803); // AF window Region #10       	// Y Start
HI541WriteCmosSensor(0xC83A, 0x3804); 					// Y end	
HI541WriteCmosSensor(0xC84E, 0xEA04); 					// X Start	
HI541WriteCmosSensor(0xC862, 0x9E05); 					// X end

//================================
//======== FINDBAND Start ========
//================================
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0xC002,0x4608);
HI541WriteCmosSensor(0xC004,0x4005);

//================================
//======== FINDBAND End   ========
//================================

//================================
//========= SYSTEM Start==========
//================================
//HI541WriteCmosSensor(0xffff,0x0040);
//HI541WriteCmosSensor(0x3824,0x0000);
//HI541WriteCmosSensor(0x3826,0x5a10);//MCU preview hif cmd

//HI541WriteCmosSensor(0x0100,0x0100);	//streaming enable
//================================
//========= SYSTEM End  ==========
//================================

//================================
//========= AE min Start==========
//================================
HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x0310,0x282b);
HI541WriteCmosSensor(0x0312,0x0000);//

HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x548a,0x0004);//

//================================
//========= AE min End  ==========
//================================

//================================
//====Return to Preview Start=====
//================================
// Para
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x3824,0x0000);
// Command
HI541WriteCmosSensor(0x3826,0x5a10);
//================================
//====Return to Preview End=======
//================================

HI541WriteCmosSensor(0x0100,0x0100);	//streaming eble

//END
//[END]

			

				
	
	

	}
	
	else{

		
	FrameRate=15;
			  //Video mode 15
			  
	//SENSORDB("[HI541]HI541YUVSetVideoMode FrameRate:%d;\n",FrameRate);
	

//[USERSET_2]
//DISP_NAME = "Video_15"
//DISP_WIDTH = 1280	
//DISP_HEIGHT = 960
//MCLK = 84.00
//PLL = 1.00

//BEGIN

///////////////////////////////////////////
// Preview SXGA SUB Setting
// 1280x960@30fps, MIPI_YUV422, 1/2 Digital Scalex1/2 Analog Sub-sampling
///////////////////////////////////////////
//I2C_ID = 0x40
//I2C_BYTE  = 0x22

//================================
//========= SYSTEM Start =========
//================================
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x0100,0x0000);	//streaming disable
HI541WriteCmosSensor(0x5402,0x0004);	//tg_ctl3
HI541WriteCmosSensor(0x0900,0x0002);	//binning
HI541WriteCmosSensor(0x0340,0xf503);//frame_length_lines  //15 fps
HI541WriteCmosSensor(0x0342,0xca0a);//line_length_pck
HI541WriteCmosSensor(0x0344,0x1000);//x_addr_start
HI541WriteCmosSensor(0x0346,0x0e00);//y_addr_start
HI541WriteCmosSensor(0x0348,0x570a);//x_addr_end
HI541WriteCmosSensor(0x034a,0xc907);//y_addr_end
HI541WriteCmosSensor(0x034c,0x0005);//x_output_size
HI541WriteCmosSensor(0x034e,0xc003);//y_output_size
HI541WriteCmosSensor(0x0380,0x0100);//x_even_inc
HI541WriteCmosSensor(0x0382,0x0100);//x_odd_inc
HI541WriteCmosSensor(0x0384,0x0100);//y_even_inc
HI541WriteCmosSensor(0x0386,0x0300);//y_odd_inc

HI541WriteCmosSensor(0x540e,0x3677);//JH.LEE_140904
HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x0384,0xc073);//L
HI541WriteCmosSensor(0x0386,0x2600);//H exposure max 100 16.67fps
HI541WriteCmosSensor(0x0388,0x80b9);//L
HI541WriteCmosSensor(0x038a,0x2a00);//H exposure max 120 15fps
//================================
//========= SYSTEM End ===========
//================================

//================================
//======= ISP HW Start ===========
//================================
HI541WriteCmosSensor(0xffff,0x0040);

// ISP enable
HI541WriteCmosSensor(0x4830,0xfeef);
HI541WriteCmosSensor(0x4832,0x7f7a);
HI541WriteCmosSensor(0x4834,0x0401);
  
// BScaler
HI541WriteCmosSensor(0x6000,0x2800);//mode_byrsc1
HI541WriteCmosSensor(0x6002,0x2000);//byrsc_fifo_delay

//Yscaler 1280x960
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0xa800,0x2000); //mode_zoom1
HI541WriteCmosSensor(0xa804,0x0005); //zoom_dst_width
HI541WriteCmosSensor(0xa806,0xc003); //zoom_dst_height
		HI541WriteCmosSensor(0xa810,0x1308); //zoom_hor_step
HI541WriteCmosSensor(0xa812,0x1908); //zoom_ver_step
		HI541WriteCmosSensor(0xa814,0x3E02); //zoom_hor_step_remain
HI541WriteCmosSensor(0xa816,0x9909); //zoom_ver_step_remain
		HI541WriteCmosSensor(0xa818,0x1000); //zoom_fifo_delay
HI541WriteCmosSensor(0xa824,0x0f00); //zoom_intpol1
HI541WriteCmosSensor(0xa826,0x0000); //zoom_intpol3

// Iridix
HI541WriteCmosSensor(0x8404,0x1405);//hdr_frame_width
HI541WriteCmosSensor(0x8406,0xd003);//hdr_frame_height
//================================
//======= ISP HW End   ===========
//================================

//================================
//=========== SSD Start ==========
//================================
//Hardware SSD SET
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0xc414,0x0000);//ae_patch_xy_offset
HI541WriteCmosSensor(0xc416,0xa03c);//ae_patch_xy_w_h x 16
HI541WriteCmosSensor(0xc418,0x0404);//awb_size_xy_offset
HI541WriteCmosSensor(0xc41a,0xa034);//awb_size_xy_w_h x 16

//Firmware SSD SET
HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x1fe4,0x0404);//awb_size_y_offset
HI541WriteCmosSensor(0x1fe6,0x200a);//awb_size_xy_w_h x 16
HI541WriteCmosSensor(0x1fe8,0x0404);//ae_size_xy_offset
HI541WriteCmosSensor(0x1fea,0xa034);//ae_size_xy_w_h x 16

//================================
//=========== SSD End   ==========
//================================

//=====================================================
//=========== AF Filter Start - preview    ============
//=====================================================
HI541WriteCmosSensor(0xffff, 0x0040);
HI541WriteCmosSensor(0xC806, 0xffC3); // af_ctl8 // manual window setting + x_half[6] 
HI541WriteCmosSensor(0xC808, 0x8000); // af_ctl8 // manual window setting + x_half[6] 

HI541WriteCmosSensor(0xC814, 0x1303); // AF window Region #1       	// Y Start
HI541WriteCmosSensor(0xC828, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC83C, 0xa601); 					// X Start	
HI541WriteCmosSensor(0xC850, 0x4a03); 					// X end	
HI541WriteCmosSensor(0xC816, 0x9f02); // AF window Region #2       	// Y Start
HI541WriteCmosSensor(0xC82A, 0xe504); 					// Y end	
HI541WriteCmosSensor(0xC83E, 0x3204); 					// X Start	
HI541WriteCmosSensor(0xC852, 0x5206); 					// X end	
HI541WriteCmosSensor(0xC818, 0x1303); // AF window Region #3       	// Y Start
HI541WriteCmosSensor(0xC82C, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC840, 0x3107); 					// X Start	
HI541WriteCmosSensor(0xC854, 0xd508); 					// X end	
HI541WriteCmosSensor(0xC81A, 0x5b05); // AF window Region #4       	// Y Start
HI541WriteCmosSensor(0xC82E, 0x8206); 					// Y end	
HI541WriteCmosSensor(0xC842, 0x6e04); 					// X Start	
HI541WriteCmosSensor(0xC856, 0x1206); 					// X end	
HI541WriteCmosSensor(0xC81C, 0x4803); // AF window Region #5       	// Y Start
HI541WriteCmosSensor(0xC830, 0x3804); 					// Y end	
HI541WriteCmosSensor(0xC844, 0xEA04); 					// X Start	
HI541WriteCmosSensor(0xC858, 0x9E05); 					// X end	
HI541WriteCmosSensor(0xC81E, 0x1303); // AF window Region #6       	// Y Start
HI541WriteCmosSensor(0xC832, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC846, 0xa601); 					// X Start	
HI541WriteCmosSensor(0xC85A, 0x4a03); 					// X end	
HI541WriteCmosSensor(0xC820, 0x9f02); // AF window Region #7       	// Y Start
HI541WriteCmosSensor(0xC834, 0xe504); 					// Y end	
HI541WriteCmosSensor(0xC848, 0x3204); 					// X Start	
HI541WriteCmosSensor(0xC85C, 0x5206); 					// X end	
HI541WriteCmosSensor(0xC822, 0x1303); // AF window Region #8       	// Y Start
HI541WriteCmosSensor(0xC836, 0x6204); 					// Y end	
HI541WriteCmosSensor(0xC84A, 0x3107); 					// X Start	
HI541WriteCmosSensor(0xC85E, 0xd508); 					// X end	
HI541WriteCmosSensor(0xC824, 0x5b05); // AF window Region #9       	// Y Start
HI541WriteCmosSensor(0xC838, 0x8206); 					// Y end	
HI541WriteCmosSensor(0xC84C, 0x6e04); 					// X Start	
HI541WriteCmosSensor(0xC860, 0x1206); 					// X end	
HI541WriteCmosSensor(0xC826, 0x4803); // AF window Region #10       	// Y Start
HI541WriteCmosSensor(0xC83A, 0x3804); 					// Y end	
HI541WriteCmosSensor(0xC84E, 0xEA04); 					// X Start	
HI541WriteCmosSensor(0xC862, 0x9E05); 					// X end

//================================
//======== FINDBAND Start ========
//================================
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0xC002,0x4608);
HI541WriteCmosSensor(0xC004,0x4005);

//================================
//======== FINDBAND End   ========
//================================

//================================
//========= SYSTEM Start==========
//================================
// (10) Skhynix JHKIM - duplicated command
//HI541WriteCmosSensor(0xffff,0x0040);
//HI541WriteCmosSensor(0x3824,0x0000);
//HI541WriteCmosSensor(0x3826,0x5a10);//MCU preview hif cmd

//HI541WriteCmosSensor(0x0100,0x0100);	//streaming enable
//================================
//========= SYSTEM End  ==========
//================================

//================================
//========= AE min Start==========
//================================
HI541WriteCmosSensor(0xffff,0x0020);
HI541WriteCmosSensor(0x0310,0x282b);
HI541WriteCmosSensor(0x0312,0x0000);//

HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x548a,0x0004);//

//================================
//========= AE min End  ==========
//================================

//================================
//====Return to Preview Start=====
//================================
// Para
HI541WriteCmosSensor(0xffff,0x0040);
HI541WriteCmosSensor(0x3824,0x0000);
// Command
HI541WriteCmosSensor(0x3826,0x5a10);
//================================
//====Return to Preview End=======
//================================

HI541WriteCmosSensor(0x0100,0x0100);	//streaming eble

//END
//[END]
	

	}

}

void HI541_3ACtrl(ACDK_SENSOR_3A_LOCK_ENUM action)
{

	//SENSORDB(" HHL_HI541_3ACtrl is %d\n",action);

	switch (action)
	{
	case SENSOR_3A_AE_LOCK:
		HI541SetAeMode(KAL_FALSE);
		break;
	case SENSOR_3A_AE_UNLOCK:
		HI541SetAeMode(KAL_TRUE);
		break;

	case SENSOR_3A_AWB_LOCK:
		HI541SetAwbMode(KAL_FALSE);
		break;

	case SENSOR_3A_AWB_UNLOCK:
		HI541SetAwbMode(KAL_TRUE);
		break;
	default:
		break;
	}
	return;
}

void HI541GetExifInfo(UINT32 exifAddr)
{
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 28;
    pExifInfo->AEISOSpeed = hi541_isospeed;
    pExifInfo->AWBMode = HI541CurrentStatus.iWB;
    pExifInfo->CapExposureTime = hi541_exposuretime;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = hi541_isospeed;
}

#define FLASH_BV_THRESHOLD 0x10
static void HI541MIPI_FlashTriggerCheck(unsigned int *pFeatureReturnPara32)
{
unsigned int NormBr;
HI541WriteCmosSensor(0xffff,0x0020);
NormBr = HI541ReadCmosSensor(0x032A); 
printk("techainsh debug value of flash threshold is === %x \n",NormBr);
if (NormBr > FLASH_BV_THRESHOLD)
{
   *pFeatureReturnPara32 = FALSE;
   hi541_strobe = FALSE;
return;
}
*pFeatureReturnPara32 = TRUE;
hi541_strobe = TRUE;

return;
}
UINT32 HI541FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
						   UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
	SENSORDB("HI541_DebugHHL_enter the HI541FeatureControl function the FeatureId is %d\n",FeatureId);

	switch (FeatureId)
	{
	case SENSOR_FEATURE_GET_RESOLUTION:
		*pFeatureReturnPara16++=HI541_FULL_WIDTH;
		*pFeatureReturnPara16=HI541_FULL_HEIGHT;
		*pFeatureParaLen=4;
		break;
	case SENSOR_FEATURE_GET_PERIOD:
		*pFeatureReturnPara16++=HI541_PV_PERIOD_PIXEL_NUMS+HI541Status.PvDummyPixels;
		*pFeatureReturnPara16=HI541_PV_PERIOD_LINE_NUMS+HI541Status.PvDummyLines;
		*pFeatureParaLen=4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*pFeatureReturnPara32 = HI541Status.PvOpClk*2;
		*pFeatureParaLen=4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		if( CAPTURE_FLAG == 0)
			HI541NightMode((BOOL) *pFeatureData16);
		else
        {      
	    spin_lock(&HI541_drv_lock);
		CAPTURE_FLAG = 0;
		spin_unlock(&HI541_drv_lock);
        }
		break;
	case SENSOR_FEATURE_SET_GAIN:
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
    case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO:
    	HI541MIPI_FlashTriggerCheck(pFeatureData32);
    	SENSORDB("[HI541] F_GET_TRIGGER_FLASHLIGHT_INFO: %d\n", pFeatureData32);
    	break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		HI541WriteCmosSensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		pSensorRegData->RegData = HI541ReadCmosSensor(pSensorRegData->RegAddr);
		break;
	case SENSOR_FEATURE_GET_CONFIG_PARA:
		*pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
		break;
	case SENSOR_FEATURE_SET_CCT_REGISTER:
	case SENSOR_FEATURE_GET_CCT_REGISTER:
	case SENSOR_FEATURE_SET_ENG_REGISTER:
	case SENSOR_FEATURE_GET_ENG_REGISTER:
	case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
	case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
	case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
	case SENSOR_FEATURE_GET_GROUP_INFO:
	case SENSOR_FEATURE_GET_ITEM_INFO:
	case SENSOR_FEATURE_SET_ITEM_INFO:
	case SENSOR_FEATURE_GET_ENG_INFO:
		break;
	case SENSOR_FEATURE_GET_GROUP_COUNT:
		*pFeatureReturnPara32++=0;
		*pFeatureParaLen=4;
		break; 
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
		// if EEPROM does not exist in camera module.
		*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
		*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_INITIALIZE_AF:
			SENSORDB("[HI541]SENSOR_FEATURE_INITIALIZE_AF\n");
			HI541_FOCUS_AFC_Init();
            break;
		case SENSOR_FEATURE_GET_AF_STATUS:
			SENSORDB("[HI541]SENSOR_FEATURE_GET_AF_STATUS\n");
            HI541_FOCUS_OVT_AFC_Get_AF_Status(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break;
		case SENSOR_FEATURE_SINGLE_FOCUS_MODE:
			SENSORDB("[HI541]SENSOR_FEATURE_SINGLE_FOCUS_MODE\n");
			HI541_FOCUS_AFC_Single_Focus();
            break;
		case SENSOR_FEATURE_CONSTANT_AF:
			SENSORDB("[HI541]SENSOR_FEATURE_CONSTANT_AF\n");
			HI541_FOCUS_AFC_Constant_Focus();
			break;
		case SENSOR_FEATURE_CANCEL_AF:
			SENSORDB("[HI541]SENSOR_FEATURE_CANCEL_AF\n");
            HI541_FOCUS_AFC_Cancel_Focus();
            break;
		case SENSOR_FEATURE_GET_AF_INF:
			SENSORDB("[HI541]SENSOR_FEATURE_GET_AF_INF\n");
            HI541_FOCUS_Get_AF_Inf(pFeatureReturnPara32);
            *pFeatureParaLen=4;            
            break;
		case SENSOR_FEATURE_GET_AF_MACRO:
			SENSORDB("[HI541]SENSOR_FEATURE_GET_AF_MACRO\n");
            HI541_FOCUS_Get_AF_Macro(pFeatureReturnPara32);
            *pFeatureParaLen=4;            
            break;
		case SENSOR_FEATURE_SET_AF_WINDOW: 
			SENSORDB("[HI541]SENSOR_FEATURE_SET_AF_WINDOW\n");
			Hi541_FOCUS_Set_AF_Window(*pFeatureData32);
            break;       					
        case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
			SENSORDB("[HI541]SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS\n");
            HI541_FOCUS_Get_AF_Max_Num_Focus_Areas(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break; 			
        case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
			SENSORDB("[HI541]AE zone addr = 0x%x\n",*pFeatureData32);
            HI541_FOCUS_Get_AE_Max_Num_Metering_Areas(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break;        
        case SENSOR_FEATURE_SET_AE_WINDOW:
            SENSORDB("[HI541]AE zone addr = 0x%x\n",*pFeatureData32);			
            HI541_FOCUS_Set_AE_Window(*pFeatureData32);
            break; 
	case SENSOR_FEATURE_SET_YUV_CMD:
		HI541YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		HI541YUVSetVideoMode(*pFeatureData16);
		break; 
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		HI541GetSensorID(pFeatureReturnPara32); 
		break; 
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		HI541SetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		HI541GetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
		break;

	case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
		HI541GetAEAWBLock((*pFeatureData32),*(pFeatureData32+1));
		break;
	case SENSOR_FEATURE_GET_DELAY_INFO:
		SENSORDB("SENSOR_FEATURE_GET_DELAY_INFO\n");
		HI541GetDelayInfo(*pFeatureData32);
		break;
	case SENSOR_FEATURE_SET_YUV_3A_CMD:
		HI541_3ACtrl((ACDK_SENSOR_3A_LOCK_ENUM)*pFeatureData32);
		break;
	case SENSOR_FEATURE_AUTOTEST_CMD:
		SENSORDB("SENSOR_FEATURE_AUTOTEST_CMD\n");
		HI541AutoTestCmd((*pFeatureData32),*(pFeatureData32+1));
		break;
    case SENSOR_FEATURE_GET_EXIF_INFO:       
        HI541GetExifInfo(*pFeatureData32);
        break;
	default:
		break;
	}
	return ERROR_NONE;
} /* HI541FeatureControl() */

SENSOR_FUNCTION_STRUCT SensorFuncHI541=
{
	HI541Open,
	HI541GetInfo,
	HI541GetResolution,
	HI541FeatureControl,
	HI541Control,
	HI541Close
};


UINT32 HI541_MIPI_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{   


	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncHI541;
	return ERROR_NONE;
};
