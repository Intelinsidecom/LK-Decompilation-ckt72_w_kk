/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.h
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 *   Header file of Sensor driver
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
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
/* SENSOR FULL SIZE */
#ifndef __SENSOR_H
#define __SENSOR_H
    
#define HI541_WRITE_ID        0x40

#define HI541_GRAB_START_X    (1)
#define HI541_GRAB_START_Y    (1)
#define HI541_PV_WIDTH        (1280 - 8)
#define HI541_PV_HEIGHT       (960 - 6)
#define HI541_FULL_WIDTH      (2592 - 16)
#define HI541_FULL_HEIGHT     (1944 - 12)

/* Sesnor Pixel/Line Numbers in One Period */  
#define HI541_PV_PERIOD_PIXEL_NUMS      (816)    /* Default preview line length */
#define HI541_PV_PERIOD_LINE_NUMS       (612)     /* Default preview frame length */
#define HI541_FULL_PERIOD_PIXEL_NUMS    (1620)    /* Default full size line length */
#define HI541_FULL_PERIOD_LINE_NUMS     (1220)    /* Default full size frame length */

/* Sensor Exposure Line Limitation */
#define HI541_PV_EXPOSURE_LIMITATION        (0x750)
#define HI541_FULL_EXPOSURE_LIMITATION      (0xfa0)

#define HI541_FRAME_RATE_UNIT         10
#define HI541_FPS(x)                  (HI541_FRAME_RATE_UNIT * (x))
#define HI541_MAX_FPS                 (HI541_FRAME_RATE_UNIT * 30)
typedef enum {
    SENSOR_MODE_INIT = 0,
    SENSOR_MODE_PREVIEW,
    SENSOR_MODE_VIDEO,
    SENSOR_MODE_ZSD,
    SENSOR_MODE_CAPTURE
} HI541_SENSOR_MODE;

UINT32 HI541Open(void);
UINT32 HI541GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 HI541GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 HI541Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 HI541FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 HI541Close(void);
#endif /* __SENSOR_H */
