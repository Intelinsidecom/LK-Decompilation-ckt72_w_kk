#ifndef __CUST_RTC_H__
#define __CUST_RTC_H__

/*
 * Default values for RTC initialization
 * Year (YEA)        : 1970 ~ 2037
 * Month (MTH)       : 1 ~ 12
 * Day of Month (DOM): 1 ~ 31
 * pengfei.zhong-HOPE-63,set default year to 2014
 */
#define RTC_DEFAULT_YEA		2015
#define RTC_DEFAULT_MTH		2
#define RTC_DEFAULT_DOM		1

#define RTC_2SEC_REBOOT_ENABLE 0
#define RTC_2SEC_MODE 0    //2:2s 1:1s 0:0.5s

#endif /* __CUST_RTC_H__ */
