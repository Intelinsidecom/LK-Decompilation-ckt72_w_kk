CKT_AUTO_ADD_GLOBAL_DEFINE_BY_NAME = CKT_INTERPOLATION LCD_UPDATE_TWO_TIMES CKT_ALSPS_TMD2772_CALI_SUPPORT TP_TYPE_NEW_SUPPORT
CKT_AUTO_ADD_GLOBAL_DEFINE_BY_NAME_VALUE =PROJ_NAME CUST_NAME SOFTCODE USB_MANUFACTURER_STRING USB_PRODUCT_STRING USB_STRING_SERIAL_IDX CUSTOM_EXIF_STRING_MAKE CUSTOM_EXIF_STRING_MODEL CUSTOM_EXIF_STRING_SOFTWARE CUSTOM_BTMTK_ANDROID_DEFAULT_REMOTE_NAME CUSTOM_BTMTK_ANDROID_DEFAULT_REMOTE_NAME CUSTOM_BTMTK_ANDROID_ADAPTER_PREFIX CUSTOM_BTMTK_ANDROID_DEFAULT_LOCAL_NAME CUSTOM_BTMTK_ANDROID_DEFAULT_REMOTE_NAME CUSTOM_BTMTK_ANDROID_ADAPTER_PREFIX
CKT_AUTO_ADD_GLOBAL_DEFINE_BY_VALUE = PROJ_NAME CUST_NAME LCD_BACKLIGHT_MODE
#############################
#############################
#############################

#项目的相关定义
PROJ_NAME = HOPE01A
CUST_NAME = DORO
SOFTCODE = S01A
BASEVERNO=401
#############################
#会用他设置ro.product.model
#CKT_PRODUCT_MODEL="DoroPhoneEasy820"
#会用他设置缺省时区persist.sys.timezone
TIMEZONE=Europe/Paris


############usb相关#################
USB_MANUFACTURER_STRING="DORO"
USB_PRODUCT_STRING="DoroLiberto820Mini"
USB_STRING_SERIAL_IDX=$(strip $(USB_PRODUCT_STRING) )

############exif相关#################
#CUSTOM_EXIF_STRING_MAKE="DORO"
#CUSTOM_EXIF_STRING_MODEL="DoroPhoneEasy820"
#CUSTOM_EXIF_STRING_SOFTWARE=""

############bt相关#################
#CUSTOM_BTMTK_ANDROID_DEFAULT_LOCAL_NAME =$(strip $(PROJ_NAME) )_BT
#CUSTOM_BTMTK_ANDROID_DEFAULT_REMOTE_NAME=$(strip $(PROJ_NAME) )_DEVICE
#CUSTOM_BTMTK_ANDROID_ADAPTER_PREFIX=$(strip $(CUST_NAME) )BT

#$(error $(CUSTOM_BTMTK_ANDROID_ADAPTER_PREFIX))
#############################
#功能的开关,会导入到mediatek/source/frameworks/featureoption/java/com/mediatek/featureoption/FeatureOption.java
#修改的时候注意,在 mediatek/build/tools/javaoption.pm中添加相关模块
#另外注意如果enable只可以用yes,不可以用其他
TESTA = yes
TESTB = no
TESTC = testc_none

#工程模式13646633显示其他信息,包括硬件和软件版本等
CKT_DISPLAY_OTHER_INFO_ENGINEERMODE = yes
#############################

#如果要固定版本号,请在这设置,否则注释调它,而不是留空!!!
#CKT_BUILD_VERNO = PANDORA-S0A_CKT_L2EN_111_111111
#CKT_BUILD_INTERNAL_VERNO =PANDORA-S0A_CKT_L2EN_111_111111111111

#############################
#摄像头软件插值,如果本身是800w摄像头,请直接打开它,比如使用ov8825
CKT_INTERPOLATION = no

#lcd背光的情况,可以使用USE_LED_PMIC_LCD_BOOST,USE_LED_MODE_PWM,目前由于闪烁问题暂时使用USE_LED_PMIC_LCD_BOOST
LCD_BACKLIGHT_MODE=USE_LED_PMIC_LCD_BOOST

#LCD update screen two times controler,because MT6575M can only up to HVGA, but now need up to WVGA,so must default this.
LCD_UPDATE_TWO_TIMES = yes

#alsps is tmd2772,need to add the offset calibration
CKT_ALSPS_TMD2772_CALI_SUPPORT = yes














































###########以下为产生的东西,一般不需要理会
_CKT_BUILD_VERNO  = $(strip $(PROJ_NAME) )-$(strip $(SOFTCODE) )_$(strip $(CUST_NAME) )_L$(words $(subst hdpi, ,$(strip $(MTK_PRODUCT_LOCALES))))$(word 1,$(subst _, ,$(subst zh_TW,TR,$(subst zh_CN,SM,$(strip $(MTK_PRODUCT_LOCALES))))))_$(strip $(BASEVERNO))

DATA_FOR_VERO=$(shell date +%y%m%d)
DATA_FOR_INTERNAL_VERO=$(shell date +%y%m%d%H%M%S)

DORO_BUILD_NO ?=  $(call uc , $(strip $(PROJ_NAME) )-$(strip $(SOFTCODE) )_$(strip $(BASEVERNO)))
DORO_SOFTCODE = $(call uc , $(SOFTCODE))

CKT_BUILD_VERNO  ?= $(call uc, $(_CKT_BUILD_VERNO)_$(strip $(DATA_FOR_VERO)))
CKT_BUILD_INTERNAL_VERNO  ?= $(call uc, $(_CKT_BUILD_VERNO)_$(strip $(DATA_FOR_INTERNAL_VERO)))

MTK_BUILD_VERNO  ?= $(call uc, $(_CKT_BUILD_VERNO)_$(strip $(DATA_FOR_VERO)))


#############################

