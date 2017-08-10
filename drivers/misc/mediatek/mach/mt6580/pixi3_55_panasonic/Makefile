
include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifneq ($(CONFIG_ARCH_MTK_PROJECT),)
ARCH_MTK_PROJECT := $(shell echo $(CONFIG_ARCH_MTK_PROJECT))
obj-y   += core/
obj-y   += dct/dct/
endif

#add by jinlong.sang for 252656
ifeq (true,$(strip $(ENABLE_USER2ENG)))
ccflags-y += -DUSER2ENG
ccflags-y += -DEVB_PLATFORM
endif
#end by jinlong.sang for 252656

ifneq ($(CONFIG_ARM64), y)
obj-y   +=  camera/camera/
endif
#obj-y   +=  flashlight/constant_flashlight/
obj-$(CONFIG_MTK_VIBRATOR)   +=  vibrator/
obj-$(CONFIG_MTK_LEDS)   +=  leds/
#obj-y   +=  keypad/
#obj-y   +=  sound/
obj-$(CONFIG_MTK_ACCDET)   +=  accdet/
obj-y   += dct/dct/

ifeq ($(CONFIG_MTK_SENSOR_SUPPORT),y)
ifeq ($(CONFIG_MTK_CM36652_NEW),y)
obj-y	+=  alsps/cm36652-new/
endif
ifeq ($(CONFIG_MTK_STK3X1X_NEW),y)
obj-y	+=  alsps/stk3x1x-new/
endif
ifeq ($(CONFIG_MTK_BMA222E_NEW),y)
obj-y	+=  accelerometer/bma222E-new/
endif
ifeq ($(CONFIG_MTK_STK3X1X_NEW_PIXI355),y)
obj-y	+=  alsps/stk3x1x-new-pixi355/
endif
ifeq ($(CONFIG_MTK_MC3410_NEW),y)
obj-y   +=  accelerometer/mc3410-new/
endif

# jiangjingjing-add-for-pixi3-55-20150521-begin
ifeq ($(CONFIG_MTK_KXTJ2_1009_NEW),y)
obj-y   +=  accelerometer/KXTJ2_1009-new/
endif

ifeq ($(CONFIG_MTK_AKM09911_NEW_PIXI355),y)
obj-y   +=  magnetometer/akm09911-new_pixi355/
endif
# jiangjingjing-add-for-pixi3-55-20150521-end

ifeq ($(CONFIG_MTK_ITG1010_NEW),y)
obj-y   +=  gyroscope/ITG1010-new/
endif

ifeq ($(CONFIG_MTK_BMG160),y)
obj-y   +=  gyroscope/BMG160/
endif

#remove to makefile.mt6580,wenzhiquan20150611
#ifeq ($(CONFIG_TUNER),y)
#obj-y	+=  tuner/fc8180/
#endif

endif

obj-$(CONFIG_HW_INFO) += hq_hardwareinfo.o
#obj-y	+=  tuner/fc8180/
