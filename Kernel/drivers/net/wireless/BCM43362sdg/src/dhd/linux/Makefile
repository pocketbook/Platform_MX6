
# GNU Makefile for Broadcom Dongle Host Driver
#
# Copyright (C) 2011, Broadcom Corporation
# All Rights Reserved.
# 
# THIS SOFTWARE IS OFFERED "AS IS", AND BROADCOM GRANTS NO WARRANTIES OF ANY
# KIND, EXPRESS OR IMPLIED, BY STATUTE, COMMUNICATION OR OTHERWISE. BROADCOM
# SPECIFICALLY DISCLAIMS ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A SPECIFIC PURPOSE OR NONINFRINGEMENT CONCERNING THIS SOFTWARE.
#
# $Id: Makefile,v 1.93.6.24 2011-02-10 02:31:08 $
#

# Although 32ON64 flag is supposed to be passed from caller, set it 
# if not set by caller on certain kernels and platforms
ifeq ($(shell uname -m),x86_64)
     ifneq ($(findstring x86_64,$(LINUXVER)),x86_64)
          export 32ON64=1
          # $(warning Passing 32ON64 flag to force 32bit driver for $(LINUXVER))
     endif # LINUXVER
endif # CURPROC

# Try a couple of places for LINUXDIR if not specified
ifeq ($(LINUXDIR),)
ifeq ($(LINUXVER),)
# Neither one is specified, use uname for version
LINUXVER := $(shell uname -r)
endif
ifneq ($(wildcard /lib/modules/$(LINUXVER)/build/include/linux/version.h),)
LINUXDIR := /lib/modules/$(LINUXVER)/build
else
ifneq ($(wildcard /tools/linux/src/linux-$(LINUXVER)/include/linux/version.h),)
LINUXDIR := /tools/linux/src/linux-$(LINUXVER)
else
LINUXDIR := /usr/src/linux
endif
endif
endif

# Derive LINUXVER from LINUXDIR
MYKERNEL_RELEASE_KEYWORD:="KERNELRELEASE[[:space:]]*=.*kernel.release"
MYKERNEL_DEFINITION:=$(if \
  $(shell grep $(MYKERNEL_RELEASE_KEYWORD) $(LINUXDIR)/Makefile 2> /dev/null),\
  grep $(MYKERNEL_RELEASE_KEYWORD) $(LINUXDIR)/Makefile,\
  cat $(LINUXDIR)/Makefile)

LINUXVER:=$(shell ($(MYKERNEL_DEFINITION); echo "show_kernel_version_number$$$$:;@echo \$$(KERNELRELEASE)") 2> /dev/null | $(MAKE) --no-print-directory -k -C $(LINUXDIR) MYUNAME="" -f - show_kernel_version_number$$$$ 2> /dev/null)

ifeq ($(LINUXVER),)
     $(error LINUXVER=$(LINUXVER) is empty)
endif

# Keep consistency with Linux kernel naming rules
KRNL_VERSION	:= $(shell echo $(LINUXVER) | sed -n 's/^\([0-9]\)\..*/\1/p')
ifneq ($(KRNL_VERSION),2)
KRNL_PATCHLEVEL	:= $(shell echo $(LINUXVER) | sed -n 's/^3\.\([0-9]\+\).*/\1/p')
else
KRNL_PATCHLEVEL	:= $(shell echo $(LINUXVER) | sed -n 's/^2\.\([0-9]\+\).*/\1/p')
KRNL_SUBLEVEL	:= $(shell echo $(LINUXVER) | cut -c5-6)
endif

ifneq ($(words $(KRNL_VERSION) $(KRNL_PATCHLEVEL)),2)
     $(error Could not parse LINUXVER=$(LINUXVER))
endif

KERNELVER_2_5_0_OR_LATER := $(strip $(shell \
		if [ $(KRNL_VERSION) -lt 3 ]; then \
			if [ $(KRNL_PATCHLEVEL) -lt 5 ]; then \
				echo FALSE; \
			else \
				echo TRUE; \
			fi \
		else \
			echo TRUE; \
		fi \
	))

# 2.6.37 kernel and older kernel need compat wireless
KRNL_NEED_COMPAT_WIRELESS := $(strip $(shell \
		if [ $(KRNL_VERSION) -eq 2 ]; then \
			if [ $(KRNL_PATCHLEVEL) -eq 6 ]; then \
				if [ $(KRNL_SUBLEVEL) -le 37 ]; then \
					echo TRUE; \
				else \
					echo FALSE; \
				fi \
			else\
				echo FALSE; \
			fi \
		else \
			echo FALSE; \
		fi \
	))

KERNELVER_2_6_35_OR_LATER := $(strip $(shell \
		if [ $(KRNL_VERSION) -lt 3 ]; then \
			if [ $(KRNL_PATCHLEVEL) -ge 6 ]; then \
				if [ $(KRNL_SUBLEVEL) -ge 35 ]; then \
					echo TRUE; \
				else \
					echo FALSE; \
				fi \
			else \
				echo FALSE; \
			fi \
		else \
			echo TRUE; \
		fi \
	))

# Allow CROSS_COMPILE to specify compiler base
CC := $(CROSS_COMPILE)gcc
LD := $(CROSS_COMPILE)ld
NM := $(CROSS_COMPILE)nm
OBJCOPY := $(CROSS_COMPILE)objcopy

# driver source base and C file path
ifeq ($(SRCBASE),)
SRCBASE := $(shell /bin/pwd)/../..
endif
vpath %.c $(SRCBASE)/dhd/sys $(SRCBASE)/shared $(SRCBASE)/bcmsdio/sys $(SRCBASE)/wl/sys $(SRCBASE)/wl/phy

## Initialize DFLAGS
DFLAGS :=
CFILES :=


# basic options (defines in DFLAGS, includes in IFLAGS)
DFLAGS += -DLINUX -DSRCBASE=\"$(SRCBASE)\" -DBCMDRIVER -DBCMDONGLEHOST -DBCMSUP_PSK -DWLFBT
DFLAGS += -DUNRELEASEDCHIP -DBCMDMA32
DFLAGS += -DBCMFILEIMAGE
DFLAGS += -DPROP_TXSTATUS
ifeq ($(BCMQT),1)
	DFLAGS += -DBCMSLTGT -DBCMQT
endif
ifeq ($(WLTEST),1)
	DFLAGS += -DIOCTL_RESP_TIMEOUT=20000
	DFLAGS += -DDHD_SPROM
endif

# Past 2.6.29 kernels, arch specific bits are re-organized in linux kernel. So
# append new include paths to existing ones to get 2.6.29+ kernels compile

# Default DHDARCH is x86
DHDARCH ?= x86
ifneq ($(findstring native,$(TARGET)),)
   DHDARCH = x86
endif
ifneq ($(findstring mips,$(TARGET)),)
   DHDARCH = mips
endif
ifneq ($(findstring arm,$(TARGET)),)
   DHDARCH = arm
endif

ifneq ($(findstring arm,$(CROSS_COMPILE)),)
   DFLAGS += -DDHDARCH_ARM
else
   DFLAGS += -DDHDARCH_X86
endif

# First include from linux kernel dirs
IFLAGS := -I$(LINUXDIR)/include 
IFLAGS += -I$(LINUXDIR)/include/asm/mach-default 
# Followed by 2.6.29+ specific paths
IFLAGS += -I$(LINUXDIR)/arch/$(DHDARCH)/include
IFLAGS += -I$(LINUXDIR)/arch/$(DHDARCH)/include/asm/mach-default

# From current workspace
IFLAGS += -I. 
IFLAGS += -I$(SRCBASE)/include 
IFLAGS += -I$(SRCBASE)/shared 
IFLAGS += -I$(SRCBASE)/dhd/sys 
IFLAGS += -I$(SRCBASE)/dongle 
IFLAGS += -I$(SRCBASE)/wl/sys
IFLAGS += -I$(SRCBASE)/wl/phy

ifneq ($(wildcard $(LINUXDIR)/.config),)
include $(LINUXDIR)/.config
else
# This is dangerous, since we don't know if they are really configured.
CONFIG_WIRELESS_EXT=y
DFLAGS += -DCONFIG_WIRELESS_EXT
endif

ifeq ($(CONFIG_MMC_MSM7X00A),y) 
DFLAGS += -Dlinux
DFLAGS += -DDHD_SDALIGN=64 -DMAX_HDR_READ=64 -DDHD_FIRSTREAD=64 
endif 

WFLAGS := -Wall -Wstrict-prototypes
ifeq (,$(findstring 2.4.18,$(LINUXVER)))
WFLAGS += -Werror
endif

ifeq ($(filter arm% mips%,$(TARGETARCH))$(filter arm% mips%,$(ARCH)),)
    ifneq ($(filter 2.6.38% 2.6.39% 2.6.40%,$(LINUXVER)),)
        WFLAGS += -Wno-unused-but-set-variable
        WFLAGS += -Wno-unused-but-set-parameter
        WFLAGS += -Wframe-larger-than=1024
        WFLAGS += -maccumulate-outgoing-args
    endif # LINUXVER
endif # TARGETARCH

CFILES += dhd_linux.c dhd_linux_sched.c linux_osl.c bcmutils.c bcmevent.c dhd_common.c
CFILES += dhd_custom_gpio.c siutils.c sbutils.c aiutils.c hndpmu.c

# threading options
ifeq ($(findstring -nothread-,-$(TARGET)-),)
DFLAGS += -DDHDTHREAD
endif

# Building hsic for 43239, 43236, 4330 and 4334
ifneq ($(findstring -hsic-,-$(TARGET)-),)
DFLAGS += -DEXTENDED_VID_PID=' { USB_DEVICE(0x0a5c, 0xbd1b) }, \
                               { USB_DEVICE(0x0a5c, 0xbd17) }, \
                               { USB_DEVICE(0x0a5c, 0xbd19) }, \
                               { USB_DEVICE(0x0a5c, 0xbd1c) }, \
                                { USB_DEVICE(0x0a5c, 0xbd1a) }'
#Enable interrupt ep support for HSIC driver
DFLAGS += -DINTR_EP_ENABLE
else
# Compiled out the BTAMP code for HSIC (for now).
CFILES += dhd_bta.c
DFLAGS += -DWLBTAMP
endif
ifeq ($(WLTEST),1)
	CFILES += bcmsrom.c bcmotp.c
endif

OFILES=$(CFILES:.c=.o)

# Make debug a separate option
ifneq ($(findstring -debug-,-$(TARGET)-),)
DFLAGS += -DDHD_DEBUG -DSDTEST
DFLAGS += -DPROP_TXSTATUS_DEBUG
endif

# Make big-endian a separate option
ifneq ($(findstring -be-,-$(TARGET)-),)
DFLAGS += -DIL_BIGENDIAN
endif

ifneq ($(findstring -dnglimage-,-$(TARGET)-),)
## Embeddable dongle image name
DNGL_IMAGE_NAME?=4330b2-roml/sdio-ag
DFLAGS += -DBCM_DNGL_EMBEDIMAGE -DIMAGE_NAME="$(DNGL_IMAGE_NAME)"
IFLAGS += -I$(SRCBASE)/dongle/rte/wl/builds/$(DNGL_IMAGE_NAME)
endif

ifneq ($(findstring -cdc-,-$(TARGET)-),)
DFLAGS += -DBDC -DTOE 
DFLAGS += -DDHD_BCMEVENTS -DSHOW_EVENTS
CFILES += dhd_cdc.c
endif
ifneq ($(findstring -rndis-,-$(TARGET)-),)
DFLAGS += -DRNDIS
CFILES += dhd_rndis.c
endif
		  
ifneq ($(findstring -dbususb-,-$(TARGET)-),)
DFLAGS += -DBCMDHDUSB -DBCMDBUS
CFILES += dbus.c dbus_usb.c dbus_usb_linux.c
endif
ifneq ($(findstring -dbususb-dnglimage-,-$(TARGET)-),)
DFLAGS += -DBCMDHDUSB -DBCMDBUS
DFLAGS += -DBCM_DNGL_EMBEDIMAGE
CFILES += dbus.c dbus_usb.c dbus_usb_linux.c
endif
ifneq ($(findstring -usb-,-$(TARGET)-),)
DFLAGS += -DBCMDHDUSB -DBCMDBUS
CFILES += dbus.c dbus_usb.c dbus_usb_linux.c
endif
ifneq ($(findstring -dbus-,$(TARGET)-),)
DFLAGS += -DBCMDBUS -DBCMSDIO
CFILES += dbus.c dbus_sdio.c dbus_sdio_linux.c
endif
ifneq ($(findstring -dbussdstd-,$(TARGET)-),)
DFLAGS += -DBCMSDIO -DBCMSDIOH_STD
CFILES += bcmsdh.c bcmsdstd.c bcmsdstd_linux.c bcmsdh_linux.c
DFLAGS += -DBCMDBUS
CFILES += dbus.c dbus_sdio.c dbus_sdio_linux.c
endif
ifneq ($(findstring -sdio-,-$(TARGET)-),)
DFLAGS += -DBCMSDIO
CFILES += dhd_sdio.c
endif
ifneq ($(findstring -sdstd-,$(TARGET)-),)
DFLAGS += -DBCMSDIO -DBCMSDIOH_STD
CFILES += dhd_sdio.c bcmsdh.c bcmsdstd.c bcmsdstd_linux.c bcmsdh_linux.c
ifneq ($(findstring -hc3-,$(TARGET)-),)
DFLAGS += -DSDHOST3=1
endif
endif
ifeq ($(KERNELVER_2_6_35_OR_LATER), TRUE)
DFLAGS += -DARP_OFFLOAD_SUPPORT
endif
ifneq ($(findstring -nexus-,-$(TARGET)-),)
CFILES += wldev_common.c wl_android.c
CFILES += wl_linux_mon.c
DFLAGS += -Dlinux
DFLAGS += -DOEM_ANDROID
DFLAGS += -DBCMDBG
DFLAGS += -DCONFIG_DHD_STATIC_BUF
DFLAGS += -DCUSTOMER_HW2
DFLAGS += -DCUSTOM_OOB_GPIO_NUM=299
DFLAGS += -DOOB_INTR_ONLY
DFLAGS += -DMMC_SDIO_ABORT
DFLAGS += -DBCMOKC
#DFLAGS += -DCONFIG_FIRST_SCAN
DFLAGS += -DKEEP_ALIVE
DFLAGS += -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS -DEMBEDDED_PLATFORM

ifeq ($(KERNELVER_2_6_35_OR_LATER), TRUE)
$(warning : extra defintion for Android Gingerbread **)
DFLAGS += -DHW_OOB
DFLAGS += -DCSCAN
endif
ifeq ($(CONFIG_WIRELESS_EXT),y)
DFLAGS += -DSOFTAP
endif
CFILES += dhd_sdio.c bcmsdh_sdmmc.c bcmsdh.c bcmsdh_linux.c bcmsdh_sdmmc_linux.c

ifeq ($(CONFIG_WIRELESS_EXT),y)
## do not use the flag defined in the kernel config file(CONFIG_WIRELESS_EXT),
## use driver-wide flag instead (WL_WIRELESS_EXT).
DFLAGS += -DWL_WIRELESS_EXT
CFILES += wl_iw.c bcmwifi.c
endif

ifneq ($(findstring -cfg80211-,-$(TARGET)-),)

ifndef COMPAT_WIRELESS
ifeq ($(KRNL_NEED_COMPAT_WIRELESS), TRUE)
$(warning : old Kernel needs compat-wireless**)
COMPAT_WIRELESS=/projects/hnd/tools/linuxwireless/compat-wireless
export COMPAT_WIRELESS
endif
endif

DFLAGS += -DWL_CFG80211
DFLAGS += -DWLP2P
DFLAGS += -DWL_ENABLE_P2P_IF
ifdef COMPAT_WIRELESS
DFLAGS += -DWL_COMPAT_WIRELESS
endif

CFILES += wl_cfg80211.c
CFILES += wl_cfgp2p.c
CFILES += wldev_common.c
CFILES += bcmwifi.c
endif

else
ifneq ($(findstring -oob-,-$(TARGET)-),)
DFLAGS += -DOOB_INTR_ONLY
DFLAGS += -DHW_OOB
DFLAGS += -DMMC_SDIO_ABORT
else
ifneq ($(findstring -sdmmc-,-$(TARGET)-),)
ifeq ($(findstring -prime-,-$(TARGET)-),)
# -- PRIME is NOT using SDIO_ISR_THREAD --
DFLAGS += -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS -DSDIO_ISR_THREAD
endif
CFILES += dhd_sdio.c bcmsdh_sdmmc.c bcmsdh.c bcmsdh_linux.c bcmsdh_sdmmc_linux.c
endif
endif
endif

ifneq ($(CONFIG_WIRELESS_EXT),y)
#$(warning : == SoftAP == **)
DFLAGS += -DSOFTAP
endif

ifneq ($(findstring -dk8k-,-$(TARGET)-),)
CFILES += wldev_common.c wl_android.c
CFILES += wl_linux_mon.c
DFLAGS += -Dlinux
DFLAGS += -DOEM_ANDROID
DFLAGS += -DMMC_SDIO_ABORT 
DFLAGS += -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS 
endif

ifneq ($(findstring -prime-,-$(TARGET)-),)
$(info : ******* prime build **********************)
DFLAGS :=$(filter-out -DSOFTAP,$(DFLAGS))
DFLAGS :=$(filter-out -DPROP_TXSTATUS,$(DFLAGS))
CFILES += wldev_common.c wl_android.c
CFILES += wl_linux_mon.c
CFILES += wl_cfg80211.c wl_cfgp2p.c
CFILES += dhd_cfg80211.c 
CFILES += bcmwifi.c
DFLAGS += -Dlinux
DFLAGS += -DOEM_ANDROID
DFLAGS += -DMMC_SDIO_ABORT 
DFLAGS += -DWL_CFG80211
DFLAGS += -DCUSTOM_OOB_GPIO_NUM=2
ifneq ($(findstring -icsmr1-,-$(TARGET)-),)
$(warning : =====> IceCream MR1 P2P back compatible <======)
DFLAGS :=$(filter-out -DWL_ENABLE_P2P_IF,$(DFLAGS))
DFLAGS :=$(filter-out -DWL_CFG80211_STA_EVENT,$(DFLAGS))
DFLAGS :=$(filter-out -DDHD_USE_EARLYSUSPEND,$(DFLAGS))
endif
ifneq ($(findstring -jellybean-,-$(TARGET)-),)
$(warning : =====> Jellybean compatible <======)
DFLAGS += -DDHD_USE_EARLYSUSPEND
DFLAGS += -DWL_ENABLE_P2P_IF
DFLAGS += -DWL_CFG80211_STA_EVENT
DFLAGS += -DSET_RANDOM_MAC_SOFTAP
DFLAGS += -DWL_SCHED_SCAN
DFLAGS += -DCUSTOM_ROAM_TRIGGER_SETTING=-75
DFLAGS += -DCUSTOM_ROAM_DELTA_SETTING=10
DFLAGS += -DWL_SUPPORT_BACKPORTED_KPATCHES
endif
DFLAGS += -DBCMDRIVER -DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32				\
	 -DWLBTAMP -DBCMFILEIMAGE  																						\
	-DDHDTHREAD -DDHD_GPL -DDHD_SCHED -DDHD_DEBUG -DSDTEST -DBDC -DTOE    \
	-DDHD_BCMEVENTS -DSHOW_EVENTS -DBCMDBG               									\
	-DCUSTOMER_HW2 -DCUSTOM_OOB_GPIO_NUM=2 -DOOB_INTR_ONLY -DHW_OOB       \
	-DMMC_SDIO_ABORT -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS -DWLP2P     \
	-DNEW_COMPAT_WIRELESS -DWIFI_ACT_FRAME -DARP_OFFLOAD_SUPPORT          \
	-DKEEP_ALIVE -DCSCAN -DGET_CUSTOM_MAC_ENABLE -DPKT_FILTER_SUPPORT     \
	-DEMBEDDED_PLATFORM -DENABLE_INSMOD_NO_FW_LOAD -DPNO_SUPPORT
endif

ifneq ($(findstring -panda-,-$(TARGET)-),)
$(warning : =====> Panda ICS-1.1.0 CFG80211 with HW <======)
DFLAGS += -DBCMWAPI_WPI -DBCMWAPI_WAI
DFLAGS += -DDHD_USE_EARLYSUSPEND
ifneq ($(findstring -icsmr1-,-$(TARGET)-),)
$(warning : =====> IceCream MR1 P2P back compatible <======)
DFLAGS :=$(filter-out -DWL_ENABLE_P2P_IF,$(DFLAGS))
DFLAGS :=$(filter-out -DWL_CFG80211_STA_EVENT,$(DFLAGS))
endif
ifneq ($(findstring -jellybean-,-$(TARGET)-),)
$(warning : =====> Jellybean P2P back compatible <======)
DFLAGS += -DWL_ENABLE_P2P_IF
DFLAGS += -DWL_CFG80211_STA_EVENT
endif
DFLAGS :=$(filter-out -DSDIO_ISR_THREAD,$(DFLAGS))
DFLAGS :=$(filter-out -DSOFTAP,$(DFLAGS))
CFILES += wldev_common.c wl_android.c
CFILES += wl_linux_mon.c
DFLAGS += -DBOARD_PANDA
DFLAGS += -Dlinux
DFLAGS += -DOEM_ANDROID
DFLAGS += -DBCMDBG
DFLAGS += -DCUSTOMER_HW2
DFLAGS += -DBCM_REQUEST_FW
DFLAGS += -DMMC_SDIO_ABORT
DFLAGS += -DKEEP_ALIVE
DFLAGS += -DENABLE_INSMOD_NO_FW_LOAD
DFLAGS += -DEMBEDDED_PLATFORM
# cfg80211
CFILES += wl_cfg80211.c
CFILES += wl_cfgp2p.c
CFILES += dhd_cfg80211.c
CFILES += wldev_common.c
CFILES += bcmwifi.c
DFLAGS += -DWL_CFG80211 -DNEW_COMPAT_WIRELESS -DWLP2P
#HW_OOB
DFLAGS += -DOOB_INTR_ONLY
DFLAGS += -DHW_OOB
DFLAGS += -DCUSTOM_OOB_GPIO_NUM=138
endif

ifneq ($(findstring -cfg80211-,-$(TARGET)-),)

ifeq ($(findstring -prime-,-$(TARGET)-),)
# -- PRIME already set cfg80211 flags --
DFLAGS += -DWL_CFG80211
DFLAGS += -DWLP2P
ifdef COMPAT_WIRELESS
DFLAGS += -DWL_COMPAT_WIRELESS
endif
CFILES += wl_cfg80211.c
CFILES += dhd_cfg80211.c
CFILES += wl_cfgp2p.c
CFILES += wl_linux_mon.c
CFILES += wldev_common.c
CFILES += bcmwifi.c
endif

else

ifeq ($(CONFIG_WIRELESS_EXT),y)
## do not use the flag defined in the kernel config file(CONFIG_WIRELESS_EXT),
## use driver-wide flag instead (WL_WIRELESS_EXT).
DFLAGS += -DWL_WIRELESS_EXT
CFILES += wl_iw.c bcmwifi.c
endif

endif

ifneq ($(findstring -sdspi-,$(TARGET)-),)
DFLAGS += -DBCMSDIO -DBCMSDIOH_SPI -DTESTDONGLE # -DBCMSDYIELD
CFILES += dhd_sdio.c bcmsdh.c bcmsdspi.c bcmsdspi_linux.c bcmsdh_linux.c
endif
ifneq ($(findstring -gspi-,$(TARGET)-),)
DFLAGS += -DBCMSDIO -DBCMSDIOH_SPI -DTESTDONGLE -DBCMSPI
CFILES += dhd_sdio.c bcmsdh.c bcmspibrcm.c bcmsdspi_linux.c bcmsdh_linux.c
endif
ifneq ($(findstring -gspibcm,$(TARGET)-),)
DFLAGS += -DBCMSDIO -DBCMSDIOH_SPI -DTESTDONGLE -DBCMSPI
CFILES += dhd_sdio.c bcmsdh.c bcmgspi.c bcmsdbrcm_linux.c bcmsdh_linux.c hnddma.c
endif
ifneq ($(findstring -pci,$(TARGET)-),)
CFILES += bcmpcispi.c
endif
ifneq ($(findstring -sdext-,$(TARGET)-),)
DFLAGS += -DBCMSDIO -DTESTDONGLE
CFILES += dhd_sdio.c
endif

ifneq ($(findstring -n2dbg-,$(TARGET)-),)
DFLAGS += -DWLMEDIA_HTSF -DHOST_TS
endif

ifneq ($(findstring -whdmi-,$(TARGET)-),)
DFLAGS += -DDHD_BCM_WIFI_HDMI -DOEM_ANDROID
CFILES += wl_android.c wldev_common.c
ifneq ($(findstring -cfg80211-,-$(TARGET)-),)
#CFILES += wl_cfg80211.c
#CFILES += wl_cfgp2p.c
#CFILES += wldev_common.c
#CFILES += bcmwifi.c
endif
endif

#ifdef WIFI_ACT_FRAME
DFLAGS += -DWIFI_ACT_FRAME
#endif

ifneq ($(findstring -usb-gpl-,$(TARGET)-),)
#DFLAGS += -DBCM_REQUEST_FW
endif

CFLAGS += -fshort-wchar $(DFLAGS) $(WFLAGS) $(IFLAGS)

LDFLAGS := -r
MODULES := dhd.o
ifeq ($(KERNELVER_2_5_0_OR_LATER), TRUE)
  ##Kernel module names in 2.6 kernel have .ko suffix
  KMODULES:=dhd.ko
  ifneq ($(findstring -nexus-,$(TARGET)-),)
      KMODULES:=bcm4329.ko
  endif

  ifneq ($(findstring -panda-,-$(TARGET)-),)
      KMODULES:=bcmdhd.ko
  endif

  ifneq ($(findstring -prime-,$(TARGET)-),)
      KMODULES:=bcmdhd.ko
  endif  
  
else
  KMODULES:=$(MODULES)
endif

# host options
HOSTCC := $(CC)
ifeq ($(KERNELVER_2_5_0_OR_LATER), FALSE)
  HOSTCFLAGS := $(CFLAGS) $(shell $(MAKE) --no-print-directory -s -C $(LINUXDIR) script 'SCRIPT=@echo $$(CFLAGS) $$(MODFLAGS)')
else
  HOSTCFLAGS := $(CFLAGS) -D__KERNEL__
  DHDCFLAGS = $(HOSTCFLAGS) -I$(shell pwd)
  export DHDCFLAGS
  DHDOFILES = $(OFILES)
  export DHDOFILES
endif

TARGETS := \
	dhd-cdc-usb dhd-cdc-dbususb dhd-cdc-dbususb-dnglimage dhd-cdc-sdio dhd-cdc-dbus dhd-cdc-sdstd \
	dhd-cdc-dbussdstd dhd-cdc-sdspi-pci dhd-cdc-sdmmc-nexus-gpl dhd-cdc-sdmmc-nexus-cfg80211-gpl\
	dhd-cdc-sdmmc-panda-cfg80211-jellybean-gpl \
	dhd-cdc-sdmmc-panda-cfg80211-icsmr1-gpl \
	dhd-cdc-sdmmc-gpl-dk8k \
	dhd-cdc-sdmmc-prime-cfg80211-gpl \
	dhd-cdc-sdmmc-prime-cfg80211-icsmr1-gpl \
	dhd-cdc-sdmmc-prime-cfg80211-jellybean-gpl \
	dhd-cdc-sdmmc-cfg80211-gpl \
	dhd-cdc-sdstd-nothread dhd-cdc-sdmmc-gpl \
	dhd-cdc-usb-gpl dhd-cdc-usb-hsic-gpl dhd-cdc-usb-hsic-padwar-gpl dhd-cdc-panda-usb-hsic-gpl

TARGETS += dhd-cdc-usb-apsta-gpl
TARGETS += dhd-cdc-usb-fdaggr-gpl
TARGETS += dhd-cdc-sdstd-apsta
TARGETS += \
	dhd-cdc-sdio-dnglimage \
	dhd-cdc-sdstd-dnglimage dhd-cdc-dbussdstd-dnglimage \
	dhd-cdc-dbus-dnglimage dhd-cdc-sdspi-pci-dnglimage \
	dhd-cdc-gspi-pci \
	dhd-cdc-gspibcm \
	dhd-cdc-sdstd-hc3 \
	dhd-cdc-sdstd-n2dbg \
	dhd-cdc-sdstd-n2dbg-debug-nothread \
	dhd-cdc-sdstd-debug-nothread \
	dhd-cdc-sdstd-largemtu \
	dhd-cdc-sdstd-debug-largemtu 

#ifdef RNDIS
TARGETS += dhd-rndis-usb
#endif
TARGETS += dhd-cdc-sdext-be
TARGETS += dhd-cdc-sdext-be-dnglimage
TARGETS += dhd-cdc-sdstd-cfg80211-whdmi
TARGETS += $(foreach tgt, $(TARGETS), $(tgt)-debug)

OBJDIR=$(TARGET)-$(LINUXVER)$(if $(BCMQT),-bcmqt)

all: $(filter %-sdio %-sdstd %-usb %sdspi-pci %-sdmmc, $(TARGETS))
sdio: $(filter %-sdio %-sdstd, $(TARGETS))
usb: $(filter %-usb, $(TARGETS))
sdspi: $(filter %-sdspi-pci, $(TARGETS))

# Allow making target with the LINUXVER suffix already on it.
# (Typical of command line tab completion; trailing slash still not allowed)
%-$(LINUXVER): force
	$(MAKE) $(@:%-$(LINUXVER)=%)

$(TARGETS):
	@echo "MAKING $@"
	$(MAKE) TARGET=$@ objdir

# Show compiler version, for the current target build
showenv:
	@echo "CC = $(CC) (ver=`$(CC) -dumpversion`; host=`hostname`; processor=`uname -m`)"

objdir: showenv
	@echo "Making objdir $(OBJDIR)"
	@echo "TARGET is $(TARGET)"
	mkdir -p $(OBJDIR)
ifeq ($(KERNELVER_2_5_0_OR_LATER), FALSE)
	$(MAKE) -C $(OBJDIR) -f $(SRCBASE)/dhd/linux/Makefile SRCBASE=$(SRCBASE) dep
endif
	$(MAKE) -C $(OBJDIR) -f $(SRCBASE)/dhd/linux/Makefile SRCBASE=$(SRCBASE) modules
ifeq ($(KERNELVER_2_5_0_OR_LATER), TRUE)
	$(OBJCOPY) --strip-unneeded $(OBJDIR)/$(KMODULES) $(OBJDIR)/$(KMODULES).stripped
else
	$(OBJCOPY) --strip-unneeded $(OBJDIR)/dhd.o $(OBJDIR)/dhd.o.stripped
endif

dep: $(foreach file,$(CFILES),.$(file).depend)
.%.c.depend: %.c
	$(HOSTCC) $(HOSTCFLAGS) -M $< > $@ || (rm -f $@; exit 1)
.%.c.depend::
	touch $@

ifeq ($(KERNELVER_2_5_0_OR_LATER), TRUE)
modules: $(OFILES)
	test -r ./Makefile || ln -s $(SRCBASE)/dhd/linux/makefile.26 ./Makefile
	$(MAKE) -C $(LINUXDIR) M=$(shell pwd) $(if $(VERBOSE),V=1) modules
else
modules: $(MODULES)
endif

dhd.o: $(OFILES)
	$(LD) $(LDFLAGS) -o $@ $^

ifeq ($(KERNELVER_2_5_0_OR_LATER), TRUE)
%.o: %.c
  # when make is called from 2.6, vpath doesn't work so we need to link the files.
	test -r ./$< || ln -s $< .
else
%.o: %.c
	$(HOSTCC) $(HOSTCFLAGS) -c -o $@ $<
	@( \
	echo 'ifneq ($$(HOSTCFLAGS),$(HOSTCFLAGS))' ; \
	echo '$@: force' ; \
	echo 'endif' ; \
	) > .$*.c.flags
endif

force:

clean:
	rm -rf dhd-*

ifneq ($(wildcard .*.depend),)
include $(wildcard .*.depend)
endif
ifneq ($(wildcard .*.flags),)
include $(wildcard .*.flags)
endif
