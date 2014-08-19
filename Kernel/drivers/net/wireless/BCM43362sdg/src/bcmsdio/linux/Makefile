
# GNU Makefile for Broadcom BCMSDH Lower-level Driver
#
# Copyright (C) 2011, Broadcom Corporation
# All Rights Reserved.
# 
# THIS SOFTWARE IS OFFERED "AS IS", AND BROADCOM GRANTS NO WARRANTIES OF ANY
# KIND, EXPRESS OR IMPLIED, BY STATUTE, COMMUNICATION OR OTHERWISE. BROADCOM
# SPECIFICALLY DISCLAIMS ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A SPECIFIC PURPOSE OR NONINFRINGEMENT CONCERNING THIS SOFTWARE.
#
# $Id: Makefile 315339 2012-02-16 07:07:41Z $
#

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

LINUXVER := $(shell { cat $(LINUXDIR)/Makefile; \
	echo "bcm$$$$:;@echo \$$(KERNELRELEASE)"; } | \
	$(MAKE) --no-print-directory $(if $(ARCH),ARCH=$(ARCH),) -C $(LINUXDIR) -f - bcm$$$$)

# check if 2.4 kernel or 2.5+ kernel
BCM_KVER:=$(shell echo $(LINUXVER) | cut -c1-3 | sed 's/2\.[56]/2\.6/')

# Allow CROSS_COMPILE to specify compiler base
CC := $(CROSS_COMPILE)gcc
LD := $(CROSS_COMPILE)ld
NM := $(CROSS_COMPILE)nm
OBJCOPY := $(CROSS_COMPILE)objcopy

# driver source base and C file path
ifeq ($(SRCBASE),)
SRCBASE := $(shell /bin/pwd)/../..
endif

vpath %.c $(SRCBASE)/wl/sys $(SRCBASE)/bcmsdio/sys $(WLAN_ComponentSrcDirs)

## Initialize DFLAGS
DFLAGS :=


# basic options (defines in DFLAGS, includes in IFLAGS)
DFLAGS += -DLINUX -DSRCBASE=\"$(SRCBASE)\" -DBCMDRIVER -DBCMSDH_MODULE
DFLAGS += -DBCMDONGLEHOST
DFLAGS += -DBCM4325

IFLAGS := -I$(LINUXDIR)/include -I$(LINUXDIR)/include/asm/mach-default -I. -I$(SRCBASE)/include -I$(SRCBASE)/shared -I$(SRCBASE)/dongle -I$(SRCBASE)/wl/sys

IFLAGS += $(WLAN_ComponentIncPath)

WFLAGS := -Wall -Wstrict-prototypes
ifeq (,$(findstring 2.4.18,$(LINUXVER)))
WFLAGS += -Werror
endif

CFILES:= bcmsdh_linux.c linux_osl.c bcmsdh.c
CFILES += siutils.c sbutils.c aiutils.c bcmutils.c hndpmu.c

OFILES=$(CFILES:.c=.o)

# Make debug a separate option
ifneq ($(findstring -debug-,-$(TARGET)-),)
DFLAGS += -DBCMDBG -DSDTEST
endif

# Make big-endian a separate option
ifneq ($(findstring -be-,-$(TARGET)-),)
DFLAGS += -DIL_BIGENDIAN
endif

ifneq ($(findstring -sdstd,$(TARGET)-),)
DFLAGS += -DBCMSDIO -DBCMSDIOH_STD
CFILES += bcmsdstd.c bcmsdstd_linux.c
endif
ifneq ($(findstring -sdspi-,$(TARGET)-),)
DFLAGS += -DBCMSDIO -DBCMSDIOH_SPI -DTESTDONGLE
CFILES += bcmsdspi.c bcmsdspi_linux.c 
endif

CFLAGS += -fshort-wchar $(DFLAGS) $(WFLAGS) $(IFLAGS)

LDFLAGS := -r
MODULES := bcmsdh_driver.o
ifeq ($(BCM_KVER), 2.6)
  ##Kernel module names in 2.6 kernel have .ko suffix
  KMODULES:=bcmsdh_driver.ko
else
  KMODULES:=$(MODULES)
endif

# host options
HOSTCC := $(CC)
ifneq ($(BCM_KVER), 2.6)
  HOSTCFLAGS := $(CFLAGS) $(shell $(MAKE) --no-print-directory -s -C $(LINUXDIR) script 'SCRIPT=@echo $$(CFLAGS) $$(MODFLAGS)')
else
  HOSTCFLAGS := $(CFLAGS) -D__KERNEL__
  BCMSDHCFLAGS = $(HOSTCFLAGS) -I$(shell pwd)
  export BCMSDHCFLAGS
  BCMSDHOFILES = $(OFILES)
  export BCMSDHOFILES
endif

TARGETS := \
	bcmsdh-sdstd
TARGETS += $(foreach tgt, $(TARGETS), $(tgt)-debug)

OBJDIR=$(TARGET)-$(LINUXVER)$(if $(BCMQT),-bcmqt)

all: $(TARGETS)

# Allow making target with the LINUXVER suffix already on it.
# (Typical of command line tab completion; trailing slash still not allowed)
%-$(LINUXVER): force
	$(MAKE) $(@:%-$(LINUXVER)=%)

$(TARGETS):
	@echo "MAKING $@"
	$(MAKE) TARGET=$@ objdir

objdir:
	@echo "Making objdir $(OBJDIR)"
	@echo "TARGET is $(TARGET)"
	mkdir -p $(OBJDIR)
	$(MAKE) -C $(OBJDIR) -f ../Makefile SRCBASE=$(SRCBASE) dep
	$(MAKE) -C $(OBJDIR) -f ../Makefile SRCBASE=$(SRCBASE) modules
ifeq ($(BCM_KVER), 2.6)
	$(OBJCOPY) --strip-unneeded $(OBJDIR)/bcmsdh_driver.ko $(OBJDIR)/bcmsdh_driver.ko.stripped
else
	$(OBJCOPY) --strip-unneeded $(OBJDIR)/bcmsdh_driver.o $(OBJDIR)/bcmsdh_driver.o.stripped
endif

dep: $(foreach file,$(CFILES),.$(file).depend)
.%.c.depend: %.c
	$(HOSTCC) $(HOSTCFLAGS) -M $< > $@
.%.c.depend::
	touch $@

ifeq ($(BCM_KVER), 2.6)
modules: $(OFILES)
	test -r ./Makefile || ln -s ../makefile.26 ./Makefile
	$(MAKE) -C $(LINUXDIR) M=$(shell pwd) $(if $(VERBOSE),V=1) modules
else
modules: $(MODULES)
endif

bcmsdh_driver.o: $(OFILES)
	$(LD) $(LDFLAGS) -o $@ $^

ifeq ($(BCM_KVER), 2.6)
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
	rm -rf bcmsdh-*

ifneq ($(wildcard .*.depend),)
include $(wildcard .*.depend)
endif
ifneq ($(wildcard .*.flags),)
include $(wildcard .*.flags)
endif
