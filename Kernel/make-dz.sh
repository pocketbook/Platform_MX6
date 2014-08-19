#!/bin/sh

if [ "$1" != "defconfig" -a "$1" != "menuconfig" -a "$1" != "uImage" -a "$1" != "modules" -a "$1" != "clean" ] ; then
	echo "usage: make.sh {defconfig|menuconfig|uImage|modules|clean}"
	exit 1
fi

export PATH=/opt/freescale/usr/local/gcc-4.4.4-glibc-2.11.1-multilib-1.0/arm-fsl-linux-gnueabi/bin:$PATH
#export PATH=/opt/freescale/usr/local/gcc-4.1.2-glibc-2.5-nptl-3/arm-none-linux-gnueabi/bin:$PATH
#export PATH=/opt/arm-2008q3/bin:$PATH

[ $1 = "uImage" ] && rm -f uImage vmlinux
[ $1 = "modules" ] && rm -rf modules_build
[ $1 = "modules" ] && find . -name "*.ko" -exec rm -f \{} \;

make ARCH=arm CROSS_COMPILE=arm-fsl-linux-gnueabi- "$@" -j8

[ $1 = "uImage" ] && cp arch/arm/boot/uImage .

[ $1 = "modules" ] && mkdir -p modules_build
[ $1 = "modules" ] && find . -name "*.ko" -exec cp -f \{} modules_build/ \;

