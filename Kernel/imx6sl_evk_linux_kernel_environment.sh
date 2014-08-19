#!/bin/bash
echo "set imx6sl evk linux kernel build environment"

export ARCH=arm

export CROSS_COMPILE=/opt/freescale/usr/local/gcc-4.6.2-glibc-2.13-linaro-multilib-2011.12/fsl-linaro-toolchain/bin/arm-fsl-linux-gnueabi-
export PATH=/mnt/vdisk/zhuwenzhi/freescale/imx6slEvk/linux/L3.0.35_ga/uboot-imx/tools:$PATH
