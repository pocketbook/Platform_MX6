#!/bin/bash

echo "set xrzebook uboot build environment"
export ARCH=arm

export CROSS_COMPILE=/opt/freescale/usr/local/gcc-4.6.2-glibc-2.13-linaro-multilib-2011.12/fsl-linaro-toolchain/bin/arm-fsl-linux-gnueabi-

echo "configure uboot parameter"
make mx6sl_evk_config

echo "build start"
make

echo "exit build"
exit 0
