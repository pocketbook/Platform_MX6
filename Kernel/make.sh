#!/bin/sh

set -x

MODULES_DIR=/media/mike/rootfs

# enter scripts working directory
cd "$(cd "$(dirname "$0")" && pwd)"

KERNEL_PRJ=$(basename `pwd` | grep -oh '[0-9]\{3\}')

if [ "$USER" = "jenkins" ] ;
then
	CROSS_PREFIX="/usr/local/gcc-4.4.4-glibc-2.11.1-multilib-1.0/arm-fsl-linux-gnueabi/bin/arm-fsl-linux-gnueabi-"
else
#	CROSS_PREFIX="/opt/freescale/usr/local/gcc-4.4.4-glibc-2.11.1-multilib-1.0/arm-fsl-linux-gnueabi/bin/arm-fsl-linux-gnueabi-"
	CROSS_PREFIX="/opt/freescale/usr/local/gcc-4.6.2-glibc-2.13-linaro-multilib-2011.12/fsl-linaro-toolchain/bin/arm-fsl-linux-gnueabi-"
fi

dpkg -s ccache &> /dev/null
[ $? -eq 0 ] && CROSS_PREFIX="ccache ${CROSS_PREFIX}"

NCPUS=`cat /proc/cpuinfo | grep processor | wc -l`

make INSTALL_HDR_PATH=$HEADERS_DIR INSTALL_MOD_PATH=$MODULES_DIR CROSS_COMPILE="$CROSS_PREFIX" ARCH=arm -j $NCPUS $1

