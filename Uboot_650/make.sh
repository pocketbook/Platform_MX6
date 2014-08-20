#!/bin/sh

set -x

# enter scripts working directory
cd "$(cd "$(dirname "$0")" && pwd)"

if [ "$USER" = "jenkins" ] ;
then
	CROSS_PREFIX="/usr/local/gcc-4.4.4-glibc-2.11.1-multilib-1.0/arm-fsl-linux-gnueabi/bin/arm-fsl-linux-gnueabi-"
else
#	CROSS_PREFIX="/opt/freescale/usr/local/gcc-4.4.4-glibc-2.11.1-multilib-1.0/arm-fsl-linux-gnueabi/bin/arm-fsl-linux-gnueabi-"
#	CROSS_PREFIX="/opt/freescale/usr/local/gcc-4.6.2-glibc-2.13-linaro-multilib-2011.12/fsl-linaro-toolchain/bin/arm-fsl-linux-gnueabi-"
	CROSS_PREFIX="/opt/freescale/usr/local/gcc-4.4.4-glibc-2.11.1-multilib-1.0/arm-fsl-linux-gnueabi/bin/arm-fsl-linux-gnueabi-"
fi

dpkg -s ccache > /dev/null 2>&1
if [ $? -eq 0 ] ;
then
	# package installed
	CROSS_PREFIX="ccache ${CROSS_PREFIX}"
fi

NCPUS=`cat /proc/cpuinfo | grep processor | wc -l`

#make HOSTCC="${CROSS_PREFIX}" ARCH=arm CROSS_COMPILE="${CROSS_PREFIX}" -j ${NCPUS} $1
make ARCH=arm CROSS_COMPILE="${CROSS_PREFIX}"  $1

