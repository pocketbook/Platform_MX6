#!/bin/bash
set -x 
./make.sh mrproper

./make.sh 631_defconfig

./make.sh uImage && ./make.sh modules

echo "Kernel building has been finished!"
