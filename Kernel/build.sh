#!/bin/sh

MODEL=$(basename `pwd` | grep -oh '[0-9]\{3\}')

./make.sh mrproper && \
./make.sh ${MODEL}_defconfig && \
./make.sh uImage && \
./make.sh modules
