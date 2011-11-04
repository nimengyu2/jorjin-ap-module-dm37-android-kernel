#!/bin/sh
export ANDROID_PATH=`pwd`
export PATH=$ANDROID_PATH/../prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin:$PATH
export CROSS_COMPILE=arm-eabi-                                                                                               
export ARCH=arm
export TARGET_PRODUCT=beagleboard 
export PATH=$ANDROID_PATH/../u-boot/tools:$PATH
#make clean && \
make panther_android_defconfig && \
make uImage -j1 2>&1 |tee kernel_make.out
#make modules -j4 2>&1 |tee kernel_module.out
