#!/bin/bash
export TOOLCHAIN="/home/zefie/dev/toolchains/x86_64-linux-android-4.9/bin/x86_64-linux-androidkernel-"
export MODULES_DESTDIR="/home/zefie/dev/ares8/bootimg/boot_stock/initrd/lib/modules"
export ARCH=x86_64
export KERNEL_DEFCONFIG=ares8_android_defconfig
export CROSS_COMPILE="${TOOLCHAIN}"
