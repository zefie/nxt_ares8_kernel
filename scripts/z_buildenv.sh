#!/bin/bash
export TOOLCHAIN="/home/zefie/dev/ares8/cm12.1/prebuilts/gcc/linux-x86/x86/x86_64-linux-android-4.9/bin/x86_64-linux-android-"
export ARCH=x86_64
export CROSS_COMPILE="${TOOLCHAIN}"
if [ -z "#NOD" ]; then
	export DISTCC=1
fi
$*
