#!/bin/bash
rm -rf build/
source scripts/z_buildenv.sh
scripts/z_make.sh ${KERNEL_DEFCONFIG}
scripts/z_build.sh

