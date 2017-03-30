#!/bin/bash
source scripts/z_buildenv.sh
mkdir -p build
make O=build $*
