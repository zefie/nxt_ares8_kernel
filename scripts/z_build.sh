#!/bin/bash
CPUS=$(grep -c ^processor /proc/cpuinfo)
scripts/z_make.sh -j${CPUS}
