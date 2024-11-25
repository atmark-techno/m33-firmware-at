#!/bin/bash

# for `make | tee`
set -o pipefail
set -xe

workdir="$(dirname $(readlink -f $0))"

cd "$workdir" || exit 1
if [ -d "CMakeFiles" ];then rm -rf CMakeFiles; fi
if [ -f "Makefile" ];then rm -f Makefile; fi
if [ -f "cmake_install.cmake" ];then rm -f cmake_install.cmake; fi
if [ -f "CMakeCache.txt" ];then rm -f CMakeCache.txt; fi
cmake -DCMAKE_TOOLCHAIN_FILE="${SdkRootDirPath}/core/tools/cmake_toolchain_files/armgcc.cmake" -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=debug

[ -e debug/sdk20-app.bin ] && mv debug/sdk20-app.bin debug/sdk20-app.bin.old
make -j 2>&1 | tee build_log.txt
if [ -e debug/sdk20-app.bin ]; then cp debug/sdk20-app.bin ../m33-firmware-at.bin; rm -f debug/sdk20-app.bin.old;
else mv debug/sdk20-app.bin.old debug/sdk20-app.bin; fi
