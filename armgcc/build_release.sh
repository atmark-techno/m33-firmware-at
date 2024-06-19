#!/bin/sh
set -x
workdir="$(dirname $(readlink -f $0))"
patch_dir="$workdir/patches"
cd /mcux-sdk/core
for pat in "$patch_dir"/*; do patch -p 1 -N -i "$pat" || exit; done

cd "$workdir" || exit 1
if [ -d "CMakeFiles" ];then rm -rf CMakeFiles; fi
if [ -f "Makefile" ];then rm -f Makefile; fi
if [ -f "cmake_install.cmake" ];then rm -f cmake_install.cmake; fi
if [ -f "CMakeCache.txt" ];then rm -f CMakeCache.txt; fi
cmake -DCMAKE_TOOLCHAIN_FILE="${SdkRootDirPath}/core/tools/cmake_toolchain_files/armgcc.cmake" -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=release  .
# "cr_newlib_nohost" does not exist and is not needed.
sed -i CMakeFiles/power_mode_switch.elf.dir/link.txt -e 's/-lcr_newlib_nohost //g'

[ -e release/sdk20-app.bin ] && mv release/sdk20-app.bin release/sdk20-app.bin.old
make -j 2>&1 | tee build_log.txt
if [ -e release/sdk20-app.bin ]; then cp release/sdk20-app.bin ../m33-firmware-at.bin; rm -f release/sdk20-app.bin.old;
else mv release/sdk20-app.bin.old release/sdk20-app.bin; fi
