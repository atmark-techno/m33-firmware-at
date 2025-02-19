#!/bin/bash
# SPDX-License-Identifier: BSD-3-Clause

# for `make | tee`
set -o pipefail
set -xe

if [ -z "$SdkRootDirPath" ]; then
	echo "\$SdkRootDirPath must be set!" >&2
	exit 1
fi

workdir="$(dirname "$(readlink -f "$0")")"

cd "$workdir" || exit 1

# clear cache unless already configure in same mode with same toolchain
if ! [ -e Makefile ] \
    || ! grep -q CMAKE_BUILD_TYPE:STRING=release CMakeCache.txt \
    || ! grep -q "CMAKE_TOOLCHAIN_FILE:FILEPATH=$SdkRootDirPath/core/tools/cmake_toolchain_files/armgcc.cmake" CMakeCache.txt; then
	rm -rf CMakeFiles Makefile cmake_install.cmake CMakeCache.txt release/m33-firmware-at.bin
	cmake -DCMAKE_TOOLCHAIN_FILE="$SdkRootDirPath/core/tools/cmake_toolchain_files/armgcc.cmake" \
		-G "Unix Makefiles" -DCMAKE_BUILD_TYPE=release
fi

make -j 2>&1 | tee build_log.txt

cp release/m33-firmware-at.bin ../
