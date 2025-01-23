#!/bin/bash

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
    || ! grep -q CMAKE_BUILD_TYPE:STRING=debug CMakeCache.txt \
    || ! grep -q "CMAKE_TOOLCHAIN_FILE:FILEPATH=$SdkRootDirPath/core/tools/cmake_toolchain_files/armgcc.cmake" CMakeCache.txt; then
	rm -rf CMakeFiles Makefile cmake_install.cmake CMakeCache.txt debug/sdk20-app.bin
	cmake -DCMAKE_TOOLCHAIN_FILE="$SdkRootDirPath/core/tools/cmake_toolchain_files/armgcc.cmake" \
		-G "Unix Makefiles" -DCMAKE_BUILD_TYPE=debug
fi

make -j 2>&1 | tee build_log.txt

cp debug/sdk20-app.bin ../m33-firmware-at.bin
