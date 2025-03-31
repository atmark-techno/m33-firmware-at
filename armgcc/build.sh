#!/bin/bash
# SPDX-License-Identifier: BSD-3-Clause

# for `make | tee`
set -o pipefail
set -xe

workdir="$(dirname "$(readlink -f "$0")")"

cd "$workdir" || exit 1

if [ -z "$SdkRootDirPath" ]; then
	echo "\$SdkRootDirPath must be set!" >&2
	exit 1
fi

if ! [ -e "$ARMGCC_DIR/bin/arm-none-eabi-gcc" ]; then
	# no toolchain, use existing binary if present or error out
	if [ -e "../m33-firmware-at.bin" ]; then
		echo "Not rebuilding m33-firmware-at.bin without arm-none-eabi-gcc"
		exit
	fi
	echo "Install gcc-arm-none-eabi to build m33-firmware-at.bin" >&2
	exit 1
fi

case "$1" in
release|debug)
	target="$1"
	;;
"")
	if [ -e CMakeCache.txt ]; then
		target=$(sed -ne 's/CMAKE_BUILD_TYPE:STRING=//p' CMakeCache.txt)
	fi
	[ -n "$target" ] || target=release
	;;
*)
	echo "Usage: $0 [release|debug]" >&2
	exit 1
	;;
esac

# clear cache unless already configure in same mode with same toolchain
if ! [ -e Makefile ] \
    || ! grep -q "CMAKE_BUILD_TYPE:STRING=$target" CMakeCache.txt \
    || ! grep -q "CMAKE_TOOLCHAIN_FILE:FILEPATH=$SdkRootDirPath/core/tools/cmake_toolchain_files/armgcc.cmake" CMakeCache.txt; then
	rm -rf CMakeFiles Makefile cmake_install.cmake CMakeCache.txt "$target/m33-firmware-at.bin"
	cmake -DCMAKE_TOOLCHAIN_FILE="$SdkRootDirPath/core/tools/cmake_toolchain_files/armgcc.cmake" \
		-G "Unix Makefiles" -DCMAKE_BUILD_TYPE="$target"
fi

make -j 2>&1 | tee build_log.txt

cp "$target/m33-firmware-at.bin" ../
