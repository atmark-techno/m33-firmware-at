#!/bin/sh
# SPDX-License-Identifier: BSD-3-Clause

workdir="$(dirname "$(readlink -f "$0")")"

rm -rf "$workdir"/debug "$workdir"/release "$workdir"/CMakeFiles
rm -rf "$workdir"/Makefile "$workdir"/cmake_install.cmake "$workdir"/CMakeCache.txt
rm -rf "$workdir"/../m33-firmware-at.bin
