/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

/* uboot protocol as per arch/arm/include/asm/arch-imx8ulp/sys_proto.h in uboot sources */
/* protocol for uboot <-> m33 */
/* handshake
 * uboot -> m33: handshake command
 * m33 -> uboot: handshake ACK
 */
#define UBOOT_HANDSHAKE 0

/* boot into linux
 * uboot -> m33: boot command
 * (no reply)
 */
#define UBOOT_BOOT 1

/* cold reset
 * uboot -> m33: reset command
 * (no reply)
 */
#define UBOOT_RESET 2
