// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Atmark Techno, Inc. All Rights Reserved.
 */
#ifndef _TTY_H_
#define _TTY_H_

#include "fsl_lpuart.h"

speed_t tty_baudrate(tcflag_t cflag);
lpuart_parity_mode_t tty_parity(tcflag_t cflag);
bool tty_cmsparity(tcflag_t cflag);
lpuart_data_bits_t tty_databits(tcflag_t cflag);
lpuart_stop_bit_count_t tty_stopbits(tcflag_t cflag);
bool tty_rtscts(tcflag_t cflag);

#endif
