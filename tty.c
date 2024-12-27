// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Atmark Techno, Inc. All Rights Reserved.
 *
 * Based on: linux-5.10/drivers/tty/tty_baudrate.c
 *  Copyright (C) 1991, 1992, 1993, 1994  Linus Torvalds
 */

#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "termbits.h"
#include "tty.h"

/*
 * Routine which returns the baud rate of the tty
 *
 * Note that the baud_table needs to be kept in sync with the
 * termbits.h file.
 */
static const speed_t baud_table[] = { 0,       50,      75,      110,     134,     150,     200,    300,
                                      600,     1200,    1800,    2400,    4800,    9600,    19200,  38400,
                                      57600,   115200,  230400,  460800,  500000,  576000,  921600, 1000000,
                                      1152000, 1500000, 2000000, 2500000, 3000000, 3500000, 4000000 };

static const tcflag_t __unused baud_bits[] = { B0,       B50,      B75,      B110,     B134,     B150,     B200,
                                               B300,     B600,     B1200,    B1800,    B2400,    B4800,    B9600,
                                               B19200,   B38400,   B57600,   B115200,  B230400,  B460800,  B500000,
                                               B576000,  B921600,  B1000000, B1152000, B1500000, B2000000, B2500000,
                                               B3000000, B3500000, B4000000 };

static int n_baud_table = ARRAY_SIZE(baud_table);

speed_t tty_baudrate(tcflag_t cflag)
{
    unsigned int cbaud;

    cbaud = cflag & CBAUD;

    if (cbaud & CBAUDEX)
    {
        cbaud &= ~CBAUDEX;

        if (cbaud + 15 < n_baud_table)
            cbaud += 15;
    }

    return cbaud >= n_baud_table ? 0 : baud_table[cbaud];
}

lpuart_parity_mode_t tty_parity(tcflag_t cflag)
{
    if (cflag & PARENB)
    {
        if (cflag & PARODD)
            return kLPUART_ParityOdd;
        return kLPUART_ParityEven;
    }

    return kLPUART_ParityDisabled;
}

bool tty_cmsparity(tcflag_t cflag)
{
    if (cflag & CMSPAR)
        return true;

    return false;
}

lpuart_data_bits_t tty_databits(tcflag_t cflag)
{
    switch (cflag & CSIZE)
    {
        case CS5:
            PRINTF("%s: Unsupported 5-bit data characters. Use 8-bit.\r\n", __func__);
            break;
        case CS6:
            PRINTF("%s: Unsupported 6-bit data characters. Use 8-bit.\r\n", __func__);
            break;
        case CS7:
            return kLPUART_SevenDataBits;
        case CS8:
            break;
        default:
            assert(false);
    }

    return kLPUART_EightDataBits;
}

lpuart_stop_bit_count_t tty_stopbits(tcflag_t cflag)
{
    if (cflag & CSTOPB)
        return kLPUART_TwoStopBit;

    return kLPUART_OneStopBit;
}

bool tty_rtscts(tcflag_t cflag)
{
    if (cflag & CRTSCTS)
        return true;

    return false;
}
