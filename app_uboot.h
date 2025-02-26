/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

#include "fsl_mu.h"

/* helpers for m33 implementation */
static inline uint32_t uboot_recv(void)
{
    /* open code MU_ReceiveMsg(MU0_MUA, 0) but yield to allow CLI to work in uboot */
    while (0U == (MU0_MUA->RSR & 1))
    {
        vTaskDelay(1);
    }
    return MU0_MUA->RR[0];
}

static inline void uboot_recv_many(void *buf, int len)
{
    int i;
    uint32_t *word_buf = buf;

    for (i = 0; i < len / 4; i++)
    {
        word_buf[i] = uboot_recv();
    }

    if (len % 4)
    {
        uint32_t last  = uboot_recv();
        char *byte_buf = (char *)(&word_buf[len / 4]);
        for (i = 0; i < len % 4; i++)
        {
            byte_buf[i] = (last >> (i * 8)) & 0xff;
        }
    }
}

static inline void uboot_send(uint32_t val)
{
    MU_SendMsg(MU0_MUA, 0, val);
}

static inline void uboot_send_many(void *buf, int len)
{
    int i;
    uint32_t *word_buf = buf;

    for (i = 0; i < len / 4; i++)
    {
        uboot_send(word_buf[i]);
    }

    if (len % 4)
    {
        uint32_t last  = 0;
        char *byte_buf = (char *)(&word_buf[len / 4]);
        for (i = 0; i < len % 4; i++)
        {
            last |= byte_buf[i] << (i * 8);
        }
        uboot_send(last);
    }
}

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

/* pinctrl
 * uboot -> m33: pinctrl command, pinctrl values (u32 x6)
 * m33 -> uboot: ack
 * (Unlike linux's this is not registered anywhere on m33 side)
 */
#define UBOOT_PINCTRL 3

/* I2C operations.
 * uboot -> m33: (i2c | (subcommand <<8) | (bus_id << 16))
 * rest depends on subcommand as follow. Anything else gets error back.
 */
#define UBOOT_I2C 4
/* i2c init subcommand:
 * uboot -> m33: i2c type, index, baudrate
 * (type must be lpi2c, if we support lpuart later that one needs to send scl/sda too)
 * m33 -> uboot: status
 */
#define UBOOT_I2C_INIT 0
/* i2c reset subcommand:
 * uboot -> m33: nothing else
 * m33 -> uboot: status
 */
#define UBOOT_I2C_RESET 1
/* i2c read subcommand:
 * uboot -> m33: addr, len (in byte)
 * m33 -> uboot: status, actual read len (in byte), followed by data one word at a time
 */
#define UBOOT_I2C_READ 2
/* i2c write subcommand:
 * uboot -> m33: addr, len (in byte), data one word at a time
 * m33 -> uboot: status
 */
#define UBOOT_I2C_WRITE 3
