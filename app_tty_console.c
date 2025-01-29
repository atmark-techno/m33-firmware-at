/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "app_tty.h"
#include "debug_console.h"
#include "build_bug.h"

#define RX_BUF_LEN 128
/* use global variables so debug console can access these */
static char rx_buf[RX_BUF_LEN];
static uint16_t rx_start;
static uint16_t rx_end;
static uint8_t port_idx = 255;

/* tx from linux = send to m33 console.
 * We can't interrupt it so keep in rx_buf until we're polled */
static int console_tx(struct tty_settings *settings, uint8_t *buf, uint16_t len)
{
    uint16_t num;

    /* Safety:
     * - only the dispatcher task (through this function) writes to rx_end, rx_buf
     * - but rx_start can be modified under us, so remember/use its initial value */
    uint16_t start = rx_start;

    if (rx_end >= start)
    {
        num = MIN(RX_BUF_LEN - rx_end, len);
        memcpy(rx_buf + rx_end, buf, num);
        rx_end += num;
        buf += num;
        len -= num;
        if (rx_end == RX_BUF_LEN)
            rx_end = 0;
    }

    if (!len)
        return 0;

    /* Either we started here, or everything was copied, or we now are there.
     * Equal is possible when both = 0. */
    assert(rx_end <= start);

    num = MIN(start - rx_end, len);
    memcpy(rx_buf + rx_end, buf, num);
    rx_end += num;
    buf += num;
    len -= num;

    if (len)
    {
        PRINTF("Lost input: %*s\r\n", len, buf);
    }

    return 0;
}

static int console_init(struct tty_settings *settings, struct srtm_tty_init_payload *generic_init)
{
    port_idx = settings->port_idx;

    PRINTF("initialized tty %d for M33 console\r\n", settings->port_idx);

    return 0;
}

/* manually added to tty_hooks top of app_tty.c */
const struct tty_hooks tty_console_hooks = {
    /* unset hooks are skipped */
    .tx            = console_tx,
    .init          = console_init,
    .settings_size = 0,
};

int APP_TTY_Console_Getchar(uint8_t *ch)
{
    if (rx_end == rx_start)
    {
        return -1;
    }
    *ch = rx_buf[rx_start];
    rx_start++;
    if (rx_start == RX_BUF_LEN)
        rx_start = 0;
    return 0;
}

void APP_TTY_Console_Write(uint8_t *buf, uint16_t len)
{
    /* not init */
    if (port_idx == 255)
        return;

    while (len > 0)
    {
        uint8_t *back_buf;
        uint16_t maxlen;
        srtm_notification_t notif = SRTM_TtyService_NotifyAlloc(port_idx, &back_buf, &maxlen);

        /* truncate to whatever srtm sent us */
        uint16_t num = MIN(maxlen, len);
        memcpy(back_buf, buf, num);
        SRTM_TtyService_NotifySend(ttyService, notif, num);

        len -= num;
        buf += num;
    }
}
