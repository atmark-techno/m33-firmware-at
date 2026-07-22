/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "app_tty.h"
#include "build_bug.h"

/* custom settings private data */
struct custom_tty_settings
{
};

struct custom_tty_settings *get_custom(struct tty_settings *settings)
{
    BUILD_BUG_ON((uintptr_t)(((struct tty_settings *)0) + 1) % _Alignof(struct custom_tty_settings) != 0);
    return (struct custom_tty_settings *)(settings + 1);
}

/* This helper sends data to linux.
 * It's also possible to get a buffer early and write directly into it to
 * avoid a copy */
static void custom_tty_to_linux(struct tty_settings *settings, char *buf, uint16_t len)
{
    /* We should not send data to linux when the tty is not active to avoid filling buffer on linux side */
    if ((settings->state & (TTY_ACTIVE | TTY_SUSPENDED)) != TTY_ACTIVE)
        return;

    while (len > 0)
    {
        uint8_t *send_buf;
        uint16_t send_len = len;

        /* get buffer from SRTM */
        srtm_notification_t notif = SRTM_TtyService_NotifyAlloc_Sized(settings->port_idx, &send_buf, &send_len);
        assert(notif);

        /* copy data and send */
        memcpy(send_buf, buf, send_len);
        SRTM_TtyService_NotifySend(ttyService, notif, send_len);

        len -= send_len;
        buf += send_len;
    }
}

/* tx is from point of view of linux, this function is called when
 * data is sent from linux */
static int custom_tty_tx(struct tty_settings *settings, uint8_t *buf, uint16_t len)
{
    struct custom_tty_settings *custom = get_custom(settings);

    /* Handle input from linux here.
     * The default implementation just echoes back
     */
    custom_tty_to_linux(settings, (char *)buf, len);

    /* (unused variable warning workaround) */
    (void)custom;

    return 0;
}

static int custom_tty_activate(struct tty_settings *settings)
{
    /* This function is called when linux first opens or last close the tty */
    PRINTF("custom tty is %s\r\n", (settings->state & TTY_ACTIVE) ? "open" : "closed");

    return 0;
}

static int custom_tty_init(struct tty_settings *settings, struct srtm_tty_init_payload *generic_init)
{
    struct custom_tty_settings *custom        = get_custom(settings);
    struct srtm_tty_init_custom_payload *init = &generic_init->custom;

    PRINTF("initializing tty %d as CUSTOM '%s'\r\n", settings->port_idx, init->name);

    /* Init more things here... */
    (void)custom;

    return 0;
}

/* manually added to tty_hooks top of app_tty.c */
const struct tty_hooks tty_custom_hooks = {
    /* unset hooks are skipped */
    .tx            = custom_tty_tx,
    .activate      = custom_tty_activate,
    .init          = custom_tty_init,
    .settings_size = sizeof(struct custom_tty_settings),
};
