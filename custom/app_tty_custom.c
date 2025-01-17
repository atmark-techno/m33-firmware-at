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

static int custom_tx(struct tty_settings *settings, uint8_t *buf, uint16_t len)
{
    struct custom_tty_settings *custom = get_custom(settings);

    /* Handle input from linux here.
     * The default implementation just echoes back
     */
    uint8_t *back_buf;
    uint16_t maxlen;
    srtm_notification_t notif = SRTM_TtyService_NotifyAlloc(settings->port_idx, &back_buf, &maxlen);

    /* truncate to whatever srtm sent us */
    len = MIN(maxlen, len);
    memcpy(back_buf, buf, len);
    SRTM_TtyService_NotifySend(ttyService, notif, len);

    /* (unused variable warning workaround) */
    (void)custom;

    return 0;
}

static int custom_init(struct tty_settings *settings, struct srtm_tty_init_payload *generic_init)
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
    .tx            = custom_tx,
    .init          = custom_init,
    .settings_size = sizeof(struct custom_tty_settings),
};
