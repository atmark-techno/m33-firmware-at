/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "printf.h"
#include "app_srtm_internal.h"
#include "app_tty.h"

/* global settings */
#define TTY_MAX_PORTS 4
static struct tty_settings *tty_settings[TTY_MAX_PORTS];
srtm_service_t ttyService;

/* Hooks for each type.
 * We'd ideally use a linker-generated array like linux/u-boot for this,
 * but this is overkill so just list all valid types manually. */
extern const struct tty_hooks tty_lpuart_hooks;
extern const struct tty_hooks tty_custom_hooks;
extern const struct tty_hooks tty_console_hooks;
extern const struct tty_hooks tty_flexio_hooks;
static struct tty_hooks const *tty_hooks[_TTY_TYPE_COUNT] = {
    [TTY_TYPE_LPUART]      = &tty_lpuart_hooks,
    [TTY_TYPE_CUSTOM]      = &tty_custom_hooks,
    [TTY_TYPE_M33_CONSOLE] = &tty_console_hooks,
    [TTY_TYPE_FLEXIO]      = &tty_flexio_hooks,
};

/* get settings or NULL, log error if caller name given */
static struct tty_settings *get_settings(uint8_t port_idx, const char *caller)
{
    if (port_idx >= TTY_MAX_PORTS)
    {
        if (caller)
            PRINTF("tty %d %s: port_idx %d too big\r\n", port_idx, caller, port_idx);
        return NULL;
    }

    if (!tty_settings[port_idx])
    {
        if (caller)
            PRINTF("tty %d %s without init?\r\n", port_idx, caller);
        return NULL;
    }
    return tty_settings[port_idx];
}

/**********************************************************
 * init/PM hooks called from APP_SRTM
 * SRTM callbacks
 * (just redirect to appropriate app_tty_* implem)
 *********************************************************/

static int APP_TTY_tx(uint8_t port_idx, uint8_t *buf, uint16_t len)
{
    struct tty_settings *settings = get_settings(port_idx, "tx");

    if (!settings)
        return kStatus_Fail;

    if (!tty_hooks[settings->type]->tx)
        return kStatus_Fail;

    return tty_hooks[settings->type]->tx(settings, buf, len);
}

static int APP_TTY_setcflag(uint8_t port_idx, tcflag_t cflag)
{
    struct tty_settings *settings = get_settings(port_idx, "setcflag");

    if (!settings)
        return kStatus_Fail;

    if (!tty_hooks[settings->type]->setcflag)
        return kStatus_Success;

    return tty_hooks[settings->type]->setcflag(settings, cflag);
}

static int APP_TTY_setwake(uint8_t port_idx, bool enable)
{
    struct tty_settings *settings = get_settings(port_idx, "setwake");

    if (!settings)
        return kStatus_Fail;

    if (!tty_hooks[settings->type]->setwake)
        return kStatus_Success;

    return tty_hooks[settings->type]->setwake(settings, enable);
}

static int APP_TTY_init(uint8_t port_idx, struct srtm_tty_init_payload *init)
{
    struct tty_settings *settings;
    const struct tty_hooks *hooks;
    int rc = 0;

    if (port_idx >= TTY_MAX_PORTS)
    {
        PRINTF("tty %d %s: port_idx %d too big\r\n", port_idx, "init", port_idx);
        return kStatus_Fail;
    }

    if (tty_settings[port_idx])
    {
        PRINTF("tty port %d was already init!\r\n", port_idx);
        return kStatus_Fail;
    }

    if (init->port_type > _TTY_TYPE_COUNT || !tty_hooks[init->port_type])
    {
        PRINTF("tty port %d type %" PRIu32 " either type too high or not defined\r\n", port_idx, init->port_type);
        return kStatus_Fail;
    }

    hooks    = tty_hooks[init->port_type];
    settings = pvPortMalloc(sizeof(*settings) + hooks->settings_size);
    if (!settings)
    {
        return kStatus_Fail;
    }

    settings->type     = init->port_type;
    settings->port_idx = port_idx;

    if (hooks->init)
        rc = hooks->init(settings, init);
    if (rc)
    {
        vPortFree(settings);
        return rc;
    }

    tty_settings[port_idx] = settings;

    return 0;
}

/**********************************************************
 * init/PM hooks called from APP_SRTM
 *********************************************************/

void APP_TTY_InitService(void)
{
    ttyService = SRTM_TtyService_Create(APP_TTY_tx, APP_TTY_setcflag, APP_TTY_setwake, APP_TTY_init);
    SRTM_Dispatcher_RegisterService(disp, ttyService);
}

void APP_TTY_SuspendTask(void)
{
    uint8_t i;

    for (i = 0; i < TTY_MAX_PORTS; i++)
    {
        struct tty_settings *settings = get_settings(i, NULL);

        if (!settings)
            continue;

        if (tty_hooks[settings->type]->suspendTask)
            tty_hooks[settings->type]->suspendTask(settings);
    }
}

void APP_TTY_ResumeTask(void)
{
    uint8_t i;

    for (i = 0; i < TTY_MAX_PORTS; i++)
    {
        struct tty_settings *settings = get_settings(i, NULL);

        if (!settings)
            continue;

        if (tty_hooks[settings->type]->resumeTask)
            tty_hooks[settings->type]->resumeTask(settings);
    }
}

void APP_TTY_Suspend(void)
{
    uint8_t i;

    for (i = 0; i < TTY_MAX_PORTS; i++)
    {
        struct tty_settings *settings = get_settings(i, NULL);

        if (!settings)
            continue;

        if (tty_hooks[settings->type]->suspend)
            tty_hooks[settings->type]->suspend(settings);
    }
}

void APP_TTY_Resume(void)
{
    uint8_t i;

    for (i = 0; i < TTY_MAX_PORTS; i++)
    {
        struct tty_settings *settings = get_settings(i, NULL);

        if (!settings)
            continue;

        if (tty_hooks[settings->type]->resume)
            tty_hooks[settings->type]->resume(settings);
    }
}
