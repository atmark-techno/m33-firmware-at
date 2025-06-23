/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "app_tty.h"
#include "build_bug.h"

// demo variables:
// timer handler to periodically report flow rate
static TimerHandle_t xAutoReloadTimer;
// tty handle to send message to linux from timer
static struct tty_settings *tty;
// time of last edge pulses
portTickType gOldTime, gNewTime, gOldDiffTime;

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
void custom_tty_to_linux(struct tty_settings *settings, char *buf, uint16_t len)
{
    /* We should not send data to linux when the tty is not active to avoid filling buffer on linux side */
    if ((settings->state & (TTY_ACTIVE | TTY_SUSPENDED)) != TTY_ACTIVE)
        return;

    while (len > 0)
    {
        uint8_t *send_buf;
        uint16_t send_len;

        /* get buffer from SRTM */
        srtm_notification_t notif = SRTM_TtyService_NotifyAlloc(settings->port_idx, &send_buf, &send_len);

        /* copy data and send */
        send_len = MIN(send_len, len);
        memcpy(send_buf, buf, send_len);
        SRTM_TtyService_NotifySend(ttyService, notif, len);

        len -= send_len;
        buf += send_len;
    }
}

/* tx is from point of view of linux, this function is called when
 * data is sent from linux */
static int custom_tty_tx(struct tty_settings *settings, uint8_t *buf, uint16_t len)
{
    // ignore data sent here

    return 0;
}

static void autoReloadTimerCallback(TimerHandle_t xTimer)
{
    portTickType NowTime, DiffTime, OldTime, NewTime;
    uint32_t ml;
    taskENTER_CRITICAL();
    NewTime = gNewTime;
    OldTime = gOldTime;
    taskEXIT_CRITICAL();
    if ((OldTime != 0) && (NewTime != 0))
    {
        NowTime = xTaskGetTickCount();
        if ((NowTime - NewTime) >= pdMS_TO_TICKS(7500))
        {
            //最終取得から7.5秒経過していたら、0mlにし、パルス間隔取得はリスタート
            ml           = 0;
            gOldDiffTime = 0;
            taskENTER_CRITICAL();
            gNewTime = 0;
            gOldTime = 0;
            taskEXIT_CRITICAL();
        }
        else
        {
            if ((gOldDiffTime != 0) && ((NowTime - NewTime) > gOldDiffTime))
            {
                //最終パルスと今の時間の差分が、最終パルス間隔より大きいなら
                //今の時間からパルス間隔を計算する
                DiffTime = NowTime - NewTime;
            }
            else
            {
                //上記以外はふつうに計算
                DiffTime     = NewTime - OldTime;
                gOldDiffTime = DiffTime;
            }
            ml = 50 * pdMS_TO_TICKS(60000) / pdMS_TO_TICKS(DiffTime);
        }
    }
    else
    {
        ml = 0;
    }

    char str[32];
    int len;

    len = snprintf(str, sizeof(str), "now=%05umL/min\r\n", ml);
    custom_tty_to_linux(tty, str, len);
}

static int custom_tty_activate(struct tty_settings *settings)
{
    /* This function is called when linux first opens or last close the tty */
    PRINTF("custom tty is %s\r\n", (settings->state & TTY_ACTIVE) ? "open" : "closed");

    if (settings->state & TTY_ACTIVE)
    {
        // create timer to send data regularily
        if (!xAutoReloadTimer)
        {
            gOldTime         = 0;
            gNewTime         = 0;
            gOldDiffTime     = 0;
            xAutoReloadTimer = xTimerCreate("Reload", pdMS_TO_TICKS(1000), pdTRUE, NULL, autoReloadTimerCallback);
            assert(xAutoReloadTimer != NULL);
            xTimerStart(xAutoReloadTimer, portMAX_DELAY);
        }
    }
    else if (xAutoReloadTimer)
    {
        // stop it all
        xTimerStop(xAutoReloadTimer, portMAX_DELAY);
        xTimerDelete(xAutoReloadTimer, portMAX_DELAY);
        xAutoReloadTimer = NULL;
    }

    return 0;
}

static int custom_tty_init(struct tty_settings *settings, struct srtm_tty_init_payload *generic_init)
{
    struct custom_tty_settings *custom        = get_custom(settings);
    struct srtm_tty_init_custom_payload *init = &generic_init->custom;

    PRINTF("initializing tty %d as CUSTOM '%s'\r\n", settings->port_idx, init->name);

    // remember tty for timer
    tty = settings;

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
