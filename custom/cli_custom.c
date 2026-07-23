/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Use this file to define custom commands */
#include "FreeRTOS.h"
#include "timers.h"

#include <assert.h>
#include <string.h>

#include "fsl_common.h"

#include "cli.h"
#include "printf.h"

static struct tty_settings *linux_tty;
static TimerHandle_t xSendTimer;
// use static data to avoid consuming stack for large buffers
static char buf[3000];

// app_tty_custom.c
void custom_tty_to_linux(struct tty_settings *settings, char *buf, uint16_t len);
static void sendTimerCallback(TimerHandle_t xTiemr)
{
    static char fill = '0';
    int i = 0, step = 100;
    while (i < sizeof(buf))
    {
        memset(buf + i, fill, MIN(step, sizeof(buf) - i));
        i += step;
        fill++;
        if (fill > 'z')
            fill = '0';
        else if (fill < 'a' && fill > 'Z')
            fill = 'a';
        else if (fill < 'A' && fill > '9')
            fill = 'A';
    }
    buf[sizeof(buf) - 1] = '\n';
    buf[sizeof(buf) - 2] = '\r';
    custom_tty_to_linux(linux_tty, buf, sizeof(buf));
}

static int start(int argc, char **argv)
{
    if (!linux_tty)
    {
        PRINTF("No console\r\n");
        return 0;
    }
    xSendTimer = xTimerCreate("send", pdMS_TO_TICKS(1), pdTRUE, NULL, sendTimerCallback);
    assert(xSendTimer != NULL);
    xTimerStart(xSendTimer, portMAX_DELAY);
    return 0;
}
static int stop(int argc, char **argv)
{
    xTimerStop(xSendTimer, portMAX_DELAY);
    return 0;
}
void remember_tty(struct tty_settings *settings)
{
    linux_tty = settings;
    start(0, NULL);
}

const static struct CLI_command custom_commands[] = {
    { "start", start, /* help message */ NULL, /* usage message */ NULL },
    { "stop", stop, /* help message */ NULL, /* usage message */ NULL },
    {
        0,
    }, /* sentinel */
};
const struct CLI_command *CLI_Custom_commands = custom_commands;
