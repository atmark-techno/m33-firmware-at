/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

#include "fsl_lpuart_freertos.h"

#include "srtm_tty_service.h"

/*
 * state machine shared for all ttys:
 * starts at 0
 * activate toggles ACTIVE bit
 *    (before calling activate hook)
 * suspend toggles SUSPENDED bit
 *    (before calling suspendTask/resumeTask hooks)
 * in particular note resume() runs before resumeTask (and suspend() after
 * suspendTask()), so if resume/suspend temporarily kills the hardware then
 * it is safe to rely on the suspended bit for that.
 */
enum tty_state
{
    TTY_ACTIVE    = (1 << 0),
    TTY_SUSPENDED = (1 << 1),
} state;

struct tty_settings
{
    enum tty_rpmsg_init_type type;
    uint8_t port_idx;
    enum tty_state state;
    /* per-type data follows, force alignment to ensure 'settings + 1' works */
} __attribute__((aligned(4)));

struct tty_hooks
{
    int (*tx)(struct tty_settings *settings, uint8_t *buf, uint16_t len);
    int (*setcflag)(struct tty_settings *settings, tcflag_t cflag);
    int (*setwake)(struct tty_settings *settings, bool enable);
    int (*init)(struct tty_settings *settings, struct srtm_tty_init_payload *init);
    int (*activate)(struct tty_settings *settings);
    void (*suspendTask)(struct tty_settings *settings);
    void (*resumeTask)(struct tty_settings *settings);
    void (*suspend)(struct tty_settings *settings);
    void (*resume)(struct tty_settings *settings);
    size_t settings_size;
};

/* required by tty drivers to send messages back to linux */
extern srtm_service_t ttyService;

/* Init hook for app_srtm.c */
void APP_TTY_InitService(void);

/* PM hooks from app_srtm.c */
void APP_TTY_Suspend(void);
void APP_TTY_Resume(void);
