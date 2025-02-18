/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

enum can_state
{
    CAN_ACTIVE    = (1 << 0),
    CAN_SUSPENDED = (1 << 1),
};

/* required by tty drivers to send messages back to linux */
extern srtm_service_t canService;

/* Init hook for app_srtm.c */
void APP_CAN_InitService(void);

/* PM hooks from app_srtm.c */
void APP_CAN_Suspend(void);
void APP_CAN_SuspendTask(void);
void APP_CAN_Resume(void);
void APP_CAN_ResumeTask(void);
