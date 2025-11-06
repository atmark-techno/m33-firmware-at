/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

void APP_RTC_NotifyAlarm(void);

/* Init hook for main/app_srtm */
void APP_RTC_EarlyInit(void);
void APP_RTC_InitService(void);
