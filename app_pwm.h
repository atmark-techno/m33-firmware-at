/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

/* Init hook for app_srtm.c */
void APP_PWM_InitService(void);

/* PM hooks from app_srtm.c */
void APP_PWM_Resume(void);
