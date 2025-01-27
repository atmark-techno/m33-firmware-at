/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

#include "srtm_dispatcher.h"

/* Required to register srtm dispatchers from app_*.c */
extern srtm_dispatcher_t disp;

/* app_adc.c */
void APP_ADC_InitService(void);
void APP_ADC_Resume(void);

/* app_i2c.c */
void APP_I2C_InitService(void);
void APP_I2C_Resume(void);
