/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

#include "srtm_dispatcher.h"
#include "fsl_rgpio.h"

/* Required to register srtm dispatchers from app_*.c */
extern srtm_dispatcher_t disp;

/* Used by drivers allowing flexio to ensure only one use it at the same time */
extern bool flexio_used;

/* app_adc.c */
void APP_ADC_InitService(void);
void APP_ADC_Resume(void);

/* app_gpio.c */
void APP_GPIO_InitService(void);
void APP_GPIO_ResetService(srtm_peercore_t core);
void pinctrl_set(uint32_t pinctrl0, uint32_t pinctrl1, uint32_t pinctrl2, uint32_t pinctrl3, uint32_t pinctrl4,
                 uint32_t pinctrl5);

/* app_i2c.c */
void APP_I2C_InitService(void);
void APP_I2C_Resume(void);
void APP_I2C_ResetService(void);
void APP_I2C_uboot(uint32_t subcommand);

/* app_pwm.c */
void APP_PWM_InitService(void);

/* app_spi.c */
void APP_SPI_InitService(void);

/* app_audio.c */
void APP_AUDIO_InitService(void);

/* app_dac.c */
void APP_DAC_InitService(void);

/* app_wdog.c */
void APP_WDOG_InitService(void);
void APP_WDOG_Resume(void);
void APP_WDOG_Suspend(void);
void APP_WDOG_uboot(uint32_t subcommand);
void APP_WDOG_ResetLog(void);
