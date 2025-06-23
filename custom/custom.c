/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FreeRTOS.h"
#include "timers.h"

#include "app_gpio.h"
#include "custom.h"
#include "main.h"

/* This file provides a base for custom application development (that does nothing at all). */

/*******************
 * Global variables
 *******************/
// portTickType lastIrqTick;

/*******************
 * Hooks
 *******************/
/* These functions are run at various point during lifecycle:
 * - custom_early_init(): m33 startup
 * - custom_init(): m33 startup, after srtm/drivers init
 * - custom_linux_boot(): linux startup
 * - custom_early_suspend()/custom_suspend()/custom_resume()/custom_late_resume():
 *   a35 (linux) suspend/resume, specifically:
 *   * linux announces suspend (spurrious wakeup sources disabling)
 *   * A35 core poweroff
 *   * M33 signals A35 to wakeup (before srtm drivers init)
 *   * M33 signals A35 to wakeup (after srtm drivers init)
 * - custom_m33_suspend()/custom_m33_resume(): m33 low power mode enter/exit
 *
 * From there one can create FreeRTOS tasks/timers and implement features as appropriate. */
void custom_early_init(void) {}
void custom_init(void)
{
    /* PTB12 = DI1 */
    APP_GPIO_PinctrlSet(IOMUXC_PTB12_PTB12, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
    APP_GPIO_SetupGPIO_Input(1, 12);
    APP_GPIO_SetupIRQ(1, 12, kRGPIO_InterruptRisingEdge, APP_GPIO_IRQCallback_Custom);

    /* Setup to keep m33 alive at lower rate when linux suspends */
    sleepWithLinux = LPM_PowerModeActiveUD;

    initTimer();
}
void custom_linux_boot(void) {}
void custom_early_suspend(void) {}
void custom_suspend(void) {}
void custom_resume(void) {}
void custom_late_resume(void) {}
void custom_m33_suspend(lpm_rtd_power_mode_e targetMode) {}
void custom_m33_resume(lpm_rtd_power_mode_e targetMode) {}

/*******************
 * GPIOs
 *******************/
/* Note: GPIOs are shared with linux and must be used with the API defined in app_gpio.h
 * to avoid conflicts */
/* This function is called when and IRQ has been registered with APP_GPIO_IRQCallback_Custom */
void custom_GPIO_IRQHandler(uint8_t gpioIdx, uint8_t pinIdx)
{
    /* note: asserts are only compiled in debug build */
    assert(gpioIdx == 1 && pinIdx == 12);

    /* implement callback here */
    gOldTime = gNewTime;
    gNewTime = xTaskGetTickCountFromISR();
}
