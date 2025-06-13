/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FreeRTOS.h"
// #include "timers.h"

#include "app_gpio.h"
#include "custom.h"

/* This file provides a base for custom application development (that does nothing at all).
 *
 * These functions are run at various point during lifecycle:
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
 * From there one can create FreeRTOS tasks/timers and implement features as appropriate.
 */

/* global variables */
// portTickType lastIrqTick;

void custom_early_init(void) {}
void custom_init(void)
{
    /* register GPIOs etc */
    // APP_GPIO_PinctrlSet(1, 12, IOMUXC_PTB12_PTB12, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
    // APP_GPIO_SetupGPIO_Input(1, 12);
    // APP_GPIO_SetupIRQ(1, 12, kRGPIO_InterruptRisingEdge, APP_GPIO_IRQCallback_Custom);
}
void custom_linux_boot(void) {}
void custom_early_suspend(void) {}
void custom_suspend(void) {}
void custom_resume(void) {}
void custom_late_resume(void) {}
void custom_m33_suspend(lpm_rtd_power_mode_e targetMode) {}
void custom_m33_resume(lpm_rtd_power_mode_e targetMode) {}

/* This function is called when and IRQ fires after having been registered with
 *   APP_GPIO_SetupIRQ(gpioIdx, pinIdx, edge, APP_GPIO_IRQCallback_Custom);
 * where edge is one of:
 *   kRGPIO_InterruptOrDMADisabled        // Interrupt/DMA request is disabled.
 *   kRGPIO_DMARisingEdge                 // DMA request on rising edge.
 *   kRGPIO_DMAFallingEdge                // DMA request on falling edge.
 *   kRGPIO_DMAEitherEdge                 // DMA request on either edge.
 *   kRGPIO_FlagRisingEdge                // Flag sets on rising edge.
 *   kRGPIO_FlagFallingEdge               // Flag sets on falling edge.
 *   kRGPIO_FlagEitherEdge                // Flag sets on either edge.
 *   kRGPIO_InterruptLogicZero            // Interrupt when logic zero.
 *   kRGPIO_InterruptRisingEdge           // Interrupt on rising edge.
 *   kRGPIO_InterruptFallingEdge          // Interrupt on falling edge.
 *   kRGPIO_InterruptEitherEdge           // Interrupt on either edge.
 *   kRGPIO_InterruptLogicOne             // Interrupt when logic one.
 *   kRGPIO_ActiveHighTriggerOutputEnable // Enable active high-trigger output.
 *   kRGPIO_ActiveLowTriggerOutputEnable  // Enable active low-trigger output.
 */
void custom_GPIO_IRQHandler(uint8_t gpioIdx, uint8_t pinIdx)
{
    /* note: asserts are only compiled in debug build */
    // assert(gpioIdx == 1 && pinIdx == 12);

    /* implement callback here */
    // lastIrqTick = xTaskGetTickCountFromISR();

    /* if required disable irq */
    // APP_GPIO_SetupIRQ(gpioIdx, pinIdx, kRGPIO_InterruptOrDMADisabled, APP_GPIO_IRQCallback_Custom);
}
