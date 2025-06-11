/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "app_gpio.h"

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
