/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

#include "fsl_rgpio.h"
#include "fsl_wuu.h"
#include "fsl_iomuxc.h"

/* driver m33 API */
int APP_GPIO_PinctrlSet(uint32_t pinctrl0, uint32_t pinctrl1, uint32_t pinctrl2, uint32_t pinctrl3, uint32_t pinctrl4,
                        uint32_t pinctrl5);

/* Not used directly anymore, use APP_GPIO_write/read below */
extern RGPIO_Type *const gpios[];

static inline void APP_GPIO_Write(uint8_t gpioIdx, uint8_t pinIdx, uint8_t output)
{
    RGPIO_PinWrite(gpios[gpioIdx], pinIdx, output);
}
static inline uint32_t APP_GPIO_Read(uint8_t gpioIdx, uint8_t pinIdx)
{
    return RGPIO_PinRead(gpios[gpioIdx], pinIdx);
}

void APP_GPIO_SetupGPIO_Input(uint8_t gpioIdx, uint8_t pinIdx);
void APP_GPIO_SetupGPIO_Output(uint8_t gpioIdx, uint8_t pinIdx, uint8_t value);

enum APP_GPIO_IRQCallback
{
    APP_GPIO_IRQCallback_Linux,  /* reserved for srtm driver */
    APP_GPIO_IRQCallback_Custom, /* calls custom_GPIO_IRQHandler(gpioIdx, pinIdx) on irq */
};
void APP_GPIO_SetupIRQ(uint8_t gpioIdx, uint8_t pinIdx, rgpio_interrupt_config_t edge, enum APP_GPIO_IRQCallback cb);

void APP_GPIO_SetupWUU(uint8_t gpioIdx, uint8_t pinIdx, wuu_external_pin_edge_detection_t wuuEdge);

/* defined in custom/gpio.c */
void custom_GPIO_IRQHandler(uint8_t gpioIdx, uint8_t pinIdx);

/* various constants and index manipulation helpers... */
#define APP_IO_PINS_PER_CHIP 25U
#define APP_IO_CHIPS 3U /* Only support GPIOA, GPIOB and GPIOC */
#define APP_IO_NUM (APP_IO_CHIPS * APP_IO_PINS_PER_CHIP)
#define APP_WUU_PINS_NUM (2 * APP_IO_PINS_PER_CHIP)
/* extract gpio and pin indices from an 'ioId' */
#define APP_GPIO_IDX(ioId) ((uint8_t)(((uint16_t)ioId) >> 8U))
#define APP_PIN_IDX(ioId) ((uint8_t)ioId)
#define APP_IO_SPLIT_ID(ioId) APP_GPIO_IDX(ioId), APP_PIN_IDX(ioId)
/* compute ioId from gpio/pin */
#define APP_IO_ID(gpio, pin) ((uint16_t)(((uint8_t)gpio << 8U) | (uint8_t)pin))

/* compute index in flat array from gpio/pin or ioId */
static inline uint16_t APP_IO_Index(uint8_t gpio_idx, uint8_t pin_idx)
{
    if (gpio_idx >= APP_IO_CHIPS)
        return 0xffff;
    if (pin_idx >= APP_IO_PINS_PER_CHIP)
        return 0xffff;
    return gpio_idx * APP_IO_PINS_PER_CHIP + pin_idx;
}
static inline uint16_t APP_IO_GetIndex(uint16_t ioId)
{
    uint8_t gpio_idx = APP_GPIO_IDX(ioId);
    uint8_t pin_idx  = APP_PIN_IDX(ioId);

    return APP_IO_Index(gpio_idx, pin_idx);
}
static inline uint16_t APP_IO_GetId(uint8_t inputIdx)
{
    if (inputIdx >= APP_IO_NUM)
        return 0xffff;
    return ((inputIdx / APP_IO_PINS_PER_CHIP) << 8U) | (inputIdx % APP_IO_PINS_PER_CHIP);
}
extern const uint8_t wuuPins[];
static inline uint8_t APP_IO_GetWUUPin(uint8_t gpio_idx, uint8_t pin_idx)
{
    /* only PTA/PTB */
    if (gpio_idx > 1)
        return 255;
    if (pin_idx >= APP_IO_PINS_PER_CHIP)
        return 255;
    return wuuPins[APP_IO_Index(gpio_idx, pin_idx)];
}

static inline uint16_t APP_WUUPin_TO_IoId(uint8_t pin)
{
    bool found = false;
    int i;

    for (i = 0; i < APP_WUU_PINS_NUM; i++)
    {
        if (wuuPins[i] == pin)
        {
            found = true;
            break;
        }
    }
    if (!found)
        return 0xffff;

    return APP_IO_ID((i / APP_IO_PINS_PER_CHIP), (i % APP_IO_PINS_PER_CHIP));
}
