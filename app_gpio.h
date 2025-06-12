/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

/* Used by drivers for performance sensitive code manipulating gpios */
extern RGPIO_Type *const gpios[];

/* various constants and index manipulation helpers... */
#define APP_IO_PINS_PER_CHIP 25U
#define APP_IO_CHIPS 3U /* Only support GPIOA, GPIOB and GPIOC */
#define APP_IO_NUM (APP_IO_CHIPS * APP_IO_PINS_PER_CHIP)
#define APP_WUU_PINS_NUM (2 * APP_IO_PINS_PER_CHIP)
/* extract gpio and pin indices from an 'ioId' */
#define APP_GPIO_IDX(ioId) ((uint8_t)(((uint16_t)ioId) >> 8U))
#define APP_PIN_IDX(ioId) ((uint8_t)ioId)
/* compute ioId from gpio/pin */
#define APP_IO_ID(gpio, pin) ((uint16_t)(((uint8_t)gpio << 8U) | (uint8_t)pin))

/* compute index in flat array from gpio/pin or ioId */
#define APP_IO_Index(gpio, pin) ((uint16_t)((uint8_t)gpio * APP_IO_PINS_PER_CHIP + (uint8_t)pin))
static inline uint16_t APP_IO_GetIndex(uint16_t ioId)
{
    uint8_t gpio_idx = APP_GPIO_IDX(ioId);
    uint8_t pin_idx  = APP_PIN_IDX(ioId);

    if (gpio_idx >= APP_IO_CHIPS)
        return 0xffff;
    if (pin_idx >= APP_IO_PINS_PER_CHIP)
        return 0xffff;

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
