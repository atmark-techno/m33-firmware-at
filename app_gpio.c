/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* GPIO (and WUU) handling */
#include <errno.h>

#include "FreeRTOS.h"
#include "timers.h"

#include "fsl_wuu.h"
#include "pin_mux.h"
#include "fsl_iomuxc.h"
#include "fsl_rgpio.h"

#include "main.h"
#include "app_srtm.h"
#include "app_gpio.h"
#include "app_srtm_internal.h"
#include "srtm_io_service.h"
#include "build_bug.h"

#define APP_GPIO_INT_SEL (kRGPIO_InterruptOutput2)

static srtm_service_t ioService;
const uint8_t wuuPins[] = {
    0,   /* WUU_P0 PTA0 */
    255, /* PTA1 */
    255, /* PTA2 */
    1,   /* WUU0_P1 PTA3 */
    2,   /* WUU0_P2 PTA4 */
    255, /* PTA5 */
    3,   /* WUU0_P3 PTA6 */
    4,   /* WUU0_P4 PTA7 */
    5,   /* WUU0_P5 PTA8 */
    6,   /* WUU0_P6 PTA9 */
    7,   /* WUU0_P7 PTA10 */
    8,   /* WUU0_P8 PTA11 */
    9,   /* WUU0_P9 PTA12 */
    10,  /* WUU0_P10 PTA13 */
    11,  /* WUU0_P11 PTA14 */
    12,  /* WUU0_P12 PTA15 */
    13,  /* WUU0_P13 PTA16 */
    14,  /* WUU0_P14 PTA17 */
    15,  /* WUU0_P15 PTA18 */
    255, /* PTA19 */
    255, /* PTA20 */
    255, /* PTA21 */
    255, /* PTA22 */
    255, /* PTA23 */
    16,  /* WUU0_P16 PTA24 */
    17,  /* WUU0_P17 PTB0 */
    18,  /* WUU0_P18 PTB1 */
    19,  /* WUU0_P19 PTB2 */
    20,  /* WUU0_P20 PTB3 */
    21,  /* WUU0_P21 PTB4 */
    22,  /* WUU0_P22 PTB5 */
    23,  /* WUU0_P23 PTB6 */
    255, /* PTB7 */
    255, /* PTB8 */
    255, /* PTB9 */
    255, /* PTB10 */
    255, /* PTB11 */
    24,  /* WUU0_P24 PTB12 */
    25,  /* WUU0_P25 PTB13 */
    26,  /* WUU0_P26 PTB14 */
    27,  /* WUU0_P27 PTB15 */
    255, /* 16 (no more pins after PTB15) */
    255, /* 17 */
    255, /* 18 */
    255, /* 19 */
    255, /* 20 */
    255, /* 21 */
    255, /* 22 */
    255, /* 23 */
    255, /* 24 */
};

RGPIO_Type *const gpios[] = RGPIO_BASE_PTRS;

#define IO_PINCTRL_UNSET 0xffffffffU
#define PIN_FUNC_ID_SIZE (6)
static uint32_t pinFuncId[][PIN_FUNC_ID_SIZE] = {
    { IOMUXC_PTA0_PTA0, -1 },
    { IOMUXC_PTA1_PTA1, -1 },
    { IOMUXC_PTA2_PTA2, -1 },
    { IOMUXC_PTA3_PTA3, -1 },
    { IOMUXC_PTA4_PTA4, -1 },
    { IOMUXC_PTA5_PTA5, -1 },
    { IOMUXC_PTA6_PTA6, -1 },
    { IOMUXC_PTA7_PTA7, -1 },
    { IOMUXC_PTA8_PTA8, -1 },
    { IOMUXC_PTA9_PTA9, -1 },
    { IOMUXC_PTA10_PTA10, -1 },
    { IOMUXC_PTA11_PTA11, -1 },
    { IOMUXC_PTA12_PTA12, -1 },
    { IOMUXC_PTA13_PTA13, -1 },
    { IOMUXC_PTA14_PTA14, -1 },
    { IOMUXC_PTA15_PTA15, -1 },
    { IOMUXC_PTA16_PTA16, -1 },
    { IOMUXC_PTA17_PTA17, -1 },
    { IOMUXC_PTA18_PTA18, -1 },
    { IOMUXC_PTA19_PTA19, -1 },
    { IOMUXC_PTA20_PTA20, -1 },
    { IOMUXC_PTA21_PTA21, -1 },
    { IOMUXC_PTA22_PTA22, -1 },
    { IOMUXC_PTA23_PTA23, -1 },
    { IOMUXC_PTA24_PTA24, -1 },
    { IOMUXC_PTB0_PTB0, -1 },
    { IOMUXC_PTB1_PTB1, -1 },
    { IOMUXC_PTB2_PTB2, -1 },
    { IOMUXC_PTB3_PTB3, -1 },
    { IOMUXC_PTB4_PTB4, -1 },
    { IOMUXC_PTB5_PTB5, -1 },
    { IOMUXC_PTB6_PTB6, -1 },
    { IOMUXC_PTB7_PTB7, -1 },
    { IOMUXC_PTB8_PTB8, -1 },
    { IOMUXC_PTB9_PTB9, -1 },
    { 0 }, /* PTB10 and 11 are used for upower and should never be used here */
    { 0 },
    { IOMUXC_PTB12_PTB12, -1 },
    { IOMUXC_PTB13_PTB13, -1 },
    { IOMUXC_PTB14_PTB14, -1 },
    { IOMUXC_PTB15_PTB15, -1 },
    { 0 }, /* no PTB after 15 */
    { 0 },
    { 0 },
    { 0 },
    { 0 },
    { 0 },
    { 0 },
    { 0 },
    { 0 },
    { IOMUXC_PTC0_PTC0, -1 },
    { IOMUXC_PTC1_PTC1, -1 },
    { IOMUXC_PTC2_PTC2, -1 },
    { IOMUXC_PTC3_PTC3, -1 },
    { IOMUXC_PTC4_PTC4, -1 },
    { IOMUXC_PTC5_PTC5, -1 },
    { IOMUXC_PTC6_PTC6, -1 },
    { IOMUXC_PTC7_PTC7, -1 },
    { IOMUXC_PTC8_PTC8, -1 },
    { IOMUXC_PTC9_PTC9, -1 },
    { IOMUXC_PTC10_PTC10, -1 },
    { IOMUXC_PTC11_PTC11, -1 },
    { IOMUXC_PTC12_PTC12, -1 },
    { IOMUXC_PTC13_PTC13, -1 },
    { IOMUXC_PTC14_PTC14, -1 },
    { IOMUXC_PTC15_PTC15, -1 },
    { IOMUXC_PTC16_PTC16, -1 },
    { IOMUXC_PTC17_PTC17, -1 },
    { IOMUXC_PTC18_PTC18, -1 },
    { IOMUXC_PTC19_PTC19, -1 },
    { IOMUXC_PTC20_PTC20, -1 },
    { IOMUXC_PTC21_PTC21, -1 },
    { IOMUXC_PTC22_PTC22, -1 },
    { IOMUXC_PTC23_PTC23, -1 },
    { 0 }, /* no PTC24 */
};

void pinctrl_set(uint32_t pinctrl0, uint32_t pinctrl1, uint32_t pinctrl2, uint32_t pinctrl3, uint32_t pinctrl4,
                 uint32_t pinctrl5)
{
    IOMUXC_SetPinMux(pinctrl0, pinctrl1, pinctrl2, pinctrl3, pinctrl4, 0);
    IOMUXC_SetPinConfig(pinctrl0, pinctrl1, pinctrl2, pinctrl3, pinctrl4, pinctrl5);
}

/*
 * @brief Set pad control register
 * @param asInput    use gpio as input, unless use as output
 */
static void APP_IO_SetPinConfig(uint16_t ioId, uint32_t defaultPinctrl)
{
    int index = APP_IO_GetIndex(ioId);

    /* check table is sound... */
    BUILD_BUG_ON(ARRAY_SIZE(pinFuncId) != APP_IO_NUM);
    BUILD_BUG_ON(ARRAY_SIZE(wuuPins) != APP_WUU_PINS_NUM);

    assert(index < APP_IO_NUM);

    pinctrl_set(pinFuncId[index][0], pinFuncId[index][1], pinFuncId[index][2], pinFuncId[index][3], pinFuncId[index][4],
                pinFuncId[index][5] == -1 ? defaultPinctrl : pinFuncId[index][5]);
}

static bool APP_IO_PinIsGPIO(uint16_t ioId)
{
    int index = APP_IO_GetIndex(ioId);
    assert(index < APP_IO_NUM);

    /* in imx8ulp, pinmux for GPIO is always function 1 */
    return pinFuncId[index][1] == 1;
}

/**********************************************
 * SRTM service callbacks
 **********************************************/
static srtm_status_t APP_IO_PinctrlSet(srtm_service_t service, srtm_peercore_t core, uint16_t ioId, uint32_t pinctrl[6])
{
    int index = APP_IO_GetIndex(ioId);
    assert(index < APP_IO_NUM);

    if (pinFuncId[index][5] != -1)
    {
        PRINTF("pinctrl %x was already set\r\n", ioId);
        return SRTM_Status_Error;
    }
    if (pinFuncId[index][0] != pinctrl[0])
    {
        PRINTF("pinctrl %x first value %x did not match expected %x\r\n", ioId, pinctrl[0], pinFuncId[index][0]);
        return SRTM_Status_Error;
    }

    /* remember pinctrl and apply it */
    memcpy(pinFuncId[index], pinctrl, sizeof(pinFuncId[0]));
    APP_IO_SetPinConfig(ioId, 0);

    return SRTM_Status_Success;
}

static srtm_status_t APP_IO_OutputInit(srtm_service_t service, srtm_peercore_t core, uint16_t ioId,
                                       srtm_io_value_t ioValue)
{
    uint8_t gpioIdx = APP_GPIO_IDX(ioId);
    uint8_t pinIdx  = APP_PIN_IDX(ioId);

    assert(gpioIdx < 3U);
    assert(pinIdx < 32U);
    assert(index < APP_IO_NUM);

    if (!APP_IO_PinIsGPIO(ioId))
    {
        PRINTF("Refusing to configure non-GPIO pin %x as output (val %d)\r\n", ioId, ioValue);
        return SRTM_Status_Error;
    }

    /* clear any WUU config if any... */
    uint8_t wuuIdx = APP_IO_GetWUUPin(gpioIdx, pinIdx);
    APP_IO_SetupWUU(wuuIdx, kWUU_ExternalPinDisable);

    APP_IO_SetPinConfig(ioId, IOMUXC_PCR_OBE_MASK);

    rgpio_pin_config_t config = {
        .outputLogic  = ioValue,
        .pinDirection = kRGPIO_DigitalOutput,
    };

    RGPIO_PinInit(gpios[gpioIdx], pinIdx, &config);

    return SRTM_Status_Success;
}

static srtm_status_t APP_IO_InputGet(srtm_service_t service, srtm_peercore_t core, uint16_t ioId,
                                     srtm_io_value_t *pIoValue)
{
    uint8_t gpioIdx = APP_GPIO_IDX(ioId);
    uint8_t pinIdx  = APP_PIN_IDX(ioId);

    assert(gpioIdx < 3U); /* We only support GPIOA, GPIOB and GPIOC */
    assert(pinIdx < 32U);
    assert(pIoValue);

    if (!APP_IO_PinIsGPIO(ioId))
    {
        PRINTF("Refusing to get non-GPIO pin %x value\r\n", ioId);
        return SRTM_Status_Error;
    }

    *pIoValue = RGPIO_PinRead(gpios[gpioIdx], pinIdx) ? SRTM_IoValueHigh : SRTM_IoValueLow;

    return SRTM_Status_Success;
}

static srtm_status_t APP_IO_OutputSet(srtm_service_t service, srtm_peercore_t core, uint16_t ioId,
                                      srtm_io_value_t ioValue)
{
    uint8_t gpioIdx = APP_GPIO_IDX(ioId);
    uint8_t pinIdx  = APP_PIN_IDX(ioId);

    assert(index < APP_IO_NUM);
    assert(gpioIdx < 3U); /* We only support GPIOA, GPIOB and GPIOC */
    assert(pinIdx < 32U);

    if (!APP_IO_PinIsGPIO(ioId))
    {
        PRINTF("Refusing to set non-GPIO pin %x (val %d)\r\n", ioId, ioValue);
        return SRTM_Status_Error;
    }

    RGPIO_PinWrite(gpios[gpioIdx], pinIdx, (uint8_t)ioValue);

    return SRTM_Status_Success;
}

void APP_IO_SetupWUU(uint8_t wuuIdx, wuu_external_pin_edge_detection_t wuuEdge)
{
    if (wuuIdx == 255)
        return;

    wuu_external_wakeup_pin_config_t config = {
        .event = kWUU_ExternalPinInterrupt,
        .mode  = kWUU_ExternalPinActiveAlways,
        .edge  = wuuEdge,
    };
    WUU_SetExternalWakeUpPinsConfig(WUU0, wuuIdx, &config);
}

static srtm_status_t APP_IO_ConfInput(uint16_t ioId, srtm_io_event_t event, bool wakeup)
{
    uint8_t gpioIdx                           = APP_GPIO_IDX(ioId);
    uint8_t pinIdx                            = APP_PIN_IDX(ioId);
    uint8_t wuuIdx                            = APP_IO_GetWUUPin(gpioIdx, pinIdx);
    uint8_t inputIdx                          = APP_IO_GetIndex(ioId);
    wuu_external_pin_edge_detection_t wuuEdge = kWUU_ExternalPinDisable;

    if (gpioIdx >= APP_IO_CHIPS || pinIdx >= APP_IO_PINS_PER_CHIP || inputIdx >= APP_IO_NUM)
    {
        PRINTF("Invalid pin 0x%x\r\n", ioId);
        return SRTM_Status_Error;
    }
    if (wakeup && wuuIdx == 255)
    {
        PRINTF("Wakeup requested on %d/%d which has no wakeup\r\n", gpioIdx, pinIdx);
        return SRTM_Status_Error;
    }
    if (!APP_IO_PinIsGPIO(ioId))
    {
        PRINTF("Refusing to configure non-GPIO pin %x as input\r\n", ioId);
        return SRTM_Status_Error;
    }
    if (wakeup)
        PRINTF("Wakeup requested on %d/%d (WUU %d), mode %d\r\n", gpioIdx, pinIdx, wuuIdx, event);

    APP_IO_SetPinConfig(ioId, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
    /* set direction as input */
    rgpio_pin_config_t gpio_config = {
        .pinDirection = kRGPIO_DigitalInput,
    };
    RGPIO_PinInit(gpios[gpioIdx], pinIdx, &gpio_config);

    switch (event)
    {
        case SRTM_IoEventRisingEdge:
            RGPIO_SetPinInterruptConfig(gpios[gpioIdx], pinIdx, APP_GPIO_INT_SEL, kRGPIO_InterruptRisingEdge);
            wuuEdge = kWUU_ExternalPinRisingEdge;
            break;
        case SRTM_IoEventFallingEdge:
            RGPIO_SetPinInterruptConfig(gpios[gpioIdx], pinIdx, APP_GPIO_INT_SEL, kRGPIO_InterruptFallingEdge);
            wuuEdge = kWUU_ExternalPinFallingEdge;
            break;
        case SRTM_IoEventEitherEdge:
            RGPIO_SetPinInterruptConfig(gpios[gpioIdx], pinIdx, APP_GPIO_INT_SEL, kRGPIO_InterruptEitherEdge);
            wuuEdge = kWUU_ExternalPinAnyEdge;
            break;
        case SRTM_IoEventLowLevel:
            RGPIO_SetPinInterruptConfig(gpios[gpioIdx], pinIdx, APP_GPIO_INT_SEL, kRGPIO_InterruptLogicZero);
            /* WUU cannot do level, wake on falling edge */
            wuuEdge = kWUU_ExternalPinFallingEdge;
            break;
        case SRTM_IoEventHighLevel:
            RGPIO_SetPinInterruptConfig(gpios[gpioIdx], pinIdx, APP_GPIO_INT_SEL, kRGPIO_InterruptLogicOne);
            /* WUU cannot do level, wake on rising edge */
            wuuEdge = kWUU_ExternalPinRisingEdge;
            break;
        case SRTM_IoEventDisable:
            RGPIO_SetPinInterruptConfig(gpios[gpioIdx], pinIdx, APP_GPIO_INT_SEL, kRGPIO_InterruptOrDMADisabled);
            break;
        default:
            break;
    }

    if (!wakeup)
        wuuEdge = kWUU_ExternalPinDisable;
    APP_IO_SetupWUU(wuuIdx, wuuEdge);

    return SRTM_Status_Success;
}

static srtm_status_t APP_IO_InputInit(srtm_service_t service, srtm_peercore_t core, uint16_t ioId,
                                      srtm_io_event_t event, bool wakeup)
{
    return APP_IO_ConfInput(ioId, event, wakeup);
}

/**********************************************
 * IRQ handling
 **********************************************/
void APP_SRTM_EmulateGPIOHandler(uint8_t wuuPin)
{
    uint16_t ioId            = APP_WUUPin_TO_IoId(wuuPin);
    uint8_t __unused gpioIdx = APP_GPIO_IDX(ioId);
    uint8_t __unused pinIdx  = APP_PIN_IDX(ioId);

    assert(gpioIdx < 2); /* Only support GPIOA and GPIOB */
    assert(pinIdx < APP_IO_PINS_PER_CHIP);

    SRTM_IoService_NotifyInputEvent(ioService, ioId);
}

static void APP_HandleGPIOHander(uint8_t gpioIdx)
{
    RGPIO_Type *gpio = gpios[gpioIdx];
    uint32_t flags   = RGPIO_GetPinsInterruptFlags(gpio, APP_GPIO_INT_SEL);
    uint16_t ioId;
    uint8_t i;
    uint32_t idx;

    for (i = 0; i < APP_IO_PINS_PER_CHIP; i++)
    {
        idx = 1U << i;
        if (!(flags & idx))
            continue;
        ioId = APP_IO_ID(gpioIdx, i);
        SRTM_IoService_NotifyInputEvent(ioService, ioId);
        // disable further irq for pin, linux will re-enable after processing
        // (this is necessary e.g. for level interrupts to not spam)
        RGPIO_SetPinInterruptConfig(gpio, i, APP_GPIO_INT_SEL, kRGPIO_InterruptOrDMADisabled);
        // clear isr
        RGPIO_ClearPinsInterruptFlags(gpio, APP_GPIO_INT_SEL, idx);
    }
}

void GPIOA_INT0_IRQHandler(void)
{
    APP_HandleGPIOHander(0U);
}

void GPIOA_INT1_IRQHandler(void)
{
    APP_HandleGPIOHander(0U);
}

void GPIOB_INT0_IRQHandler(void)
{
    APP_HandleGPIOHander(1U);
}

void GPIOB_INT1_IRQHandler(void)
{
    APP_HandleGPIOHander(1U);
}

void GPIOC_INT0_IRQHandler(void)
{
    APP_HandleGPIOHander(2U);
}

void GPIOC_INT1_IRQHandler(void)
{
    APP_HandleGPIOHander(2U);
}

/**********************************************
 * Service lifecycle
 **********************************************/
void APP_GPIO_InitService(void)
{
    /* Enable interrupt for GPIO. */
    NVIC_SetPriority(GPIOA_INT0_IRQn, APP_GPIO_IRQ_PRIO);
    NVIC_SetPriority(GPIOA_INT1_IRQn, APP_GPIO_IRQ_PRIO);
    NVIC_SetPriority(GPIOB_INT0_IRQn, APP_GPIO_IRQ_PRIO);
    NVIC_SetPriority(GPIOB_INT1_IRQn, APP_GPIO_IRQ_PRIO);
    NVIC_SetPriority(GPIOC_INT0_IRQn, APP_GPIO_IRQ_PRIO);
    NVIC_SetPriority(GPIOC_INT1_IRQn, APP_GPIO_IRQ_PRIO);

    EnableIRQ(GPIOA_INT0_IRQn);
    EnableIRQ(GPIOA_INT1_IRQn);
    EnableIRQ(GPIOB_INT0_IRQn);
    EnableIRQ(GPIOB_INT1_IRQn);
    EnableIRQ(GPIOC_INT0_IRQn);
    EnableIRQ(GPIOC_INT1_IRQn);

    ioService = SRTM_IoService_Create(APP_IO_NUM, APP_IO_InputInit, APP_IO_OutputInit, APP_IO_InputGet,
                                      APP_IO_OutputSet, APP_IO_PinctrlSet);
    SRTM_Dispatcher_RegisterService(disp, ioService);
}

void APP_GPIO_ResetService(srtm_peercore_t core)
{
    SRTM_IoService_Reset(ioService, core);
}
