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

#include "main.h"
#include "app_srtm.h"
#include "app_gpio.h"
#include "app_srtm_internal.h"
#include "srtm_io_service.h"
#include "build_bug.h"

#define APP_GPIO_INT_SEL (kRGPIO_InterruptOutput2)

static srtm_service_t ioService;
const uint8_t wuuPins[APP_WUU_PINS_NUM] = {
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

static uint8_t gpio_irq_cb[APP_IO_NUM];

#define IO_PINCTRL_UNSET 0xffffffffU
#define PIN_FUNC_ID_SIZE (6)
static uint32_t pinFuncId[APP_IO_NUM][PIN_FUNC_ID_SIZE] = {
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
    { IOMUXC_PTB10_PTB10, -1 },
    { IOMUXC_PTB11_PTB11, -1 },
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
static void APP_IO_SetPinConfig(uint8_t gpio, uint8_t pin, uint32_t defaultPinctrl)
{
    int index = APP_IO_Index(gpio, pin);

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
 * API for other services/internal functions
 **********************************************/
int APP_GPIO_PinctrlSet(uint32_t pinctrl0, uint32_t pinctrl1, uint32_t pinctrl2, uint32_t pinctrl3, uint32_t pinctrl4,
                        uint32_t pinctrl5)
{
    /* pinctrl0 is the address of IOMUXC0->PCR0_IOMUXCARRAY[0-2], so we can get the pin back
     * from there, and thus compute the index without having gpioIdx and pinIdx as explicit
     * arguments */
    uint8_t gpioIdx = 255, pinIdx = 255;
    if (pinctrl0 < (uintptr_t)IOMUXC0->PCR0_IOMUXCARRAY0)
    {
        goto inval;
    }
    else if (pinctrl0 < (uintptr_t)IOMUXC0->PCR0_IOMUXCARRAY1)
    {
        gpioIdx = 0;
        // guaranteed to be < 256 so no need for extra check
        pinIdx = (pinctrl0 - (uintptr_t)IOMUXC0->PCR0_IOMUXCARRAY0) / sizeof(IOMUXC0->PCR0_IOMUXCARRAY0[0]);
    }
    else if (pinctrl0 < (uintptr_t)IOMUXC0->PCR0_IOMUXCARRAY2)
    {
        gpioIdx = 1;
        pinIdx  = (pinctrl0 - (uintptr_t)IOMUXC0->PCR0_IOMUXCARRAY1) / sizeof(IOMUXC0->PCR0_IOMUXCARRAY1[0]);
    }
    else if (pinctrl0 < (uintptr_t)(IOMUXC0->PCR0_IOMUXCARRAY2 + 32))
    {
        gpioIdx = 2;
        pinIdx  = (pinctrl0 - (uintptr_t)IOMUXC0->PCR0_IOMUXCARRAY2) / sizeof(IOMUXC0->PCR0_IOMUXCARRAY2[0]);
    }
    else
    {
        goto inval;
    }

    int index = APP_IO_Index(gpioIdx, pinIdx);
    if (index >= APP_IO_NUM)
        goto inval;

    if (pinFuncId[index][5] != -1)
    {
        PRINTF("pinctrl %d/%d was already set\r\n", gpioIdx, pinIdx);
        return -EBUSY;
    }

    /* remember pinctrl and apply it */
    pinFuncId[index][1] = pinctrl1;
    pinFuncId[index][2] = pinctrl2;
    pinFuncId[index][3] = pinctrl3;
    pinFuncId[index][4] = pinctrl4;
    pinFuncId[index][5] = pinctrl5;
    APP_IO_SetPinConfig(gpioIdx, pinIdx, 0);

    return 0;

inval:
    PRINTF("PinctrlSet first value (%x) does not look valid (%d/%d)\r\n", pinctrl0, gpioIdx, pinIdx);
    return -EINVAL;
}

void APP_GPIO_SetupGPIO_Input(uint8_t gpioIdx, uint8_t pinIdx)
{
    if (gpioIdx >= APP_IO_CHIPS || pinIdx >= APP_IO_PINS_PER_CHIP)
    {
        PRINTF("SetupGPIO_Input invalid pins %d/%d\r\n", gpioIdx, pinIdx);
        return;
    }
    assert(gpioIdx < 3U);
    assert(pinIdx < 32U);
    rgpio_pin_config_t config = {
        .pinDirection = kRGPIO_DigitalInput,
    };

    RGPIO_PinInit(gpios[gpioIdx], pinIdx, &config);
}

void APP_GPIO_SetupGPIO_Output(uint8_t gpioIdx, uint8_t pinIdx, uint8_t value)
{
    if (gpioIdx >= APP_IO_CHIPS || pinIdx >= APP_IO_PINS_PER_CHIP)
    {
        PRINTF("SetupGPIO_Output invalid pins %d/%d\r\n", gpioIdx, pinIdx);
        return;
    }
    rgpio_pin_config_t config = {
        .outputLogic  = value,
        .pinDirection = kRGPIO_DigitalOutput,
    };

    RGPIO_PinInit(gpios[gpioIdx], pinIdx, &config);
}

void APP_GPIO_SetupIRQ(uint8_t gpioIdx, uint8_t pinIdx, rgpio_interrupt_config_t edge, enum APP_GPIO_IRQCallback cb)
{
    uint16_t index = APP_IO_Index(gpioIdx, pinIdx);

    assert(index < APP_IO_NUM);
    gpio_irq_cb[index] = cb;

    RGPIO_SetPinInterruptConfig(gpios[gpioIdx], pinIdx, APP_GPIO_INT_SEL, edge);
}

void APP_GPIO_SetupWUU(uint8_t gpioIdx, uint8_t pinIdx, wuu_external_pin_edge_detection_t wuuEdge)
{
    uint8_t wuuIdx = APP_IO_GetWUUPin(gpioIdx, pinIdx);
    if (wuuIdx == 255)
        return;

    wuu_external_wakeup_pin_config_t config = {
        .event = kWUU_ExternalPinInterrupt,
        .mode  = kWUU_ExternalPinActiveAlways,
        .edge  = wuuEdge,
    };
    WUU_SetExternalWakeUpPinsConfig(WUU0, wuuIdx, &config);
}

/**********************************************
 * SRTM service callbacks
 **********************************************/
static srtm_status_t APP_IO_PinctrlSet(srtm_service_t service, srtm_peercore_t core, uint16_t ioId, uint32_t pinctrl[6])
{
    uint8_t gpioIdx = APP_GPIO_IDX(ioId);
    uint8_t pinIdx  = APP_PIN_IDX(ioId);
    int index       = APP_IO_Index(gpioIdx, pinIdx);

    if (index >= APP_IO_NUM)
    {
        PRINTF("pinctrl %d/%d invalid pin\r\n", gpioIdx, pinIdx);
    }

    if (pinFuncId[index][0] != pinctrl[0])
    {
        PRINTF("pinctrl %d/%d first value %x did not match expected %x\r\n", gpioIdx, pinIdx, pinctrl[0],
               pinFuncId[index][0]);
        return -EINVAL;
    }

    if (APP_GPIO_PinctrlSet(pinctrl[0], pinctrl[1], pinctrl[2], pinctrl[3], pinctrl[4], pinctrl[5]))
        return SRTM_Status_Error;

    return SRTM_Status_Success;
}

static srtm_status_t APP_IO_OutputInit(srtm_service_t service, srtm_peercore_t core, uint16_t ioId,
                                       srtm_io_value_t ioValue)
{
    uint8_t gpioIdx = APP_GPIO_IDX(ioId);
    uint8_t pinIdx  = APP_PIN_IDX(ioId);

    assert(gpioIdx < 3U);
    assert(pinIdx < 32U);

    if (!APP_IO_PinIsGPIO(ioId))
    {
        PRINTF("Refusing to configure non-GPIO pin %x as output (val %d)\r\n", ioId, ioValue);
        return SRTM_Status_Error;
    }

    /* clear any WUU config if any... */
    APP_GPIO_SetupWUU(gpioIdx, pinIdx, kWUU_ExternalPinDisable);

    APP_IO_SetPinConfig(gpioIdx, pinIdx, IOMUXC_PCR_OBE_MASK);

    APP_GPIO_SetupGPIO_Output(gpioIdx, pinIdx, ioValue);

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

static srtm_status_t APP_IO_ConfInput(uint16_t ioId, srtm_io_event_t event, bool wakeup)
{
    uint8_t gpioIdx                           = APP_GPIO_IDX(ioId);
    uint8_t pinIdx                            = APP_PIN_IDX(ioId);
    uint8_t wuuIdx                            = APP_IO_GetWUUPin(gpioIdx, pinIdx);
    uint8_t inputIdx                          = APP_IO_GetIndex(ioId);
    wuu_external_pin_edge_detection_t wuuEdge = kWUU_ExternalPinDisable;
    rgpio_interrupt_config_t irqEdge          = kRGPIO_InterruptOrDMADisabled;

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

    APP_IO_SetPinConfig(gpioIdx, pinIdx, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
    APP_GPIO_SetupGPIO_Input(gpioIdx, pinIdx);

    switch (event)
    {
        case SRTM_IoEventRisingEdge:
            irqEdge = kRGPIO_InterruptRisingEdge;
            wuuEdge = kWUU_ExternalPinRisingEdge;
            break;
        case SRTM_IoEventFallingEdge:
            irqEdge = kRGPIO_InterruptFallingEdge;
            wuuEdge = kWUU_ExternalPinFallingEdge;
            break;
        case SRTM_IoEventEitherEdge:
            irqEdge = kRGPIO_InterruptEitherEdge;
            wuuEdge = kWUU_ExternalPinAnyEdge;
            break;
        case SRTM_IoEventLowLevel:
            irqEdge = kRGPIO_InterruptLogicZero;
            /* WUU cannot do level, wake on falling edge */
            wuuEdge = kWUU_ExternalPinFallingEdge;
            break;
        case SRTM_IoEventHighLevel:
            irqEdge = kRGPIO_InterruptLogicOne;
            /* WUU cannot do level, wake on rising edge */
            wuuEdge = kWUU_ExternalPinRisingEdge;
            break;
        case SRTM_IoEventDisable:
            irqEdge = kRGPIO_InterruptOrDMADisabled;
            break;
        default:
            break;
    }

    if (event != SRTM_IoEventNone)
        APP_GPIO_SetupIRQ(gpioIdx, pinIdx, irqEdge, APP_GPIO_IRQCallback_Linux);
    if (!wakeup)
        wuuEdge = kWUU_ExternalPinDisable;
    APP_GPIO_SetupWUU(gpioIdx, pinIdx, wuuEdge);

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
    uint8_t pin;
    uint32_t mask;

    for (pin = 0; pin < APP_IO_PINS_PER_CHIP; pin++)
    {
        mask = 1U << pin;
        if (!(flags & mask))
            continue;
        ioId = APP_IO_ID(gpioIdx, pin);
        switch (gpio_irq_cb[APP_IO_Index(gpioIdx, pin)])
        {
            case APP_GPIO_IRQCallback_Linux:
                SRTM_IoService_NotifyInputEvent(ioService, ioId);
                // disable further irq for pin, linux will re-enable after processing
                // (this is necessary e.g. for level interrupts to not spam)
                RGPIO_SetPinInterruptConfig(gpio, pin, APP_GPIO_INT_SEL, kRGPIO_InterruptOrDMADisabled);
                break;
            case APP_GPIO_IRQCallback_Custom:
                custom_GPIO_IRQHandler(gpioIdx, pin);
                break;
            default:
                PRINTF("Invalid irq cb for gpio %d/%d\r\n", gpio, pin);
                RGPIO_SetPinInterruptConfig(gpio, pin, APP_GPIO_INT_SEL, kRGPIO_InterruptOrDMADisabled);
        }
        // clear isr
        RGPIO_ClearPinsInterruptFlags(gpio, APP_GPIO_INT_SEL, mask);
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
