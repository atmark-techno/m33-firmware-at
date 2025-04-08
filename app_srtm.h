/*
 * Copyright 2021-2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

#include "rpmsg_lite.h"
#include "fsl_wuu.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*
 * AD: Application Domain
 * LP: Low Power
 * Low Power Modes for Application Domain is indroduced in AD_PMCTRL of CMC1:
 * Active,
 * Sleep,
 * Deep Sleep,
 * Partial Active,
 * Power Down(PD),
 * Deep Power Down(DPD),
 * Hold
 */
typedef enum
{
    AD_UNKOWN,
    AD_ACT, /* Note: linux is in idle state(Switch between Active mode and Sleep Mode of APD) */
    AD_DSL, /* Application Domain enter Deep Sleep Mode when linux execute suspend command(echo mem > /sys/power/state,
                suspend to ram) */
    AD_PD,  /* Application Domain enter Power Down Mode when linux execute suspend command(echo mem > /sys/power/state,
                 suspend to ram) */
    AD_DPD, /* Application Domian enter Deep Power Down Mode when linux execute poweroff command */
} lpm_ad_power_mode_e;

typedef enum
{
    APP_SRTM_StateRun = 0x0U,
    APP_SRTM_StateLinkedUp,
    APP_SRTM_StateReboot,
    APP_SRTM_StateShutdown,
} app_srtm_state_t;

#define APP_MS2TICK(ms) ((ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS)
#define APP_DMA_IRQN(channel) (IRQn_Type)((uint32_t)DMA0_0_IRQn + channel)

/* Task priority definition, bigger number stands for higher priority */
#define APP_SRTM_MONITOR_TASK_PRIO (4U)
#define APP_SRTM_DISPATCHER_TASK_PRIO (3U)

/* IRQ handler priority definition, bigger number stands for lower priority */
#define APP_LPI2C_IRQ_PRIO (5U)
#define APP_GPIO_IRQ_PRIO (5U)
#define APP_WUU_IRQ_PRIO (5U)
#define APP_WDT_IRQ_PRIO (5U)
#define APP_CMC1_IRQ_PRIO (5U)
#define APP_BBNSM_IRQ_PRIO (5U)

/* Define the timeout ms to polling the A Core link up status */
#define APP_LINKUP_TIMER_PERIOD_MS (10U)

/* Define the timeout ms to refresh s400 watchdog timer to keep s400 alive
 * (1 hour, dies after 24h) */
#define APP_REFRESH_S400_WDG_TIMER_PERIOD_MS (60 * 60 * 1000U)

#define RPMSG_LITE_SRTM_SHMEM_BASE (VDEV0_VRING_BASE)
#define RPMSG_LITE_SRTM_LINK_ID (RL_PLATFORM_IMX8ULP_M33_A35_SRTM_LINK_ID)

#define APP_SRTM_I2C_CHANNEL_NAME "rpmsg-i2c-channel"
#define APP_SRTM_IO_CHANNEL_NAME "rpmsg-io-channel"
#define APP_SRTM_PWM_CHANNEL_NAME "rpmsg-pwm-channel"
#define APP_SRTM_ADC_CHANNEL_NAME "rpmsg-adc-channel"
#define APP_SRTM_LFCL_CHANNEL_NAME "rpmsg-life-cycle-channel"
#define APP_SRTM_WDOG_CHANNEL_NAME "rpmsg-wdog-channel"
#define APP_SRTM_TTY_CHANNEL_NAME "rpmsg-tty-channel"
#define APP_SRTM_CAN_CHANNEL_NAME "rpmsg-can-channel"
#define APP_SRTM_SPI_CHANNEL_NAME "rpmsg-spi-channel"

#define PEER_CORE_ID (1U)

/* GPIO */
#define APP_IO_PINS_PER_CHIP 25U
#define APP_IO_CHIPS 3U /* Only support GPIOA, GPIOB and GPIOC */
#define APP_IO_NUM (APP_IO_CHIPS * APP_IO_PINS_PER_CHIP)
#define APP_WUU_PINS_NUM (2 * APP_IO_PINS_PER_CHIP)
#define APP_GPIO_IDX(ioId) ((uint8_t)(((uint16_t)ioId) >> 8U))
#define APP_PIN_IDX(ioId) ((uint8_t)ioId)
#define APP_IO_ID(gpio, pin) ((uint16_t)(((uint8_t)gpio << 8U) | (uint8_t)pin))
#define APP_IO_IDX(gpio, pin) ((uint16_t)((uint8_t)gpio * APP_IO_PINS_PER_CHIP + (uint8_t)pin))

static inline uint16_t APP_IO_GetIndex(uint16_t ioId)
{
    uint8_t gpio_idx = APP_GPIO_IDX(ioId);
    uint8_t pin_idx  = APP_PIN_IDX(ioId);

    if (gpio_idx >= APP_IO_CHIPS)
        return 0xffff;
    if (pin_idx >= APP_IO_PINS_PER_CHIP)
        return 0xffff;

    return gpio_idx * APP_IO_PINS_PER_CHIP + pin_idx;
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
    return wuuPins[APP_IO_IDX(gpio_idx, pin_idx)];
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

#define APP_GPIO_INT_SEL (kRGPIO_InterruptOutput2)

/* XXX interrupts etc still hardcoded*/
#define APP_PIN_PTA19 (0x0013U) /* PTA19 use for it6161(mipi to hdmi converter ic) interrupt */
#define APP_PIN_IT6161_INT (APP_PIN_PTA19)
#define APP_INPUT_IT6161_INT (APP_IO_GetIndex(APP_PIN_IT6161_INT))
#define APP_PIN_PTB4 (0x0104U) /* PTB4 */

extern int32_t RPMsg_MU0_A_IRQHandler(void);

typedef void (*app_rpmsg_monitor_t)(struct rpmsg_lite_instance *rpmsgHandle, bool ready, void *param);
typedef void (*app_irq_handler_t)(IRQn_Type irq, void *param);

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/* Wake linux */
void APP_SRTM_WakeupCA35(void);

/* Initialize SRTM contexts */
void APP_SRTM_Init(void);

/* Create RPMsg channel and start SRTM communication */
void APP_SRTM_StartCommunication(void);

/* Set RPMsg channel init/deinit monitor */
void APP_SRTM_SetRpmsgMonitor(app_rpmsg_monitor_t monitor, void *param);

uint8_t APP_Read_I2C_Register(uint8_t busID, uint16_t slaveAddr, uint8_t regIndex);
uint8_t APP_Write_I2C_Register(uint8_t busID, uint16_t slaveAddr, uint8_t regIndex, uint8_t value);

/* used for tty wakeup by gpio */
void APP_IO_SetupWUU(uint8_t wuuIdx, wuu_external_pin_edge_detection_t wuuEdge);

/* Set IRQ handler for application */
void APP_SRTM_SetIRQHandler(app_irq_handler_t handler, void *param);

/* Enable or disable wakeup pin
 * event[7:0]: llwu_external_pin_mode_t
 * event[8]: LLWU wakeup enable
 */
void APP_SRTM_SetWakeupPin(uint16_t ioId, uint16_t event);

void APP_SRTM_Suspend(void);
void APP_SRTM_Resume(void);

bool APP_SRTM_GetSupportDSLForApd(void);
void APP_SRTM_SetSupportDSLForApd(bool support);

void APP_SRTM_EmulateGPIOHandler(uint8_t wuuPin);
#if defined(__cplusplus)
}
#endif
