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
#define APP_SRTM_AUDIO_CHANNEL_NAME "rpmsg-audio-channel"
#define APP_SRTM_PDM_CHANNEL_NAME "rpmsg-micfil-channel"
#define APP_SRTM_DAC_CHANNEL_NAME "rpmsg-dac-channel"

#define PEER_CORE_ID (1U)

extern int32_t RPMsg_MU0_A_IRQHandler(void);

typedef void (*app_rpmsg_monitor_t)(struct rpmsg_lite_instance *rpmsgHandle, bool ready, void *param);
typedef void (*app_irq_handler_t)(IRQn_Type irq, void *param);

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/* for hardfault handler */
void hardfault_process_uboot_messages(void);

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

/* Set IRQ handler for application */
void APP_SRTM_SetIRQHandler(app_irq_handler_t handler, void *param);

/* Enable or disable wakeup pin
 * event[7:0]: llwu_external_pin_mode_t
 * event[8]: LLWU wakeup enable
 */
void APP_SRTM_SetWakeupPin(uint16_t ioId, uint16_t event);

/*
 * suspend/resume functions timing:
 * - linux sends "will suspend" lfcl message: APP_SRTM_EarlySuspend()
 * - MU0_A_IRQHandler gets A core powerdown event: A core power change,
 *   trigger m33 suspend (APP_SleepWithLinux)
 * - main's HandleSuspendTask enters low power mode: APP_SRTM_Suspend()
 * - no task is scheduled for a while: idle task:
 *   vPortSuppressTicksAndSleep -> APP_PowerPreSwitchHook
 * - wakeup event: APP_PowerPostSwitchHook() + give semaphore for HandleSuspendTask
 * - HandleSuspendTask wakeup: APP_SRTM_Resume()
 * - APP_SRTM_Resume() calls APP_SRTM_LateResume() as there is no difference for now.
 *
 * If MU0_A_IRQHandler did not get A core powerdown in 3s APP_AbortSuspendCallback()
 * is called and that calls APP_SRTM_LateResume()
 */
void APP_SRTM_EarlySuspend(void);
void APP_SRTM_Suspend(void);
void APP_SRTM_Resume(void);
void APP_SRTM_LateResume(void);

bool APP_SRTM_GetSupportDSLForApd(void);
void APP_SRTM_SetSupportDSLForApd(bool support);

void APP_SRTM_EmulateGPIOHandler(uint8_t wuuPin);
#if defined(__cplusplus)
}
#endif
