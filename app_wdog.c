/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>

#include "FreeRTOS.h"
#include "timers.h"

#include "fsl_ewm.h"
#include "fsl_upower.h"

#include "main.h"
#include "app_srtm.h"
#include "app_srtm_internal.h"
#include "app_uboot.h"
#include "srtm_wdog_service.h"

#include "build_bug.h"

static srtm_service_t wdogService;
static uint16_t wdogTimeout;
static bool wdogFirstPingLogged;
static TimerHandle_t wdogSuspendPingTimer;

void EWM_IRQHandler(void)
{
    PRINTF("WATCHDOG IRQ!\r\n");
    SRTM_WdogService_NotifyPreTimeout(wdogService);
    EWM_DisableInterrupts(EWM0, kEWM_InterruptEnable);
}

static srtm_status_t wdog_ping(void)
{
    if (!wdogFirstPingLogged)
    {
        PRINTF("first watchdog ping\r\n");
        wdogFirstPingLogged = true;
    }
    EWM_Refresh(EWM0);
    return SRTM_Status_Success;
}

/* during light sleep we need to ping EWM regularily as it cannot be stopped */
static void wdog_suspend_ping_timer(TimerHandle_t xTimer)
{
    wdog_ping();
    xTimerStart(wdogSuspendPingTimer, portMAX_DELAY);
}

static srtm_status_t wdog_enable(bool enabled, uint16_t timeout_ms)
{

    bool is_running = EWM_GetStatusFlags(EWM0) & kEWM_RunningFlag;
    PRINTF("Watchdog %s (timeout %d)\r\n", enabled ? "start" : "stop", timeout_ms);

    /* timeout cannot be changed for EWM, just ignore any re-enable... */
    if (is_running)
    {
        PRINTF("(ignored action on running wdog; had %d)\r\n", wdogTimeout);
        return SRTM_Status_Success;
        /* wait for deinit to actually be effective */
        // while (0U == ((WDOG1->CS) & WDOG_CS_RCS_MASK))
        //     ;
    }

    if (enabled)
    {
        ewm_config_t config;
        EWM_GetDefaultConfig(&config);

        config.compareHighValue = MIN(timeout_ms / 250, 0xff);
        config.prescaler        = 250;
        /* get a warning before reset (log message) */
        config.enableInterrupt = true;

        EWM_Init(EWM0, &config);
        NVIC_SetPriority(EWM_IRQn, APP_WDT_IRQ_PRIO);
        NVIC_EnableIRQ(EWM_IRQn);
        EWM_EnableInterrupts(EWM0, kEWM_InterruptEnable);

        /* enable PMIC WDOG_B reset */
        UPOWER_SetPmicReg(8 /* RESET_CTRL */, 0xa0 /* WDOG_B_CFG = 10b | PMIC_RST_CFG = 10b*/);

        /* pre-create timer for light sleep */
        wdogSuspendPingTimer =
            xTimerCreate("wdogSuspendPingTimer", APP_MS2TICK(timeout_ms / 2), pdFALSE, NULL, wdog_suspend_ping_timer);
    }
    wdogTimeout = enabled ? timeout_ms : 0;

    return SRTM_Status_Success;
}

void APP_WDOG_ResetLog(void)
{
    /* used when linux boots to print a message on first linux ping as well */
    wdogFirstPingLogged = false;
}

void APP_WDOG_Suspend(void)
{
    if (wdogTimeout == 0)
        return;

    if (sleepWithLinux == LPM_PowerModeActive)
    {
        /* take over in m33 */
        PRINTF("Starting m33 wdt timer\r\n");
        wdogFirstPingLogged = false;
        xTimerStart(wdogSuspendPingTimer, portMAX_DELAY);
    }
    else
    {
        PRINTF("disabling wdog side-effects\r\n");
        /* disable PMIC WDOG_B reset */
        UPOWER_SetPmicReg(8 /* RESET_CTRL */, 0x20 /* WDOG_B_CFG = 00b | PMIC_RST_CFG = 10b*/);
        EWM_DisableInterrupts(EWM0, kEWM_InterruptEnable);
    }
}

void APP_WDOG_Resume(void)
{
    if (wdogTimeout == 0)
        return;

    PRINTF("wdog resume\r\n");
    wdogFirstPingLogged = false;
    EWM_Refresh(EWM0);
    EWM_EnableInterrupts(EWM0, kEWM_InterruptEnable);
    /* enable PMIC WDOG_B reset */
    UPOWER_SetPmicReg(8 /* RESET_CTRL */, 0xa0 /* WDOG_B_CFG = 10b | PMIC_RST_CFG = 10b*/);
    /* disable autoping if it was active */
    xTimerStop(wdogSuspendPingTimer, portMAX_DELAY);
}

void APP_WDOG_InitService(void)
{
    wdogService = SRTM_WdogService_Create(wdog_enable, wdog_ping);
    SRTM_Dispatcher_RegisterService(disp, wdogService);
}

void APP_WDOG_uboot(uint32_t command)
{
    uint8_t subcommand = (command >> 8) & 0xff;

    switch (subcommand)
    {
        case UBOOT_WDOG_INIT:
        {
            uint32_t timeout;

            timeout = uboot_recv();
            wdog_enable(true, timeout);
            MU_SendMsg(MU0_MUA, 0, 0);
        }
        break;
        case UBOOT_WDOG_PING:
            wdog_ping();
            MU_SendMsg(MU0_MUA, 0, 0);
            break;
        default:
            MU_SendMsg(MU0_MUA, 0, EINVAL);
            break;
    }
}
