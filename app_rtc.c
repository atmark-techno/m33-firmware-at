/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_reset.h"

#include "app_srtm.h"
#include "app_srtm_internal.h"
#include "app_rtc.h"

#include "board.h"
#include "srtm_rtc_adapter.h"
#include "srtm_rtc_service.h"

static srtm_rtc_adapter_t rtcAdapter;
static srtm_service_t rtcService;

static HAL_RTC_HANDLE_DEFINE(rtcHandle);

void APP_RTC_EarlyInit(void)
{
    HAL_RtcInit(rtcHandle, 0);
}

static void APP_RTC_Init(void)
{
    NVIC_ClearPendingIRQ(BBNSM_IRQn);
    NVIC_SetPriority(BBNSM_IRQn, APP_BBNSM_IRQ_PRIO);
    EnableIRQ(BBNSM_IRQn);
}

void APP_RTC_NotifyAlarm(void)
{
    SRTM_RtcAdapter_NotifyAlarm(rtcAdapter);
}

/**********************************************************
 * init/PM hooks called from APP_SRTM
 *********************************************************/
void APP_RTC_InitService(void)
{
    rtcAdapter = SRTM_RtcAdapter_Create(rtcHandle);
    assert(rtcAdapter);

    /* Initialize regardless of whether it is used or not. There is no
     * difference in current consumption. */
    APP_RTC_Init();

    /* Create and register rtc service */
    rtcService = SRTM_RtcService_Create(rtcAdapter);
    SRTM_Dispatcher_RegisterService(disp, rtcService);
}
