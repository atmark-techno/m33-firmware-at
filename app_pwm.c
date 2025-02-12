/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_reset.h"

#include "app_srtm_internal.h"

#include "board.h"
#include "srtm_pwm_adapter.h"
#include "srtm_pwm_service.h"

static srtm_service_t pwmService;

/* pwmHandles must strictly follow TPM instances. If you don't provide service for some TPM instance,
 * set the corresponding handle to NULL. */
static HAL_PWM_HANDLE_DEFINE(pwmHandle0);
static hal_pwm_handle_t pwmHandles[2] = { (hal_pwm_handle_t)pwmHandle0, NULL };

static void pwm_init_device(void)
{
    CLOCK_SetIpSrcDiv(kCLOCK_Tpm0, kCLOCK_Pcc1BusIpSrcCm33Bus, 1U, 0U);
    RESET_PeripheralReset(kRESET_Tpm0);
    HAL_PwmInit(pwmHandles[0], 0U, CLOCK_GetTpmClkFreq(0U));
}

/**********************************************************
 * init/PM hooks called from APP_SRTM
 *********************************************************/
void APP_PWM_InitService(void)
{
    pwm_init_device();
    srtm_pwm_adapter_t pwmAdapter = SRTM_PwmAdapter_Create(pwmHandles, ARRAY_SIZE(pwmHandles));
    assert(pwmAdapter);

    /* Create and register pwm service */
    pwmService = SRTM_PwmService_Create(pwmAdapter);
    SRTM_Dispatcher_RegisterService(disp, pwmService);
}

void APP_PWM_Resume(void)
{
    pwm_init_device();
}
