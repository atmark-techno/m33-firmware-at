/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_reset.h"

#include "app_srtm_internal.h"
#include "app_pwm.h"

#include "board.h"
#include "srtm_pwm_adapter.h"
#include "srtm_pwm_service.h"

static srtm_pwm_adapter_t pwmAdapter;
static srtm_service_t pwmService;

/* pwmHandles must strictly follow TPM instances. If you don't provide service for some TPM instance,
 * set the corresponding handle to NULL. */
static HAL_PWM_HANDLE_DEFINE(pwmHandle0);
static HAL_PWM_HANDLE_DEFINE(pwmHandle1);
static HAL_PWM_HANDLE_DEFINE(pwmHandle2);
static HAL_PWM_HANDLE_DEFINE(pwmHandle3);
static hal_pwm_handle_t pwmHandles[4] = { pwmHandle0, pwmHandle1, pwmHandle2, pwmHandle3 };

/* Since TPM2 and TPM3 initialization must be performed after the TRDC register setting, which is
 * performed immediately after the handshake with U-boot. Therefore, this initialization function is
 * passed to the adapter as a handler rather than performed here. */
static void pwm_init_device(hal_pwm_handle_t *halPwmHandle, uint8_t chipId)
{
    switch (chipId)
    {
        case 0:
            /* 16-bit -> Max period = 6.55 ms, Min period = 100 ns */
            CLOCK_SetIpSrcDiv(kCLOCK_Tpm0, kCLOCK_Pcc1BusIpSrcCm33Bus, 7U, 0U);
            RESET_PeripheralReset(kRESET_Tpm0);
            HAL_PwmInit(halPwmHandle[0], 0U, CLOCK_GetTpmClkFreq(0U), true);
            break;
        case 1:
            /* 32-bit -> Max period = 53.7 s, Min period = 12.5 ns */
            CLOCK_SetIpSrcDiv(kCLOCK_Tpm1, kCLOCK_Pcc1BusIpSrcCm33Bus, 0U, 0U);
            RESET_PeripheralReset(kRESET_Tpm1);
            HAL_PwmInit(halPwmHandle[1], 1U, CLOCK_GetTpmClkFreq(1U), true);
            break;
        case 2:
            /* 32-bit -> Max period = 44.7 s, Min period = 10.4 ns */
            /* Mainly for setting TPM3_2CLK register (CLOCK_SetIpSrc) */
            CLOCK_SetIpSrc(kCLOCK_Tpm2, kCLOCK_FusionTpm2ClkSrcFusionDspBus);
            CLOCK_SetIpSrcDiv(kCLOCK_Tpm2, kCLOCK_Pcc2BusIpSrcFusionDspBus, 0U, 0U);
            RESET_PeripheralReset(kRESET_Tpm2);
            HAL_PwmInit(halPwmHandle[2], 2U, CLOCK_GetIpFreq(kCLOCK_Tpm2), true);
            break;
        case 3:
            /* 16-bit -> Max period = 5.46 ms, Min period = 83.3 ns */
            /* Mainly for setting TPM3_2CLK register (CLOCK_SetIpSrc) */
            CLOCK_SetIpSrc(kCLOCK_Tpm3, kCLOCK_FusionTpm3ClkSrcFusionDspBus);
            CLOCK_SetIpSrcDiv(kCLOCK_Tpm3, kCLOCK_Pcc2BusIpSrcFusionDspBus, 7U, 0U);
            RESET_PeripheralReset(kRESET_Tpm3);
            HAL_PwmInit(halPwmHandle[3], 3U, CLOCK_GetIpFreq(kCLOCK_Tpm3), true);
            break;
    }
}

/**********************************************************
 * init/PM hooks called from APP_SRTM
 *********************************************************/
void APP_PWM_InitService(void)
{
    pwmAdapter = SRTM_PwmAdapter_Create(pwmHandles, ARRAY_SIZE(pwmHandles), pwm_init_device);
    assert(pwmAdapter);

    /* Create and register pwm service */
    pwmService = SRTM_PwmService_Create(pwmAdapter);
    SRTM_Dispatcher_RegisterService(disp, pwmService);
}

void APP_PWM_Resume(void)
{
    SRTM_PWMAdapter_flag_init(pwmAdapter, 2, false);
    SRTM_PWMAdapter_flag_init(pwmAdapter, 3, false);
}
