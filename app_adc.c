/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "app_srtm_internal.h"
#include "srtm_adc_service.h"

#include "fsl_iomuxc.h"
#include "fsl_reset.h"
#include "fsl_lpadc.h"

#define ADC_MAX_PORTS 4

static srtm_status_t APP_ADC_Get(struct adc_handle *handle, size_t idx, uint16_t *value);

static srtm_service_t adcService;
static struct adc_handle adcHandles[] = {
    {
        .chan    = 5,
        .side    = kLPADC_SampleChannelSingleEndSideB,
        .scale   = kLPADC_SamplePartScale,
        .average = kLPADC_HardwareAverageCount4,
        .cmdid   = 1,
        .pinmux  = { IOMUXC_PTA24_ADC1_CH5B },
    },
};
static struct _srtm_adc_adapter adcAdapter = {
    .get           = APP_ADC_Get,
    .handles       = adcHandles,
    .handles_count = sizeof(adcHandles) / sizeof(*adcHandles),
};

/**********************************************************
 * SRTM callbacks
 *********************************************************/

static srtm_status_t APP_ADC_Get(struct adc_handle *handle, size_t idx, uint16_t *value)
{
    lpadc_conv_result_t resultConfig;

    if (idx > ARRAY_SIZE(adcHandles))
    {
        PRINTF("APP_ADC_Get called with idx %d > %d\r\n", idx, ARRAY_SIZE(adcHandles));
        return kStatus_Fail;
    }

    /* reset pinmux everytime to avoid surprises: if someone used the pin as gpio or
     * something in between this could get an invalid value */
    IOMUXC_SetPinMux(handle[idx].pinmux[0], handle[idx].pinmux[1], handle[idx].pinmux[2], handle[idx].pinmux[3],
                     handle[idx].pinmux[4], 0);
    IOMUXC_SetPinConfig(handle[idx].pinmux[0], handle[idx].pinmux[1], handle[idx].pinmux[2], handle[idx].pinmux[3],
                        handle[idx].pinmux[4], 0);

    LPADC_DoSoftwareTrigger(ADC1, 1 << idx);
    LPADC_GetConvResultBlocking(ADC1, &resultConfig);
    *value = resultConfig.convValue >> 3U;

    return kStatus_Success;
}

/**********************************************************
 * init/PM hooks called from APP_SRTM
 *********************************************************/

void APP_ADC_InitService(void)
{
    APP_ADC_Resume();
    /* Create and register adc service */
    adcService = SRTM_AdcService_Create(&adcAdapter);
    SRTM_Dispatcher_RegisterService(disp, adcService);
}

void APP_ADC_Resume(void)
{
    size_t i;

    CLOCK_SetIpSrc(kCLOCK_Adc1, kCLOCK_Pcc1BusIpSrcCm33Bus);
    RESET_PeripheralReset(kRESET_Adc1);

    lpadc_config_t config;
    LPADC_GetDefaultConfig(&config);
    config.enableAnalogPreliminary = true;
    LPADC_Init(ADC1, &config);

    for (i = 0; i < adcAdapter.handles_count; i++)
    {
        lpadc_conv_command_config_t commandConfig;
        LPADC_GetDefaultConvCommandConfig(&commandConfig);
        commandConfig.channelNumber       = adcAdapter.handles[i].chan;
        commandConfig.sampleChannelMode   = adcAdapter.handles[i].side;
        commandConfig.sampleScaleMode     = adcAdapter.handles[i].scale;
        commandConfig.hardwareAverageMode = adcAdapter.handles[i].average;
        LPADC_SetConvCommandConfig(ADC1, adcAdapter.handles[i].cmdid, &commandConfig);

        lpadc_conv_trigger_config_t triggerConfig;
        LPADC_GetDefaultConvTriggerConfig(&triggerConfig);
        triggerConfig.targetCommandId       = adcAdapter.handles[i].cmdid;
        triggerConfig.enableHardwareTrigger = false;
        LPADC_SetConvTriggerConfig(ADC1, i, &triggerConfig);
    }
}
