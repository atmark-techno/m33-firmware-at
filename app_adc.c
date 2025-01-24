/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_iomuxc.h"
#include "fsl_reset.h"
#include "fsl_lpadc.h"

#include "app_srtm_internal.h"
#include "srtm_adc_service.h"

#include "build_bug.h"

#define ADC_MAX_PORTS 4
#define ADC_MAX 2

/* forward declarations to use in adcAdapter */
static srtm_status_t adc_get(uint8_t idx, uint16_t *value);
static srtm_status_t adc_init(uint8_t idx, struct srtm_adc_init_payload *init);

/* global state */
static struct adc_handle *adcHandles[ADC_MAX_PORTS];
static bool adcInit[ADC_MAX];
static srtm_service_t adcService;
static struct _srtm_adc_adapter adcAdapter = {
    .get  = adc_get,
    .init = adc_init,
};

/**********************************************************
 * SRTM callbacks
 *********************************************************/

static srtm_status_t adc_get(uint8_t idx, uint16_t *value)
{
    lpadc_conv_result_t resultConfig;

    if (idx > ARRAY_SIZE(adcHandles))
    {
        PRINTF("adc_get called with idx %d > %d\r\n", idx, ARRAY_SIZE(adcHandles));
        return kStatus_Fail;
    }
    struct adc_handle *handle = adcHandles[idx];
    if (!handle)
    {
        PRINTF("adc_get called with non-init'd adc %d\r\n", idx);
        return kStatus_Fail;
    }

    LPADC_DoSoftwareTrigger(handle->base, 1 << idx);
    LPADC_GetConvResultBlocking(handle->base, &resultConfig);
    *value = resultConfig.convValue >> 3U;

    return kStatus_Success;
}

int adc_init_device(struct adc_handle *handle, uint8_t idx)
{
    int adcIdx;

    switch ((uint32_t)handle->base)
    {
        case ADC0_BASE:
            adcIdx = 0;
            if (!adcInit[adcIdx])
            {
                CLOCK_SetIpSrc(kCLOCK_Adc0, kCLOCK_Pcc0BusIpSrcCm33Bus);
                RESET_PeripheralReset(kRESET_Adc0);
            }
            break;
        case ADC1_BASE:
            adcIdx = 1;
            if (!adcInit[adcIdx])
            {
                CLOCK_SetIpSrc(kCLOCK_Adc1, kCLOCK_Pcc1BusIpSrcCm33Bus);
                RESET_PeripheralReset(kRESET_Adc1);
            }
            break;
        default:
            return kStatus_InvalidArgument;
    }

    /* only init ADC once per ADC */
    if (!adcInit[adcIdx])
    {
        lpadc_config_t config;
        LPADC_GetDefaultConfig(&config);
        config.enableAnalogPreliminary = true;
        LPADC_Init(handle->base, &config);
        adcInit[adcIdx] = true;
    }

    lpadc_conv_command_config_t commandConfig;
    LPADC_GetDefaultConvCommandConfig(&commandConfig);
    commandConfig.channelNumber       = handle->chan;
    commandConfig.sampleChannelMode   = handle->side;
    commandConfig.sampleScaleMode     = handle->scale;
    commandConfig.hardwareAverageMode = handle->average;
    LPADC_SetConvCommandConfig(handle->base, idx + 1, &commandConfig);

    lpadc_conv_trigger_config_t triggerConfig;
    LPADC_GetDefaultConvTriggerConfig(&triggerConfig);
    triggerConfig.targetCommandId       = idx + 1;
    triggerConfig.enableHardwareTrigger = false;
    LPADC_SetConvTriggerConfig(handle->base, idx, &triggerConfig);

    return 0;
}

srtm_status_t adc_init(uint8_t idx, struct srtm_adc_init_payload *init)
{
    /* We use index as command id / trigger id; this will need rework if we use more than 4:
     * - for command id, 1-15 are usable
     * - for trigger id, 0-3 are usable
     * - (note this is per ADC so a first improvement would be to count per ADC, then
     *   we'd need to disable/re-enable triggers as appropriate...)
     */
    BUILD_BUG_ON(ADC_MAX_PORTS > 4);

    if (idx >= ADC_MAX_PORTS)
    {
        PRINTF("adc %d: index %d too big\r\n", idx, idx);
        return kStatus_Fail;
    }
    if (adcHandles[idx])
    {
        PRINTF("adc %d: index %d already setup\r\n", idx, idx);
        return kStatus_Fail;
    }

    ADC_Type *adc_base;
    switch (init->adc_index)
    {
        case 0:
            adc_base = ADC0;
            break;
        case 1:
            adc_base = ADC1;
            break;
        default:
            BUILD_BUG_ON(ADC_MAX != 2);
            PRINTF("adc %d: adc_index %d unsupported\r\n", idx, init->adc_index);
            return kStatus_Fail;
    }

    PRINTF("ADC %d init: ADC %x, chan %d, side %d, scale %d, average %d\r\n", idx, init->adc_index, init->adc_chan,
           init->adc_side, init->adc_scale, init->adc_average);

    struct adc_handle *handle = pvPortMalloc(sizeof(handle));
    if (!handle)
        return kStatus_Fail;

    handle->base    = adc_base;
    handle->chan    = init->adc_chan;
    handle->side    = init->adc_side;
    handle->scale   = init->adc_scale;
    handle->average = init->adc_average;

    int rc = adc_init_device(handle, idx);
    if (rc)
    {
        vPortFree(handle);
        return rc;
    }

    adcHandles[idx] = handle;
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

    /* reset and re-init */
    memset(&adcInit, 0, sizeof(adcInit));

    for (i = 0; i < ADC_MAX_PORTS; i++)
    {
        if (!adcHandles[i])
            continue;

        adc_init_device(adcHandles[i], i);
    }
}
