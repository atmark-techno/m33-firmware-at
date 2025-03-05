/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

#include "srtm_service.h"
#include "fsl_lpadc.h"

/*!
 * @addtogroup srtm_service
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** @brief Switch to disable ADC service debugging messages. */
#ifndef SRTM_ADC_SERVICE_DEBUG_OFF
#define SRTM_ADC_SERVICE_DEBUG_OFF (0)
#endif

#if SRTM_ADC_SERVICE_DEBUG_OFF
#undef SRTM_DEBUG_VERBOSE_LEVEL
#define SRTM_DEBUG_VERBOSE_LEVEL SRTM_DEBUG_VERBOSE_NONE
#endif

struct adc_handle
{
    ADC_Type *base;
    uint8_t chan;
    lpadc_sample_channel_mode_t side;
    lpadc_sample_scale_mode_t scale;
    lpadc_hardware_average_mode_t average;
};

typedef struct _srtm_adc_adapter *srtm_adc_adapter_t;

SRTM_PACKED_BEGIN struct srtm_adc_init_payload
{
    uint8_t adc_index;
    uint8_t adc_chan;
    uint8_t adc_side;
    uint8_t adc_scale;
    uint8_t adc_average;
} SRTM_PACKED_END;

struct _srtm_adc_adapter
{
    srtm_status_t (*get)(uint8_t idx, uint16_t *value);
    srtm_status_t (*init)(uint8_t idx, struct srtm_adc_init_payload *init);
};

/**
 * @brief SRTM ADC payload structure
 */
SRTM_PACKED_BEGIN struct _srtm_adc_payload
{
    uint8_t requestID;
    uint8_t idx;
    SRTM_PACKED_BEGIN union
    {
        /* get has no argument other than idx */
        struct srtm_adc_init_payload init;
        SRTM_PACKED_BEGIN struct
        {
            uint8_t retCode;
            uint16_t value;
        } SRTM_PACKED_END response;
    } SRTM_PACKED_END;
} SRTM_PACKED_END;

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create ADC service.
 *
 * @param adapter ADC adapter to handle real adc operations.
 * @return SRTM service handle on success and NULL on failure.
 */
srtm_service_t SRTM_AdcService_Create(srtm_adc_adapter_t adapter);

/*!
 * @brief Destroy ADC service.
 *
 * @param service SRTM service to destroy.
 */
void SRTM_AdcService_Destroy(srtm_service_t service);

#ifdef __cplusplus
}
#endif

/*! @} */
