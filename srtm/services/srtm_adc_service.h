/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __SRTM_ADC_SERVICE_H__
#define __SRTM_ADC_SERVICE_H__

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
    uint32_t chan;
    lpadc_sample_channel_mode_t side;
    lpadc_sample_scale_mode_t scale;
    lpadc_hardware_average_mode_t average;
    uint32_t cmdid;
};

typedef struct _srtm_adc_adapter *srtm_adc_adapter_t;

struct _srtm_adc_adapter
{
    srtm_status_t (*get)(struct adc_handle *handle, size_t idx, uint16_t *value);
    struct adc_handle *handles;
    size_t handles_count;
};

/**
 * @brief SRTM ADC payload structure
 */
SRTM_PACKED_BEGIN struct _srtm_adc_payload
{
    uint8_t requestID;
    uint8_t idx;
    uint8_t retCode; /* used in response packet */
    uint16_t value;
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

#endif /* __SRTM_ADC_SERVICE_H__ */
