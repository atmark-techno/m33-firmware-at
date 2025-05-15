/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

#include "srtm_service.h"
#include "fsl_dac12.h"

/*!
 * @addtogroup srtm_service
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** @brief Switch to disable DAC service debugging messages. */
#ifndef SRTM_DAC_SERVICE_DEBUG_OFF
#define SRTM_DAC_SERVICE_DEBUG_OFF (0)
#endif

#if SRTM_DAC_SERVICE_DEBUG_OFF
#undef SRTM_DEBUG_VERBOSE_LEVEL
#define SRTM_DEBUG_VERBOSE_LEVEL SRTM_DEBUG_VERBOSE_NONE
#endif

struct dac_handle
{
    DAC_Type *base;
    dac12_reference_voltage_source_t referenceVoltageSource;
    dac12_reference_current_source_t referenceCurrentSource;
    dac12_speed_mode_t speedMode;
    uint16_t lastData;
};

typedef struct _srtm_dac_adapter *srtm_dac_adapter_t;

SRTM_PACKED_BEGIN struct srtm_dac_init_data
{
    uint8_t dac_vref;
    uint8_t dac_cref;
    uint8_t dac_speed;
} SRTM_PACKED_END;

struct _srtm_dac_adapter
{
    srtm_status_t (*get)(uint8_t idx, uint32_t *value);
    srtm_status_t (*set)(uint8_t idx, uint32_t value);
    srtm_status_t (*init)(uint8_t idx, struct srtm_dac_init_data *init);
};

/**
 * @brief SRTM DAC payload structure
 */
SRTM_PACKED_BEGIN struct _srtm_dac_payload
{
    uint8_t requestID;
    uint8_t idx;
    union
    {
        uint8_t reserved; /* unused in request packet */
        uint8_t retCode;  /* used in response packet */
    };
    union
    {
        uint32_t value;
        struct srtm_dac_init_data initData;
    };
} SRTM_PACKED_END;

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create DAC service.
 *
 * @param adapter DAC adapter to handle real dac operations.
 * @return SRTM service handle on success and NULL on failure.
 */
srtm_service_t SRTM_DacService_Create(srtm_dac_adapter_t adapter);

/*!
 * @brief Destroy DAC service.
 *
 * @param service SRTM service to destroy.
 */
void SRTM_DacService_Destroy(srtm_service_t service);

#ifdef __cplusplus
}
#endif

/*! @} */
