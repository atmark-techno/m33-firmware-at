/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_iomuxc.h"
#include "fsl_reset.h"
#include "fsl_dac12.h"

#include "app_srtm_internal.h"
#include "srtm_dac_service.h"

#include "build_bug.h"

#define DAC_MAX_PORTS 2

/* forward declarations to use in dacAdapter */
static srtm_status_t dac_get(uint8_t idx, uint32_t *value);
static srtm_status_t dac_set(uint8_t idx, uint32_t value);
static srtm_status_t dac_init(uint8_t idx, struct srtm_dac_init_data *init);

/* global state */
static struct dac_handle *dacHandles[DAC_MAX_PORTS];
static srtm_service_t dacService;
static struct _srtm_dac_adapter dacAdapter = {
    .get  = dac_get,
    .set  = dac_set,
    .init = dac_init,
};

/**********************************************************
 * SRTM callbacks
 *********************************************************/

static srtm_status_t dac_get(uint8_t idx, uint32_t *value)
{
    if (idx > ARRAY_SIZE(dacHandles))
    {
        PRINTF("dac_get called with idx %d > %d\r\n", idx, ARRAY_SIZE(dacHandles));
        return kStatus_Fail;
    }
    struct dac_handle *handle = dacHandles[idx];
    if (!handle)
    {
        PRINTF("dac_get called with non-init'd dac %d\r\n", idx);
        return kStatus_Fail;
    }

    /* The DAC Data Register cannot be read, so the saved data is returned. */
    *value = handle->lastData;

    return kStatus_Success;
}

static srtm_status_t dac_set(uint8_t idx, uint32_t value)
{
    if (idx > ARRAY_SIZE(dacHandles))
    {
        PRINTF("dac_set called with idx %d > %d\r\n", idx, ARRAY_SIZE(dacHandles));
        return kStatus_Fail;
    }
    struct dac_handle *handle = dacHandles[idx];
    if (!handle)
    {
        PRINTF("dac_set called with non-init'd dac %d\r\n", idx);
        return kStatus_Fail;
    }

    if (value & (uint32_t)~DAC_DATA_DATA0_MASK)
        PRINTF("dac_set: value 0x%x is too large so I round it down.\r\n", value);

    DAC12_SetData(handle->base, value);
    handle->lastData = value;

    return kStatus_Success;
}

static int dac_init_device(uint8_t idx, struct dac_handle *handle)
{
    DAC_Type *base = handle->base;
    ;
    dac12_config_t dacConfigStruct;

    switch (idx)
    {
        case 0: /* DAC0 */
            CLOCK_SetIpSrc(kCLOCK_Dac0, kCLOCK_Pcc0BusIpSrcLpo);
            RESET_PeripheralReset(kRESET_Dac0);
            break;
        case 1: /* DAC1 */
            CLOCK_SetIpSrc(kCLOCK_Dac1, kCLOCK_Pcc0BusIpSrcLpo);
            RESET_PeripheralReset(kRESET_Dac1);
            break;
        default:
            return kStatus_InvalidArgument;
    }

    DAC12_GetDefaultConfig(&dacConfigStruct);
    dacConfigStruct.fifoWatermarkLevel     = 4U;
    dacConfigStruct.referenceVoltageSource = handle->referenceVoltageSource;
    dacConfigStruct.referenceCurrentSource = handle->referenceCurrentSource;
    dacConfigStruct.speedMode              = handle->speedMode;

    DAC12_Init(base, &dacConfigStruct);
    DAC12_Enable(base, true); /* Enable output. */

    return 0;
}

srtm_status_t dac_init(uint8_t idx, struct srtm_dac_init_data *init)
{
    BUILD_BUG_ON(DAC_MAX_PORTS > 2);

    if (!init)
    {
        PRINTF("dac: no initialization data\r\n");
        return kStatus_Fail;
    }

    if (idx >= DAC_MAX_PORTS)
    {
        PRINTF("dac: index %d too big\r\n", idx);
        return kStatus_Fail;
    }
    if (dacHandles[idx])
    {
        PRINTF("dac: index %d already setup\r\n", idx);
        return kStatus_Fail;
    }

    DAC_Type *dac_base;
    switch (idx)
    {
        case 0:
            dac_base = DAC0;
            break;
        case 1:
            dac_base = DAC1;
            break;
        default:
            PRINTF("dac: index %d unsupported\r\n", idx);
            return kStatus_Fail;
    }

    PRINTF("DAC init: DAC %x, vref %d, cref %d, speed %d\r\n", idx, init->dac_vref, init->dac_cref, init->dac_speed);

    struct dac_handle *handle = pvPortMalloc(sizeof(*handle));
    if (!handle)
        return kStatus_Fail;

    handle->base                   = dac_base;
    handle->referenceVoltageSource = init->dac_vref;
    handle->referenceCurrentSource = init->dac_cref;
    handle->speedMode              = init->dac_speed;
    handle->lastData               = 0; /* Initial value of DAC Data Register */

    int rc = dac_init_device(idx, handle);
    if (rc)
    {
        vPortFree(handle);
        return rc;
    }

    dacHandles[idx] = handle;
    return kStatus_Success;
}

/**********************************************************
 * init/PM hooks called from APP_SRTM
 *********************************************************/

void APP_DAC_InitService(void)
{
    /* Create and register dac service */
    dacService = SRTM_DacService_Create(&dacAdapter);
    SRTM_Dispatcher_RegisterService(disp, dacService);
}
