/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <string.h>

#include "srtm_heap.h"
#include "srtm_dispatcher.h"
#include "srtm_service.h"
#include "srtm_service_struct.h"
#include "srtm_adc_service.h"
#include "srtm_message.h"

/* uncomment to debug this service */
//#undef SRTM_DEBUG_VERBOSE_LEVEL
//#define SRTM_DEBUG_VERBOSE_LEVEL SRTM_DEBUG_VERBOSE_DEBUG

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Protocol definition */
#define SRTM_ADC_CATEGORY (0x0bU)

#define SRTM_ADC_VERSION (0x0100U)

#define SRTM_ADC_RETURN_CODE_SUCCESS (0x0U)
#define SRTM_ADC_RETURN_CODE_FAIL (0x1U)
#define SRTM_ADC_RETURN_CODE_UNSUPPORTED (0x2U)

#define SRTM_ADC_CMD_GET (0x0U)

/* Service handle */
typedef struct _srtm_adc_service
{
    struct _srtm_service service;
    srtm_adc_adapter_t adapter;
} * srtm_adc_service_t;

/*******************************************************************************
 * Code
 ******************************************************************************/
/* Both request and notify are called from SRTM dispatcher context */
static srtm_status_t SRTM_AdcService_Request(srtm_service_t service, srtm_request_t request)
{
    srtm_status_t status;
    srtm_adc_service_t handle  = (srtm_adc_service_t)(void *)service;
    srtm_adc_adapter_t adapter = handle->adapter;
    srtm_channel_t channel;
    uint8_t command;
    uint32_t payloadLen;
    srtm_response_t response;
    struct _srtm_adc_payload *adcReq;
    struct _srtm_adc_payload *adcResp;
    uint16_t value;

    assert(adapter);
    assert(service->dispatcher);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    channel    = SRTM_CommMessage_GetChannel(request);
    command    = SRTM_CommMessage_GetCommand(request);
    adcReq     = (struct _srtm_adc_payload *)(void *)SRTM_CommMessage_GetPayload(request);
    payloadLen = SRTM_CommMessage_GetPayloadLen(request);

    response = SRTM_Response_Create(channel, SRTM_ADC_CATEGORY, SRTM_ADC_VERSION, command,
                                    (uint16_t)sizeof(struct _srtm_adc_payload));
    if (response == NULL)
    {
        return SRTM_Status_OutOfMemory;
    }

    adcResp = (struct _srtm_adc_payload *)(void *)SRTM_CommMessage_GetPayload(response);
    /* don't leak uninitialized values to linux */
    memset(adcResp, 0, sizeof(*adcResp));

    status = SRTM_Service_CheckVersion(service, request, SRTM_ADC_VERSION);
    if ((status != SRTM_Status_Success) || (adcReq == NULL) || (payloadLen != sizeof(struct _srtm_adc_payload)))
    {
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s format error %d / %d / %d / %d!\r\n", __func__, status,
                           adcReq == NULL, payloadLen, sizeof(struct _srtm_adc_payload));
        adcResp->retCode = SRTM_ADC_RETURN_CODE_UNSUPPORTED;
    }
    else if (adcReq->idx >= adapter->handles_count)
    {
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "ADC cmd 0x%x idx out of range (0x%x / 0x%x)\r\n", command,
                           adcReq->idx, adapter->handles_count);
        adcResp->retCode = SRTM_ADC_RETURN_CODE_FAIL;
    }
    else
    {
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "SRTM receive ADC request: cmd = 0x%x, idx = 0x%x\r\n", command,
                           adcReq->idx);
        adcResp->requestID = adcReq->requestID;
        switch (command)
        {
            case SRTM_ADC_CMD_GET:
                assert(adapter->get);

                status = adapter->get(&adapter->handles[adcReq->idx], adcReq->idx, &value);
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_DEBUG, "Got ADC value %d\r\n", value);
                if (status == SRTM_Status_Success)
                {
                    adcResp->value   = value;
                    adcResp->retCode = SRTM_ADC_RETURN_CODE_SUCCESS;
                }
                else
                {
                    adcResp->retCode = SRTM_ADC_RETURN_CODE_FAIL;
                }
                break;
            default:
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported\r\n", __func__, command);
                adcResp->retCode = SRTM_ADC_RETURN_CODE_UNSUPPORTED;
                break;
        }
    }

    return SRTM_Dispatcher_DeliverResponse(service->dispatcher, response);
}

static srtm_status_t SRTM_AdcService_Notify(srtm_service_t service, srtm_notification_t notif)
{
    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported\r\n", __func__,
                       SRTM_CommMessage_GetCommand(notif));

    return SRTM_Status_ServiceNotFound;
}

srtm_service_t SRTM_AdcService_Create(srtm_adc_adapter_t adapter)
{
    srtm_adc_service_t handle;

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    handle = (srtm_adc_service_t)SRTM_Heap_Malloc(sizeof(struct _srtm_adc_service));
    assert(handle);

    handle->adapter = adapter;

    SRTM_List_Init(&handle->service.node);
    handle->service.dispatcher = NULL;
    handle->service.category   = SRTM_ADC_CATEGORY;
    handle->service.destroy    = SRTM_AdcService_Destroy;
    handle->service.request    = SRTM_AdcService_Request;
    handle->service.notify     = SRTM_AdcService_Notify;

    return &handle->service;
}

void SRTM_AdcService_Destroy(srtm_service_t service)
{
    srtm_adc_service_t handle = (srtm_adc_service_t)(void *)service;

    assert(service);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    /* Service must be unregistered from dispatcher before destroy */
    assert(SRTM_List_IsEmpty(&service->node));

    SRTM_Heap_Free(handle);
}
