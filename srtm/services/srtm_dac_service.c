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
#include "srtm_dac_service.h"
#include "srtm_message.h"

/* uncomment to debug this service */
//#undef SRTM_DEBUG_VERBOSE_LEVEL
//#define SRTM_DEBUG_VERBOSE_LEVEL SRTM_DEBUG_VERBOSE_DEBUG

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Protocol definition */
#define SRTM_DAC_CATEGORY (0xf5)

#define SRTM_DAC_VERSION (0x0100U)

#define SRTM_DAC_RETURN_CODE_SUCCESS (0x0U)
#define SRTM_DAC_RETURN_CODE_FAIL (0x1U)
#define SRTM_DAC_RETURN_CODE_UNSUPPORTED (0x2U)

#define SRTM_DAC_CMD_GET (0x0U)
#define SRTM_DAC_CMD_SET (0x1U)
#define SRTM_DAC_CMD_INIT (0x2U)

/* Service handle */
typedef struct _srtm_dac_service
{
    struct _srtm_service service;
    srtm_dac_adapter_t adapter;
} *srtm_dac_service_t;

/*******************************************************************************
 * Code
 ******************************************************************************/
/* Both request and notify are called from SRTM dispatcher context */
static srtm_status_t SRTM_DacService_Request(srtm_service_t service, srtm_request_t request)
{
    srtm_status_t status;
    srtm_dac_service_t handle  = (srtm_dac_service_t)(void *)service;
    srtm_dac_adapter_t adapter = handle->adapter;
    srtm_channel_t channel;
    uint8_t command;
    uint32_t payloadLen;
    srtm_response_t response;
    struct _srtm_dac_payload *dacReq;
    struct _srtm_dac_payload *dacResp;
    uint32_t value;

    assert(adapter);
    assert(service->dispatcher);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    channel    = SRTM_CommMessage_GetChannel(request);
    command    = SRTM_CommMessage_GetCommand(request);
    dacReq     = (struct _srtm_dac_payload *)(void *)SRTM_CommMessage_GetPayload(request);
    payloadLen = SRTM_CommMessage_GetPayloadLen(request);

    response = SRTM_Response_Create(channel, SRTM_DAC_CATEGORY, SRTM_DAC_VERSION, command,
                                    (uint16_t)sizeof(struct _srtm_dac_payload));
    if (response == NULL)
    {
        return SRTM_Status_OutOfMemory;
    }

    dacResp = (struct _srtm_dac_payload *)(void *)SRTM_CommMessage_GetPayload(response);
    /* don't leak uninitialized values to linux */
    memset(dacResp, 0, sizeof(*dacResp));

    status = SRTM_Service_CheckVersion(service, request, SRTM_DAC_VERSION);
    if ((status != SRTM_Status_Success) || (dacReq == NULL) ||
        (payloadLen < offsetof(struct _srtm_dac_payload, retCode)))
    {
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s format error %d / %d / %" PRIu32 " / %d!\r\n", __func__, status,
                           dacReq == NULL, payloadLen, sizeof(struct _srtm_dac_payload));
        dacResp->retCode = SRTM_DAC_RETURN_CODE_UNSUPPORTED;
        goto out;
    }

    dacResp->requestID = dacReq->requestID;
    dacResp->idx       = dacReq->idx;
    switch (command)
    {
        case SRTM_DAC_CMD_GET:
            if (!adapter->get)
                goto unsupported;

            SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_DEBUG, "DAC %d get\r\n", dacReq->idx);
            status = adapter->get(dacReq->idx, &value);
            SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_DEBUG, "Got DAC value %d\r\n", value);
            if (status == SRTM_Status_Success)
            {
                memcpy(&dacResp->value, &value, sizeof(value));
                dacResp->retCode = SRTM_DAC_RETURN_CODE_SUCCESS;
            }
            else
            {
                dacResp->retCode = SRTM_DAC_RETURN_CODE_FAIL;
            }
            break;
        case SRTM_DAC_CMD_SET:
            if (!adapter->set)
                goto unsupported;

            SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_DEBUG, "DAC %d set\r\n", dacReq->idx);
            status = adapter->set(dacReq->idx, dacReq->value);
            SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_DEBUG, "Sot DAC value %d\r\n", dacReq->value);
            if (status == SRTM_Status_Success)
            {
                dacResp->retCode = SRTM_DAC_RETURN_CODE_SUCCESS;
            }
            else
            {
                dacResp->retCode = SRTM_DAC_RETURN_CODE_FAIL;
            }
            break;
        case SRTM_DAC_CMD_INIT:
            if (!adapter->init)
                goto unsupported;

            if (payloadLen < sizeof(*dacReq))
            {
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "DAC %d size too small %" PRIu32 "\r\n", dacReq->idx,
                                   payloadLen);
                dacResp->retCode = SRTM_DAC_RETURN_CODE_FAIL;
                goto out;
            }
            {
                struct srtm_dac_init_data init;
                memcpy(&init, &dacReq->initData, sizeof(init));
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_DEBUG, "DAC %d init\r\n", dacReq->idx);
                status = adapter->init(dacReq->idx, &init);
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_DEBUG, "DAC %d init status: %d\r\n", dacReq->idx, status);
            }
            if (status == SRTM_Status_Success)
            {
                dacResp->value   = value;
                dacResp->retCode = SRTM_DAC_RETURN_CODE_SUCCESS;
            }
            else
            {
                dacResp->retCode = SRTM_DAC_RETURN_CODE_FAIL;
            }
            break;
        default:
        unsupported:
            SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported\r\n", __func__, command);
            dacResp->retCode = SRTM_DAC_RETURN_CODE_UNSUPPORTED;
            break;
    }

out:
    return SRTM_Dispatcher_DeliverResponse(service->dispatcher, response);
}

static srtm_status_t SRTM_DacService_Notify(srtm_service_t service, srtm_notification_t notif)
{
    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported\r\n", __func__,
                       SRTM_CommMessage_GetCommand(notif));

    return SRTM_Status_ServiceNotFound;
}

srtm_service_t SRTM_DacService_Create(srtm_dac_adapter_t adapter)
{
    srtm_dac_service_t handle;

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    handle = (srtm_dac_service_t)SRTM_Heap_Malloc(sizeof(struct _srtm_dac_service));
    assert(handle);

    handle->adapter = adapter;

    SRTM_List_Init(&handle->service.node);
    handle->service.dispatcher = NULL;
    handle->service.category   = SRTM_DAC_CATEGORY;
    handle->service.destroy    = SRTM_DacService_Destroy;
    handle->service.request    = SRTM_DacService_Request;
    handle->service.notify     = SRTM_DacService_Notify;

    return &handle->service;
}

void SRTM_DacService_Destroy(srtm_service_t service)
{
    srtm_dac_service_t handle = (srtm_dac_service_t)(void *)service;

    assert(service);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    /* Service must be unregistered from dispatcher before destroy */
    assert(SRTM_List_IsEmpty(&service->node));

    SRTM_Heap_Free(handle);
}
