/*
 * Copyright 2018-2021, NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include <assert.h>
#include <string.h>

#include "fsl_common.h"

#include "srtm_heap.h"
#include "srtm_list.h"
#include "srtm_dispatcher.h"
#include "srtm_service.h"
#include "srtm_service_struct.h"
#include "srtm_spi_service.h"
#include "srtm_message.h"
#include "srtm_message_struct.h"
#include "srtm_channel.h"
#include "srtm_channel_struct.h"
#include "rpmsg_lite.h"

/* uncomment to debug this service */
//#undef SRTM_DEBUG_VERBOSE_LEVEL
//#define SRTM_DEBUG_VERBOSE_LEVEL SRTM_DEBUG_VERBOSE_DEBUG

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Protocol definition */
#define SRTM_SPI_CATEGORY (0xf4U)
#define SRTM_SPI_VERSION (0x0100U)

static srtm_status_t SRTM_SPIService_Request(srtm_service_t service, srtm_request_t request);
static srtm_status_t SRTM_SPIService_Notify(srtm_service_t service, srtm_notification_t notif);

typedef struct _srtm_spi_service
{
    /* Bound service */
    struct _srtm_service service;

    /* Interfaces implemented for SPI */
    srtm_spi_init_t init;
    srtm_spi_transfer_t transfer;
} * srtm_spi_service_t;

static srtm_status_t SRTM_SPIService_Init(srtm_spi_service_t handle, uint8_t busID, uint8_t *buf, uint16_t len)
{
    struct srtm_spi_init_payload init;
    srtm_status_t status;

    /* allow bigger for forward compat */
    if (len < sizeof(init))
    {
        return SRTM_Status_InvalidParameter;
    }

    memcpy(&init, buf, sizeof(init));

    status = handle->init(busID, &init);
    return status;
}

srtm_service_t SRTM_SPIService_Create(srtm_spi_init_t init, srtm_spi_transfer_t transfer)
{
    srtm_spi_service_t handle;

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);
    handle = (srtm_spi_service_t)SRTM_Heap_Malloc(sizeof(struct _srtm_spi_service));
    assert(handle);

    handle->init     = init;
    handle->transfer = transfer;

    SRTM_List_Init(&handle->service.node);
    handle->service.dispatcher = NULL;
    handle->service.category   = SRTM_SPI_CATEGORY;
    handle->service.destroy    = SRTM_SPIService_Destroy;
    handle->service.request    = SRTM_SPIService_Request;
    handle->service.notify     = SRTM_SPIService_Notify;

    return &handle->service;
}

void SRTM_SPIService_Destroy(srtm_service_t service)
{
    srtm_spi_service_t handle = (srtm_spi_service_t)(void *)service;

    assert(service);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    /* Service must be unregistered from dispatcher before destroy */
    assert(SRTM_List_IsEmpty(&service->node));

    SRTM_Heap_Free(handle);
}

void SRTM_SPIService_Reset(srtm_service_t service, srtm_peercore_t core)
{
    assert(service);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);
}

srtm_status_t SRTM_SPIService_SendResponse(srtm_service_t service, srtm_response_t response, uint8_t retCode)
{
    struct _srtm_spi_payload *spiResp = (struct _srtm_spi_payload *)(void *)SRTM_CommMessage_GetPayload(response);

    spiResp->retCode = retCode;
    return SRTM_Dispatcher_DeliverResponse(service->dispatcher, response);
}

static srtm_status_t SRTM_SPIService_Request(srtm_service_t service, srtm_request_t request)
{
    srtm_spi_service_t handle = (srtm_spi_service_t)service;
    srtm_status_t status;
    srtm_channel_t channel;
    uint8_t retCode = SRTM_SPI_RETCODE_SUCCESS;
    uint8_t command;
    uint16_t responseLen = 0, requestLen = 0;
    uint32_t payloadLen;
    srtm_response_t response;
    struct _srtm_spi_payload *spiReq;
    struct _srtm_spi_payload *spiResp;

    assert(service->dispatcher);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    channel    = SRTM_CommMessage_GetChannel(request);
    command    = SRTM_CommMessage_GetCommand(request);
    spiReq     = (struct _srtm_spi_payload *)(void *)SRTM_CommMessage_GetPayload(request);
    payloadLen = SRTM_CommMessage_GetPayloadLen(request);
    (void)payloadLen; /* try to fix warning: variable 'payloadLen' set but not used */
    assert(spiReq);
    if (payloadLen >= sizeof(struct _srtm_spi_payload))
    {
        requestLen = spiReq->len;
    }
    switch (command)
    {
        case SRTM_SPI_CMD_TRANSFER:
            responseLen = requestLen;
            break;
        case SRTM_SPI_CMD_INIT:
            break;
    }
    if (payloadLen < sizeof(struct _srtm_spi_payload) + requestLen ||
        requestLen + sizeof(struct _srtm_spi_payload) + sizeof(struct _srtm_packet_head) > SRTM_DISPATCHER_MSG_MAX_LEN)
    {
        retCode     = SRTM_SPI_RETCODE_EINVAL;
        responseLen = 0;
    }

    status = SRTM_Service_CheckVersion(service, request, SRTM_SPI_VERSION);
    if (status != SRTM_Status_Success)
    {
        retCode     = SRTM_SPI_RETCODE_UNSUPPORTED;
        responseLen = 0;
    }

retry_alloc:
    response = SRTM_Response_Create(channel, SRTM_SPI_CATEGORY, SRTM_SPI_VERSION, command,
                                    sizeof(struct _srtm_spi_payload) + responseLen);
    if (response == NULL)
    {
        PRINTF("spi rpmsg out of memory (%d)\r\n", sizeof(struct _srtm_spi_payload) + responseLen);
        if (retCode == SRTM_SPI_RETCODE_SUCCESS)
        {
            // retry once with smaller buffer to signal error
            retCode     = SRTM_SPI_RETCODE_ENOMEM;
            responseLen = 0;
            goto retry_alloc;
        }
        return SRTM_Status_OutOfMemory;
    }

    spiResp = (struct _srtm_spi_payload *)(void *)SRTM_CommMessage_GetPayload(response);

    if (retCode != SRTM_SPI_RETCODE_SUCCESS)
    {
        spiResp->retCode = retCode;
        goto out;
    }

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO,
                       "SRTM receive SPI request:cmd=%x, busID %d, bits_per_word %d, len %dr\n", command, spiReq->busID,
                       spiReq->bits_per_word, spiReq->len);
    (void)memcpy(spiResp, spiReq, sizeof(struct _srtm_spi_payload));
    spiResp->len = responseLen;

    switch (command)
    {
        case SRTM_SPI_CMD_TRANSFER:
            /* transfer is responsible for sending response as it can be async to avoid holding task */
            return handle->transfer(service, response, spiReq->busID, spiReq->bits_per_word, spiReq->len, spiReq->data,
                                    spiResp->data);
            break;
        case SRTM_SPI_CMD_INIT:
            spiResp->retCode = SRTM_SPIService_Init(handle, spiResp->busID, spiReq->data, requestLen);
            break;

        default:
            SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported!\r\n", __func__, command);
            spiResp->retCode = SRTM_SPI_RETCODE_UNSUPPORTED;
            break;
    }

out:
    return SRTM_Dispatcher_DeliverResponse(service->dispatcher, response);
}

static srtm_status_t SRTM_SPIService_Notify(srtm_service_t service, srtm_notification_t notif)
{
    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported!\r\n", __func__,
                       SRTM_CommMessage_GetCommand(notif));

    return SRTM_Status_ServiceNotFound;
}
