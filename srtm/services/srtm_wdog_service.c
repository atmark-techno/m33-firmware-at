/*
 * Copyright 2017, NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>

#include "srtm_heap.h"
#include "srtm_dispatcher.h"
#include "srtm_peercore.h"
#include "srtm_service.h"
#include "srtm_service_struct.h"
#include "srtm_channel.h"
#include "srtm_channel_struct.h"
#include "srtm_wdog_service.h"
#include "srtm_message.h"
#include "srtm_message_pool.h"
#include "srtm_message_struct.h"

#include "app_srtm.h"

#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Protocol definition */
#define SRTM_WDOG_CATEGORY (0xF2U)

#define SRTM_WDOG_VERSION (0x0100U)

/* Service handle */
typedef struct _srtm_wdog_service
{
    struct _srtm_service service;
    srtm_wdog_service_enable_t enableCb;
    srtm_wdog_service_ping_t pingCb;
    srtm_channel_t channel;
} * srtm_wdog_service_t;

enum wdog_rpmsg_command
{
    WDOG_RPMSG_ENABLE = 0,
    WDOG_RPMSG_PING,
    WDOG_RPMSG_NOTIFY,
};

enum wdog_rpmsg_retcode
{
    WDOG_RPMSG_SUCCESS = 0,
    WDOG_RPMSG_ERROR,
    WDOG_RPMSG_UNSUPPORTED,
};

SRTM_PACKED_BEGIN struct _srtm_wdog_payload
{
    uint8_t request_id;

    union
    {
        SRTM_PACKED_BEGIN struct
        {
            bool enabled;
            uint16_t timeout;
        } SRTM_PACKED_END enable;
        // ping: no arg
        // notify: no arg
        SRTM_PACKED_BEGIN struct
        {
            uint8_t retcode; // enum wdog_rpmsg_retcode
        } SRTM_PACKED_END reply;
    };
} SRTM_PACKED_END;

static srtm_status_t SRTM_WdogService_Request(srtm_service_t service, srtm_request_t request)
{
    srtm_status_t status;
    srtm_wdog_service_t handle = (srtm_wdog_service_t)service;
    srtm_channel_t channel;
    uint8_t command, retCode, request_id = 0;
    struct _srtm_wdog_payload *payload;
    srtm_response_t response;
    uint32_t len;

    assert(service->dispatcher);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    channel = SRTM_CommMessage_GetChannel(request);
    assert(channel);
    command = SRTM_CommMessage_GetCommand(request);
    payload = (struct _srtm_wdog_payload *)SRTM_CommMessage_GetPayload(request);
    len     = SRTM_CommMessage_GetPayloadLen(request);

    status = SRTM_Service_CheckVersion(service, request, SRTM_WDOG_VERSION);
    if ((status != SRTM_Status_Success) || (payload == NULL) || (len < sizeof(*payload)))
    {
        /* Either version mismatch or empty payload is not supported */
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: format error, len %d (want %d)!\r\n", __func__, len,
                           sizeof(*payload));
        retCode = WDOG_RPMSG_UNSUPPORTED;
        goto out;
    }

    /* Record channel for notify */
    handle->channel = channel;
    request_id      = payload->request_id;
    switch (command)
    {
        case WDOG_RPMSG_ENABLE:
            status  = handle->enableCb(payload->enable.enabled, payload->enable.timeout);
            retCode = status == SRTM_Status_Success ? WDOG_RPMSG_SUCCESS : WDOG_RPMSG_ERROR;
            break;
        case WDOG_RPMSG_PING:
            status  = handle->pingCb();
            retCode = status == SRTM_Status_Success ? WDOG_RPMSG_SUCCESS : WDOG_RPMSG_ERROR;
            break;
        default:
            SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported!\r\n", __func__, command);
            retCode = WDOG_RPMSG_UNSUPPORTED;
            break;
    }

out:
    response = SRTM_Response_Create(channel, SRTM_WDOG_CATEGORY, SRTM_WDOG_VERSION, command, sizeof(*payload));
    if (!response)
    {
        return SRTM_Status_OutOfMemory;
    }

    payload                = (struct _srtm_wdog_payload *)SRTM_CommMessage_GetPayload(response);
    payload->request_id    = request_id;
    payload->reply.retcode = retCode;

    /* Now the response is ready */
    return SRTM_Dispatcher_DeliverResponse(service->dispatcher, response);
}

srtm_status_t SRTM_WdogService_NotifyPreTimeout(srtm_service_t service)
{
    srtm_wdog_service_t handle = (srtm_wdog_service_t)service;

    assert(service);

    // skip if no channel available
    if (!handle->channel)
        return SRTM_Status_Success;

    struct _srtm_wdog_payload *payload;
    srtm_notification_t notif =
        SRTM_Notification_Create(NULL, SRTM_WDOG_CATEGORY, SRTM_WDOG_VERSION, WDOG_RPMSG_NOTIFY, sizeof(*payload));
    if (!notif)
    {
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_ERROR, "%s: alloc notification failed.\r\n", __func__);
        return SRTM_Status_OutOfMemory;
    }
    payload             = (struct _srtm_wdog_payload *)SRTM_CommMessage_GetPayload(notif);
    payload->request_id = 0;
    notif->channel      = handle->channel;
    SRTM_Dispatcher_DeliverNotification(handle->service.dispatcher, notif);

    return SRTM_Status_Success;
}

srtm_service_t SRTM_WdogService_Create(srtm_wdog_service_enable_t enableCb, srtm_wdog_service_ping_t pingCb)
{
    srtm_wdog_service_t handle;

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    handle = (srtm_wdog_service_t)SRTM_Heap_Malloc(sizeof(struct _srtm_wdog_service));
    assert(handle);

    handle->enableCb = enableCb;
    handle->pingCb   = pingCb;

    SRTM_List_Init(&handle->service.node);
    handle->service.dispatcher = NULL;
    handle->service.category   = SRTM_WDOG_CATEGORY;
    handle->service.destroy    = SRTM_WdogService_Destroy;
    handle->service.request    = SRTM_WdogService_Request;

    return &handle->service;
}

void SRTM_WdogService_Destroy(srtm_service_t service)
{
    srtm_wdog_service_t handle = (srtm_wdog_service_t)service;

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    assert(service);
    /* Service must be unregistered from dispatcher before destroy */
    assert(SRTM_List_IsEmpty(&service->node));

    SRTM_Heap_Free(handle);
}
