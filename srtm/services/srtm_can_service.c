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
#include "srtm_can_service.h"
#include "srtm_message.h"
#include "srtm_message_pool.h"
#include "srtm_message_struct.h"

#include "app_srtm.h"
#include "build_bug.h"

#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Protocol definition */
#define SRTM_CAN_CATEGORY (0xF3U)

#define SRTM_CAN_VERSION (0x0100U)

/* Service handle */
typedef struct _srtm_can_service
{
    struct _srtm_service service;
    srtm_can_service_tx_t tx;
    srtm_can_service_open_t open;
    srtm_can_service_close_t close;
    srtm_can_service_restart_t restart;
    srtm_can_service_get_status_t getStatus;
    srtm_can_service_init_t init;
    srtm_can_service_set_wake_t setWake;
    srtm_channel_t channel;
} * srtm_can_service_t;

#define CAN_MTU_MAX MAX(CAN_MTU, CANFD_MTU)
#define CAN_OPEN_PARAMS_SIZE (sizeof(struct _srtm_can_open_params))
#define CAN_BUF_MAX MAX(CAN_MTU_MAX, CAN_OPEN_PARAMS_SIZE)

SRTM_PACKED_BEGIN struct _srtm_can_payload
{
    uint8_t request_id;
    uint8_t retcode;
    uint16_t len;
    uint8_t buf[CAN_BUF_MAX];
} SRTM_PACKED_END;

#define msg_size(s) (sizeof(struct _srtm_can_payload) - CAN_BUF_MAX + s)

enum can_rpmsg_header_type
{
    CAN_RPMSG_REQUEST,
    CAN_RPMSG_RESPONSE,
    CAN_RPMSG_NOTIFICATION,
};

enum can_rpmsg_header_cmd
{
    CAN_RPMSG_COMMAND_PAYLOAD,
    CAN_RPMSG_COMMAND_OPEN,
    CAN_RPMSG_COMMAND_STOP,
    CAN_RPMSG_COMMAND_RESTART,
    CAN_RPMSG_COMMAND_GET_STATUS,
    CAN_RPMSG_COMMAND_NOTIFY,
    CAN_RPMSG_COMMAND_INIT,
    CAN_RPMSG_COMMAND_SET_WAKE,
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/* Both request and notify are called from SRTM dispatcher context */
static srtm_status_t SRTM_CanService_Request(srtm_service_t service, srtm_request_t request)
{
    int status;
    srtm_can_service_t handle = (srtm_can_service_t)service;
    srtm_channel_t channel;
    uint8_t command, retCode;
    struct _srtm_can_payload *payload;
    srtm_response_t response;
    uint16_t response_len = 0;
    uint8_t response_buf[CAN_BUF_MAX];
    uint8_t request_id = 0;
    uint32_t len;

    assert(service->dispatcher);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    channel = SRTM_CommMessage_GetChannel(request);
    assert(channel);
    command = SRTM_CommMessage_GetCommand(request);
    payload = (struct _srtm_can_payload *)SRTM_CommMessage_GetPayload(request);
    len     = SRTM_CommMessage_GetPayloadLen(request);

    status = SRTM_Service_CheckVersion(service, request, SRTM_CAN_VERSION);

    if ((status != SRTM_Status_Success) || (payload == NULL) || len < msg_size(0))
    {
        /* Either version mismatch or empty payload is not supported */
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: format error or too short, len %d (want at least %d)!\r\n",
                           __func__, len, msg_size(0));
        retCode = kStatus_InvalidArgument;
        goto out;
    }
    if (len < msg_size(payload->len))
    {
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: too short, len %d (want %d)!\r\n", __func__, len,
                           msg_size(payload->len));
        retCode = kStatus_InvalidArgument;
        goto out;
    }
    request_id = payload->request_id;
    /* Record channel for further input event */
    handle->channel = channel;
    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_DEBUG, "can got %d (len %d)\r\n", command, payload->len);
    switch (command)
    {
        case CAN_RPMSG_COMMAND_PAYLOAD:
            if (handle->tx != NULL)
            {
                status  = handle->tx(payload->len, payload->buf);
                retCode = MIN(status, 255);
            }
            else
            {
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: Command payload not allowed?!?\r\n", __func__);
                retCode = kStatus_InvalidArgument;
            }
            break;
        case CAN_RPMSG_COMMAND_OPEN:
            if (payload->len == sizeof(srtm_can_open_params_t) && handle->open != NULL)
            {
                srtm_can_open_params_t params;
                memcpy(&params, payload->buf, sizeof(params));
                status  = handle->open(&params);
                retCode = MIN(status, 255);
            }
            else
            {
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: Command open not allowed?!? (or bad len %d)\r\n",
                                   __func__, payload->len);
                retCode = kStatus_InvalidArgument;
            }
            break;
        case CAN_RPMSG_COMMAND_STOP:
            if (payload->len == 0 && handle->close != NULL)
            {
                status  = handle->close();
                retCode = MIN(status, 255);
            }
            else
            {
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: Command close not allowed?!? (or bad len %d)\r\n",
                                   __func__, payload->len);
                retCode = kStatus_InvalidArgument;
            }
            break;
        case CAN_RPMSG_COMMAND_RESTART:
            if (payload->len == 0 && handle->restart != NULL)
            {
                status  = handle->restart();
                retCode = MIN(status, 255);
            }
            else
            {
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: Command restart not allowed?!? (or bad len %d)\r\n",
                                   __func__, payload->len);
                retCode = kStatus_InvalidArgument;
            }
            break;
        case CAN_RPMSG_COMMAND_GET_STATUS:
            if (payload->len == 0 && handle->getStatus != NULL)
            {
                BUILD_BUG_ON(sizeof(response_buf) < sizeof(uint32_t));

                status = handle->getStatus((uint32_t *)response_buf);
                if (!status)
                    response_len = sizeof(uint32_t);
                retCode = MIN(status, 255);
            }
            else
            {
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: Command get_status not allowed?!? (or bad len %d)\r\n",
                                   __func__, payload->len);
                retCode = kStatus_InvalidArgument;
            }
            break;
        case CAN_RPMSG_COMMAND_INIT:
            if (payload->len == sizeof(srtm_can_init_params_t) && handle->init != NULL)
            {
                srtm_can_init_params_t params;
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_DEBUG, "can: init\r\n");
                memcpy(&params, payload->buf, sizeof(params));
                status  = handle->init(&params);
                retCode = MIN(status, 255);
            }
            else
            {
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: Command init not allowed?!? (or bad len %d)\r\n",
                                   __func__, payload->len);
                retCode = kStatus_InvalidArgument;
            }
            break;
        case CAN_RPMSG_COMMAND_SET_WAKE:
            if (payload->len == sizeof(bool) && handle->setWake != NULL)
            {
                bool enable = payload->buf[0];
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_DEBUG, "can set wake %d\r\n", payload->buf[0]);
                status  = handle->setWake(enable);
                retCode = MIN(status, 255);
            }
            else
            {
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: Command set wake not allowed?!? (or bad len %d)\r\n",
                                   __func__, payload->len);
                retCode = kStatus_InvalidArgument;
            }
            break;
        default:
            SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported!\r\n", __func__, command);
            retCode = kStatus_InvalidArgument;
            break;
    }

out:
    response = SRTM_Response_Create(channel, SRTM_CAN_CATEGORY, SRTM_CAN_VERSION, command, msg_size(response_len));
    if (!response)
    {
        return SRTM_Status_OutOfMemory;
    }

    payload             = (struct _srtm_can_payload *)SRTM_CommMessage_GetPayload(response);
    payload->request_id = request_id;
    payload->retcode    = retCode;
    payload->len        = response_len;
    if (response_len)
        memcpy(payload->buf, response_buf, response_len);
    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_DEBUG, "can resp %d\r\n", retCode);

    /* Now the response is ready */
    return SRTM_Dispatcher_DeliverResponse(service->dispatcher, response);
}

static srtm_status_t SRTM_CanService_Notify(srtm_service_t service, srtm_notification_t notif)
{
    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported!\r\n", __func__,
                       SRTM_CommMessage_GetCommand(notif));

    return SRTM_Status_ServiceNotFound;
}

srtm_service_t SRTM_CanService_Create(srtm_can_service_tx_t tx, srtm_can_service_open_t open,
                                      srtm_can_service_close_t close, srtm_can_service_restart_t restart,
                                      srtm_can_service_get_status_t getStatus, srtm_can_service_init_t init,
                                      srtm_can_service_set_wake_t setWake)
{
    srtm_can_service_t handle;

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    handle = (srtm_can_service_t)SRTM_Heap_Malloc(sizeof(struct _srtm_can_service));
    assert(handle);

    handle->tx        = tx;
    handle->open      = open;
    handle->close     = close;
    handle->restart   = restart;
    handle->getStatus = getStatus;
    handle->init      = init;
    handle->setWake   = setWake;

    SRTM_List_Init(&handle->service.node);
    handle->service.dispatcher = NULL;
    handle->service.category   = SRTM_CAN_CATEGORY;
    handle->service.destroy    = SRTM_CanService_Destroy;
    handle->service.request    = SRTM_CanService_Request;
    handle->service.notify     = SRTM_CanService_Notify;

    return &handle->service;
}

void SRTM_CanService_Destroy(srtm_service_t service)
{
    srtm_can_service_t handle = (srtm_can_service_t)service;

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    assert(service);
    /* Service must be unregistered from dispatcher before destroy */
    assert(SRTM_List_IsEmpty(&service->node));

    SRTM_Heap_Free(handle);
}

void SRTM_CanService_Reset(srtm_service_t service, srtm_peercore_t core)
{
    srtm_can_service_t handle = (srtm_can_service_t)service;

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    assert(service);

    /* Currently assume just one peer core, need to update all pins. */
    handle->channel = NULL;
}

srtm_notification_t SRTM_CanService_NotifyAlloc(struct canfd_frame **cfd)
{
    struct _srtm_can_payload *payload;
    srtm_notification_t notif =
        SRTM_Notification_Create(NULL, SRTM_CAN_CATEGORY, SRTM_CAN_VERSION, CAN_RPMSG_COMMAND_NOTIFY, sizeof(*payload));
    if (!notif)
    {
        PRINTF("Could not alloc can rx buffer\r\n");
        return NULL;
    }
    payload = (struct _srtm_can_payload *)SRTM_CommMessage_GetPayload(notif);

    *cfd = (struct canfd_frame *)payload->buf;
    return notif;
}

srtm_status_t SRTM_CanService_NotifySend(srtm_service_t service, srtm_notification_t notif, uint16_t len)
{
    srtm_can_service_t handle = (srtm_can_service_t)service;

    assert(service);
    /* Service must be running in dispatcher when notifying input event */
    assert(!SRTM_List_IsEmpty(&service->node));

    if (!handle->channel)
    {
        PRINTF("no channel, never got anything from linux?\r\n");
        SRTM_Notification_Destroy(notif);
        return SRTM_Status_ServiceNotFound;
    }

    struct _srtm_can_payload *payload;
    payload        = (struct _srtm_can_payload *)SRTM_CommMessage_GetPayload(notif);
    payload->len   = len;
    notif->channel = handle->channel;

    SRTM_Dispatcher_DeliverNotification(handle->service.dispatcher, notif);

    return SRTM_Status_Success;
}
