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
#include "srtm_tty_service.h"
#include "srtm_message.h"
#include "srtm_message_pool.h"
#include "srtm_message_struct.h"

#include "build_bug.h"
#include "app_srtm.h"

#include "fsl_common.h"

/* uncomment to debug this service */
//#undef SRTM_DEBUG_VERBOSE_LEVEL
//#define SRTM_DEBUG_VERBOSE_LEVEL SRTM_DEBUG_VERBOSE_DEBUG

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Protocol definition */
#define SRTM_TTY_CATEGORY (0xF1U)

#define SRTM_TTY_VERSION (0x0300U)

/* Service handle */
typedef struct _srtm_tty_service
{
    struct _srtm_service service;
    srtm_tty_service_tx_t tx;
    srtm_tty_service_set_cflag_t setCflag;
    srtm_tty_service_set_wake_t setWake;
    srtm_tty_service_init_t init;
    srtm_channel_t channel;
} * srtm_tty_service_t;

#if !(defined(RL_ALLOW_CUSTOM_SHMEM_CONFIG) && (RL_ALLOW_CUSTOM_SHMEM_CONFIG == 1))
#define RS485_LPUART_BUFFER_LENGTH (RL_BUFFER_PAYLOAD_SIZE)
#else
#define RS485_LPUART_BUFFER_LENGTH (RL_BUFFER_PAYLOAD_SIZE(0))
#endif
/* sizeof(app_rpmsg_msg) must be less than or equal to LPUART_BUFFER_LENGTH */
#define RPMSG_MAX_SIZE (RS485_LPUART_BUFFER_LENGTH - 14)

SRTM_PACKED_BEGIN struct _srtm_tty_payload
{
    uint8_t port_idx;
    uint8_t request_id;
    uint16_t len;
    SRTM_PACKED_BEGIN union
    {
        uint8_t buf[RPMSG_MAX_SIZE];
        uint32_t cflag;
        /* note: packed only by design, sanity is ensured by checking size */
        struct srtm_tty_init_payload init;
        uint8_t retcode;
    } SRTM_PACKED_END;
} SRTM_PACKED_END;

#define msg_size(s) (sizeof(struct _srtm_tty_payload) - RPMSG_MAX_SIZE + s)

enum tty_rpmsg_header_type
{
    TTY_RPMSG_REQUEST,
    TTY_RPMSG_RESPONSE,
    TTY_RPMSG_NOTIFICATION,
};

enum tty_rpmsg_header_cmd
{
    TTY_RPMSG_COMMAND_PAYLOAD,
    TTY_RPMSG_COMMAND_SET_CFLAG,
    TTY_RPMSG_COMMAND_NOTIFY,
    TTY_RPMSG_COMMAND_SET_WAKE,
    TTY_RPMSG_COMMAND_INIT,
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
static srtm_status_t SRTM_TtyService_Request(srtm_service_t service, srtm_request_t request)
{
    int status;
    srtm_tty_service_t handle = (srtm_tty_service_t)service;
    srtm_channel_t channel;
    uint8_t command, retCode;
    struct _srtm_tty_payload *payload;
    srtm_response_t response;
    uint8_t request_id = 0, port_idx = 0;
    uint32_t len;

    /* fails if RPMSG_MAX_SIZE is too big */
    BUILD_BUG_ON(sizeof(*payload) + sizeof(srtm_packet_head_t) > RS485_LPUART_BUFFER_LENGTH);
    assert(service->dispatcher);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    channel = SRTM_CommMessage_GetChannel(request);
    assert(channel);
    command = SRTM_CommMessage_GetCommand(request);
    payload = (struct _srtm_tty_payload *)SRTM_CommMessage_GetPayload(request);
    len     = SRTM_CommMessage_GetPayloadLen(request);

    status = SRTM_Service_CheckVersion(service, request, SRTM_TTY_VERSION);

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
    port_idx   = payload->port_idx;
    request_id = payload->request_id;
    /* Record channel for further input event */
    handle->channel = channel;
    switch (command)
    {
        case TTY_RPMSG_COMMAND_PAYLOAD:
            if (handle->tx != NULL)
            {
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_DEBUG, "tty %d tx, len %d\r\n", port_idx, payload->len);
                status  = handle->tx(port_idx, payload->buf, payload->len);
                retCode = MIN(status, 255);
            }
            else
            {
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: Command payload not allowed?!?\r\n", __func__);
                retCode = kStatus_InvalidArgument;
            }
            break;
        case TTY_RPMSG_COMMAND_SET_CFLAG:
            if (payload->len == sizeof(tcflag_t) && handle->setCflag != NULL)
            {
                uint32_t cflag;
                memcpy(&cflag, &payload->cflag, sizeof(cflag));
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_DEBUG, "tty %d set cflag %d\r\n", port_idx, cflag);
                status  = handle->setCflag(port_idx, cflag);
                retCode = MIN(status, 255);
            }
            else
            {
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN,
                                   "%s: Command set cflag not allowed?!? (or bad len %d, expected %zu)\r\n", __func__,
                                   payload->len, sizeof(tcflag_t));
                retCode = kStatus_InvalidArgument;
            }
            break;
        case TTY_RPMSG_COMMAND_SET_WAKE:
            if (payload->len == sizeof(bool) && handle->setWake != NULL)
            {
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_DEBUG, "tty %d set wake %d\r\n", port_idx, payload->buf[0]);
                status  = handle->setWake(port_idx, payload->buf[0]);
                retCode = MIN(status, 255);
            }
            else
            {
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN,
                                   "%s: Command set wake not allowed?!? (or bad len %d, expected %zu)\r\n", __func__,
                                   payload->len, sizeof(bool));
                retCode = kStatus_InvalidArgument;
            }
            break;
        case TTY_RPMSG_COMMAND_INIT:
            if (payload->len == sizeof(struct srtm_tty_init_payload) && handle->init != NULL)
            {
                struct srtm_tty_init_payload init;
                memcpy(&init, &payload->init, sizeof(init));
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_DEBUG, "tty %d: init\r\n", port_idx);
                status  = handle->init(port_idx, &init);
                retCode = MIN(status, 255);
            }
            else
            {
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN,
                                   "%s: Command init not allowed?!? (or bad len %d, expected %zd)\r\n", __func__,
                                   payload->len, sizeof(struct srtm_tty_init_payload));
                retCode = kStatus_InvalidArgument;
            }
            break;
        default:
            SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported!\r\n", __func__, command);
            retCode = kStatus_InvalidArgument;
            break;
    }

out:
    response = SRTM_Response_Create(channel, SRTM_TTY_CATEGORY, SRTM_TTY_VERSION, command, msg_size(1));
    if (!response)
    {
        return SRTM_Status_OutOfMemory;
    }

    payload             = (struct _srtm_tty_payload *)SRTM_CommMessage_GetPayload(response);
    payload->port_idx   = port_idx;
    payload->request_id = request_id;
    payload->len        = 1;
    payload->buf[0]     = retCode;
    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_DEBUG, "tty resp %d\r\n", retCode);

    /* Now the response is ready */
    return SRTM_Dispatcher_DeliverResponse(service->dispatcher, response);
}

static srtm_status_t SRTM_TtyService_Notify(srtm_service_t service, srtm_notification_t notif)
{
    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported!\r\n", __func__,
                       SRTM_CommMessage_GetCommand(notif));

    return SRTM_Status_ServiceNotFound;
}

srtm_service_t SRTM_TtyService_Create(srtm_tty_service_tx_t tx, srtm_tty_service_set_cflag_t setCflag,
                                      srtm_tty_service_set_wake_t setWake, srtm_tty_service_init_t init)
{
    srtm_tty_service_t handle;

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    handle = (srtm_tty_service_t)SRTM_Heap_Malloc(sizeof(struct _srtm_tty_service));
    assert(handle);

    handle->tx       = tx;
    handle->setCflag = setCflag;
    handle->setWake  = setWake;
    handle->init     = init;

    SRTM_List_Init(&handle->service.node);
    handle->service.dispatcher = NULL;
    handle->service.category   = SRTM_TTY_CATEGORY;
    handle->service.destroy    = SRTM_TtyService_Destroy;
    handle->service.request    = SRTM_TtyService_Request;
    handle->service.notify     = SRTM_TtyService_Notify;

    return &handle->service;
}

void SRTM_TtyService_Destroy(srtm_service_t service)
{
    srtm_tty_service_t handle = (srtm_tty_service_t)service;

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    assert(service);
    /* Service must be unregistered from dispatcher before destroy */
    assert(SRTM_List_IsEmpty(&service->node));

    SRTM_Heap_Free(handle);
}

void SRTM_TtyService_Reset(srtm_service_t service, srtm_peercore_t core)
{
    srtm_tty_service_t handle = (srtm_tty_service_t)service;

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    assert(service);

    /* Currently assume just one peer core, need to update all pins. */
    handle->channel = NULL;
}

srtm_notification_t SRTM_TtyService_NotifyAlloc(uint8_t port_idx, uint8_t **buf, uint16_t *len)
{
    struct _srtm_tty_payload *payload;
    srtm_notification_t notif =
        SRTM_Notification_Create(NULL, SRTM_TTY_CATEGORY, SRTM_TTY_VERSION, TTY_RPMSG_COMMAND_NOTIFY, sizeof(*payload));
    if (!notif)
    {
        PRINTF("Could not alloc tty rx buffer\r\n");
        return NULL;
    }

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s port %d\r\n", __func__, port_idx);

    payload           = (struct _srtm_tty_payload *)SRTM_CommMessage_GetPayload(notif);
    payload->port_idx = port_idx;

    /* allocate smaller buffers than max: since rx times out in 1ms then
     * with the default 115200 baud rate we never send more than 14-15
     * chars at a timer.
     * At 1M baud rate we'd need 125chars/ms so increasing the size depending
     * on baud rate would make sense, to be tried later.
     */
    *len = MIN(RPMSG_MAX_SIZE, 64);
    *buf = payload->buf;
    return notif;
}

srtm_status_t SRTM_TtyService_NotifySend(srtm_service_t service, srtm_notification_t notif, uint16_t len)
{
    srtm_tty_service_t handle = (srtm_tty_service_t)service;

    assert(service);
    /* Service must be running in dispatcher when notifying input event */
    assert(!SRTM_List_IsEmpty(&service->node));

    if (!handle->channel)
    {
        PRINTF("no channel, never got anything from linux?\r\n");
        SRTM_Notification_Destroy(notif);
        return SRTM_Status_ServiceNotFound;
    }

    struct _srtm_tty_payload *payload;
    payload        = (struct _srtm_tty_payload *)SRTM_CommMessage_GetPayload(notif);
    payload->len   = len;
    notif->channel = handle->channel;

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s port %d len %d\r\n", __func__, payload->port_idx, len);

    uint32_t dataLen = msg_size(len) + sizeof(srtm_packet_head_t);
    assert(dataLen <= notif->dataLen);
    /* This controls how much is actually sent to linux, and is not
     * used for any free/recycling so we can just override this. */
    notif->dataLen = dataLen;
    SRTM_Dispatcher_DeliverNotification(handle->service.dispatcher, notif);

    return SRTM_Status_Success;
}
