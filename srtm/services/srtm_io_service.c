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
#include "srtm_io_service.h"
#include "srtm_message.h"
#include "srtm_message_pool.h"
#include "srtm_message_struct.h"

#include "app_srtm.h"

#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Protocol definition */
#define SRTM_IO_CATEGORY (0x5U)

#define SRTM_IO_VERSION (0x0300U)

#define SRTM_IO_RETURN_CODE_SUCCESS     (0x0U)
#define SRTM_IO_RETURN_CODE_FAIL        (0x1U)
#define SRTM_IO_RETURN_CODE_UNSUPPORTED (0x2U)

#define SRTM_IO_NTF_INPUT_EVENT (0x00U)

/* IO pin list node */
typedef struct _srtm_io_pin
{
    bool notified;
} *srtm_io_pin_t;

/* Service handle */
typedef struct _srtm_io_service
{
    struct _srtm_service service;
    srtm_io_service_input_init_t inputInit;
    srtm_io_service_output_init_t outputInit;
    srtm_io_service_input_get_t inputGet;
    srtm_io_service_output_set_t outputSet;
    int pin_count;
    srtm_channel_t channel;
    struct _srtm_io_pin pins[];
} *srtm_io_service_t;

SRTM_PACKED_BEGIN struct _srtm_io_payload
{
    uint8_t request_id;
    uint8_t pin_idx;
    uint8_t port_idx;
    union {
        SRTM_PACKED_BEGIN struct {
            uint8_t event;
            uint8_t wakeup;
            uint32_t pinctrl;
        } SRTM_PACKED_END input_init;
        SRTM_PACKED_BEGIN struct {
            uint8_t value;
            uint32_t pinctrl;
        } SRTM_PACKED_END output_init;
        /* no arg for input_get */
        SRTM_PACKED_BEGIN struct {
            uint8_t value;
        } SRTM_PACKED_END output_set;
        SRTM_PACKED_BEGIN struct {
            uint8_t retcode;
            uint8_t value; /* only valid for input_get */
        } SRTM_PACKED_END reply;
    };
} SRTM_PACKED_END;

enum gpio_rpmsg_header_cmd {
    GPIO_RPMSG_INPUT_INIT,
    GPIO_RPMSG_OUTPUT_INIT,
    GPIO_RPMSG_INPUT_GET,
    GPIO_RPMSG_OUTPUT_SET,
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
static void SRTM_IoService_RecycleMessage(srtm_message_t msg, void *param)
{
    uint32_t primask;
    srtm_io_pin_t pin = (srtm_io_pin_t)param;

    assert(pin);
    assert(pin->notified == true);

    primask = DisableGlobalIRQ();
    pin->notified = false;
    EnableGlobalIRQ(primask);

    SRTM_MessagePool_Free(msg);
}

static srtm_io_pin_t SRTM_IoService_FindPin(srtm_io_service_t handle, uint16_t ioId)
{
    /* XXX abstraction leak */
    uint16_t idx = APP_IO_GetIndex(ioId);
    if (idx >= handle->pin_count) {
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_ERROR,
                               "%s: ioId too high %x\r\n", __func__, ioId);
        return NULL;
    }
    return handle->pins + idx;
}

/* Both request and notify are called from SRTM dispatcher context */
static srtm_status_t SRTM_IoService_Request(srtm_service_t service, srtm_request_t request)
{
    srtm_status_t status;
    srtm_io_service_t handle = (srtm_io_service_t)service;
    srtm_io_pin_t pin;
    srtm_channel_t channel;
    uint8_t command, retCode;
    struct _srtm_io_payload *payload;
    srtm_response_t response;
    uint8_t request_id = 0;
    uint16_t ioId = 0U;
    uint32_t len;
    srtm_io_value_t value = SRTM_IoValueLow;

    assert(service->dispatcher);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    channel = SRTM_CommMessage_GetChannel(request);
    assert(channel);
    command = SRTM_CommMessage_GetCommand(request);
    payload = (struct _srtm_io_payload*)SRTM_CommMessage_GetPayload(request);
    len     = SRTM_CommMessage_GetPayloadLen(request);

    status = SRTM_Service_CheckVersion(service, request, SRTM_IO_VERSION);
    if ((status != SRTM_Status_Success) || (payload == NULL) || (len != sizeof(*payload)))
    {
        /* Either version mismatch or empty payload is not supported */
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: format error, len %d (want %d)!\r\n",
                           __func__, len, sizeof(*payload));
        retCode = SRTM_IO_RETURN_CODE_UNSUPPORTED;
        goto out;
    }
    request_id = payload->request_id;
    ioId = (payload->port_idx << 8) | payload->pin_idx;
    pin  = SRTM_IoService_FindPin(handle, ioId);
    if (!pin)
    {
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: Pin 0x%x not registered!\r\n", __func__, ioId);
        retCode = SRTM_IO_RETURN_CODE_FAIL;
        goto out;
    }
    /* Record channel for further input event */
    handle->channel = channel;
    switch (command)
    {
        case GPIO_RPMSG_INPUT_INIT:
            if (handle->inputInit != NULL)
            {
                status = handle->inputInit(service, channel->core, ioId,
                                         payload->input_init.event,
                                         payload->input_init.wakeup,
                                         payload->input_init.pinctrl);
                retCode =
                    status == SRTM_Status_Success ? SRTM_IO_RETURN_CODE_SUCCESS : SRTM_IO_RETURN_CODE_FAIL;
            }
            else
            {
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN,
                                   "%s: Command input init not allowed!\r\n", __func__);
                retCode = SRTM_IO_RETURN_CODE_FAIL;
            }
            break;
        case GPIO_RPMSG_OUTPUT_INIT:
            if (handle->outputInit != NULL)
            {
                status = handle->outputInit(service, channel->core, ioId,
                        payload->output_init.value, payload->output_init.pinctrl);
                retCode =
                    status == SRTM_Status_Success ? SRTM_IO_RETURN_CODE_SUCCESS : SRTM_IO_RETURN_CODE_FAIL;
            }
            else
            {
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN,
                                   "%s: Command output init not allowed!\r\n", __func__);
                retCode = SRTM_IO_RETURN_CODE_FAIL;
            }
            break;
        case GPIO_RPMSG_INPUT_GET:
            if (handle->inputGet)
            {
                status = handle->inputGet(service, channel->core, ioId, &value);
                retCode =
                    status == SRTM_Status_Success ? SRTM_IO_RETURN_CODE_SUCCESS : SRTM_IO_RETURN_CODE_FAIL;
            }
            else
            {
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: Command input get function not registered!\r\n",
                                   __func__);
                retCode = SRTM_IO_RETURN_CODE_FAIL;
            }
            break;
        case GPIO_RPMSG_OUTPUT_SET:
            if (handle->outputSet != NULL)
            {
                status = handle->outputSet(service, channel->core, ioId, payload->output_set.value);
                retCode =
                    status == SRTM_Status_Success ? SRTM_IO_RETURN_CODE_SUCCESS : SRTM_IO_RETURN_CODE_FAIL;
            }
            else
            {
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN,
                                   "%s: Command ouput set not allowed!\r\n", __func__);
                retCode = SRTM_IO_RETURN_CODE_FAIL;
            }
            break;
        default:
            SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported!\r\n", __func__, command);
            retCode = SRTM_IO_RETURN_CODE_UNSUPPORTED;
            break;
    }

out:
    response = SRTM_Response_Create(channel, SRTM_IO_CATEGORY, SRTM_IO_VERSION,
                                    command, sizeof(*payload));
    if (!response)
    {
        return SRTM_Status_OutOfMemory;
    }

    payload = (struct _srtm_io_payload*)SRTM_CommMessage_GetPayload(response);
    payload->request_id = request_id;
    payload->pin_idx = (uint8_t)ioId;
    payload->port_idx = (uint8_t)(ioId >> 8U);
    payload->reply.retcode = retCode;
    payload->reply.value = value; /* Only used in GPIO_RPMSG_INPUT_GET */

    /* Now the response is ready */
    return SRTM_Dispatcher_DeliverResponse(service->dispatcher, response);
}

static srtm_status_t SRTM_IoService_Notify(srtm_service_t service, srtm_notification_t notif)
{
    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported!\r\n", __func__,
                       SRTM_CommMessage_GetCommand(notif));

    return SRTM_Status_ServiceNotFound;
}

srtm_service_t SRTM_IoService_Create(int pin_count,
                                     srtm_io_service_input_init_t inputInit,
                                     srtm_io_service_output_init_t outputInit,
                                     srtm_io_service_input_get_t inputGet,
                                     srtm_io_service_output_set_t outputSet)
{
    srtm_io_service_t handle;

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    handle = (srtm_io_service_t)SRTM_Heap_Malloc(
            sizeof(struct _srtm_io_service) + pin_count * sizeof(struct _srtm_io_pin));
    assert(handle);

    memset(handle->pins, 0, pin_count * sizeof(struct _srtm_io_pin));
    handle->pin_count = pin_count;
    handle->inputInit = inputInit;
    handle->outputInit = outputInit;
    handle->inputGet = inputGet;
    handle->outputSet = outputSet;

    SRTM_List_Init(&handle->service.node);
    handle->service.dispatcher = NULL;
    handle->service.category   = SRTM_IO_CATEGORY;
    handle->service.destroy    = SRTM_IoService_Destroy;
    handle->service.request    = SRTM_IoService_Request;
    handle->service.notify     = SRTM_IoService_Notify;

    return &handle->service;
}

void SRTM_IoService_Destroy(srtm_service_t service)
{
    srtm_io_service_t handle = (srtm_io_service_t)service;

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    assert(service);
    /* Service must be unregistered from dispatcher before destroy */
    assert(SRTM_List_IsEmpty(&service->node));

    SRTM_Heap_Free(handle);
}

void SRTM_IoService_Reset(srtm_service_t service, srtm_peercore_t core)
{
    srtm_io_service_t handle = (srtm_io_service_t)service;

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    assert(service);

    /* Currently assume just one peer core, need to update all pins. */
    handle->channel = NULL;
}

/* Called in ISR */
srtm_status_t SRTM_IoService_NotifyInputEvent(srtm_service_t service, uint16_t ioId)
{
    srtm_io_service_t handle = (srtm_io_service_t)service;
    srtm_io_pin_t pin;

    assert(service);
    /* Service must be running in dispatcher when notifying input event */
    assert(!SRTM_List_IsEmpty(&service->node));

    pin = SRTM_IoService_FindPin(handle, ioId);
    if (!pin)
    {
        /* Pin not registered */
        return SRTM_Status_InvalidParameter;
    }

    uint32_t primask;
    bool notify = false;
    primask = DisableGlobalIRQ();
    if (!pin->notified && handle->channel)
    {
            notify = true;
            pin->notified = true;
    }
    EnableGlobalIRQ(primask);

    if (notify)
    {
        srtm_notification_t notif = SRTM_Notification_Create(
                    NULL, SRTM_IO_CATEGORY, SRTM_IO_VERSION,
                    SRTM_IO_NTF_INPUT_EVENT, 2U);
        if (!notif) {
            SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_ERROR,
                            "%s: alloc notification failed.\r\n", __func__);
            /* can't be raced as we didn't send a notification */
            pin->notified = false;
            return SRTM_Status_OutOfMemory;
        }
        /* restore pin->notified before free when sent is done */
        SRTM_Message_SetFreeFunc(notif, SRTM_IoService_RecycleMessage, pin);
        struct _srtm_io_payload *payload;
        payload = (struct _srtm_io_payload*)SRTM_CommMessage_GetPayload(notif);
        payload->pin_idx = (uint8_t)ioId;
        payload->port_idx = (uint8_t)(ioId >> 8U);
        notif->channel = handle->channel;
        SRTM_Dispatcher_DeliverNotification(handle->service.dispatcher, notif);
    }



    return SRTM_Status_Success;
}
