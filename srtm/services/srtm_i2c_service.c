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
#include "srtm_i2c_service.h"
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
#define SRTM_I2C_CATEGORY (0x9U)
#define SRTM_I2C_VERSION (0x0100U)

/* from linux drivers/i2c/busses/i2c-rpmsg-imx.c top comment.
 * values are only printed as int anyway, non-zero is error. */
#define SRTM_I2C_RETURN_CODE_SUCCESS (0x0U)
#define SRTM_I2C_RETURN_CODE_FAIL (0x1U)
#define SRTM_I2C_RETURN_CODE_UNSUPPORTED (0x2U)
#define SRTM_I2C_RETURN_CODE_EINVAL (0x3U)
#define SRTM_I2C_RETURN_CODE_ENOMEM (0x5U)

typedef struct _srtm_i2c_service
{
    struct _srtm_service service;
    srtm_i2c_adapter_t adapter;
} * srtm_i2c_service_t;

static srtm_status_t SRTM_I2CService_Request(srtm_service_t service, srtm_request_t request);
static srtm_status_t SRTM_I2CService_Notify(srtm_service_t service, srtm_notification_t notif);

static i2c_bus_t SRTM_I2C_SearchBus(srtm_i2c_adapter_t adapter, uint8_t busID)
{
    uint8_t bus_num    = adapter->bus_structure.bus_num;
    i2c_bus_t busArray = adapter->bus_structure.buses;
    uint8_t i;

    for (i = 0U; i != bus_num; i++)
    {
        if (busArray[i].bus_id == busID)
        {
            break;
        }
    }

    if (i == bus_num)
    {
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "i2c bus %d not found!\r\n", busID);
        return NULL;
    }

    return busArray + i;
}

static srtm_status_t SRTM_I2CService_ReadBus(srtm_service_t service, uint8_t busID, uint16_t slaveAddr, uint8_t *buf,
                                             uint16_t len, uint16_t flags)
{
    srtm_i2c_service_t handle  = (srtm_i2c_service_t)(void *)service;
    srtm_i2c_adapter_t adapter = handle->adapter;
    i2c_bus_t targetBus;
    uint32_t base_addr;
    uint8_t switch_index;
    uint16_t switch_addr;
    srtm_i2c_switch_channel switch_channel;
    srtm_status_t status;
    srtm_i2c_type_t type;
    i2c_switch_t switch_inst;

    targetBus = SRTM_I2C_SearchBus(adapter, busID);
    if (!targetBus)
        return SRTM_Status_Error;
    base_addr    = targetBus->base_addr;
    switch_index = targetBus->switch_idx;
    type         = targetBus->type;
    /*
     * Switch Channel
     */
    if (switch_index < adapter->bus_structure.switch_num)
    {
        switch_inst    = &adapter->bus_structure.switches[switch_index];
        switch_addr    = switch_inst->slaveAddr;
        switch_channel = targetBus->switch_channel;
        if (switch_inst->cur_channel != switch_channel)
        {
            (void)adapter->switchchannel(adapter, base_addr, type, switch_addr, switch_channel);
            switch_inst->cur_channel = switch_channel;
        }
    }
    /*
     * Read
     */
    status = adapter->read(adapter, base_addr, type, slaveAddr, buf, len, flags); // APP_SRTM_I2C_Read
    return status;
}

static srtm_status_t SRTM_I2CService_WriteBus(srtm_service_t service, uint8_t busID, uint16_t slaveAddr, uint8_t *buf,
                                              uint16_t len, uint16_t flags)
{
    srtm_i2c_service_t handle  = (srtm_i2c_service_t)(void *)service;
    srtm_i2c_adapter_t adapter = handle->adapter;
    i2c_bus_t targetBus;
    uint32_t base_addr;
    uint8_t switch_index;
    uint16_t switch_addr;
    srtm_i2c_switch_channel switch_channel;
    srtm_status_t status;
    srtm_i2c_type_t type;
    i2c_switch_t switch_inst;

    targetBus = SRTM_I2C_SearchBus(adapter, busID);
    if (!targetBus)
        return SRTM_Status_Error;
    base_addr    = targetBus->base_addr;
    switch_index = targetBus->switch_idx;
    type         = targetBus->type;
    /*
     * Switch Channel
     */
    if (switch_index < adapter->bus_structure.switch_num)
    {
        switch_inst    = &adapter->bus_structure.switches[switch_index];
        switch_addr    = switch_inst->slaveAddr;
        switch_channel = targetBus->switch_channel;
        if (switch_inst->cur_channel != switch_channel)
        {
            (void)adapter->switchchannel(adapter, base_addr, type, switch_addr, switch_channel);
            switch_inst->cur_channel = switch_channel;
        }
    }
    /*
     * Write
     */
    status = adapter->write(adapter, base_addr, type, slaveAddr, buf, len, flags); // APP_SRTM_I2C_Write
    return status;
}
static srtm_status_t SRTM_I2CService_Init(srtm_service_t service, uint8_t busID, uint8_t *buf, uint16_t len)
{
    srtm_i2c_service_t handle  = (srtm_i2c_service_t)(void *)service;
    srtm_i2c_adapter_t adapter = handle->adapter;
    struct srtm_i2c_init_payload init;
    srtm_status_t status;

    if (len != sizeof(init))
    {
        return SRTM_Status_InvalidParameter;
    }

    memcpy(&init, buf, sizeof(init));

    status = adapter->init(adapter, busID, &init);
    return status;
}

srtm_service_t SRTM_I2CService_Create(srtm_i2c_adapter_t adapter)
{
    srtm_i2c_service_t handle;

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);
    handle = (srtm_i2c_service_t)SRTM_Heap_Malloc(sizeof(struct _srtm_i2c_service));
    assert(handle);

    adapter->service = &handle->service;
    handle->adapter  = adapter;

    SRTM_List_Init(&handle->service.node);
    handle->service.dispatcher = NULL;
    handle->service.category   = SRTM_I2C_CATEGORY;
    handle->service.destroy    = SRTM_I2CService_Destroy;
    handle->service.request    = SRTM_I2CService_Request;
    handle->service.notify     = SRTM_I2CService_Notify;

    return &handle->service;
}

void SRTM_I2CService_Destroy(srtm_service_t service)
{
    srtm_i2c_service_t handle = (srtm_i2c_service_t)(void *)service;

    assert(service);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    /* Service must be unregistered from dispatcher before destroy */
    assert(SRTM_List_IsEmpty(&service->node));

    SRTM_Heap_Free(handle);
}

void SRTM_I2CService_Reset(srtm_service_t service, srtm_peercore_t core)
{
    assert(service);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);
}

static srtm_status_t SRTM_I2CService_Request(srtm_service_t service, srtm_request_t request)
{
    srtm_status_t status;
    srtm_channel_t channel;
    uint8_t retCode = SRTM_I2C_RETURN_CODE_SUCCESS;
    uint8_t command;
    uint16_t responseLen = 0, requestLen = 0;
    uint32_t payloadLen;
    srtm_response_t response;
    struct _srtm_i2c_payload *i2cReq;
    struct _srtm_i2c_payload *i2cResp;

    assert(service->dispatcher);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    channel    = SRTM_CommMessage_GetChannel(request);
    command    = SRTM_CommMessage_GetCommand(request);
    i2cReq     = (struct _srtm_i2c_payload *)(void *)SRTM_CommMessage_GetPayload(request);
    payloadLen = SRTM_CommMessage_GetPayloadLen(request);
    (void)payloadLen; /* try to fix warning: variable 'payloadLen' set but not used */
    assert(i2cReq);
    switch (command)
    {
        case SRTM_I2C_CMD_READ:
            responseLen = i2cReq->len;
            break;
        case SRTM_I2C_CMD_WRITE:
        case SRTM_I2C_CMD_INIT:
            requestLen = i2cReq->len;
            break;
        default:
            // bad req will fail with unsupported below
            break;
    }
    if (payloadLen < sizeof(struct _srtm_i2c_payload) + requestLen ||
        requestLen + sizeof(struct _srtm_i2c_payload) + sizeof(struct _srtm_packet_head) > SRTM_DISPATCHER_MSG_MAX_LEN)
    {
        retCode     = SRTM_I2C_RETURN_CODE_EINVAL;
        responseLen = 0;
    }

retry_alloc:
    response = SRTM_Response_Create(channel, SRTM_I2C_CATEGORY, SRTM_I2C_VERSION, command,
                                    sizeof(struct _srtm_i2c_payload) + responseLen);
    if (response == NULL)
    {
        PRINTF("i2c rpmsg out of memory (%d)\r\n", sizeof(struct _srtm_i2c_payload) + responseLen);
        if (retCode == SRTM_I2C_RETURN_CODE_SUCCESS)
        {
            // retry once with smaller buffer to signal error
            retCode     = SRTM_I2C_RETURN_CODE_ENOMEM;
            responseLen = 0;
            goto retry_alloc;
        }
        return SRTM_Status_OutOfMemory;
    }

    i2cResp = (struct _srtm_i2c_payload *)(void *)SRTM_CommMessage_GetPayload(response);

    if (retCode != SRTM_I2C_RETURN_CODE_SUCCESS)
    {
        i2cResp->retCode = retCode;
        goto out;
    }

    status = SRTM_Service_CheckVersion(service, request, SRTM_I2C_VERSION);
    if (status != SRTM_Status_Success)
    {
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: format error!\r\n", __func__);
        i2cResp->retCode = SRTM_I2C_RETURN_CODE_UNSUPPORTED;
        goto out;
    }

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO,
                       "SRTM receive I2C request:cmd=%x, busID %d, slaveAddr 0x%x!, data %d bytes\r\n", command,
                       i2cReq->busID, i2cReq->slaveAddr, i2cReq->len);
    (void)memcpy(i2cResp, i2cReq, sizeof(struct _srtm_i2c_payload));
    i2cResp->len = responseLen;

    switch (command)
    {
        case SRTM_I2C_CMD_READ:
            status = SRTM_I2CService_ReadBus(service, i2cResp->busID, i2cResp->slaveAddr, i2cResp->data, responseLen,
                                             i2cReq->flags);
            i2cResp->retCode = status == SRTM_Status_Success ? SRTM_I2C_RETURN_CODE_SUCCESS : SRTM_I2C_RETURN_CODE_FAIL;
            break;
        case SRTM_I2C_CMD_WRITE:
            status = SRTM_I2CService_WriteBus(service, i2cResp->busID, i2cResp->slaveAddr, i2cReq->data, requestLen,
                                              i2cReq->flags);
            i2cResp->retCode = status == SRTM_Status_Success ? SRTM_I2C_RETURN_CODE_SUCCESS : SRTM_I2C_RETURN_CODE_FAIL;
            break;
        case SRTM_I2C_CMD_INIT:
            status           = SRTM_I2CService_Init(service, i2cResp->busID, i2cReq->data, requestLen);
            i2cResp->retCode = status == SRTM_Status_Success ? SRTM_I2C_RETURN_CODE_SUCCESS : SRTM_I2C_RETURN_CODE_FAIL;
            break;

        default:
            SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported!\r\n", __func__, command);
            i2cResp->retCode = SRTM_I2C_RETURN_CODE_UNSUPPORTED;
            break;
    }

out:
    return SRTM_Dispatcher_DeliverResponse(service->dispatcher, response);
}

static srtm_status_t SRTM_I2CService_Notify(srtm_service_t service, srtm_notification_t notif)
{
    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported!\r\n", __func__,
                       SRTM_CommMessage_GetCommand(notif));

    return SRTM_Status_ServiceNotFound;
}
