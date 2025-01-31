/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_reset.h"
#include "fsl_lpi2c_freertos.h"
#include "fsl_flexio_i2c_master.h"

#include "board.h"
#include "app_srtm_internal.h"
#include "srtm_i2c_service.h"

#define I2C_MAX_BUSES 4

static srtm_status_t i2c_read(srtm_i2c_adapter_t adapter, uint32_t base_addr, srtm_i2c_type_t type, uint16_t slaveAddr,
                              uint8_t *buf, uint16_t len, uint16_t flags);
static srtm_status_t i2c_write(srtm_i2c_adapter_t adapter, uint32_t base_addr, srtm_i2c_type_t type, uint16_t slaveAddr,
                               uint8_t *buf, uint16_t len, uint16_t flags);
static srtm_status_t i2c_init(srtm_i2c_adapter_t adapter, int bus_id, struct srtm_i2c_init_payload *init);
static srtm_status_t i2c_switchChannel(srtm_i2c_adapter_t adapter, uint32_t base_addr, srtm_i2c_type_t type,
                                       uint16_t slaveAddr, srtm_i2c_switch_channel channel);

static srtm_service_t i2cService;

/* flexio i2c */
struct flexio_i2c
{
    FLEXIO_I2C_Type dev;
    flexio_i2c_master_handle_t handle;
    bool completionFlag;
    bool nakFlag;
};

static void flexio_i2c_master_Callback(FLEXIO_I2C_Type *base, flexio_i2c_master_handle_t *handle, status_t status,
                                       void *userData)
{
    struct flexio_i2c *flexio = userData;

    if (status == kStatus_Success)
    {
        flexio->completionFlag = true;
    }

    if (status == kStatus_FLEXIO_I2C_Nak)
    {
        flexio->nakFlag = true;
    }
}
/* end of flexio i2c */

static struct _i2c_bus platform_i2c_buses[I2C_MAX_BUSES];

static struct _srtm_i2c_adapter i2c_adapter = { .read          = i2c_read,
                                                .write         = i2c_write,
                                                .init          = i2c_init,
                                                .switchchannel = i2c_switchChannel,
                                                .bus_structure = {
                                                    .buses   = platform_i2c_buses,
                                                    .bus_num = 0,
                                                } };

/**********************************************************
 * SRTM callbacks
 *********************************************************/

static srtm_status_t i2c_read(srtm_i2c_adapter_t adapter, uint32_t base_addr, srtm_i2c_type_t type, uint16_t slaveAddr,
                              uint8_t *buf, uint16_t len, uint16_t flags)
{
    status_t retVal = kStatus_Fail;
    uint32_t needStop;

    switch (type)
    {
        case SRTM_I2C_TYPE_LPI2C:
            needStop = (flags & SRTM_I2C_FLAG_NEED_STOP) ? kLPI2C_TransferDefaultFlag : kLPI2C_TransferNoStopFlag;
            retVal   = BOARD_LPI2C_Receive((LPI2C_Type *)base_addr, slaveAddr, 0, 0, buf, len, needStop);
            break;
        case SRTM_I2C_TYPE_FLEXIO_I2C:
        {
            // flexio_i2c has no NoStop flag... print error if used?
            flexio_i2c_master_transfer_t xfer = {
                .direction    = kFLEXIO_I2C_Read,
                .slaveAddress = slaveAddr,
                // subaddress, subAddressSize, flags = 0,
                .data     = buf,
                .dataSize = len,
            };
            struct flexio_i2c *flexio = (void *)base_addr;

            retVal = FLEXIO_I2C_MasterTransferNonBlocking(&flexio->dev, &flexio->handle, &xfer);
            while ((!flexio->nakFlag) && (!flexio->completionFlag))
            {
            }
            if (flexio->nakFlag)
                retVal = kStatus_FLEXIO_I2C_Nak;

            flexio->nakFlag        = false;
            flexio->completionFlag = false;
            break;
        }
        default:
            break;
    }
    return (retVal == kStatus_Success) ? SRTM_Status_Success : SRTM_Status_TransferFailed;
}
static srtm_status_t i2c_write(srtm_i2c_adapter_t adapter, uint32_t base_addr, srtm_i2c_type_t type, uint16_t slaveAddr,
                               uint8_t *buf, uint16_t len, uint16_t flags)
{
    status_t retVal = kStatus_Fail;
    uint32_t needStop;

    switch (type)
    {
        case SRTM_I2C_TYPE_LPI2C:
            needStop = (flags & SRTM_I2C_FLAG_NEED_STOP) ? kLPI2C_TransferDefaultFlag : kLPI2C_TransferNoStopFlag;
            retVal   = BOARD_LPI2C_Send((LPI2C_Type *)base_addr, slaveAddr, 0, 0, buf, len, needStop);
            break;
        case SRTM_I2C_TYPE_FLEXIO_I2C:
        {
            // flexio_i2c has no NoStop flag... print error if used?
            flexio_i2c_master_transfer_t xfer = {
                .direction    = kFLEXIO_I2C_Write,
                .slaveAddress = slaveAddr,
                // subaddress, subAddressSize, flags = 0,
                .data     = buf,
                .dataSize = len,
            };
            struct flexio_i2c *flexio = (void *)base_addr;

            retVal = FLEXIO_I2C_MasterTransferNonBlocking(&flexio->dev, &flexio->handle, &xfer);
            while ((!flexio->nakFlag) && (!flexio->completionFlag))
            {
            }
            if (flexio->nakFlag)
                retVal = kStatus_FLEXIO_I2C_Nak;

            flexio->nakFlag        = false;
            flexio->completionFlag = false;
            break;
        }
    }
    return (retVal == kStatus_Success) ? SRTM_Status_Success : SRTM_Status_TransferFailed;
}

static srtm_status_t i2c_switchChannel(srtm_i2c_adapter_t adapter, uint32_t base_addr, srtm_i2c_type_t type,
                                       uint16_t slaveAddr, srtm_i2c_switch_channel channel)
{
    return SRTM_Status_Error; // error is ignored...
}

static srtm_status_t i2c_init_device(int i)
{
    switch (platform_i2c_buses[i].type)
    {
        case SRTM_I2C_TYPE_LPI2C:
        {
            lpi2c_master_config_t masterConfig;
            uint32_t clock_freq;
            LPI2C_MasterGetDefaultConfig(&masterConfig);

            masterConfig.baudRate_Hz = platform_i2c_buses[i].baudrate;
            switch (platform_i2c_buses[i].base_addr)
            {
                case LPI2C0_BASE:
                    CLOCK_SetIpSrcDiv(kCLOCK_Lpi2c0, kCLOCK_Pcc1BusIpSrcCm33Bus, 0U, 0U);
                    clock_freq = CLOCK_GetIpFreq(kCLOCK_Lpi2c0);
                    RESET_PeripheralReset(kRESET_Lpi2c0);
                    break;
                case LPI2C1_BASE:
                    CLOCK_SetIpSrcDiv(kCLOCK_Lpi2c1, kCLOCK_Pcc1BusIpSrcCm33Bus, 0U, 0U);
                    clock_freq = CLOCK_GetIpFreq(kCLOCK_Lpi2c1);
                    RESET_PeripheralReset(kRESET_Lpi2c1);
                    break;
                default:
                    return SRTM_Status_Error;
            }
            LPI2C_MasterInit((LPI2C_Type *)platform_i2c_buses[i].base_addr, &masterConfig, clock_freq);
            break;
        }
        case SRTM_I2C_TYPE_FLEXIO_I2C:
        {
            flexio_i2c_master_config_t flexConfig;
            struct flexio_i2c *flexio = (void *)platform_i2c_buses[i].base_addr;
            CLOCK_SetIpSrcDiv(kCLOCK_Flexio0, kCLOCK_Pcc0BusIpSrcCm33Bus, 0U, 0U);
            uint32_t clock_freq = CLOCK_GetIpFreq(kCLOCK_Flexio0);
            RESET_PeripheralReset(kRESET_Flexio0);
            FLEXIO_I2C_MasterGetDefaultConfig(&flexConfig);
            flexConfig.baudRate_Bps = platform_i2c_buses[i].baudrate;
            FLEXIO_I2C_MasterInit(&flexio->dev, &flexConfig, clock_freq);
            FLEXIO_I2C_MasterTransferCreateHandle(&flexio->dev, &flexio->handle, flexio_i2c_master_Callback, flexio);
            break;
        }
        default:
            return SRTM_Status_Error;
    }
    return SRTM_Status_Success;
}

static srtm_status_t i2c_init(srtm_i2c_adapter_t adapter, int bus_id, struct srtm_i2c_init_payload *init)
{
    int i;

    for (i = 0; i < adapter->bus_structure.bus_num; i++)
    {
        if (platform_i2c_buses[i].bus_id == bus_id)
        {
            PRINTF("i2c bus %d already init\r\n", bus_id);
            return SRTM_Status_Error;
        }
    }
    if (adapter->bus_structure.bus_num >= I2C_MAX_BUSES)
    {
        PRINTF("Too many i2c buses (max %d)\r\n", I2C_MAX_BUSES);
        return SRTM_Status_Error;
    }

    PRINTF("initializing i2c %d\r\n", bus_id);

    i = adapter->bus_structure.bus_num;

    platform_i2c_buses[i].bus_id   = bus_id;
    platform_i2c_buses[i].type     = init->i2c_type;
    platform_i2c_buses[i].baudrate = init->baudrate;

    switch (init->i2c_type)
    {
        case SRTM_I2C_TYPE_LPI2C:
            switch (init->i2c_index)
            {
                case 0:
                    platform_i2c_buses[i].base_addr = LPI2C0_BASE;
                    break;
                case 1:
                    platform_i2c_buses[i].base_addr = LPI2C1_BASE;
                    break;
                default:
                    PRINTF("I2C %d invalid i2c index %d\r\n", init->i2c_index);
                    return SRTM_Status_Error;
            }
            break;
        case SRTM_I2C_TYPE_FLEXIO_I2C:
        {
            if (init->i2c_index != 0)
            {
                PRINTF("I2C %d invalid i2c index %d\r\n", init->i2c_index);
                return SRTM_Status_Error;
            }
            struct flexio_i2c *flexio;
            flexio = pvPortMalloc(sizeof(*flexio));
            if (!flexio)
                return SRTM_Status_Error;

            memset(flexio, 0, sizeof(*flexio));
            flexio->dev.flexioBase      = FLEXIO0;
            flexio->dev.SCLPinIndex     = init->flexio.scl_pin;
            flexio->dev.SDAPinIndex     = init->flexio.sda_pin;
            flexio->dev.shifterIndex[1] = 1;
            flexio->dev.timerIndex[1]   = 1;
            flexio->dev.timerIndex[2]   = 2;

            platform_i2c_buses[i].base_addr = (uint32_t)flexio;
            break;
        }
        default:
            PRINTF("I2C %d invalid type %d\r\n", bus_id, init->i2c_type);
            return SRTM_Status_Error;
    }

    srtm_status_t rc = i2c_init_device(i);

    if (rc)
    {
        PRINTF("I2C %d device init failed\r\n", bus_id);
        if (init->i2c_type == SRTM_I2C_TYPE_FLEXIO_I2C)
        {
            vPortFree((void *)platform_i2c_buses[i].base_addr);
        }
        return rc;
    }

    adapter->bus_structure.bus_num++;

    return 0;
}

/**********************************************************
 * init/PM hooks called from APP_SRTM
 *********************************************************/
void APP_I2C_InitService(void)
{
    i2cService = SRTM_I2CService_Create(&i2c_adapter);
    SRTM_Dispatcher_RegisterService(disp, i2cService);
}

void APP_I2C_Resume(void)
{
    int i;

    for (i = 0; i < i2c_adapter.bus_structure.bus_num; i++)
    {
        i2c_init_device(i);
    }
}
