/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>

#include "fsl_reset.h"
#include "fsl_lpi2c_freertos.h"
#include "fsl_flexio_i2c_master.h"

#include "board.h"
#include "app_srtm_internal.h"
#include "srtm_i2c_service.h"
#include "app_uboot.h"

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

static int i2c_find_bus(int bus_id)
{
    int i;

    for (i = 0; i < i2c_adapter.bus_structure.bus_num; i++)
    {
        if (platform_i2c_buses[i].bus_id == bus_id)
            return i;
    }

    return -1;
}

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

static void i2c_reset_device(int i)
{
    switch (platform_i2c_buses[i].type)
    {
        case SRTM_I2C_TYPE_LPI2C:
        {
            switch (platform_i2c_buses[i].base_addr)
            {
                case LPI2C0_BASE:
                    // XXX disable clock?
                    RESET_PeripheralReset(kRESET_Lpi2c0);
                    break;
                case LPI2C1_BASE:
                    RESET_PeripheralReset(kRESET_Lpi2c1);
                    break;
            }
            break;
        }
        case SRTM_I2C_TYPE_FLEXIO_I2C:
        {
            RESET_PeripheralReset(kRESET_Flexio0);
            break;
        }
    }
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

    i = i2c_find_bus(bus_id);
    if (i >= 0)
    {
        PRINTF("i2c bus %d already init\r\n", bus_id);
        return SRTM_Status_Error;
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

            if (flexio_used)
            {
                PRINTF("i2c: flexio %d already used by another driver, refusing to init\r\n", init->i2c_index);
                return kStatus_Fail;
            }
            flexio_used = true;

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

void APP_I2C_ResetService(void)
{
    int i;

    for (i = 0; i < i2c_adapter.bus_structure.bus_num; i++)
    {
        if (platform_i2c_buses[i].bus_id == 0xff)
            continue;
        i2c_reset_device(i);
    }
    i2c_adapter.bus_structure.bus_num = 0;
}

void APP_I2C_uboot(uint32_t command)
{
    uint8_t subcommand = (command >> 8) & 0xff;
    uint8_t bus_id     = (command >> 16) & 0xff;
    int rc;

    switch (subcommand)
    {
        case UBOOT_I2C_INIT:
        {
            struct srtm_i2c_init_payload init = { 0 };

            init.i2c_type  = uboot_recv();
            init.i2c_index = uboot_recv();
            init.baudrate  = uboot_recv();

            /* only support lpi2c for now but flexio support would be trivial */
            if (init.i2c_type != SRTM_I2C_TYPE_LPI2C)
            {
                uboot_send(EINVAL);
                return;
            }

            rc = i2c_init(&i2c_adapter, bus_id, &init);
            uboot_send(rc);
            break;
        }
        case UBOOT_I2C_RESET:
        {
            int i = i2c_find_bus(bus_id);
            if (i < 0)
            {
                uboot_send(EINVAL);
                return;
            }

            i2c_reset_device(i);

            /* mark as reset and also shrink array if it was last */
            platform_i2c_buses[i].bus_id = 0xff;
            if (i == i2c_adapter.bus_structure.bus_num - 1)
                i2c_adapter.bus_structure.bus_num--;

            uboot_send(0);
            break;
        }
        case UBOOT_I2C_READ:
        {
            /* buffer on the stack so limit to something sane.
             * If needed later we could allocate a bigger buffer... */
            uint8_t buf[32];
            uint32_t addr;
            uint32_t len;

            addr = uboot_recv();
            len  = uboot_recv();
            if (len > sizeof(buf))
            {
                PRINTF("uboot i2c read too big %d\r\n", len);
                uboot_send(EINVAL);
                return;
            }

            // this needs to be after all uboot_recv()s to avoid getting stuck
            int i = i2c_find_bus(bus_id);
            if (i < 0)
            {
                uboot_send(EINVAL);
                return;
            }

            rc = i2c_read(&i2c_adapter, platform_i2c_buses[i].base_addr, platform_i2c_buses[i].type, addr, buf, len, 0);

            uboot_send(rc);
            if (rc)
                return;

            uboot_send_many(buf, len);
            break;
        }
        case UBOOT_I2C_WRITE:
        {
            /* buffer on the stack so limit to something sane.
             * If needed later we could allocate a bigger buffer... */
            uint8_t buf[32];
            uint32_t addr;
            uint32_t len;

            addr = uboot_recv();
            len  = uboot_recv();
            if (len > sizeof(buf))
            {
                PRINTF("uboot i2c write too big %d\r\n", len);
                // we still need to recv this to not get uboot stuck...
                len = (len + 3) / 4;
                while (len > 0)
                    (void)uboot_recv();
                uboot_send(EINVAL);
                return;
            }
            uboot_recv_many(buf, len);

            int i = i2c_find_bus(bus_id);
            if (i < 0)
            {
                uboot_send(EINVAL);
                return;
            }

            rc =
                i2c_write(&i2c_adapter, platform_i2c_buses[i].base_addr, platform_i2c_buses[i].type, addr, buf, len, 0);

            uboot_send(rc);
            break;
        }
        default:
            MU_SendMsg(MU0_MUA, 0, EINVAL);
            break;
    }
}
