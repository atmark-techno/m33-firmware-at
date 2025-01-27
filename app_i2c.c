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

#define LPI2C0_BAUDRATE (400000)
#define I2C_SOURCE_CLOCK_FREQ_LPI2C0 CLOCK_GetIpFreq(kCLOCK_Lpi2c0)

#define LPI2C1_BAUDRATE (100000)
#define I2C_SOURCE_CLOCK_FREQ_LPI2C1 CLOCK_GetIpFreq(kCLOCK_Lpi2c1)

#define I2C_SWITCH_NONE 1

static srtm_status_t i2c_read(srtm_i2c_adapter_t adapter, uint32_t base_addr, srtm_i2c_type_t type, uint16_t slaveAddr,
                              uint8_t *buf, uint16_t len, uint16_t flags);

static srtm_status_t i2c_write(srtm_i2c_adapter_t adapter, uint32_t base_addr, srtm_i2c_type_t type, uint16_t slaveAddr,
                               uint8_t *buf, uint16_t len, uint16_t flags);

static srtm_status_t i2c_switchChannel(srtm_i2c_adapter_t adapter, uint32_t base_addr, srtm_i2c_type_t type,
                                       uint16_t slaveAddr, srtm_i2c_switch_channel channel);

static srtm_service_t i2cService;

/* flexio i2c */
// uses maximum I2C speed from fastmode, 400k
#define FLEXIO_I2C_FREQ 400000
#define FLEXIO_CLOCK_FREQUENCY CLOCK_GetIpFreq(kCLOCK_Flexio0)
FLEXIO_I2C_Type flexioI2cDev = {
    .flexioBase = FLEXIO0,
    .SCLPinIndex = 20, // PTB4
    .SDAPinIndex = 21, // PTB5
    .shifterIndex = { 0, 1, },
    .timerIndex = { 0, 1, 2, },
};
flexio_i2c_master_handle_t flexio_i2c_handle;
volatile bool i2c_completionFlag = false;
volatile bool i2c_nakFlag        = false;

static void flexio_i2c_master_Callback(FLEXIO_I2C_Type *base, flexio_i2c_master_handle_t *handle, status_t status,
                                       void *userData)
{
    if (status == kStatus_Success)
    {
        i2c_completionFlag = true;
    }

    if (status == kStatus_FLEXIO_I2C_Nak)
    {
        i2c_nakFlag = true;
    }
}
/* end of flexio i2c */

static struct _i2c_bus platform_i2c_buses[] = {
    { .bus_id         = 0,
      .base_addr      = LPI2C0_BASE,
      .type           = SRTM_I2C_TYPE_LPI2C,
      .switch_idx     = I2C_SWITCH_NONE,
      .switch_channel = SRTM_I2C_SWITCH_CHANNEL_UNSPECIFIED },
    { .bus_id         = 1,
      .base_addr      = LPI2C1_BASE,
      .type           = SRTM_I2C_TYPE_LPI2C,
      .switch_idx     = I2C_SWITCH_NONE,
      .switch_channel = SRTM_I2C_SWITCH_CHANNEL_UNSPECIFIED },
    // LPI2C2 as flexio
    { .bus_id         = 2,
      .base_addr      = (uint32_t)&flexioI2cDev,
      .type           = SRTM_I2C_TYPE_FLEXIO_I2C,
      .switch_idx     = I2C_SWITCH_NONE,
      .switch_channel = SRTM_I2C_SWITCH_CHANNEL_UNSPECIFIED },
};

static struct _srtm_i2c_adapter i2c_adapter = { .read          = i2c_read,
                                                .write         = i2c_write,
                                                .switchchannel = i2c_switchChannel,
                                                .bus_structure = {
                                                    .buses      = platform_i2c_buses,
                                                    .bus_num    = sizeof(platform_i2c_buses) / sizeof(struct _i2c_bus),
                                                    .switch_num = 0,
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

            retVal = FLEXIO_I2C_MasterTransferNonBlocking((FLEXIO_I2C_Type *)base_addr, &flexio_i2c_handle, &xfer);
            while ((!i2c_nakFlag) && (!i2c_completionFlag))
            {
            }
            if (i2c_nakFlag)
                retVal = kStatus_FLEXIO_I2C_Nak;

            i2c_nakFlag        = false;
            i2c_completionFlag = false;
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

            retVal = FLEXIO_I2C_MasterTransferNonBlocking((FLEXIO_I2C_Type *)base_addr, &flexio_i2c_handle, &xfer);
            while ((!i2c_nakFlag) && (!i2c_completionFlag))
            {
            }
            if (i2c_nakFlag)
                retVal = kStatus_FLEXIO_I2C_Nak;

            i2c_nakFlag        = false;
            i2c_completionFlag = false;
            break;
        }
    }
    return (retVal == kStatus_Success) ? SRTM_Status_Success : SRTM_Status_TransferFailed;
}

static srtm_status_t i2c_switchChannel(srtm_i2c_adapter_t adapter, uint32_t base_addr, srtm_i2c_type_t type,
                                       uint16_t slaveAddr, srtm_i2c_switch_channel channel)
{
    uint8_t txBuff[1];
    assert(channel < SRTM_I2C_SWITCH_CHANNEL_UNSPECIFIED);
    txBuff[0] = 1 << (uint8_t)channel;
    return adapter->write(adapter, base_addr, type, slaveAddr, txBuff, sizeof(txBuff),
                          SRTM_I2C_FLAG_NEED_STOP); // i2c_Write
}

static void i2c_init_device(void)
{
    lpi2c_master_config_t masterConfig;

    LPI2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Hz = LPI2C0_BAUDRATE;
    LPI2C_MasterInit(LPI2C0, &masterConfig, I2C_SOURCE_CLOCK_FREQ_LPI2C0);
    masterConfig.baudRate_Hz = LPI2C1_BAUDRATE;
    LPI2C_MasterInit(LPI2C1, &masterConfig, I2C_SOURCE_CLOCK_FREQ_LPI2C1);

    /* flexio i2c */
    flexio_i2c_master_config_t flexConfig;

    CLOCK_SetIpSrcDiv(kCLOCK_Flexio0, kCLOCK_Pcc0BusIpSrcCm33Bus, 0U, 0U);
    RESET_PeripheralReset(kRESET_Flexio0);

    FLEXIO_I2C_MasterGetDefaultConfig(&flexConfig);
    flexConfig.baudRate_Bps = FLEXIO_I2C_FREQ;
    FLEXIO_I2C_MasterInit(&flexioI2cDev, &flexConfig, FLEXIO_CLOCK_FREQUENCY);
    FLEXIO_I2C_MasterTransferCreateHandle(&flexioI2cDev, &flexio_i2c_handle, flexio_i2c_master_Callback, NULL);
}

/**********************************************************
 * init/PM hooks called from APP_SRTM
 *********************************************************/
void APP_I2C_InitService(void)
{
    i2c_init_device();
    i2cService = SRTM_I2CService_Create(&i2c_adapter);
    SRTM_Dispatcher_RegisterService(disp, i2cService);
}

void APP_I2C_Resume(void)
{
    i2c_init_device();
}
