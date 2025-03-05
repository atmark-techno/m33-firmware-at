/*
 * Copyright 2018-2021, NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

#include "srtm_service.h"

/*!
 * @addtogroup srtm_service
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** @brief Switch to disable I2C service debugging messages. */
#ifndef SRTM_I2C_SERVICE_DEBUG_OFF
#define SRTM_I2C_SERVICE_DEBUG_OFF (0)
#endif

#if SRTM_I2C_SERVICE_DEBUG_OFF
#undef SRTM_DEBUG_VERBOSE_LEVEL
#define SRTM_DEBUG_VERBOSE_LEVEL SRTM_DEBUG_VERBOSE_NONE
#endif

/* Protocol definition */
#define SRTM_I2C_FLAG_NEED_STOP (0x200U)

/* I2C Service Notification Command definition */

typedef enum
{
    /* I2C Service Request Command definition */
    SRTM_I2C_CMD_READ = 0U,
    SRTM_I2C_CMD_WRITE,
    SRTM_I2C_CMD_INIT,
} srtm_i2c_cmd_t;

struct srtm_i2c_init_payload
{
    uint32_t i2c_type;
    uint32_t i2c_index;
    uint32_t baudrate;
    union
    {
        // nothing for lpi2c
        struct
        {
            uint32_t scl_pin;
            uint32_t sda_pin;
        } flexio;
    };
};

/**
 * @brief SRTM I2C payload structure
 */
SRTM_ANON_DEC_BEGIN
SRTM_PACKED_BEGIN struct _srtm_i2c_payload
{
    uint8_t busID;
    union
    {
        uint8_t reserved; /* used in request packet */
        uint8_t retCode;  /* used in response packet */
    };
    uint16_t slaveAddr;
    uint16_t flags;
    uint16_t len;
    uint8_t data[0]; /* data size is decided by uint16_t len */
} SRTM_PACKED_END;
SRTM_ANON_DEC_END

typedef struct _srtm_i2c_payload *srtm_i2c_payload_t;

typedef enum
{
    SRTM_I2C_TYPE_LPI2C = 0U,
    SRTM_I2C_TYPE_FLEXIO_I2C,
} srtm_i2c_type_t;

static inline const char *srtm_i2c_type_to_str(srtm_i2c_type_t type)
{
    switch (type)
    {
        case SRTM_I2C_TYPE_LPI2C:
            return "lpi2c";
        case SRTM_I2C_TYPE_FLEXIO_I2C:
            return "flexio";
        default:
            return "???";
    }
}

typedef enum
{
    SRTM_I2C_SWITCH_CHANNEL0 = 0U,
    SRTM_I2C_SWITCH_CHANNEL1,
    SRTM_I2C_SWITCH_CHANNEL2,
    SRTM_I2C_SWITCH_CHANNEL3,
    SRTM_I2C_SWITCH_CHANNEL_UNSPECIFIED
} srtm_i2c_switch_channel;

typedef struct _i2c_bus
{
    uint8_t bus_id;
    uint32_t base_addr;
    srtm_i2c_type_t type;
    uint8_t switch_idx;
    srtm_i2c_switch_channel switch_channel;
    uint32_t baudrate;
} * i2c_bus_t;

typedef struct _i2c_switch
{
    uint16_t slaveAddr;
    srtm_i2c_switch_channel cur_channel;
} * i2c_switch_t;

typedef struct _i2c_bus_structure
{
    i2c_bus_t buses;
    uint8_t bus_num;
    i2c_switch_t switches;
    uint8_t switch_num;
} i2c_bus_structure_t;

/**
 * @brief SRTM I2C adapter structure pointer.
 */
typedef struct _srtm_i2c_adapter *srtm_i2c_adapter_t;

/**
 * @brief SRTM I2C adapter structure
 */
struct _srtm_i2c_adapter
{
    /* Bound service */
    srtm_service_t service;

    i2c_bus_structure_t bus_structure;

    /* Interfaces implemented by I2C adapter */
    srtm_status_t (*read)(srtm_i2c_adapter_t adapter, i2c_bus_t bus, uint16_t slaveAddr, uint8_t *buf, uint16_t len,
                          uint16_t flags);
    srtm_status_t (*write)(srtm_i2c_adapter_t adapter, i2c_bus_t bus, uint16_t slaveAddr, uint8_t *buf, uint16_t len,
                           uint16_t flags);
    srtm_status_t (*init)(srtm_i2c_adapter_t adapter, int bus_id, struct srtm_i2c_init_payload *init);
    srtm_status_t (*switchchannel)(srtm_i2c_adapter_t adapter, i2c_bus_t bus, uint16_t slaveAddr,
                                   srtm_i2c_switch_channel channel);
};
/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
/*!
 * @brief Create I2C service.
 *
 * @param adapter I2C adapter to provide real I2C features.
 * @return SRTM service handle on success and NULL on failure.
 */
srtm_service_t SRTM_I2CService_Create(srtm_i2c_adapter_t adapter);

/*!
 * @brief Destroy I2C service.
 *
 * @param service SRTM service to destroy.
 */
void SRTM_I2CService_Destroy(srtm_service_t service);

/*!
 * @brief Reset I2C service. This is used to stop sending events and return to initial state.
 *
 * @param service SRTM service to reset.
 * @param core Identify which core is to be reset.
 */
void SRTM_I2CService_Reset(srtm_service_t service, srtm_peercore_t core);
#ifdef __cplusplus
}
#endif

/*! @} */
