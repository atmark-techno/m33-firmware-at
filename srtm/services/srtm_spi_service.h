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
/** @brief Switch to disable SPI service debugging messages. */
#ifndef SRTM_SPI_SERVICE_DEBUG_OFF
#define SRTM_SPI_SERVICE_DEBUG_OFF (0)
#endif

#if SRTM_SPI_SERVICE_DEBUG_OFF
#undef SRTM_DEBUG_VERBOSE_LEVEL
#define SRTM_DEBUG_VERBOSE_LEVEL SRTM_DEBUG_VERBOSE_NONE
#endif

/* Protocol definition */
#define SRTM_SPI_FLAG_NEED_STOP (0x200U)

/* SPI Service Notification Command definition */

enum srtm_spi_retcodes
{
    SRTM_SPI_RETCODE_SUCCESS,
    SRTM_SPI_RETCODE_FAIL,
    SRTM_SPI_RETCODE_EBUSY,
    SRTM_SPI_RETCODE_UNSUPPORTED,
    SRTM_SPI_RETCODE_EINVAL,
    SRTM_SPI_RETCODE_ENOMEM,
};

typedef enum
{
    /* SPI Service Request Command definition */
    SRTM_SPI_CMD_TRANSFER = 0U,
    SRTM_SPI_CMD_INIT,
    SRTM_SPI_CMD_SET_MODE,
    SRTM_SPI_CMD_TRANSFER_CONTINUOUS,
} srtm_spi_cmd_t;

typedef enum
{
    SRTM_SPI_TYPE_GPIO  = 0U,
    SRTM_SPI_TYPE_LPSPI = 1U,
    // FLEXIO..
    SRTM_SPI_TYPES_COUNT,
} srtm_spi_type_t;

struct srtm_spi_init_payload
{
    srtm_spi_type_t type;
    union
    {
        struct srtm_spi_init_gpio_payload
        {
            uint32_t sck_pin;
            uint32_t miso_pin;
            uint32_t mosi_pin;
            /* It exists to maintain compatibility, but it is not used. */
            uint32_t mode;
        } gpio;
        struct srtm_spi_init_lpspi_payload
        {
            uint32_t spi_index;
            /* It exists to maintain compatibility, but 0 is always passed. */
            uint32_t mode;
        } lpspi;
    };
};

/**
 * @brief SRTM SPI payload structure
 */
SRTM_ANON_DEC_BEGIN
SRTM_PACKED_BEGIN struct _srtm_spi_payload
{
    uint8_t busID;
    union
    {
        uint8_t reserved; /* unused in request packet */
        uint8_t retCode;  /* used in response packet */
    };
    uint16_t bits_per_word;
    uint32_t speed_hz;
    uint16_t len;    /* in bytes (e.g. 2 bytes if bits_per_word=12, 4 if 20) */
    uint8_t data[0]; /* data size is decided by uint16_t len */
} SRTM_PACKED_END;
SRTM_ANON_DEC_END

typedef struct _srtm_spi_payload *srtm_spi_payload_t;

static inline const char *srtm_spi_type_to_str(srtm_spi_type_t type)
{
    switch (type)
    {
        case SRTM_SPI_TYPE_GPIO:
            return "gpio";
        case SRTM_SPI_TYPE_LPSPI:
            return "lpspi";
        default:
            return "???";
    }
}

struct _srtm_spi_service;
typedef uint8_t (*srtm_spi_init_t)(uint8_t bus_id, struct srtm_spi_init_payload *init);
typedef uint8_t (*srtm_spi_transfer_t)(srtm_response_t response, uint8_t bus_id, uint16_t bits_per_word,
                                       uint32_t speed_hz, uint16_t len, uint8_t *tx_buf, uint8_t *rx_buf,
                                       bool continuous);
typedef uint8_t (*srtm_spi_set_mode_t)(uint8_t bus_id, uint32_t mode);

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
/*!
 * @brief Create SPI service.
 *
 * @param adapter SPI adapter to provide real SPI features.
 * @return SRTM service handle on success and NULL on failure.
 */
srtm_service_t SRTM_SPIService_Create(srtm_spi_init_t init, srtm_spi_transfer_t transfer, srtm_spi_set_mode_t set_mode);

/*!
 * @brief Destroy SPI service.
 *
 * @param service SRTM service to destroy.
 */
void SRTM_SPIService_Destroy(srtm_service_t service);

/*!
 * @brief Reset SPI service. This is used to stop sending events and return to initial state.
 *
 * @param service SRTM service to reset.
 * @param core Identify which core is to be reset.
 */
void SRTM_SPIService_Reset(srtm_service_t service, srtm_peercore_t core);

/*!
 * @brief send response for transfer callback
 */
srtm_status_t SRTM_SPIService_SendResponse(srtm_service_t service, srtm_response_t response, uint8_t retCode);

#ifdef __cplusplus
}
#endif

/*! @} */
