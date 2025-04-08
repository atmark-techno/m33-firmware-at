/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "srtm_spi_service.h"
#include "fsl_reset.h"
#include "fsl_upower.h"

#include "task.h"
#include "event_groups.h"
#include "srtm_message.h"
#include "srtm_message_struct.h"
#include "srtm_spi_service.h"
#include "app_srtm_internal.h"
#include "build_bug.h"
#include "main.h"

static srtm_service_t spiService;

#define MAX_SPI_BUSES 4
struct spi_device
{
    srtm_spi_type_t type;
    uint32_t sck_pin;
    uint32_t miso_pin;
    uint32_t mosi_pin;
} * spi_devices[MAX_SPI_BUSES];

// used in spi_bitbang_txrx.h
static inline void setsck(const struct spi_device *spi, int is_on)
{
    uint8_t gpioIdx = APP_GPIO_IDX(spi->sck_pin);
    uint8_t pinIdx  = APP_PIN_IDX(spi->sck_pin);

    RGPIO_PinWrite(gpios[gpioIdx], pinIdx, is_on);
}
static inline void setmosi(const struct spi_device *spi, int is_on)
{
    uint8_t gpioIdx = APP_GPIO_IDX(spi->mosi_pin);
    uint8_t pinIdx  = APP_PIN_IDX(spi->mosi_pin);

    RGPIO_PinWrite(gpios[gpioIdx], pinIdx, is_on ? 1 : 0);
}
static inline int getmiso(const struct spi_device *spi)
{
    uint8_t gpioIdx = APP_GPIO_IDX(spi->miso_pin);
    uint8_t pinIdx  = APP_PIN_IDX(spi->miso_pin);

    return RGPIO_PinRead(gpios[gpioIdx], pinIdx);
}
/* There seem to be no problem with no delay.
 * If we need a delay for testing (or some other SPI device), this stub
 * can be used with delay parameter of bitbang_txrx_be_cpha0.
 * loops=1000 takes about 150us.
 */
#if 0
static inline void spidelay(unsigned int loops)
{
    unsigned i;
    for (i = 0; i < loops; i++)
    {
        __asm__ volatile("" : "+g"(i) : :);
    }
}
#else
#define spidelay(...)
#endif

#include "spi_bitbang_txrx.h"

static inline void txrx_8(struct spi_device *spi, uint16_t bits_per_word, uint16_t len, uint8_t *tx_buf,
                          uint8_t *rx_buf)
{
    while (len > 0)
    {
        uint8_t word = *tx_buf++;
        word         = bitbang_txrx_be_cpha0(spi, 1 /*delay*/, 0 /* cpol */, 0 /* flags */, word, bits_per_word);
        *rx_buf++    = word;
        len -= 1;
    }
}

static inline void txrx_16(struct spi_device *spi, uint16_t bits_per_word, uint16_t len, uint16_t *tx_buf,
                           uint16_t *rx_buf)
{
    while (len > 0)
    {
        uint16_t word = *tx_buf++;
        word          = bitbang_txrx_be_cpha0(spi, 1 /*delay*/, 0 /* cpol */, 0 /* flags */, word, bits_per_word);
        *rx_buf++     = word;
        len -= 2;
    }
}

static inline void txrx_32(struct spi_device *spi, uint16_t bits_per_word, uint16_t len, uint32_t *tx_buf,
                           uint32_t *rx_buf)
{
    while (len > 0)
    {
        uint32_t word = *tx_buf++;
        word          = bitbang_txrx_be_cpha0(spi, 1 /*delay*/, 0 /* cpol */, 0 /* flags */, word, bits_per_word);
        *rx_buf++     = word;
        len -= 4;
    }
}

static srtm_status_t APP_SPI_transfer(srtm_service_t service, srtm_response_t response, uint8_t bus_id,
                                      uint16_t bits_per_word, uint16_t len, uint8_t *tx_buf, uint8_t *rx_buf)
{
    uint8_t ret = 0;
    if (bus_id >= MAX_SPI_BUSES)
    {
        ret = SRTM_SPI_RETCODE_EINVAL;
        goto out;
    }
    if (!spi_devices[bus_id])
    {
        ret = SRTM_SPI_RETCODE_EINVAL;
        goto out;
    }
    // check type/dispatch if we add more types

    // XXX process transfer / send reply in another task to avoid hogging dispatcher
    struct spi_device *spi = spi_devices[bus_id];

    if (bits_per_word <= 8)
    {
        txrx_8(spi, bits_per_word, len, tx_buf, rx_buf);
    }
    else if (bits_per_word <= 16)
    {
        txrx_16(spi, bits_per_word, len, (uint16_t *)tx_buf, (uint16_t *)rx_buf);
    }
    else if (bits_per_word <= 32)
    {
        txrx_32(spi, bits_per_word, len, (uint32_t *)tx_buf, (uint32_t *)rx_buf);
    }
    else
    {
        ret = SRTM_SPI_RETCODE_EINVAL;
    }

out:
    return SRTM_SPIService_SendResponse(service, response, ret);
}

static uint8_t APP_SPI_init(uint8_t bus_id, struct srtm_spi_init_payload *init)
{
    if (bus_id >= MAX_SPI_BUSES)
        return SRTM_SPI_RETCODE_EINVAL;
    if (spi_devices[bus_id])
        return SRTM_SPI_RETCODE_EBUSY;

    if (init->type != SRTM_SPI_TYPE_GPIO)
    {
        PRINTF("spi %d: type %d not supported in this version\r\n", bus_id, init->type);
        return SRTM_SPI_RETCODE_UNSUPPORTED;
    }
    if (init->gpio.mode != 0)
    {
        PRINTF("spi %d: modes not supported in this version\r\n", bus_id);
        return SRTM_SPI_RETCODE_UNSUPPORTED;
    }

    if (APP_IO_GetIndex(init->gpio.sck_pin) == 0xffff)
    {
        PRINTF("spi %d: invalid %s\r\n", bus_id, "sck_pin");
        return SRTM_SPI_RETCODE_EINVAL;
    }
    if (APP_IO_GetIndex(init->gpio.miso_pin) == 0xffff)
    {
        PRINTF("spi %d: invalid %s\r\n", bus_id, "miso_pin");
        return SRTM_SPI_RETCODE_EINVAL;
    }
    if (APP_IO_GetIndex(init->gpio.mosi_pin) == 0xffff)
    {
        PRINTF("spi %d: invalid %s\r\n", bus_id, "mosi_pin");
        return SRTM_SPI_RETCODE_EINVAL;
    }

    spi_devices[bus_id] = pvPortMalloc(sizeof(*spi_devices[0]));
    if (!spi_devices[bus_id])
        return SRTM_SPI_RETCODE_ENOMEM;

    spi_devices[bus_id]->type     = init->type;
    spi_devices[bus_id]->sck_pin  = init->gpio.sck_pin;
    spi_devices[bus_id]->miso_pin = init->gpio.miso_pin;
    spi_devices[bus_id]->mosi_pin = init->gpio.mosi_pin;

    PRINTF("spi %d: init ok\r\n", bus_id);

    return 0;
}

/**********************************************************
 * init/PM hooks called from APP_SRTM
 *********************************************************/

void APP_SPI_InitService(void)
{
    spiService = SRTM_SPIService_Create(APP_SPI_init, APP_SPI_transfer);
    SRTM_Dispatcher_RegisterService(disp, spiService);
}
