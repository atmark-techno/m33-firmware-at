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
#include "app_spi.h"
#include "app_gpio.h"
#include "build_bug.h"
#include "main.h"
#include "semphr.h"
#include "spi.h"

#define SPI_TASK_PRIORITY (3U)

struct spi_gpio_settings
{
    uint32_t sck_pin;
    uint32_t miso_pin;
    uint32_t mosi_pin;
};

static SemaphoreHandle_t spi_xfer_sem;
static uint8_t *spi_xfer_buf;
static TaskHandle_t spi_xfer_task;
static srtm_response_t spi_xfer_response;
static uint32_t spi_xfer_mode;
static uint16_t spi_xfer_bits_per_word;
static uint16_t spi_xfer_len;
static struct spi_gpio_settings *spi_xfer_dev;

static struct spi_gpio_settings *get_spi_gpio(struct spi_settings *settings)
{
    BUILD_BUG_ON((uintptr_t)(((struct spi_settings *)0) + 1) % _Alignof(struct spi_gpio_settings) != 0);
    return (struct spi_gpio_settings *)(settings + 1);
}

// used in spi_bitbang_txrx.h
static inline void setsck(const struct spi_gpio_settings *spi, int is_on)
{
    uint8_t gpioIdx = APP_GPIO_IDX(spi->sck_pin);
    uint8_t pinIdx  = APP_PIN_IDX(spi->sck_pin);

    APP_GPIO_Write(gpioIdx, pinIdx, is_on);
}
static inline void setmosi(const struct spi_gpio_settings *spi, int is_on)
{
    uint8_t gpioIdx = APP_GPIO_IDX(spi->mosi_pin);
    uint8_t pinIdx  = APP_PIN_IDX(spi->mosi_pin);

    APP_GPIO_Write(gpioIdx, pinIdx, is_on ? 1 : 0);
}
static inline int getmiso(const struct spi_gpio_settings *spi)
{
    uint8_t gpioIdx = APP_GPIO_IDX(spi->miso_pin);
    uint8_t pinIdx  = APP_PIN_IDX(spi->miso_pin);

    return APP_GPIO_Read(gpioIdx, pinIdx);
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

static inline u32 txrx(struct spi_gpio_settings *spi, uint32_t mode, uint8_t word, uint16_t bits_per_word)
{
    unsigned cpol = !!(mode & SPI_CPOL);
    if (mode & SPI_CPHA)
        return bitbang_txrx_be_cpha1(spi, 1 /*delay*/, cpol, 0 /* flags */, word, bits_per_word);
    else
        return bitbang_txrx_be_cpha0(spi, 1 /*delay*/, cpol, 0 /* flags */, word, bits_per_word);
}

static inline void txrx_8(struct spi_gpio_settings *spi, uint32_t mode, uint16_t bits_per_word, uint16_t len,
                          uint8_t *buf)
{
    while (len > 0)
    {
        uint8_t word = *buf;
        word         = txrx(spi, mode, word, bits_per_word);
        *buf++       = word;
        len -= 1;
    }
}

static inline void txrx_16(struct spi_gpio_settings *spi, uint32_t mode, uint16_t bits_per_word, uint16_t len,
                           uint16_t *buf)
{
    while (len > 0)
    {
        uint16_t word = *buf;
        word          = txrx(spi, mode, word, bits_per_word);
        *buf++        = word;
        len -= 2;
    }
}

static inline void txrx_32(struct spi_gpio_settings *spi, uint32_t mode, uint16_t bits_per_word, uint16_t len,
                           uint32_t *buf)
{
    while (len > 0)
    {
        uint32_t word = *buf;
        word          = txrx(spi, mode, word, bits_per_word);
        *buf++        = word;
        len -= 4;
    }
}

static void spi_xfer_loop(void *pvPatameters)
{
    uint8_t ret;

    while (true)
    {
        xSemaphoreTake(spi_xfer_sem, portMAX_DELAY);
        ret = 0;

        if (!spi_xfer_buf)
            continue;

        if (spi_xfer_bits_per_word <= 8)
        {
            txrx_8(spi_xfer_dev, spi_xfer_mode, spi_xfer_bits_per_word, spi_xfer_len, spi_xfer_buf);
        }
        else if (spi_xfer_bits_per_word <= 16)
        {
            txrx_16(spi_xfer_dev, spi_xfer_mode, spi_xfer_bits_per_word, spi_xfer_len, (uint16_t *)spi_xfer_buf);
        }
        else if (spi_xfer_bits_per_word <= 32)
        {
            txrx_32(spi_xfer_dev, spi_xfer_mode, spi_xfer_bits_per_word, spi_xfer_len, (uint32_t *)spi_xfer_buf);
        }
        else
        {
            ret = SRTM_SPI_RETCODE_EINVAL;
        }
        spi_xfer_buf = NULL;
        SRTM_SPIService_SendResponse(spiService, spi_xfer_response, ret);
    }
}

static int spi_gpio_transfer(struct spi_settings *settings, srtm_response_t response, uint16_t bits_per_word,
                             uint32_t speed_hz, uint16_t len, uint8_t *tx_buf, uint8_t *rx_buf,
                             bool __unused continuous)
{
    struct spi_gpio_settings *spi = get_spi_gpio(settings);

    if (spi_xfer_buf)
    {
        PRINTF("SPI xfer while previous one still in progress!\r\n");
        return SRTM_SPI_RETCODE_EBUSY;
    }

    // tx_buffer is gone when we return here, copy to rx_buffer (that stays) which will be overwritten
    memcpy(rx_buf, tx_buf, len);
    spi_xfer_buf           = rx_buf;
    spi_xfer_mode          = settings->mode;
    spi_xfer_bits_per_word = bits_per_word;
    spi_xfer_len           = len;
    spi_xfer_response      = response;
    spi_xfer_dev           = spi;
    xSemaphoreGive(spi_xfer_sem);

    return 0;
}

static int spi_gpio_set_mode(struct spi_settings *settings, uint32_t mode)
{
    settings->mode = mode;
    return 0;
}

static int spi_gpio_init(struct spi_settings *settings, struct srtm_spi_init_payload *generic_init)
{
    struct spi_gpio_settings *spi           = get_spi_gpio(settings);
    struct srtm_spi_init_gpio_payload *init = &generic_init->gpio;
    uint8_t port_idx                        = settings->port_idx;

    if (APP_IO_GetIndex(init->sck_pin) == 0xffff)
    {
        PRINTF("spi %d: invalid %s\r\n", port_idx, "sck_pin");
        return SRTM_SPI_RETCODE_EINVAL;
    }
    if (APP_IO_GetIndex(init->miso_pin) == 0xffff)
    {
        PRINTF("spi %d: invalid %s\r\n", port_idx, "miso_pin");
        return SRTM_SPI_RETCODE_EINVAL;
    }
    if (APP_IO_GetIndex(init->mosi_pin) == 0xffff)
    {
        PRINTF("spi %d: invalid %s\r\n", port_idx, "mosi_pin");
        return SRTM_SPI_RETCODE_EINVAL;
    }

    spi->sck_pin  = init->sck_pin;
    spi->miso_pin = init->miso_pin;
    spi->mosi_pin = init->mosi_pin;
    APP_GPIO_SetupGPIO_Input(APP_IO_SPLIT_ID(init->miso_pin));
    /* we use SPI_MODE_0 so init to 0 (start of transfer will set clk to 1) */
    APP_GPIO_SetupGPIO_Output(APP_IO_SPLIT_ID(init->sck_pin), 0);
    APP_GPIO_SetupGPIO_Output(APP_IO_SPLIT_ID(init->mosi_pin), 0);
    settings->mode = init->mode;

    /* only init once */
    if (spi_xfer_sem == NULL)
    {
        spi_xfer_sem = xSemaphoreCreateBinary();
        if (!spi_xfer_sem)
        {
            return SRTM_SPI_RETCODE_ENOMEM;
        }
        xTaskCreate(spi_xfer_loop, "SPI xfer task", 128U, NULL, SPI_TASK_PRIORITY, &spi_xfer_task);
    }

    PRINTF("spi %d: init ok (gpio %#x/%#x/%#x)\r\n", port_idx, init->sck_pin, init->miso_pin, init->mosi_pin);

    return SRTM_SPI_RETCODE_SUCCESS;
}

/* manually added to spi_hooks top of app_spi.c */
const struct spi_hooks spi_gpio_hooks = {
    .init          = spi_gpio_init,
    .transfer      = spi_gpio_transfer,
    .set_mode      = spi_gpio_set_mode,
    .settings_size = sizeof(struct spi_gpio_settings),
};
