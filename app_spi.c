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
#include "build_bug.h"
#include "main.h"
#include "semphr.h"

/* global settings */
#define SPI_MAX_PORTS 4
static struct spi_settings *spi_settings[SPI_MAX_PORTS];
srtm_service_t spiService;

/* Hooks for each type.
 * We'd ideally use a linker-generated array like linux/u-boot for this,
 * but this is overkill so just list all valid types manually. */
extern const struct spi_hooks spi_gpio_hooks;
static struct spi_hooks const *spi_hooks[SRTM_SPI_TYPES_COUNT] = {
    [SRTM_SPI_TYPE_GPIO] = &spi_gpio_hooks,
};

/* get settings or NULL, log error if caller name given */
static struct spi_settings *get_settings(uint8_t port_idx, const char *caller)
{
    if (port_idx >= SPI_MAX_PORTS)
    {
        if (caller)
            PRINTF("spi %d %s: port_idx %d too big\r\n", port_idx, caller, port_idx);
        return NULL;
    }

    if (!spi_settings[port_idx])
    {
        if (caller)
            PRINTF("spi %d %s without init?\r\n", port_idx, caller);
        return NULL;
    }
    return spi_settings[port_idx];
}

static srtm_status_t APP_SPI_transfer(srtm_response_t response, uint8_t port_idx, uint16_t bits_per_word, uint16_t len,
                                      uint8_t *tx_buf, uint8_t *rx_buf)
{
    struct spi_settings *settings = get_settings(port_idx, "transfer");
    uint8_t ret                   = 0;

    if (!settings)
    {
        ret = SRTM_SPI_RETCODE_EINVAL;
        goto out_fail;
    }

    if (!spi_hooks[settings->type]->transfer)
    {
        ret = SRTM_SPI_RETCODE_UNSUPPORTED;
        goto out_fail;
    }

    ret = spi_hooks[settings->type]->transfer(settings, response, bits_per_word, len, tx_buf, rx_buf);
    if (ret)
        goto out_fail;

    return 0;

out_fail:
    return SRTM_SPIService_SendResponse(spiService, response, ret);
}

static uint8_t APP_SPI_init(uint8_t port_idx, struct srtm_spi_init_payload *init)
{
    struct spi_settings *settings;
    const struct spi_hooks *hooks;
    int rc = 0;

    if (port_idx >= SPI_MAX_PORTS)
    {
        PRINTF("spi %d %s: port_idx %d too big\r\n", port_idx, "init", port_idx);
        return SRTM_SPI_RETCODE_EINVAL;
    }
    if (spi_settings[port_idx])
    {
        PRINTF("spi port %d was already init!\r\n", port_idx);
        return SRTM_SPI_RETCODE_EBUSY;
    }
    if (init->type >= SRTM_SPI_TYPES_COUNT || !spi_hooks[init->type])
    {
        PRINTF("spi port %d type %" PRIu32 " either type too high or not defined\r\n", port_idx, init->type);
        return SRTM_SPI_RETCODE_UNSUPPORTED;
    }

    hooks    = spi_hooks[init->type];
    settings = pvPortMalloc(sizeof(*settings) + hooks->settings_size);
    if (!settings)
    {
        return SRTM_SPI_RETCODE_ENOMEM;
    }

    settings->type     = init->type;
    settings->port_idx = port_idx;

    if (hooks->init)
        rc = hooks->init(settings, init);
    if (rc)
    {
        vPortFree(settings);
        return rc;
    }

    spi_settings[port_idx] = settings;

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
