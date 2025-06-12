/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

#include "srtm_spi_service.h"

struct spi_settings
{
    srtm_spi_type_t type;
    uint8_t port_idx;
    /* per-type data follows, force alignment to ensure 'settings + 1' works */
} __attribute__((aligned(4)));

struct spi_hooks
{
    int (*init)(struct spi_settings *settings, struct srtm_spi_init_payload *init);
    int (*transfer)(struct spi_settings *settings, srtm_response_t response, uint16_t bits_per_word, uint16_t len,
                    uint8_t *tx_buf, uint8_t *rx_buf);
    size_t settings_size;
};

/* required by spi drivers to send messages back to linux */
extern srtm_service_t spiService;

/* Init hook for app_srtm.c */
void APP_SPI_InitService(void);
