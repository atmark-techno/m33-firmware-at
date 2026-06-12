/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "srtm_spi_service.h"
#include "fsl_reset.h"
#include "fsl_upower.h"
#include "fsl_lpspi.h"

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
#include "spi.h"

#define SPI_TASK_PRIORITY (3U)
#define APP_LPSPI_IRQ_PRIO (5U)

struct spi_lpspi_settings
{
    uint32_t spi_index;

    LPSPI_Type *base;
    IRQn_Type irqn;
    clock_ip_name_t clock_ip_name;
    clock_ip_src_t clock_ip_src;
    reset_ip_name_t reset;

    lpspi_master_handle_t handle;
    srtm_response_t response;

    bool transmit_in_progress;
};

static struct spi_lpspi_settings *get_spi_lpspi(struct spi_settings *settings)
{
    BUILD_BUG_ON((uintptr_t)(((struct spi_settings *)0) + 1) % _Alignof(struct spi_lpspi_settings) != 0);
    return (struct spi_lpspi_settings *)(settings + 1);
}

static void spi_lpspi_reset(struct spi_lpspi_settings *spi)
{
    LPSPI_Deinit(spi->base);
}

static void spi_lpspi_callback(LPSPI_Type *base, lpspi_master_handle_t *handle, status_t status, void *userData)
{
    struct spi_lpspi_settings *spi = userData;

    if (status == kStatus_Success)
    {
        __NOP();
    }

    spi_lpspi_reset(spi);
    SRTM_SPIService_SendResponse(spiService, spi->response, status);
    spi->transmit_in_progress = false;
}

static void spi_lpspi_setup_transfer(struct spi_lpspi_settings *spi, uint32_t mode, uint16_t bits_per_word,
                                     uint32_t speed_hz)
{
    lpspi_master_config_t masterConfig;
    uint32_t srcClock_Hz;

    /* Master config. */
    LPSPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate                      = speed_hz;
    masterConfig.whichPcs                      = kLPSPI_Pcs0; /*! Peripheral Chip Select (PCS) - 0 */
    masterConfig.pcsToSckDelayInNanoSec        = 1000000000U / masterConfig.baudRate;
    masterConfig.lastSckToPcsDelayInNanoSec    = 1000000000U / masterConfig.baudRate;
    masterConfig.betweenTransferDelayInNanoSec = 1000000000U / masterConfig.baudRate;
    masterConfig.bitsPerFrame                  = bits_per_word;
    if (mode & SPI_CPHA)
        masterConfig.cpha = kLPSPI_ClockPhaseSecondEdge;
    if (mode & SPI_CPOL)
        masterConfig.cpol = kLPSPI_ClockPolarityActiveLow;
    if (mode & SPI_CS_HIGH)
        masterConfig.pcsActiveHighOrLow = kLPSPI_PcsActiveHigh;

    srcClock_Hz = CLOCK_GetIpFreq(spi->clock_ip_name);
    LPSPI_MasterInit(spi->base, &masterConfig, srcClock_Hz);

    LPSPI_MasterTransferCreateHandle(spi->base, &spi->handle, spi_lpspi_callback, spi);
}

static int spi_lpspi_transfer(struct spi_settings *settings, srtm_response_t response, uint16_t bits_per_word,
                              uint32_t speed_hz, uint16_t len, uint8_t *tx_buf, uint8_t *rx_buf)
{
    struct spi_lpspi_settings *spi = get_spi_lpspi(settings);
    lpspi_transfer_t masterXfer;
    uint8_t ret = SRTM_SPI_RETCODE_SUCCESS;

    taskENTER_CRITICAL();
    if (spi->transmit_in_progress)
    {
        PRINTF("SPI xfer while previous one still in progress!\r\n");
        ret = SRTM_SPI_RETCODE_EBUSY;
        goto out_busy;
    }
    spi->transmit_in_progress = true;
    taskEXIT_CRITICAL();

    if (bits_per_word > 32)
    {
        ret = SRTM_SPI_RETCODE_EINVAL;
        goto out_inval;
    }

    spi_lpspi_setup_transfer(spi, settings->mode, bits_per_word, speed_hz);
    spi->response = response;

    /* Start master transfer, send data to slave */
    masterXfer.txData      = tx_buf;
    masterXfer.rxData      = rx_buf;
    masterXfer.dataSize    = len;
    masterXfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
    LPSPI_MasterTransferNonBlocking(spi->base, &spi->handle, &masterXfer);

    return 0;

out_inval:
    taskENTER_CRITICAL();
    spi->transmit_in_progress = false;
out_busy:
    taskEXIT_CRITICAL();
    SRTM_SPIService_SendResponse(spiService, response, ret);

    return 0;
}

static int spi_lpspi_set_mode(struct spi_settings *settings, uint32_t mode)
{
    settings->mode = mode;
    return 0;
}

static int spi_lpspi_init(struct spi_settings *settings, struct srtm_spi_init_payload *generic_init)
{
    struct spi_lpspi_settings *spi           = get_spi_lpspi(settings);
    struct srtm_spi_init_lpspi_payload *init = &generic_init->lpspi;
    uint8_t port_idx                         = settings->port_idx;

    switch (init->spi_index)
    {
        case 0:
            spi->base          = LPSPI0;
            spi->irqn          = LPSPI0_IRQn;
            spi->clock_ip_name = kCLOCK_Lpspi0;
            spi->clock_ip_src  = kCLOCK_Pcc0BusIpSrcSysOscDiv2;
            spi->reset         = kRESET_Lpspi0;
            break;
        case 1:
            spi->base          = LPSPI1;
            spi->irqn          = LPSPI1_IRQn;
            spi->clock_ip_name = kCLOCK_Lpspi1;
            spi->clock_ip_src  = kCLOCK_Pcc0BusIpSrcSysOscDiv2;
            spi->reset         = kRESET_Lpspi1;
            break;
        case 2:
            spi->base          = LPSPI2;
            spi->irqn          = LPSPI2_IRQn;
            spi->clock_ip_name = kCLOCK_Lpspi2;
            spi->clock_ip_src  = kCLOCK_Pcc2BusIpSrcFusionDspBus;
            spi->reset         = kRESET_Lpspi2;
            break;
        case 3:
            spi->base          = LPSPI3;
            spi->irqn          = LPSPI3_IRQn;
            spi->clock_ip_name = kCLOCK_Lpspi3;
            spi->clock_ip_src  = kCLOCK_Pcc2BusIpSrcFusionDspBus;
            spi->reset         = kRESET_Lpspi3;
            break;
        default:
            PRINTF("lpspi index %d not supported\r\n", init->spi_index);
            return SRTM_SPI_RETCODE_EINVAL;
    }
    spi->spi_index = init->spi_index;
    settings->mode = init->mode;

    /* IRQ enable by can but priority isn't set, set it now */
    NVIC_SetPriority(spi->irqn, APP_LPSPI_IRQ_PRIO);

    /*Set clock source for LPSPI and get master clock source*/
    CLOCK_SetIpSrc(spi->clock_ip_name, spi->clock_ip_src);
    RESET_PeripheralReset(spi->reset);

    PRINTF("spi %d: init ok (LPSPI%d)\r\n", port_idx, spi->spi_index);

    return SRTM_SPI_RETCODE_SUCCESS;
}

static void spi_lpspi_resume(struct spi_settings *settings)
{
    struct spi_lpspi_settings *spi = get_spi_lpspi(settings);

    /* Fusion has been reset. The clock needs to be reset. */
    if (spi->clock_ip_src == kCLOCK_Pcc2BusIpSrcFusionDspBus)
    {
        CLOCK_SetIpSrc(spi->clock_ip_name, spi->clock_ip_src);
        RESET_PeripheralReset(spi->reset);
    }
}

/* manually added to spi_hooks top of app_spi.c */
const struct spi_hooks spi_lpspi_hooks = {
    .init          = spi_lpspi_init,
    .transfer      = spi_lpspi_transfer,
    .set_mode      = spi_lpspi_set_mode,
    .resume        = spi_lpspi_resume,
    .settings_size = sizeof(struct spi_lpspi_settings),
};
