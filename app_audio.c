/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "srtm_audio_service.h"
#include "fsl_reset.h"
#include "fsl_sai.h"
#include "fsl_sai_edma.h"
#include "fsl_upower.h"

#include "task.h"
#include "event_groups.h"
#include "srtm_message.h"
#include "srtm_message_struct.h"
#include "srtm_sai_edma_adapter.h"
#include "srtm_spi_service.h"
#include "app_srtm_internal.h"
#include "build_bug.h"
#include "main.h"
#include "semphr.h"

#define APP_SAI_TX_DMA_IRQ_PRIO (5U)
#define APP_SAI_RX_DMA_IRQ_PRIO (5U)
#define APP_SAI_IRQ_PRIO (5U)

static srtm_service_t audioService;

#define MAX_AUDIO_BUSES 2
struct audio_device
{
    I2S_Type *base;
    uint32_t irq;
    clock_ip_name_t clock;
    reset_ip_name_t reset;
    /* eDMA */
    uint32_t tx_channel;
    uint32_t rx_channel;
    clock_ip_name_t tx_clock;
    clock_ip_name_t rx_clock;
    uint32_t tx_mux;
    uint32_t rx_mux;
};

static struct audio_device audio_devices[MAX_AUDIO_BUSES] = { {
                                                                  .base       = SAI0,
                                                                  .irq        = SAI0_IRQn,
                                                                  .clock      = kCLOCK_Sai0,
                                                                  .reset      = kRESET_Sai0,
                                                                  .tx_channel = 14,
                                                                  .rx_channel = 15,
                                                                  .tx_clock   = kCLOCK_Dma0Ch14,
                                                                  .rx_clock   = kCLOCK_Dma0Ch15,
                                                                  .tx_mux     = kDmaRequestMux0SAI0Tx,
                                                                  .rx_mux     = kDmaRequestMux0SAI0Rx,
                                                              },
                                                              {
                                                                  .base       = SAI1,
                                                                  .irq        = SAI1_IRQn,
                                                                  .clock      = kCLOCK_Sai1,
                                                                  .reset      = kRESET_Sai1,
                                                                  .tx_channel = 16,
                                                                  .rx_channel = 17,
                                                                  .tx_clock   = kCLOCK_Dma0Ch16,
                                                                  .rx_clock   = kCLOCK_Dma0Ch17,
                                                                  .tx_mux     = kDmaRequestMux0SAI1Tx,
                                                                  .rx_mux     = kDmaRequestMux0SAI1Rx,
                                                              } };

#define AUDIO_TASK_PRIORITY (3U)

static void APP_AUDIO_init_device(uint8_t bus_id)
{
    struct audio_device *audioDevice = &audio_devices[bus_id];
    edma_config_t dmaConfig;

    /* Use Pll1Pfd2Div clock source 12.288MHz. */
    CLOCK_SetIpSrc(audioDevice->clock, kCLOCK_Cm33SaiClkSrcPll1Pfd2Div);

    RESET_PeripheralReset(audioDevice->reset);
    CLOCK_EnableClock(audioDevice->tx_clock);
    CLOCK_EnableClock(audioDevice->rx_clock);

    /* Initialize DMA0 for SAI */
    EDMA_GetDefaultConfig(&dmaConfig);
    EDMA_Init(DMA0, &dmaConfig);

    /* Initialize DMAMUX for SAI */
    EDMA_SetChannelMux(DMA0, audioDevice->tx_channel, audioDevice->tx_mux);
    EDMA_SetChannelMux(DMA0, audioDevice->rx_channel, audioDevice->rx_mux);
}

static srtm_sai_adapter_t APP_AUDIO_init(uint8_t bus_id, srtm_channel_t channel)
{
    srtm_sai_adapter_t saiAdapter;
    struct audio_device *audioDevice;
    srtm_sai_edma_config_t saiTxConfig;
    srtm_sai_edma_config_t saiRxConfig;

    if (bus_id >= MAX_AUDIO_BUSES)
        return NULL;

    APP_AUDIO_init_device(bus_id);

    memset(&saiTxConfig, 0, sizeof(saiTxConfig));
    memset(&saiRxConfig, 0, sizeof(saiRxConfig));

    audioDevice = &audio_devices[bus_id];

    /*  Set SAI DMA IRQ Priority. */
    NVIC_SetPriority(APP_DMA_IRQN(audioDevice->tx_channel), APP_SAI_TX_DMA_IRQ_PRIO);
    NVIC_SetPriority(APP_DMA_IRQN(audioDevice->rx_channel), APP_SAI_RX_DMA_IRQ_PRIO);
    NVIC_SetPriority(audioDevice->irq, APP_SAI_IRQ_PRIO);

    /* Create SAI EDMA adapter */
    SAI_GetClassicI2SConfig(&saiTxConfig.config, kSAI_WordWidth16bits, kSAI_Stereo, kSAI_Channel0Mask);
    saiTxConfig.config.syncMode           = kSAI_ModeAsync; /* Tx in async mode */
    saiTxConfig.config.fifo.fifoWatermark = FSL_FEATURE_SAI_FIFO_COUNTn(audioDevice->base) - 1;
    saiTxConfig.mclk                      = CLOCK_GetIpFreq(audioDevice->clock);

#if SRTM_SAI_EDMA_LOCAL_BUF_ENABLE
    saiTxConfig.stopOnSuspend = false; /* Keep playing audio on APD suspend. */
#else
    saiTxConfig.stopOnSuspend = true;
#endif
    saiTxConfig.threshold = 1U; /* Every period transmitted triggers periodDone message to A core. */
    saiTxConfig.guardTime =
        1000; /* Unit:ms. This is a lower limit that M core should reserve such time data to wakeup A core. */
    saiTxConfig.dmaChannel = audioDevice->tx_channel;

    SAI_GetClassicI2SConfig(&saiRxConfig.config, kSAI_WordWidth16bits, kSAI_Stereo, kSAI_Channel0Mask);
    saiRxConfig.config.syncMode = kSAI_ModeSync; /* Rx in sync mode */

    saiRxConfig.config.fifo.fifoWatermark = 1;
    saiRxConfig.mclk                      = saiTxConfig.mclk;
#if SRTM_SAI_EDMA_LOCAL_BUF_ENABLE
    saiRxConfig.stopOnSuspend = false; /* Keep recording data on APD suspend. */
#else
    saiRxConfig.stopOnSuspend = true;
#endif
    saiRxConfig.threshold  = UINT32_MAX; /* Every period received triggers periodDone message to A core. */
    saiRxConfig.dmaChannel = audioDevice->rx_channel;

    saiAdapter = SRTM_SaiEdmaAdapter_Create(audioDevice->base, DMA0, &saiTxConfig, &saiRxConfig);
    assert(saiAdapter);

#if SRTM_SAI_EDMA_LOCAL_BUF_ENABLE
    SRTM_SaiEdmaAdapter_SetTxLocalBuf(saiAdapter, &g_local_buf);
    SRTM_SaiEdmaAdapter_SetTxPreCopyCallback(saiAdapter, APP_SRTM_PreCopyCallback);
    SRTM_SaiEdmaAdapter_SetTxPostCopyCallback(saiAdapter, APP_SRTM_PostCopyCallback);
#endif

    PRINTF("audio %d: init ok\r\n", bus_id);

    return saiAdapter;
}

/**********************************************************
 * init/PM hooks called from APP_SRTM
 *********************************************************/

void APP_AUDIO_InitService(void)
{
    audioService = SRTM_AudioService_Create(APP_AUDIO_init);
    SRTM_Dispatcher_RegisterService(disp, audioService);
}
