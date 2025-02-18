/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "srtm_io_service.h"
#include "fsl_reset.h"
#include "fsl_flexcan.h"
#include "fsl_upower.h"

#include "task.h"
#include "event_groups.h"
#include "srtm_message.h"
#include "srtm_message_struct.h"
#include "srtm_can_service.h"
#include "app_srtm_internal.h"
#include "build_bug.h"
#include "main.h"

static srtm_service_t canService;

#define CAN_RX_TASK_PRIORITY (3U)
#define APP_CAN_IRQ_PRIO (5U)

#define APP_CAN CAN0
#define APP_CAN_IRQn FlexCAN_IRQn
#define APP_CAN_CLOCK_NAME (kCLOCK_Flexcan)
#define APP_CAN_CLK_FREQ (CLOCK_GetFlexcanClkFreq())
#define APP_CAN_RX_MB_NUM (9)
#define APP_CAN_TX_MB_NUM (8)

static TaskHandle_t s_canRxTask;
static flexcan_handle_t s_flexcanHandle;
static srtm_can_open_params_t s_canOpenParams;
static srtm_can_init_params_t s_canInitParams;
static bool s_canIsWakeupSource;
EventGroupHandle_t s_flexcanTxEvent;
EventGroupHandle_t s_flexcanRxEvent;
#define APP_FLEXCAN_TX_COMPLETE 0x1U
#define APP_FLEXCAN_RX_COMPLETE 0x2U
#define APP_FLEXCAN_ERROR_STATUS 0x4U
#define APP_FLEXCAN_CLOSE 0x8U

struct app_srtm_can_state
{
    unsigned int open : 1;
    unsigned int fd : 1;
} s_canState;

#define bswap32 __builtin_bswap32

static FLEXCAN_CALLBACK(flexcan_Callback)
{
    BaseType_t xHigherPriorityTaskWoken, xResult;

    xHigherPriorityTaskWoken = pdFALSE;
    xResult                  = pdFAIL;

    switch (status)
    {
        case kStatus_FLEXCAN_RxIdle:
            if (APP_CAN_RX_MB_NUM == result)
            {
                xResult =
                    xEventGroupSetBitsFromISR(s_flexcanRxEvent, APP_FLEXCAN_RX_COMPLETE, &xHigherPriorityTaskWoken);
            }
            break;

        case kStatus_FLEXCAN_TxIdle:
            if (APP_CAN_TX_MB_NUM == result)
            {
                xResult =
                    xEventGroupSetBitsFromISR(s_flexcanTxEvent, APP_FLEXCAN_TX_COMPLETE, &xHigherPriorityTaskWoken);
            }
            break;

        case kStatus_FLEXCAN_ErrorStatus:
            xResult = xEventGroupSetBitsFromISR(s_flexcanTxEvent, APP_FLEXCAN_ERROR_STATUS, &xHigherPriorityTaskWoken);
            break;

        default:
            break;
    }

    if (xResult != pdFAIL)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static uint8_t dlc2len(uint8_t dlc)
{
    switch (dlc)
    {
        case 0 ... 8:
            return dlc;
        case 9:
            return 12;
        case 10:
            return 16;
        case 11:
            return 20;
        case 12:
            return 24;
        case 13:
            return 32;
        case 14:
            return 48;
        case 15:
        default:
            return CANFD_MAX_DLEN;
    }
}

static uint8_t len2dlc(uint8_t len)
{
    switch (len)
    {
        case 0 ... 8:
            return len;
        case 9 ... 12:
            return 9;
        case 13 ... 16:
            return 10;
        case 17 ... 20:
            return 11;
        case 21 ... 24:
            return 12;
        case 25 ... 32:
            return 13;
        case 33 ... 48:
            return 14;
        case 49 ... 64:
        default:
            return CANFD_MAX_DLC;
    }
}

static int APP_CAN_tx(uint16_t len, uint8_t *buf)
{
    struct canfd_frame *cfd = (struct canfd_frame *)buf;
    flexcan_mb_transfer_t txXfer;
    const TickType_t txTickTimeout = 1000 / portTICK_PERIOD_MS;
    EventBits_t ev;
    uint32_t data;
    int i;
    int rc = kStatus_Fail;

    if (s_canState.fd)
    {
        flexcan_fd_frame_t fd_frame;

        memset(&fd_frame, 0, sizeof(flexcan_fd_frame_t));
        if (cfd->can_id & CAN_EFF_FLAG)
        {
            fd_frame.id     = FLEXCAN_ID_EXT(cfd->can_id);
            fd_frame.format = kFLEXCAN_FrameFormatExtend;
        }
        else
        {
            fd_frame.id     = FLEXCAN_ID_STD(cfd->can_id);
            fd_frame.format = kFLEXCAN_FrameFormatStandard;
        }

        if (cfd->can_id & CAN_RTR_FLAG)
            fd_frame.type = kFLEXCAN_FrameTypeRemote;
        else
            fd_frame.type = kFLEXCAN_FrameTypeData;

        fd_frame.length = len2dlc(cfd->len);
        for (i = 0; i < cfd->len; i += sizeof(uint32_t))
        {
            data                                    = bswap32(*(uint32_t *)&cfd->data[i]);
            fd_frame.dataWord[i / sizeof(uint32_t)] = data;
        }

        if (cfd->flags & CANFD_ESI)
            fd_frame.esi = 1U;
        if (cfd->flags & CANFD_BRS)
            fd_frame.brs = 1U;
        fd_frame.edl = 1U;

        txXfer.mbIdx   = (uint8_t)APP_CAN_TX_MB_NUM;
        txXfer.framefd = &fd_frame;
        FLEXCAN_TransferFDSendNonBlocking(APP_CAN, &s_flexcanHandle, &txXfer);
    }
    else
    {
        flexcan_frame_t frame;

        memset(&frame, 0, sizeof(flexcan_frame_t));
        if (cfd->can_id & CAN_EFF_FLAG)
        {
            frame.id     = FLEXCAN_ID_EXT(cfd->can_id);
            frame.format = kFLEXCAN_FrameFormatExtend;
        }
        else
        {
            frame.id     = FLEXCAN_ID_STD(cfd->can_id);
            frame.format = kFLEXCAN_FrameFormatStandard;
        }

        if (cfd->can_id & CAN_RTR_FLAG)
            frame.type = kFLEXCAN_FrameTypeRemote;
        else
            frame.type = kFLEXCAN_FrameTypeData;

        frame.length = cfd->len;
        if (0 < cfd->len)
        {
            data            = bswap32(*(uint32_t *)&cfd->data[0]);
            frame.dataWord0 = data;
        }
        if (4 < cfd->len)
        {
            data            = bswap32(*(uint32_t *)&cfd->data[4]);
            frame.dataWord1 = data;
        }

        txXfer.mbIdx = (uint8_t)APP_CAN_TX_MB_NUM;
        txXfer.frame = &frame;
        FLEXCAN_TransferSendNonBlocking(APP_CAN, &s_flexcanHandle, &txXfer);
    }

    ev = xEventGroupWaitBits(s_flexcanTxEvent, APP_FLEXCAN_TX_COMPLETE | APP_FLEXCAN_ERROR_STATUS | APP_FLEXCAN_CLOSE,
                             pdTRUE, pdFALSE, txTickTimeout);
    if (ev & APP_FLEXCAN_TX_COMPLETE)
    {
        rc = kStatus_Success;
    }
    else if (ev & APP_FLEXCAN_CLOSE)
    {
        if (s_canState.fd)
            FLEXCAN_TransferFDAbortSend(APP_CAN, &s_flexcanHandle, APP_CAN_TX_MB_NUM);
        else
            FLEXCAN_TransferAbortSend(APP_CAN, &s_flexcanHandle, APP_CAN_TX_MB_NUM);
    }
    else /* timeout expired or error*/
    {
        (void)xEventGroupClearBits(s_flexcanTxEvent, APP_FLEXCAN_TX_COMPLETE);
    }

    return rc;
}

static void can_rx_task(void *pvParameters)
{
    srtm_notification_t notif = NULL;
    flexcan_mb_transfer_t rxXfer;
    flexcan_fd_frame_t fd_frame;
    flexcan_frame_t frame;
    struct canfd_frame *cfd;
    uint16_t len;
    EventBits_t ev;
    uint32_t data;

    while (true)
    {
        notif = SRTM_CanService_NotifyAlloc(&cfd);
        if (!notif)
        {
            if (xEventGroupGetBits(s_flexcanRxEvent) & APP_FLEXCAN_CLOSE)
                break;

            // message already printed in alloc failure
            SDK_DelayAtLeastUs(1000000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
            continue;
        }

        /* Start receive data through Rx Message Buffer. */
        rxXfer.mbIdx = (uint8_t)APP_CAN_RX_MB_NUM;
        if (s_canState.fd)
        {
            rxXfer.framefd = &fd_frame;
            (void)FLEXCAN_TransferFDReceiveNonBlocking(APP_CAN, &s_flexcanHandle, &rxXfer);
        }
        else
        {
            rxXfer.frame = &frame;
            (void)FLEXCAN_TransferReceiveNonBlocking(APP_CAN, &s_flexcanHandle, &rxXfer);
        }

        /* Wait until Rx MB full. */
        ev = xEventGroupWaitBits(s_flexcanRxEvent, APP_FLEXCAN_RX_COMPLETE | APP_FLEXCAN_CLOSE, pdTRUE, pdFALSE,
                                 portMAX_DELAY);
        if (ev == APP_FLEXCAN_CLOSE)
            break;

        memset(cfd, 0, sizeof(*cfd));
        if (s_canState.fd)
        {
            int i;

            if (fd_frame.format == kFLEXCAN_FrameFormatExtend)
            {
                cfd->can_id = fd_frame.id >> CAN_ID_EXT_SHIFT;
                cfd->can_id |= CAN_EFF_FLAG;
            }
            else
                cfd->can_id = fd_frame.id >> CAN_ID_STD_SHIFT;

            if (fd_frame.type == kFLEXCAN_FrameTypeRemote)
                cfd->can_id |= CAN_RTR_FLAG;

            cfd->len = dlc2len(fd_frame.length);

            if (fd_frame.brs)
                cfd->flags |= CANFD_BRS;
            if (fd_frame.esi)
                cfd->flags |= CANFD_ESI;

            for (i = 0; i < cfd->len; i += sizeof(uint32_t))
            {
                data                       = bswap32(fd_frame.dataWord[i / sizeof(uint32_t)]);
                *(uint32_t *)&cfd->data[i] = data;
            }

            len = CANFD_MTU;
        }
        else
        {
            if (frame.format == kFLEXCAN_FrameFormatExtend)
            {
                cfd->can_id = frame.id >> CAN_ID_EXT_SHIFT;
                cfd->can_id |= CAN_EFF_FLAG;
            }
            else
                cfd->can_id = frame.id >> CAN_ID_STD_SHIFT;

            if (frame.type == kFLEXCAN_FrameTypeRemote)
                cfd->can_id |= CAN_RTR_FLAG;

            cfd->len = frame.length;

            if (0 < cfd->len)
            {
                data                       = bswap32(*(uint32_t *)&frame.dataWord0);
                *(uint32_t *)&cfd->data[0] = data;
            }
            if (4 < cfd->len)
            {
                data                       = bswap32(*(uint32_t *)&frame.dataWord1);
                *(uint32_t *)&cfd->data[4] = data;
            }

            len = CAN_MTU;
        }

        SRTM_CanService_NotifySend(canService, notif, len);
    }

    if (s_canState.fd)
        FLEXCAN_TransferFDAbortReceive(APP_CAN, &s_flexcanHandle, APP_CAN_RX_MB_NUM);
    else
        FLEXCAN_TransferAbortReceive(APP_CAN, &s_flexcanHandle, APP_CAN_RX_MB_NUM);

    if (notif)
        SRTM_Notification_Destroy(notif);
    vTaskDelete(NULL);
}

static int can_start(srtm_can_open_params_t *params)
{
    flexcan_config_t canConfig;
    flexcan_rx_mb_config_t mbConfig;

    FLEXCAN_GetDefaultConfig(&canConfig);
    memset(&canConfig.timingConfig, 0, sizeof(flexcan_timing_config_t));

    if (params->ctrlmode & CAN_CTRLMODE_LOOPBACK)
        canConfig.enableLoopBack = true;
    else
        canConfig.disableSelfReception = true;
    if (params->ctrlmode & CAN_CTRLMODE_LISTENONLY)
        canConfig.enableListenOnlyMode = true;

    canConfig.enableSelfWakeup = true;
    canConfig.wakeupSrc        = kFLEXCAN_WakeupSrcUnfiltered;

    if (params->ctrlmode & CAN_CTRLMODE_FD)
    {
        canConfig.bitRate   = params->bittiming.bitrate;
        canConfig.bitRateFD = params->data_bittiming.bitrate;

        if (!FLEXCAN_FDCalculateImprovedTimingValues(APP_CAN, canConfig.bitRate, canConfig.bitRateFD, APP_CAN_CLK_FREQ,
                                                     &canConfig.timingConfig))
        {
            PRINTF("failed to FLEXCAN_FDCalculateImprovedTimingValues\r\n");
            return kStatus_Fail;
        }

        FLEXCAN_FDInit(APP_CAN, &canConfig, APP_CAN_CLK_FREQ, kFLEXCAN_64BperMB, true);
    }
    else
    {
        canConfig.bitRate = params->bittiming.bitrate;

        if (!FLEXCAN_CalculateImprovedTimingValues(APP_CAN, canConfig.bitRate, APP_CAN_CLK_FREQ,
                                                   &canConfig.timingConfig))
        {
            PRINTF("failed to FLEXCAN_CalculateImprovedTimingValues\r\n");
            return kStatus_Fail;
        }

        FLEXCAN_Init(APP_CAN, &canConfig, APP_CAN_CLK_FREQ);
    }

    /* acceptance mask/acceptance code (accept everything) */
    FLEXCAN_SetRxMbGlobalMask(APP_CAN, FLEXCAN_RX_MB_STD_MASK(0, 0, 0));

    /* Setup Message Buffer. */
    mbConfig.format = kFLEXCAN_FrameFormatStandard;
    mbConfig.type   = kFLEXCAN_FrameTypeData;
    mbConfig.id     = FLEXCAN_ID_STD(0);
    if (params->ctrlmode & CAN_CTRLMODE_FD)
    {
        FLEXCAN_SetFDRxMbConfig(APP_CAN, APP_CAN_RX_MB_NUM, &mbConfig, true);
        FLEXCAN_SetFDTxMbConfig(APP_CAN, APP_CAN_TX_MB_NUM, true);
    }
    else
    {
        FLEXCAN_SetRxMbConfig(APP_CAN, APP_CAN_RX_MB_NUM, &mbConfig, true);
        FLEXCAN_SetTxMbConfig(APP_CAN, APP_CAN_TX_MB_NUM, true);
    }

    return kStatus_Success;
}

static int APP_CAN_open(srtm_can_open_params_t *params)
{
    if (s_canState.open)
    {
        PRINTF("CAN is already opened\r\n");
        return kStatus_Fail;
    }

    if (can_start(params))
        return kStatus_Fail;

    /* Create FlexCAN handle structure and set call back function. */
    FLEXCAN_TransferCreateHandle(APP_CAN, &s_flexcanHandle, flexcan_Callback, NULL);

    s_canOpenParams = *params;

    s_canState.open = true;
    if (params->ctrlmode & CAN_CTRLMODE_FD)
        s_canState.fd = true;
    else
        s_canState.fd = false;

    /* we need this task to be higher priority than HandleSuspendTask in main,
     * in order to exit out of LPUART_RTOS_Receive safely as suspend deinits it */
    BUILD_BUG_ON(CAN_RX_TASK_PRIORITY <= SUSPEND_TASK_PRIORITY);
    xTaskCreate(can_rx_task, "CAN rx task", 256U, NULL, CAN_RX_TASK_PRIORITY, &s_canRxTask);

    return 0;
}

static int APP_CAN_close(void)
{
    if (!s_canState.open)
    {
        PRINTF("CAN is not opened\r\n");
        return kStatus_Fail;
    }

    xEventGroupSetBits(s_flexcanRxEvent, APP_FLEXCAN_CLOSE);
    while (eTaskGetState(s_canRxTask) != eDeleted)
        ;

    xEventGroupSetBits(s_flexcanTxEvent, APP_FLEXCAN_CLOSE);

    s_canState.open = false;

    FLEXCAN_Deinit(APP_CAN);

    return 0;
}

static int APP_CAN_restart(void)
{
    if (s_canState.open)
        return can_start(&s_canOpenParams);

    return 0;
}

static int APP_CAN_get_status(uint32_t *status)
{
    uint8_t txErrBuf, rxErrBuf;

    if (!s_canState.open)
        CLOCK_EnableClock(APP_CAN_CLOCK_NAME);

    FLEXCAN_GetBusErrCount(APP_CAN, &txErrBuf, &rxErrBuf);
    *status = (rxErrBuf << CAN_ECR_RXERRCNT_SHIFT) | (txErrBuf << CAN_ECR_TXERRCNT_SHIFT);

    if (!s_canState.open)
        CLOCK_DisableClock(APP_CAN_CLOCK_NAME);

    return 0;
}

static int APP_CAN_init(srtm_can_init_params_t *params)
{
    s_flexcanTxEvent = xEventGroupCreate();
    s_flexcanRxEvent = xEventGroupCreate();

    /* IRQ enable by can but priority isn't set, set it now */
    NVIC_SetPriority(APP_CAN_IRQn, APP_CAN_IRQ_PRIO);

    CLOCK_SetIpSrc(APP_CAN_CLOCK_NAME, kCLOCK_Pcc1BusIpSrcSysOscDiv2);
    RESET_PeripheralReset(kRESET_Flexcan);

    if (params->suspend_wakeup_gpio != -1)
    {
        if (APP_IO_GetIndex(params->suspend_wakeup_gpio) == 0xffff)
        {
            PRINTF("can: invalid wakeup gpio %x\r\n", params->suspend_wakeup_gpio);
            return kStatus_Fail;
        }

        uint8_t gpioIdx = APP_GPIO_IDX(params->suspend_wakeup_gpio);
        uint8_t pinIdx  = APP_PIN_IDX(params->suspend_wakeup_gpio);
        if (APP_IO_GetWUUPin(gpioIdx, pinIdx) == 255)
        {
            PRINTF("can: wakeup gpio %x has no WUU\r\n", params->suspend_wakeup_gpio);
            return kStatus_Fail;
        }
    }
    s_canInitParams = *params;

    return 0;
}

static int APP_CAN_set_wake(bool enable)
{
    if (s_canInitParams.suspend_wakeup_gpio == -1)
        return enable; /* failure if enabled */

    s_canIsWakeupSource = enable;

    return 0;
}

/**********************************************************
 * init/PM hooks called from APP_SRTM
 *********************************************************/

void APP_CAN_InitService(void)
{
    canService = SRTM_CanService_Create(APP_CAN_tx, APP_CAN_open, APP_CAN_close, APP_CAN_restart, APP_CAN_get_status,
                                        APP_CAN_init, APP_CAN_set_wake);
    SRTM_Dispatcher_RegisterService(disp, canService);
}

void APP_CAN_Suspend(void)
{
    if (s_canState.open)
        vTaskSuspend(s_canRxTask);

    if (s_canInitParams.suspend_wakeup_gpio == -1)
        return;

    uint8_t gpioIdx = APP_GPIO_IDX(s_canInitParams.suspend_wakeup_gpio);
    uint8_t pinIdx  = APP_PIN_IDX(s_canInitParams.suspend_wakeup_gpio);

    APP_IO_SetupWUU(APP_IO_GetWUUPin(gpioIdx, pinIdx),
                    s_canIsWakeupSource ? kWUU_ExternalPinFallingEdge : kWUU_ExternalPinDisable);
}

extern lpm_rtd_power_mode_e sleepWithLinux;
void APP_CAN_Resume(void)
{
    if (sleepWithLinux == LPM_PowerModeDeepSleep)
    {
        /* In Deep Sleep Mode, the FlexCAN memory partition is powered
         * down. The contents of the message buffer are preserved and no
         * reconfiguration is required. */
        UPOWER_PowerOnMemPart(0U, (uint32_t)kUPOWER_MP1_FLEXCAN);
    }

    if (s_canState.open)
        vTaskResume(s_canRxTask);
}
