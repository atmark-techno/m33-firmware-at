/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "srtm_io_service.h"
#include "fsl_flexio_uart.h"
#include "fsl_reset.h"

#include "app_srtm_internal.h"
#include "app_tty.h"
#include "tty.h"
#include "build_bug.h"
#include "power_mode_switch.h"

#define TTY_RX_TASK_PRIORITY (3U)
#define APP_FLEXIO_IRQ_PRIO (5U)
#define RX_POLL_TIMEOUT_MS (1)

/* flexio settings type
 * We need to remember everything from init to reinit after suspend/resume */
struct flexio_tty_settings
{
    FLEXIO_UART_Type uart_dev;
    flexio_uart_handle_t uart_handle;
    uint32_t flexio_irqn;
    uint32_t clock_ip_name;
    uint32_t clock_ip_src;
    uint32_t reset;
    uint8_t rx_pin;
    uint8_t tx_pin;
    uint32_t suspend_wakeup_gpio;
    uint32_t cflag;
    bool wakeup_source;
    TaskHandle_t rx_task;
    EventGroupHandle_t rx_event;
    EventGroupHandle_t tx_event;
    uint8_t *rx_next_buf;
    uint16_t rx_next_maxlen;
};

struct flexio_tty_settings *get_flexio(struct tty_settings *settings)
{
    BUILD_BUG_ON((uintptr_t)(((struct tty_settings *)0) + 1) % _Alignof(struct flexio_tty_settings) != 0);
    return (struct flexio_tty_settings *)(settings + 1);
}

static void flexio_tty_usercallback(FLEXIO_UART_Type *base, flexio_uart_handle_t *handle, status_t status,
                                    void *userData)
{
    struct flexio_tty_settings *flexio = userData;

    BaseType_t xResult                  = pdFAIL;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (status == kStatus_FLEXIO_UART_RxIdle)
    {
        xResult = xEventGroupSetBitsFromISR(flexio->rx_event, 1, &xHigherPriorityTaskWoken);
    }
    else if (status == kStatus_FLEXIO_UART_TxIdle)
    {
        xResult = xEventGroupSetBitsFromISR(flexio->tx_event, 1, &xHigherPriorityTaskWoken);
    }

    if (xResult != pdFAIL)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static void flexio_tty_rx_task(void *pvParameters)
{
    struct tty_settings *settings      = pvParameters;
    struct flexio_tty_settings *flexio = get_flexio(settings);

    assert(setting);
    assert(setting->type == TTY_TYPE_FLEXIO);

    /* if task was created during suspend make it wait here... */
    if (settings->state & TTY_SUSPENDED)
        vTaskSuspend(NULL);

    uint8_t *buf, *tmp_buf;
    uint16_t maxlen, next_maxlen, tmp_maxlen, tmp_maxlen2, recv_len;
    srtm_notification_t notif, next_notif, tmp_notif, tmp_notif2;

    notif                  = SRTM_TtyService_NotifyAlloc(settings->port_idx, &buf, &maxlen);
    next_notif             = SRTM_TtyService_NotifyAlloc(settings->port_idx, &flexio->rx_next_buf, &next_maxlen);
    flexio->rx_next_maxlen = next_maxlen;

    flexio_uart_transfer_t xfer = {
        .data     = buf,
        .dataSize = maxlen,
    };
    FLEXIO_UART_TransferReceiveNonBlocking(&flexio->uart_dev, &flexio->uart_handle, &xfer, NULL);

    while (true)
    {
        /* suspend if not active or suspended */
        if ((settings->state & (TTY_ACTIVE | TTY_SUSPENDED)) != TTY_ACTIVE)
            vTaskSuspend(NULL);

        /* 3 cases:
         * - no data received yet, wait a bit
         * - full read (buffer already swapped in irq)
         *   * alloc new buffer
         *   * make new buffer available for next swap
         *   * check 2nd buffer was not fully read as well
         *   * send data to linux
         * - partial read
         *   * alloc new buffer
         *   * swap buffer, no need to check if still running
         *   * send data to linux
         */
        if (flexio->rx_next_buf && flexio->uart_handle.rxDataSize == flexio->uart_handle.rxDataSizeAll)
        {
            /* wait a short time -- XXX ideally use idle irq if possible instead */
            (void)xEventGroupWaitBits(flexio->rx_event, 1, pdTRUE, pdFALSE, RX_POLL_TIMEOUT_MS);
            continue;
        }

        /* prepare new buffer to swap in */
        tmp_notif = SRTM_TtyService_NotifyAlloc(settings->port_idx, &tmp_buf, &tmp_maxlen);

        /* disable interrupt to swap buffers */
        FLEXIO_UART_DisableInterrupts(&flexio->uart_dev, kFLEXIO_UART_RxDataRegFullInterruptEnable);
        if (!flexio->rx_next_buf)
        {
            /* full read */
            if (flexio->uart_handle.rxState == kStatus_FLEXIO_UART_RxBusy)
            {
                /* still running, just fill in next pointers */
                flexio->rx_next_buf    = tmp_buf;
                flexio->rx_next_maxlen = tmp_maxlen;
                FLEXIO_UART_EnableInterrupts(&flexio->uart_dev, kFLEXIO_UART_RxDataRegFullInterruptEnable);

                /* send full read */
                SRTM_TtyService_NotifySend(ttyService, notif, maxlen);
                notif       = next_notif;
                maxlen      = next_maxlen;
                next_notif  = tmp_notif;
                next_maxlen = tmp_maxlen;
            }
            else
            {
                /* 2nd buffer was used up too.
                 * Optimal recovery speed would have us re-enable receive with single buffer
                 * but for code simpliciy just re-inits everything upfront */
                tmp_notif2 = SRTM_TtyService_NotifyAlloc(settings->port_idx, &flexio->rx_next_buf, &tmp_maxlen2);
                flexio->rx_next_maxlen = tmp_maxlen2;
                xfer.data              = tmp_buf;
                xfer.dataSize          = tmp_maxlen;

                /* re-enables irq */
                FLEXIO_UART_TransferReceiveNonBlocking(&flexio->uart_dev, &flexio->uart_handle, &xfer, NULL);

                /* send both full reads */
                SRTM_TtyService_NotifySend(ttyService, notif, maxlen);
                SRTM_TtyService_NotifySend(ttyService, next_notif, tmp_maxlen);
                notif       = tmp_notif;
                maxlen      = tmp_maxlen;
                next_notif  = tmp_notif2;
                next_maxlen = tmp_maxlen2;
            }
        }
        else
        {
            assert(flexio->uart_handle.rxDataSize < flexio->uart_handle.rxDataSizeAll);
            recv_len                       = flexio->uart_handle.rxDataSizeAll - flexio->uart_handle.rxDataSize;
            flexio->uart_handle.rxData     = tmp_buf;
            flexio->uart_handle.rxDataSize = tmp_maxlen;
            FLEXIO_UART_EnableInterrupts(&flexio->uart_dev, kFLEXIO_UART_RxDataRegFullInterruptEnable);

            /* send what we got */
            SRTM_TtyService_NotifySend(ttyService, notif, recv_len);
            notif  = tmp_notif;
            maxlen = tmp_maxlen;
        }
    }
}

static int flexio_tx(struct tty_settings *settings, uint8_t *buf, uint16_t len)
{
    struct flexio_tty_settings *flexio = get_flexio(settings);
    flexio_uart_transfer_t xfer        = {
               .data     = buf,
               .dataSize = len,
    };

    if (settings->state & TTY_SUSPENDED)
    {
        PRINTF("tty %d write while not ready (state %x)\r\n", settings->port_idx, settings->state);
        return kStatus_Fail;
    }

    FLEXIO_UART_TransferSendNonBlocking(&flexio->uart_dev, &flexio->uart_handle, &xfer);

    (void)xEventGroupWaitBits(flexio->tx_event, 1, pdTRUE, pdFALSE, portMAX_DELAY);

    return kStatus_Success;
}

static int flexio_setcflag(struct tty_settings *settings, tcflag_t cflag)
{
    struct flexio_tty_settings *flexio = get_flexio(settings);

    if (settings->state & TTY_SUSPENDED)
    {
        PRINTF("tty %d setcflags while not ready (state %x)\r\n", settings->port_idx, settings->state);
        return kStatus_Fail;
    }

    PRINTF("TODO: flexio cflags ignored\r\n");
    /* XXX re-do device init?
     * fsl_flexio_uart does not allow setting things but we have some in flexio_uart_config_t */

    /* remember for resume */
    flexio->cflag = cflag;

    return 0;
}

static int flexio_setwake(struct tty_settings *settings, bool enable)
{
    struct flexio_tty_settings *flexio = get_flexio(settings);

    if (flexio->suspend_wakeup_gpio == -1)
        return enable; /* failure if enabled */

    flexio->wakeup_source = enable;
    return 0;
}

static int flexio_activate(struct tty_settings *settings)
{
    struct flexio_tty_settings *flexio = get_flexio(settings);

    if (settings->state & TTY_ACTIVE)
    {
        /* create task on first open */
        if (!flexio->rx_task)
        {
            /* we need this task to be higher priority than HandleSuspendTask in main,
             * in order to exit out of LPUART_RTOS_Receive safely as suspend deinits it */
            BUILD_BUG_ON(TTY_RX_TASK_PRIORITY <= SUSPEND_TASK_PRIORITY);
            xTaskCreate(flexio_tty_rx_task, "flexio rx task", 256U, (void *)settings, TTY_RX_TASK_PRIORITY,
                        &flexio->rx_task);
        }
        else
        {
            vTaskResume(flexio->rx_task);
        }
    }

    return 0;
}

static int flexio_init_device(struct tty_settings *settings)
{
    struct flexio_tty_settings *flexio = get_flexio(settings);
    int rc;

    NVIC_SetPriority(flexio->flexio_irqn, APP_FLEXIO_IRQ_PRIO);
    CLOCK_SetIpSrcDiv(flexio->clock_ip_name, flexio->clock_ip_src, 0, 0);
    RESET_PeripheralReset(flexio->reset);

    flexio_uart_config_t config;
    FLEXIO_UART_GetDefaultConfig(&config);
    config.baudRate_Bps = tty_baudrate(flexio->cflag);
    config.enableUart   = true;

    rc = FLEXIO_UART_Init(&flexio->uart_dev, &config, CLOCK_GetIpFreq(flexio->clock_ip_name));
    if (rc)
    {
        PRINTF("port %d flexio init fail: %d (baud rate %d / clock %d)\r\n", settings->port_idx, rc,
               config.baudRate_Bps, CLOCK_GetIpFreq(flexio->clock_ip_name));

        return rc;
    }

    rc = FLEXIO_UART_TransferCreateHandle(&flexio->uart_dev, &flexio->uart_handle, flexio_tty_usercallback, flexio);
    if (rc)
    {
        PRINTF("port %d flexio handle init fail: %d\r\n", settings->port_idx, rc);
        return rc;
    }

    return 0;
}

static int flexio_init(struct tty_settings *settings, struct srtm_tty_init_payload *generic_init)
{
    struct flexio_tty_settings *flexio        = get_flexio(settings);
    struct srtm_tty_init_flexio_payload *init = &generic_init->flexio;
    int rc;

    PRINTF("initializing tty %d as FLEXIO %x\r\n", settings->port_idx, init->flexio_index);

    switch (init->flexio_index)
    {
        case 0:
            flexio->uart_dev.flexioBase = FLEXIO0;
            flexio->flexio_irqn         = FLEXIO0_IRQn;
            flexio->clock_ip_name       = kCLOCK_Flexio0;
            flexio->clock_ip_src        = kCLOCK_Pcc0BusIpSrcSysOscDiv2;
            flexio->reset               = kRESET_Flexio0;
            break;
        default:
            PRINTF("flexio index %d not supported\r\n", init->flexio_index);
            return kStatus_Fail;
    }

    if (flexio_used)
    {
        PRINTF("tty: flexio %d already used by another driver, refusing to init\r\n", init->flexio_index);
        return kStatus_Fail;
    }
    flexio_used = true;

    flexio->uart_dev.TxPinIndex      = init->flexio_tx_pin;
    flexio->uart_dev.RxPinIndex      = init->flexio_rx_pin;
    flexio->uart_dev.shifterIndex[0] = 0;
    flexio->uart_dev.shifterIndex[1] = 1;
    flexio->uart_dev.timerIndex[0]   = 0;
    flexio->uart_dev.timerIndex[1]   = 1;
    flexio->cflag                    = init->cflag;
    flexio->suspend_wakeup_gpio      = init->suspend_wakeup_gpio;
    flexio->rx_event                 = xEventGroupCreate();
    if (!flexio->rx_event)
        return kStatus_Fail;
    flexio->tx_event = xEventGroupCreate();
    if (!flexio->tx_event)
    {
        vEventGroupDelete(flexio->rx_event);
        return kStatus_Fail;
    }

    if (flexio->suspend_wakeup_gpio != -1)
    {
        if (APP_IO_GetIndex(flexio->suspend_wakeup_gpio) == 0xffff)
        {
            PRINTF("tty %d: invalid wakeup gpio %x\r\n", settings->port_idx, flexio->suspend_wakeup_gpio);
            return kStatus_Fail;
        }

        uint8_t gpioIdx = APP_GPIO_IDX(flexio->suspend_wakeup_gpio);
        uint8_t pinIdx  = APP_PIN_IDX(flexio->suspend_wakeup_gpio);
        if (APP_IO_GetWUUPin(gpioIdx, pinIdx) == 255)
        {
            PRINTF("tty %d: wakeup gpio %x has no WUU\r\n", settings->port_idx, flexio->suspend_wakeup_gpio);
            return kStatus_Fail;
        }
    }

    rc = flexio_init_device(settings);
    if (rc)
        return rc;

    return 0;
}

void flexio_resumeTask(struct tty_settings *settings)
{
    struct flexio_tty_settings *flexio = get_flexio(settings);

    if (settings->state & TTY_ACTIVE)
    {
        vTaskResume(flexio->rx_task);
    }
}

void flexio_suspend(struct tty_settings *settings)
{
    struct flexio_tty_settings *flexio = get_flexio(settings);

    FLEXIO_UART_Deinit(&flexio->uart_dev);

    if (flexio->suspend_wakeup_gpio == -1)
        return;

    uint8_t gpioIdx = APP_GPIO_IDX(flexio->suspend_wakeup_gpio);
    uint8_t pinIdx  = APP_PIN_IDX(flexio->suspend_wakeup_gpio);

    APP_IO_SetupWUU(APP_IO_GetWUUPin(gpioIdx, pinIdx),
                    flexio->wakeup_source ? kWUU_ExternalPinRisingEdge : kWUU_ExternalPinDisable);
    if (flexio->wakeup_source)
    {
        /* This runs after APP_Suspend() that does this for other IOs,
         * so set pinmux to wuu manually. Restore is handled by APP_Resume() */
        PinMuxPrepareSuspend(gpioIdx, pinIdx);
    }
}

void flexio_resume(struct tty_settings *settings)
{
    flexio_init_device(settings);
}

/* manually added to tty_hooks top of app_tty.c */
const struct tty_hooks tty_flexio_hooks = {
    .tx            = flexio_tx,
    .setcflag      = flexio_setcflag,
    .setwake       = flexio_setwake,
    .activate      = flexio_activate,
    .init          = flexio_init,
    .resumeTask    = flexio_resumeTask,
    .suspend       = flexio_suspend,
    .resume        = flexio_resume,
    .settings_size = sizeof(struct flexio_tty_settings),
};
