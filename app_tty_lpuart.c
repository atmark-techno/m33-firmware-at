/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "srtm_io_service.h"
#include "fsl_reset.h"

#include "app_tty.h"
#include "tty.h"
#include "build_bug.h"
#include "main.h"

#define TTY_RX_TASK_PRIORITY (3U)
#define APP_LPUART_IRQ_PRIO (5U)

/* lpuart settings type
 * We need to remember everything from init to reinit after suspend/resume */
struct lpuart_tty_settings
{
    LPUART_Type *uart_base;
    uint32_t uart_irqn;
    uint32_t clock_ip_name;
    uint32_t clock_ip_src;
    uint32_t reset;
    uint32_t rs485_flags;
    uint32_t rs485_de_gpio;
    uint32_t cflag;
    uint32_t suspend_wakeup_gpio; /* rx pin to use for wakeup */
    enum tty_state state;
    bool wakeup_source;
    TaskHandle_t rx_task;
    lpuart_rtos_handle_t lpuart_rtos_handle;
    lpuart_handle_t lpuart_handle;
    uint8_t *background_buffer;
    size_t background_buffer_size;
};

struct lpuart_tty_settings *get_lpuart(struct tty_settings *settings)
{
    BUILD_BUG_ON((uintptr_t)(((struct tty_settings *)0) + 1) % _Alignof(struct lpuart_tty_settings) != 0);
    return (struct lpuart_tty_settings *)(settings + 1);
}

static void lpuart_tty_rx_task(void *pvParameters)
{
    struct tty_settings *settings      = pvParameters;
    struct lpuart_tty_settings *lpuart = get_lpuart(settings);

    assert(settings);
    assert(settings->type == TTY_TYPE_LPUART);

    /* if task was created during suspend make it wait here... */
    if (settings->state & TTY_SUSPENDED)
        vTaskSuspend(NULL);

    uint8_t *buf;
    uint16_t maxlen;
    while (true)
    {
        size_t recvlen = 0;

        srtm_notification_t notif = SRTM_TtyService_NotifyAlloc(settings->port_idx, &buf, &maxlen);
        if (!notif)
        {
            // message already printed in alloc failure
            SDK_DelayAtLeastUs(1000000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
            continue;
        }

        do
        {
            LPUART_RTOS_Receive(&lpuart->lpuart_rtos_handle, buf, maxlen, &recvlen);

            /* suspend if not active or suspended */
            if ((settings->state & (TTY_ACTIVE | TTY_SUSPENDED)) != TTY_ACTIVE)
                vTaskSuspend(NULL);
        } while (recvlen == 0);

        SRTM_TtyService_NotifySend(ttyService, notif, recvlen);
    }
}

static int lpuart_tx(struct tty_settings *settings, uint8_t *buf, uint16_t len)
{
    struct lpuart_tty_settings *lpuart = get_lpuart(settings);
    int rc;

    if (settings->state & TTY_SUSPENDED)
    {
        PRINTF("tty %d write while not ready (state %x)\r\n", settings->port_idx, settings->state);
        return kStatus_Fail;
    }

    rc = LPUART_RTOS_Send(&lpuart->lpuart_rtos_handle, buf, len);
    if (rc)
    {
        PRINTF("%d send fail for buf len %d, first byte %x: %d\r\n", settings->port_idx, len, len > 0 ? buf[0] : 0, rc);
    }
    return rc;
}

static int setcflag(struct tty_settings *settings, tcflag_t cflag)
{
    struct lpuart_tty_settings *lpuart = get_lpuart(settings);
    speed_t baudrate                   = tty_baudrate(cflag);
    lpuart_parity_mode_t parity        = tty_parity(cflag);
    bool cmsparity                     = tty_cmsparity(cflag);
    lpuart_data_bits_t databits        = tty_databits(cflag);
    lpuart_stop_bit_count_t stopbits   = tty_stopbits(cflag);

    /*
     * only support CS8 and CS7, and for CS7 must enable parity.
     * supported mode:
     *  - (7,e/o,1/2)
     *  - (8,n,1/2)
     *  - (8,m/s,1/2)
     *  - (8,e/o,1/2)
     */
    if (parity == kLPUART_ParityDisabled && databits == kLPUART_SevenDataBits)
    {
        PRINTF("Invalid cflag: 0x%x\r\n", cflag);
        return kStatus_Fail;
    }

    /* set baud rate */
    LPUART_SetBaudRate(lpuart->uart_base, baudrate, CLOCK_GetIpFreq(lpuart->clock_ip_name));

    /* set databits */
    if (databits == kLPUART_EightDataBits && (parity != kLPUART_ParityDisabled || cmsparity))
        LPUART_Enable9bitMode(lpuart->uart_base, true); /* (8,e/o) or (8,m/s) */
    else
        LPUART_Enable9bitMode(lpuart->uart_base, false); /* (7,e/o) or (8,n) */

    /* set parity */
    LPUART_SetParity(lpuart->uart_base, parity); /* LPUART_Enable9bitMode() may disable parity.
                                               Do not call LPUART_SetParity() first. */

    /* set stop bits */
    LPUART_SetStopBit(lpuart->uart_base, stopbits);

    /* remember for resume */
    lpuart->cflag = cflag;

    return 0;
}

static int lpuart_setcflag(struct tty_settings *settings, tcflag_t cflag)
{
    if (settings->state & TTY_SUSPENDED)
    {
        PRINTF("tty %d setcflags while not ready (state %x)\r\n", settings->port_idx, settings->state);
        return kStatus_Fail;
    }

    return setcflag(settings, cflag);
}

static int lpuart_setwake(struct tty_settings *settings, bool enable)
{
    struct lpuart_tty_settings *lpuart = get_lpuart(settings);

    if (lpuart->suspend_wakeup_gpio == -1)
        return enable; /* failure if enabled */

    lpuart->wakeup_source = enable;
    return 0;
}

static int lpuart_activate(struct tty_settings *settings)
{
    struct lpuart_tty_settings *lpuart = get_lpuart(settings);

    if (settings->state & TTY_ACTIVE)
    {
        /* create task on first open */
        if (!lpuart->rx_task)
        {
            /* we need this task to be higher priority than HandleSuspendTask in main,
             * in order to exit out of LPUART_RTOS_Receive safely as suspend deinits it */
            BUILD_BUG_ON(TTY_RX_TASK_PRIORITY <= SUSPEND_TASK_PRIORITY);
            xTaskCreate(lpuart_tty_rx_task, "lpuart rx task", 256U, (void *)settings, TTY_RX_TASK_PRIORITY,
                        &lpuart->rx_task);
        }
        else
        {
            vTaskResume(lpuart->rx_task);
        }
    }

    return 0;
}

static int lpuart_init_device(struct tty_settings *settings)
{
    struct lpuart_tty_settings *lpuart = get_lpuart(settings);
    int rc;

    NVIC_SetPriority(lpuart->uart_irqn, APP_LPUART_IRQ_PRIO);
    CLOCK_SetIpSrc(lpuart->clock_ip_name, lpuart->clock_ip_src);
    RESET_PeripheralReset(lpuart->reset);

    lpuart_rtos_config_t config = {
        .base = lpuart->uart_base,
        .baudrate = tty_baudrate(lpuart->cflag),
        .parity = tty_parity(lpuart->cflag),
        .stopbits = tty_stopbits(lpuart->cflag),
        .buffer = lpuart->background_buffer,
        .buffer_size = lpuart->background_buffer_size,
        .rs485 = {
            .flags = lpuart->rs485_flags,
            .deGpio = lpuart->rs485_de_gpio,
        },
        .srcclk = CLOCK_GetIpFreq(lpuart->clock_ip_name),
    };

    rc = LPUART_RTOS_Init(&lpuart->lpuart_rtos_handle, &lpuart->lpuart_handle, &config);
    if (rc)
    {
        PRINTF("port %d lpuart init fail: %d\r\n", settings->port_idx, rc);
        return rc;
    }

    LPUART_RTOS_SetRxTimeout(&lpuart->lpuart_rtos_handle, 1, 0); /* short timeout to give data back asap */
    LPUART_RTOS_SetTxTimeout(&lpuart->lpuart_rtos_handle, 0, 0); /* XXX no timeout, depends on baud rate */

    return setcflag(settings, lpuart->cflag);
}

static int lpuart_init(struct tty_settings *settings, struct srtm_tty_init_payload *generic_init)
{
    struct lpuart_tty_settings *lpuart        = get_lpuart(settings);
    struct srtm_tty_init_lpuart_payload *init = &generic_init->lpuart;
    int rc;

    PRINTF("initializing tty %d as LPUART %x\r\n", settings->port_idx, init->uart_index);

    switch (init->uart_index)
    {
        case 0:
            lpuart->uart_base     = LPUART0;
            lpuart->uart_irqn     = LPUART0_IRQn;
            lpuart->clock_ip_name = kCLOCK_Lpuart0;
            lpuart->clock_ip_src  = kCLOCK_Pcc1BusIpSrcSysOscDiv2;
            lpuart->reset         = kRESET_Lpuart0;
            break;
        case 1:
            lpuart->uart_base     = LPUART1;
            lpuart->uart_irqn     = LPUART1_IRQn;
            lpuart->clock_ip_name = kCLOCK_Lpuart1;
            lpuart->clock_ip_src  = kCLOCK_Pcc1BusIpSrcSysOscDiv2;
            lpuart->reset         = kRESET_Lpuart1;
            break;
        default:
            PRINTF("lpuart index %d not supported\r\n", init->uart_index);
            return kStatus_Fail;
    }

    lpuart->rs485_flags         = init->rs485_flags;
    lpuart->rs485_de_gpio       = init->rs485_de_gpio;
    lpuart->cflag               = init->cflag;
    lpuart->suspend_wakeup_gpio = init->suspend_wakeup_gpio;

    if (lpuart->rs485_flags && APP_IO_GetIndex(lpuart->rs485_de_gpio) == 0xffff)
    {
        PRINTF("tty %d: invalid rs485_de gpio %x\r\n", settings->port_idx, lpuart->rs485_de_gpio);
        return kStatus_Fail;
    }
    if (lpuart->suspend_wakeup_gpio != -1)
    {
        if (APP_IO_GetIndex(lpuart->suspend_wakeup_gpio) == 0xffff)
        {
            PRINTF("tty %d: invalid wakeup gpio %x\r\n", settings->port_idx, lpuart->suspend_wakeup_gpio);
            return kStatus_Fail;
        }

        uint8_t gpioIdx = APP_GPIO_IDX(lpuart->suspend_wakeup_gpio);
        uint8_t pinIdx  = APP_PIN_IDX(lpuart->suspend_wakeup_gpio);
        if (APP_IO_GetWUUPin(gpioIdx, pinIdx) == 255)
        {
            PRINTF("tty %d: wakeup gpio %x has no WUU\r\n", settings->port_idx, lpuart->suspend_wakeup_gpio);
            return kStatus_Fail;
        }
    }

    /* This buffer is only used for the short period of time during which rx task
     * allocates a new buffer and does not need to be big */
    lpuart->background_buffer_size = 128;
    lpuart->background_buffer      = pvPortMalloc(lpuart->background_buffer_size);
    if (!lpuart->background_buffer)
    {
        return kStatus_Fail;
    }

    rc = lpuart_init_device(settings);
    if (rc)
        return rc;

    return 0;
}

void lpuart_resumeTask(struct tty_settings *settings)
{
    struct lpuart_tty_settings *lpuart = get_lpuart(settings);

    if (lpuart->state & TTY_ACTIVE)
    {
        vTaskResume(lpuart->rx_task);
    }
}

void lpuart_suspend(struct tty_settings *settings)
{
    struct lpuart_tty_settings *lpuart = get_lpuart(settings);

    LPUART_RTOS_Deinit(&lpuart->lpuart_rtos_handle);

    if (lpuart->suspend_wakeup_gpio == -1)
        return;

    uint8_t gpioIdx = APP_GPIO_IDX(lpuart->suspend_wakeup_gpio);
    uint8_t pinIdx  = APP_PIN_IDX(lpuart->suspend_wakeup_gpio);

    APP_IO_SetupWUU(APP_IO_GetWUUPin(gpioIdx, pinIdx),
                    lpuart->wakeup_source ? kWUU_ExternalPinRisingEdge : kWUU_ExternalPinDisable);
}

void lpuart_resume(struct tty_settings *settings)
{
    lpuart_init_device(settings);
}

/* manually added to tty_hooks top of app_tty.c */
const struct tty_hooks tty_lpuart_hooks = {
    .tx            = lpuart_tx,
    .setcflag      = lpuart_setcflag,
    .setwake       = lpuart_setwake,
    .activate      = lpuart_activate,
    .init          = lpuart_init,
    .resumeTask    = lpuart_resumeTask,
    .suspend       = lpuart_suspend,
    .resume        = lpuart_resume,
    .settings_size = sizeof(struct lpuart_tty_settings),
};
