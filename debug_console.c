/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>

#include "FreeRTOS.h"
#include "task.h"

#include "fsl_lpuart.h"
#include "fsl_reset.h"
#include "fsl_iomuxc.h"
#include "lpuart.h"

#include "debug_console.h"
#include "app_tty_console.h"
#include "printf.h"
#include "build_bug.h"

/* build time configuration -- priority over runtime settings if not empty.
 * Default is uart disabled until uboot picks one, rpmsg enabled in dts */
//#define DEFAULT_UART_CONSOLE_TX 22
//#define DEFAULT_UART_CONSOLE_RX 23
//#define DISABLE_RPMSG 1
#define DBG_UART_BAUDRATE 115200

/* console selection */
static LPUART_Type *debug_uart;

/* can't use debug_uart if either debug_uart == NULL or consoleSuspsend */
static bool consoleSuspended;
static bool consoleQuiet;

/* internal buffer */
#define CONSOLE_BUFLEN 4096
static uint8_t consoleBuffer[CONSOLE_BUFLEN];
static int consoleBufferStart, consoleBufferEnd;
static bool consoleBufferFull;

/**********************************************************
 * init/PM hooks
 *********************************************************/

static int DebugConsole_InitDevice(LPUART_Type *uart)
{
    uint32_t reset, clock_ip_name, clock_ip_src;
    int uart_idx;

    if (!uart)
    {
        return 0;
    }

    if (uart == LPUART0)
    {
        uart_idx      = 0;
        reset         = kRESET_Lpuart0;
        clock_ip_name = kCLOCK_Lpuart0;
        clock_ip_src  = kCLOCK_Pcc1BusIpSrcSysOscDiv2;
    }
    else if (uart == LPUART1)
    {
        uart_idx      = 1;
        reset         = kRESET_Lpuart1;
        clock_ip_name = kCLOCK_Lpuart1;
        clock_ip_src  = kCLOCK_Pcc1BusIpSrcSysOscDiv2;
    }
    else
    {
        return EINVAL;
    }

    CLOCK_SetIpSrc(clock_ip_name, clock_ip_src);
    RESET_PeripheralReset(reset);
    lpuart_config_t config;
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = DBG_UART_BAUDRATE;
    config.enableTx     = true;
    config.enableRx     = true;
    LPUART_Init(uart, &config, CLOCK_GetLpuartClkFreq(uart_idx));
    debug_uart = uart;

    return 0;
}

void DebugConsole_Suspend(void)
{
    consoleSuspended = true;
}
void DebugConsole_Resume(void)
{
    DebugConsole_InitDevice(debug_uart);
    consoleSuspended = false;
}

int DebugConsole_uboot(uint32_t command)
{
    /* only accept if not hardcoded (or first command) */
    if (debug_uart)
        return EBUSY;

    uint8_t pin_tx = (command >> 8) & 0xff;
    uint8_t pin_rx = (command >> 16) & 0xff;
    LPUART_Type *uart;

    switch (pin_tx)
    {
        case 2:
            IOMUXC_SetPinMux(IOMUXC_PTA2_LPUART0_TX, 0U);
            IOMUXC_SetPinConfig(IOMUXC_PTA2_LPUART0_TX, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
            uart = LPUART0;
            break;
        case 14:
            IOMUXC_SetPinMux(IOMUXC_PTA14_LPUART0_TX, 0U);
            IOMUXC_SetPinConfig(IOMUXC_PTA14_LPUART0_TX, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
            uart = LPUART0;
            break;
        case 18:
            IOMUXC_SetPinMux(IOMUXC_PTA18_LPUART0_TX, 0U);
            IOMUXC_SetPinConfig(IOMUXC_PTA18_LPUART0_TX, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
            uart = LPUART0;
            break;
        case 6:
            IOMUXC_SetPinMux(IOMUXC_PTA6_LPUART1_TX, 0U);
            IOMUXC_SetPinConfig(IOMUXC_PTA6_LPUART1_TX, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
            uart = LPUART1;
            break;
        case 10:
            IOMUXC_SetPinMux(IOMUXC_PTA10_LPUART1_TX, 0U);
            IOMUXC_SetPinConfig(IOMUXC_PTA10_LPUART1_TX, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
            uart = LPUART1;
            break;
        case 22:
            IOMUXC_SetPinMux(IOMUXC_PTA22_LPUART1_TX, 0U);
            IOMUXC_SetPinConfig(IOMUXC_PTA22_LPUART1_TX, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
            uart = LPUART1;
            break;
        default:
            goto inval;
    }
    switch (pin_rx)
    {
        case 3:
            if (uart != LPUART0)
                goto inval;
            IOMUXC_SetPinMux(IOMUXC_PTA3_LPUART0_RX, 0U);
            IOMUXC_SetPinConfig(IOMUXC_PTA3_LPUART0_RX, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
            break;
        case 15:
            if (uart != LPUART0)
                goto inval;
            IOMUXC_SetPinMux(IOMUXC_PTA15_LPUART0_RX, 0U);
            IOMUXC_SetPinConfig(IOMUXC_PTA15_LPUART0_RX, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
            break;
        case 19:
            if (uart != LPUART0)
                goto inval;
            IOMUXC_SetPinMux(IOMUXC_PTA19_LPUART0_RX, 0U);
            IOMUXC_SetPinConfig(IOMUXC_PTA19_LPUART0_RX, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
            break;
        case 7:
            if (uart != LPUART1)
                goto inval;
            IOMUXC_SetPinMux(IOMUXC_PTA7_LPUART1_RX, 0U);
            IOMUXC_SetPinConfig(IOMUXC_PTA7_LPUART1_RX, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
            break;
        case 11:
            if (uart != LPUART1)
                goto inval;
            IOMUXC_SetPinMux(IOMUXC_PTA11_LPUART1_RX, 0U);
            IOMUXC_SetPinConfig(IOMUXC_PTA11_LPUART1_RX, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
            break;
        case 23:
            if (uart != LPUART1)
                goto inval;
            IOMUXC_SetPinMux(IOMUXC_PTA23_LPUART1_RX, 0U);
            IOMUXC_SetPinConfig(IOMUXC_PTA23_LPUART1_RX, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
            break;
        default:
            goto inval;
    }

    int rc = DebugConsole_InitDevice(uart);
    if (rc)
        return rc;

    /* replay any previous log */
    DebugConsole_Replay();

    PRINTF("Init console on %d/%d\r\n", pin_tx, pin_rx);

    return 0;

inval:
    PRINTF("Invalid console %d/%d\r\n", pin_tx, pin_rx);
    return EINVAL;
}

void DebugConsole_Init(void)
{
#ifdef DEFAULT_UART_CONSOLE_TX
    /* serialize as uboot would to reuse code */
    DebugConsole_uboot((DEFAULT_UART_CONSOLE_TX << 8) | (DEFAULT_UART_CONSOLE_RX << 16));
#endif
}

void *DebugConsole_get_uart(void)
{
    return debug_uart;
}

/**********************************************************
 * input
 *********************************************************/

char getchar(void)
{
    uint8_t ch;

    while (true)
    {
        while (consoleSuspended)
            vTaskDelay(pdMS_TO_TICKS(1));

#ifndef DISABLE_RPMSG
        /* try all possible inputs */
        if (APP_TTY_Console_Getchar(&ch) == 0)
            break;
#endif

        if (debug_uart && LPUART_ReadBlockingTimes(debug_uart, &ch, 1, 100) == kStatus_Success)
            break;
    }

    return (char)ch;
}

/**********************************************************
 * output
 *********************************************************/

static void send_all(uint8_t *buf, int len)
{
    if (debug_uart)
        LPUART_WriteBlocking(debug_uart, buf, len);

#ifndef DISABLE_RPMSG
    /* split one line at a time so as to not send \r to linux.
     * This is suboptimal but debug console is not meant to be high
     * throughput... */
    int start, end;
    for (start = 0, end = 0; end < len; end++)
    {
        if (buf[end] != '\r')
            continue;
        if (start != end)
        {
            APP_TTY_Console_Write(buf + start, end - start);
        }
        start = end + 1;
    }
    if (start != end)
        APP_TTY_Console_Write(buf + start, end - start);
#endif
}

static void flush(void)
{
    if (consoleSuspended)
        return;

    /* we will buffer in an intermediate buffer next */
    if (consoleBufferEnd < consoleBufferStart)
    {
        /* looped */
        send_all(consoleBuffer + consoleBufferStart, CONSOLE_BUFLEN - consoleBufferStart);
        consoleBufferStart = 0;
    }
    send_all(consoleBuffer + consoleBufferStart, consoleBufferEnd - consoleBufferStart);
    consoleBufferStart = consoleBufferEnd;
}

void putchar(char c, bool is_last, bool ignore_quiet)
{
    if (consoleQuiet && !ignore_quiet && consoleBufferEnd != consoleBufferStart)
    {
        /* If quiet we need to flush now to avoid buffer mixing together */
        flush();
    }

    consoleBuffer[consoleBufferEnd++] = c;
    if (consoleBufferEnd == CONSOLE_BUFLEN)
    {
        consoleBufferEnd  = 0;
        consoleBufferFull = true;
    }
    if (consoleBufferEnd == consoleBufferStart)
        consoleBufferStart++;
    if (consoleBufferStart == CONSOLE_BUFLEN)
        consoleBufferStart = 0;

    if (consoleQuiet && !ignore_quiet)
    {
        /* if quiet, also advance start so message is not printed on next flush */
        consoleBufferStart = consoleBufferEnd;
        return;
    }

    if (is_last)
        flush();
}

/* for printf */
void putchar_(char c)
{
    /* line buffered */
    putchar(c, /* is_last */ c == '\n', /* ignore_quiet */ false);
}

void DebugConsole_Replay(void)
{
    if (consoleBufferFull)
    {
        send_all(consoleBuffer + consoleBufferEnd, CONSOLE_BUFLEN - consoleBufferEnd);
    }
    if (consoleBufferEnd)
        send_all(consoleBuffer, consoleBufferEnd);
}

void DebugConsole_Clear(void)
{
    consoleBufferFull  = 0;
    consoleBufferEnd   = 0;
    consoleBufferStart = 0;
}

void DebugConsole_Quiet(bool quiet)
{
    consoleQuiet = quiet;
}

/**********************************************************
 * abort
 *********************************************************/

__attribute__((__noreturn__)) void _abort(const char *condstr, const char *func, const char *file, int line)
{
    /* we're already screwed so might as well ignore suspended state... */
    consoleSuspended = false;
    PRINTF("%s:%d: %s: Assert failure %s\r\n", file, line, func, condstr);
    while (1)
        ;
}

void _DebugConsole_Emergency(const char *buf, int len)
{
    if (!debug_uart)
        return;

    /* manual flush */
    if (consoleBufferEnd < consoleBufferStart)
    {
        LPUART_WriteBlocking(debug_uart, consoleBuffer + consoleBufferStart, CONSOLE_BUFLEN - consoleBufferStart);
        consoleBufferStart = 0;
    }
    LPUART_WriteBlocking(debug_uart, consoleBuffer + consoleBufferStart, consoleBufferEnd - consoleBufferStart);
    consoleBufferStart = consoleBufferEnd;

    /* And then directly print to uart */
    LPUART_WriteBlocking(debug_uart, (uint8_t *)buf, len);
}
