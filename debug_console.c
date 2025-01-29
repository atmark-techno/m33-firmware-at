/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FreeRTOS.h"
#include "task.h"

#include "fsl_lpuart.h"
#include "fsl_reset.h"
#include "lpuart.h"

#include "debug_console.h"
#include "app_tty_console.h"
#include "printf.h"
#include "build_bug.h"

/* build time configuration -- priority over runtime settings if not empty */
//#define DISABLE_RPMSG 1

/* we will stop using this soon so copy from board.h */
#define DBG_UART LPUART1
#define DBG_UART_BAUDRATE 115200
#define DBG_UART_CLK kCLOCK_Lpuart1
#define DBG_UART_IDX 1
#define DBG_UART_CLKSRC kCLOCK_Pcc1BusIpSrcSysOscDiv2
#define DBG_UART_RESET kRESET_Lpuart1

/* can't use immediately on boot */
static bool consoleSuspended = true;

/* internal buffer */
#define CONSOLE_BUFLEN 4096
static uint8_t consoleBuffer[CONSOLE_BUFLEN];
static int consoleBufferStart, consoleBufferEnd;
static bool consoleBufferFull;

/**********************************************************
 * init/PM hooks
 *********************************************************/

void DebugConsole_Init(void)
{
    CLOCK_SetIpSrc(DBG_UART_CLK, DBG_UART_CLKSRC);
    RESET_PeripheralReset(DBG_UART_RESET);
    lpuart_config_t config;
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = DBG_UART_BAUDRATE;
    config.enableTx     = true;
    config.enableRx     = true;
    LPUART_Init(DBG_UART, &config, CLOCK_GetLpuartClkFreq(DBG_UART_IDX));
    consoleSuspended = false;
}
void DebugConsole_Suspend(void)
{
    consoleSuspended = true;
}
void DebugConsole_Resume(void)
{
    DebugConsole_Init();
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

        if (LPUART_ReadBlockingTimes(DBG_UART, &ch, 1, 100) == kStatus_Success)
            break;
    }

    return (char)ch;
}

/**********************************************************
 * output
 *********************************************************/

static void send_all(uint8_t *buf, int len)
{
    LPUART_WriteBlocking(DBG_UART, buf, len);
#ifndef DISABLE_RPMSG
    APP_TTY_Console_Write(buf, len);
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

void putchar(char c, bool is_last)
{
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

    if (is_last)
        flush();
}

/* for printf */
void putchar_(char c)
{
    /* line buffered */
    putchar(c, c == '\n');
}

void DebugConsole_Replay(void)
{
    if (consoleBufferFull)
    {
        send_all(consoleBuffer + consoleBufferEnd, CONSOLE_BUFLEN - consoleBufferEnd);
    }
    send_all(consoleBuffer, consoleBufferEnd);
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
