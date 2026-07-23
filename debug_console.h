/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

/* more generic */
void DebugConsole_putchar(char c, bool is_last, bool ignore_quiet);
/* input from console */
char DebugConsole_getchar(void);

/* for hardfault */
void _DebugConsole_Emergency(const char *buf, int len);
#define DebugConsole_Emergency(buf) _DebugConsole_Emergency(buf, strlen(buf))

/* lifecycle */
void DebugConsole_Init(void);
void *DebugConsole_get_uart(void);
int DebugConsole_uboot(uint32_t command);
void DebugConsole_Suspend(void);
void DebugConsole_Resume(void);
void DebugConsole_Replay(void);
void DebugConsole_Clear(void);
void DebugConsole_Quiet(bool quiet);

/* abort, gone as fsl's implem depended on its debug console at cmake level */
__attribute__((__noreturn__)) void _abort(const char *condstr, const char *func, const char *file, int line);
#define STRINGIFY(X) #X

#define abort_msg(msg) __assert_func(__FILE__, __LINE__, __func__, msg)
