/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

/* for printf, output one char to console */
void putchar_(char c);

/* input from console */
char getchar(void);

/* lifecycle */
void DebugConsole_Init(void);
void DebugConsole_Suspend(void);
void DebugConsole_Resume(void);

/* abort, gone as fsl's implem depended on its debug console at cmake level */
__attribute__((__noreturn__)) void _abort(const char *condstr, const char *func, const char *file, int line);
#define STRINGIFY(X) #X

/* defined as noop by newlib in some files for some reason... override */
#undef assert
#define assert(cond) \
    if (!(cond))     \
    _abort(STRINGIFY(cond), __func__, __FILE__, __LINE__)
