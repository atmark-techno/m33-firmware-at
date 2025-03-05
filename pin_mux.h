/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

void BOARD_InitBootPins(void);
void BOARD_InitConsolePins(void);
void BOARD_InitPmicI2cPins(void);
void BOARD_InitPmicModePins(void);
void BOARD_InitWdogPins(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
