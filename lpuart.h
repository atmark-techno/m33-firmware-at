/*
 * Copyright 2021-2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _LPUART_H_
#define _LPUART_H_

status_t LPUART_ReadBlockingTimes(LPUART_Type *base, uint8_t *data, size_t length, uint32_t retry_times);

#endif
