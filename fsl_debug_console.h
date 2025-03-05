/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* /!\ same guard as 3rdparty/printf/printf.h */
#ifndef PRINTF_H_

/* some files in nxp tree hardcode fsl_debug_console.h so provide it:
 * we just redirect to printf.h -- with format warnings ignored on top
 * so we don't need to fix these...
 * We only do this if printf.h was not included before, because srtm code
 * will always include this as srtm_config.h is included in NXP files that
 * have such warnings... So each file need to opt in to printf.h to get
 * warnings at this point: TODO make this work based on included file path? */
#pragma GCC diagnostic ignored "-Wformat"
#include "printf.h"

#endif
