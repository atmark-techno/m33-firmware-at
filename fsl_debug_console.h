/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

/* some files in nxp tree hardcode fsl_debug_console.h so provide it:
 * we just redirect to printf.h */
#include "printf.h"
