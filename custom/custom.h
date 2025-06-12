/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

#include "lpm.h"

void custom_early_init(void);
void custom_init(void);
void custom_linux_boot(void);
void custom_early_suspend(void);
void custom_suspend(void);
void custom_resume(void);
void custom_late_resume(void);
void custom_m33_suspend(lpm_rtd_power_mode_e targetMode);
void custom_m33_resume(lpm_rtd_power_mode_e targetMode);
