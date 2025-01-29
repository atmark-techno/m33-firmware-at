/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

/* for cli_custom.c */
struct CLI_command
{
    const char *command;
    int (*cb)(int argc, char **argv);
};
int CLI_Custom_help(void);
extern struct CLI_command *CLI_Custom_commands;

/* for power_mode_switch main loop */
void CLI_Task(void *pvParameters);
