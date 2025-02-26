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
    const char *help;
    const char *usage;
};
extern const struct CLI_command *CLI_Custom_commands;

/* for main loop */
void CLI_Task(void *pvParameters);

/* helpers */
int CLI_help_usage(const char *name, int retval);
