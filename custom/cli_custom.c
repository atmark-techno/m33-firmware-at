/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Use this file to define custom commands */

#include "cli.h"
#include "printf.h"

static int customize_me(int argc, char **argv)
{
    PRINTF("Do something here\r\n");
    return 0;
}

static struct CLI_command custom_commands[] = {
    { "customize_me", customize_me },
    {
        0,
    }, /* sentinel */
};
struct CLI_command *CLI_Custom_commands = custom_commands;

int CLI_Custom_help(void)
{
    // PRINTF("command\r\n");
    return 0;
}
