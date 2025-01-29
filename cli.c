/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>

#include "cli.h"
#include "3rdparty/EmbeddedCLI/embedded_cli.h"
#include "debug_console.h"
#include "printf.h"

#include "power_mode_switch.h"

static int CLI_help(int argc, char **argv)
{
    PRINTF("Available commands:\r\n");
    PRINTF("log   - replay logs in buffer\r\n");
    PRINTF("reset - cold reset\r\n");
    PRINTF("help  - this message\r\n");
    return CLI_Custom_help();
}

static int CLI_reset(int argc, char **argv)
{
    PRINTF("Cold reset\r\n");
    PMIC_Reset();
    return 1;
}

static int CLI_log(int argc, char **argv)
{
    DebugConsole_Replay();
    return 0;
}

static struct CLI_command CLI_commands[] = {
    { "help", CLI_help },
    { "reset", CLI_reset },
    { "log", CLI_log },
    {
        0,
    }, /* sentinel */
};

static void putch_wrapper(void *data, char ch, bool is_last)
{
    putchar(ch, is_last);
}

static bool try_commands(struct CLI_command *commands, int argc, char **argv)
{
    while (commands->command)
    {
        if (!strcmp(commands->command, argv[0]))
        {
            int rc = commands->cb(argc, argv);
            if (rc)
                PRINTF("'%s' exited with %d\r\n", argv[0], rc);

            return true;
        }
        commands++;
    }

    return false;
}

void CLI_Task(void *pvParameters)
{
    struct embedded_cli cli;

    embedded_cli_init(&cli, "M33> ", putch_wrapper, NULL);

    embedded_cli_prompt(&cli);

    while (true)
    {
        char ch = getchar();

        if (!embedded_cli_insert_char(&cli, ch))
            continue;

        /* full line, get command... */
        int argc;
        char **argv;

        argc = embedded_cli_argc(&cli, &argv);

        /* skip empty commands */
        if (argc < 1 || !argv[0] || !argv[0][0])
            goto next;

        if (try_commands(CLI_Custom_commands, argc, argv))
            goto next;
        if (try_commands(CLI_commands, argc, argv))
            goto next;

        PRINTF("Unknown command '%s'\r\n", argv[0]);

    next:
        embedded_cli_prompt(&cli);
    }
}
