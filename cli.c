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

/* remember quiet state as we toggle it everytime we run a command */
static bool cli_quiet;

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

static int CLI_quiet(int argc, char **argv)
{
    DebugConsole_Quiet(true);
    cli_quiet = true;
    return 0;
}

static int CLI_verbose(int argc, char **argv)
{
    DebugConsole_Quiet(false);
    cli_quiet = false;
    return 0;
}

static int CLI_clear(int argc, char **argv)
{
    DebugConsole_Clear();
    return 0;
}

static int CLI_help(int argc, char **argv);
static const struct CLI_command CLI_commands[] = {
    { "?", CLI_help, NULL },
    { "help", CLI_help, "this help" },
    { "reset", CLI_reset, "cold reset" },
    { "log", CLI_log, "print buffer log" },
    { "clear", CLI_clear, "clear buffer log" },
    { "quiet", CLI_quiet, "disable background messages" },
    { "verbose", CLI_verbose, "enable background messages" },
    {
        0,
    }, /* sentinel */
};

#define foreach_command(command)                                                                       \
    for (const struct CLI_command *commands = CLI_Custom_commands; commands;                           \
         commands                           = (commands == CLI_Custom_commands) ? CLI_commands : NULL) \
        for (command = commands; command->command; command++)

static int CLI_help(int argc, char **argv)
{
    int maxlength = 0;
    const struct CLI_command *command;

    foreach_command(command)
    {
        if (!command->help)
            continue;
        int len   = strlen(command->command);
        maxlength = MAX(len, maxlength);
    }
    foreach_command(command)
    {
        if (!command->help)
            continue;
        PRINTF("%*s - %s\r\n", -maxlength, command->command, command->help);
    }
    return 0;
}

static void putch_wrapper(void *data, char ch, bool is_last)
{
    putchar(ch, is_last, /* ignore_quiet */ true);
}

static bool try_commands(int argc, char **argv)
{
    const struct CLI_command *command;

    foreach_command(command)
    {
        if (!strcmp(command->command, argv[0]))
        {
            int rc = command->cb(argc, argv);
            if (rc)
                PRINTF("'%s' exited with %d\r\n", argv[0], rc);

            return true;
        }
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

        /* Temporarily re-allow output for our commands */
        DebugConsole_Quiet(false);

        if (try_commands(argc, argv))
            goto next;

        PRINTF("Unknown command '%s'\r\n", argv[0]);

    next:
        embedded_cli_prompt(&cli);

        if (cli_quiet)
            DebugConsole_Quiet(true);
    }
}
