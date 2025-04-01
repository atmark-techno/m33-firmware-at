/*
 * Copyright 2025 Atmark Techno
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include <ctype.h>

#include "cli.h"
#include "3rdparty/EmbeddedCLI/embedded_cli.h"
#include "debug_console.h"
#include "printf.h"
#include "version.h"

#include "main.h"

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
    cli_quiet = true;
    return 0;
}

static int CLI_verbose(int argc, char **argv)
{
    cli_quiet = false;
    return 0;
}

static int CLI_version(int argc, char **argv)
{
    PRINTF(M33_FW_NAME " version " M33_FW_VERSION "\r\n");
    return 0;
}

static int CLI_clear(int argc, char **argv)
{
    DebugConsole_Clear();
    return 0;
}

static int CLI_sleepMode(int argc, char **argv)
{
    if (argc == 1)
    {
        switch (sleepWithLinux)
        {
            case LPM_PowerModeDeepSleep:
                printf("deepsleep\r\n");
                break;
            case LPM_PowerModeSleep:
                printf("sleep\r\n");
                break;
            case LPM_PowerModePowerDown:
                printf("powerdown\r\n");
                break;
            case LPM_PowerModeActive:
                printf("active\r\n");
                break;
            case LPM_PowerModeIgnore:
                printf("ignore\r\n");
                break;
            default:
                printf("unknown %d\r\n", sleepWithLinux);
        }
        return 0;
    }
    if (argc != 2)
        goto usage;

    /* deepsleep = default */
    if (!strcmp(argv[1], "deepsleep"))
        sleepWithLinux = LPM_PowerModeDeepSleep;
    /* sleep/powerdown are different "m33 suspend" states that can behave differently */
    else if (!strcmp(argv[1], "sleep"))
        sleepWithLinux = LPM_PowerModeSleep;
    else if (!strcmp(argv[1], "powerdown"))
        sleepWithLinux = LPM_PowerModePowerDown;
    /* active suspends linux drivers and CLI, but m33 stays alive */
    else if (!strcmp(argv[1], "active"))
        sleepWithLinux = LPM_PowerModeActive;
    /* ignore does not suspend anything (even watchdog!) */
    else if (!strcmp(argv[1], "ignore"))
        sleepWithLinux = LPM_PowerModeIgnore;
    else
        goto usage;

    return 0;

usage:
    if (argc == 2)
        PRINTF("Invalid mode %s\r\n", argv[1]);
    return CLI_help_usage("sleep_mode", 1);
}

static int CLI_wakeup(int argc, char **argv)
{
    APP_SRTM_WakeupCA35();
    return 0;
}

static int CLI_wakeupTimer(int argc, char **argv)
{
    unsigned long time;
    char *end;

    if (argc == 1)
    {
        printf("%" PRIu32 "\r\n", s_wakeupTimeoutMs);
        return 0;
    }

    if (argc != 2)
        return CLI_help_usage("wakeup_timer", 1);

    time = strtoul(argv[1], &end, 10);
    if (!end || end[0] != 0)
    {
        PRINTF("Invalid duration %s\r\n", argv[1]);
        return CLI_help_usage("wakeup_timer", 1);
    }
    /* LPTMR CMR register fits 1-0xFFFF multiples of 16ms.
     * 16*0xFFFF/1000 = 1048.56 */
    if ((time > 1048 * 1000 || time < 16) && time != 0)
    {
        PRINTF("timer must be between 16ms and 1048s, or 0\r\n");
        return 1;
    }

    s_wakeupTimeoutMs = time;
    return 0;
}

#ifdef CLI_RAW_MEM
static int parse_size_switch(const char *arg)
{
    if (arg[0] != '-' || arg[1] == 0 || arg[2] != 0)
        return -1;
    switch (arg[1])
    {
        case 'b':
            return 1;
        case 'w':
            return 2;
        case 'l':
            return 4;
        case 'q':
            return 8;
    }
    return -1;
}

static int CLI_memdump(int argc, char **argv)
{
    uintptr_t addr;
    size_t i, count = 1;
    int size = 4;
    char *end;

    if (argc < 2)
        return CLI_help_usage("md", 1);

    if (argv[1][0] == '-')
    {
        size = parse_size_switch(argv[1]);
        if (size < 0)
        {
            PRINTF("Invalid switch %s\r\n", argv[1]);
            return 1;
        }
        argv++;
        argc--;

        if (argc < 2)
            return CLI_help_usage("md", 1);
    }
    if (argc > 3)
        return CLI_help_usage("md", 1);

    addr = strtoul(argv[1], &end, 16);
    if (!end || end[0] != 0)
    {
        PRINTF("Invalid addr %s\r\n", argv[1]);
        return CLI_help_usage("md", 1);
    }
    if (argc == 3)
    {
        count = strtoul(argv[2], &end, 16);
        if (!end || end[0] != 0)
        {
            PRINTF("Invalid count %s\r\n", argv[2]);
            return CLI_help_usage("md", 1);
        }
    }

    int line_length = 16 / size;
    char buf[17]    = { 0 };
    for (i = 0; i < count; i++)
    {
        if (i % line_length == 0)
        {
            if (buf[0])
                PRINTF("    %s\r\n", buf);
            PRINTF("%08x: ", addr);
            memset(buf, 0, sizeof(buf));
        }

        uint64_t val;
        switch (size)
        {
            case 1:
                val = *(uint8_t *)addr;
                break;
            case 2:
                val = *(uint16_t *)addr;
                break;
            case 4:
                val = *(uint32_t *)addr;
                break;
            case 8:
                val = *(uint64_t *)addr;
                break;
            default:
                __builtin_unreachable();
        }
        addr += size;

        PRINTF("%0*llx ", size * 2, val);
        for (int j = 0; j < size; j++)
        {
            char c = (val >> (j * 8)) & 0xff;
            if (!isprint(c))
                c = '.';
            sprintf(buf + (i % line_length) * size + j, "%c", c);
        }
    }
    /* fill line with spaces if not round */
    for (; i % line_length != 0; i++)
        PRINTF("%*s ", size * 2, "");
    PRINTF("    %s\r\n", buf);

    return 0;
}

static int CLI_memwrite(int argc, char **argv)
{
    uintptr_t addr;
    uint64_t value;
    int size = 4;
    char *end;

    if (argc < 2)
        return CLI_help_usage("mw", 1);

    if (argv[1][0] == '-')
    {
        size = parse_size_switch(argv[1]);
        if (size < 0)
        {
            PRINTF("Invalid switch %s\r\n", argv[1]);
            return 1;
        }
        argv++;
        argc--;

        if (argc < 2)
            return CLI_help_usage("mw", 1);
    }
    if (argc > 3)
        return CLI_help_usage("mw", 1);

    addr = strtoul(argv[1], &end, 16);
    if (!end || end[0] != 0)
    {
        PRINTF("Invalid addr %s\r\n", argv[1]);
        return CLI_help_usage("mw", 1);
    }
    value = strtoul(argv[2], &end, 16);
    if (!end || end[0] != 0)
    {
        PRINTF("Invalid value %s\r\n", argv[2]);
        return CLI_help_usage("mw", 1);
    }
    if (size < 8 && value >= (1ULL << (size * 8)))
    {
        PRINTF("%llx too big\r\n", value);
        return 1;
    }

    switch (size)
    {
        case 1:
            *(uint8_t *)addr = value;
            break;
        case 2:
            *(uint16_t *)addr = value;
            break;
        case 4:
            *(uint32_t *)addr = value;
            break;
        case 8:
            *(uint64_t *)addr = value;
            break;
        default:
            __builtin_unreachable();
    }

    return 0;
}
#endif

static int CLI_help(int argc, char **argv);
static const struct CLI_command CLI_commands[] = {
    { "?", CLI_help },
    { "help", CLI_help, "this help" },
    { "reset", CLI_reset, "cold reset" },
    { "log", CLI_log, "print buffer log" },
    { "clear", CLI_clear, "clear buffer log" },
    { "quiet", CLI_quiet, "disable background messages" },
    { "verbose", CLI_verbose, "enable background messages" },
    { "version", CLI_version, "print firmware version" },
    { "sleep_mode", CLI_sleepMode, NULL /* debug function, not listed */,
      "sleep_mode [deepsleep|sleep|powerdown|active|ignore]" },
    { "wakeup", CLI_wakeup }, /* wake up linux if sleeping (only used for testing) */
    { "wakeup_timer", CLI_wakeupTimer, NULL /* debug function, not listed */,
      "wakeup_timer [time_in_ms] (0 disables)" },
#ifdef CLI_RAW_MEM
    { "md", CLI_memdump, "memory dump", "md -[bwlq] addr [count]" },
    { "mw", CLI_memwrite, "memory write", "mw -[bwlq] addr value" },
#endif
    {
        0,
    }, /* sentinel */
};

#define foreach_command(command)                                                                       \
    for (const struct CLI_command *commands = CLI_Custom_commands; commands;                           \
         commands                           = (commands == CLI_Custom_commands) ? CLI_commands : NULL) \
        for (command = commands; command->command; command++)

int CLI_help_usage(const char *cmd, int retval)
{
    const struct CLI_command *command;

    foreach_command(command)
    {
        if (strcmp(cmd, command->command))
            continue;

        if (!command->help)
            PRINTF("%s\r\n", cmd);
        else
            PRINTF("%s - %s\r\n", cmd, command->help);

        if (command->usage)
            PRINTF("\r\nUsage:\r\n%s\r\n", command->usage);

        return retval;
    }

    return 1;
}

static int CLI_help(int argc, char **argv)
{
    int maxlength = 0;
    const struct CLI_command *command;

    if (argc > 1)
    {
        int rc = 0;

        for (int i = 1; i < argc; i++)
        {
            rc += CLI_help_usage(argv[i], 0);
        }
        return rc;
    }
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
