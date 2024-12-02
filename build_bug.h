/* SPDX-License-Identifier: GPL-2.0 */
// from linux include/linux/compiler_types.h and others
#ifndef _BUILD_BUG_H
#define _BUILD_BUG_H

#if __has_attribute(__error__)
#define __compiletime_error(msg) __attribute__((__error__(msg)))
#else
#define __compiletime_error(msg)
#endif

#define __compiletime_assert(condition, msg, prefix, suffix)       \
    do                                                             \
    {                                                              \
        extern void prefix##suffix(void) __compiletime_error(msg); \
        if (!(condition))                                          \
            prefix##suffix();                                      \
    } while (0)

#define _compiletime_assert(condition, msg, prefix, suffix) __compiletime_assert(condition, msg, prefix, suffix)

#define compiletime_assert(condition, msg) _compiletime_assert(condition, msg, __compiletime_assert_, __COUNTER__)

#define BUILD_BUG_ON(condition) compiletime_assert(!(condition), "BUILD_BUG_ON failed: " #condition)

#endif
