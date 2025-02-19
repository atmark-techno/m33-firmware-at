/*
 * Copyright 2021-2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _POWER_MODE_SWITCH_H_
#define _POWER_MODE_SWITCH_H_

#include "app_srtm.h"
#include "lpm.h"

/****************
 * debug toggles
 ****************/

/* some basic function tracing when entering sleep */
// #define DEBUG_SUSPEND 1

/* do not reset jtag pinmux on suspend */
// #define DEBUG_SUSPEND_SKIP_JTAG_PINS 1

/* Also see SRTM_DEBUG_VERBOSE_LEVEL in srtm_config.h and each srtm/services .c file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifndef GEN_CASE_ENUM_NAME
#define GEN_CASE_ENUM_NAME(e) \
    case (e):                 \
        return (char *)#e
#endif

#define MAIN_TASK_PRIORITY 1
#define SUSPEND_TASK_PRIORITY 2

/* Power mode definition used in application. */
typedef enum _app_power_mode
{
    kAPP_PowerModeActive = 'A',  /* Active mode */
    kAPP_PowerModeWait,          /* WAIT mode. */
    kAPP_PowerModeStop,          /* STOP mode. */
    kAPP_PowerModeSleep,         /* Sleep mode. */
    kAPP_PowerModeDeepSleep,     /* Deep Sleep mode. */
    kAPP_PowerModePowerDown,     /* Power Down mode. */
    kAPP_PowerModeDeepPowerDown, /* Deep Power Down mode */
} app_power_mode_e;

/* wakeup source for m core */
typedef enum _app_wakeup_source
{
    kAPP_WakeupSourceLptmr, /*!< Wakeup by LPTMR.        */
    kAPP_WakeupSourcePin    /*!< Wakeup by external pin. */
} app_wakeup_source_t;

typedef enum
{
    MODE_COMBI_NO  = 0,
    MODE_COMBI_YES = 1,
} allow_combi_e;

typedef struct
{
    lpm_rtd_power_mode_e rtd_mode;
    lpm_ad_power_mode_e ad_mode;
    allow_combi_e allow_combi; /* Allow Combination */
} mode_combi_t;

/* WUU module index. Should be a soc define... */
#define WUU_MODULE_LPTMR0 (0U)
#define WUU_MODULE_LPTMR1 (1U)
#define WUU_MODULE_CMP0 (2U)
#define WUU_MODULE_CMP1 (3U)
#define WUU_MODULE_UPOWER (4U)
#define WUU_MODULE_TAMPER (5U)
#define WUU_MODULE_NSRTC (6U)
#define WUU_MODULE_SRTC (7U)

/* WUU wakeup event for internal module lptimer1  */
#define LPTMR1_WUU_WAKEUP_EVENT (kWUU_InternalModuleInterrupt)

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

void PinMuxPrepareSuspend(uint8_t gpioIdx, uint8_t pinIdx);
void PMIC_Reset(void);
void APP_PowerModeSwitch(lpm_rtd_power_mode_e targetPowerMode, app_wakeup_source_t source);

#if defined(__cplusplus)
}
#endif

#endif /* _POWER_MODE_SWITCH_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
