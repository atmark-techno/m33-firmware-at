/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FreeRTOS.h"
#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "fsl_rgpio.h"
#include "fsl_lptmr.h"
#include "fsl_upower.h"
#include "fsl_mu.h"
#include "printf.h"
#include "debug_console.h"
#include "build_bug.h"

#include "pin_mux.h"
#include "board.h"
#include "app_srtm.h"
#include "cli.h"
#include "lpm.h"
#include "main.h"
#include "fsl_rtd_cmc.h"
#include "fsl_sentinel.h"
#include "fsl_rgpio.h"
#include "fsl_wuu.h"
#include "fsl_lpuart.h"
#include "lpuart.h"

#include "fsl_iomuxc.h"
#include "fsl_reset.h"

#include "srtm_message.h"
#include "srtm_message_struct.h"
/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
#define RPMSG_LITE_LINK_ID (RL_PLATFORM_IMX8ULP_M33_A35_USER_LINK_ID)
#define RPMSG_LITE_SHMEM_BASE (VDEV1_VRING_BASE)
/* Communicate with linux/drivers/tty/serial/imx_rpmsg.c */
#define RPMSG_LITE_NS_ANNOUNCE_STRING "rpmsg-tty-channel"
#define RPMSG_LITE_MASTER_IS_LINUX

#define APP_DEBUG_UART_BAUDRATE (115200U)                   /* Debug console baud rate. */
#define APP_DEBUG_UART_DEFAULT_CLKSRC kCLOCK_IpSrcSircAsync /* SCG SIRC clock. */
#ifndef LOCAL_EPT_ADDR
#define LOCAL_EPT_ADDR (30)
#endif

/* LPTMR0 is WUU internal module 0. */
#define WUU_MODULE_SYSTICK WUU_MODULE_LPTMR0
/* Allow systick to be a wakeup source in Power Down mode. */
#define SYSTICK_WUU_WAKEUP_EVENT (kWUU_InternalModuleInterrupt)

#define APP_LPTMR1_IRQ_PRIO (5U)

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
extern void APP_PowerPreSwitchHook(lpm_rtd_power_mode_e targetMode);
extern void APP_PowerPostSwitchHook(lpm_rtd_power_mode_e targetMode, bool result);
extern void APP_SRTM_WakeupCA35(void);
extern void APP_RebootCA35(void);
extern void APP_ShutdownCA35(void);
extern void APP_BootCA35(void);
extern void UPOWER_InitBuck2Buck3Table(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static TaskHandle_t cliTask;
static uint32_t s_wakeupTimeout;           /* Wakeup timeout. (Unit: Second) */
static app_wakeup_source_t s_wakeupSource; /* Wakeup source.                 */
static uint32_t s_wakeupPinFlag;           /* Wakeup pin flag.               */
static bool wakeWithLinux  = true;
static bool sleepWithLinux = true;
static SemaphoreHandle_t s_wakeupSig;
static SemaphoreHandle_t handleSuspendSig;
static lpm_rtd_power_mode_e suspendPowerMode;
static app_wakeup_source_t suspendWakeupSource;
static const char *s_modeNames[] = { "ACTIVE", "WAIT", "STOP", "Sleep", "Deep Sleep", "Power Down", "Deep Power Down" };
extern lpm_ad_power_mode_e AD_CurrentMode;
extern bool support_dsl_for_apd;
extern bool option_v_boot_flag;
extern lpm_rtd_power_mode_e s_curMode;
extern lpm_rtd_power_mode_e s_lastMode;
extern pca9460_buck3ctrl_t buck3_ctrl;
extern pca9460_ldo1_cfg_t ldo1_cfg;
// clang-format off
mode_combi_t mode_combi_array_for_single_boot[] = {
    {LPM_PowerModeActive, AD_ACT, MODE_COMBI_YES},
    {LPM_PowerModeWait, AD_ACT, MODE_COMBI_YES},
    {LPM_PowerModeStop, AD_ACT, MODE_COMBI_YES},
    {LPM_PowerModeSleep, AD_ACT, MODE_COMBI_YES},
    {LPM_PowerModeDeepSleep, AD_ACT, MODE_COMBI_NO},
    {LPM_PowerModePowerDown, AD_ACT, MODE_COMBI_NO},
    {LPM_PowerModeDeepPowerDown, AD_ACT, MODE_COMBI_NO},

    {LPM_PowerModeActive, AD_DSL, MODE_COMBI_YES},
    {LPM_PowerModeWait, AD_DSL, MODE_COMBI_YES},
    {LPM_PowerModeStop, AD_DSL, MODE_COMBI_YES},
    {LPM_PowerModeSleep, AD_DSL, MODE_COMBI_YES},
    {LPM_PowerModeDeepSleep, AD_DSL, MODE_COMBI_YES},
    {LPM_PowerModePowerDown, AD_DSL, MODE_COMBI_NO},
    {LPM_PowerModeDeepPowerDown, AD_DSL, MODE_COMBI_NO},

    {LPM_PowerModeActive, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModeWait, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModeStop, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModeSleep, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModeDeepSleep, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModePowerDown, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModeDeepPowerDown, AD_PD, MODE_COMBI_NO},

    {LPM_PowerModeActive, AD_DPD, MODE_COMBI_YES},
    {LPM_PowerModeWait, AD_DPD, MODE_COMBI_YES},
    {LPM_PowerModeStop, AD_DPD, MODE_COMBI_YES},
    {LPM_PowerModeSleep, AD_DPD, MODE_COMBI_YES},
    {LPM_PowerModeDeepSleep, AD_DPD, MODE_COMBI_YES},
    {LPM_PowerModePowerDown, AD_DPD, MODE_COMBI_YES},
    /* RTD not support Deep Power Down Mode when boot type is SINGLE BOOT TYPE */
    {LPM_PowerModeDeepPowerDown, AD_DPD, MODE_COMBI_NO},
};

mode_combi_t mode_combi_array_for_dual_or_lp_boot[] = {
    {LPM_PowerModeActive, AD_ACT, MODE_COMBI_YES},
    {LPM_PowerModeWait, AD_ACT, MODE_COMBI_YES},
    {LPM_PowerModeStop, AD_ACT, MODE_COMBI_YES},
    {LPM_PowerModeSleep, AD_ACT, MODE_COMBI_YES},
    {LPM_PowerModeDeepSleep, AD_ACT, MODE_COMBI_NO},
    {LPM_PowerModePowerDown, AD_ACT, MODE_COMBI_NO},
    {LPM_PowerModeDeepPowerDown, AD_ACT, MODE_COMBI_NO},

    {LPM_PowerModeActive, AD_DSL, MODE_COMBI_YES},
    {LPM_PowerModeWait, AD_DSL, MODE_COMBI_YES},
    {LPM_PowerModeStop, AD_DSL, MODE_COMBI_YES},
    {LPM_PowerModeSleep, AD_DSL, MODE_COMBI_YES},
    {LPM_PowerModeDeepSleep, AD_DSL, MODE_COMBI_YES},
    {LPM_PowerModePowerDown, AD_DSL, MODE_COMBI_NO},
    {LPM_PowerModeDeepPowerDown, AD_DSL, MODE_COMBI_NO},

    {LPM_PowerModeActive, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModeWait, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModeStop, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModeSleep, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModeDeepSleep, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModePowerDown, AD_PD, MODE_COMBI_YES},
    {LPM_PowerModeDeepPowerDown, AD_PD, MODE_COMBI_NO},

    {LPM_PowerModeActive, AD_DPD, MODE_COMBI_YES},
    {LPM_PowerModeWait, AD_DPD, MODE_COMBI_YES},
    {LPM_PowerModeStop, AD_DPD, MODE_COMBI_YES},
    {LPM_PowerModeSleep, AD_DPD, MODE_COMBI_YES},
    {LPM_PowerModeDeepSleep, AD_DPD, MODE_COMBI_YES},
    {LPM_PowerModePowerDown, AD_DPD, MODE_COMBI_YES},
    {LPM_PowerModeDeepPowerDown, AD_DPD, MODE_COMBI_YES},
};
// clang-format on

char hardfault_buf[32];
void HardFault_Handler(void)
{
    /* volatile apparently helps compiler not optimize these variables away,
     * making access easier in debugger */
    volatile uint32_t HFSR;
    volatile uint32_t CFSR;
    volatile uint32_t MMFAR;
    volatile uint32_t BFAR;

    /* These do not seem to have any define provided...
     * https://developer.arm.com/documentation/100235/0004/the-cortex-m33-peripherals/system-control-block/system-control-block-registers-summary?lang=en
     * https://interrupt.memfault.com/blog/cortex-m-hardfault-debug
     */
    HFSR = *(uint32_t *)(0xE000ED2C);
#define HFSR_FORCED (1 << 30)
    CFSR = *(uint32_t *)(0xE000ED28);
#define CFSR_MMFSR_MMARVALID (1 << 7)
#define CFSR_BFSR_BFARVALID (1 << (8 + 7))
#define CFSR_UFSR_DIVBYZERO (1 << (16 + 9))
#define CFSR_UFSR_UNALIGNED (1 << (16 + 8))
#define CFSR_UFSR_STKOF (1 << (16 + 4))
#define CFSR_UFSR_INVPC (1 << (16 + 2))
#define CFSR_UFSR_INVSTATE (1 << (16 + 1))
#define CFSR_UFSR_UNDEFINSTR (1 << (16 + 0))
    MMFAR = *(uint32_t *)(0xE000ED34);
    BFAR  = *(uint32_t *)(0xE000ED38);

    /* If jtag is attached, give them a chance to look at it first */
    if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)
        __asm("bkpt 1");

    DebugConsole_Emergency("\r\n\r\n====================\r\n\r\nHARDFAULT!\r\n");
    if (HFSR & HFSR_FORCED)
    {
        if (CFSR & CFSR_MMFSR_MMARVALID)
        {
            sprintf(hardfault_buf, "MMFAR %x\r\n", MMFAR);
            DebugConsole_Emergency(hardfault_buf);
        }
        if (CFSR & CFSR_BFSR_BFARVALID)
        {
            sprintf(hardfault_buf, "BFAR %x\r\n", BFAR);
            DebugConsole_Emergency(hardfault_buf);
        }
        /* XXX try to print which task caused that? */
        if (CFSR & CFSR_UFSR_DIVBYZERO)
            DebugConsole_Emergency("Divide by zero\r\n");
        if (CFSR & CFSR_UFSR_UNALIGNED)
            DebugConsole_Emergency("Unaligned access\r\n");
        if (CFSR & CFSR_UFSR_STKOF)
            DebugConsole_Emergency("Stack overflow\r\n");
        if (CFSR & CFSR_UFSR_INVPC)
            DebugConsole_Emergency("Invalid PC\r\n");
        if (CFSR & CFSR_UFSR_INVSTATE)
            DebugConsole_Emergency("Invalid state flag\r\n");
        if (CFSR & CFSR_UFSR_UNDEFINSTR)
            DebugConsole_Emergency("Undefined instruction\r\n");
    }
    DebugConsole_Emergency("====================\r\n\r\n");

    /* we don't know if it's safe to print something in console here, just reset in doubt..
     * Ideally try to store a flag in something that survives reset (rtc?) and use that? */
    PMIC_Reset();
}

/*******************************************************************************
 * Function Code
 ******************************************************************************/
extern lpm_ad_power_mode_e AD_CurrentMode;
extern pca9460_buck3ctrl_t buck3_ctrl;
extern pca9460_ldo1_cfg_t ldo1_cfg;
static uint32_t iomuxBackup[25 + 16 + 24]; /* Backup 25 PTA, 16 PTB and 24 PTC IOMUX registers */
static uint32_t gpioICRBackup[25 + 16 + 24];

typedef struct
{
    uint32_t pdor;
    uint32_t pddr;
} gpioOutputConfig_t;
static gpioOutputConfig_t gpioOutputBackup[3]; /* Backup PTA, PTB and PTC Output-related GPIO registers */

static uint32_t WuuPinToBackupIndex(uint8_t wuuPin)
{
    uint16_t ioId   = APP_WUUPin_TO_IoId(wuuPin);
    uint8_t gpioIdx = APP_GPIO_IDX(ioId);
    uint8_t pinIdx  = APP_PIN_IDX(ioId);
    uint32_t backupIndex;

    assert(gpioIdx < 2); /* Only support GPIOA and GPIOB */
    assert(pinIdx < APP_IO_PINS_PER_CHIP);

    switch (gpioIdx)
    {
        case 0:
            backupIndex = 0;
            break;
        case 1:
            backupIndex = 25;
            break;
        default:
            return UINT32_MAX;
    }

    return (backupIndex + pinIdx);
}

static bool IsWuuPinMuxGPIO(uint8_t wuuPin)
{
    uint32_t backupIndex = WuuPinToBackupIndex(wuuPin);

    assert(backupIndex < ARRAY_SIZE(iomuxBackup));

    /* All PTx function values ​​are 0x01. */
    if ((iomuxBackup[backupIndex] & IOMUXC_PCR_MUX_MODE_MASK) == IOMUXC_PCR_MUX_MODE(0x1))
        return true;

    return false;
}

static bool IsWuuPinGPIOIrqcInterruptEdge(uint8_t wuuPin)
{
    uint32_t backupIndex = WuuPinToBackupIndex(wuuPin);

    assert(backupIndex < ARRAY_SIZE(gpioICRBackup));

    switch (gpioICRBackup[backupIndex] & RGPIO_ICR_IRQC_MASK)
    {
        case RGPIO_ICR_IRQC(kRGPIO_InterruptRisingEdge):
        case RGPIO_ICR_IRQC(kRGPIO_InterruptFallingEdge):
        case RGPIO_ICR_IRQC(kRGPIO_InterruptEitherEdge):
            return true;
    }

    return false;
}

void IoSuspendOne(uint8_t gpioIdx, uint8_t pinIdx, uint32_t backupIndex)
{
    uint8_t wuuIndex = APP_IO_GetWUUPin(gpioIdx, pinIdx);
    __IO uint32_t *IOMUXC;
    RGPIO_Type *GPIO;

    switch (gpioIdx)
    {
        case 0:
            IOMUXC = IOMUXC0->PCR0_IOMUXCARRAY0 + pinIdx;
            GPIO   = GPIOA;
            break;
        case 1:
            IOMUXC = IOMUXC0->PCR0_IOMUXCARRAY1 + pinIdx;
            GPIO   = GPIOB;
            break;
        case 2:
            IOMUXC = IOMUXC0->PCR0_IOMUXCARRAY2 + pinIdx;
            GPIO   = GPIOC;
            break;
        default:
            abort_msg("IoSuspendOne called with invalid index?");
    }
    iomuxBackup[backupIndex]   = *IOMUXC;
    gpioICRBackup[backupIndex] = GPIO->ICR[pinIdx];
    GPIO->ICR[pinIdx]          = 0; /* disable interrupts */

    /* regroup skipped pins logic here */
    /* JTAG pins, for easier debugging -- PTA[20-23] skipped if DEBUG_SUSPEND_SKIP_JTAG_PINS set */
#if DEBUG_SUSPEND_SKIP_JTAG_PINS
    if (gpioIdx == 0 && (pinIdx >= 20 && pinIdx <= 23))
        return;
#endif
    /* PTB[10-11] for upower i2c */
    if (gpioIdx == 1 && (pinIdx == 10 || pinIdx == 11))
        return;

    /* preserve output gpios.
     * iomux mode 0x1 is gpio, PDDR is direction output */
    if ((*IOMUXC & IOMUXC_PCR_MUX_MODE(0x1)) && (GPIO->PDDR & 1 << pinIdx))
        return;

    if (wuuIndex == 255)
    {
        *IOMUXC = 0;
        return;
    }

    // WUU_PEx_WUPEx_SHIFT is exactly WUU (index * 2)
    uint32_t mask     = 3 << (wuuIndex * 2 % 32);
    __IO uint32_t *PE = wuuIndex < 16 ? &WUU0->PE1 : &WUU0->PE2;
    uint32_t setting  = *PE & mask;
    if (!setting)
    {
        *IOMUXC = 0;
        return;
    }

    /*
     * Disable interrupt temperarily to prevent glitch
     * interrupt during switching IOMUXC pin selection
     */
    *PE &= ~mask;

    /* select WUU pin. They're all 0xD! */
    *IOMUXC = IOMUXC_PCR_MUX_MODE(0xD);

    *PE |= setting;
}

static void IoSuspend(void)
{
    uint32_t i;
    uint32_t backupIndex;

    backupIndex = 0;

    /* Backup PTA IOMUXC and GPIOA ICR registers then disable */
    for (i = 0; i <= 24; i++)
    {
        IoSuspendOne(0, i, backupIndex);

        backupIndex++;
    }
    gpioOutputBackup[0].pdor = GPIOA->PDOR;
    gpioOutputBackup[0].pddr = GPIOA->PDDR;

    /* Backup PTB IOMUXC and GPIOB ICR registers then disable */
    for (i = 0; i <= 15; i++)
    {
        IoSuspendOne(1, i, backupIndex);

        backupIndex++;
    }
    gpioOutputBackup[1].pdor = GPIOB->PDOR;
    gpioOutputBackup[1].pddr = GPIOB->PDDR;

    /* Backup PTC IOMUXC and GPIOC ICR registers then disable */
    for (i = 0; i <= 23; i++)
    {
        IoSuspendOne(2, i, backupIndex);

        backupIndex++;
    }
    gpioOutputBackup[2].pdor = GPIOC->PDOR;
    gpioOutputBackup[2].pddr = GPIOC->PDDR;

    /* Clear any potential interrupts before enter Power Down */
    WUU0->PF = WUU0->PF;
}

static void IoResume(void)
{
    uint32_t i;
    uint32_t backupIndex;

    backupIndex = 0;

    /* Restore PTA IOMUXC and GPIOA ICR registers */
    GPIOA->PDOR = gpioOutputBackup[0].pdor;
    GPIOA->PDDR = gpioOutputBackup[0].pddr;
    for (i = 0; i <= 24; i++)
    {
        IOMUXC0->PCR0_IOMUXCARRAY0[i] = iomuxBackup[backupIndex];
        GPIOA->ICR[i]                 = gpioICRBackup[backupIndex];
        backupIndex++;
    }

    /* Restore PTB IOMUXC and GPIOB ICR registers */
    GPIOB->PDOR = gpioOutputBackup[1].pdor;
    GPIOB->PDDR = gpioOutputBackup[1].pddr;
    for (i = 0; i <= 15; i++)
    {
        IOMUXC0->PCR0_IOMUXCARRAY1[i] = iomuxBackup[backupIndex];
        GPIOB->ICR[i]                 = gpioICRBackup[backupIndex];
        backupIndex++;
    }

    /* Restore PTC IOMUXC and GPIOC ICR registers */
    GPIOC->PDOR = gpioOutputBackup[2].pdor;
    GPIOC->PDDR = gpioOutputBackup[2].pddr;
    for (i = 0; i <= 23; i++)
    {
        IOMUXC0->PCR0_IOMUXCARRAY2[i] = iomuxBackup[backupIndex];
        GPIOC->ICR[i]                 = gpioICRBackup[backupIndex];
        backupIndex++;
    }
}

void APP_PowerPreSwitchHook(lpm_rtd_power_mode_e targetMode)
{
#ifdef DEBUG_SUSPEND
    PRINTF("%s\r\n", __func__);
#endif
    if ((LPM_PowerModeActive != targetMode))
    {
        PRINTF("Preparing for %s\r\n", s_modeNames[targetMode]);

        DebugConsole_Suspend();
    }
}

void APP_PowerPostSwitchHook(lpm_rtd_power_mode_e targetMode, bool result)
{
#ifdef DEBUG_SUSPEND
    PRINTF("%s\r\n", __func__);
#endif
    if (LPM_PowerModeActive != targetMode)
    {
        /* init clock first as apps use it indirectly */
        BOARD_InitClock();

        DebugConsole_Resume();
    }
    PRINTF("== Power switch %s ==\r\n", result ? "OK" : "FAIL");
    /* Reinitialize TRDC */
    if (AD_CurrentMode == AD_PD)
    {
        BOARD_SetTrdcGlobalConfig();
    }
    else if (AD_CurrentMode == AD_DPD)
    {
        /*
         * For oem_open lifecycle and closed lifecycle part,
         * there are different default setting,
         * Sentinel will apply the default settings after sentinel exit from power down mode.
         * If the sillicon is in oem_open lifecycle, sentinel apply default settings A.
         * If the sillicon is in closed lifecycle, sentinel apply default settings B.
         * A and B is different.
         * So whatever sillicon is in which lifecycle,
         * change default settings directly.
         */
        BOARD_SetTrdcGlobalConfig();
        BOARD_SetTrdcAfterApdReset();
    }
}
static inline const char *APP_GetAllowCombiName(allow_combi_e allow)
{
    switch (allow)
    {
        GEN_CASE_ENUM_NAME(MODE_COMBI_NO);
        GEN_CASE_ENUM_NAME(MODE_COMBI_YES);
        default:
            return (char *)"WRONG_MODE_COMBI";
    }
}
static inline const char *APP_GetRtdPwrModeName(lpm_rtd_power_mode_e mode)
{
    switch (mode)
    {
        GEN_CASE_ENUM_NAME(LPM_PowerModeActive);
        GEN_CASE_ENUM_NAME(LPM_PowerModeWait);
        GEN_CASE_ENUM_NAME(LPM_PowerModeStop);
        GEN_CASE_ENUM_NAME(LPM_PowerModeSleep);
        GEN_CASE_ENUM_NAME(LPM_PowerModeDeepSleep);
        GEN_CASE_ENUM_NAME(LPM_PowerModePowerDown);
        GEN_CASE_ENUM_NAME(LPM_PowerModeDeepPowerDown);
        default:
            return (char *)"WRONG_LPM_RTD_PowerMode";
    }
}

static inline const char *APP_GetAdPwrModeName(lpm_ad_power_mode_e mode)
{
    switch (mode)
    {
        GEN_CASE_ENUM_NAME(AD_UNKOWN);
        GEN_CASE_ENUM_NAME(AD_ACT);
        GEN_CASE_ENUM_NAME(AD_DSL);
        GEN_CASE_ENUM_NAME(AD_PD);
        GEN_CASE_ENUM_NAME(AD_DPD);
        default:
            return (char *)"WRONG_LPM_AD_PowerMode";
    }
}

static allow_combi_e APP_GetModeAllowCombi(lpm_ad_power_mode_e ad_mode, lpm_rtd_power_mode_e rtd_mode)
{
    int i               = 0;
    allow_combi_e allow = MODE_COMBI_NO;
    ;

    if (BOARD_IsSingleBootType())
    {
        for (i = 0; i < ARRAY_SIZE(mode_combi_array_for_single_boot); i++)
        {
            if ((mode_combi_array_for_single_boot[i].rtd_mode == rtd_mode) &&
                (mode_combi_array_for_single_boot[i].ad_mode == ad_mode))
            {
                allow = mode_combi_array_for_single_boot[i].allow_combi;
                break;
            }
        }
    }
    else
    {
        for (i = 0; i < ARRAY_SIZE(mode_combi_array_for_dual_or_lp_boot); i++)
        {
            if ((mode_combi_array_for_dual_or_lp_boot[i].rtd_mode == rtd_mode) &&
                (mode_combi_array_for_dual_or_lp_boot[i].ad_mode == ad_mode))
            {
                allow = mode_combi_array_for_dual_or_lp_boot[i].allow_combi;
                break;
            }
        }
    }

    return allow;
}

/* WUU0 interrupt handler. */
void WUU0_IRQHandler(void)
{
    /* "really" wake up for LPTMR1 alarm or GPIO.
     * If only LPTRM0 (SYSTICK) then we'll sleep again */
    bool wakeup = false;
    uint32_t pinsFlag;

    if (WUU_GetInternalWakeupModuleFlag(WUU0, WUU_MODULE_LPTMR1))
    {
        /* Woken up by LPTMR, then clear LPTMR flag. */
        LPTMR_ClearStatusFlags(LPTMR1, kLPTMR_TimerCompareFlag);
        wakeup = true;
    }
    if (LPTMR_GetEnabledInterrupts(LPTMR1))
    {
        /* disable anyway if enabled, or would fire late.*/
        LPTMR_DisableInterrupts(LPTMR1, kLPTMR_TimerInterruptEnable);
        LPTMR_StopTimer(LPTMR1);
    }

    if (WUU_GetInternalWakeupModuleFlag(WUU0, WUU_MODULE_SYSTICK))
    {
        /* Woken up by Systick LPTMR0, then clear LPTMR flag. */
        LPTMR_ClearStatusFlags(SYSTICK_BASE, kLPTMR_TimerCompareFlag);
    }

    pinsFlag = WUU_GetExternalWakeUpPinsFlag(WUU0);
    s_wakeupPinFlag |= pinsFlag;
    if (pinsFlag)
    {
        /* clear pin interrupt flag */
        WUU_ClearExternalWakeUpPinsFlag(WUU0, pinsFlag);
        wakeup = true;
    }

    if (!wakeup)
        return;

    xSemaphoreGiveFromISR(s_wakeupSig, NULL);
    portYIELD_FROM_ISR(pdTRUE);
}

/* LPTMR1 interrupt handler. */
void LPTMR1_IRQHandler(void)
{
    bool wakeup = false;

    if (kLPTMR_TimerInterruptEnable & LPTMR_GetEnabledInterrupts(LPTMR1))
    {
        LPTMR_ClearStatusFlags(LPTMR1, kLPTMR_TimerCompareFlag);
        LPTMR_DisableInterrupts(LPTMR1, kLPTMR_TimerInterruptEnable);
        LPTMR_StopTimer(LPTMR1);
        wakeup = true;
    }

    if (wakeup)
    {
        xSemaphoreGiveFromISR(s_wakeupSig, NULL);
        portYIELD_FROM_ISR(pdTRUE);
    }
}

/* Get input from user about wakeup timeout. */
static uint32_t APP_GetWakeupTimeout(void)
{
    uint32_t timeout = 0U;
    uint8_t c;

    while (1)
    {
        PRINTF("Select the wake up timeout in seconds.\r\n");
        PRINTF("The allowed range is 1s ~ 999s.\r\n");
        PRINTF("Eg. enter 5 to wake up in 5 seconds.\r\n");
        PRINTF("\r\nWaiting for input timeout value...\r\n\r\n");

        do
        {
            c = getchar();
            if ((c >= '0') && (c <= '9'))
            {
                PRINTF("%c", c);
                timeout = timeout * 10U + c - '0';
            }
            else if ((c == '\r') || (c == '\n'))
            {
                break;
            }
            else
            {
                PRINTF("%c\r\nWrong value!\r\n", c);
                timeout = 0U;
            }
        } while (timeout != 0U && timeout < 100U);

        if (timeout > 0U)
        {
            PRINTF("\r\n");
            break;
        }
    }

    return timeout;
}

/* Get wakeup timeout and wakeup source. */
static void APP_GetWakeupConfig(app_wakeup_source_t source)
{
    /* Get wakeup source by user input. */
    s_wakeupSource = source;

    if (kAPP_WakeupSourceLptmr == s_wakeupSource)
    {
        /* Wakeup source is LPTMR, user should input wakeup timeout value. */
        s_wakeupTimeout = APP_GetWakeupTimeout();
        PRINTF("Will wakeup in %d seconds.\r\n", s_wakeupTimeout);
    }
}

static void APP_SetWakeupConfig(lpm_rtd_power_mode_e targetMode)
{
    if (kAPP_WakeupSourceLptmr == s_wakeupSource)
    {
        LPTMR_SetTimerPeriod(LPTMR1, (1000UL * s_wakeupTimeout / 16U));
        LPTMR_StartTimer(LPTMR1);
        LPTMR_EnableInterrupts(LPTMR1, kLPTMR_TimerInterruptEnable);
    }

    /* To avoid conflicting access of WUU with SRTM dispatcher, we put the WUU setting into SRTM dispatcher context.*/
    /* If targetMode is PD/DPD, setup WUU. */
    if ((LPM_PowerModePowerDown == targetMode) || (LPM_PowerModeDeepPowerDown == targetMode))
    {
        if (kAPP_WakeupSourceLptmr == s_wakeupSource)
        {
            /* Set WUU LPTMR1 module wakeup source. */
            WUU_SetInternalWakeUpModulesConfig(WUU0, WUU_MODULE_LPTMR1, LPTMR1_WUU_WAKEUP_EVENT);
            PCC1->PCC_LPTMR1 &= ~PCC1_PCC_LPTMR1_SSADO_MASK;
            PCC1->PCC_LPTMR1 |= PCC1_PCC_LPTMR1_SSADO(1);
        }
    }

    EnableIRQ(WUU0_IRQn);
}

static void APP_ClearWakeupConfig(lpm_rtd_power_mode_e targetMode)
{
    if ((LPM_PowerModePowerDown == targetMode) || (LPM_PowerModeDeepPowerDown == targetMode))
    {
        WUU_ClearInternalWakeUpModulesConfig(WUU0, WUU_MODULE_LPTMR1, LPTMR1_WUU_WAKEUP_EVENT);
    }
    DisableIRQ(WUU0_IRQn);
}

static void APP_CreateTask(void) {}

static void APP_Suspend(void)
{
    APP_SRTM_Suspend();
    vTaskSuspend(cliTask);
    IoSuspend();
}

static void APP_Resume(void)
{
    IoResume();

    APP_SRTM_Resume();
    vTaskResume(cliTask);
}

static void APP_DestroyTask(void) {}

static void APP_RpmsgMonitor(struct rpmsg_lite_instance *rpmsgHandle, bool ready, void *rpmsgMonitorParam)
{
    if (ready)
    {
        APP_CreateTask();
    }
    else
    {
        APP_DestroyTask();
    }
}

static void HandleSuspendTask(void *pvParameters)
{
    lpm_rtd_power_mode_e targetPowerMode;
    app_wakeup_source_t source;

    while (true)
    {
        xSemaphoreTake(handleSuspendSig, portMAX_DELAY);
        targetPowerMode = suspendPowerMode;
        source          = suspendWakeupSource;
        if (source == kAPP_WakeupSourcePin && !sleepWithLinux)
        {
            PRINTF("Not sleeping with linux\r\n");
            continue;
        }
        PRINTF("HandleSuspendTask: target %d / %d\r\n", targetPowerMode, source);
        if (targetPowerMode == s_curMode)
        {
            /* Same mode, skip it */
            continue;
        }
        if (APP_GetModeAllowCombi(AD_CurrentMode, targetPowerMode) == MODE_COMBI_NO)
        {
            PRINTF("Not support the mode combination: %s + %s\r\n", APP_GetAdPwrModeName(AD_CurrentMode),
                   APP_GetRtdPwrModeName(targetPowerMode));
            continue;
        }
        if (!LPM_SetPowerMode(targetPowerMode))
        {
            PRINTF("Some task doesn't allow to enter mode %s\r\n", s_modeNames[targetPowerMode]);
        }
        else /* Idle task will handle the low power state. */
        {
            /* clear any previous wakeup we might have had */
            xSemaphoreTake(s_wakeupSig, 0);

            APP_Suspend();
            APP_GetWakeupConfig(source);
            APP_SetWakeupConfig(targetPowerMode);

            PRINTF("Suspended tasks...\r\n");

            // xSemaphoreTake(s_wakeupSig, portMAX_DELAY);
            while (!xSemaphoreTake(s_wakeupSig, 5000))
            {
                PRINTF("not sleeping?\r\n");
            }

            PRINTF("Waking up...\r\n");
            /* The call might be blocked by SRTM dispatcher task. Must be called after power mode reset. */
            APP_ClearWakeupConfig(targetPowerMode);
            APP_Resume();
        }

        /*update Mode state*/
        s_lastMode = s_curMode;
        s_curMode  = LPM_PowerModeActive;

        if (s_wakeupPinFlag)
        {
            uint8_t pins = (WUU0->PARAM & WUU_PARAM_PINS_MASK) >> WUU_PARAM_PINS_SHIFT;
            bool wakeup  = false;
            int i;

            for (i = 0; i < pins; i++)
            {
                if (s_wakeupPinFlag & BIT(i))
                {
                    /* The interrupt received by wuu is reported as an rgpio
                     * interrupt. This is because rgpio cannot receive edge
                     * interrupts while suspended. */
                    if (IsWuuPinMuxGPIO(i) && IsWuuPinGPIOIrqcInterruptEdge(i))
                    {
                        APP_SRTM_EmulateGPIOHandler(i);
                    }

                    wakeup = true;
                }
            }

            if ((AD_CurrentMode == AD_PD || (support_dsl_for_apd == true && AD_CurrentMode == AD_DSL)) &&
                wakeWithLinux && wakeup)
            {
                /* Wakeup A Core(CA35) when A Core is in Power Down Mode */
                PRINTF("Wake up from WUU_Pn 0x%x\r\n", s_wakeupPinFlag);
                APP_SRTM_WakeupCA35();
            }

            s_wakeupPinFlag = 0;
        }
    }
}

void APP_PowerModeSwitch(lpm_rtd_power_mode_e targetPowerMode, app_wakeup_source_t source)
{
    suspendPowerMode    = targetPowerMode;
    suspendWakeupSource = source;
    xSemaphoreGive(handleSuspendSig);
}

/* Power Mode Switch task */
static void PowerModeSwitchInit()
{
    lptmr_config_t lptmrConfig;

    /* Add Systick as Power Down wakeup source, depending on SYSTICK_WUU_WAKEUP_EVENT value. */
    WUU_SetInternalWakeUpModulesConfig(WUU0, WUU_MODULE_SYSTICK, SYSTICK_WUU_WAKEUP_EVENT);
    NVIC_SetPriority(WUU0_IRQn, APP_WUU_IRQ_PRIO);

    /* Setup LPTMR. */
    LPTMR_GetDefaultConfig(&lptmrConfig);
    lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_1; /* Use RTC 1KHz as clock source. */
    lptmrConfig.bypassPrescaler      = false;
    lptmrConfig.value                = kLPTMR_Prescale_Glitch_3; /* Divide clock source by 16. */
    LPTMR_Init(LPTMR1, &lptmrConfig);
    NVIC_SetPriority(LPTMR1_IRQn, APP_LPTMR1_IRQ_PRIO);

    EnableIRQ(LPTMR1_IRQn);

    SIM_SEC->DGO_GP10  = 2;
    SIM_SEC->DGO_CTRL1 = SIM_SEC_DGO_CTRL1_UPDATE_DGO_GP10_MASK;
    /* Wait DGO GP0 updated */
    while ((SIM_SEC->DGO_CTRL1 & SIM_SEC_DGO_CTRL1_WR_ACK_DGO_GP10_MASK) == 0)
    {
    }
    /* Clear DGO GP0 ACK and UPDATE bits */
    SIM_SEC->DGO_CTRL1 =
        (SIM_SEC->DGO_CTRL1 & ~(SIM_SEC_DGO_CTRL1_UPDATE_DGO_GP10_MASK)) | SIM_SEC_DGO_CTRL1_WR_ACK_DGO_GP10_MASK;

    SIM_SEC->DGO_GP11  = 1; // PTB range to 1.8V
    SIM_SEC->DGO_CTRL1 = SIM_SEC_DGO_CTRL1_UPDATE_DGO_GP11_MASK;
    /* Wait DGO GP0 updated */
    while ((SIM_SEC->DGO_CTRL1 & SIM_SEC_DGO_CTRL1_WR_ACK_DGO_GP11_MASK) == 0)
    {
    }
    /* Clear DGO GP0 ACK and UPDATE bits */
    SIM_SEC->DGO_CTRL1 =
        (SIM_SEC->DGO_CTRL1 & ~(SIM_SEC_DGO_CTRL1_UPDATE_DGO_GP11_MASK)) | SIM_SEC_DGO_CTRL1_WR_ACK_DGO_GP11_MASK;

    SIM_RTD->PTC_COMPCELL = 0x0; // PTC compensation off
}

void vApplicationMallocFailedHook(void)
{
    PRINTF("Malloc Failed!!!\r\n");
}

/* called by FreeRTOS */
void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
    uint32_t irqMask;
    lpm_rtd_power_mode_e targetPowerMode;
    /* lpm_rtd_power_mode_e targetMode; */
    upwr_pwm_param_t param;
    bool result;

    /* targetMode = LPM_GetPowerMode(); */

    /* Workround for PD/DPD exit fail if sleep more than 1 second */
    /* if ((LPM_PowerModePowerDown == targetMode) || (LPM_PowerModeDeepPowerDown == targetMode)) */
    {
        param.R              = 0;
        param.B.DPD_ALLOW    = 0;
        param.B.DSL_DIS      = 0;
        param.B.SLP_ALLOW    = 0;
        param.B.DSL_BGAP_OFF = 1;
        param.B.DPD_BGAP_ON  = 0;

        UPOWER_SetPwrMgmtParam(&param);
    }

    irqMask = DisableGlobalIRQ();

    /* Only when no context switch is pending and no task is waiting for the scheduler
     * to be unsuspended then enter low power entry.
     */
    if (eTaskConfirmSleepModeStatus() != eAbortSleep)
    {
        targetPowerMode = LPM_GetPowerMode();
        if (targetPowerMode != LPM_PowerModeActive)
        {
            /* Only wait when target power mode is not running */
            APP_PowerPreSwitchHook(targetPowerMode);
            result = LPM_WaitForInterrupt((uint64_t)1000 * xExpectedIdleTime / configTICK_RATE_HZ);
            APP_PowerPostSwitchHook(targetPowerMode, result);
        }
    }
    EnableGlobalIRQ(irqMask);
    /* Recovery clock(switch clock source from FRO to PLL0/1) after interrupt is enabled */
    BOARD_ResumeClockInit();
}

void PMIC_Reset(void)
{
    UPOWER_SetPmicReg(9 /* SW_RST */, 0x14 /* Cold reset */);
    /* full poweroff actually takes 16x T OFF_STEP (default 8ms),
     * just wait a while to be sure it should have worked,
     * and warn if it failed. */
    SDK_DelayAtLeastUs(250 * 1000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    PRINTF("Reset failed?!!\r\n");
    while (1)
        ;
}

/*! @brief Main function */
int main(void)
{
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_BootClockRUN();
    DebugConsole_Init();

    /* The default 1.0V settings in normal mode might go below 1.0,
     * so set them to 1.05V early on */
    if (UPOWER_ChngPmicVoltage(PMIC_BUCK2, 1050 * 1000))
    {
        PRINTF("failed to set PMIC_BUCK2 voltage to 1.05 [V]\r\n");
    }
    if (UPOWER_ChngPmicVoltage(PMIC_BUCK3, 1050 * 1000))
    {
        PRINTF("failed to set PMIC_BUCK3 voltage to 1.05 [V]\r\n");
    }

    /* if we didn't reset from power on also reset pmic */
    if (!(CMC_RTD->SRS & CMC_SRS_POR_MASK))
    {
        PRINTF("Reset cause not POR (%x), resetting PMIC\r\n", CMC_RTD->SRS);
        /* XXX try to remember somewhere we failed for uboot to log wdt... */
        PMIC_Reset();
    }

    UPOWER_PowerOnMemPart(0U, (uint32_t)kUPOWER_MP1_DMA0);

    /* Use Pll1Pfd2Div clock source 12.288MHz. */

    CLOCK_EnableClock(kCLOCK_Dma0Ch16);
    CLOCK_EnableClock(kCLOCK_Dma0Ch17);
    CLOCK_EnableClock(kCLOCK_RgpioA);
    CLOCK_EnableClock(kCLOCK_RgpioB);
    CLOCK_EnableClock(kCLOCK_RgpioC);
    CLOCK_EnableClock(kCLOCK_Wuu0);
    CLOCK_EnableClock(kCLOCK_Bbnsm);

    RESET_PeripheralReset(kRESET_Lpuart0);

    // XXX check what accesses these early, should not be needed
    CLOCK_SetIpSrcDiv(kCLOCK_Lpi2c0, kCLOCK_Pcc1BusIpSrcCm33Bus, 0U, 0U);
    CLOCK_SetIpSrcDiv(kCLOCK_Lpi2c1, kCLOCK_Pcc1BusIpSrcCm33Bus, 0U, 0U);
    // CLOCK_SetIpSrcDiv(kCLOCK_Lpi2c2, kCLOCK_Pcc2BusIpSrcFusionDspBus, 0U, 0U); // Secure Element
    RESET_PeripheralReset(kRESET_Lpi2c0);
    RESET_PeripheralReset(kRESET_Lpi2c1);
    // RESET_PeripheralReset(kRESET_Lpi2c2); // Secure Element

    APP_SRTM_Init();

    /* register callback for restart the app task when A35 reset */
    APP_SRTM_SetRpmsgMonitor(APP_RpmsgMonitor, NULL);

    /* If RTD reset is due to DPD exit, should go different flow here */
    if (CMC_RTD->SSRS & CMC_SSRS_WAKEUP_MASK)
    {
        CMC_RTD->SSRS = CMC_SSRS_WAKEUP_MASK;
        /* Assume that Application Domain is entered Deep Power Down Mode */
        AD_CurrentMode = AD_DPD;
        /*
         * AD is also in Deep Power Down mode when RTD is in Deep Power Down Mode.
         * AD/RTD exiting from Deep Power Down Mode is same with cold boot flow.
         * So don't need setup TRDC when RTD exit from Deep Power Down mode.
         *
         */
        // BOARD_SetTrdcAfterApdReset();
        MU_Init(MU0_MUA);
        MU_BootOtherCore(MU0_MUA, (mu_core_boot_mode_t)0);
    }
    else
    {
        APP_SRTM_StartCommunication();
    }

    LPM_Init();

    s_wakeupSig      = xSemaphoreCreateBinary();
    handleSuspendSig = xSemaphoreCreateBinary();

    PowerModeSwitchInit();

    /* suspend task must have higher priority than CLI task to properly
     * run and suspend main */
    BUILD_BUG_ON(MAIN_TASK_PRIORITY >= SUSPEND_TASK_PRIORITY);
    xTaskCreate(CLI_Task, "CLI Task", 512U, NULL, MAIN_TASK_PRIORITY, &cliTask);
    /* fails with task size of 256 and async debug console... */
    xTaskCreate(HandleSuspendTask, "HandleSuspendTask", 512U, NULL, SUSPEND_TASK_PRIORITY, NULL);

#ifdef MCMGR_USED
    /* Initialize MCMGR before calling its API */
    (void)MCMGR_Init();
#endif /* MCMGR_USED */

    APP_CreateTask();

    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();

    /* Application should never reach this point. */
    for (;;)
    {
    }
}
