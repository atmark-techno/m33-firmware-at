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
#include "fsl_debug_console.h"

#include "pin_mux.h"
#include "board.h"
#include "app_srtm.h"
#include "lpm.h"
#include "power_mode_switch.h"
#include "fsl_rtd_cmc.h"
#include "fsl_sentinel.h"
#include "fsl_rgpio.h"
#include "fsl_wuu.h"
#include "fsl_lpuart.h"

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

typedef enum _app_wakeup_source
{
    kAPP_WakeupSourceLptmr, /*!< Wakeup by LPTMR.        */
    kAPP_WakeupSourcePin    /*!< Wakeup by external pin. */
} app_wakeup_source_t;

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
static uint32_t s_wakeupTimeout;           /* Wakeup timeout. (Unit: Second) */
static app_wakeup_source_t s_wakeupSource; /* Wakeup source.                 */
static SemaphoreHandle_t s_wakeupSig;
static const char *s_modeNames[] = { "ACTIVE", "WAIT", "STOP", "Sleep", "Deep Sleep", "Power Down", "Deep Power Down" };
extern lpm_ad_power_mode_e AD_CurrentMode;
extern bool option_v_boot_flag;
extern lpm_rtd_power_mode_e s_curMode;
extern lpm_rtd_power_mode_e s_lastMode;
extern pca9460_buck3ctrl_t buck3_ctrl;
extern pca9460_ldo1_cfg_t ldo1_cfg;
extern bool wake_acore_flag;
rtd_mode_and_irq_allow_t current_state = { LPM_PowerModeActive, LPM_PowerModeActive, NotAvail_IRQn, RTD_GIVE_SIG_YES };
// clang-format off
/*
 * For some system low power combine, such as APD->PD, RTD->PD, use GPIO as RTD wakeup source, it will trigger WUU + GPIO irq handler in RTD side, release twice Semaphore Sig.
 * In below mode_combi_array_for_give_sig table, use current_rtd_mode and irq_num to judge whether RTD give semaphore sig.
 * Use current_rtd_mode and last_rtd_mode to judge whether RTD can wakeup APD.
 */
rtd_mode_and_irq_allow_t mode_combi_array_for_give_sig[] = {
  {LPM_PowerModeActive, LPM_PowerModeActive, NotAvail_IRQn, RTD_GIVE_SIG_YES},
  {LPM_PowerModeDeepSleep, LPM_PowerModeActive, WUU0_IRQn, RTD_GIVE_SIG_YES},
  {LPM_PowerModePowerDown, LPM_PowerModeActive, WUU0_IRQn, RTD_GIVE_SIG_YES},
  {LPM_PowerModeDeepPowerDown, LPM_PowerModeActive, WUU0_IRQn, RTD_GIVE_SIG_YES},

  {LPM_PowerModeWait, LPM_PowerModeActive, LPTMR1_IRQn, RTD_GIVE_SIG_YES},
  {LPM_PowerModeStop, LPM_PowerModeActive, LPTMR1_IRQn, RTD_GIVE_SIG_YES},
  {LPM_PowerModeSleep, LPM_PowerModeActive, LPTMR1_IRQn, RTD_GIVE_SIG_YES},
  {LPM_PowerModeDeepSleep, LPM_PowerModeActive, LPTMR1_IRQn, RTD_GIVE_SIG_YES},
  {LPM_PowerModePowerDown, LPM_PowerModeActive, LPTMR1_IRQn, RTD_GIVE_SIG_YES},

  {LPM_PowerModeWait, LPM_PowerModeActive, GPIOB_INT0_IRQn, RTD_GIVE_SIG_YES},
  {LPM_PowerModeStop, LPM_PowerModeActive, GPIOB_INT0_IRQn, RTD_GIVE_SIG_YES},
  {LPM_PowerModeSleep, LPM_PowerModeActive, GPIOB_INT0_IRQn, RTD_GIVE_SIG_YES},
  {LPM_PowerModeDeepSleep, LPM_PowerModeActive, GPIOB_INT0_IRQn, RTD_GIVE_SIG_YES},
};

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

/*******************************************************************************
 * Function Code
 ******************************************************************************/
#define SKIP_JTAG_PINS (0)
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

static void PinMuxPrepareSuspend(uint8_t gpioIdx, uint8_t pinIdx)
{
    uint8_t wuuIndex      = APP_IO_GetWUUPin(gpioIdx, pinIdx);
    __IO uint32_t *IOMUXC = &(gpioIdx == 0 ? IOMUXC0->PCR0_IOMUXCARRAY0 : IOMUXC0->PCR0_IOMUXCARRAY1)[pinIdx];

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

static void APP_Suspend(void)
{
    uint32_t i;
    uint32_t backupIndex;

    backupIndex = 0;

    /* Backup PTA IOMUXC and GPIOA ICR registers then disable */
    for (i = 0; i <= 24; i++)
    {
        iomuxBackup[backupIndex] = IOMUXC0->PCR0_IOMUXCARRAY0[i];

        gpioICRBackup[backupIndex] = GPIOA->ICR[i];

        GPIOA->ICR[i] = 0; /* Disable interrupts */

        /*
         * Skip PTA20 ~ 23(JTAG pins)[define SKIP_JTAG_PINS as 1] if want to debug code with JTAG before entering power
         * down/deep sleep mode
         */
#if SKIP_JTAG_PINS
        if (i < 20 || i > 23)
#endif
        {
            PinMuxPrepareSuspend(0, i);
        }
        backupIndex++;
    }
    gpioOutputBackup[0].pdor = GPIOA->PDOR;
    gpioOutputBackup[0].pddr = GPIOA->PDDR;

    /* Backup PTB IOMUXC and GPIOB ICR registers then disable */
    for (i = 0; i <= 15; i++)
    {
        iomuxBackup[backupIndex] = IOMUXC0->PCR0_IOMUXCARRAY1[i];

        gpioICRBackup[backupIndex] = GPIOB->ICR[i];

        GPIOB->ICR[i] = 0; /* disable interrupts */

        if ((i != 10) && (i != 11)) /* PTB10 and PTB11 is used as i2c function by upower */
        {
            PinMuxPrepareSuspend(1, i);
        }
        backupIndex++;
    }
    gpioOutputBackup[1].pdor = GPIOB->PDOR;
    gpioOutputBackup[1].pddr = GPIOB->PDDR;

    /* Backup PTC IOMUXC and GPIOC ICR registers then disable */
    for (i = 0; i <= 23; i++)
    {
        iomuxBackup[backupIndex] = IOMUXC0->PCR0_IOMUXCARRAY2[i];

        gpioICRBackup[backupIndex] = GPIOC->ICR[i];

        GPIOC->ICR[i] = 0; /* disable interrupts */

        /* Skip PTC0 ~ 10(FlexSPI0 pins) if run on flash(XiP) */
        if ((i > 10) || !BOARD_IS_XIP_FLEXSPI0())
        {
            IOMUXC0->PCR0_IOMUXCARRAY2[i] = 0;
        }

        backupIndex++;
    }
    gpioOutputBackup[2].pdor = GPIOC->PDOR;
    gpioOutputBackup[2].pddr = GPIOC->PDDR;

    /* Cleare any potential interrupts before enter Power Down */
    WUU0->PF = WUU0->PF;

    /* Save SRTM context */
    APP_SRTM_Suspend();
}

static void APP_Resume(bool resume)
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

    EnableIRQ(WUU0_IRQn);

    APP_SRTM_Resume(resume);
}

/* Disable gpio to save power */
void APP_DisableGPIO(void)
{
    int i = 0;

    /* Disable PTA and set PTA to Analog/HiZ state to save power */
    for (i = 0; i <= 24; i++)
    {
        GPIOA->ICR[i] = 0; /* Disable interrupts */

        /*
         * Skip PTA20 ~ 23(JTAG pins)[define SKIP_JTAG_PINS as 1] if want to debug code with JTAG before entering deep
         * power down mode
         */
#if SKIP_JTAG_PINS
        if (i < 20 || i > 23)
#endif
        {
            PinMuxPrepareSuspend(0, i);
        }
    }

    /* Disable PTB and set PTB to Analog/HiZ state to save power */
    for (i = 0; i <= 15; i++)
    {
        if ((i != 10) && (i != 11)) /* PTB10 and PTB11 is used as i2c function by upower */
        {
            GPIOB->ICR[i] = 0; /* Disable interrupts */
            PinMuxPrepareSuspend(1, i);
        }
    }

    /* Disable PTC and set PTC to Analog/HiZ state to save power */
    for (i = 0; i <= 23; i++)
    {
        GPIOC->ICR[i] = 0; /* Disable interrupts */

        /* Skip PTC0 ~ 10(FlexSPI0 pins) if run on flash(XiP) */
        if ((i > 10) || !BOARD_IS_XIP_FLEXSPI0())
        {
            IOMUXC0->PCR0_IOMUXCARRAY2[i] = 0; /* Set to Analog/HiZ state */
        }
    }
}

void APP_PowerPreSwitchHook(lpm_rtd_power_mode_e targetMode)
{
    if ((LPM_PowerModeActive != targetMode))
    {
        /* Wait for debug console output finished. */
        while (!(kLPUART_TransmissionCompleteFlag & LPUART_GetStatusFlags((LPUART_Type *)BOARD_DEBUG_UART_BASEADDR)))
        {
        }
        DbgConsole_Deinit();
        /*
         * Set pin for current leakage.
         * Debug console RX pin: Set to pinmux to analog.
         * Debug console TX pin: Set to pinmux to analog.
         */
        BOARD_DeinitConsolePins();

        if (LPM_PowerModePowerDown == targetMode || LPM_PowerModeDeepSleep == targetMode)
        {
            APP_Suspend();
        }
        else if (LPM_PowerModeDeepPowerDown == targetMode)
        {
            APP_DisableGPIO();

            /* Cleare any potential interrupts before enter Deep Power Down */
            WUU0->PF = WUU0->PF;
        }
    }
}

void APP_PowerPostSwitchHook(lpm_rtd_power_mode_e targetMode, bool result)
{
    if (LPM_PowerModeActive != targetMode)
    {
        if (LPM_PowerModePowerDown == targetMode || LPM_PowerModeDeepSleep == targetMode)
        {
            APP_Resume(result);
        }

        /*
         * Debug console RX pin was set to disable for current leakage, need to re-configure pinmux.
         * Debug console TX pin was set to disable for current leakage, need to re-configure pinmux.
         */
        BOARD_InitConsolePins();

        BOARD_InitClock(); /* initialize system osc for uart(using osc as clock source) */
        BOARD_InitDebugConsole();
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

static void APP_ShowModeCombi(void)
{
    int i = 0;

    PRINTF("###############################################\r\n");
    PRINTF("For Single Boot Type\r\n");
    for (i = 0; i < ARRAY_SIZE(mode_combi_array_for_single_boot); i++)
    {
        PRINTF("%s + %s: %s\r\n", APP_GetAdPwrModeName(mode_combi_array_for_single_boot[i].ad_mode),
               APP_GetRtdPwrModeName(mode_combi_array_for_single_boot[i].rtd_mode),
               APP_GetAllowCombiName(mode_combi_array_for_single_boot[i].allow_combi));
    }
    PRINTF("###############################################\r\n");
    PRINTF("\r\n");
    PRINTF("\r\n");

    PRINTF("###############################################\r\n");
    PRINTF("For Dual Boot Type/Low Power Boot Type\r\n");
    for (i = 0; i < ARRAY_SIZE(mode_combi_array_for_dual_or_lp_boot); i++)
    {
        PRINTF("%s + %s: %s\r\n", APP_GetAdPwrModeName(mode_combi_array_for_dual_or_lp_boot[i].ad_mode),
               APP_GetRtdPwrModeName(mode_combi_array_for_dual_or_lp_boot[i].rtd_mode),
               APP_GetAllowCombiName(mode_combi_array_for_dual_or_lp_boot[i].allow_combi));
    }
    PRINTF("###############################################\r\n");
}

static allow_give_sig_e APP_AllowGiveSig(lpm_rtd_power_mode_e current_mode, uint32_t irq)
{
    int i                  = 0;
    allow_give_sig_e allow = false;

    for (i = 0; i < ARRAY_SIZE(mode_combi_array_for_give_sig); i++)
    {
        if ((current_mode == mode_combi_array_for_give_sig[i].current_rtd_mode) &&
            (irq == mode_combi_array_for_give_sig[i].irq_num))
        {
            allow = mode_combi_array_for_give_sig[i].give_semaphore_flag;
            break;
        }
    }
    return allow;
}

/* WUU0 interrupt handler. */
void APP_WUU0_IRQHandler(void)
{
    /* "really" wake up for LPTMR1 alarm or GPIO.
     * If only LPTRM0 (SYSTICK) then we'll sleep again */
    bool wakeup = false;

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

    if (WUU0->PF)
    {
        /* clear pin interrupt flag */
        WUU0->PF = WUU0->PF;
        wakeup   = true;
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

    current_state.give_semaphore_flag = APP_AllowGiveSig(s_curMode, LPTMR1_IRQn);
    if (!current_state.give_semaphore_flag)
    {
        /* skip give semaphore */
        return;
    }

    if (wakeup)
    {
        xSemaphoreGiveFromISR(s_wakeupSig, NULL);
        portYIELD_FROM_ISR(pdTRUE);
    }
}

/*
 * Wakeup RTD: lptimer or button(sw8).
 *
 *              |APD at PD: option(W), sw6(on/off), sw7, sw8.
 * Wakeup APD:  |
 *              |APD at DPD: option(W), sw6(on/off).
 *
 * RTD Wait/Stop/Sleep mode: sw8 will trigger two GPIO interrupt(kRGPIO_FlagEitherEdge) determined by APD IO service.
 * RTD DSL/PD/DPD mode: sw8 will trigger one WUU + one GPIO interrupt.
 */
static void APP_IRQDispatcher(IRQn_Type irq, void *param)
{
    /* ensure that only RTD/APD is woken up at a time button */
    current_state.last_rtd_mode       = current_state.current_rtd_mode;
    current_state.current_rtd_mode    = s_curMode;
    current_state.irq_num             = irq;
    current_state.give_semaphore_flag = RTD_GIVE_SIG_YES;

    if (current_state.current_rtd_mode == current_state.last_rtd_mode ||
        irq == BBNSM_IRQn) /* Always set the wake_acore_flag to true for the interrupt from BBNSM */
    {
        /*
         * RTD don't update low power state by using button wakeup APD.
         * so only when the last state is same with current state, button sw8 can wakeup APD.
         */
        wake_acore_flag = true;
    }
    else
    {
        wake_acore_flag = false;
    }

    current_state.give_semaphore_flag = APP_AllowGiveSig(s_curMode, irq);

    if (!current_state.give_semaphore_flag)
    {
        /* skip give semaphore */
        return;
    }
    switch (irq)
    {
        case WUU0_IRQn:
            APP_WUU0_IRQHandler();
            break;
        default:
            break;
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
            c = GETCHAR();
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
static void APP_GetWakeupConfig(void)
{
    /* Get wakeup source by user input. */
    s_wakeupSource = kAPP_WakeupSourceLptmr;

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
            APP_SRTM_SetWakeupModule(WUU_MODULE_LPTMR1, LPTMR1_WUU_WAKEUP_EVENT);
            PCC1->PCC_LPTMR1 &= ~PCC1_PCC_LPTMR1_SSADO_MASK;
            PCC1->PCC_LPTMR1 |= PCC1_PCC_LPTMR1_SSADO(1);
        }
    }
}

static void APP_ClearWakeupConfig(lpm_rtd_power_mode_e targetMode)
{
    if ((LPM_PowerModePowerDown == targetMode) || (LPM_PowerModeDeepPowerDown == targetMode))
    {
        APP_SRTM_ClrWakeupModule(WUU_MODULE_LPTMR1, LPTMR1_WUU_WAKEUP_EVENT);
    }
}

static void APP_CreateTask(void) {}

static void APP_SuspendTask(void) {}

static void APP_ResumeTask(void) {}

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

/* Power Mode Switch task */
void PowerModeSwitchTask(void *pvParameters)
{
    lptmr_config_t lptmrConfig;
    lpm_rtd_power_mode_e targetPowerMode;
    uint32_t freq = 0U;
    uint8_t ch;

    /* As IRQ handler main entry locates in app_srtm.c to support services, here need an entry to handle application
     * IRQ events.
     */
    APP_SRTM_SetIRQHandler(APP_IRQDispatcher, NULL);
    /* Add Systick as Power Down wakeup source, depending on SYSTICK_WUU_WAKEUP_EVENT value. */
    APP_SRTM_SetWakeupModule(WUU_MODULE_SYSTICK, SYSTICK_WUU_WAKEUP_EVENT);

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

    for (;;)
    {
        freq = CLOCK_GetFreq(kCLOCK_Cm33CorePlatClk);
        PRINTF("\r\n####################  Power Mode Switch Task ####################\n\r\n");
        PRINTF("    Build Time: %s--%s \r\n", __DATE__, __TIME__);
        PRINTF("    Core Clock: %dHz \r\n", freq);
        PRINTF("    Boot Type: %s \r\n", BOARD_GetBootTypeName());
        PRINTF("\r\nSelect the desired operation \n\r\n");
        PRINTF("Press  %c to enter: Active mode\r\n", kAPP_PowerModeActive);
        PRINTF("Press  %c to enter: Cortex M33 Wait mode\r\n", kAPP_PowerModeWait);
        PRINTF("Press  %c to enter: Cortex M33 STOP mode\r\n", kAPP_PowerModeStop);
        PRINTF("Press  %c to enter: Sleep mode\r\n", kAPP_PowerModeSleep);
        PRINTF("Press  %c to enter: Deep Sleep mode\r\n", kAPP_PowerModeDeepSleep);
        PRINTF("Press  %c to enter: Power Down(PD) mode\r\n", kAPP_PowerModePowerDown);
        PRINTF("Press  %c to enter: Deep Power Down(DPD) mode\r\n", kAPP_PowerModeDeepPowerDown);
        PRINTF("Press  W for wake up CA35 core from PD/DPD mode\r\n");
        PRINTF("Press  T for reboot CA35 core\r\n");
        PRINTF("Press  U for shutdown CA35 core.\r\n");
        PRINTF("Press  V for boot CA35 core.\r\n");
        PRINTF("Press  S for showing supported LPM Mode Combination.\r\n");
        // clang-format off
        /*
         * OD: Over Drive Mode
         * ND: Norminal Drive Mode
         * Ud: Under Drive Mode
         *+-----------------------------------------------------------------------------------------------------+
         *|      Drive Mode of Cortex-M33   |           OD          |           ND         |          UD        |
         *+-----------------------------------------------------------------------------------------------------+
         *|         Voltage of BUCK2        |    1.05 V ~ 1.10 V    |     0.95 V ~ 1.00 V  |         0.9 V      |
         *+-----------------------------------------------------------------------------------------------------+
         *|         Biasing Option          |         AFBB          |         AFBB         |         ARBB       |
         *+-----------------------------------------------------------------------------------------------------+
         *|         Maximum frequency       |        216 MHz        |        160 MHz       |      38.4 MHz      |
         *+-----------------------------------------------------------------------------------------------------+
         */
        // clang-format on
        PRINTF("Press  M for switch Voltage Drive Mode between OD/ND/UD.\r\n");
        PRINTF(
            "Press  N for supporting Deep Sleep Mode(Pls set it when the option IMX8ULP_DSL_SUPPORT of TF-A is "
            "enabled) of Linux. support_dsl_for_apd = %d\r\n",
            APP_SRTM_GetSupportDSLForApd());
        PRINTF("\r\nWaiting for power mode select..\r\n\r\n");

        /* Wait for user response */
        do
        {
            ch = GETCHAR();
        } while ((ch == '\r') || (ch == '\n'));

        if ((ch >= 'a') && (ch <= 'z'))
        {
            ch -= 'a' - 'A';
        }
        targetPowerMode = (lpm_rtd_power_mode_e)(ch - 'A');
        if (targetPowerMode <= LPM_PowerModeDeepPowerDown)
        {
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
                APP_SuspendTask();
                APP_GetWakeupConfig();
                APP_SetWakeupConfig(targetPowerMode);
                xSemaphoreTake(s_wakeupSig, portMAX_DELAY);
                /* The call might be blocked by SRTM dispatcher task. Must be called after power mode reset. */
                APP_ClearWakeupConfig(targetPowerMode);
                APP_ResumeTask();
            }
        }
        else if ('W' == ch)
        {
            if (!wake_acore_flag)
            {
                wake_acore_flag = true;
            }
            APP_SRTM_WakeupCA35();
        }
        else if ('T' == ch)
        {
            APP_RebootCA35();
        }
        else if ('U' == ch)
        {
            APP_ShutdownCA35();
        }
        else if ('V' == ch)
        {
            option_v_boot_flag = true;
            APP_BootCA35();
        }
        else if ('S' == ch)
        {
            APP_ShowModeCombi();
        }
        else if ('M' == ch)
        {
            BOARD_SwitchDriveMode();
        }
        else if ('N' == ch)
        {
            PRINTF("Warning: Pls ensure that the option IMX8ULP_DSL_SUPPORT is enabled in TF-A\r\n");
            APP_SRTM_SetSupportDSLForApd(true);
        }
        else
        {
            PRINTF("Invalid command %c[0x%x]\r\n", ch, ch);
        }
        /*update Mode state*/
        s_lastMode = s_curMode;
        s_curMode  = LPM_PowerModeActive;
        PRINTF("\r\nNext loop\r\n");
    }
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
    BOARD_InitDebugConsole();

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

    CLOCK_SetIpSrcDiv(kCLOCK_Tpm0, kCLOCK_Pcc1BusIpSrcCm33Bus, 1U, 0U);
    CLOCK_SetIpSrcDiv(kCLOCK_Lpi2c0, kCLOCK_Pcc1BusIpSrcCm33Bus, 0U, 0U);
    CLOCK_SetIpSrcDiv(kCLOCK_Lpi2c1, kCLOCK_Pcc1BusIpSrcCm33Bus, 0U, 0U);
    // CLOCK_SetIpSrcDiv(kCLOCK_Lpi2c2, kCLOCK_Pcc2BusIpSrcFusionDspBus, 0U, 0U); // Secure Element
    /* Use Pll1Pfd2Div clock source 12.288MHz. */
    CLOCK_SetIpSrc(kCLOCK_Lpuart0, kCLOCK_Pcc1BusIpSrcSysOscDiv2);

    CLOCK_EnableClock(kCLOCK_Dma0Ch16);
    CLOCK_EnableClock(kCLOCK_Dma0Ch17);
    CLOCK_EnableClock(kCLOCK_RgpioA);
    CLOCK_EnableClock(kCLOCK_RgpioB);
    CLOCK_EnableClock(kCLOCK_RgpioC);
    CLOCK_EnableClock(kCLOCK_Wuu0);
    CLOCK_EnableClock(kCLOCK_Bbnsm);

    RESET_PeripheralReset(kRESET_Lpi2c0);
    RESET_PeripheralReset(kRESET_Lpi2c1);
    // RESET_PeripheralReset(kRESET_Lpi2c2); // Secure Element
    RESET_PeripheralReset(kRESET_Tpm0);
    RESET_PeripheralReset(kRESET_Lpuart0);

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

    s_wakeupSig = xSemaphoreCreateBinary();

    xTaskCreate(PowerModeSwitchTask, "Main Task", 512U, NULL, tskIDLE_PRIORITY + 1U, NULL);

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
