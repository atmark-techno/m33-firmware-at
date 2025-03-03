/*
 * Copyright 2021-2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "main.h"

#include "srtm_dispatcher.h"
#include "srtm_peercore.h"
#include "srtm_message.h"
#include "srtm_rpmsg_endpoint.h"

#include "srtm_io_service.h"
#include "srtm_lfcl_service.h"
#include "srtm_rtc_service.h"
#include "srtm_rtc_adapter.h"
#include "srtm_wdog_service.h"

#include "app_srtm.h"
#include "app_srtm_internal.h"
#include "app_uboot.h"
#include "app_tty.h"
#include "board.h"
#include "pin_mux.h"
#include "build_bug.h"
#include "fsl_mu.h"
#include "printf.h"
#include "fsl_rgpio.h"
#include "fsl_wuu.h"
#include "fsl_upower.h"
#include "fsl_iomuxc.h"
#include "rsc_table.h"
#include "fsl_bbnsm.h"
#include "fsl_sentinel.h"
#include "fsl_reset.h"
#include "fsl_ewm.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
void APP_SRTM_WakeupCA35(void);

typedef struct
{
    bool wakeup;
} app_io_t;

/* NOTE: CM33 DRIVERS DON'T SUPPORT SAVE CONTEXT FOR RESUME, BUT CA35 LINUX DRIVERS DO.
 * WHEN CM33 CORE RUNS INTO VLLS MODE, MOST PERIPHERALS STATE WILL BE LOST. HERE PROVIDES
 * AN EXAMPLE TO SAVE DEVICE STATE BY APPLICATION IN A SUSPEND CONTEXT LOCATING IN TCM
 * WHICH CAN KEEP DATA IN VLLS MODE.
 */
typedef struct
{
    struct
    {
        app_io_t data[APP_IO_NUM];
    } io;
    struct
    {
        uint16_t timeout;
    } wdog;
} app_suspend_ctx_t;

typedef enum
{
    CORE_ACT  = CMC_AD_AD_A35CORE0_LPMODE_A35CORE0_LPMODE(0x0U),
    CORE_STDB = CMC_AD_AD_A35CORE0_LPMODE_A35CORE0_LPMODE(0x1U),
    CORE_PD   = CMC_AD_AD_A35CORE0_LPMODE_A35CORE0_LPMODE(0x3U),
} core_low_power_mode_t; /* A35 core0/1 low power mode */

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile app_srtm_state_t srtmState;
bool option_v_boot_flag          = false;
static bool need_reset_peer_core = false;
bool flexio_used;

/* For CMC1_IRQHandler */
static int64_t apd_boot_cnt = 0; /* it's cold boot when apd_boot_cnt(Application Domain, A Core) == 1 */

/*
 * For Deep Sleep Mode of Application Processor Domain(APD)
 *                         +--> APD enter Power Down Mode
 *                        /
 * Linux suspend to memory
 *                        \
 *                         +--> APD enter Deep Sleep Mode(When the option IMX8ULP_DSL_SUPPORT is enabled in TF-A)
 * NOTE: cannot support them at the same time for Linux.
 */

bool support_dsl_for_apd = false; /* true: support deep sleep mode; false: not support deep sleep mode */

const uint8_t wuuPins[] = {
    0,   /* WUU_P0 PTA0 */
    255, /* PTA1 */
    255, /* PTA2 */
    1,   /* WUU0_P1 PTA3 */
    2,   /* WUU0_P2 PTA4 */
    255, /* PTA5 */
    3,   /* WUU0_P3 PTA6 */
    4,   /* WUU0_P4 PTA7 */
    5,   /* WUU0_P5 PTA8 */
    6,   /* WUU0_P6 PTA9 */
    7,   /* WUU0_P7 PTA10 */
    8,   /* WUU0_P8 PTA11 */
    9,   /* WUU0_P9 PTA12 */
    10,  /* WUU0_P10 PTA13 */
    11,  /* WUU0_P11 PTA14 */
    12,  /* WUU0_P12 PTA15 */
    13,  /* WUU0_P13 PTA16 */
    14,  /* WUU0_P14 PTA17 */
    15,  /* WUU0_P15 PTA18 */
    255, /* PTA19 */
    255, /* PTA20 */
    255, /* PTA21 */
    255, /* PTA22 */
    255, /* PTA23 */
    16,  /* WUU0_P16 PTA24 */
    17,  /* WUU0_P17 PTB0 */
    18,  /* WUU0_P18 PTB1 */
    19,  /* WUU0_P19 PTB2 */
    20,  /* WUU0_P20 PTB3 */
    21,  /* WUU0_P21 PTB4 */
    22,  /* WUU0_P22 PTB5 */
    23,  /* WUU0_P23 PTB6 */
    255, /* PTB7 */
    255, /* PTB8 */
    255, /* PTB9 */
    255, /* PTB10 */
    255, /* PTB11 */
    24,  /* WUU0_P24 PTB12 */
    25,  /* WUU0_P25 PTB13 */
    26,  /* WUU0_P26 PTB14 */
    27,  /* WUU0_P27 PTB15 */
    255, /* 16 (no more pins after PTB15) */
    255, /* 17 */
    255, /* 18 */
    255, /* 19 */
    255, /* 20 */
    255, /* 21 */
    255, /* 22 */
    255, /* 23 */
    255, /* 24 */
};

srtm_dispatcher_t disp;
static srtm_peercore_t core;
static srtm_service_t rtcService;
static srtm_rtc_adapter_t rtcAdapter;
static srtm_service_t ioService;
static srtm_service_t wdogService;
static SemaphoreHandle_t monSig;
static struct rpmsg_lite_instance *rpmsgHandle;
static app_rpmsg_monitor_t rpmsgMonitor;
static void *rpmsgMonitorParam;
static TimerHandle_t linkupTimer;
static TimerHandle_t refreshS400WdgTimer;
static TimerHandle_t rtcAlarmEventTimer; /* It is used to send alarm event to acore after acore(acore entered power down
                                            mode) is waken by rtc alarm(Avoid losting a rtc alarm event) */
static TimerHandle_t restoreRegValOfMuTimer; /* use the timer to restore register's value of mu(To make sure that
                                                register's value of mu is restored if cmc1 interrupt is not comming) */

static TimerHandle_t
    chngModeFromActToDslForApdTimer; /* use the timer to change mode of APD from active mode to Deep Sleep Mode */

static app_irq_handler_t irqHandler;
static void *irqHandlerParam;

static HAL_RTC_HANDLE_DEFINE(rtcHandle);

lpm_ad_power_mode_e AD_CurrentMode   = AD_UNKOWN;
lpm_ad_power_mode_e AD_WillEnterMode = AD_UNKOWN;

static RGPIO_Type *const gpios[] = RGPIO_BASE_PTRS;
#define IO_PINCTRL_UNSET 0xffffffffU

static app_suspend_ctx_t suspendContext;

static MU_Type mu0_mua;
/*******************************************************************************
 * Code
 ******************************************************************************/
/* For Deep Sleep Mode of APD */
bool APP_SRTM_GetSupportDSLForApd(void)
{
    return support_dsl_for_apd;
}

void APP_SRTM_SetSupportDSLForApd(bool support)
{
    support_dsl_for_apd = support;
}

void MU0_MUA_Save(void)
{
    /* Make sure the clock is on */
    MU_Init(MU0_MUA);
    mu0_mua.RCR   = MU0_MUA->RCR;
    mu0_mua.CIER0 = MU0_MUA->CIER0;
}

void MU0_MUA_Restore(void)
{
    /* Make sure the clock is on */
    MU_Init(MU0_MUA);
    if (mu0_mua.RCR != 0)
    {
        MU0_MUA->RCR = mu0_mua.RCR;
    }
    if (mu0_mua.CIER0 != 0)
    {
        MU0_MUA->CIER0 = mu0_mua.CIER0;
    }
}

/* Real Time Domain save context */
void rtdCtxSave(void)
{
    MU0_MUA_Save();
}

/* Real Time Domain restore context */
void rtdCtxRestore(void)
{
    MU0_MUA_Restore();
}

void APP_WakeupACore(void)
{
    if (UPOWER_ChngPmicVoltage(PMIC_LDO3, 3300 * 1000))
    {
        PRINTF("failed to set PMIC_LDO3 voltage to 3.3 [V]\r\n");
    }
    if (UPOWER_ChngPmicVoltage(PMIC_LSW2, 1800 * 1000))
    {
        PRINTF("failed to set PMIC_LSW2 voltage to 1.8 [V]\r\n");
    }
    if (UPOWER_ChngPmicVoltage(PMIC_LSW4, 1100 * 1000))
    {
        PRINTF("failed to set PMIC_LSW4 voltage to 1.1 [V]\r\n");
    }

    if (support_dsl_for_apd == true && AD_CurrentMode == AD_DSL)
    {
        MU_SendMsg(MU0_MUA, 1, 0xFFFFFFFF); /* MCore wakeup ACore with mu interrupt, 1: RPMSG_MU_CHANNEL */
    }
    else
    {
        UPOWER_PowerOnADInPDMode();
    }
}

static void APP_ResetSRTM(app_srtm_state_t state)
{
    srtmState = state;
    /* Wake up monitor to reinitialize the SRTM communication with CA35 */
    xSemaphoreGive(monSig);
}

static void APP_SRTM_ControlCA35(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    app_srtm_state_t state = (app_srtm_state_t)(uint32_t)param1;

    switch (state)
    {
        case APP_SRTM_StateRun:
            /* Fresh power up: Need SRTM monitor to prepare communication */
            srtmState = APP_SRTM_StateRun;
            xSemaphoreGive(monSig);
            break;
        case APP_SRTM_StateReboot:
            /* Only when CA35 is active, we can reboot it. */
            if (!core || AD_CurrentMode != AD_ACT)
            {
                PRINTF("CA35 is not active, cannot reboot!\r\n");
            }
            else
            {
                /* Now prepare reboot */
                need_reset_peer_core = true; /* set a flag to check whether need reset peer core(don't need reset peer
                                                core when peer core is in reset) */
                APP_ResetSRTM(APP_SRTM_StateReboot);
            }
            break;
        case APP_SRTM_StateShutdown:
            /* Only when CA35 goes into DPD, we can shutdown it. */
            if (core && AD_CurrentMode == AD_DPD)
            {
                /* Now prepare shutdown */
                APP_ResetSRTM(APP_SRTM_StateShutdown);
            }
            else
            {
                PRINTF("CA35 isn't in PD mode, cannot shutdown!\r\n");
            }
            break;
        default:
            break;
    }
}

void APP_RebootCA35(void)
{
    srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_ControlCA35, (void *)APP_SRTM_StateReboot, NULL);
    PRINTF("M33 reboot A35\r\n");
    assert(proc);
    SRTM_Dispatcher_PostProc(disp, proc);
}

void APP_SRTM_ShutdownCA35(void)
{
    srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_ControlCA35, (void *)APP_SRTM_StateShutdown, NULL);

    assert(proc);
    SRTM_Dispatcher_PostProc(disp, proc);
}

void APP_SRTM_EmulateGPIOHandler(uint8_t wuuPin)
{
    uint16_t ioId            = APP_WUUPin_TO_IoId(wuuPin);
    uint8_t __unused gpioIdx = APP_GPIO_IDX(ioId);
    uint8_t __unused pinIdx  = APP_PIN_IDX(ioId);

    assert(gpioIdx < 2); /* Only support GPIOA and GPIOB */
    assert(pinIdx < APP_IO_PINS_PER_CHIP);

    SRTM_IoService_NotifyInputEvent(ioService, ioId);
}

static void APP_HandleGPIOHander(uint8_t gpioIdx)
{
    RGPIO_Type *gpio = gpios[gpioIdx];
    uint32_t flags   = RGPIO_GetPinsInterruptFlags(gpio, APP_GPIO_INT_SEL);
    uint16_t ioId, ioIdx;
    uint8_t i;
    uint32_t idx;

    for (i = 0; i < APP_IO_PINS_PER_CHIP; i++)
    {
        idx = 1U << i;
        if (!(flags & idx))
            continue;
        ioId  = APP_IO_ID(gpioIdx, i);
        ioIdx = APP_IO_IDX(gpioIdx, i);
        if ((AD_CurrentMode == AD_PD || (support_dsl_for_apd == true && AD_CurrentMode == AD_DSL)) &&
            suspendContext.io.data[ioIdx].wakeup)
        {
            /* Wakeup A Core(CA35) when A Core is in Power Down Mode */
            PRINTF("Wake up from gpio %d/%d\r\n", gpioIdx, i);
            APP_WakeupACore();
        }
        SRTM_IoService_NotifyInputEvent(ioService, ioId);
        // disable further irq for pin, linux will re-enable after processing
        // (this is necessary e.g. for level interrupts to not spam)
        RGPIO_SetPinInterruptConfig(gpio, i, APP_GPIO_INT_SEL, kRGPIO_InterruptOrDMADisabled);
        // clear isr
        RGPIO_ClearPinsInterruptFlags(gpio, APP_GPIO_INT_SEL, idx);
    }
}

void GPIOA_INT0_IRQHandler(void)
{
    APP_HandleGPIOHander(0U);
}

void GPIOA_INT1_IRQHandler(void)
{
    APP_HandleGPIOHander(0U);
}

void GPIOB_INT0_IRQHandler(void)
{
    APP_HandleGPIOHander(1U);
}

void GPIOB_INT1_IRQHandler(void)
{
    APP_HandleGPIOHander(1U);
}

void GPIOC_INT0_IRQHandler(void)
{
    APP_HandleGPIOHander(2U);
}

void GPIOC_INT1_IRQHandler(void)
{
    APP_HandleGPIOHander(2U);
}

static void rtcAlarmEventTimer_Callback(TimerHandle_t xTimer)
{
    uint32_t status = MU_GetCoreStatusFlags(MU0_MUA);

    if (status & kMU_OtherSideEnterRunFlag) /* A Core in run mode */
    {
        /* Send rpmsg to A Core when Application Domain in active mode(Make sure that DDR is working) */
        SRTM_RtcAdapter_NotifyAlarm(
            rtcAdapter); /* The function SRTM_RtcAdapter_NotifyAlarm will clear alarm interrupt flag */

        xTimerStop(rtcAlarmEventTimer, portMAX_DELAY);
    }
    else
    {
        xTimerStart(rtcAlarmEventTimer, portMAX_DELAY);
    }
}

void BBNSM_IRQHandler(void)
{
    BaseType_t reschedule = pdFALSE;
    uint32_t status       = BBNSM_GetStatusFlags(BBNSM);

    /*
     * Process RTC alarm if present.
     * BBNSM IRQ enable is done in RTC service initialization. So rtcAdapter must be ready.
     */
    if (status & kBBNSM_RTC_AlarmInterruptFlag)
    {
        if (AD_CurrentMode == AD_PD ||
            (support_dsl_for_apd == true &&
             AD_CurrentMode == AD_DSL)) /* Application Domain is in Power Down Mode or Deep Sleep Mode */
        {
            /* disable rtc alarm interrupt(when will clear rtc alarm interrupt flag? it will be cleared in
             * rtcAlarmEventTimer) */
            SRTM_RtcAdapter_DisableAlarmInt(rtcAdapter);
            /* Wakeup A Core (A35) */
            APP_WakeupACore();
            /* Send rtc alarm event in timer */
            xTimerStartFromISR(rtcAlarmEventTimer, &reschedule);
        }
        else if (AD_CurrentMode == AD_ACT)
        {
            /* Send rpmsg to A Core when Application Domain in active mode(Make sure that DDR is working) */
            SRTM_RtcAdapter_NotifyAlarm(
                rtcAdapter); /* The function SRTM_RtcAdapter_NotifyAlarm will clear alarm interrupt flag */
        }
        else
        {
            /* Clear rtc alarm interrupt in other case */
            BBNSM_ClearStatusFlags(BBNSM, kBBNSM_RTC_AlarmInterruptFlag);
        }
    }

    if (status & kBBNSM_EMG_OFF_InterruptFlag)
    {
        /* Clear emergency power off interrupt flag */
        BBNSM_ClearStatusFlags(BBNSM, kBBNSM_EMG_OFF_InterruptFlag);
    }

    if (status & kBBNSM_PWR_OFF_InterruptFlag)
    {
        if (AD_CurrentMode == AD_DPD) /* Application Domain is in Deep Power Down Mode/Power Down Mode */
        {
            /* Wakeup A Core (A35) */
            APP_SRTM_WakeupCA35();
        }
        else if (AD_CurrentMode == AD_PD || (support_dsl_for_apd == true && AD_CurrentMode == AD_DSL))
        {
            /* Wakeup A Core (A35) */
            APP_WakeupACore();
        }
        /* Clear BBNSM button off interrupt */
        BBNSM_ClearStatusFlags(BBNSM, kBBNSM_PWR_OFF_InterruptFlag);
    }
    else if (status & kBBNSM_PWR_ON_InterruptFlag)
    {
        /* Clear BBNSM button on interrupt */
        BBNSM_ClearStatusFlags(BBNSM, kBBNSM_PWR_ON_InterruptFlag);
    }

    if (reschedule)
    {
        portYIELD_FROM_ISR(reschedule);
    }
}

#define PIN_FUNC_ID_SIZE (6)
static uint32_t pinFuncId[][PIN_FUNC_ID_SIZE] = {
    { IOMUXC_PTA0_PTA0, -1 },
    { IOMUXC_PTA1_PTA1, -1 },
    { IOMUXC_PTA2_PTA2, -1 },
    { IOMUXC_PTA3_PTA3, -1 },
    { IOMUXC_PTA4_PTA4, -1 },
    { IOMUXC_PTA5_PTA5, -1 },
    { IOMUXC_PTA6_PTA6, -1 },
    { IOMUXC_PTA7_PTA7, -1 },
    { IOMUXC_PTA8_PTA8, -1 },
    { IOMUXC_PTA9_PTA9, -1 },
    { IOMUXC_PTA10_PTA10, -1 },
    { IOMUXC_PTA11_PTA11, -1 },
    { IOMUXC_PTA12_PTA12, -1 },
    { IOMUXC_PTA13_PTA13, -1 },
    { IOMUXC_PTA14_PTA14, -1 },
    { IOMUXC_PTA15_PTA15, -1 },
    { IOMUXC_PTA16_PTA16, -1 },
    { IOMUXC_PTA17_PTA17, -1 },
    { IOMUXC_PTA18_PTA18, -1 },
    { IOMUXC_PTA19_PTA19, -1 },
    { IOMUXC_PTA20_PTA20, -1 },
    { IOMUXC_PTA21_PTA21, -1 },
    { IOMUXC_PTA22_PTA22, -1 },
    { IOMUXC_PTA23_PTA23, -1 },
    { IOMUXC_PTA24_PTA24, -1 },
    { IOMUXC_PTB0_PTB0, -1 },
    { IOMUXC_PTB1_PTB1, -1 },
    { IOMUXC_PTB2_PTB2, -1 },
    { IOMUXC_PTB3_PTB3, -1 },
    { IOMUXC_PTB4_PTB4, -1 },
    { IOMUXC_PTB5_PTB5, -1 },
    { IOMUXC_PTB6_PTB6, -1 },
    { IOMUXC_PTB7_PTB7, -1 },
    { IOMUXC_PTB8_PTB8, -1 },
    { IOMUXC_PTB9_PTB9, -1 },
    { IOMUXC_PTB10_PTB10, -1 },
    { IOMUXC_PTB11_PTB11, -1 },
    { IOMUXC_PTB12_PTB12, -1 },
    { IOMUXC_PTB13_PTB13, -1 },
    { IOMUXC_PTB14_PTB14, -1 },
    { IOMUXC_PTB15_PTB15, -1 },
    { 0 }, /* no PTB after 15 */
    { 0 },
    { 0 },
    { 0 },
    { 0 },
    { 0 },
    { 0 },
    { 0 },
    { 0 },
    { IOMUXC_PTC0_PTC0, -1 },
    { IOMUXC_PTC1_PTC1, -1 },
    { IOMUXC_PTC2_PTC2, -1 },
    { IOMUXC_PTC3_PTC3, -1 },
    { IOMUXC_PTC4_PTC4, -1 },
    { IOMUXC_PTC5_PTC5, -1 },
    { IOMUXC_PTC6_PTC6, -1 },
    { IOMUXC_PTC7_PTC7, -1 },
    { IOMUXC_PTC8_PTC8, -1 },
    { IOMUXC_PTC9_PTC9, -1 },
    { IOMUXC_PTC10_PTC10, -1 },
    { IOMUXC_PTC11_PTC11, -1 },
    { IOMUXC_PTC12_PTC12, -1 },
    { IOMUXC_PTC13_PTC13, -1 },
    { IOMUXC_PTC14_PTC14, -1 },
    { IOMUXC_PTC15_PTC15, -1 },
    { IOMUXC_PTC16_PTC16, -1 },
    { IOMUXC_PTC17_PTC17, -1 },
    { IOMUXC_PTC18_PTC18, -1 },
    { IOMUXC_PTC19_PTC19, -1 },
    { IOMUXC_PTC20_PTC20, -1 },
    { IOMUXC_PTC21_PTC21, -1 },
    { IOMUXC_PTC22_PTC22, -1 },
    { IOMUXC_PTC23_PTC23, -1 },
    { 0 }, /* no PTC24 */
};

static void pinctrl_set(uint32_t pinctrl0, uint32_t pinctrl1, uint32_t pinctrl2, uint32_t pinctrl3, uint32_t pinctrl4,
                        uint32_t pinctrl5)
{
    IOMUXC_SetPinMux(pinctrl0, pinctrl1, pinctrl2, pinctrl3, pinctrl4, 0);
    IOMUXC_SetPinConfig(pinctrl0, pinctrl1, pinctrl2, pinctrl3, pinctrl4, pinctrl5);
}

/*
 * @brief Set pad control register
 * @param asInput    use gpio as input, unless use as output
 */
static void APP_IO_SetPinConfig(uint16_t ioId, uint32_t defaultPinctrl)
{
    int index = APP_IO_GetIndex(ioId);

    /* check table is sound... */
    BUILD_BUG_ON(ARRAY_SIZE(pinFuncId) != APP_IO_NUM);
    BUILD_BUG_ON(ARRAY_SIZE(wuuPins) != APP_WUU_PINS_NUM);

    assert(index < APP_IO_NUM);

    pinctrl_set(pinFuncId[index][0], pinFuncId[index][1], pinFuncId[index][2], pinFuncId[index][3], pinFuncId[index][4],
                pinFuncId[index][5] == -1 ? defaultPinctrl : pinFuncId[index][5]);
}

static bool APP_IO_PinIsGPIO(uint16_t ioId)
{
    int index = APP_IO_GetIndex(ioId);
    assert(index < APP_IO_NUM);

    /* in imx8ulp, pinmux for GPIO is always function 1 */
    return pinFuncId[index][1] == 1;
}

static srtm_status_t APP_IO_PinctrlSet(srtm_service_t service, srtm_peercore_t core, uint16_t ioId, uint32_t pinctrl[6])
{
    int index = APP_IO_GetIndex(ioId);
    assert(index < APP_IO_NUM);

    if (pinFuncId[index][5] != -1)
    {
        PRINTF("pinctrl %x was already set\r\n", ioId);
        return SRTM_Status_Error;
    }
    if (pinFuncId[index][0] != pinctrl[0])
    {
        PRINTF("pinctrl %x first value %x did not match expected %x\r\n", ioId, pinctrl[0], pinFuncId[index][0]);
        return SRTM_Status_Error;
    }

    /* remember pinctrl and apply it */
    memcpy(pinFuncId[index], pinctrl, sizeof(pinFuncId[0]));
    APP_IO_SetPinConfig(ioId, 0);

    return SRTM_Status_Success;
}

static srtm_status_t APP_IO_OutputInit(srtm_service_t service, srtm_peercore_t core, uint16_t ioId,
                                       srtm_io_value_t ioValue)
{
    uint8_t gpioIdx = APP_GPIO_IDX(ioId);
    uint8_t pinIdx  = APP_PIN_IDX(ioId);

    assert(gpioIdx < 3U);
    assert(pinIdx < 32U);
    assert(index < APP_IO_NUM);

    if (!APP_IO_PinIsGPIO(ioId))
    {
        PRINTF("Refusing to configure non-GPIO pin %x as output (val %d)\r\n", ioId, ioValue);
        return SRTM_Status_Error;
    }

    APP_IO_SetPinConfig(ioId, IOMUXC_PCR_OBE_MASK);

    rgpio_pin_config_t config = {
        .outputLogic  = ioValue,
        .pinDirection = kRGPIO_DigitalOutput,
    };

    RGPIO_PinInit(gpios[gpioIdx], pinIdx, &config);

    return SRTM_Status_Success;
}

static srtm_status_t APP_IO_InputGet(srtm_service_t service, srtm_peercore_t core, uint16_t ioId,
                                     srtm_io_value_t *pIoValue)
{
    uint8_t gpioIdx = APP_GPIO_IDX(ioId);
    uint8_t pinIdx  = APP_PIN_IDX(ioId);

    assert(gpioIdx < 3U); /* We only support GPIOA, GPIOB and GPIOC */
    assert(pinIdx < 32U);
    assert(pIoValue);

    if (!APP_IO_PinIsGPIO(ioId))
    {
        PRINTF("Refusing to get non-GPIO pin %x value\r\n", ioId);
        return SRTM_Status_Error;
    }

    *pIoValue = RGPIO_PinRead(gpios[gpioIdx], pinIdx) ? SRTM_IoValueHigh : SRTM_IoValueLow;

    return SRTM_Status_Success;
}

static srtm_status_t APP_IO_OutputSet(srtm_service_t service, srtm_peercore_t core, uint16_t ioId,
                                      srtm_io_value_t ioValue)
{
    uint8_t gpioIdx = APP_GPIO_IDX(ioId);
    uint8_t pinIdx  = APP_PIN_IDX(ioId);

    assert(index < APP_IO_NUM);
    assert(gpioIdx < 3U); /* We only support GPIOA, GPIOB and GPIOC */
    assert(pinIdx < 32U);

    if (!APP_IO_PinIsGPIO(ioId))
    {
        PRINTF("Refusing to set non-GPIO pin %x (val %d)\r\n", ioId, ioValue);
        return SRTM_Status_Error;
    }

    RGPIO_PinWrite(gpios[gpioIdx], pinIdx, (uint8_t)ioValue);

    return SRTM_Status_Success;
}

void APP_IO_SetupWUU(uint8_t wuuIdx, wuu_external_pin_edge_detection_t wuuEdge)
{
    if (wuuIdx == 255)
        return;

    wuu_external_wakeup_pin_config_t config = {
        .event = kWUU_ExternalPinInterrupt,
        .mode  = kWUU_ExternalPinActiveAlways,
        .edge  = wuuEdge,
    };
    WUU_SetExternalWakeUpPinsConfig(WUU0, wuuIdx, &config);
}

static srtm_status_t APP_IO_ConfInput(uint16_t ioId, srtm_io_event_t event, bool wakeup)
{
    uint8_t gpioIdx                           = APP_GPIO_IDX(ioId);
    uint8_t pinIdx                            = APP_PIN_IDX(ioId);
    uint8_t wuuIdx                            = APP_IO_GetWUUPin(gpioIdx, pinIdx);
    uint8_t inputIdx                          = APP_IO_GetIndex(ioId);
    wuu_external_pin_edge_detection_t wuuEdge = kWUU_ExternalPinDisable;

    assert(gpioIdx < APP_IO_CHIPS); /* Only support GPIOA, GPIOB and GPIOC */
    assert(pinIdx < APP_IO_PINS_PER_CHIP);
    assert(inputIdx < APP_IO_NUM);
    if (wakeup && wuuIdx == 255)
    {
        PRINTF("Wakeup requested on %d/%d which has no wakeup\r\n", gpioIdx, pinIdx);
        return SRTM_Status_Error;
    }
    if (!APP_IO_PinIsGPIO(ioId))
    {
        PRINTF("Refusing to configure non-GPIO pin %x as input\r\n", ioId);
        return SRTM_Status_Error;
    }
    if (wakeup)
        PRINTF("Wakeup requested on %d/%d (WUU %d), mode %d\r\n", gpioIdx, pinIdx, wuuIdx, event);
    suspendContext.io.data[inputIdx].wakeup = wakeup;

    APP_IO_SetPinConfig(ioId, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
    /* set direction as input */
    rgpio_pin_config_t gpio_config = {
        .pinDirection = kRGPIO_DigitalInput,
    };
    RGPIO_PinInit(gpios[gpioIdx], pinIdx, &gpio_config);

    switch (event)
    {
        case SRTM_IoEventRisingEdge:
            RGPIO_SetPinInterruptConfig(gpios[gpioIdx], pinIdx, APP_GPIO_INT_SEL, kRGPIO_InterruptRisingEdge);
            wuuEdge = kWUU_ExternalPinRisingEdge;
            break;
        case SRTM_IoEventFallingEdge:
            RGPIO_SetPinInterruptConfig(gpios[gpioIdx], pinIdx, APP_GPIO_INT_SEL, kRGPIO_InterruptFallingEdge);
            wuuEdge = kWUU_ExternalPinFallingEdge;
            break;
        case SRTM_IoEventEitherEdge:
            RGPIO_SetPinInterruptConfig(gpios[gpioIdx], pinIdx, APP_GPIO_INT_SEL, kRGPIO_InterruptEitherEdge);
            wuuEdge = kWUU_ExternalPinAnyEdge;
            break;
        case SRTM_IoEventLowLevel:
            RGPIO_SetPinInterruptConfig(gpios[gpioIdx], pinIdx, APP_GPIO_INT_SEL, kRGPIO_InterruptLogicZero);
            /* WUU cannot do level, wake on falling edge */
            wuuEdge = kWUU_ExternalPinFallingEdge;
            break;
        case SRTM_IoEventHighLevel:
            RGPIO_SetPinInterruptConfig(gpios[gpioIdx], pinIdx, APP_GPIO_INT_SEL, kRGPIO_InterruptLogicOne);
            /* WUU cannot do level, wake on rising edge */
            wuuEdge = kWUU_ExternalPinRisingEdge;
            break;
        case SRTM_IoEventDisable:
            RGPIO_SetPinInterruptConfig(gpios[gpioIdx], pinIdx, APP_GPIO_INT_SEL, kRGPIO_InterruptOrDMADisabled);
            break;
        default:
            break;
    }

    if (!wakeup)
        wuuEdge = kWUU_ExternalPinDisable;
    APP_IO_SetupWUU(wuuIdx, wuuEdge);

    return SRTM_Status_Success;
}

static srtm_status_t APP_IO_InputInit(srtm_service_t service, srtm_peercore_t core, uint16_t ioId,
                                      srtm_io_event_t event, bool wakeup)
{
    return APP_IO_ConfInput(ioId, event, wakeup);
}

void EWM_IRQHandler(void)
{
    PRINTF("WATCHDOG IRQ!\r\n");
    SRTM_WdogService_NotifyPreTimeout(wdogService);
    EWM_DisableInterrupts(EWM0, kEWM_InterruptEnable);
}

static srtm_status_t wdog_enable(bool enabled, uint16_t timeout)
{

    bool is_running = EWM_GetStatusFlags(EWM0) & kEWM_RunningFlag;
    PRINTF("Watchdog %s (timeout %d)\r\n", enabled ? "start" : "stop", timeout);

    /* timeout cannot be changed for EWM, just ignore any re-enable... */
    if (is_running)
    {
        PRINTF("(ignored action on running wdog; had %d)\r\n", suspendContext.wdog.timeout);
        return SRTM_Status_Success;
        /* wait for deinit to actually be effective */
        // while (0U == ((WDOG1->CS) & WDOG_CS_RCS_MASK))
        //     ;
    }

    if (enabled)
    {
        ewm_config_t config;
        EWM_GetDefaultConfig(&config);

        config.compareHighValue = MIN(timeout / 250, 0xff);
        config.prescaler        = 250;
        /* get a warning before reset (log message) */
        config.enableInterrupt = true;

        EWM_Init(EWM0, &config);
        NVIC_EnableIRQ(EWM_IRQn);
        EWM_EnableInterrupts(EWM0, kEWM_InterruptEnable);

        /* enable PMIC WDOG_B reset */
        UPOWER_SetPmicReg(8 /* RESET_CTRL */, 0xa0 /* WDOG_B_CFG = 10b | PMIC_RST_CFG = 10b*/);
    }
    suspendContext.wdog.timeout = enabled ? timeout : 0;

    return SRTM_Status_Success;
}

static srtm_status_t wdog_ping(void)
{
    PRINTF("watchdog ping\r\n");
    EWM_Refresh(EWM0);
    return SRTM_Status_Success;
}

static void APP_SRTM_WdogSuspend(void)
{
    if (suspendContext.wdog.timeout == 0)
        return;

    /* disable PMIC WDOG_B reset */
    UPOWER_SetPmicReg(8 /* RESET_CTRL */, 0x20 /* WDOG_B_CFG = 00b | PMIC_RST_CFG = 10b*/);
    EWM_DisableInterrupts(EWM0, kEWM_InterruptEnable);
}

static void APP_SRTM_WdogResume(void)
{
    if (suspendContext.wdog.timeout == 0)
        return;

    EWM_Refresh(EWM0);
    wdog_enable(true, suspendContext.wdog.timeout);
}

static void APP_SRTM_InitWdogService(void)
{
    wdogService = SRTM_WdogService_Create(wdog_enable, wdog_ping);
    SRTM_Dispatcher_RegisterService(disp, wdogService);
}

static void APP_SRTM_DoWakeup(void *param)
{
    APP_WakeupACore();
}

static void APP_SRTM_DoWakeupCA35(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    if (!core || (core && SRTM_PeerCore_GetState(core) == SRTM_PeerCore_State_Deactivated))
    {
        APP_SRTM_DoWakeup(param1);
        APP_SRTM_StartCommunication();
    }
}

static void APP_SRTM_PollLinkup(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    if (srtmState == APP_SRTM_StateRun)
    {
        if (rpmsg_lite_is_link_up(rpmsgHandle))
        {
            srtmState = APP_SRTM_StateLinkedUp;
            xSemaphoreGive(monSig);
        }
        else
        {
            /* Start timer to poll linkup status. */
            xTimerStart(linkupTimer, portMAX_DELAY);
        }
    }
}

static void APP_RefreshS400WdgTimerCallback(TimerHandle_t xTimer)
{
    SENTINEL_Ping();
    PRINTF("\r\n %s: %d ping s400 wdg timer ok\r\n", __func__, __LINE__);
    xTimerStart(refreshS400WdgTimer, portMAX_DELAY);
}

static void APP_RestoreRegValOfMuTimerCallback(TimerHandle_t xTimer)
{
    rtdCtxRestore();
    xTimerStart(restoreRegValOfMuTimer, portMAX_DELAY);
}

static void APP_ChngModeFromActToDslForApdTimerCallback(TimerHandle_t xTimer)
{
    if (core != NULL && support_dsl_for_apd == true && AD_WillEnterMode == AD_DSL)
    {
        AD_CurrentMode   = AD_DSL;
        AD_WillEnterMode = AD_ACT;
        SRTM_PeerCore_SetState(core, SRTM_PeerCore_State_Deactivated);
        PRINTF("AD entered Deep Sleep Mode\r\n");
    }
}

static void APP_LinkupTimerCallback(TimerHandle_t xTimer)
{
    srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_PollLinkup, NULL, NULL);

    if (proc)
    {
        SRTM_Dispatcher_PostProc(disp, proc);
    }
}

static void APP_SRTM_NotifyPeerCoreReady(struct rpmsg_lite_instance *rpmsgHandle, bool ready)
{
    /* deinit and init app task(str_echo/pingpong rpmsg) in APP_SRTM_StateReboot only */
    if (rpmsgMonitor && (srtmState == APP_SRTM_StateReboot))
    {
        rpmsgMonitor(rpmsgHandle, ready, rpmsgMonitorParam);
    }
}

static void APP_SRTM_Linkup(void)
{
    srtm_channel_t chan;
    srtm_rpmsg_endpoint_config_t rpmsgConfig;

    /* Inform upower that m33 is using the ddr */
    UPOWER_SetRtdUseDdr(true);

    /* Create SRTM peer core */
    core = SRTM_PeerCore_Create(PEER_CORE_ID);
    /* Set peer core state to activated */
    SRTM_PeerCore_SetState(core, SRTM_PeerCore_State_Activated);

    /* Common RPMsg channel config */
    rpmsgConfig.localAddr   = RL_ADDR_ANY;
    rpmsgConfig.peerAddr    = RL_ADDR_ANY;
    rpmsgConfig.rpmsgHandle = rpmsgHandle;

    /* Create and add SRTM I2C channel to peer core*/
    rpmsgConfig.epName = APP_SRTM_I2C_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    /* Create and add SRTM IO channel to peer core */
    rpmsgConfig.epName = APP_SRTM_IO_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    /* Create and add SRTM ADC channel to peer core */
    rpmsgConfig.epName = APP_SRTM_ADC_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    /* Create and add SRTM PWM channel to peer core */
    rpmsgConfig.epName = APP_SRTM_PWM_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    /* Create and add SRTM RTC channel to peer core */
    rpmsgConfig.epName = APP_SRTM_RTC_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    /* Create and add SRTM Life Cycle channel to peer core */
    rpmsgConfig.epName = APP_SRTM_LFCL_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    rpmsgConfig.epName = APP_SRTM_WDOG_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    rpmsgConfig.epName = APP_SRTM_TTY_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    SRTM_Dispatcher_AddPeerCore(disp, core);
}

static void APP_SRTM_InitPeerCore(void)
{
    copyResourceTable();

    rpmsgHandle = rpmsg_lite_remote_init((void *)RPMSG_LITE_SRTM_SHMEM_BASE, RPMSG_LITE_SRTM_LINK_ID, RL_NO_FLAGS);
    assert(rpmsgHandle);

    /* save context, such as: MU0_MUA[RCR] */
    rtdCtxSave();

    APP_SRTM_NotifyPeerCoreReady(rpmsgHandle, true);

    if (rpmsg_lite_is_link_up(rpmsgHandle))
    {
        APP_SRTM_Linkup();
    }
    else
    {
        /* Start timer to poll linkup status. */
        xTimerStart(linkupTimer, portMAX_DELAY);
    }
}

static void APP_SRTM_ResetServices(void)
{
    /* When CA35 resets, we need to avoid async event to send to CA35. IO services have async events. */
    SRTM_RtcService_Reset(rtcService, core);
    SRTM_IoService_Reset(ioService, core);
    APP_I2C_ResetService();
}

static void APP_SRTM_DeinitPeerCore(void)
{
    /* Stop linkupTimer if it's started. */
    xTimerStop(linkupTimer, portMAX_DELAY);

    /* Notify application for the peer core disconnection. */
    APP_SRTM_NotifyPeerCoreReady(rpmsgHandle, false);

    if (core)
    {
        /* Need to let services know peer core is now down. */
        APP_SRTM_ResetServices();

        SRTM_Dispatcher_RemovePeerCore(disp, core);
        SRTM_PeerCore_Destroy(core);
        core = NULL;
    }

    if (rpmsgHandle)
    {
        rpmsg_lite_deinit(rpmsgHandle);
        rpmsgHandle = NULL;
    }

    /* Inform upower that m33 is not using the ddr(it's ready to reset ddr of lpavd) */
    UPOWER_SetRtdUseDdr(false);
}

static void APP_SRTM_InitIoKeyService(void)
{
    /* Enable interrupt for GPIO. */
    NVIC_SetPriority(GPIOA_INT0_IRQn, APP_GPIO_IRQ_PRIO);
    NVIC_SetPriority(GPIOA_INT1_IRQn, APP_GPIO_IRQ_PRIO);
    NVIC_SetPriority(GPIOB_INT0_IRQn, APP_GPIO_IRQ_PRIO);
    NVIC_SetPriority(GPIOB_INT1_IRQn, APP_GPIO_IRQ_PRIO);
    NVIC_SetPriority(GPIOC_INT0_IRQn, APP_GPIO_IRQ_PRIO);
    NVIC_SetPriority(GPIOC_INT1_IRQn, APP_GPIO_IRQ_PRIO);

    EnableIRQ(GPIOA_INT0_IRQn);
    EnableIRQ(GPIOA_INT1_IRQn);
    EnableIRQ(GPIOB_INT0_IRQn);
    EnableIRQ(GPIOB_INT1_IRQn);
    EnableIRQ(GPIOC_INT0_IRQn);
    EnableIRQ(GPIOC_INT1_IRQn);

    ioService = SRTM_IoService_Create(APP_IO_NUM, APP_IO_InputInit, APP_IO_OutputInit, APP_IO_InputGet,
                                      APP_IO_OutputSet, APP_IO_PinctrlSet);
    SRTM_Dispatcher_RegisterService(disp, ioService);
}

static void APP_SRTM_A35ResetHandler(void)
{
    portBASE_TYPE taskToWake = pdFALSE;

    /* disable interrupt */
    MU_DisableInterrupts(MU0_MUA, kMU_ResetAssertInterruptEnable);

    srtmState = APP_SRTM_StateReboot;

    /* Wake up monitor to reinitialize the SRTM communication with CA35 */
    if (pdPASS == xSemaphoreGiveFromISR(monSig, &taskToWake))
    {
        portYIELD_FROM_ISR(taskToWake);
    }
}

/* Make sure that XRDC has setup access permission for M Core(M Core can access registers of CMC_AD), unless M Core will
 * get hardfault(CMC_AD is belongs to Application Domain) */
static void CMC_ADClrAD_PSDORF(CMC_AD_Type *base, uint32_t flag)
{
    base->AD_PSDORF = flag; /* Write 1 to clear flag */
}

/*
 * MU Interrrupt RPMsg handler
 */
#ifdef MU0_A_IRQHandler
#undef MU0_A_IRQHandler
#endif

int32_t MU0_A_IRQHandler(void)
{
    uint32_t status = MU_GetStatusFlags(MU0_MUA);

    if (status & kMU_OtherSideEnterPowerDownInterruptFlag) /* PD/DPD mode */
    {
        SRTM_PeerCore_SetState(core, SRTM_PeerCore_State_Deactivated);

        PRINTF("AD entered PD(linux suspend to ram)/DPD(linux shutdown) mode\r\n");

        status = UPOWER_ChngPmicVoltage(PMIC_LDO3, 0);
        if (status == 0)
        {
            int vol;
            status = UPOWER_GetPmicVoltage(PMIC_LDO3, &vol);
            if (status == 0)
            {
                PRINTF("PMIC_LDO3 is %d [uV]\r\n", vol);
            }
            else
            {
                PRINTF("failed to get PMIC_LDO3 voltage\r\n");
            }
        }
        else
        {
            PRINTF("failed to set PMIC_LDO3 voltage to %u [uV]\r\n", 0);
        }

        status = UPOWER_ChngPmicVoltage(PMIC_LSW2, 0);
        if (status == 0)
        {
            int vol;
            status = UPOWER_GetPmicVoltage(PMIC_LSW2, &vol);
            if (status == 0)
            {
                PRINTF("PMIC_LSW2 is %d [uV]\r\n", vol);
            }
            else
            {
                PRINTF("failed to get PMIC_LSW2 voltage\r\n");
            }
        }
        else
        {
            PRINTF("failed to set PMIC_LSW2 voltage to 0 [uV]\r\n");
        }

        status = UPOWER_ChngPmicVoltage(PMIC_LSW4, 0);
        if (status == 0)
        {
            int vol;
            status = UPOWER_GetPmicVoltage(PMIC_LSW4, &vol);
            if (status == 0)
            {
                PRINTF("PMIC_LSW4 is %d [uV]\r\n", vol);
            }
            else
            {
                PRINTF("failed to get PMIC_LSW4 voltage\r\n");
            }
        }
        else
        {
            PRINTF("failed to set PMIC_LSW4 voltage to 0 [uV]\r\n");
        }

        MU_ClearStatusFlags(MU0_MUA, (uint32_t)kMU_OtherSideEnterPowerDownInterruptFlag);

        if (AD_WillEnterMode == AD_DPD)
        {
            AD_CurrentMode = AD_WillEnterMode; /* AD entered Deep Power Down Mode */
            NVIC_ClearPendingIRQ(CMC1_IRQn);
            EnableIRQ(CMC1_IRQn);
            /* Help A35 to setup TRDC after A35 entered deep power down moade */
            BOARD_SetTrdcAfterApdReset();
        }
        else
        {
            /* Relase A Core */
            MU_BootOtherCore(
                MU0_MUA,
                (mu_core_boot_mode_t)0); /* Delete the code after linux supported sending suspend rpmsg to M Core */
            AD_CurrentMode = AD_PD;      /* AD entered Power Down Mode */
        }
        AD_WillEnterMode = AD_ACT;

        /* make m33 core sleep */
        APP_PowerModeSwitch(LPM_PowerModeDeepSleep, kAPP_WakeupSourcePin);
    }

    return RPMsg_MU0_A_IRQHandler();
}

// clang-format off
/*
 * [AD_CurrentMode, AD_WillEnterMode] state machine:
 *       +----(uboot reset)---+
 *       |                    |
 *       |                    |
 *       v                    |
 * [AD_UNKOWN, AD_UNKOWN](A Core in uboot)<--+
 *    ^      |                               |
 *    |      |                        (linux reboot)
 *    |    (boot from uboot to linux)        |        +------------(linux resume from suspend)--+
 *    |      |                               |        |                                         |
 *    |      |                               |        v                                         |
 *    |      +--------------------->[AD_ACT, AD_UNKOWN] -----(linux suspend to mem)---->[AD_PD, AD_ACT]
 *    |                                        |
 *    |                                        |
 *    |                                  (linux poweroff)
 *    |                                        |
 *    |                                        v
 *    |                                [AD_DPD, AD_ACT]
 *    |                                        |
 *    |                                        |
 *    +---(A Core is woken by wakeup source)---+
 */   /* Note: When AD is [AD_DPD, AD_ACT],option V will not enter APP_SRTM_A35ResetHandler,execute reset logic,
       * but keep the process of boot A Core*/
// clang-format on
void CMC1_IRQHandler(void)
{
    apd_boot_cnt++;
    DisableIRQ(CMC1_IRQn);
    NVIC_ClearPendingIRQ(CMC1_IRQn);
    rtdCtxRestore();

    if ((AD_CurrentMode == AD_DPD && AD_WillEnterMode == AD_ACT && !option_v_boot_flag) ||
        (apd_boot_cnt > 1 && AD_CurrentMode == AD_UNKOWN && AD_WillEnterMode == AD_UNKOWN) ||
        (AD_CurrentMode == AD_ACT && AD_WillEnterMode == AD_UNKOWN))
    {
        APP_SRTM_A35ResetHandler();
    }
    if (AD_CurrentMode == AD_DPD && AD_WillEnterMode == AD_ACT)
    {
        AD_CurrentMode   = AD_UNKOWN;
        AD_WillEnterMode = AD_UNKOWN;
    }
    if (AD_CurrentMode == AD_PD && AD_WillEnterMode == AD_ACT)
    {
        PRINTF("\r\nAD resume from Power Down Mode\r\n");

        /* hold A core for next reboot */
        MU_HoldOtherCoreReset(MU0_MUA);
    }
}

static void APP_SRTM_InitRtcDevice(void)
{
    HAL_RtcInit(rtcHandle, 0);
    NVIC_ClearPendingIRQ(BBNSM_IRQn);
    NVIC_SetPriority(BBNSM_IRQn, APP_BBNSM_IRQ_PRIO);
    EnableIRQ(BBNSM_IRQn);
}

static void APP_SRTM_InitRtcService(void)
{
    APP_SRTM_InitRtcDevice();
    rtcAdapter = SRTM_RtcAdapter_Create(rtcHandle);
    assert(rtcAdapter);

    rtcService = SRTM_RtcService_Create(rtcAdapter);
    SRTM_Dispatcher_RegisterService(disp, rtcService);
}

static srtm_status_t APP_SRTM_LfclEventHandler(srtm_service_t service, srtm_peercore_t core, srtm_lfcl_event_t event,
                                               void *eventParam, void *userParam)
{
    switch (event)
    {
        case SRTM_Lfcl_Event_ShutdownReq: /* Notify M Core that Application Domain will enter Deep Power Down Mode */
            AD_WillEnterMode = AD_DPD;
            /* Relase A Core */
            MU_BootOtherCore(MU0_MUA, (mu_core_boot_mode_t)0);
            PRINTF("\r\nAD shutdown\r\n");
            /* Probably also stops RTC/alarm unless enabled with BBNSM_BBNSM_CTRL_RTC_EN/BBNSM_BBNSM_CTRL_TA_EN
             * We use external RTC so this is ok */
            BBNSM->BBNSM_CTRL = BBNSM_BBNSM_CTRL_DP_EN(1) | BBNSM_BBNSM_CTRL_TOSP(1); /* 0x03000000 */
            break;
        case SRTM_Lfcl_Event_RebootReq:
            PRINTF("\r\nAD is entering reboot.\r\nTriggering M33 reset.\r\n\n");
            PMIC_Reset(); /* never returns */
            break;
        case SRTM_Lfcl_Event_SuspendReq: /* Notify M Core that Application Domain will enter Power Down Mode */
            /* Save context(such as: MU0_MUA[RCR]) */
            rtdCtxSave();

            if (support_dsl_for_apd != true)
            {
                AD_WillEnterMode = AD_PD;
                PRINTF("\r\nAD will enter Power Down Mode\r\n");
            }
            else
            {
                AD_WillEnterMode = AD_DSL;
                xTimerStart(chngModeFromActToDslForApdTimer,
                            portMAX_DELAY); /* No way to check whether apd entered deep sleep mode, so start a timer to
                                               change mode from active mode to deep sleep mode for AD */
                PRINTF("\r\nAD Will enter Deep Sleep Mode\r\n");
            }

            /* Relase A Core */
            MU_BootOtherCore(MU0_MUA, (mu_core_boot_mode_t)0);
            break;
        case SRTM_Lfcl_Event_WakeupReq:
            /* If already deactivated, power on CA35, else CA35 will not power off,
               and wakeup will defer until CA35 enter Power Down */
            APP_SRTM_DoWakeupCA35(NULL, NULL, NULL);
            break;
        case SRTM_Lfcl_Event_Running: /* Notify M Core that Application Domain entered Active Mode */
            /* enable CMC1 IRQ */
            CMC_ADClrAD_PSDORF(
                CMC_AD, CMC_AD_AD_PSDORF_AD_PERIPH(
                            1)); /* need clear it, unless A Core cannot reboot after A Core suspend and resume back */
            NVIC_ClearPendingIRQ(CMC1_IRQn);
            EnableIRQ(CMC1_IRQn);
            rtdCtxRestore();
            AD_CurrentMode   = AD_ACT;
            AD_WillEnterMode = AD_UNKOWN;
            PRINTF("\r\nAD entered active mode\r\n");
            break;
        default:
            PRINTF("\r\n%s: %d unsupported event: 0x%x\r\n", __func__, __LINE__, event);
            break;
    }

    return SRTM_Status_Success;
}

static void APP_SRTM_InitLfclService(void)
{
    srtm_service_t service;

    /* Create and register Life Cycle service */
    service = SRTM_LfclService_Create();
    SRTM_LfclService_Subscribe(service, APP_SRTM_LfclEventHandler, NULL);
    SRTM_Dispatcher_RegisterService(disp, service);
}

static void APP_SRTM_InitServices(void)
{
    APP_I2C_InitService();
    APP_SRTM_InitIoKeyService();
    APP_PWM_InitService();
    APP_ADC_InitService();
    APP_SRTM_InitRtcService();
    APP_SRTM_InitLfclService();
    APP_SRTM_InitWdogService();
    APP_TTY_InitService();
}

void APP_PowerOffCA35(void)
{
    UPOWER_PowerOffSwitches((upower_ps_mask_t)(kUPOWER_PS_A35_0 | kUPOWER_PS_A35_1 | kUPOWER_PS_L2_CACHE |
                                               kUPOWER_PS_AD_NIC | kUPOWER_PS_AD_PER));

    AD_CurrentMode = AD_DPD;
}

static void APP_PowerOnCA35(void)
{
    MU_BootOtherCore(MU0_MUA, (mu_core_boot_mode_t)0);
    UPOWER_PowerOnSwitches((upower_ps_mask_t)(kUPOWER_PS_A35_0 | kUPOWER_PS_A35_1 | kUPOWER_PS_L2_CACHE |
                                              kUPOWER_PS_AD_NIC | kUPOWER_PS_AD_PER));
    vTaskDelay(pdMS_TO_TICKS(200));
}

static void process_uboot_messages(void)
{
    /* handle things from uboot */
    PRINTF("waiting message from uboot\r\n");
    MU_Init(MU0_MUA);
    MU_SetFlags(MU0_MUA, 0);
    while (true)
    {
        uint32_t command = uboot_recv();
        switch (command & 0xff)
        {
            case UBOOT_HANDSHAKE:
                PRINTF("uboot: handshake\r\n");

                /* Set Trdc config then reply OK; then do other CMC configs expected after handshake. */
                BOARD_SetTrdcGlobalConfig();

                uboot_send(0);

                /* CMC1(CMC_AD) is belongs to Application Domain, so if want to access these registers of CMC1,
                 * pls make sure that mcore can access CMC1(mcore can access CMC1 after
                 * BOARD_HandshakeWithUboot) */
                CMC_ADClrAD_PSDORF(CMC_AD, CMC_AD_AD_PSDORF_AD_PERIPH(1)); /* need clear it, unless A Core cannot reboot
                                                                              after A Core suspend and resume back */

                break;
            case UBOOT_BOOT:
                PRINTF("uboot: booting into linux\r\n");
                /* reset any service that might have been used by uboot */
                APP_SRTM_ResetServices();
                return;
            case UBOOT_RESET:
                PRINTF("uboot: reset\r\n");
                PMIC_Reset(); /* does not return */
                break;
            case UBOOT_PINCTRL:
            {
                uint32_t pinctrl[6];

                uboot_recv_many(pinctrl, sizeof(pinctrl));

                pinctrl_set(pinctrl[0], pinctrl[1], pinctrl[2], pinctrl[3], pinctrl[4], pinctrl[5]);

                uboot_send(0);
                break;
            }
            case UBOOT_I2C:
                /* handles replies */
                APP_I2C_uboot(command);
                break;
        }
    }
}

static void SRTM_MonitorTask(void *pvParameters)
{
    app_srtm_state_t state = APP_SRTM_StateShutdown;

    /* Initialize services and add to dispatcher */
    APP_SRTM_InitServices();

    /* Start SRTM dispatcher */
    SRTM_Dispatcher_Start(disp);

    /* Monitor peer core state change */
    while (true)
    {
        xSemaphoreTake(monSig, portMAX_DELAY);

        if (state == srtmState)
        {
            continue;
        }

        switch (srtmState)
        {
            case APP_SRTM_StateRun:
                assert(state == APP_SRTM_StateShutdown);
                PRINTF("Start SRTM communication\r\n");
                SRTM_Dispatcher_Stop(disp);
                /* Power On A Core when SoC is in low power boot type or option_v_boot_flag=true
                 * The purpose of using option_v_boot_flag is to avoid entering the APP_SRTM_A35ResetHandler
                 * in the CMC1_IRQHandler and interrupt the startup process during the process of starting Acore. */
                if (BOARD_IsLowPowerBootType() || option_v_boot_flag)
                {
                    DisableIRQ(CMC1_IRQn);
                    MU_Init(MU0_MUA);
                    if (option_v_boot_flag)
                    {
                        APP_WakeupACore();
                    }
                    else
                    {
                        APP_PowerOnCA35();
                    }
                }

                /* process messages from uboot, including handshake */
                process_uboot_messages();

                /* enable CMC1 interrupt after handshake with uboot(M Core cannot access CMC1 that belongs to
                 * Application Domain when Power On Reset; M Core can access CMC1 after uboot(running on A Core)
                 * enable accessing permission of CMC1 by XRDC) */
                EnableIRQ(CMC1_IRQn);

                APP_SRTM_InitPeerCore();
                SRTM_Dispatcher_Start(disp);

                NVIC_ClearPendingIRQ(CMC1_IRQn);
                EnableIRQ(CMC1_IRQn);
                option_v_boot_flag = false;
                state              = APP_SRTM_StateRun;
                break;

            case APP_SRTM_StateLinkedUp:
                if (state == APP_SRTM_StateRun)
                {
                    PRINTF("Handle Peer Core Linkup\r\n\r\n");
                    SRTM_Dispatcher_Stop(disp);
                    APP_SRTM_Linkup();
                    AD_CurrentMode   = AD_ACT;
                    AD_WillEnterMode = AD_UNKOWN;
                    SRTM_Dispatcher_Start(disp);
                }
                break;
            case APP_SRTM_StateShutdown:
                PRINTF("#### Shutdown CA35 ####\r\n");
                assert(state == APP_SRTM_StateRun);

                SRTM_Dispatcher_Stop(disp);
                /* Remove peer core from dispatcher */
                APP_SRTM_DeinitPeerCore();
                /* dispatcher can still handle proc message during peer core shutdown */
                SRTM_Dispatcher_Start(disp);

                /* Shutdown CA35 domain power */
                PRINTF("#### Power off CA35 ####\r\n");
                APP_PowerOffCA35();
                state = APP_SRTM_StateShutdown;
                break;
            case APP_SRTM_StateReboot:
                assert(state == APP_SRTM_StateRun);

                PRINTF("Peer Core Reboot: forcing reset\r\n");

                PMIC_Reset(); /* does not return */
                break;
            default:
                assert(false);
                break;
        }
    }
}

void APP_ShutdownCA35(void)
{
    srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_ControlCA35, (void *)APP_SRTM_StateShutdown, NULL);

    assert(proc);
    SRTM_Dispatcher_PostProc(disp, proc);
}

void APP_BootCA35(void)
{
    srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_ControlCA35, (void *)APP_SRTM_StateRun, NULL);

    assert(proc);
    /* Fresh power up: Need SRTM monitor to prepare communication */
    SRTM_Dispatcher_PostProc(disp, proc);
}

static void SRTM_DispatcherTask(void *pvParameters)
{
    SRTM_Dispatcher_Run(disp);
}

void APP_SRTM_Init(void)
{
    UPOWER_PowerOnMemPart(0U, (uint32_t)kUPOWER_MP1_DMA0);

    monSig = xSemaphoreCreateBinary();
    assert(monSig);

    /* Create a rtc alarm event timer to send rtc alarm event to A Core after A Core is waken by rtc alarm(avoid
     * losting rtc alarm event) */
    rtcAlarmEventTimer = xTimerCreate("rtcAlarmEventTimer", APP_MS2TICK(APP_RTC_ALM_EVT_TIMER_PERIOD_MS), pdFALSE, NULL,
                                      rtcAlarmEventTimer_Callback);
    assert(rtcAlarmEventTimer);

    /* Note: Create a task to refresh S400(sentinel) watchdog timer to keep S400 alive, the task will be removed
     * after the bug is fixed in soc A1 */
    SENTINEL_Init();
    refreshS400WdgTimer = xTimerCreate("refreshS400WdgTimer", APP_MS2TICK(APP_REFRESH_S400_WDG_TIMER_PERIOD_MS),
                                       pdFALSE, NULL, APP_RefreshS400WdgTimerCallback);
    assert(refreshS400WdgTimer);
    xTimerStart(refreshS400WdgTimer, portMAX_DELAY);

    restoreRegValOfMuTimer =
        xTimerCreate("restoreRegValOfMuTimer", APP_MS2TICK(100), pdFALSE, NULL, APP_RestoreRegValOfMuTimerCallback);
    assert(restoreRegValOfMuTimer);
    xTimerStart(restoreRegValOfMuTimer, portMAX_DELAY);

    chngModeFromActToDslForApdTimer = xTimerCreate("chngModeFromActToDslForApdTimer", APP_MS2TICK(300), pdFALSE, NULL,
                                                   APP_ChngModeFromActToDslForApdTimerCallback);
    assert(chngModeFromActToDslForApdTimer);

    linkupTimer =
        xTimerCreate("Linkup", APP_MS2TICK(APP_LINKUP_TIMER_PERIOD_MS), pdFALSE, NULL, APP_LinkupTimerCallback);
    assert(linkupTimer);

    /* Create SRTM dispatcher */
    disp = SRTM_Dispatcher_Create();

    NVIC_SetPriority(CMC1_IRQn, APP_CMC1_IRQ_PRIO);

    MU_Init(MU0_MUA);
    MU_EnableInterrupts(MU0_MUA, kMU_OtherSideEnterPowerDownInterruptEnable);
    rtdCtxSave(); /* try to save CIRE0 */

    /* hold A core for next reboot */
    MU_HoldOtherCoreReset(MU0_MUA);

    xTaskCreate(SRTM_MonitorTask, "SRTM monitor", 256U, NULL, APP_SRTM_MONITOR_TASK_PRIO, NULL);
    xTaskCreate(SRTM_DispatcherTask, "SRTM dispatcher", 512U, NULL, APP_SRTM_DISPATCHER_TASK_PRIO, NULL);
}

void APP_SRTM_StartCommunication(void)
{
    srtmState = APP_SRTM_StateRun;
    xSemaphoreGive(monSig);
}

void APP_SRTM_Suspend(void)
{
#ifdef DEBUG_SUSPEND
    PRINTF("%s\r\n", __func__);
#endif
    APP_TTY_Suspend();
    APP_SRTM_WdogSuspend();
}

void APP_SRTM_Resume(void)
{
#ifdef DEBUG_SUSPEND
    PRINTF("%s\r\n", __func__);
#endif
    APP_SRTM_WdogResume();
    APP_I2C_Resume();
    /*
     * IO has restored in APP_Resume(), so don't need init io again in here.
     */
    APP_ADC_Resume();
    APP_TTY_Resume();
    HAL_RtcInit(rtcHandle, 0);
}

void APP_SRTM_SetRpmsgMonitor(app_rpmsg_monitor_t monitor, void *param)
{
    rpmsgMonitor      = monitor;
    rpmsgMonitorParam = param;
}

void APP_SRTM_HandlePeerReboot(void)
{
    if (srtmState != APP_SRTM_StateShutdown)
    {
        srtmState = APP_SRTM_StateReboot;
        xSemaphoreGive(monSig);
    }
}

void APP_SRTM_SetIRQHandler(app_irq_handler_t handler, void *param)
{
    irqHandler      = handler;
    irqHandlerParam = param;
}

void APP_SRTM_WakeupCA35(void)
{
    srtm_procedure_t proc = SRTM_Procedure_Create(APP_SRTM_DoWakeupCA35, NULL, NULL);

    assert(proc);
    SRTM_Dispatcher_PostProc(disp, proc);
}
