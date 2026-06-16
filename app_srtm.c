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
#include "custom.h"

#include "srtm_dispatcher.h"
#include "srtm_peercore.h"
#include "srtm_message.h"
#include "srtm_rpmsg_endpoint.h"

#include "srtm_lfcl_service.h"
#include "srtm_wdog_service.h"

#include "app_can.h"
#include "app_gpio.h"
#include "app_srtm.h"
#include "app_srtm_internal.h"
#include "app_spi.h"
#include "app_pwm.h"
#include "app_tty.h"
#include "app_uboot.h"
#include "app_rtc.h"
#include "board.h"
#include "build_bug.h"
#include "debug_console.h"
#include "fsl_bbnsm.h"
#include "fsl_mu.h"
#include "fsl_reset.h"
#include "fsl_sentinel.h"
#include "fsl_upower.h"
#include "pin_mux.h"
#include "printf.h"
#include "rsc_table.h"
#include "errno.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile app_srtm_state_t srtmState;
bool option_v_boot_flag          = false;
static bool need_reset_peer_core = false;
bool flexio_used;
static bool uboot_done;

/* For CMC1_IRQHandler */
static int64_t apd_boot_cnt = 0; /* it's cold boot when apd_boot_cnt(Application Domain, A Core) == 1 */

srtm_dispatcher_t disp;
static srtm_peercore_t core;
static SemaphoreHandle_t monSig;
static struct rpmsg_lite_instance *rpmsgHandle;
static app_rpmsg_monitor_t rpmsgMonitor;
static void *rpmsgMonitorParam;
static TimerHandle_t linkupTimer;
static TimerHandle_t abortSuspendTimer;
static TimerHandle_t refreshS400WdgTimer;
static TimerHandle_t restoreRegValOfMuTimer; /* use the timer to restore register's value of mu(To make sure that
                                                register's value of mu is restored if cmc1 interrupt is not comming) */

static app_irq_handler_t irqHandler;
static void *irqHandlerParam;

lpm_ad_power_mode_e AD_CurrentMode   = AD_UNKOWN;
lpm_ad_power_mode_e AD_WillEnterMode = AD_UNKOWN;

static MU_Type mu0_mua;
/*******************************************************************************
 * Code
 ******************************************************************************/
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

    UPOWER_PowerOnADInPDMode();
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

static void APP_AbortSuspendCallback(TimerHandle_t xTimer)
{
    /* no actual suspend in 3s -- abort sleep */
    APP_SRTM_WakeupCA35();
    APP_SRTM_LateResume();
    PRINTF("A core suspend aborted as it did not poweroff\r\n");
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

    rpmsgConfig.epName = APP_SRTM_CAN_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    rpmsgConfig.epName = APP_SRTM_SPI_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    /* Create and add SRTM AUDIO channel to peer core*/
    rpmsgConfig.epName = APP_SRTM_AUDIO_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    /* Create and add SRTM DAC channel to peer core */
    rpmsgConfig.epName = APP_SRTM_DAC_CHANNEL_NAME;
    chan               = SRTM_RPMsgEndpoint_Create(&rpmsgConfig);
    SRTM_PeerCore_AddChannel(core, chan);

    /* Create and add SRTM RTC channel to peer core */
    rpmsgConfig.epName = APP_SRTM_RTC_CHANNEL_NAME;
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
    APP_GPIO_ResetService(core);
    APP_I2C_ResetService();
    APP_WDOG_ResetLog();
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
        // cancel wakeup-if-missed task
        xTimerStop(abortSuspendTimer, 0);

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
        APP_SleepWithLinux();
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

static void BBNSM_Poweroff(void)
{
    /* Probably also stops RTC/alarm unless enabled with BBNSM_BBNSM_CTRL_RTC_EN/BBNSM_BBNSM_CTRL_TA_EN
     * We use external RTC so this is ok */
    BBNSM->BBNSM_CTRL = BBNSM_BBNSM_CTRL_DP_EN(1) | BBNSM_BBNSM_CTRL_TOSP(1); /* 0x03000000 */
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
            BBNSM_Poweroff();
            break;
        case SRTM_Lfcl_Event_RebootReq:
            PRINTF("\r\nAD is entering reboot.\r\nTriggering M33 reset.\r\n\n");
            PMIC_Reset(); /* never returns */
            break;
        case SRTM_Lfcl_Event_SuspendReq: /* Notify M Core that Application Domain will enter Power Down Mode */
            /* Save context(such as: MU0_MUA[RCR]) */
            rtdCtxSave();

            AD_WillEnterMode = AD_PD;
            PRINTF("\r\nAD will enter Power Down Mode\r\n");

            /* Avoid further messages to A35 */
            APP_SRTM_EarlySuspend();

            /* Relase A Core */
            MU_BootOtherCore(MU0_MUA, (mu_core_boot_mode_t)0);

            /* mark a core for wakeup if we never receive MU powerdown irq event */
            xTimerStart(abortSuspendTimer, portMAX_DELAY);
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
            PRINTF("\r\n%s: %d unsupported event: %#x\r\n", __func__, __LINE__, event);
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
    APP_GPIO_InitService();
    APP_PWM_InitService();
    APP_ADC_InitService();
    APP_SRTM_InitLfclService();
    APP_WDOG_InitService();
    APP_TTY_InitService();
    APP_CAN_InitService();
    APP_SPI_InitService();
    APP_AUDIO_InitService();
    APP_DAC_InitService();
    APP_RTC_InitService();
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

void hardfault_process_uboot_messages(void)
{
    /* This only handles necessary lifecycle messages to rollback if we hardfaulted early on.
     * If we already booted then skip this. */
    if (uboot_done)
        return;
    DebugConsole_Emergency("Handling hardfault uboot messages...\r\n");
    MU_Init(MU0_MUA);
    MU_SetFlags(MU0_MUA, 0);
    while (true)
    {
        uint32_t command = uboot_recv();
        switch (command & 0xff)
        {
            case UBOOT_HANDSHAKE:
                DebugConsole_Emergency("hardfault uboot handshake\r\n");
                /* need this to relocate */
                BOARD_SetTrdcGlobalConfig();
                uboot_send(EFAULT);
                CMC_ADClrAD_PSDORF(CMC_AD, CMC_AD_AD_PSDORF_AD_PERIPH(1));
                break;
            case UBOOT_BOOT:
                uboot_done = true;
                return;
            case UBOOT_RESET:
                DebugConsole_Emergency("hardfault uboot reset\r\n");
                /* does not return */
                PMIC_Reset();
                break;
            case UBOOT_POWEROFF:
                DebugConsole_Emergency("hardfault uboot poweroff\r\n");
                /* does not return */
                BBNSM_Poweroff();
                break;
            /* need these to avoid processing next values as commands... */
            case UBOOT_PINCTRL:
            {
                uint32_t pinctrl[6];
                uboot_recv_many(pinctrl, sizeof(pinctrl));

                uboot_send(EFAULT);
                break;
            }
            case UBOOT_I2C:
                /* must be init, ignore 3 values */
                uboot_recv();
                uboot_recv();
                uboot_recv();
                uboot_send(EFAULT);
                break;
            case UBOOT_WDOG:
                /* init, ignore 1 value */
                uboot_recv();
                uboot_send(EFAULT);
                break;
            /* these do not have trailing data so could be ignored, but this avoids waiting for timeout */
            case UBOOT_PING:
            case UBOOT_DEBUG_CONSOLE:
                uboot_send(EFAULT);
                break;
        }
    }
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
            // for commands in spl we need a noop with a longer timeout first as initial handshake.
            case UBOOT_PING:
                uboot_send(0);
                break;
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
                uboot_done = true;
                /* reset any service that might have been used by uboot */
                APP_SRTM_ResetServices();
                return;
            case UBOOT_RESET:
                PRINTF("uboot: reset\r\n");
                PMIC_Reset(); /* does not return */
                break;
            case UBOOT_POWEROFF:
                PRINTF("uboot: poweroff\r\n");
                BBNSM_Poweroff(); /* does not return */
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
            case UBOOT_DEBUG_CONSOLE:
                uboot_send(DebugConsole_uboot(command));
                break;
            case UBOOT_WDOG:
                /* handles replies */
                APP_WDOG_uboot(command);
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

    abortSuspendTimer = xTimerCreate("abortSuspendTimer", APP_MS2TICK(3000), pdFALSE, NULL, APP_AbortSuspendCallback);
    assert(abortSuspendTimer);

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

    /* Initializing DSP core */
    BOARD_InitFusion();

    xTaskCreate(SRTM_MonitorTask, "SRTM monitor", 256U, NULL, APP_SRTM_MONITOR_TASK_PRIO, NULL);
    xTaskCreate(SRTM_DispatcherTask, "SRTM dispatcher", 512U, NULL, APP_SRTM_DISPATCHER_TASK_PRIO, NULL);
}

void APP_SRTM_StartCommunication(void)
{
    srtmState = APP_SRTM_StateRun;
    xSemaphoreGive(monSig);
}

void APP_SRTM_EarlySuspend(void)
{
#ifdef DEBUG_SUSPEND
    PRINTF("%s\r\n", __func__);
#endif
    custom_early_suspend();
    APP_TTY_Suspend();
    APP_CAN_Suspend();
}

void APP_SRTM_Suspend(void)
{
#ifdef DEBUG_SUSPEND
    PRINTF("%s\r\n", __func__);
#endif
    custom_suspend();
    APP_WDOG_Suspend();
    BOARD_DeinitFusion();
}

void APP_SRTM_LateResume(void)
{
#ifdef DEBUG_SUSPEND
    PRINTF("%s\r\n", __func__);
#endif
    APP_TTY_Resume();
    APP_CAN_Resume();
    APP_SPI_Resume();
    APP_PWM_Resume();
    custom_late_resume();
}

void APP_SRTM_Resume(void)
{
#ifdef DEBUG_SUSPEND
    PRINTF("%s\r\n", __func__);
#endif
    BOARD_InitFusion();
    /* Fusion has been initialized so TRDC must be reconfigured */
    BOARD_SetTrdcGlobalConfig();

    custom_resume();
    APP_WDOG_Resume();
    APP_I2C_Resume();
    /*
     * IO has restored in APP_Resume(), so don't need init io again in here.
     */
    APP_ADC_Resume();
    APP_SRTM_LateResume();
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
