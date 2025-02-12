/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitConsolePins();
    BOARD_InitPmicI2cPins();
    BOARD_InitHdmiIntPins();
    BOARD_InitPmicModePins();
    BOARD_InitWdogPins();
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitConsolePins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitConsolePins(void)
{ /*!< Function assigned for the core: Cortex-M33[cm33] */
    /*
     * LPUART1 is the debug console for RTD.
     */
    IOMUXC_SetPinMux(IOMUXC_PTA22_LPUART1_TX, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTA22_LPUART1_TX, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
    IOMUXC_SetPinMux(IOMUXC_PTA23_LPUART1_RX, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTA23_LPUART1_RX, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
}

void BOARD_DeinitConsolePins(void)
{
    IOMUXC_SetPinMux(IOMUXC_PTA22_LPUART1_TX, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTA22_LPUART1_TX, 0U);
    IOMUXC_SetPinMux(IOMUXC_PTA23_LPUART1_RX, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTA23_LPUART1_RX, 0U);
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPmicI2cPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPmicI2cPins(void)
{ /*!< Function assigned for the core: Cortex-M33[cm33] */
    IOMUXC_SetPinMux(IOMUXC_PTB10_PMIC0_SDA, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTB10_PMIC0_SDA, IOMUXC_PCR_ODE_MASK);
    IOMUXC_SetPinMux(IOMUXC_PTB11_PMIC0_SCL, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTB11_PMIC0_SCL, IOMUXC_PCR_ODE_MASK);
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitHdmiIntPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitHdmiIntPins(void)
{ /*!< Function assigned for the core: Cortex-M33[cm33] */
    IOMUXC_SetPinMux(IOMUXC_PTA19_PTA19, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTA19_PTA19,
                        0U); // ogasawara preEVA: nc, ogasawara gateway: usb2422 reset_n (pull-down)
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPmicModePins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPmicModePins(void)
{ /*!< Function assigned for the core: Cortex-M33[cm33] */
    IOMUXC_SetPinMux(IOMUXC_PTB7_PMIC0_MODE2, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTB7_PMIC0_MODE2, 0U); // pull-down on the board
    IOMUXC_SetPinMux(IOMUXC_PTB8_PMIC0_MODE1, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTB8_PMIC0_MODE1, 0U); // pull-down on the board
    IOMUXC_SetPinMux(IOMUXC_PTB9_PMIC0_MODE0, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTB9_PMIC0_MODE0, 0U); // pull-down on the board
}

void BOARD_InitWdogPins(void)
{
    IOMUXC_SetPinMux(IOMUXC_PTA14_EWM0_OUT_B, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTA14_EWM0_OUT_B, IOMUXC_PCR_ODE_MASK);
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
