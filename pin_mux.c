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
    BOARD_InitRs485Pins();
    BOARD_InitI2cPins();
    BOARD_InitPmicI2cPins();
    BOARD_InitTpmPins();
    BOARD_InitHdmiIntPins();
    BOARD_InitPmicModePins();
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

void BOARD_InitRs485Pins(void)
{
    /*
     * LPUART0 is connected to RS-485/RS-422 Transceivers(ISL83485IBZ).
     *
     *  PTA18: LPUART0_TX   : DI(Driver input)
     *  PTA15: LPUART0_RX   : RO(Receiver output)
     *  PTA16: LPUART0_CTS_B: RE_N(Receiver output enable)
     *                        This pin is not controlled and must
     *                        always be pulled down. The receiver is
     *                        controlled by the i.MX8ULP's LPUART.
     *  PTA17: LPUART0_RTS_B: DE(Driver output enable)
     */
    IOMUXC_SetPinMux(IOMUXC_PTA18_LPUART0_TX, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTA18_LPUART0_TX, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
    IOMUXC_SetPinMux(IOMUXC_PTA15_LPUART0_RX, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTA15_LPUART0_RX, IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK);
    IOMUXC_SetPinMux(IOMUXC_PTA16_PTA16, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTA16_PTA16, IOMUXC_PCR_PE_MASK);
    IOMUXC_SetPinMux(IOMUXC_PTA17_PTA17, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTA17_PTA17, IOMUXC_PCR_PE_MASK);
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitI2cPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitI2cPins(void)
{ /*!< Function assigned for the core: Cortex-M33[cm33] */
    IOMUXC_SetPinMux(IOMUXC_PTA4_LPI2C1_SCL, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTA4_LPI2C1_SCL,
                        IOMUXC_PCR_ODE_MASK | IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK); // nc
    IOMUXC_SetPinMux(IOMUXC_PTA5_LPI2C1_SDA, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTA5_LPI2C1_SDA,
                        IOMUXC_PCR_ODE_MASK | IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK); // nc
    IOMUXC_SetPinMux(IOMUXC_PTA8_LPI2C0_SCL, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTA8_LPI2C0_SCL,
                        IOMUXC_PCR_ODE_MASK | IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK); // nc
    IOMUXC_SetPinMux(IOMUXC_PTA9_LPI2C0_SDA, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTA9_LPI2C0_SDA,
                        IOMUXC_PCR_ODE_MASK | IOMUXC_PCR_PE_MASK | IOMUXC_PCR_PS_MASK); // nc
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

    IOMUXC_SetPinMux(IOMUXC_PTB4_FXIO0_D20, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTB4_FXIO0_D20, IOMUXC_PCR_SRE_MASK);
    IOMUXC_SetPinMux(IOMUXC_PTB5_FXIO0_D21, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTB5_FXIO0_D21, IOMUXC_PCR_SRE_MASK);
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitTpmPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitTpmPins(void)
{ /*!< Function assigned for the core: Cortex-M33[cm33] */
    IOMUXC_SetPinMux(IOMUXC_PTA3_TPM0_CH2, 0U);
    IOMUXC_SetPinConfig(IOMUXC_PTA3_TPM0_CH2,
                        IOMUXC_PCR_DSE_MASK); // nc
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

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
