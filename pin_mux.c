/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "app_gpio.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPmicI2cPins();
    BOARD_InitPmicModePins();
    BOARD_InitWdogPins();
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPmicI2cPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPmicI2cPins(void)
{ /*!< Function assigned for the core: Cortex-M33[cm33] */
    APP_GPIO_PinctrlSet(IOMUXC_PTB10_PMIC0_SDA, IOMUXC_PCR_ODE_MASK);
    APP_GPIO_PinctrlSet(IOMUXC_PTB11_PMIC0_SCL, IOMUXC_PCR_ODE_MASK);
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPmicModePins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPmicModePins(void)
{                                                     /*!< Function assigned for the core: Cortex-M33[cm33] */
    APP_GPIO_PinctrlSet(IOMUXC_PTB7_PMIC0_MODE2, 0U); // pull-down on the board
    APP_GPIO_PinctrlSet(IOMUXC_PTB8_PMIC0_MODE1, 0U); // pull-down on the board
    APP_GPIO_PinctrlSet(IOMUXC_PTB9_PMIC0_MODE0, 0U); // pull-down on the board
}

void BOARD_InitWdogPins(void)
{
    APP_GPIO_PinctrlSet(IOMUXC_PTA1_EWM0_OUT_B, IOMUXC_PCR_ODE_MASK);
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
