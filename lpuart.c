#include "fsl_lpuart.h"

/*!
 * brief Reads the receiver data register using a blocking method.
 *
 * This function polls the receiver register, waits for the receiver register full or receiver FIFO
 * has data, and reads data from the TX register up to retry_times times
 *
 * This is exactly LPUART_ReadBlocking from fsl_lpuart.c with retry_times taken out.
 *
 * param base LPUART peripheral base address.
 * param data Start address of the buffer to store the received data.
 * param length Size of the buffer.
 * retval kStatus_LPUART_RxHardwareOverrun Receiver overrun happened while receiving data.
 * retval kStatus_LPUART_NoiseError Noise error happened while receiving data.
 * retval kStatus_LPUART_FramingError Framing error happened while receiving data.
 * retval kStatus_LPUART_ParityError Parity error happened while receiving data.
 * retval kStatus_LPUART_Timeout Transmission timed out and was aborted.
 * retval kStatus_Success Successfully received all data.
 */
status_t LPUART_ReadBlockingTimes(LPUART_Type *base, uint8_t *data, size_t length, uint32_t retry_times)
{
    assert(NULL != data);

    status_t status = kStatus_Success;
    uint32_t statusFlag;
    uint8_t *dataAddress = data;

#if defined(FSL_FEATURE_LPUART_HAS_7BIT_DATA_SUPPORT) && FSL_FEATURE_LPUART_HAS_7BIT_DATA_SUPPORT
    uint32_t ctrl        = base->CTRL;
    bool isSevenDataBits = (((ctrl & LPUART_CTRL_M7_MASK) != 0U) ||
                            (((ctrl & LPUART_CTRL_M_MASK) == 0U) && ((ctrl & LPUART_CTRL_PE_MASK) != 0U)));
#endif
    uint32_t waitTimes;

    while (0U != (length--))
    {
        waitTimes = retry_times;
#if defined(FSL_FEATURE_LPUART_HAS_FIFO) && FSL_FEATURE_LPUART_HAS_FIFO
        while (0U == ((base->WATER & LPUART_WATER_RXCOUNT_MASK) >> LPUART_WATER_RXCOUNT_SHIFT))
#else
        while (0U == (base->STAT & LPUART_STAT_RDRF_MASK))
#endif
        {
            if (0U == --waitTimes)
            {
                status = kStatus_LPUART_Timeout;
                break;
            }
            statusFlag = LPUART_GetStatusFlags(base);

            if (0U != (statusFlag & (uint32_t)kLPUART_RxOverrunFlag))
            {
                /*
                 * $Branch Coverage Justification$
                 * $ref fsl_lpuart_c_ref_2$.
                 */
                status = ((kStatus_Success == LPUART_ClearStatusFlags(base, (uint32_t)kLPUART_RxOverrunFlag)) ?
                              (kStatus_LPUART_RxHardwareOverrun) :
                              (kStatus_LPUART_FlagCannotClearManually));
                /* Other error flags(FE, NF, and PF) are prevented from setting once OR is set, no need to check other
                 * error flags*/
                break;
            }

            if (0U != (statusFlag & (uint32_t)kLPUART_ParityErrorFlag))
            {
                /*
                 * $Branch Coverage Justification$
                 * $ref fsl_lpuart_c_ref_2$.
                 */
                status = ((kStatus_Success == LPUART_ClearStatusFlags(base, (uint32_t)kLPUART_ParityErrorFlag)) ?
                              (kStatus_LPUART_ParityError) :
                              (kStatus_LPUART_FlagCannotClearManually));
            }

            if (0U != (statusFlag & (uint32_t)kLPUART_FramingErrorFlag))
            {
                /*
                 * $Branch Coverage Justification$
                 * $ref fsl_lpuart_c_ref_2$.
                 */
                status = ((kStatus_Success == LPUART_ClearStatusFlags(base, (uint32_t)kLPUART_FramingErrorFlag)) ?
                              (kStatus_LPUART_FramingError) :
                              (kStatus_LPUART_FlagCannotClearManually));
            }

            if (0U != (statusFlag & (uint32_t)kLPUART_NoiseErrorFlag))
            {
                /*
                 * $Branch Coverage Justification$
                 * $ref fsl_lpuart_c_ref_2$.
                 */
                status = ((kStatus_Success == LPUART_ClearStatusFlags(base, (uint32_t)kLPUART_NoiseErrorFlag)) ?
                              (kStatus_LPUART_NoiseError) :
                              (kStatus_LPUART_FlagCannotClearManually));
            }
            if (kStatus_Success != status)
            {
                break;
            }
        }

        if (kStatus_Success == status)
        {
#if defined(FSL_FEATURE_LPUART_HAS_7BIT_DATA_SUPPORT) && FSL_FEATURE_LPUART_HAS_7BIT_DATA_SUPPORT
            if (isSevenDataBits)
            {
                *(dataAddress) = (uint8_t)(base->DATA & 0x7FU);
                dataAddress++;
            }
            else
            {
                *(dataAddress) = (uint8_t)base->DATA;
                dataAddress++;
            }
#else
            *(dataAddress) = (uint8_t)base->DATA;
            dataAddress++;
#endif
        }
        else
        {
            break;
        }
    }

    return status;
}
