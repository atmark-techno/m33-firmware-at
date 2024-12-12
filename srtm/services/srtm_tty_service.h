/*
 * Copyright 2017, NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __SRTM_TTY_SERVICE_H__
#define __SRTM_TTY_SERVICE_H__

#include "srtm_service.h"

/*!
 * @addtogroup srtm_service
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** @brief Switch to disable TTY service debugging messages. */
#ifndef SRTM_TTY_SERVICE_DEBUG_OFF
#define SRTM_TTY_SERVICE_DEBUG_OFF (0)
#endif

#if SRTM_TTY_SERVICE_DEBUG_OFF
#undef SRTM_DEBUG_VERBOSE_LEVEL
#define SRTM_DEBUG_VERBOSE_LEVEL SRTM_DEBUG_VERBOSE_NONE
#endif

typedef int (*srtm_tty_service_tx_t)(uint16_t len, uint8_t *buf);
typedef int (*srtm_tty_service_set_baud_t)(uint32_t baud);
typedef void (*srtm_tty_service_set_wake_t)(bool enable);

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create TTY service.
 *
 * @return SRTM service handle on success and NULL on failure.
 */
srtm_service_t SRTM_TtyService_Create(srtm_tty_service_tx_t tx, srtm_tty_service_set_baud_t setBaud,
                                      srtm_tty_service_set_wake_t setWake);

/*!
 * @brief Destroy TTY service.
 *
 * @param service SRTM service to destroy.
 */
void SRTM_TtyService_Destroy(srtm_service_t service);

/*!
 * @brief Reset TTY service.
 *  This is used to stop all TTY operations and return to initial state for corresponding core.
 *  Registered pins are kept unchanged.
 *
 * @param service SRTM service to reset.
 * @param core Identify which core is to be reset
 */
void SRTM_TtyService_Reset(srtm_service_t service, srtm_peercore_t core);

// alloc notify buffer
// store pointer to payload in arg
srtm_notification_t SRTM_TtyService_NotifyAlloc(uint8_t **buf, uint16_t *len);
// send notify buffer
srtm_status_t SRTM_TtyService_NotifySend(srtm_service_t service, srtm_notification_t notif, uint16_t len);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_TTY_SERVICE_H__ */
