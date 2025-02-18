/*
 * Copyright 2017, NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __SRTM_CAN_SERVICE_H__
#define __SRTM_CAN_SERVICE_H__

#include "srtm_service.h"
#include "can.h"
#include "can_netlink.h"

/*!
 * @addtogroup srtm_service
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** @brief Switch to disable CAN service debugging messages. */
#ifndef SRTM_CAN_SERVICE_DEBUG_OFF
#define SRTM_CAN_SERVICE_DEBUG_OFF (0)
#endif

#if SRTM_CAN_SERVICE_DEBUG_OFF
#undef SRTM_DEBUG_VERBOSE_LEVEL
#define SRTM_DEBUG_VERBOSE_LEVEL SRTM_DEBUG_VERBOSE_NONE
#endif

SRTM_PACKED_BEGIN struct _srtm_can_open_params
{
    struct can_bittiming bittiming;
    struct can_bittiming data_bittiming;

    uint32_t ctrlmode;
} SRTM_PACKED_END;
typedef struct _srtm_can_open_params srtm_can_open_params_t;

SRTM_PACKED_BEGIN struct _srtm_can_init_params
{
    uint32_t suspend_wakeup_gpio;
} SRTM_PACKED_END;
typedef struct _srtm_can_init_params srtm_can_init_params_t;

typedef int (*srtm_can_service_tx_t)(uint16_t len, uint8_t *buf);
typedef int (*srtm_can_service_open_t)(srtm_can_open_params_t *params);
typedef int (*srtm_can_service_close_t)(void);
typedef int (*srtm_can_service_restart_t)(void);
typedef int (*srtm_can_service_get_status_t)(uint32_t *status);
typedef int (*srtm_can_service_init_t)(srtm_can_init_params_t *params);
typedef int (*srtm_can_service_set_wake_t)(bool enable);

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create CAN service.
 *
 * @return SRTM service handle on success and NULL on failure.
 */
srtm_service_t SRTM_CanService_Create(srtm_can_service_tx_t tx, srtm_can_service_open_t open,
                                      srtm_can_service_close_t close, srtm_can_service_restart_t restart,
                                      srtm_can_service_get_status_t getStatus, srtm_can_service_init_t init,
                                      srtm_can_service_set_wake_t setWake);

/*!
 * @brief Destroy CAN service.
 *
 * @param service SRTM service to destroy.
 */
void SRTM_CanService_Destroy(srtm_service_t service);

/*!
 * @brief Reset CAN service.
 *  This is used to stop all CAN operations and return to initial state for corresponding core.
 *  Registered pins are kept unchanged.
 *
 * @param service SRTM service to reset.
 * @param core Identify which core is to be reset
 */
void SRTM_CanService_Reset(srtm_service_t service, srtm_peercore_t core);

// alloc notify buffer
// store pointer to payload in arg
srtm_notification_t SRTM_CanService_NotifyAlloc(struct canfd_frame **cfd);
// send notify buffer
srtm_status_t SRTM_CanService_NotifySend(srtm_service_t service, srtm_notification_t notif, uint16_t len);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_CAN_SERVICE_H__ */
