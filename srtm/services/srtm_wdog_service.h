/*
 * Copyright 2017, NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __SRTM_WDOG_SERVICE_H__
#define __SRTM_WDOG_SERVICE_H__

#include "srtm_service.h"

/*!
 * @addtogroup srtm_service
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** @brief Switch to disable WDOG service debugging messages. */
#ifndef SRTM_WDOG_SERVICE_DEBUG_OFF
#define SRTM_WDOG_SERVICE_DEBUG_OFF (0)
#endif

#if SRTM_WDOG_SERVICE_DEBUG_OFF
#undef SRTM_DEBUG_VERBOSE_LEVEL
#define SRTM_DEBUG_VERBOSE_LEVEL SRTM_DEBUG_VERBOSE_NONE
#endif

typedef srtm_status_t (*srtm_wdog_service_enable_t)(bool enabled, uint16_t timeout);
typedef srtm_status_t (*srtm_wdog_service_ping_t)(void);
// typedef srtm_status_t (*srtm_wdog_service_disable_t)(void);

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create WDOG service.
 *
 * @return SRTM service handle on success and NULL on failure.
 */
srtm_service_t SRTM_WdogService_Create(srtm_wdog_service_enable_t enableCb, srtm_wdog_service_ping_t pingCb);

/*!
 * @brief Notify WDOG service of imminent timeout
 */
srtm_status_t SRTM_WdogService_NotifyPreTimeout(srtm_service_t service);

/*!
 * @brief Destroy WDOG service.
 *
 * @param service SRTM service to destroy.
 */
void SRTM_WdogService_Destroy(srtm_service_t service);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_WDOG_SERVICE_H__ */
