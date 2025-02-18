/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Based on: linux-5.10/include/uapi/linux/can/netlink.h
 *
 * Definitions for the CAN netlink interface
 *
 * Copyright (c) 2009 Wolfgang Grandegger <wg@grandegger.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the version 2 of the GNU General Public License
 * as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _CAN_NETLINK_H
#define _CAN_NETLINK_H

#include "fsl_common.h"

/*
 * CAN bit-timing parameters
 *
 * For further information, please read chapter "8 BIT TIMING
 * REQUIREMENTS" of the "Bosch CAN Specification version 2.0"
 * at http://www.semiconductors.bosch.de/pdf/can2spec.pdf.
 */
struct can_bittiming
{
    uint32_t bitrate;      /* Bit-rate in bits/second */
    uint32_t sample_point; /* Sample point in one-tenth of a percent */
    uint32_t tq;           /* Time quanta (TQ) in nanoseconds */
    uint32_t prop_seg;     /* Propagation segment in TQs */
    uint32_t phase_seg1;   /* Phase buffer segment 1 in TQs */
    uint32_t phase_seg2;   /* Phase buffer segment 2 in TQs */
    uint32_t sjw;          /* Synchronisation jump width in TQs */
    uint32_t brp;          /* Bit-rate prescaler */
};

/*
 * CAN hardware-dependent bit-timing constant
 *
 * Used for calculating and checking bit-timing parameters
 */
struct can_bittiming_const
{
    char name[16];      /* Name of the CAN controller hardware */
    uint32_t tseg1_min; /* Time segment 1 = prop_seg + phase_seg1 */
    uint32_t tseg1_max;
    uint32_t tseg2_min; /* Time segment 2 = phase_seg2 */
    uint32_t tseg2_max;
    uint32_t sjw_max; /* Synchronisation jump width */
    uint32_t brp_min; /* Bit-rate prescaler */
    uint32_t brp_max;
    uint32_t brp_inc;
};

/*
 * CAN clock parameters
 */
struct can_clock
{
    uint32_t freq; /* CAN system clock frequency in Hz */
};

/*
 * CAN operational and error states
 */
enum can_state
{
    CAN_STATE_ERROR_ACTIVE = 0, /* RX/TX error count < 96 */
    CAN_STATE_ERROR_WARNING,    /* RX/TX error count < 128 */
    CAN_STATE_ERROR_PASSIVE,    /* RX/TX error count < 256 */
    CAN_STATE_BUS_OFF,          /* RX/TX error count >= 256 */
    CAN_STATE_STOPPED,          /* Device is stopped */
    CAN_STATE_SLEEPING,         /* Device is sleeping */
    CAN_STATE_MAX
};

/*
 * CAN bus error counters
 */
struct can_berr_counter
{
    uint16_t txerr;
    uint16_t rxerr;
};

/*
 * CAN controller mode
 */
struct can_ctrlmode
{
    uint32_t mask;
    uint32_t flags;
};

#define CAN_CTRLMODE_LOOPBACK 0x01       /* Loopback mode */
#define CAN_CTRLMODE_LISTENONLY 0x02     /* Listen-only mode */
#define CAN_CTRLMODE_3_SAMPLES 0x04      /* Triple sampling mode */
#define CAN_CTRLMODE_ONE_SHOT 0x08       /* One-Shot mode */
#define CAN_CTRLMODE_BERR_REPORTING 0x10 /* Bus-error reporting */
#define CAN_CTRLMODE_FD 0x20             /* CAN FD mode */
#define CAN_CTRLMODE_PRESUME_ACK 0x40    /* Ignore missing CAN ACKs */
#define CAN_CTRLMODE_FD_NON_ISO 0x80     /* CAN FD in non-ISO mode */

/*
 * CAN device statistics
 */
struct can_device_stats
{
    uint32_t bus_error;        /* Bus errors */
    uint32_t error_warning;    /* Changes to error warning state */
    uint32_t error_passive;    /* Changes to error passive state */
    uint32_t bus_off;          /* Changes to bus off state */
    uint32_t arbitration_lost; /* Arbitration lost errors */
    uint32_t restarts;         /* CAN controller re-starts */
};

/*
 * CAN netlink interface
 */
enum
{
    IFLA_CAN_UNSPEC,
    IFLA_CAN_BITTIMING,
    IFLA_CAN_BITTIMING_CONST,
    IFLA_CAN_CLOCK,
    IFLA_CAN_STATE,
    IFLA_CAN_CTRLMODE,
    IFLA_CAN_RESTART_MS,
    IFLA_CAN_RESTART,
    IFLA_CAN_BERR_COUNTER,
    IFLA_CAN_DATA_BITTIMING,
    IFLA_CAN_DATA_BITTIMING_CONST,
    IFLA_CAN_TERMINATION,
    IFLA_CAN_TERMINATION_CONST,
    IFLA_CAN_BITRATE_CONST,
    IFLA_CAN_DATA_BITRATE_CONST,
    IFLA_CAN_BITRATE_MAX,
    __IFLA_CAN_MAX
};

#define IFLA_CAN_MAX (__IFLA_CAN_MAX - 1)

/* u16 termination range: 1..65535 Ohms */
#define CAN_TERMINATION_DISABLED 0

#endif /* !_CAN_NETLINK_H */
