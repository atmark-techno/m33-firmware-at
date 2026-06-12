/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2005 David Brownell
 * Copyright (c) 2026 Atmark Techno, Inc. All Rights Reserved.
 *
 * Based on: linux-5.10/include/linux/spi/spi.h
 */
#pragma once

#define SPI_CPHA 0x01      /* clock phase */
#define SPI_CPOL 0x02      /* clock polarity */
#define SPI_MODE_0 (0 | 0) /* (original MicroWire) */
#define SPI_MODE_1 (0 | SPI_CPHA)
#define SPI_MODE_2 (SPI_CPOL | 0)
#define SPI_MODE_3 (SPI_CPOL | SPI_CPHA)
#define SPI_MODE_X_MASK (SPI_CPOL | SPI_CPHA)
#define SPI_CS_HIGH 0x04     /* chipselect active high? */
#define SPI_LSB_FIRST 0x08   /* per-word bits-on-wire */
#define SPI_3WIRE 0x10       /* SI/SO signals shared */
#define SPI_LOOP 0x20        /* loopback mode */
#define SPI_NO_CS 0x40       /* 1 dev/bus, no chipselect */
#define SPI_READY 0x80       /* slave pulls low to pause */
#define SPI_TX_DUAL 0x100    /* transmit with 2 wires */
#define SPI_TX_QUAD 0x200    /* transmit with 4 wires */
#define SPI_RX_DUAL 0x400    /* receive with 2 wires */
#define SPI_RX_QUAD 0x800    /* receive with 4 wires */
#define SPI_CS_WORD 0x1000   /* toggle cs after each word */
#define SPI_TX_OCTAL 0x2000  /* transmit with 8 wires */
#define SPI_RX_OCTAL 0x4000  /* receive with 8 wires */
#define SPI_3WIRE_HIZ 0x8000 /* high impedance turnaround */
