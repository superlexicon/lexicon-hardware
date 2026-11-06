/**
 * @file      flrc_config.h
 *
 * @brief     Configuration constants for LR2021 FLRC hardware-burst bridge.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FLRC_CONFIG_H
#define FLRC_CONFIG_H

#include <stdint.h>

/* Default Radio Settings — must match the host's config.json so the radio
 * is on the correct frequency/sync BEFORE the CONFIG command arrives. */
#define FLRC_DEFAULT_FREQ_HZ       915000000U  /* 915 MHz US915 Sub-GHz */
#define FLRC_DEFAULT_CODING_RATE   2           /* CR_3_4 (host coding_rate=2) */
#define FLRC_DEFAULT_TX_PWR_DBM    20          /* 20 dBm */
#define FLRC_DEFAULT_SYNC_WORD     0x534C2021U /* matches host sync 0x534C2021 */

/* Maximum application payload and on-air packet sizing.
 * Capped at the RX reassembly size (BURST_REASM_PAYLOAD_MAX): the largest
 * frame any peer can accept is 24,576B, sized to fit within the nRF54L15's
 * 188KB application SRAM alongside the 2-slot TX pipeline and 2-slot host
 * RX delivery ring. */
#define FLRC_BURST_MAX_TOTAL_PAYLOAD 24576
/* On-air packet payload: 511 — the LR2021 FLRC maximum and the Semtech
 * flrcburst demo's design point (RAC_FLRC_BURST_RADIO_PAYLOAD_MAX_LENGTH).
 * Owner directive 2026-09-05: retest on the phase-C firmware. The 2026-08-25
 * 511 failure (RAC TX wedge on all nodes) ran on the pre-phase-C
 * architecture; the current firmware drives the library exactly like the
 * flrcburst example, whose 511 default is its proven configuration.
 * Coordinated change: mixed packet sizes cannot interoperate — all radios
 * flash together. Rollback: revert this constant (the 252B/2.6M hex is
 * staged as bin/zephyr_252b_2.6m.hex). */
/* Active packet payload is RUNTIME (host CONFIG v2, byte 16 selector:
 * 0 = 252B, 1 = 511B). The static buffers are sized for the maximum. */
#define FLRC_PKT_PAYLOAD_MAX         511
#define FLRC_PKT_PAYLOAD_ALT         252
#define FLRC_CONFIG_BODY_LEN         15   /* legacy host CONFIG */
#define FLRC_CONFIG_BODY_LEN_V2      17   /* + rate selector + pkt selector */
/* READY-frame protocol version, appended to every READY ('G') frame:
 *   [0x47][max_payload:u16 LE][version:u8]
 * 0 = pre-selector builds (b63114c and earlier: 3-byte READY, v2 CONFIG
 *     desyncs their command stream / wedges TX — hosts must send legacy
 *     CONFIG only).
 * 2 = f30c712 and later (runtime packet/rate selectors safe: buffer
 *     sizing fixed, TX-fault visibility, watchdog hardening).
 * The host enables CONFIG v2 selectors only on a negotiated version >= 2,
 * so a mixed old-firmware/new-host bench degrades to today's behavior
 * instead of wedging. */
#define FLRC_PROTOCOL_VERSION        2
#define FLRC_BURST_HEADER_MAGIC      0x4642 /* 'FB' */

/* Host Protocol Messages */
#define FLRC_MSG_READY      'G'
#define FLRC_MSG_TX         'T'
#define FLRC_MSG_RX         'R'
#define FLRC_MSG_CONFIG     'C'
#define FLRC_MSG_SET_WINDOW 'W'
#define FLRC_MSG_STATS      'S'
#define FLRC_MSG_TIME       'M'
#define FLRC_MSG_ERROR      'E'
#define FLRC_MSG_RESET      'X'  /* Host-requested system reboot */
#define FLRC_MSG_SCAN       'Q'  /* Host → MCU: frequency scan request */
#define FLRC_MSG_SCAN_RESULT 'N' /* MCU → host: per-channel scan result */

/* Error codes */
#define FLRC_ERR_RADIO_INIT       0x01
#define FLRC_ERR_PAYLOAD_TOO_BIG  0x02
#define FLRC_ERR_BAD_CONFIG       0x03
/* TX terminal-status errors (owner directive 2026-09-02): every accepted
 * 'T' frame gets EXACTLY ONE terminal notification — 'G' on confirmed air,
 * or an 'E' frame with one of these codes on failure. The host's transmit
 * accounting releases on either, so no path may stay silent. */
#define FLRC_ERR_TX_BUSY          0x04  /* rejected at accept time: TX queue full (frame NOT taken) */
#define FLRC_ERR_TX_FAILED        0x05  /* aborted after accept — UNCERTAIN whether any part aired */
#define FLRC_ERR_TX_DROPPED       0x06  /* dropped before air — definitively NOT sent */

#define FLRC_CONFIG_BODY_LEN      15  /* freq(4)+bw(4)+cr(1)+txpwr(1)+sync(4)+role(1) */

/* Radio role (byte 14 of CONFIG body) */
#define FLRC_ROLE_BOTH      0   /* TX + RX — single-radio mode (default) */
#define FLRC_ROLE_TX_ONLY   1   /* TX only — skips reader task / RX */
#define FLRC_ROLE_RX_ONLY   2   /* RX only — skips discovery TX / outbound TX */

/* On-air packet header (16 bytes) */
typedef struct __attribute__((packed)) {
    uint16_t magic;             /* Magic 'FB' */
    uint16_t burst_id;          /* Incrementing burst ID */
    uint16_t packet_idx;        /* 0-indexed packet sequence number */
    uint16_t total_packets;     /* Total number of packets in this burst */
    uint32_t total_payload_len; /* Total burst application payload length */
    uint32_t payload_crc32;     /* Full payload CRC32 checksum */
} flrc_packet_header_t;

#define FLRC_PKT_CHUNK_MAX          (FLRC_PKT_PAYLOAD_MAX - sizeof(flrc_packet_header_t))
#define FLRC_MAX_PACKET_CHUNK_SIZE  FLRC_PKT_CHUNK_MAX

#endif /* FLRC_CONFIG_H */
