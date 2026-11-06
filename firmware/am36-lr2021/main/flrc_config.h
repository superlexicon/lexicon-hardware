/**
 * @file      flrc_config.h
 *
 * @brief     Configuration constants for LR2021 FLRC hardware-burst bridge
 *            ported to Lierda AM36-Pico (ESP32-S3 + Semtech LR2021).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FLRC_CONFIG_H
#define FLRC_CONFIG_H

#include <stdint.h>

/* Default Radio Settings — must match the host's config.json so the radio
 * is on the correct frequency/sync BEFORE the CONFIG command arrives. */
#define FLRC_DEFAULT_FREQ_HZ       915000000U  /* 915 MHz US915 Sub-GHz (host C command overrides to 915M/916M/2.442G) */
#define FLRC_DEFAULT_CODING_RATE   2           /* CR_3_4 (host coding_rate=2) */
#define FLRC_DEFAULT_TX_PWR_DBM    20          /* 20 dBm */
#define FLRC_DEFAULT_SYNC_WORD     0x534C2021U /* matches host sync 0x534C2021 */

/* Maximum application payload and on-air packet sizing.
 * On the ESP32-S3 (512 KB SRAM + 8 MB Octal PSRAM), we support up to 65,520 B
 * (~64 KB, fitting within the 16-bit LE length field of the 'T' and 'R' host
 * frames) across 4 concurrent RX reassembly slots! */
#define FLRC_BURST_MAX_TOTAL_PAYLOAD 65024
#define FLRC_PKT_PAYLOAD_MAX         511
#define FLRC_PKT_PAYLOAD_ALT         252
#define FLRC_CONFIG_BODY_LEN         15   /* legacy host CONFIG */
#define FLRC_CONFIG_BODY_LEN_V2      17   /* + rate selector + pkt selector */
/* READY-frame protocol version, appended to every READY ('G') frame:
 *   [0x47][max_payload:u16 LE][version:u8]
 * 0 = pre-selector builds
 * 2 = f30c712 and later (24KB nRF54L15 builds)
 * 3 = Lierda AM36-Pico ESP32-S3 Option-C builds (64KB burst payload, 4 RX slots,
 *     native high-speed USB-Serial-JTAG). */
#define FLRC_PROTOCOL_VERSION        3
#define FLRC_BURST_HEADER_MAGIC      0x4642 /* 'FB' */

/* Host Protocol Messages (100% byte-compatible with SuperLexicon radio.rs) */
#define FLRC_MSG_READY      'G'
#define FLRC_MSG_TX         'T'
#define FLRC_MSG_RX         'R'
#define FLRC_MSG_CONFIG     'C'
#define FLRC_MSG_SET_WINDOW 'W'
#define FLRC_MSG_STATS      'S'
#define FLRC_MSG_TIME       'M'
#define FLRC_MSG_ERROR      'E'
#define FLRC_MSG_RESET      'X'  /* Host-requested system reboot */
#define FLRC_MSG_SCAN       'Q'  /* Host -> MCU: frequency scan request */
#define FLRC_MSG_SCAN_RESULT 'N' /* MCU -> host: per-channel scan result */

/* Error codes */
#define FLRC_ERR_RADIO_INIT       0x01
#define FLRC_ERR_PAYLOAD_TOO_BIG  0x02
#define FLRC_ERR_BAD_CONFIG       0x03
#define FLRC_ERR_TX_BUSY          0x04  /* rejected at accept time: TX queue full */
#define FLRC_ERR_TX_FAILED        0x05  /* aborted after accept */
#define FLRC_ERR_TX_DROPPED       0x06  /* dropped before air */

/* Radio role (byte 14 of CONFIG body) */
#define FLRC_ROLE_BOTH      0   /* TX + RX — single-radio mode (default) */
#define FLRC_ROLE_TX_ONLY   1   /* TX only */
#define FLRC_ROLE_RX_ONLY   2   /* RX only */

/* On-air packet header (16 bytes) */
typedef struct __attribute__((packed)) {
    uint16_t magic;             /* Magic 'FB' (0x4642) */
    uint16_t burst_id;          /* Per-burst random 16-bit ID */
    uint16_t packet_idx;        /* 0-indexed packet sequence number */
    uint16_t total_packets;     /* Total number of packets in this burst */
    uint32_t total_payload_len; /* Total burst application payload length */
    uint32_t payload_crc32;     /* Full payload CRC32 checksum */
} flrc_packet_header_t;

#define FLRC_PKT_CHUNK_MAX          (FLRC_PKT_PAYLOAD_MAX - sizeof(flrc_packet_header_t))
#define FLRC_MAX_PACKET_CHUNK_SIZE  FLRC_PKT_CHUNK_MAX

/**
 * Lierda AM36-Pico (L-LRMAM36-FANN4-PK02) Board GPIO Definitions
 * Verified from L-LRMAM36-FANN4-PK02_SCH_V01.pdf & Hardware Design Manual Rev 1.0
 */
#define AM36_PIN_LR2021_BUSY      17   /* GPIO17 -> LR2021 BUSY */
#define AM36_PIN_LR2021_NRST      38   /* GPIO38 -> LR2021 nRST (active low) */
#define AM36_PIN_LR2021_DIO7_IRQ  21   /* GPIO21 -> LR2021 DIO7 (IRQ) */
#define AM36_PIN_LR2021_SPI_CS    39   /* GPIO39 -> LR2021 SPI3_CS (NSS) */
#define AM36_PIN_LR2021_SPI_CLK   40   /* GPIO40 -> LR2021 SPI3_CLK (SCK) */
#define AM36_PIN_LR2021_SPI_MISO  41   /* GPIO41 -> LR2021 SPI3_MISO */
#define AM36_PIN_LR2021_SPI_MOSI  42   /* GPIO42 -> LR2021 SPI3_MOSI */

#define AM36_PIN_BTN_BOOT         0    /* K2: BOOT button (GPIO0, active low) */
#define AM36_PIN_BTN_KEY          4    /* K3: KEY user button (GPIO4, active low) */
#define AM36_PIN_RGB_LED          18   /* D7: WS2812 RGB status LED (GPIO18) */

#endif /* FLRC_CONFIG_H */
