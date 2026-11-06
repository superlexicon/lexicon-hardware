/**
 * @file      main.c
 *
 * @brief     LR2021 USB-Serial-JTAG <-> FLRC hardware-burst bridge firmware
 *            Ported to ESP-IDF for Lierda AM36-Pico (ESP32-S3 + Semtech LR2021).
 *
 *            100% byte-identical host serial contract ('T'/'C'/'W'/'S'/'X'/'Q'
 *            in, 'G'/'R'/'E'/'S'/'D' out, 68-byte stats layout) with
 *            lr2021-flrc-firmware/src/main.c, upgraded with:
 *              1. Dual-core ESP32-S3 execution (Core 1 = smtc_rac_run_engine,
 *                 Core 0 = high-speed native USB-Serial-JTAG RX/TX).
 *              2. 64 KB max burst payload (FLRC_BURST_MAX_TOTAL_PAYLOAD = 65024)
 *                 and 4 concurrent RX reassembly slots in 8 MB Octal PSRAM so
 *                 overlapping validator bursts never evict each other.
 *              3. 8-slot host RX delivery ring (HOST_TX_QUEUE_DEPTH = 8).
 *              4. Onboard D7 WS2812 RGB LED status indication & GPIO38 deep reset.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "led_strip.h"

#include "flrc_config.h"
#include "flrc_burst_protocol.h"
#include "am36_usb_io.h"

/* Lierda ESP-IDF port of Semtech smtc_rac_lib / LR2021 driver */
#include "LiotLr2021.h"
#include "smtc_rac.h"
#include "lr20xx_radio_flrc.h"
#include "smtc_rac_api.h"
#include "smtc_sw_platform_helper.h"
#include "smtc_modem_hal.h"
#include "radio_planner_types.h"

static const char *TAG = "lr2021_am36";

/* Little-endian helpers matching Zephyr byteorder.h */
static inline uint16_t sys_le16_to_cpu(uint16_t v) { return v; }
static inline uint32_t sys_le32_to_cpu(uint32_t v) { return v; }
static inline uint16_t sys_cpu_to_le16(uint16_t v) { return v; }
static inline uint32_t sys_cpu_to_le32(uint32_t v) { return v; }

static inline uint32_t sys_get_le32(const uint8_t *src)
{
    return (uint32_t)src[0] | ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) | ((uint32_t)src[3] << 24);
}

static inline void sys_put_le32(uint32_t val, uint8_t *dst)
{
    dst[0] = (uint8_t)(val & 0xFFU);
    dst[1] = (uint8_t)((val >> 8) & 0xFFU);
    dst[2] = (uint8_t)((val >> 16) & 0xFFU);
    dst[3] = (uint8_t)((val >> 24) & 0xFFU);
}

static inline uint32_t k_uptime_get_32(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* Default sync words (match the Semtech burst sample). */
static const uint8_t default_syncword_1[4] = { 0x90, 0x56, 0x34, 0x12 };
static const uint8_t default_syncword_2[4] = { 0x00, 0x00, 0x00, 0x00 };
static const uint8_t default_syncword_3[4] = { 0x00, 0x00, 0x00, 0x00 };

/* ---------------------------------------------------------------------------
 * Onboard RGB Status LED (D7 / WS2812 on GPIO18)
 * ------------------------------------------------------------------------- */
static led_strip_handle_t s_led_strip = NULL;

static void am36_led_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = AM36_PIN_RGB_LED,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    if (led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip) == ESP_OK) {
        led_strip_set_pixel(s_led_strip, 0, 0, 8, 24);
        led_strip_refresh(s_led_strip);
    }
}

/* ---------------------------------------------------------------------------
 * Boot-time deep reset: hold the chip in NRST (GPIO38) for 20ms so every boot
 * starts with a clean radio sequencer.
 * ------------------------------------------------------------------------- */
static void lr2021_deep_reset(void)
{
    gpio_config_t rst_cfg = {
        .pin_bit_mask = (1ULL << AM36_PIN_LR2021_NRST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&rst_cfg);
    gpio_set_level(AM36_PIN_LR2021_NRST, 0); /* NRST asserted (active low) */
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(AM36_PIN_LR2021_NRST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

/* Globals */
static uint8_t              g_radio_access_id;
static smtc_rac_context_t*  g_rac_ctx;
static bool                 g_radio_ok = false;

static struct
{
    uint32_t freq_hz;
    uint8_t  cr_host;
    int8_t   tx_pwr_dbm;
    uint32_t sync_word;
    uint8_t  role;          /* FLRC_ROLE_BOTH / TX_ONLY / RX_ONLY */
    uint8_t  rate_sel;      /* 0 = 1.3 Mbps, 1 = 2.6 Mbps */
    uint8_t  payload_sel;   /* 0 = 252B, 1 = 511B */
} g_cfg = {
    .freq_hz     = FLRC_DEFAULT_FREQ_HZ,
    .cr_host     = FLRC_DEFAULT_CODING_RATE,
    .tx_pwr_dbm  = FLRC_DEFAULT_TX_PWR_DBM,
    .sync_word   = FLRC_DEFAULT_SYNC_WORD,
    .role        = FLRC_ROLE_BOTH,
    .rate_sel    = 1,
    .payload_sel = 0,
};

static uint16_t g_pkt_payload = FLRC_PKT_PAYLOAD_ALT;
static uint16_t g_pkt_chunk   = FLRC_PKT_PAYLOAD_ALT - sizeof(flrc_packet_header_t);

static void cfg_apply_selectors(void)
{
    g_pkt_payload = (g_cfg.payload_sel == 1) ? FLRC_PKT_PAYLOAD_MAX : FLRC_PKT_PAYLOAD_ALT;
    g_pkt_chunk   = g_pkt_payload - sizeof(flrc_packet_header_t);
}

static ral_flrc_raw_bit_rate_t cfg_raw_bit_rate(void)
{
    return (g_cfg.rate_sel == 1) ? RAL_FLRC_RAW_BIT_RATE_2_600_MBPS
                                 : RAL_FLRC_RAW_BIT_RATE_1_300_MBPS;
}

typedef enum {
    BURST_RX,
    BURST_TX_ABORTING,
    BURST_TX,
} burst_mode_t;
static volatile burst_mode_t g_burst_mode         = BURST_RX;
static volatile bool         g_restart_rx_pending = false;
static volatile bool         g_tx_rearm_pending   = false;
static volatile bool         g_tx_airing          = false;
static uint32_t              g_tx_started_ms      = 0;
static uint32_t              g_tx_requests        = 0;
static uint32_t              g_tx_requests_expected = 0;
#define TX_INFLIGHT_TIMEOUT_MS 20000
static uint32_t              g_abort_since_ms     = 0;

static uint32_t g_slot_offset_ms = 100;
static uint32_t g_slot_period_ms = 1000;

/* TX: pending (airing) + staging (queued) allocated in PSRAM */
static uint8_t *g_pending_tx_buf   = NULL;
static uint32_t g_pending_tx_len   = 0;
static bool     g_pending_tx_valid = false;
static uint8_t *g_staging_tx_buf   = NULL;
static uint32_t g_staging_tx_len   = 0;
static bool     g_staging_tx_valid = false;
static uint32_t g_staging_overwrites = 0;

static uint16_t g_pending_burst_id = 0;
static uint16_t g_staging_burst_id = 0;

static void promote_staged_tx_if_ready(void)
{
    if (!g_pending_tx_valid && g_staging_tx_valid) {
        memcpy(g_pending_tx_buf, g_staging_tx_buf, g_staging_tx_len);
        g_pending_tx_len   = g_staging_tx_len;
        g_pending_burst_id = g_staging_burst_id;
        g_pending_tx_valid = true;
        g_staging_tx_valid = false;
        g_staging_tx_len   = 0;
    }
}

#define MAX_AIR_PACKETS ((FLRC_BURST_MAX_TOTAL_PAYLOAD + FLRC_PKT_PAYLOAD_ALT - sizeof(flrc_packet_header_t) - 1) / (FLRC_PKT_PAYLOAD_ALT - sizeof(flrc_packet_header_t)))
static uint8_t  g_tx_fifo_buf[2][FLRC_PKT_PAYLOAD_MAX];
static uint32_t g_tx_payload_crc = 0;
static uint16_t g_tx_total_packets = 0;
static uint32_t g_tx_air_total_len = 0;
static uint32_t g_tx_bytes_sent = 0;
static uint16_t g_tx_fifo_len[2];
static uint16_t g_tx_next_buf_idx = 0;

/* RX: 4 concurrent burst reassembly slots in PSRAM so overlapping validator
 * bursts never evict each other (eliminating burst_id_conflicts). */
#define BURST_REASM_SLOTS       4
#define BURST_REASM_PAYLOAD_MAX FLRC_BURST_MAX_TOTAL_PAYLOAD
#define RX_SLOT_STALE_MS        2000
#define RX_MASK_WORDS           ((MAX_AIR_PACKETS + 63) / 64)

typedef struct {
    bool     active;
    uint16_t burst_id;
    uint16_t total_packets;
    uint16_t received_count;
    uint32_t total_payload_len;
    uint32_t payload_crc32;
    uint64_t received_mask[RX_MASK_WORDS];
    uint32_t start_ms;
    int16_t  last_rssi;
    uint8_t  reasm_buf[BURST_REASM_PAYLOAD_MAX];
} rx_reasm_slot_t;

static rx_reasm_slot_t *g_rx_slots = NULL;

/* Single packet RX buffer handed to the RAC */
static uint8_t g_rac_rx_pkt_buf[FLRC_PKT_PAYLOAD_MAX];

static lr20xx_radio_flrc_rx_stats_t g_hw_rx_stats = { 0 };

/* Stats — the 68-byte layout the host parses; keep byte offsets identical. */
static struct
{
    uint32_t bursts_tx;
    uint32_t bursts_rx;
    uint32_t packets_tx;
    uint32_t packets_rx;
    uint32_t crc_errors;
    uint32_t host_tx_dropped;
    uint32_t burst_id_conflicts;
} g_stats = { 0 };

#define CTRL_TX_QUEUE_DEPTH 8
#define CTRL_TX_SLOT_BYTES  80
static uint8_t  g_ctrl_tx_buf[CTRL_TX_QUEUE_DEPTH][CTRL_TX_SLOT_BYTES];
static uint16_t g_ctrl_tx_len[CTRL_TX_QUEUE_DEPTH];
static volatile uint8_t g_ctrl_tx_write_idx = 0;
static volatile uint8_t g_ctrl_tx_read_idx  = 0;

/* Expanded to 8 slots in PSRAM on ESP32-S3 */
#define HOST_TX_QUEUE_DEPTH 8
#define HOST_TX_SLOT_BYTES  (FLRC_BURST_MAX_TOTAL_PAYLOAD + 16)
static uint8_t  (*g_host_tx_buf)[HOST_TX_SLOT_BYTES] = NULL;
static uint16_t g_host_tx_len[HOST_TX_QUEUE_DEPTH];
static volatile uint8_t g_host_tx_write_idx = 0;
static volatile uint8_t g_host_tx_read_idx  = 0;
static SemaphoreHandle_t g_host_tx_sem  = NULL;
static SemaphoreHandle_t g_dbg_lock     = NULL;

#define DBG_RING_DEPTH 128
static uint8_t  g_dbg_ring[DBG_RING_DEPTH][9];
static uint16_t g_dbg_head = 0, g_dbg_tail = 0;

static volatile uint32_t g_tx_done_ms = 0;

/* SCAN MODE (roaming) */
#define SCAN_MAX_CHANNELS 16
static volatile bool     g_scan_active     = false;
static uint32_t          g_scan_freqs[SCAN_MAX_CHANNELS];
static uint32_t          g_scan_dwell_ms   = 300;
static uint32_t          g_scan_dwell_until = 0;
static uint8_t           g_scan_count      = 0;
static volatile uint8_t  g_scan_idx        = 0;
static volatile uint16_t g_scan_frames     = 0;
static volatile uint16_t g_scan_consensus  = 0;
static volatile int8_t   g_scan_best_rssi  = -128;
static volatile uint32_t g_main_loop_heartbeat_ms = 0;

static volatile uint32_t g_engine_pass_count = 0;
static volatile uint32_t g_tx_done_pass      = 0;
#define MAIN_LOOP_WATCHDOG_MS 5000

static void rac_post_callback(rp_status_t status);

static int usb_tx(const uint8_t *data, uint16_t len)
{
    am36_usb_io_write_bytes(data, len);
    return 0;
}

static void send_debug_event(uint8_t event_id, uint32_t val)
{
    uint8_t frame[9] = { 'D', 6, 0, 0, 0, 0, 0, 0, 0 };
    frame[3] = event_id;
    sys_put_le32(val, &frame[4]);
    xSemaphoreTake(g_dbg_lock, portMAX_DELAY);
    memcpy(g_dbg_ring[g_dbg_tail], frame, 9);
    g_dbg_tail = (uint16_t)((g_dbg_tail + 1) % DBG_RING_DEPTH);
    if (g_dbg_tail == g_dbg_head) {
        g_dbg_head = (uint16_t)((g_dbg_head + 1) % DBG_RING_DEPTH);
    }
    xSemaphoreGive(g_dbg_lock);
    xSemaphoreGive(g_host_tx_sem);
}

void smtc_rac_diag_event(uint16_t event, uint32_t val_ms)
{
    send_debug_event((uint8_t)event, val_ms);
}

static void queue_ctrl_tx(const uint8_t *seg1, uint16_t len1,
                          const uint8_t *seg2, uint16_t len2)
{
    uint16_t total = (uint16_t)(len1 + len2);
    if (total > CTRL_TX_SLOT_BYTES ||
        (uint8_t)(g_ctrl_tx_write_idx - g_ctrl_tx_read_idx) >= CTRL_TX_QUEUE_DEPTH) {
        g_stats.host_tx_dropped++;
        return;
    }

    uint8_t  slot = g_ctrl_tx_write_idx % CTRL_TX_QUEUE_DEPTH;
    uint8_t *dst  = g_ctrl_tx_buf[slot];
    uint16_t off  = 0;
    if (len1) { memcpy(dst + off, seg1, len1); off += len1; }
    if (len2) { memcpy(dst + off, seg2, len2); off += len2; }
    g_ctrl_tx_len[slot] = off;
    __sync_synchronize();
    g_ctrl_tx_write_idx++;
    xSemaphoreGive(g_host_tx_sem);
}

static void queue_host_tx(const uint8_t *seg1, uint16_t len1,
                          const uint8_t *seg2, uint16_t len2,
                          const uint8_t *seg3, uint16_t len3)
{
    uint32_t total = (uint32_t)len1 + (uint32_t)len2 + (uint32_t)len3;
    if (total > HOST_TX_SLOT_BYTES ||
        (uint8_t)(g_host_tx_write_idx - g_host_tx_read_idx) >= HOST_TX_QUEUE_DEPTH) {
        g_stats.host_tx_dropped++;
        return;
    }

    uint8_t  slot = g_host_tx_write_idx % HOST_TX_QUEUE_DEPTH;
    uint8_t *dst  = g_host_tx_buf[slot];
    uint16_t off  = 0;
    if (len1) { memcpy(dst + off, seg1, len1); off += len1; }
    if (len2) { memcpy(dst + off, seg2, len2); off += len2; }
    if (len3) { memcpy(dst + off, seg3, len3); off += len3; }
    g_host_tx_len[slot] = off;
    __sync_synchronize();
    g_host_tx_write_idx++;
    xSemaphoreGive(g_host_tx_sem);
}

static void send_ready(void)
{
    uint8_t pkt[4];
    pkt[0] = FLRC_MSG_READY;
    pkt[1] = (uint8_t)(g_pkt_payload & 0xFF);
    pkt[2] = (uint8_t)((g_pkt_payload >> 8) & 0xFF);
    pkt[3] = FLRC_PROTOCOL_VERSION;
    queue_ctrl_tx(pkt, sizeof(pkt), NULL, 0);
}

static void send_error(uint8_t code, const char *msg)
{
    uint8_t msg_len = msg ? (uint8_t)strlen(msg) : 0;
    uint8_t header[3] = { FLRC_MSG_ERROR, code, msg_len };
    queue_ctrl_tx(header, 3, (const uint8_t *)msg, msg_len);
}

static void send_rx_payload(const uint8_t *payload, uint16_t len, int16_t rssi)
{
    uint8_t header[3] = {
        FLRC_MSG_RX,
        (uint8_t)(len & 0xFF),
        (uint8_t)((len >> 8) & 0xFF)
    };
    uint8_t rssi_buf[2] = {
        (uint8_t)(rssi & 0xFF),
        (uint8_t)((rssi >> 8) & 0xFF)
    };
    queue_host_tx(header, 3, payload, (len > 0) ? len : 0, rssi_buf, 2);
}

static uint32_t g_rx_ring_dropped      = 0;
static uint32_t g_parser_frames_killed = 0;

static void send_stats_packet(void)
{
    uint8_t header[3] = { FLRC_MSG_STATS, 68, 0 };
    uint8_t stats_bytes[68] = { 0 };
    sys_put_le32(g_stats.bursts_tx, &stats_bytes[0]);
    sys_put_le32(g_stats.bursts_rx, &stats_bytes[4]);
    sys_put_le32(g_stats.packets_tx, &stats_bytes[8]);
    sys_put_le32(g_stats.packets_rx, &stats_bytes[12]);
    sys_put_le32(g_stats.crc_errors, &stats_bytes[16]);
    sys_put_le32(g_stats.host_tx_dropped, &stats_bytes[20]);
    stats_bytes[24] = (uint8_t)g_burst_mode;
    stats_bytes[25] = 0;
    stats_bytes[26] = g_pending_tx_valid ? 1 : 0;
    stats_bytes[27] = g_restart_rx_pending ? 1 : 0;
    sys_put_le32(0, &stats_bytes[28]);
    sys_put_le32(k_uptime_get_32(), &stats_bytes[32]);
    sys_put_le32(g_rx_ring_dropped, &stats_bytes[36]);
    sys_put_le32(g_staging_overwrites, &stats_bytes[40]);
    sys_put_le32(g_stats.burst_id_conflicts, &stats_bytes[44]);
    sys_put_le32(g_hw_rx_stats.received_packets, &stats_bytes[48]);
    sys_put_le32(g_hw_rx_stats.crc_errors, &stats_bytes[52]);
    sys_put_le32(g_hw_rx_stats.length_errors, &stats_bytes[56]);
    sys_put_le32(g_hw_rx_stats.crc_ok, &stats_bytes[60]);
    sys_put_le32(g_parser_frames_killed, &stats_bytes[64]);
    queue_ctrl_tx(header, 3, stats_bytes, sizeof(stats_bytes));
}

/* ---------------------------------------------------------------------------
 * Core 0 USB TX Consumer Thread
 * ------------------------------------------------------------------------- */
static void usb_tx_thread(void *arg)
{
    (void)arg;
    while (true) {
        (void)xSemaphoreTake(g_host_tx_sem, pdMS_TO_TICKS(500));
        if (g_radio_ok && g_main_loop_heartbeat_ms != 0) {
            uint32_t hb_ms = g_main_loop_heartbeat_ms;
            uint32_t now_wd = k_uptime_get_32();
            uint32_t stalled_ms = now_wd - hb_ms;
            if (stalled_ms > MAIN_LOOP_WATCHDOG_MS) {
                ESP_LOGE(TAG, "WATCHDOG: superloop stalled %ums — resetting MCU",
                         (unsigned)stalled_ms);
                send_debug_event(30, stalled_ms);
                while (g_dbg_head != g_dbg_tail) {
                    uint16_t nwd = g_dbg_head;
                    (void)usb_tx(g_dbg_ring[nwd], 9);
                    g_dbg_head = (uint16_t)((g_dbg_head + 1) % DBG_RING_DEPTH);
                }
                esp_restart();
            }
        }
        while (g_dbg_head != g_dbg_tail) {
            uint16_t n = g_dbg_head;
            (void)usb_tx(g_dbg_ring[n], 9);
            g_dbg_head = (uint16_t)((g_dbg_head + 1) % DBG_RING_DEPTH);
        }
        while (g_ctrl_tx_read_idx != g_ctrl_tx_write_idx) {
            uint8_t  slot   = g_ctrl_tx_read_idx % CTRL_TX_QUEUE_DEPTH;
            uint16_t tx_len = g_ctrl_tx_len[slot];
            (void)usb_tx(g_ctrl_tx_buf[slot], tx_len);
            __sync_synchronize();
            g_ctrl_tx_read_idx++;
        }
        while (g_host_tx_read_idx != g_host_tx_write_idx) {
            uint8_t  slot   = g_host_tx_read_idx % HOST_TX_QUEUE_DEPTH;
            uint16_t tx_len = g_host_tx_len[slot];
            (void)usb_tx(g_host_tx_buf[slot], tx_len);
            __sync_synchronize();
            g_host_tx_read_idx++;
            while (g_ctrl_tx_read_idx != g_ctrl_tx_write_idx) {
                uint8_t  cslot = g_ctrl_tx_read_idx % CTRL_TX_QUEUE_DEPTH;
                uint16_t clen  = g_ctrl_tx_len[cslot];
                (void)usb_tx(g_ctrl_tx_buf[cslot], clen);
                __sync_synchronize();
                g_ctrl_tx_read_idx++;
            }
        }
    }
}

/* ---------------------------------------------------------------------------
 * FLRC Burst RX & Multi-Slot Reassembly
 * ------------------------------------------------------------------------- */
static void start_burst_rx(void)
{
    g_rac_ctx->modulation_type = SMTC_RAC_MODULATION_FLRC_BURST;

    smtc_rac_radio_flrc_burst_params_t *p = &g_rac_ctx->radio_params.flrc_burst;
    p->is_tx                     = false;
    p->frequency_in_hz           = g_cfg.freq_hz;
    p->rx_frequency_offset_in_hz = 0;
    p->raw_bit_rate              = cfg_raw_bit_rate();
    p->cr                        = RAL_FLRC_CR_1_1;
    p->pulse_shape               = RAL_FLRC_PULSE_SHAPE_BT_05;
    p->preamble_len              = RAL_FLRC_PREAMBLE_LENGTH_32_BITS;
    p->sync_word_len             = RAL_FLRC_SYNCWORD_LENGTH_4_BYTES;
    p->tx_syncword_index         = RAL_FLRC_TX_SYNCWORD_1;
    p->match_sync_word           = RAL_FLRC_RX_MATCH_SYNCWORD_1;
    p->pld_is_fix                = false;
    p->crc_type                  = RAL_FLRC_CRC_2_BYTES;
    p->sync_word[0]              = default_syncword_1;
    p->sync_word[1]              = default_syncword_2;
    p->sync_word[2]              = default_syncword_3;
    p->crc_seed                  = 0xFFFFFFFF;
    p->crc_polynomial            = 0x755B;
    p->max_rx_size               = g_pkt_payload;
    p->burst_rx_size             = FLRC_BURST_MAX_TOTAL_PAYLOAD;
    p->rx_burst_timeout_margin_ms = 30000;
    p->min_interframe_delay_us   = 220;

    g_rac_ctx->smtc_rac_data_buffer_setup.rx_payload_buffer = g_rac_rx_pkt_buf;
    g_rac_ctx->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer =
        sizeof(g_rac_rx_pkt_buf);

    g_rac_ctx->scheduler_config.scheduling    = SMTC_RAC_ASAP_TRANSACTION;
    g_rac_ctx->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms();
    g_rac_ctx->scheduler_config.callback_post_radio_transaction = rac_post_callback;
    g_rac_ctx->scheduler_config.callback_pre_radio_transaction  = NULL;

    g_burst_mode = BURST_RX;

    if (smtc_rac_submit_radio_transaction(g_radio_access_id) != SMTC_RAC_SUCCESS) {
        g_restart_rx_pending = true;
    }
}

static rx_reasm_slot_t *find_or_alloc_rx_slot(uint16_t burst_id, uint16_t total_packets,
                                              uint32_t total_len, uint32_t payload_crc,
                                              uint32_t now_ms, int16_t rssi)
{
    /* 1. Check if burst_id is already active in one of our 4 slots */
    for (int i = 0; i < BURST_REASM_SLOTS; i++) {
        rx_reasm_slot_t *s = &g_rx_slots[i];
        if (s->active && s->burst_id == burst_id) {
            if (s->total_packets != total_packets ||
                s->total_payload_len != total_len ||
                s->payload_crc32 != payload_crc) {
                g_stats.crc_errors++;
                return NULL;
            }
            return s;
        }
    }

    /* 2. Find an inactive or stale slot, or evict the oldest slot */
    int victim = -1;
    uint32_t oldest_ms = 0xFFFFFFFFU;
    for (int i = 0; i < BURST_REASM_SLOTS; i++) {
        rx_reasm_slot_t *s = &g_rx_slots[i];
        if (!s->active || (now_ms - s->start_ms) > RX_SLOT_STALE_MS) {
            if (s->active) {
                g_stats.burst_id_conflicts++;
            }
            victim = i;
            break;
        }
        if (s->start_ms < oldest_ms) {
            oldest_ms = s->start_ms;
            victim = i;
        }
    }
    if (victim < 0) victim = 0;

    rx_reasm_slot_t *s = &g_rx_slots[victim];
    if (s->active) {
        g_stats.burst_id_conflicts++;
    }
    s->active            = true;
    s->burst_id          = burst_id;
    s->total_packets     = total_packets;
    s->received_count    = 0;
    s->total_payload_len = total_len;
    s->payload_crc32     = payload_crc;
    memset(s->received_mask, 0, sizeof(s->received_mask));
    s->start_ms          = now_ms;
    s->last_rssi         = rssi;
    return s;
}

static void on_rx_packet(void)
{
    if (g_scan_active) {
        g_scan_frames++;
        uint16_t scan_size = g_rac_ctx->smtc_rac_data_result.rx_size;
        int16_t  scan_rssi = g_rac_ctx->smtc_rac_data_result.rssi_result;
        const uint8_t *scan_buf = g_rac_ctx->smtc_rac_data_buffer_setup.rx_payload_buffer;
        if (scan_rssi > g_scan_best_rssi) {
            g_scan_best_rssi = (int8_t)scan_rssi;
        }
        if (scan_size >= 4 && scan_buf[0] == 0x53 && scan_buf[1] == 0x4c &&
            scan_buf[2] == 0x52 && scan_buf[3] == 0x46) {
            g_scan_consensus++;
        } else if (scan_size >= 2 && scan_buf[0] == 0x42 && scan_buf[1] == 0x46) {
            g_scan_consensus++;
        }
        return;
    }
    g_stats.packets_rx++;

    uint16_t rx_size = g_rac_ctx->smtc_rac_data_result.rx_size;
    int16_t  rssi    = g_rac_ctx->smtc_rac_data_result.rssi_result;
    uint8_t *rx_buf  = g_rac_ctx->smtc_rac_data_buffer_setup.rx_payload_buffer;

    if (rx_size < sizeof(flrc_packet_header_t) || rx_size > sizeof(g_rac_rx_pkt_buf)) {
        g_stats.crc_errors++;
        return;
    }

    flrc_packet_header_t hdr;
    memcpy(&hdr, rx_buf, sizeof(hdr));
    uint16_t magic         = sys_le16_to_cpu(hdr.magic);
    uint16_t burst_id      = sys_le16_to_cpu(hdr.burst_id);
    uint16_t packet_idx    = sys_le16_to_cpu(hdr.packet_idx);
    uint16_t total_packets = sys_le16_to_cpu(hdr.total_packets);
    uint32_t total_len     = sys_le32_to_cpu(hdr.total_payload_len);
    uint32_t payload_crc   = sys_le32_to_cpu(hdr.payload_crc32);

    uint32_t expected_packets = (total_len + g_pkt_chunk - 1) / g_pkt_chunk;
    if (magic != FLRC_BURST_HEADER_MAGIC || total_packets == 0 ||
        total_packets > MAX_AIR_PACKETS || total_len == 0 ||
        total_len > BURST_REASM_PAYLOAD_MAX || packet_idx >= total_packets ||
        total_packets != expected_packets) {
        g_stats.crc_errors++;
        return;
    }

    rx_reasm_slot_t *slot = find_or_alloc_rx_slot(
        burst_id, total_packets, total_len, payload_crc, k_uptime_get_32(), rssi);
    if (slot == NULL) {
        return;
    }

    uint16_t mask_word = packet_idx >> 6;
    uint64_t mask_bit  = 1ULL << (packet_idx & 63);
    if ((slot->received_mask[mask_word] & mask_bit) == 0) {
        uint32_t offset    = (uint32_t)packet_idx * g_pkt_chunk;
        uint32_t space     = slot->total_payload_len - offset;
        uint32_t chunk_len = rx_size - sizeof(flrc_packet_header_t);
        if (chunk_len > space) {
            chunk_len = space;
        }
        memcpy(&slot->reasm_buf[offset], rx_buf + sizeof(flrc_packet_header_t), chunk_len);
        slot->received_mask[mask_word] |= mask_bit;
        slot->received_count++;
    }
    slot->last_rssi = rssi;

    if (slot->received_count == slot->total_packets) {
        if (flrc_burst_calc_crc32(slot->reasm_buf, slot->total_payload_len) ==
            slot->payload_crc32) {
            g_stats.bursts_rx++;
            send_rx_payload(slot->reasm_buf, (uint16_t)slot->total_payload_len,
                            slot->last_rssi);
        } else {
            g_stats.crc_errors++;
        }
        slot->active = false;
        (void)smtc_rac_flrc_burst_rx_done(g_radio_access_id);
    }
}

/* ---------------------------------------------------------------------------
 * FLRC Burst TX
 * ------------------------------------------------------------------------- */
static void build_tx_air_packet(uint16_t pkt_idx, uint8_t *pkt)
{
    uint32_t payload_len = g_pending_tx_len;
    uint32_t offset      = (uint32_t)pkt_idx * g_pkt_chunk;
    uint32_t chunk_len   = (payload_len - offset < g_pkt_chunk)
                               ? (payload_len - offset)
                               : g_pkt_chunk;

    flrc_packet_header_t hdr;
    hdr.magic             = sys_cpu_to_le16(FLRC_BURST_HEADER_MAGIC);
    hdr.burst_id          = sys_cpu_to_le16(g_pending_burst_id);
    hdr.packet_idx        = sys_cpu_to_le16(pkt_idx);
    hdr.total_packets     = sys_cpu_to_le16(g_tx_total_packets);
    hdr.total_payload_len = sys_cpu_to_le32(payload_len);
    hdr.payload_crc32     = sys_cpu_to_le32(g_tx_payload_crc);

    memcpy(pkt, &hdr, sizeof(hdr));
    memcpy(pkt + sizeof(hdr), g_pending_tx_buf + offset, chunk_len);
    if (sizeof(hdr) + chunk_len < g_pkt_payload) {
        memset(pkt + sizeof(hdr) + chunk_len, 0,
               g_pkt_payload - (sizeof(hdr) + chunk_len));
    }
}

static void prepare_next_tx_fragment(void)
{
    if (g_tx_bytes_sent >= g_tx_air_total_len) return;

    uint16_t pkt_idx = g_tx_next_buf_idx;
    uint16_t buf_idx = pkt_idx % 2;

    build_tx_air_packet(pkt_idx, g_tx_fifo_buf[buf_idx]);
    g_rac_ctx->radio_params.flrc_burst.tx_fifo_payload_buffer[buf_idx] = g_tx_fifo_buf[buf_idx];
    g_tx_fifo_len[buf_idx] = g_pkt_payload;
    g_tx_bytes_sent += g_pkt_payload;
    g_tx_next_buf_idx++;
}

static void execute_burst_tx(void)
{
    uint32_t payload_len = g_pending_tx_len;
    if (payload_len == 0) {
        g_restart_rx_pending = true;
        return;
    }

    send_debug_event(46, ((uint32_t)g_burst_mode << 8) | ((payload_len >> 8) & 0xFF));

    g_tx_payload_crc   = flrc_burst_calc_crc32(g_pending_tx_buf, payload_len);
    g_tx_total_packets = (payload_len + g_pkt_chunk - 1) / g_pkt_chunk;
    g_tx_air_total_len = (uint32_t)g_tx_total_packets * g_pkt_payload;

    g_rac_ctx->modulation_type = SMTC_RAC_MODULATION_FLRC_BURST;

    smtc_rac_radio_flrc_burst_params_t *p = &g_rac_ctx->radio_params.flrc_burst;
    p->is_tx                   = true;
    p->frequency_in_hz         = g_cfg.freq_hz;
    p->tx_power_in_dbm         = g_cfg.tx_pwr_dbm;
    p->raw_bit_rate            = cfg_raw_bit_rate();
    p->cr                      = RAL_FLRC_CR_1_1;
    p->pulse_shape             = RAL_FLRC_PULSE_SHAPE_BT_05;
    p->preamble_len            = RAL_FLRC_PREAMBLE_LENGTH_32_BITS;
    p->sync_word_len           = RAL_FLRC_SYNCWORD_LENGTH_4_BYTES;
    p->tx_syncword_index       = RAL_FLRC_TX_SYNCWORD_1;
    p->match_sync_word         = RAL_FLRC_RX_MATCH_SYNCWORD_1;
    p->pld_is_fix              = false;
    p->crc_type                = RAL_FLRC_CRC_2_BYTES;
    p->sync_word[0]            = default_syncword_1;
    p->sync_word[1]            = default_syncword_2;
    p->sync_word[2]            = default_syncword_3;
    p->crc_seed                = 0xFFFFFFFF;
    p->crc_polynomial          = 0x755B;
    p->min_interframe_delay_us = 220;
    p->max_rx_size             = g_pkt_payload;
    p->burst_tx_size           = g_tx_air_total_len;

    g_tx_bytes_sent   = 0;
    g_tx_next_buf_idx = 0;
    p->size_of_tx_fifo_payload_buffer[0] = g_pkt_payload;
    p->size_of_tx_fifo_payload_buffer[1] = g_pkt_payload;

    build_tx_air_packet(0, g_tx_fifo_buf[0]);
    p->tx_fifo_payload_buffer[0] = g_tx_fifo_buf[0];
    g_tx_fifo_len[0]             = g_pkt_payload;
    p->tx_fifo_payload_length[0] = &g_tx_fifo_len[0];
    g_tx_bytes_sent += g_pkt_payload;
    g_tx_next_buf_idx = 1;

    p->tx_fifo_payload_buffer[1] = g_tx_fifo_buf[1];
    g_tx_fifo_len[1]             = 0;
    p->tx_fifo_payload_length[1] = &g_tx_fifo_len[1];
    if (g_tx_bytes_sent < g_tx_air_total_len) {
        build_tx_air_packet(1, g_tx_fifo_buf[1]);
        g_tx_fifo_len[1]  = g_pkt_payload;
        g_tx_bytes_sent  += g_pkt_payload;
        g_tx_next_buf_idx = 2;
    }

    g_rac_ctx->scheduler_config.scheduling    = SMTC_RAC_ASAP_TRANSACTION;
    g_rac_ctx->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms();
    g_rac_ctx->scheduler_config.callback_post_radio_transaction = rac_post_callback;
    g_rac_ctx->scheduler_config.callback_pre_radio_transaction  = NULL;
    g_rac_ctx->lbt_context.lbt_enabled = false;

    g_burst_mode    = BURST_TX;
    g_tx_airing     = false;
    g_tx_started_ms = k_uptime_get_32();
    g_tx_requests   = 0;
    g_tx_requests_expected = (g_tx_air_total_len + g_pkt_payload - 1) / g_pkt_payload;
    if (g_tx_requests_expected >= 2) {
        g_tx_requests_expected -= 2;
    } else {
        g_tx_requests_expected = 0;
    }
    g_stats.bursts_tx++;

    if (smtc_rac_submit_radio_transaction(g_radio_access_id) != SMTC_RAC_SUCCESS) {
        send_error(FLRC_ERR_TX_FAILED, "tx submit failed");
        g_pending_tx_valid   = false;
        g_pending_tx_len     = 0;
        g_burst_mode         = BURST_RX;
        g_restart_rx_pending = true;
    }
}

static void rac_post_callback(rp_status_t status)
{
    switch (status) {
    case RP_STATUS_TX_DONE:
        g_tx_airing    = true;
        g_tx_done_ms   = k_uptime_get_32();
        g_tx_done_pass = g_engine_pass_count;
        send_debug_event(43, g_tx_done_ms);
        break;

    case RP_STATUS_RX_PACKET:
        on_rx_packet();
        break;

    case RP_STATUS_REQUEST_NEXT_TX_PAYLOAD:
        g_stats.packets_tx++;
        g_tx_airing = true;
        g_tx_requests++;
        prepare_next_tx_fragment();
        break;

    case RP_STATUS_RX_CRC_ERROR:
        g_stats.crc_errors++;
        break;

    case RP_STATUS_RX_TIMEOUT:
        start_burst_rx();
        send_debug_event(44, k_uptime_get_32() - g_tx_done_ms);
        send_debug_event(49, g_engine_pass_count - g_tx_done_pass);
        break;

    case RP_STATUS_RADIO_UNLOCKED:
        if (g_burst_mode == BURST_TX_ABORTING) {
            (void)smtc_rac_flrc_burst_rx_done(g_radio_access_id);
            g_tx_rearm_pending = true;
        } else if (g_burst_mode == BURST_TX) {
            g_tx_done_ms = k_uptime_get_32();
            send_debug_event(43, g_tx_done_ms);
            uint32_t ev_ms = 0;
            uint16_t ev_err = 0;
            bool     ev_valid = false;
            smtc_rac_flrc_burst_get_tx_fail_evidence(&ev_ms, &ev_err, &ev_valid);
            if (g_tx_airing && g_tx_requests >= g_tx_requests_expected) {
                send_ready();
            } else if (g_tx_airing) {
                if (ev_valid) {
                    send_debug_event(42, ev_err);
                }
                send_error(FLRC_ERR_TX_DROPPED, "burst truncated");
            } else if (ev_valid) {
                send_debug_event(41, ev_err);
                send_error(FLRC_ERR_TX_DROPPED, "tx never aired");
            } else {
                (void)smtc_rac_flrc_burst_rx_done(g_radio_access_id);
                send_error(FLRC_ERR_TX_DROPPED, "tx unlock no air");
            }
            g_pending_tx_valid = false;
            g_pending_tx_len   = 0;
            promote_staged_tx_if_ready();
            g_burst_mode       = BURST_RX;
            start_burst_rx();
            send_debug_event(44, k_uptime_get_32() - g_tx_done_ms);
            send_debug_event(49, g_engine_pass_count - g_tx_done_pass);
            {
                uint8_t pending = (uint8_t)(g_host_tx_write_idx - g_host_tx_read_idx);
                send_debug_event(45, pending);
            }
        } else {
            start_burst_rx();
            send_debug_event(44, k_uptime_get_32() - g_tx_done_ms);
            send_debug_event(49, g_engine_pass_count - g_tx_done_pass);
        }
        break;

    case RP_STATUS_TASK_ABORTED:
        if (g_burst_mode == BURST_TX_ABORTING) {
            (void)smtc_rac_flrc_burst_rx_done(g_radio_access_id);
            g_tx_rearm_pending = true;
        } else if (g_burst_mode == BURST_TX) {
            send_error(FLRC_ERR_TX_FAILED, "tx aborted");
            g_pending_tx_valid = false;
            g_pending_tx_len   = 0;
            g_burst_mode       = BURST_RX;
            start_burst_rx();
        } else {
            start_burst_rx();
        }
        break;

    default:
        break;
    }
}

/* ---------------------------------------------------------------------------
 * Host Command Handling & USB RX Parser (Core 0 -> Ring -> Superloop)
 * ------------------------------------------------------------------------- */
#define UART_RX_RING_SIZE 65536 /* 64 KB power-of-2 ring in PSRAM */
static uint8_t          *g_rx_ring = NULL;
static volatile uint32_t g_rx_ring_head = 0;
static volatile uint32_t g_rx_ring_tail = 0;
static volatile uint32_t g_rx_last_arrival_ms = 0;

static void usb_rx_reader_task(void *arg)
{
    (void)arg;
    uint8_t chunk[512];
    while (true) {
        int n = am36_usb_io_read(chunk, sizeof(chunk), 2);
        if (n > 0) {
            uint32_t now = k_uptime_get_32();
            for (int i = 0; i < n; i++) {
                uint32_t next = (g_rx_ring_head + 1) & (UART_RX_RING_SIZE - 1);
                if (next == g_rx_ring_tail) {
                    g_rx_ring_dropped++;
                    break;
                }
                g_rx_ring[g_rx_ring_head] = chunk[i];
                g_rx_ring_head = next;
            }
            g_rx_last_arrival_ms = now;
        }
    }
}

static void handle_config(const uint8_t *body, uint16_t body_len)
{
    uint32_t req_freq = sys_get_le32(&body[0]);
    if (req_freq == 0xFFFFFFFF) {
        g_scan_freqs[0] = g_cfg.freq_hz;
        uint32_t start  = sys_get_le32(&body[10]);
        uint32_t step   = sys_get_le32(&body[4]);
        uint8_t  count  = body[8];
        uint32_t dwell  = (uint32_t)body[9] * 100;
        if (count == 0 || count > SCAN_MAX_CHANNELS) count = SCAN_MAX_CHANNELS;
        for (uint8_t i = 0; i < count; i++) {
            g_scan_freqs[i] = start + i * step;
        }
        g_scan_count       = count;
        g_scan_dwell_ms    = dwell;
        g_scan_idx         = 0;
        g_scan_frames      = 0;
        g_scan_consensus   = 0;
        g_scan_best_rssi   = -128;
        g_scan_dwell_until = k_uptime_get_32() + g_scan_dwell_ms;
        g_pending_tx_valid = false;
        g_burst_mode       = BURST_RX;
        g_restart_rx_pending = true;
        g_scan_active      = true;
        return;
    }
    g_cfg.freq_hz    = req_freq;
    g_cfg.cr_host    = body[8];
    g_cfg.tx_pwr_dbm = (int8_t)body[9];
    g_cfg.sync_word  = sys_get_le32(&body[10]);
    g_cfg.role       = body[14];
    if (body_len >= 16) {
        g_cfg.rate_sel = (body[15] <= 1) ? body[15] : g_cfg.rate_sel;
    }
    if (body_len >= 17) {
        g_cfg.payload_sel = (body[16] <= 1) ? body[16] : g_cfg.payload_sel;
        cfg_apply_selectors();
    }
    g_restart_rx_pending = true;
}

typedef enum {
    RX_STATE_TAG,
    RX_STATE_SCAN_COUNT,
    RX_STATE_SCAN_BODY,
    RX_STATE_TX_MAGIC,
    RX_STATE_LEN_LO,
    RX_STATE_LEN_HI,
    RX_STATE_PAYLOAD,
    RX_STATE_CONFIG,
    RX_STATE_SET_WINDOW,
} rx_state_t;

static volatile rx_state_t state      = RX_STATE_TAG;
static uint8_t             tag        = 0;
static uint8_t             scan_count = 0;
static uint8_t             scan_have  = 0;
static uint8_t             scan_buf[4 + SCAN_MAX_CHANNELS * 4];
static uint16_t            frame_len  = 0;
static uint16_t            have       = 0;
static bool                rx_drop_busy = false;
static uint16_t            skip_remain  = 0;
static uint8_t             config_body[FLRC_CONFIG_BODY_LEN_V2];
static uint8_t             window_body[8];

static void scan_emit_result(void)
{
    send_debug_event(50, g_scan_freqs[g_scan_idx]);
    send_debug_event(51, g_scan_frames);
    send_debug_event(52, g_scan_consensus);
    send_debug_event(53, (uint32_t)((int32_t)g_scan_best_rssi + 128));
}

static void scan_tick(void)
{
    uint32_t now = k_uptime_get_32();
    if ((int32_t)(now - g_scan_dwell_until) < 0) {
        return;
    }
    scan_emit_result();
    g_scan_idx++;
    if (g_scan_idx >= g_scan_count) {
        g_scan_active        = false;
        g_burst_mode         = BURST_RX;
        g_restart_rx_pending = true;
        return;
    }
    g_cfg.freq_hz      = g_scan_freqs[g_scan_idx];
    g_scan_frames      = 0;
    g_scan_consensus   = 0;
    g_scan_best_rssi   = -128;
    g_scan_dwell_until = now + g_scan_dwell_ms;
    g_burst_mode       = BURST_RX;
    g_restart_rx_pending = true;
}

static int usb_rx_poll(void)
{
    uint32_t last_arrival = g_rx_last_arrival_ms;
    uint32_t now          = k_uptime_get_32();

    if (state != RX_STATE_TAG && g_rx_ring_tail == g_rx_ring_head &&
        (int32_t)(now - last_arrival) > 1500) {
        if (tag == FLRC_MSG_TX &&
            (state == RX_STATE_LEN_LO || state == RX_STATE_LEN_HI ||
             state == RX_STATE_PAYLOAD)) {
            g_parser_frames_killed++;
            send_error(FLRC_ERR_TX_DROPPED, "rx timeout");
        } else if (state == RX_STATE_CONFIG || state == RX_STATE_SET_WINDOW ||
                   state == RX_STATE_LEN_HI) {
            g_parser_frames_killed++;
        }
        skip_remain  = 0;
        rx_drop_busy = false;
        state        = RX_STATE_TAG;
        have         = 0;
    }

    uint8_t b;
    while (g_rx_ring_tail != g_rx_ring_head) {
        b = g_rx_ring[g_rx_ring_tail];
        g_rx_ring_tail = (g_rx_ring_tail + 1) & (UART_RX_RING_SIZE - 1);

        if (skip_remain > 0) {
            skip_remain--;
            continue;
        }
        switch (state) {
        case RX_STATE_TAG:
            tag = b;
            if (tag == FLRC_MSG_TX) {
                state = RX_STATE_TX_MAGIC;
                have  = 0;
            } else if (tag == FLRC_MSG_SET_WINDOW) {
                state = RX_STATE_SET_WINDOW;
                have  = 0;
            } else if (tag == FLRC_MSG_CONFIG) {
                state = RX_STATE_LEN_LO;
                have  = 0;
            } else if (tag == FLRC_MSG_STATS) {
                send_stats_packet();
            } else if (tag == FLRC_MSG_SCAN) {
                state = RX_STATE_SCAN_COUNT;
                have  = 0;
            } else if (tag == FLRC_MSG_RESET) {
                uint8_t ack = FLRC_MSG_READY;
                usb_tx(&ack, 1);
                esp_rom_delay_us(10000);
                esp_restart();
            }
            break;

        case RX_STATE_SCAN_COUNT:
            scan_count = b;
            scan_have  = 0;
            if (scan_count == 0 || scan_count > SCAN_MAX_CHANNELS) {
                state = RX_STATE_TAG;
            } else {
                state = RX_STATE_SCAN_BODY;
            }
            break;

        case RX_STATE_SCAN_BODY:
            scan_buf[scan_have++] = b;
            if (scan_have >= 4 + (uint16_t)scan_count * 4) {
                g_scan_dwell_ms =
                    (uint32_t)scan_buf[1] | ((uint32_t)scan_buf[2] << 8) |
                    ((uint32_t)scan_buf[3] << 16) | ((uint32_t)scan_buf[4] << 24);
                if (g_scan_dwell_ms < 50) {
                    g_scan_dwell_ms = 50;
                }
                g_scan_count = scan_count;
                for (uint8_t i = 0; i < scan_count; i++) {
                    const uint8_t *fp = &scan_buf[5 + i * 4];
                    g_scan_freqs[i] = (uint32_t)fp[0] | ((uint32_t)fp[1] << 8) |
                                      ((uint32_t)fp[2] << 16) | ((uint32_t)fp[3] << 24);
                }
                g_scan_idx           = 0;
                g_scan_frames        = 0;
                g_scan_consensus     = 0;
                g_scan_best_rssi     = -128;
                g_cfg.freq_hz        = g_scan_freqs[0];
                g_scan_dwell_until   = k_uptime_get_32() + g_scan_dwell_ms;
                g_pending_tx_valid   = false;
                g_burst_mode         = BURST_RX;
                g_restart_rx_pending = true;
                g_scan_active        = true;
                state                = RX_STATE_TAG;
            }
            break;

        case RX_STATE_TX_MAGIC:
            if (b == 'X') {
                state = RX_STATE_LEN_LO;
                have  = 0;
            } else {
                state = RX_STATE_TAG;
            }
            break;

        case RX_STATE_SET_WINDOW:
            window_body[have++] = b;
            if (have >= 8) {
                g_slot_offset_ms = sys_get_le32(&window_body[0]);
                g_slot_period_ms = sys_get_le32(&window_body[4]);
                send_ready();
                state = RX_STATE_TAG;
            }
            break;

        case RX_STATE_LEN_LO:
            frame_len = b;
            state     = RX_STATE_LEN_HI;
            break;

        case RX_STATE_LEN_HI:
            frame_len |= (uint16_t)(b << 8);
            have = 0;
            if (tag == FLRC_MSG_CONFIG) {
                if (frame_len != FLRC_CONFIG_BODY_LEN &&
                    frame_len != FLRC_CONFIG_BODY_LEN_V2) {
                    send_error(FLRC_ERR_BAD_CONFIG, "bad len");
                    state = RX_STATE_TAG;
                    break;
                }
                state = RX_STATE_CONFIG;
            } else {
                if (frame_len == 0 || frame_len > FLRC_BURST_MAX_TOTAL_PAYLOAD) {
                    send_error(FLRC_ERR_PAYLOAD_TOO_BIG, "bad tx len");
                    state = RX_STATE_TAG;
                    break;
                }
                promote_staged_tx_if_ready();
                rx_drop_busy = g_staging_tx_valid;
                state        = RX_STATE_PAYLOAD;
            }
            break;

        case RX_STATE_PAYLOAD:
            if (have < frame_len) {
                if (!rx_drop_busy) {
                    g_staging_tx_buf[have] = b;
                }
                have++;
            }
            if (have >= frame_len) {
                if (rx_drop_busy || g_staging_tx_valid) {
                    rx_drop_busy = false;
                    g_staging_overwrites++;
                    send_error(FLRC_ERR_TX_BUSY, "tx queue full");
                    state = RX_STATE_TAG;
                    break;
                }
                g_staging_tx_len   = frame_len;
                g_staging_burst_id = (uint16_t)(esp_random() & 0xFFFFU);
                if (g_staging_burst_id == 0) {
                    g_staging_burst_id = 1;
                }
                g_staging_tx_valid = true;
                send_debug_event(47, frame_len);
                state = RX_STATE_TAG;
            }
            break;

        case RX_STATE_CONFIG:
            if (have < sizeof(config_body)) {
                config_body[have++] = b;
            }
            if (have >= frame_len) {
                handle_config(config_body, frame_len);
                send_ready();
                state = RX_STATE_TAG;
            }
            break;

        default:
            state = RX_STATE_TAG;
            break;
        }
    }
    return 0;
}

static int init_radio(void)
{
    SMTC_SW_PLATFORM_INIT();
    SMTC_SW_PLATFORM_VOID(smtc_rac_init());

    g_radio_access_id = SMTC_SW_PLATFORM(smtc_rac_open_radio(RAC_HIGH_PRIORITY));
    if (g_radio_access_id == 0xFF) return -1;

    g_rac_ctx = smtc_rac_get_context(g_radio_access_id);
    if (g_rac_ctx == NULL) return -1;

    g_rac_ctx->scheduler_config.callback_post_radio_transaction = rac_post_callback;

    start_burst_rx();
    return 0;
}

/* ---------------------------------------------------------------------------
 * Core 1 Dedicated Radio Superloop
 * ------------------------------------------------------------------------- */
static void rac_superloop_task(void *arg)
{
    (void)arg;
    lr2021_deep_reset();

    if (init_radio() == 0) {
        g_radio_ok = true;
        send_ready();
    } else {
        g_radio_ok = false;
        send_error(FLRC_ERR_RADIO_INIT, "radio init failed");
    }

    uint32_t g_last_stats_emit_ms = 0;

    while (true) {
        g_main_loop_heartbeat_ms = k_uptime_get_32();
        g_engine_pass_count++;
        promote_staged_tx_if_ready();
        (void)usb_rx_poll();

        if (g_radio_ok && g_scan_active) {
            g_main_loop_heartbeat_ms = k_uptime_get_32();
            scan_tick();
            (void)smtc_rac_run_engine();
        }

        if (g_radio_ok && !g_scan_active) {
            uint32_t now_ms = k_uptime_get_32();

            if (g_tx_rearm_pending) {
                g_tx_rearm_pending = false;
                if (g_pending_tx_valid) {
                    execute_burst_tx();
                } else {
                    g_burst_mode         = BURST_RX;
                    g_restart_rx_pending = true;
                }
            } else if (g_burst_mode == BURST_TX &&
                       (now_ms - g_tx_started_ms) > TX_INFLIGHT_TIMEOUT_MS) {
                (void)smtc_rac_abort_radio_submit(g_radio_access_id);
                g_burst_mode     = BURST_TX_ABORTING;
                g_abort_since_ms = now_ms;
            } else if (g_burst_mode == BURST_TX_ABORTING &&
                       (now_ms - g_abort_since_ms) > 200) {
                g_burst_mode = BURST_RX;
                if (g_pending_tx_valid) {
                    execute_burst_tx();
                } else {
                    g_restart_rx_pending = true;
                }
            }

            if (g_restart_rx_pending && g_burst_mode == BURST_RX) {
                g_restart_rx_pending = false;
                start_burst_rx();
            }

            promote_staged_tx_if_ready();

            if (g_cfg.role != FLRC_ROLE_RX_ONLY && g_pending_tx_valid &&
                g_burst_mode == BURST_RX) {
                send_debug_event(48, g_pending_tx_len);
                g_burst_mode     = BURST_TX_ABORTING;
                g_abort_since_ms = now_ms;
                smtc_rac_abort_radio_submit(g_radio_access_id);
            }

            for (int i = 0; i < BURST_REASM_SLOTS; i++) {
                if (g_rx_slots[i].active &&
                    (now_ms - g_rx_slots[i].start_ms) > RX_SLOT_STALE_MS) {
                    g_stats.burst_id_conflicts++;
                    g_rx_slots[i].active = false;
                }
            }

            (void)smtc_rac_run_engine();

            if (g_last_stats_emit_ms == 0) {
                g_last_stats_emit_ms = now_ms;
            } else if ((now_ms - g_last_stats_emit_ms) >= 10000) {
                g_last_stats_emit_ms = now_ms;
                lr20xx_radio_flrc_get_rx_stats(smtc_rac_get_radio_driver_context(),
                                               &g_hw_rx_stats);
                send_stats_packet();
            }
        }

        esp_rom_delay_us(250);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "LR2021 FLRC bridge booting on Lierda AM36-Pico (ESP32-S3, 64KB burst, 4 RX slots)");

    /* Allocate 64KB-capable TX/RX buffers in 8MB Octal PSRAM */
    g_pending_tx_buf = heap_caps_calloc(1, FLRC_BURST_MAX_TOTAL_PAYLOAD, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_staging_tx_buf = heap_caps_calloc(1, FLRC_BURST_MAX_TOTAL_PAYLOAD, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_rx_slots       = heap_caps_calloc(BURST_REASM_SLOTS, sizeof(rx_reasm_slot_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_host_tx_buf    = heap_caps_calloc(HOST_TX_QUEUE_DEPTH, HOST_TX_SLOT_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_rx_ring        = heap_caps_calloc(1, UART_RX_RING_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    g_host_tx_sem = xSemaphoreCreateCounting(64, 0);
    g_dbg_lock    = xSemaphoreCreateMutex();

    am36_led_init();
    ESP_ERROR_CHECK(am36_usb_io_init());

    /* Core 0: USB RX reader and USB TX consumer threads */
    xTaskCreatePinnedToCore(usb_rx_reader_task, "usb_rx", 8192,  NULL, 12, NULL, 0);
    xTaskCreatePinnedToCore(usb_tx_thread,      "usb_tx", 8192,  NULL, 10, NULL, 0);

    /* Core 1: Dedicated Semtech RAC radio superloop */
    xTaskCreatePinnedToCore(rac_superloop_task, "rac_eng", 16384, NULL, 22, NULL, 1);
}
