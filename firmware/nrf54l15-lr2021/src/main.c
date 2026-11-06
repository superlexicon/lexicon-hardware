/**
 * @file      main.c
 *
 * @brief     LR2021 USB-UART <-> FLRC hardware-burst bridge firmware (Zephyr RTOS).
 *
 *            PHASE C CUT-OVER (2026-09-04): the app is now thin, following the
 *            Semtech flrc_burst example's discipline on top of the RAC burst
 *            library. Everything radio-related that this file used to hand-roll
 *            (TX staging ledgers, re-arm state machines, retry ladders, six
 *            stacked watchdogs, IRQ resuscitation, diagnostic event streams)
 *            is deleted; the RAC library owns the burst protocol.
 *
 *            RX:  one continuous FLRC-burst transaction; each received packet
 *                 is memcpy'd into a burst-reassembly slot keyed by the on-air
 *                 FB header (magic/burst_id/idx/total/len/crc32 — our in-band
 *                 replacement for the example's WOR preamble, since this link
 *                 is fire-and-forget broadcast with no handshake). On burst
 *                 completion the whole verified payload is delivered to the
 *                 host as ONE 'R' envelope, and smtc_rac_flrc_burst_rx_done()
 *                 ends the transaction so every burst starts freshly armed —
 *                 exactly the example's lifecycle.
 *            TX:  the host's 'T' frame is chunked into FB-headered 252B air
 *                 packets (format unchanged from the previous generation),
 *                 streamed by the library's FIFO ping-pong. Completion maps to
 *                 the host 'G' terminal; a burst that never aired maps to 'E'.
 *            The host serial contract ('T'/'C'/'W'/'S'/'X' in, 'G'/'R'/'E'/'S'
 *            out, 68-byte stats layout) is byte-identical to the previous
 *             firmware generation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <errno.h>

#include "flrc_config.h"
#include "flrc_burst_protocol.h"

#include "smtc_rac.h"
#include <lr20xx_radio_flrc.h>
#include "smtc_rac_api.h"
#include "smtc_sw_platform_helper.h"
#include "smtc_modem_hal.h"
#include "radio_planner_types.h"
#include <zephyr/usp/smtc_sw_platform_helper.h>
#include <zephyr/usp/smtc_zephyr_usp_api.h>

LOG_MODULE_REGISTER( lr2021_bridge, LOG_LEVEL_INF );

#define LR2021_NODE DT_ALIAS( lora_transceiver )
#if !DT_NODE_HAS_STATUS( LR2021_NODE, okay )
#error "LR2021 radio node not found."
#endif

/* GPIO control for Seeed XIAO nRF54L15 onboard RF switch power */
#if DT_NODE_EXISTS(DT_PATH(rfsw_pwr))
static const struct gpio_dt_spec rfsw_pwr_gpio = GPIO_DT_SPEC_GET(DT_PATH(rfsw_pwr), enable_gpios);
#endif
#if DT_NODE_EXISTS(DT_PATH(rfsw_ctl))
static const struct gpio_dt_spec rfsw_ctl_gpio = GPIO_DT_SPEC_GET(DT_PATH(rfsw_ctl), enable_gpios);
#endif

/* Default sync words (match the Semtech burst sample). */
static const uint8_t default_syncword_1[4] = { 0x90, 0x56, 0x34, 0x12 };
static const uint8_t default_syncword_2[4] = { 0x00, 0x00, 0x00, 0x00 };
static const uint8_t default_syncword_3[4] = { 0x00, 0x00, 0x00, 0x00 };

/* ---------------------------------------------------------------------------
 * Boot-time deep reset: hold the chip in NRST for 20ms so every boot starts
 * with a clean radio sequencer (ral_reset's 100us pulse does not clear a
 * wedged sequencer). Init-time only — not a runtime recovery path.
 * ------------------------------------------------------------------------- */
static void lr2021_deep_reset( void )
{
    static const struct gpio_dt_spec rst =
        GPIO_DT_SPEC_GET( DT_NODELABEL( lr2021_radio ), reset_gpios );
    ( void ) gpio_pin_configure_dt( &rst, GPIO_OUTPUT_ACTIVE ); /* NRST asserted */
    k_sleep( K_MSEC( 20 ) );
    ( void ) gpio_pin_configure_dt( &rst, GPIO_OUTPUT_INACTIVE );
    k_sleep( K_MSEC( 10 ) ); /* chip boot before any SPI traffic */
}

/* Globals */
static const struct device* const cdc_dev = DEVICE_DT_GET( DT_CHOSEN( zephyr_console ) );

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
    /* CONFIG v2 selectors (owner directive, 2026-09-05): the raw bit rate
     * and on-air packet payload are host-configurable at runtime — no
     * reflash to change speed or packet size. Defaults = the currently
     * validated bench configuration. */
    uint8_t  rate_sel;      /* 0 = 1.3 Mbps, 1 = 2.6 Mbps */
    uint8_t  payload_sel;   /* 0 = 252B, 1 = 511B */
} g_cfg = {
    .freq_hz    = FLRC_DEFAULT_FREQ_HZ,
    .cr_host    = FLRC_DEFAULT_CODING_RATE,
    .tx_pwr_dbm = FLRC_DEFAULT_TX_PWR_DBM,
    .sync_word  = FLRC_DEFAULT_SYNC_WORD,
    .role       = FLRC_ROLE_BOTH,
    .rate_sel   = 1,
    .payload_sel = 0,
};

/* Active packet payload (252 or 511) — set from the CONFIG v2 selector.
 * The static buffers are sized for the maximum (FLRC_PKT_PAYLOAD_MAX). */
// Defaults must be the smallest safe configuration (ALT = 252B, the
// known-good bench format): the rows below are sized for the MAXIMUM
// runtime selection, but the runtime value decides the chunk stride —
// a MAX default with ALT-sized anything is a buffer overflow
// (SuperLexicon 2026-09-05: 511 default + 252 rows wedged TX from boot
// on every node).
static uint16_t g_pkt_payload  = FLRC_PKT_PAYLOAD_ALT;
static uint16_t g_pkt_chunk    = FLRC_PKT_PAYLOAD_ALT - sizeof( flrc_packet_header_t );

static void cfg_apply_selectors( void )
{
    g_pkt_payload = ( g_cfg.payload_sel == 1 ) ? FLRC_PKT_PAYLOAD_MAX : FLRC_PKT_PAYLOAD_ALT;
    g_pkt_chunk   = g_pkt_payload - sizeof( flrc_packet_header_t );
}

static ral_flrc_raw_bit_rate_t cfg_raw_bit_rate( void )
{
    return ( g_cfg.rate_sel == 1 ) ? RAL_FLRC_RAW_BIT_RATE_2_600_MBPS
                                   : RAL_FLRC_RAW_BIT_RATE_1_300_MBPS;
}

/* Radio state machine — the example's shape: one transaction at a time,
 * RX re-armed after every burst, TX handoff via abort + deferred submit
 * (a submit inside a RAC callback reenters the engine and deadlocks). */
typedef enum {
    BURST_RX,
    BURST_TX_ABORTING,
    BURST_TX,
} burst_mode_t;
static volatile burst_mode_t g_burst_mode        = BURST_RX;
static volatile bool         g_restart_rx_pending = false;
static volatile bool         g_tx_rearm_pending   = false;
static volatile bool         g_tx_airing          = false; /* set by the first packet-sent callback */
static uint32_t              g_tx_started_ms      = 0;     /* superloop in-flight watchdog anchor */
static uint32_t              g_tx_requests        = 0;     /* packet-refill callbacks consumed */
static uint32_t              g_tx_requests_expected = 0;   /* packets-2: the last packet is never requested */
#define TX_INFLIGHT_TIMEOUT_MS 20000
static uint32_t              g_abort_since_ms     = 0;

/* Transmit Window Configuration — parsed for host compatibility; the host
 * owns scheduling, the firmware transmits ASAP. */
static uint32_t g_slot_offset_ms = 100;
static uint32_t g_slot_period_ms = 1000;

/* TX: pending (airing) + staging (queued) — the 2-deep pipeline the host's
 * windowed transmitter expects. The parser writes staging only; the
 * superloop promotes when the radio is free. */
static uint8_t  g_pending_tx_buf[FLRC_BURST_MAX_TOTAL_PAYLOAD];
static uint32_t g_pending_tx_len = 0;
static bool     g_pending_tx_valid = false;
static uint8_t  g_staging_tx_buf[FLRC_BURST_MAX_TOTAL_PAYLOAD];
static uint32_t g_staging_tx_len = 0;
static bool     g_staging_tx_valid = false;
static uint32_t g_staging_overwrites = 0;

/* burst_id must not collide across nodes: XOR the incrementing counter with
 * a per-boot random offset when building air headers.
 * Keep staging and pending burst IDs separate so staging a pipelined 2nd
 * frame never overwrites g_pending_burst_id while frame 1 is airing. */
static uint16_t g_pending_burst_id = 0;
static uint16_t g_staging_burst_id = 0;

static void promote_staged_tx_if_ready( void )
{
    if( !g_pending_tx_valid && g_staging_tx_valid )
    {
        memcpy( g_pending_tx_buf, g_staging_tx_buf, g_staging_tx_len );
        g_pending_tx_len   = g_staging_tx_len;
        g_pending_burst_id = g_staging_burst_id;
        g_pending_tx_valid = true;
        g_staging_tx_valid = false;
        g_staging_tx_len   = 0;
    }
}

/* TX: on-air 2-slot ping-pong FIFO buffer.
 * The RAC burst engine only ever accesses 2 active packet slots at a time
 * (pre-pushing packets 0 and 1 at launch, then requesting packet N into
 * slot N%2 on each TX_TIMESTAMP IRQ and immediately pushing it over SPI).
 * Formatting packets on demand from g_pending_tx_buf eliminates the
 * MAX_AIR_PACKETS * 511B staging table. */
#define MAX_AIR_PACKETS ((FLRC_BURST_MAX_TOTAL_PAYLOAD + FLRC_PKT_PAYLOAD_ALT - sizeof( flrc_packet_header_t ) - 1) / ( FLRC_PKT_PAYLOAD_ALT - sizeof( flrc_packet_header_t ) ))
static uint8_t  g_tx_fifo_buf[2][FLRC_PKT_PAYLOAD_MAX];
static uint32_t g_tx_payload_crc = 0;
static uint16_t g_tx_total_packets = 0;
static uint32_t g_tx_air_total_len = 0;
static uint32_t g_tx_bytes_sent = 0;
static uint16_t g_tx_fifo_len[2];
static uint16_t g_tx_next_buf_idx = 0;

/* RX: burst reassembly slot. Chunks land at packet_idx-strided offsets and a
 * multi-word bitmask + packet counter track completeness, supporting bursts
 * beyond 64 air packets (up to MAX_AIR_PACKETS = 105 at 24KB / 236B chunk)
 * while ignoring duplicate or reordered chunks. */
#define BURST_REASM_PAYLOAD_MAX FLRC_BURST_MAX_TOTAL_PAYLOAD
#define RX_SLOT_STALE_MS        2000
#define RX_MASK_WORDS           ((MAX_AIR_PACKETS + 63) / 64)
static struct
{
    bool     active;
    uint16_t burst_id;
    uint16_t total_packets;
    uint16_t received_count;
    uint32_t total_payload_len;
    uint32_t payload_crc32;
    uint64_t received_mask[RX_MASK_WORDS];
    uint32_t start_ms;
    int16_t  last_rssi;
} g_rx_slot = { 0 };
static uint8_t g_rx_reasm_buf[BURST_REASM_PAYLOAD_MAX];

/* Single packet RX buffer handed to the RAC */
static uint8_t g_rac_rx_pkt_buf[FLRC_PKT_PAYLOAD_MAX];

/* Latest hardware FLRC RX stats snapshot (chip-level counters, reported in
 * the stats line so the host can distinguish chip-heard from app-delivered). */
static lr20xx_radio_flrc_rx_stats_t g_hw_rx_stats = { 0 };

/* Stats — the 68-byte layout the host parses; keep byte offsets stable. */
static struct
{
    uint32_t bursts_tx;
    uint32_t bursts_rx;
    uint32_t packets_tx;
    uint32_t packets_rx;
    uint32_t crc_errors;         /* per-packet CRC fails + bad payload CRC32 + malformed headers */
    uint32_t host_tx_dropped;    /* hostbound frames dropped (queue full / too big) */
    uint32_t burst_id_conflicts; /* incomplete slots evicted by a different burst_id */
} g_stats = { 0 };

/* ---------------------------------------------------------------------------
 * Deferred host UART transfer queues + dedicated TX consumer thread.
 *
 * The radio callback runs inside smtc_rac_run_engine() polled by the main
 * superloop. Producers (radio callback, parser — both main-thread context)
 * copy frames into these rings and post a semaphore; a lower-priority
 * preemptible thread streams slots to the UART while the main thread keeps
 * polling the radio engine every ~1ms.
 *
 * Control frames ('G', 'E', 'S', <= 71B) use a dedicated 8-slot control
 * ring drained BEFORE the large 'R' data ring so TX terminal notifications
 * ('G'/'E') can never be dropped or head-of-line blocked by RX bursts.
 *
 * Both rings use monotonic uint8_t write/read indices (depths are powers of
 * 2), eliminating the wasted sentinel slot of classic head/tail rings, and
 * usb_tx_thread streams directly from the ring slot before advancing read_idx
 * (eliminating the redundant 12.4KB scratch buffer).
 * ------------------------------------------------------------------------- */
#define CTRL_TX_QUEUE_DEPTH 8
#define CTRL_TX_SLOT_BYTES  80      /* 'S' frame is 71B; 'G' is 4B; 'E' <= 32B */
static uint8_t  g_ctrl_tx_buf[CTRL_TX_QUEUE_DEPTH][CTRL_TX_SLOT_BYTES];
static uint16_t g_ctrl_tx_len[CTRL_TX_QUEUE_DEPTH];
static volatile uint8_t g_ctrl_tx_write_idx = 0;
static volatile uint8_t g_ctrl_tx_read_idx  = 0;

#define HOST_TX_QUEUE_DEPTH 2
#define HOST_TX_SLOT_BYTES  ( FLRC_BURST_MAX_TOTAL_PAYLOAD + 16 ) /* 3B 'R' hdr + payload + 2B RSSI + align */
static uint8_t  g_host_tx_buf[HOST_TX_QUEUE_DEPTH][HOST_TX_SLOT_BYTES];
static uint16_t g_host_tx_len[HOST_TX_QUEUE_DEPTH];
static volatile uint8_t g_host_tx_write_idx = 0;
static volatile uint8_t g_host_tx_read_idx  = 0;
static struct k_sem g_host_tx_sem;

static K_THREAD_STACK_DEFINE( usb_tx_thread_stack, 2048 );
static struct k_thread usb_tx_thread_cb;

/* Crashlog captured at boot (panic text from .noinit), emitted on first CONFIG.
 * The only remaining 'D' diagnostic frames — the event stream is gone. */
static uint8_t  g_crashlog_buf[CRASH_LOG_SIZE];
static uint8_t  g_crashlog_len = 0;

/* Lossless 9-byte-slot diagnostic ring: overflow overwrites the OLDEST event
 * (never silently drops data frames — events have their own ring). */
#define DBG_RING_DEPTH 128
static uint8_t  g_dbg_ring[DBG_RING_DEPTH][9];
static uint16_t g_dbg_head = 0, g_dbg_tail = 0;
static struct k_mutex g_dbg_lock; /* two producers: main + usb_tx watchdog */

static int  usb_tx( const uint8_t* data, uint16_t len );
static void rac_post_callback( rp_status_t status );

static void send_debug_event( uint8_t event_id, uint32_t val )
{
    uint8_t frame[9] = { 'D', 6, 0, 0, 0, 0, 0, 0, 0 };
    frame[3] = event_id;
    sys_put_le32( val, &frame[4] );
    /* Two producers (main + usb_tx watchdog thread) — serialize. */
    k_mutex_lock( &g_dbg_lock, K_FOREVER );
    memcpy( g_dbg_ring[g_dbg_tail], frame, 9 );
    g_dbg_tail = ( uint16_t )( ( g_dbg_tail + 1 ) % DBG_RING_DEPTH );
    if( g_dbg_tail == g_dbg_head )
    {
        g_dbg_head = ( uint16_t )( ( g_dbg_head + 1 ) % DBG_RING_DEPTH );
    }
    k_mutex_unlock( &g_dbg_lock );
    k_sem_give( &g_host_tx_sem );
}

/* Main-loop heartbeat, stamped every superloop iteration; the usb_tx thread
 * resets the MCU when it stalls (see MAIN_LOOP_WATCHDOG_MS). */
static volatile uint32_t g_tx_done_ms = 0;
/* ── SCAN MODE (roaming, owner spec 2026-09-09) ── */
#define SCAN_MAX_CHANNELS 16
static volatile bool     g_scan_active    = false;
static uint32_t          g_scan_freqs[SCAN_MAX_CHANNELS];
static uint32_t          g_scan_dwell_ms  = 300;
static uint32_t          g_scan_dwell_until = 0;
static uint8_t           g_scan_count     = 0;
static volatile uint8_t  g_scan_idx       = 0;
static volatile uint16_t g_scan_frames    = 0;
static volatile uint16_t g_scan_consensus = 0;
static volatile int8_t   g_scan_best_rssi = -128;
static volatile uint32_t g_main_loop_heartbeat_ms = 0;
void smtc_rac_diag_event( uint16_t event, uint32_t val_ms )
{
    send_debug_event( event, val_ms );
}
/* RE-ARM ATTRIBUTION (2026-10-01): the host-measured POST-TX GAP medians
 * 1.7-2.5s while event 44 (TX_DONE → re-arm, measured here) medians
 * ~1.2s — the delay is on the MCU, but is it the RAC planner withholding
 * the unlock, or the superloop not running? Discriminate with a per-pass
 * counter: event 49 emits the number of superloop passes between TX_DONE
 * and the re-arm. Thousands of passes = the engine polled the whole time
 * (planner withheld the unlock — fix in the RAC/library); a handful =
 * the loop itself was stalled (fix in the superloop's other work). */
static volatile uint32_t g_engine_pass_count = 0;
static volatile uint32_t g_tx_done_pass      = 0;
/* DEAFNESS INSTRUMENTATION (2026-09-07): TX_DONE instant and the re-arm
 * completion instant, both in k_uptime ms — the delta (event 44 minus
 * event 43 on the host) is the measured turnaround. */
/* The one watchdog that stays: a frozen superloop is invisible to the host
 * except as silence, and only this thread can act on it. Radio-state
 * recovery needs no watchdog anymore — the library holds no state that can
 * strand outside the RAC. */
#define MAIN_LOOP_WATCHDOG_MS 5000

static void queue_ctrl_tx( const uint8_t* p1, uint16_t l1, const uint8_t* p2, uint16_t l2 );
static void queue_host_tx( const uint8_t* p1, uint16_t l1, const uint8_t* p2, uint16_t l2,
                            const uint8_t* p3, uint16_t l3 );

static void usb_tx_thread( void* p1, void* p2, void* p3 )
{
    ARG_UNUSED( p1 ); ARG_UNUSED( p2 ); ARG_UNUSED( p3 );

    while( true )
    {
        /* Bounded wait: this thread is also the main-loop watchdog, so it
         * must wake even with no host traffic. */
        ( void ) k_sem_take( &g_host_tx_sem, K_MSEC( 500 ) );
        if( g_radio_ok && g_main_loop_heartbeat_ms != 0 )
        {
            /* Snapshot the heartbeat BEFORE reading the clock: the
             * superloop updates the heartbeat every fast iteration, so
             * reading the clock first allowed an update to land between
             * the two loads — now - heartbeat wrapped to 0xFFFFFFFF and
             * passed the >5000 test, cold-rebooting the MCU spuriously
             * every few minutes (bench 2026-09-08: all 293 event-30s
             * carried val=0xFFFFFFFF; a genuine stall reports the true
             * ms). Heartbeat-first guarantees now >= heartbeat. */
            uint32_t hb_ms = g_main_loop_heartbeat_ms;
            uint32_t now_wd = k_uptime_get_32( );
            uint32_t stalled_ms = now_wd - hb_ms;
            if( stalled_ms > MAIN_LOOP_WATCHDOG_MS )
            {
                LOG_ERR( "WATCHDOG: superloop stalled %ums — resetting MCU",
                         ( unsigned ) stalled_ms );
                send_debug_event( 30, stalled_ms );
                /* Flush the marker before dying: usb_tx is synchronous. */
                while( g_dbg_head != g_dbg_tail )
                {
                    uint16_t nwd = g_dbg_head;
                    ( void ) usb_tx( g_dbg_ring[nwd], 9 );
                    g_dbg_head = ( uint16_t )( ( g_dbg_head + 1 ) % DBG_RING_DEPTH );
                }
                sys_reboot( SYS_REBOOT_COLD );
            }
        }
        while( g_dbg_head != g_dbg_tail )
        {
            uint16_t n = g_dbg_head;
            ( void ) usb_tx( g_dbg_ring[n], 9 );
            g_dbg_head = ( uint16_t )( ( g_dbg_head + 1 ) % DBG_RING_DEPTH );
        }
        while( g_ctrl_tx_read_idx != g_ctrl_tx_write_idx )
        {
            uint8_t  slot   = g_ctrl_tx_read_idx % CTRL_TX_QUEUE_DEPTH;
            uint16_t tx_len = g_ctrl_tx_len[slot];
            ( void ) usb_tx( g_ctrl_tx_buf[slot], tx_len );
            __DMB( );
            g_ctrl_tx_read_idx++;
        }
        while( g_host_tx_read_idx != g_host_tx_write_idx )
        {
            uint8_t  slot   = g_host_tx_read_idx % HOST_TX_QUEUE_DEPTH;
            uint16_t tx_len = g_host_tx_len[slot];
            /* Stream directly from the ring slot and release the slot only
             * AFTER usb_tx completes. With monotonic indices, the producer
             * sees the slot as occupied until g_host_tx_read_idx advances. */
            ( void ) usb_tx( g_host_tx_buf[slot], tx_len );
            __DMB( );
            g_host_tx_read_idx++;
            /* Drain any control frames ('G'/'E'/'S') that were queued while
             * the large 'R' frame was streaming over UART. */
            while( g_ctrl_tx_read_idx != g_ctrl_tx_write_idx )
            {
                uint8_t  cslot = g_ctrl_tx_read_idx % CTRL_TX_QUEUE_DEPTH;
                uint16_t clen  = g_ctrl_tx_len[cslot];
                ( void ) usb_tx( g_ctrl_tx_buf[cslot], clen );
                __DMB( );
                g_ctrl_tx_read_idx++;
            }
        }
    }
}

/* ---------------------------------------------------------------------------
 * USB (CDC UART) I/O
 * ------------------------------------------------------------------------- */

/* Interrupt-driven UART RX ring: poll-mode RX has ~6 bytes of buffering; any
 * main-loop window longer than that silently drops bytes. The ISR drains the
 * FIFO into this ring; usb_rx_poll() consumes it (hw_modem sample pattern).
 * Single producer (ISR) / single consumer (main loop). */
#define UART_RX_RING_SIZE 8192 /* power of 2 */
static uint8_t  g_rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t g_rx_ring_head = 0;
static volatile uint16_t g_rx_ring_tail = 0;
static volatile uint32_t g_rx_ring_dropped = 0;
/* Wall time of the last byte the ISR accepted — the parser's idle reset keys
 * off ARRIVAL, not consumption, so a superloop stall can't eat a half-parsed
 * frame whose bytes sit intact in the ring. */
static volatile uint32_t g_rx_last_arrival_ms = 0;
static volatile uint32_t g_parser_frames_killed = 0;

static void uart_rx_isr( const struct device* dev, void* user_data )
{
    ARG_UNUSED( user_data );

    if( !uart_irq_update( dev ) )
    {
        return;
    }

    while( uart_irq_rx_ready( dev ) )
    {
        uint8_t b;
        int n = uart_fifo_read( dev, &b, 1 );
        if( n <= 0 )
        {
            break;
        }
        uint16_t next = ( g_rx_ring_head + 1 ) & ( UART_RX_RING_SIZE - 1 );
        if( next == g_rx_ring_tail )
        {
            g_rx_ring_dropped++;
            break;
        }
        g_rx_ring[g_rx_ring_head] = b;
        g_rx_ring_head = next;
        g_rx_last_arrival_ms = k_uptime_get_32( );
    }
}

static int usb_init( void )
{
    if( !device_is_ready( cdc_dev ) ) return -ENODEV;

    uart_irq_callback_user_data_set( cdc_dev, uart_rx_isr, NULL );
    uart_irq_rx_enable( cdc_dev );

    return 0;
}

static int usb_tx( const uint8_t* data, uint16_t len )
{
    if( !device_is_ready( cdc_dev ) ) return -ENODEV;
    for( uint16_t i = 0; i < len; i++ )
    {
        uart_poll_out( cdc_dev, data[i] );
        if( ( i & 0x7F ) == 0x7F )
        {
            k_busy_wait( 100 );
        }
    }
    return 0;
}

/* Stage a small control frame ('G', 'E', 'S') into the dedicated control ring. */
static void queue_ctrl_tx( const uint8_t* seg1, uint16_t len1,
                            const uint8_t* seg2, uint16_t len2 )
{
    uint16_t total = ( uint16_t )( len1 + len2 );
    if( total > CTRL_TX_SLOT_BYTES )
    {
        g_stats.host_tx_dropped++;
        return;
    }

    if( ( uint8_t )( g_ctrl_tx_write_idx - g_ctrl_tx_read_idx ) >= CTRL_TX_QUEUE_DEPTH )
    {
        g_stats.host_tx_dropped++;
        return;
    }

    uint8_t  slot = g_ctrl_tx_write_idx % CTRL_TX_QUEUE_DEPTH;
    uint8_t* dst  = g_ctrl_tx_buf[slot];
    uint16_t off  = 0;
    if( len1 ) { memcpy( dst + off, seg1, len1 ); off += len1; }
    if( len2 ) { memcpy( dst + off, seg2, len2 ); off += len2; }
    g_ctrl_tx_len[slot] = off;
    __DMB( );
    g_ctrl_tx_write_idx++;
    k_sem_give( &g_host_tx_sem );
}

/* Stage a large data frame ('R') for deferred UART transfer: up to three
 * segments concatenated into one ring slot. Never blocks; drops (counted)
 * when full. */
static void queue_host_tx( const uint8_t* seg1, uint16_t len1,
                            const uint8_t* seg2, uint16_t len2,
                            const uint8_t* seg3, uint16_t len3 )
{
    uint16_t total = ( uint16_t )( len1 + len2 + len3 );
    if( total > HOST_TX_SLOT_BYTES )
    {
        g_stats.host_tx_dropped++;
        return;
    }

    if( ( uint8_t )( g_host_tx_write_idx - g_host_tx_read_idx ) >= HOST_TX_QUEUE_DEPTH )
    {
        g_stats.host_tx_dropped++;
        return;
    }

    uint8_t  slot = g_host_tx_write_idx % HOST_TX_QUEUE_DEPTH;
    uint8_t* dst  = g_host_tx_buf[slot];
    uint16_t off  = 0;
    if( len1 ) { memcpy( dst + off, seg1, len1 ); off += len1; }
    if( len2 ) { memcpy( dst + off, seg2, len2 ); off += len2; }
    if( len3 ) { memcpy( dst + off, seg3, len3 ); off += len3; }
    g_host_tx_len[slot] = off;
    __DMB( );
    g_host_tx_write_idx++;
    k_sem_give( &g_host_tx_sem );
}

static void send_ready( void )
{
    /* v2 frame: [0x47][max_payload:u16 LE][FLRC_PROTOCOL_VERSION]. The
     * trailing version byte is how the host learns CONFIG v2 selectors are
     * safe to send (see flrc_config.h). Pre-v2 hosts read only the first
     * 3 bytes and ignore the tail. */
    uint8_t pkt[4];
    pkt[0] = FLRC_MSG_READY;
    pkt[1] = ( uint8_t )( g_pkt_payload & 0xFF );
    pkt[2] = ( uint8_t )( ( g_pkt_payload >> 8 ) & 0xFF );
    pkt[3] = FLRC_PROTOCOL_VERSION;
    queue_ctrl_tx( pkt, sizeof( pkt ), NULL, 0 );
}

static void send_error( uint8_t code, const char* msg )
{
    uint8_t msg_len = msg ? ( uint8_t ) strlen( msg ) : 0;
    uint8_t header[3] = { FLRC_MSG_ERROR, code, msg_len };
    queue_ctrl_tx( header, 3, ( const uint8_t* ) msg, msg_len );
}

/* Deliver a whole reassembled payload to the host: 'R' + len + payload + rssi */
static void send_rx_payload( const uint8_t* payload, uint16_t len, int16_t rssi )
{
    uint8_t header[3] = {
        FLRC_MSG_RX,
        ( uint8_t )( len & 0xFF ),
        ( uint8_t )( ( len >> 8 ) & 0xFF )
    };
    uint8_t rssi_buf[2] = {
        ( uint8_t )( rssi & 0xFF ),
        ( uint8_t )( ( rssi >> 8 ) & 0xFF )
    };
    queue_host_tx( header, 3, payload, ( len > 0 ) ? len : 0, rssi_buf, 2 );
}

static void send_stats_packet( void )
{
    uint8_t header[3] = { FLRC_MSG_STATS, 68, 0 };
    uint8_t stats_bytes[68] = { 0 };
    sys_put_le32( g_stats.bursts_tx, &stats_bytes[0] );
    sys_put_le32( g_stats.bursts_rx, &stats_bytes[4] );
    sys_put_le32( g_stats.packets_tx, &stats_bytes[8] );
    sys_put_le32( g_stats.packets_rx, &stats_bytes[12] );
    sys_put_le32( g_stats.crc_errors, &stats_bytes[16] );
    sys_put_le32( g_stats.host_tx_dropped, &stats_bytes[20] );
    stats_bytes[24] = ( uint8_t ) g_burst_mode;        /* 0=RX, 1=ABORTING, 2=TX */
    stats_bytes[25] = 0;                               /* was LBT retry count (LBT removed) */
    stats_bytes[26] = g_pending_tx_valid ? 1 : 0;      /* TX payload pending? */
    stats_bytes[27] = g_restart_rx_pending ? 1 : 0;    /* RX restart needed? */
    sys_put_le32( 0, &stats_bytes[28] );               /* was backoff-remaining (LBT removed) */
    sys_put_le32( k_uptime_get_32( ), &stats_bytes[32] );
    sys_put_le32( g_rx_ring_dropped, &stats_bytes[36] );
    sys_put_le32( g_staging_overwrites, &stats_bytes[40] );
    sys_put_le32( g_stats.burst_id_conflicts, &stats_bytes[44] );
    sys_put_le32( g_hw_rx_stats.received_packets, &stats_bytes[48] );
    sys_put_le32( g_hw_rx_stats.crc_errors, &stats_bytes[52] );
    sys_put_le32( g_hw_rx_stats.length_errors, &stats_bytes[56] );
    sys_put_le32( g_hw_rx_stats.crc_ok, &stats_bytes[60] );
    sys_put_le32( g_parser_frames_killed, &stats_bytes[64] );
    queue_ctrl_tx( header, 3, stats_bytes, sizeof( stats_bytes ) );
}

/* ---------------------------------------------------------------------------
 * FLRC burst RX — the example's lifecycle on our in-band framing
 * ------------------------------------------------------------------------- */

static void start_burst_rx( void )
{
    g_rac_ctx->modulation_type = SMTC_RAC_MODULATION_FLRC_BURST;

    smtc_rac_radio_flrc_burst_params_t* p = &g_rac_ctx->radio_params.flrc_burst;
    p->is_tx                     = false;
    p->frequency_in_hz           = g_cfg.freq_hz;
    p->rx_frequency_offset_in_hz = 0;
    p->raw_bit_rate              = cfg_raw_bit_rate( );
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
    /* 60s margin made the armed timeout (ToA+interframes+margin+10%)
     * ~66s — longer than the host's 15s TX watchdog, so a wedged TX was
     * always MCU-reset before the timeout could fire (SuperLexicon
     * 2026-09-05). 2s covers the slowest legit burst (~150ms at 1.3Mbps
     * 252B) many times over while actually bounding the wedge.
     * 30000 (2026-09-14): event-44 telemetry showed the armed timeout
     * cycling the RX every ~2.2 s of silence after each TX (clusters at
     * 2.2/4.7/6.2 s = 1-3 timeout re-arms), and each re-arm transition
     * is a deaf window that ate exactly one peer proposal/vote per
     * window. A whole quiet window is NORMAL silence now (the host
     * gates its own airs) — keep the transaction armed through it. The
     * wedge bound moves to the host TX watchdog (15 s) + this timeout:
     * a genuinely wedged RX still recovers within 30 s. */
    p->rx_burst_timeout_margin_ms = 30000;
    /* Inter-packet pacing: 2ms — 9x the flrc_burst example's 220us, ample
     * for the memcpy-scale RX callback + 250us superloop poll now that the
     * cut-over removed the serial-drain stalls that motivated the old 20ms
     * (the chip's 511B FIFO holds the next packet across the callback; the
     * burst ends with rx_done + a fresh arm). Owner directive 2026-09-02
     * set 20ms against the OLD architecture; air time is now 1.64ms/packet
     * + 2ms, making a 53-packet max burst 191ms instead of 1127ms.
     * TX and RX transactions must MATCH. */
    /* INTERFRAME PACING: the flrc_burst reference sample's 220us (owner
     * directive, 2026-09-28: base the pacing on the reference). The old
     * 2ms was sized for the pre-cut-over architecture's serial-drain
     * stalls; the RX callback per packet is memcpy-scale + the 250us
     * superloop poll, well inside 220us. Halves every burst's air time. */
    p->min_interframe_delay_us   = 220;

    g_rac_ctx->smtc_rac_data_buffer_setup.rx_payload_buffer = g_rac_rx_pkt_buf;
    g_rac_ctx->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer =
        sizeof( g_rac_rx_pkt_buf );

    g_rac_ctx->scheduler_config.scheduling    = SMTC_RAC_ASAP_TRANSACTION;
    g_rac_ctx->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms( );
    g_rac_ctx->scheduler_config.callback_post_radio_transaction = rac_post_callback;
    g_rac_ctx->scheduler_config.callback_pre_radio_transaction  = NULL;

    g_burst_mode = BURST_RX;

    if( smtc_rac_submit_radio_transaction( g_radio_access_id ) != SMTC_RAC_SUCCESS )
    {
        LOG_ERR( "start_burst_rx: submit failed" );
        g_restart_rx_pending = true; /* retry next superloop pass */
    }
}

/* Assemble one received air packet into the burst slot; deliver + re-arm on
 * burst completion. Runs in the RAC callback — memcpy-scale work only, the
 * host write goes through the deferred ring. */
static void on_rx_packet( void )
{
    /* SCAN counting (roaming): during a sweep, every received frame is
     * a channel-activity sample; SLRF magic marks consensus traffic. */
    if( g_scan_active )
    {
        g_scan_frames++;
        uint16_t scan_size = g_rac_ctx->smtc_rac_data_result.rx_size;
        int16_t  scan_rssi = g_rac_ctx->smtc_rac_data_result.rssi_result;
        const uint8_t* scan_buf = g_rac_ctx->smtc_rac_data_buffer_setup.rx_payload_buffer;
        if( scan_rssi > g_scan_best_rssi )
        {
            g_scan_best_rssi = ( int8_t ) scan_rssi;
        }
        if( scan_size >= 4 && scan_buf[0] == 0x53 && scan_buf[1] == 0x4c &&
            scan_buf[2] == 0x52 && scan_buf[3] == 0x46 )
        {
            g_scan_consensus++;
        }
        else if( scan_size >= 2 && scan_buf[0] == 0x42 && scan_buf[1] == 0x46 )
        {
            /* Chunked consensus burst: multi-packet frames air as [FB]
             * header chunks (magic 0x4642 LE = 'B','F'), the SLRF prefix
             * only surviving on the reassembled payload — count chunks
             * too or long block bursts read as plain activity. */
            g_scan_consensus++;
        }
        return; /* scan frames are not consensus traffic */
    }
    g_stats.packets_rx++;

    uint16_t rx_size = g_rac_ctx->smtc_rac_data_result.rx_size;
    int16_t  rssi    = g_rac_ctx->smtc_rac_data_result.rssi_result;
    uint8_t* rx_buf  = g_rac_ctx->smtc_rac_data_buffer_setup.rx_payload_buffer;

    if( rx_size < sizeof( flrc_packet_header_t ) || rx_size > sizeof( g_rac_rx_pkt_buf ) )
    {
        g_stats.crc_errors++;
        return;
    }

    flrc_packet_header_t hdr;
    memcpy( &hdr, rx_buf, sizeof( hdr ) );
    uint16_t magic         = sys_le16_to_cpu( hdr.magic );
    uint16_t burst_id      = sys_le16_to_cpu( hdr.burst_id );
    uint16_t packet_idx    = sys_le16_to_cpu( hdr.packet_idx );
    uint16_t total_packets = sys_le16_to_cpu( hdr.total_packets );
    uint32_t total_len     = sys_le32_to_cpu( hdr.total_payload_len );
    uint32_t payload_crc   = sys_le32_to_cpu( hdr.payload_crc32 );

    uint32_t expected_packets = ( total_len + g_pkt_chunk - 1 ) / g_pkt_chunk;
    if( magic != FLRC_BURST_HEADER_MAGIC || total_packets == 0 ||
        total_packets > MAX_AIR_PACKETS || total_len == 0 ||
        total_len > BURST_REASM_PAYLOAD_MAX || packet_idx >= total_packets ||
        total_packets != expected_packets )
    {
        g_stats.crc_errors++;
        return;
    }

    if( !g_rx_slot.active || burst_id != g_rx_slot.burst_id )
    {
        if( g_rx_slot.active )
        {
            /* Incomplete burst evicted by a new one — a chunk was lost. */
            g_stats.burst_id_conflicts++;
        }
        g_rx_slot.active            = true;
        g_rx_slot.burst_id          = burst_id;
        g_rx_slot.total_packets     = total_packets;
        g_rx_slot.received_count    = 0;
        g_rx_slot.total_payload_len = total_len;
        g_rx_slot.payload_crc32     = payload_crc;
        memset( g_rx_slot.received_mask, 0, sizeof( g_rx_slot.received_mask ) );
        g_rx_slot.start_ms          = k_uptime_get_32( );
        g_rx_slot.last_rssi         = rssi;
    }
    else if( total_packets != g_rx_slot.total_packets ||
             total_len != g_rx_slot.total_payload_len ||
             payload_crc != g_rx_slot.payload_crc32 )
    {
        /* Header mismatch against the slot it mapped to: cross-sender
         * burst_id collision. Drop the packet; the slot stays for its own
         * burst's remaining chunks. */
        g_stats.crc_errors++;
        return;
    }

    uint16_t mask_word = packet_idx >> 6;
    uint64_t mask_bit  = 1ULL << ( packet_idx & 63 );
    if( ( g_rx_slot.received_mask[mask_word] & mask_bit ) == 0 )
    {
        uint32_t offset    = ( uint32_t ) packet_idx * g_pkt_chunk;
        uint32_t space     = g_rx_slot.total_payload_len - offset;
        uint32_t chunk_len = rx_size - sizeof( flrc_packet_header_t );
        if( chunk_len > space )
        {
            chunk_len = space; /* last packet: ignore TX zero-padding */
        }
        memcpy( &g_rx_reasm_buf[offset], rx_buf + sizeof( flrc_packet_header_t ), chunk_len );
        g_rx_slot.received_mask[mask_word] |= mask_bit;
        g_rx_slot.received_count++;
    }
    g_rx_slot.last_rssi = rssi;

    if( g_rx_slot.received_count == g_rx_slot.total_packets )
    {
        if( flrc_burst_calc_crc32( g_rx_reasm_buf, g_rx_slot.total_payload_len ) ==
            g_rx_slot.payload_crc32 )
        {
            g_stats.bursts_rx++;
            send_rx_payload( g_rx_reasm_buf, ( uint16_t ) g_rx_slot.total_payload_len,
                             g_rx_slot.last_rssi );
            LOG_INF( "RX burst delivered: %u bytes, %u packets, burst_id=%u",
                     g_rx_slot.total_payload_len, g_rx_slot.total_packets, g_rx_slot.burst_id );
        }
        else
        {
            g_stats.crc_errors++;
            LOG_INF( "RX burst payload CRC mismatch: %u bytes, burst_id=%u",
                     g_rx_slot.total_payload_len, g_rx_slot.burst_id );
        }
        g_rx_slot.active = false;
        /* End the transaction so the next burst starts freshly armed —
         * RADIO_UNLOCKED follows and the superloop re-submits RX. This is
         * the example's re-arm discipline (smtc_rac_flrc_burst_rx_done is
         * documented callable from this callback context). */
        ( void ) smtc_rac_flrc_burst_rx_done( g_radio_access_id );
    }
}

/* ---------------------------------------------------------------------------
 * FLRC burst TX — library-streamed, FB-headered air packets
 * ------------------------------------------------------------------------- */

static void build_tx_air_packet( uint16_t pkt_idx, uint8_t* pkt )
{
    uint32_t payload_len = g_pending_tx_len;
    uint32_t offset      = ( uint32_t ) pkt_idx * g_pkt_chunk;
    uint32_t chunk_len   = ( payload_len - offset < g_pkt_chunk )
                               ? ( payload_len - offset )
                               : g_pkt_chunk;

    flrc_packet_header_t hdr;
    hdr.magic             = sys_cpu_to_le16( FLRC_BURST_HEADER_MAGIC );
    hdr.burst_id          = sys_cpu_to_le16( g_pending_burst_id );
    hdr.packet_idx        = sys_cpu_to_le16( pkt_idx );
    hdr.total_packets     = sys_cpu_to_le16( g_tx_total_packets );
    hdr.total_payload_len = sys_cpu_to_le32( payload_len );
    hdr.payload_crc32     = sys_cpu_to_le32( g_tx_payload_crc );

    memcpy( pkt, &hdr, sizeof( hdr ) );
    memcpy( pkt + sizeof( hdr ), g_pending_tx_buf + offset, chunk_len );
    if( sizeof( hdr ) + chunk_len < g_pkt_payload )
    {
        memset( pkt + sizeof( hdr ) + chunk_len, 0,
                g_pkt_payload - ( sizeof( hdr ) + chunk_len ) );
    }
}

static void prepare_next_tx_fragment( void )
{
    if( g_tx_bytes_sent >= g_tx_air_total_len ) return;

    uint16_t pkt_idx = g_tx_next_buf_idx;
    uint16_t buf_idx = pkt_idx % 2;

    build_tx_air_packet( pkt_idx, g_tx_fifo_buf[buf_idx] );
    g_rac_ctx->radio_params.flrc_burst.tx_fifo_payload_buffer[buf_idx] = g_tx_fifo_buf[buf_idx];
    g_tx_fifo_len[buf_idx] = g_pkt_payload;
    g_tx_bytes_sent += g_pkt_payload;
    g_tx_next_buf_idx++;
}

static void execute_burst_tx( void )
{
    uint32_t payload_len = g_pending_tx_len;
    if( payload_len == 0 )
    {
        g_restart_rx_pending = true;
        return;
    }

    /* STALL TRACE (bench 2026-09-08): a host-submitted frame vanished
     * with no 'G'/'E' terminal for 20s+. Event 46 marks the launch
     * attempt (mode<<8 | payload_len_hi), 47 the host-command accept
     * (staged=1/pending=0), 48 the superloop trigger firing. */
    send_debug_event( 46, ( ( uint32_t ) g_burst_mode << 8 ) |
                               ( ( payload_len >> 8 ) & 0xFF ) );

    g_tx_payload_crc   = flrc_burst_calc_crc32( g_pending_tx_buf, payload_len );
    g_tx_total_packets = ( payload_len + g_pkt_chunk - 1 ) / g_pkt_chunk;
    g_tx_air_total_len = ( uint32_t ) g_tx_total_packets * g_pkt_payload;

    LOG_INF( "Executing burst TX: %u bytes, %u packets, burst_id=%u", payload_len,
             g_tx_total_packets, g_pending_burst_id );

    g_rac_ctx->modulation_type = SMTC_RAC_MODULATION_FLRC_BURST;

    smtc_rac_radio_flrc_burst_params_t* p = &g_rac_ctx->radio_params.flrc_burst;
    p->is_tx             = true;
    p->frequency_in_hz   = g_cfg.freq_hz;
    p->tx_power_in_dbm   = g_cfg.tx_pwr_dbm;
    p->raw_bit_rate      = cfg_raw_bit_rate( );
    p->cr                = RAL_FLRC_CR_1_1;
    p->pulse_shape       = RAL_FLRC_PULSE_SHAPE_BT_05;
    p->preamble_len      = RAL_FLRC_PREAMBLE_LENGTH_32_BITS;
    p->sync_word_len     = RAL_FLRC_SYNCWORD_LENGTH_4_BYTES;
    p->tx_syncword_index = RAL_FLRC_TX_SYNCWORD_1;
    p->match_sync_word   = RAL_FLRC_RX_MATCH_SYNCWORD_1;
    p->pld_is_fix        = false;
    p->crc_type          = RAL_FLRC_CRC_2_BYTES;
    p->sync_word[0]      = default_syncword_1;
    p->sync_word[1]      = default_syncword_2;
    p->sync_word[2]      = default_syncword_3;
    p->crc_seed          = 0xFFFFFFFF;
    p->crc_polynomial    = 0x755B;
    /* Must match the TX transaction (owner directive, 2026-09-28:
     * reference-sample pacing). */
    p->min_interframe_delay_us = 220;
    p->max_rx_size       = g_pkt_payload;
    p->burst_tx_size     = g_tx_air_total_len;

    /* Prime the double-buffer FIFO. FIFO[1] must ALWAYS be registered: the
     * RAC validates both pointers non-NULL at submit time, and a zero length
     * is never consumed once burst_tx_size bytes are pushed. */
    g_tx_bytes_sent   = 0;
    g_tx_next_buf_idx = 0;
    p->size_of_tx_fifo_payload_buffer[0] = g_pkt_payload;
    p->size_of_tx_fifo_payload_buffer[1] = g_pkt_payload;

    build_tx_air_packet( 0, g_tx_fifo_buf[0] );
    p->tx_fifo_payload_buffer[0] = g_tx_fifo_buf[0];
    g_tx_fifo_len[0]             = g_pkt_payload;
    p->tx_fifo_payload_length[0] = &g_tx_fifo_len[0];
    g_tx_bytes_sent += g_pkt_payload;
    g_tx_next_buf_idx = 1;

    p->tx_fifo_payload_buffer[1] = g_tx_fifo_buf[1];
    g_tx_fifo_len[1]             = 0;
    p->tx_fifo_payload_length[1] = &g_tx_fifo_len[1];
    if( g_tx_bytes_sent < g_tx_air_total_len )
    {
        build_tx_air_packet( 1, g_tx_fifo_buf[1] );
        g_tx_fifo_len[1]  = g_pkt_payload;
        g_tx_bytes_sent  += g_pkt_payload;
        g_tx_next_buf_idx = 2;
    }

    g_rac_ctx->scheduler_config.scheduling    = SMTC_RAC_ASAP_TRANSACTION;
    g_rac_ctx->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms( );
    g_rac_ctx->scheduler_config.callback_post_radio_transaction = rac_post_callback;
    g_rac_ctx->scheduler_config.callback_pre_radio_transaction  = NULL;

    g_rac_ctx->lbt_context.lbt_enabled = false;

    g_burst_mode = BURST_TX;
    g_tx_airing  = false;
    g_tx_started_ms = k_uptime_get_32( );
    g_tx_requests = 0;
    g_tx_requests_expected =
        ( g_tx_air_total_len + g_pkt_payload - 1 ) / g_pkt_payload; /* total packets */
    if( g_tx_requests_expected >= 2 )
    {
        g_tx_requests_expected -= 2; /* the first two are pre-pushed, never requested */
    }
    else
    {
        g_tx_requests_expected = 0;
    }
    g_stats.bursts_tx++;

    if( smtc_rac_submit_radio_transaction( g_radio_access_id ) != SMTC_RAC_SUCCESS )
    {
        LOG_ERR( "execute_burst_tx: submit failed" );
        send_error( FLRC_ERR_TX_FAILED, "tx submit failed" );
        g_pending_tx_valid   = false;
        g_pending_tx_len     = 0;
        g_burst_mode         = BURST_RX;
        g_restart_rx_pending = true;
    }
}

/* ---------------------------------------------------------------------------
 * RAC post-transaction callback — the example's shape
 * ------------------------------------------------------------------------- */

static void rac_post_callback( rp_status_t status )
{
    switch( status )
    {
    case RP_STATUS_TX_DONE:
        g_tx_airing = true;
        g_tx_done_ms = k_uptime_get_32( );
        g_tx_done_pass = g_engine_pass_count;
        send_debug_event( 43, g_tx_done_ms ); /* TX_DONE instant (ms) */
        break;

    case RP_STATUS_RX_PACKET:
        on_rx_packet( );
        break;

    case RP_STATUS_REQUEST_NEXT_TX_PAYLOAD:
        g_stats.packets_tx++;
        g_tx_airing = true;
        g_tx_requests++;
        prepare_next_tx_fragment( );
        break;

    case RP_STATUS_RX_CRC_ERROR:
        g_stats.crc_errors++;
        break;

    case RP_STATUS_RX_TIMEOUT:
        /* Defensive: the RX transaction runs with an infinite timeout, so
         * this should never fire — but if it does, the transaction just
         * ended and must be re-armed immediately (reference ping-pong
         * discipline: chain the next op INSIDE the callback — the radio is
         * never left un-armed awaiting the superloop). */
        start_burst_rx( );
        send_debug_event( 44, k_uptime_get_32( ) - g_tx_done_ms ); /* re-arm turnaround */
        send_debug_event( 49, g_engine_pass_count - g_tx_done_pass ); /* passes since TX_DONE */
        break;

    case RP_STATUS_RADIO_UNLOCKED:
        if( g_burst_mode == BURST_TX_ABORTING )
        {
            /* RX aborted to free the radio for the queued TX. Release the
             * access and defer the TX submit to the superloop — a submit
             * from inside this callback reenters the engine. */
            ( void ) smtc_rac_flrc_burst_rx_done( g_radio_access_id );
            g_tx_rearm_pending = true;
        }
        else if( g_burst_mode == BURST_TX )
        {
            /* End of the TX transaction (the example's completion status). */
            g_tx_done_ms = k_uptime_get_32( );
            send_debug_event( 43, g_tx_done_ms ); /* TX_DONE instant (ms) */
            uint32_t ev_ms = 0;
            uint16_t ev_err = 0;
            bool     ev_valid = false;
            smtc_rac_flrc_burst_get_tx_fail_evidence( &ev_ms, &ev_err, &ev_valid );
            if( g_tx_airing && g_tx_requests >= g_tx_requests_expected )
            {
                send_ready( ); /* 'G' terminal: the burst aired */
            }
            else if( g_tx_airing )
            {
                /* Some packets aired but the burst ended early (FIFO
                 * underrun, error abort): report honestly so the host
                 * retries instead of trusting a half-aired burst. */
                if( ev_valid )
                {
                    send_debug_event( 42, ev_err ); /* chip error bits to the host */
                }
                send_error( FLRC_ERR_TX_DROPPED, "burst truncated" );
            }
            else if( ev_valid )
            {
                /* A real latched chip fault: definitively not sent. */
                send_debug_event( 41, ev_err );
                send_error( FLRC_ERR_TX_DROPPED, "tx never aired" );
            }
            else
            {
                /* NO chip evidence and nothing aired: release RX access and
                 * emit an explicit terminal so the host's TX_IN_FLIGHT window
                 * never leaks a credit when g_pending_tx_valid is cleared. */
                ( void ) smtc_rac_flrc_burst_rx_done( g_radio_access_id );
                send_error( FLRC_ERR_TX_DROPPED, "tx unlock no air" );
            }
            g_pending_tx_valid   = false;
            g_pending_tx_len     = 0;
            promote_staged_tx_if_ready( );
            g_burst_mode         = BURST_RX;
            /* CHAIN the RX re-arm inside the callback (reference ping-pong
             * discipline): the transaction is complete, the planner is idle,
             * and a submit here keeps the radio armed through whatever the
             * superloop is busy with (host forwarding of P1 bursts ran
             * 260ms+ each, stretching the un-armed gap to seconds of deaf
             * windows — bench 2026-09-07). start_burst_rx falls back to the
             * flag itself if the submit fails. */
            start_burst_rx( );
            /* DEAFNESS INSTRUMENTATION: turnaround = re-arm - TX_DONE, plus
             * the host-write ring pending depth at re-arm time. */
            send_debug_event( 44, k_uptime_get_32( ) - g_tx_done_ms );
            send_debug_event( 49, g_engine_pass_count - g_tx_done_pass ); /* passes since TX_DONE */
            {
                uint8_t pending = ( uint8_t )( g_host_tx_write_idx - g_host_tx_read_idx );
                send_debug_event( 45, pending );
            }
        }
        else
        {
            /* BURST_RX: the transaction ended after a completed burst
             * (rx_done) — re-arm a fresh one, chained as above. */
            start_burst_rx( );
            send_debug_event( 44, k_uptime_get_32( ) - g_tx_done_ms );
            send_debug_event( 49, g_engine_pass_count - g_tx_done_pass ); /* passes since TX_DONE */
        }
        break;

    case RP_STATUS_TASK_ABORTED:
        if( g_burst_mode == BURST_TX_ABORTING )
        {
            /* Same handoff as RADIO_UNLOCKED above (either termination
             * status may arrive depending on the abort timing). */
            ( void ) smtc_rac_flrc_burst_rx_done( g_radio_access_id );
            g_tx_rearm_pending = true;
        }
        else if( g_burst_mode == BURST_TX )
        {
            /* The TX task was aborted mid-flight: uncertain whether any
             * part aired. One terminal, then back to listening — chained
             * immediately per the reference discipline. */
            send_error( FLRC_ERR_TX_FAILED, "tx aborted" );
            g_pending_tx_valid   = false;
            g_pending_tx_len     = 0;
            g_burst_mode         = BURST_RX;
            start_burst_rx( );
        }
        else
        {
            start_burst_rx( );
        }
        break;

    default:
        break;
    }
}

/* ---------------------------------------------------------------------------
 * Host command handling
 * ------------------------------------------------------------------------- */

static void handle_config( const uint8_t* body, uint16_t body_len )
{
    /* SCAN MODE ENTRY (owner spec, 2026-09-09): freq_hz = 0xFFFFFFFF
     * tells the firmware to sweep the frequency grid and report results
     * via FW-DIAG events 50-54 instead of tuning to a specific channel.
     * The host reads the events from the serial log. */
    uint32_t req_freq = sys_get_le32( &body[0] );
    if( req_freq == 0xFFFFFFFF )
    {
        /* Enter scan mode: set the grid from the existing config fields
         * (bandwidth = step Hz, cr = channel count, pwr = dwell ms). */
        g_scan_freqs[0] = g_cfg.freq_hz;  /* home freq (return to after scan) */
        uint32_t start  = sys_get_le32( &body[10] );  /* sync word field = start Hz */
        uint32_t step   = sys_get_le32( &body[4] );    /* bw field = step Hz */
        uint8_t  count  = body[8];                     /* cr field = channel count */
        uint32_t dwell  = ( uint32_t ) body[9] * 100;  /* pwr field = dwell ms × 100 */
        if( count == 0 || count > SCAN_MAX_CHANNELS ) count = SCAN_MAX_CHANNELS;
        if( count > SCAN_MAX_CHANNELS ) count = SCAN_MAX_CHANNELS;
        for( uint8_t i = 0; i < count; i++ )
        {
            g_scan_freqs[i] = start + i * step;
        }
        g_scan_count = count;
        g_scan_dwell_ms = dwell;
        g_scan_idx = 0;
        g_scan_frames = 0;
        g_scan_consensus = 0;
        g_scan_best_rssi = -128;
        g_scan_dwell_until = k_uptime_get_32( ) + g_scan_dwell_ms;
        g_pending_tx_valid = false;
        g_burst_mode = BURST_RX;
        g_restart_rx_pending = true;
        g_scan_active = true;
        LOG_INF( "Scan mode: %u channels starting at %u Hz, dwell %u ms", count, start, dwell );
        return;
    }
    g_cfg.freq_hz    = req_freq;
    g_cfg.cr_host    = body[8];
    g_cfg.tx_pwr_dbm = ( int8_t ) body[9];
    g_cfg.sync_word  = sys_get_le32( &body[10] );
    g_cfg.role       = body[14];
    // CONFIG v2 (owner directive, 2026-09-05): byte 15 = raw-bit-rate
    // selector (0 = 1.3 Mbps, 1 = 2.6 Mbps), byte 16 = packet-payload
    // selector (0 = 252B, 1 = 511B) — both host-configurable at runtime.
    // A legacy 15-byte body keeps the current selections.
    if( body_len >= 16 )
    {
        g_cfg.rate_sel = ( body[15] <= 1 ) ? body[15] : g_cfg.rate_sel;
    }
    if( body_len >= 17 )
    {
        g_cfg.payload_sel = ( body[16] <= 1 ) ? body[16] : g_cfg.payload_sel;
        cfg_apply_selectors( );
    }

    /* First CONFIG after boot: deliver any crashlog captured at boot
     * (event 32 = 4 packed chars per chunk, 33 = done + total length). */
    if( g_crashlog_len > 0 )
    {
        for( uint8_t i = 0; i < g_crashlog_len; i += 4 )
        {
            uint8_t  n     = ( uint8_t )( ( i + 4 <= g_crashlog_len ) ? 4 : ( g_crashlog_len - i ) );
            uint32_t chunk = 0;
            for( uint8_t j = 0; j < n; j++ )
            {
                chunk |= ( uint32_t ) g_crashlog_buf[i + j] << ( 8 * j );
            }
            send_debug_event( 32, chunk );
        }
        send_debug_event( 33, g_crashlog_len );
        g_crashlog_len = 0;
    }

    g_restart_rx_pending = true;
}

/* USB parser state */
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

static volatile rx_state_t state     = RX_STATE_TAG;
static uint8_t              tag       = 0;
static uint8_t              scan_count = 0;
static uint8_t              scan_have  = 0;
static uint8_t              scan_buf[4 + SCAN_MAX_CHANNELS * 4];
static uint16_t             frame_len = 0;
static uint16_t             have      = 0;
static bool                 rx_drop_busy = false;
/* Bytes of a killed frame still to discard before resync: tag-scanning a
 * payload remainder can hit "TX"+bogus-length and swallow the next real
 * frame; the ring is FIFO and the remainder length is exact, so skipping it
 * restores tag alignment deterministically. */
static uint16_t             skip_remain = 0;
static uint8_t              config_body[FLRC_CONFIG_BODY_LEN_V2];
static uint8_t              window_body[8];

static void scan_emit_result( void )
{
    /* Results via FW-DIAG events (no new serial tags):
     * event 50 = freq Hz
     * event 51 = frames heard
     * event 52 = consensus frames
     * event 53 = best RSSI + 128 */
    send_debug_event( 50, g_scan_freqs[g_scan_idx] );
    send_debug_event( 51, g_scan_frames );
    send_debug_event( 52, g_scan_consensus );
    send_debug_event( 53, ( uint32_t )( ( int32_t ) g_scan_best_rssi + 128 ) );
}

/// One superloop pass of the frequency scan (owner spec, roaming):
/// dwell on the current channel (packets arrive via the RX callback and
/// are counted), then emit its result and retune+resubmit to the next.
static void scan_tick( void )
{
    uint32_t now = k_uptime_get_32( );
    if( ( int32_t )( now - g_scan_dwell_until ) < 0 )
    {
        return;
    }
    scan_emit_result( );
    g_scan_idx++;
    if( g_scan_idx >= g_scan_count )
    {
        g_scan_active    = false;
        g_burst_mode     = BURST_RX;
        g_restart_rx_pending = true;
        LOG_INF( "Scan complete: %u channels", g_scan_count );
        return;
    }
    g_cfg.freq_hz    = g_scan_freqs[g_scan_idx];
    g_scan_frames    = 0;
    g_scan_consensus = 0;
    g_scan_best_rssi = -128;
    g_scan_dwell_until = now + g_scan_dwell_ms;
    g_burst_mode     = BURST_RX;
    g_restart_rx_pending = true;
    LOG_INF( "Scan: ch%u @ %u Hz", g_scan_idx, g_cfg.freq_hz );
}

static int usb_rx_poll( void )
{
    /* Snapshot g_rx_last_arrival_ms BEFORE reading k_uptime_get_32(), and
     * compare with signed difference only when the RX ring is empty:
     * reading `now` first allowed uart_rx_isr to preempt between the two
     * loads and stamp g_rx_last_arrival_ms = now + 1, wrapping unsigned
     * (now - g_rx_last_arrival_ms) to 0xFFFFFFFF and killing active frames
     * mid-stream! */
    uint32_t last_arrival = g_rx_last_arrival_ms;
    uint32_t now          = k_uptime_get_32( );

    if( state != RX_STATE_TAG && g_rx_ring_tail == g_rx_ring_head &&
        ( int32_t )( now - last_arrival ) > 1500 )
    {
        if( tag == FLRC_MSG_TX &&
            ( state == RX_STATE_LEN_LO || state == RX_STATE_LEN_HI ||
              state == RX_STATE_PAYLOAD ) )
        {
            g_parser_frames_killed++;
            send_error( FLRC_ERR_TX_DROPPED, "rx timeout" );
        }
        else if( state == RX_STATE_CONFIG || state == RX_STATE_SET_WINDOW ||
                 state == RX_STATE_LEN_HI )
        {
            g_parser_frames_killed++;
        }
        skip_remain  = 0;
        rx_drop_busy = false;
        state        = RX_STATE_TAG;
        have         = 0;
    }

    uint8_t b;
    while( g_rx_ring_tail != g_rx_ring_head )
    {
        b = g_rx_ring[g_rx_ring_tail];
        g_rx_ring_tail = ( g_rx_ring_tail + 1 ) & ( UART_RX_RING_SIZE - 1 );

        if( skip_remain > 0 )
        {
            skip_remain--;
            continue;
        }
        switch( state )
        {
        case RX_STATE_TAG:
            tag = b;
            if( tag == FLRC_MSG_TX )
            {
                state = RX_STATE_TX_MAGIC;
                have  = 0;
            }
            else if( tag == FLRC_MSG_SET_WINDOW )
            {
                state = RX_STATE_SET_WINDOW;
                have  = 0;
            }
            else if( tag == FLRC_MSG_CONFIG )
            {
                state = RX_STATE_LEN_LO;
                have  = 0;
            }
            else if( tag == FLRC_MSG_STATS )
            {
                send_stats_packet( );
            }
            else if( tag == FLRC_MSG_SCAN )
            {
                state = RX_STATE_SCAN_COUNT;
                have  = 0;
            }
            else if( tag == FLRC_MSG_RESET )
            {
                /* ACK directly (the deferred queue would be lost on reboot),
                 * then cold-reset the MCU. */
                LOG_INF( "Reset requested by host — rebooting" );
                uint8_t ack = FLRC_MSG_READY;
                usb_tx( &ack, 1 );
                k_busy_wait( 10000 );
                NVIC_SystemReset( );
            }
            break;

        case RX_STATE_SCAN_COUNT:
            scan_count = b;
            scan_have = 0;
            if( scan_count == 0 || scan_count > SCAN_MAX_CHANNELS )
            {
                state = RX_STATE_TAG;
            }
            else
            {
                state = RX_STATE_SCAN_BODY;
            }
            break;

        case RX_STATE_SCAN_BODY:
            scan_buf[scan_have++] = b;
            if( scan_have >= 4 + ( uint16_t ) scan_count * 4 )
            {
                /* 'Q' [count][dwell le32][freq le32 × count] */
                g_scan_dwell_ms =
                    ( uint32_t ) scan_buf[1] | ( ( uint32_t ) scan_buf[2] << 8 ) |
                    ( ( uint32_t ) scan_buf[3] << 16 ) | ( ( uint32_t ) scan_buf[4] << 24 );
                if( g_scan_dwell_ms < 50 )
                {
                    g_scan_dwell_ms = 50;
                }
                g_scan_count = scan_count;
                for( uint8_t i = 0; i < scan_count; i++ )
                {
                    const uint8_t* fp = &scan_buf[5 + i * 4];
                    g_scan_freqs[i] = ( uint32_t ) fp[0] | ( ( uint32_t ) fp[1] << 8 ) |
                                      ( ( uint32_t ) fp[2] << 16 ) | ( ( uint32_t ) fp[3] << 24 );
                }
                g_scan_idx       = 0;
                g_scan_frames    = 0;
                g_scan_consensus = 0;
                g_scan_best_rssi = -128;
                g_cfg.freq_hz    = g_scan_freqs[0];
                g_scan_dwell_until = k_uptime_get_32( ) + g_scan_dwell_ms;
                g_pending_tx_valid = false;
                g_burst_mode     = BURST_RX;
                g_restart_rx_pending = true;
                g_scan_active    = true;
                LOG_INF( "Scan started: %u channels, %u ms dwell", scan_count, g_scan_dwell_ms );
                state = RX_STATE_TAG;
            }
            break;

        case RX_STATE_TX_MAGIC:
            if( b == 'X' )
            {
                state = RX_STATE_LEN_LO;
                have  = 0;
            }
            else
            {
                state = RX_STATE_TAG;
            }
            break;

        case RX_STATE_SET_WINDOW:
            window_body[have++] = b;
            if( have >= 8 )
            {
                g_slot_offset_ms = sys_get_le32( &window_body[0] );
                g_slot_period_ms = sys_get_le32( &window_body[4] );
                send_ready( );
                state = RX_STATE_TAG;
            }
            break;

        case RX_STATE_LEN_LO:
            frame_len = b;
            state     = RX_STATE_LEN_HI;
            break;

        case RX_STATE_LEN_HI:
            frame_len |= ( uint16_t )( b << 8 );
            have = 0;
            if( tag == FLRC_MSG_CONFIG )
            {
                // CONFIG v2 (owner directive, 2026-09-05): 17-byte bodies
                // carry the raw-bit-rate and packet-payload selectors;
                // legacy 15-byte bodies keep the current selections.
                if( frame_len != FLRC_CONFIG_BODY_LEN &&
                    frame_len != FLRC_CONFIG_BODY_LEN_V2 )
                {
                    send_error( FLRC_ERR_BAD_CONFIG, "bad len" );
                    state = RX_STATE_TAG;
                    break;
                }
                state = RX_STATE_CONFIG;
            }
            else /* FLRC_MSG_TX */
            {
                if( frame_len == 0 || frame_len > FLRC_BURST_MAX_TOTAL_PAYLOAD )
                {
                    send_error( FLRC_ERR_PAYLOAD_TOO_BIG, "bad tx len" );
                    state = RX_STATE_TAG;
                    break;
                }
                promote_staged_tx_if_ready( );
                rx_drop_busy = g_staging_tx_valid;
                state = RX_STATE_PAYLOAD;
            }
            break;

        case RX_STATE_PAYLOAD:
            if( have < frame_len )
            {
                if( !rx_drop_busy )
                {
                    g_staging_tx_buf[have] = b;
                }
                have++;
            }
            if( have >= frame_len )
            {
                /* Explicit rejection when staging is occupied (the host's
                 * 2-outstanding window makes this a host/firmware desync,
                 * never a routine event). Guarded by rx_drop_busy so an
                 * occupied g_staging_tx_buf is never overwritten in place. */
                if( rx_drop_busy || g_staging_tx_valid )
                {
                    rx_drop_busy = false;
                    g_staging_overwrites++;
                    send_error( FLRC_ERR_TX_BUSY, "tx queue full" );
                    state = RX_STATE_TAG;
                    break;
                }
                g_staging_tx_len = frame_len;
                /* PER-BURST RANDOM ID (owner directive, 2026-09-28): a
                 * fresh random 16-bit ID per accepted frame. Stored in
                 * g_staging_burst_id until promoted so an in-flight burst's
                 * g_pending_burst_id is never mutated mid-air. */
                sys_rand_get( &g_staging_burst_id, sizeof( g_staging_burst_id ) );
                if( g_staging_burst_id == 0 )
                {
                    g_staging_burst_id = 1;
                }
                g_staging_tx_valid = true;
                send_debug_event( 47, frame_len ); /* STALL TRACE: accepted */
                LOG_INF( "Host payload staged: %u bytes, burst_id=%u", g_staging_tx_len,
                         g_staging_burst_id );
                state = RX_STATE_TAG;
            }
            break;

        case RX_STATE_CONFIG:
            if( have < sizeof( config_body ) )
            {
                config_body[have++] = b;
            }
            if( have >= frame_len )
            {
                handle_config( config_body, frame_len );
                send_ready( );
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

/* ---------------------------------------------------------------------------
 * Init + main loop
 * ------------------------------------------------------------------------- */

static int init_radio( void )
{
    /* Enable the onboard RF switch power regulator */
#if DT_NODE_EXISTS(DT_PATH(rfsw_pwr))
    if( device_is_ready( rfsw_pwr_gpio.port ) )
    {
        gpio_pin_configure_dt( &rfsw_pwr_gpio, GPIO_OUTPUT_ACTIVE );
        LOG_INF( "Enabled RF switch power" );
    }
#endif
#if DT_NODE_EXISTS(DT_PATH(rfsw_ctl))
    if( device_is_ready( rfsw_ctl_gpio.port ) )
    {
        gpio_pin_configure_dt( &rfsw_ctl_gpio, GPIO_OUTPUT_INACTIVE );
    }
#endif

    SMTC_SW_PLATFORM_INIT( );
    SMTC_SW_PLATFORM_VOID( smtc_rac_init( ) );

    g_radio_access_id = SMTC_SW_PLATFORM( smtc_rac_open_radio( RAC_HIGH_PRIORITY ) );
    if( g_radio_access_id == 0xFF ) return -1;

    g_rac_ctx = smtc_rac_get_context( g_radio_access_id );
    if( g_rac_ctx == NULL ) return -1;

    g_rac_ctx->scheduler_config.callback_post_radio_transaction = rac_post_callback;

    /* TxOnly keeps an RX transaction armed as well: the RAC needs an active
     * transaction to abort when a TX is queued (and this bench has no
     * separate RX board in this role today). */
    start_burst_rx( );
    return 0;
}

int main( void )
{
    /* Capture a boot panic ("func:line expr" stashed in .noinit) and emit it
     * on the first CONFIG, so a panic-reboot loop names its own failure. */
    if( smtc_modem_hal_crashlog_get_status( ) )
    {
        smtc_modem_hal_crashlog_restore( g_crashlog_buf, &g_crashlog_len );
        smtc_modem_hal_crashlog_set_status( false );
        printk( "CRASHLOG(%u): %.*s\n", g_crashlog_len, g_crashlog_len,
                ( const char* ) g_crashlog_buf );
    }

    /* (The per-boot burst_id offset was removed 2026-09-28: burst ids
     * are now drawn fresh from the RNG per accepted frame, which makes
     * the per-boot offset redundant.) */

    LOG_INF( "LR2021 FLRC bridge booting (phase C, per-burst random burst ids)" );

    k_sem_init( &g_host_tx_sem, 0, K_SEM_MAX_LIMIT );
    k_mutex_init( &g_dbg_lock );
    k_thread_create( &usb_tx_thread_cb, usb_tx_thread_stack,
                     K_THREAD_STACK_SIZEOF( usb_tx_thread_stack ),
                     usb_tx_thread, NULL, NULL, NULL,
                     /* Below main: a 12KB envelope drains in ~123ms of
                      * blocking writes; at equal priority the timeslicer
                      * starred the radio loop's 2ms interframe servicing
                      * and the chip overran its RX FIFO mid-burst. */
                     K_PRIO_PREEMPT( 3 ), 0, K_NO_WAIT );
    k_thread_name_set( &usb_tx_thread_cb, "usb_tx" );

    if( usb_init( ) != 0 ) return 0;
    k_sleep( K_SECONDS( 1 ) );

    lr2021_deep_reset( );

    if( init_radio( ) == 0 )
    {
        g_radio_ok = true;
        send_ready( );
    }
    else
    {
        g_radio_ok = false;
        send_error( FLRC_ERR_RADIO_INIT, "radio init failed" );
    }

    uint32_t g_last_stats_emit_ms = 0;

    while( true )
    {
        g_main_loop_heartbeat_ms = k_uptime_get_32( );
        g_engine_pass_count++;
        promote_staged_tx_if_ready( );
        ( void ) usb_rx_poll( );

        if( g_radio_ok && g_scan_active )
        {
            /* SCAN MODE owns the radio: dwell/count per channel, no
             * consensus machinery until the sweep completes. */
            g_main_loop_heartbeat_ms = k_uptime_get_32( );
            scan_tick( );
            ( void ) smtc_rac_run_engine( );
        }

        if( g_radio_ok && !g_scan_active )
        {
            uint32_t now_ms = k_uptime_get_32( );

            /* Complete the abort->TX handoff deferred out of the callback. */
            if( g_tx_rearm_pending )
            {
                g_tx_rearm_pending = false;
                if( g_pending_tx_valid )
                {
                    execute_burst_tx( );
                }
                else
                {
                    g_burst_mode         = BURST_RX;
                    g_restart_rx_pending = true;
                }
            }
            else if( g_burst_mode == BURST_TX &&
                     ( now_ms - g_tx_started_ms ) > TX_INFLIGHT_TIMEOUT_MS )
            {
                /* In-flight TX watchdog (SuperLexicon 2026-09-05): the
                 * planner timeout normally bounds a TX, but any gap in
                 * that chain (timeout not armed, IRQ edge lost, planner
                 * stuck) previously left BURST_TX set forever — the host
                 * saw submits with zero completions until it MCU-reset
                 * us. Abort here; the RADIO_UNLOCKED callback reports
                 * 'G'/'E' honestly and RX re-arms. 20s = 3x the longest
                 * legit burst airtime at the slowest configured rate. */
                LOG_ERR( "TX in-flight watchdog: no completion in %u ms — aborting burst",
                         ( unsigned ) ( now_ms - g_tx_started_ms ) );
                ( void ) smtc_rac_abort_radio_submit( g_radio_access_id );
                g_burst_mode      = BURST_TX_ABORTING;
                g_abort_since_ms  = now_ms;
            }
            else if( g_burst_mode == BURST_TX_ABORTING &&
                     ( now_ms - g_abort_since_ms ) > 200 )
            {
                /* Aborting an already-finished task is a planner no-op with
                 * no callback; the access is free in that case — submit
                 * directly instead of waiting forever. */
                g_burst_mode = BURST_RX;
                if( g_pending_tx_valid )
                {
                    execute_burst_tx( );
                }
                else
                {
                    g_restart_rx_pending = true;
                }
            }

            if( g_restart_rx_pending && g_burst_mode == BURST_RX )
            {
                g_restart_rx_pending = false;
                start_burst_rx( );
            }

            /* Promote a completed staging frame so the host can pipeline the
             * next one while this airs. */
            promote_staged_tx_if_ready( );

            /* TX trigger: abort the live RX transaction; the completion
             * callback defers the actual submit back to this loop. */
            if( g_cfg.role != FLRC_ROLE_RX_ONLY && g_pending_tx_valid &&
                g_burst_mode == BURST_RX )
            {
                send_debug_event( 48, g_pending_tx_len ); /* STALL TRACE */
                g_burst_mode     = BURST_TX_ABORTING;
                g_abort_since_ms = now_ms;
                smtc_rac_abort_radio_submit( g_radio_access_id );
            }

            /* Evict a burst slot whose remaining chunks never arrived. */
            if( g_rx_slot.active && ( now_ms - g_rx_slot.start_ms ) > RX_SLOT_STALE_MS )
            {
                g_stats.burst_id_conflicts++;
                g_rx_slot.active = false;
            }

            ( void ) smtc_rac_run_engine( );

            /* Periodic stats every 10s, including the chip-level RX counters
             * (chip-heard vs app-delivered divergence is the starvation
             * signature, should now always be zero). */
            if( g_last_stats_emit_ms == 0 )
            {
                g_last_stats_emit_ms = now_ms;
            }
            else if( ( now_ms - g_last_stats_emit_ms ) >= 10000 )
            {
                g_last_stats_emit_ms = now_ms;
                lr20xx_radio_flrc_get_rx_stats( smtc_rac_get_radio_driver_context( ),
                                                &g_hw_rx_stats );
                send_stats_packet( );
            }
        }

        /* Sleep, not yield: the main thread is cooperative and the TX
         * consumer is preemptible — it only runs while we sleep. 250us
         * bounds the engine-poll gap and the RX-ring service latency. */
        k_usleep( 250 );
    }

    return 0;
}
