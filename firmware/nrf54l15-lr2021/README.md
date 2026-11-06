# LR2021 FLRC Burst Firmware

Host-side firmware for the SuperLexicon blockchain's FLRC radio transport:
a burst-oriented bridge between a host (running the superlexicon node) and
a Semtech LR2021 radio, carrying consensus traffic over multi-point FLRC
broadcasts. The host-side counterpart is documented in the superlexicon
repo's `docs/architecture/02-transport.md`.

## Hardware & setup

- **Board**: Seeed Studio XIAO nRF54L15 (the nRF54L15 MCU wires the LR2021
  RAC through its radio peripheral; pin/RF-switch config in
  `overlay-xiao_nrf54l15_lr2021.dts`).
- **Host link**: the XIAO's USB-C routes through an **onboard USB-UART
  bridge on `uart20`** — this is a plain UART device, **NOT USB CDC ACM**.
  Enumerates as `/dev/ttyACM*`; runs at **1,000,000 baud** (the bridge's
  supported maximum; was 921600).
- **Toolchain**: Zephyr v4.3.0; modules per `west.yml` (upstream
  `usp_zephyr` v1.1.2-feature-202604; the **superlexicon usp fork**
  `lost-edge-resuscitation` @ `877ae25` — see `PATCHES.md`).
- **Flash**: `pyocd flash -t nrf54l -u <probe_id> bin/zephyr.hex` then
  `pyocd reset`. Probe IDs (CMSIS-DAP on the XIAO's debug USB):
  `071BAB41`, `7C1E0AEB`, `3033B527`. Non-root flashing requires udev
  rules applied (`sudo udevadm trigger` after adding the CMSIS-DAP/pyocd
  rules, or a replug).

## Architecture

- **RAC superloop**: the radio planner runs via `smtc_rac_run_engine()`
  polled from the main loop every ~250 µs (radio IRQ processing is
  ~0.5–1.5 ms; the poll gap bounds the RX-drain race — see pacing below).
- **Continuous RX transaction**: one long-lived RX transaction harvests
  packets into `RP_STATUS_RX_PACKET` callbacks across many bursts
  (`burst_rx_size` = full 24,576 B range, 60 s timeout margin; the ~66 s
  active timeout ends it via `RP_STATUS_TASK_ABORTED`, which re-arms RX).
  **Do not call `smtc_rac_flrc_burst_rx_done()` per completed burst** — it
  unlocks the transaction, and the restart gap silently ate ~12% of packets
  (counted by the chip, never surfaced) on every node, phase-correlated
  into stable per-direction loss.
- **Fire-and-forget windowed TX**: `'TX'` commands stage into a 2-deep
  queue (one airing, one staged); the RAC airs the staged burst
  autonomously with FIFO-fed fragments
  (`RP_STATUS_REQUEST_NEXT_TX_PAYLOAD`). A new frame arriving while both
  slots are full overwrites the staged one (counted in
  `g_staging_overwrites`).
- **Multi-slot out-of-order reassembly**: up to 4 concurrent reassembly
  slots keyed by the full tuple `(burst_id, total_packets, payload_len,
  payload_crc32)`. The tuple matters: burst IDs are per-node counters XOR
  a boot-random offset, so ID-only keying fed one sender's packets into
  another's incomplete slot and the burst rotted until timeout. Cross-sender
  ID collisions are counted in `burst_id_conflicts`.
- **Dedicated USB TX consumer thread**: the host-direction ring
  (`HOST_TX_QUEUE_DEPTH 3` × 12,400 B slots) is drained by a preemptible
  thread streaming to the UART — the old in-superloop drain starved the
  radio engine (143 ms stalls) and caused the post-TX receive blackout.
- **LBT/CSMA**: on busy channel, abort RX to seize the radio, retry with
  binary-exponential backoff (capped 64 ms, drop after 10 deferrals and
  re-arm RX).

## Host ↔ MCU wire protocol

Frames are tag-led; multi-byte integers little-endian.

| Tag | Name | Direction | Layout |
|-----|------|-----------|--------|
| `'G'` (0x47) | READY | MCU→host | `[max_payload:u16]` — sent after every TX completion and on CONFIG/SET_WINDOW ack |
| `'TX'` (0x54 0x58) | TRANSMIT | host→MCU | `[len:u16][payload]` — 2-byte header, NOT a single `'T'` |
| `'R'` (0x52) | RX | MCU→host | `[len:u16][payload][rssi:i16]` — one reassembled burst payload |
| `'C'` (0x43) | CONFIG | host→MCU | `[len:u16][15-byte body]` (below) → MCU responds `'G'` |
| `'W'` (0x57) | SET_WINDOW | host→MCU | `[8-byte body: slot_offset:u32, slot_period:u32]` (currently unused by the host) |
| `'S'` (0x53) | STATS | MCU→host¹ / host→MCU | periodic (10 s) or on-demand snapshot — 64-byte layout below |
| `'D'` (0x44) | DEBUG EVENT | MCU→host | `[len:u16][event_id:u8][val:u32]` — registry below |
| `'E'` (0x45) | ERROR | MCU→host | `[code:u8][msg_len:u8][msg]` |
| `'X'` (0x58) | RESET | host→MCU | cold reset (MCU acks `'G'` first) |

¹ stats are MCU-initiated every `STATS_EMIT_INTERVAL_MS` (10 s).

**CONFIG body (15 bytes)**: `frequency_hz:u32, bandwidth_hz:u32,
coding_rate:u8, tx_power_dbm:i8, sync_word:u32, role:u8` where role
0 = BOTH, 1 = TX_ONLY, 2 = RX_ONLY.

## Burst protocol (on-air)

Host payloads are split into FLRC packets, each carrying a 16-byte header
(`flrc_packet_header_t`): `magic (0x4642 'FB')`, `burst_id:u16`,
`packet_idx:u16`, `total_packets:u16`, `total_payload_len:u32`,
`payload_crc32:u32`. Packet payload chunk = 236 B (252 B on-air including
FLRC overhead); total payload cap 12,288 B. CRC is IEEE 802.3 CRC-32
(poly `0xEDB88320`) over the whole payload.

**`min_interframe_delay_us = 1200`**: TX pacing between packets of one
burst MUST exceed the receiver's engine-poll granularity (~250 µs–1 ms).
At 300 µs spacing, back-to-back packets completed within one poll interval
and the second died in the chip's RX buffer — counted by
`hw.received_packets` but never drained to a callback (a steady 1–4
packets/10 s loss per node, phase-correlated into per-direction loss).

## Radio parameters (must match the host's config.json)

| Parameter | Value |
|---|---|
| Frequency | 915,000,000 Hz |
| Bandwidth | 1,200,000 Hz |
| Coding rate | 1 |
| TX power | 22 dBm (firmware default 20 — host CONFIG overrides) |
| Sync word | `0x534C2021` |

Mismatched parameters produce a silently deaf link.

## Debug events (`'D'` frames)

Only low-frequency events (≥ 22) go on the wire — per-packet events would
flood the 3-deep host ring and drop real traffic.

| ID | Meaning |
|----|---------|
| 22 | TX-complete → RX-restart gap (ms) |
| 23 | run_engine stall > 20 ms (ms) |
| 24 | hw-received minus app-delivered packets this interval — packed `(gap << 16) \| tx_dist` where tx_dist = ms since last TX-complete (0xFFFF = none since boot) |
| 25 | LBT busy-channel deferral count |

## Stats frame (64 bytes, offsets)

| Offset | Field |
|--------|-------|
| 0 / 4 / 8 / 12 | bursts_tx / bursts_rx / packets_tx / packets_rx |
| 16 | crc_errors (app-level) |
| 20 | host_tx_dropped (host-direction ring full / frame too big) |
| 24–27 | burst_mode, tx_repeat_count, tx_pending, restart_rx_pending (1 B each) |
| 28 | ms until LBT backoff clears |
| 32 | MCU uptime ms |
| 36 | UART RX ring drops (must stay 0) |
| 40 | TX staging overwrites (must stay 0) |
| 44 | burst-id conflicts (tuple-keyed reassembly detected a cross-sender ID collision) |
| 48 / 52 / 56 / 60 | chip-level: hw received / hw CRC fail / hw length fail / hw CRC ok |

The `hwrx` vs `packets_rx` divergence (chip counted, callback never fired)
is the primary RF-health signal: zero divergence = every received packet
surfaced.

## Test scripts

Host-side bench scripts at the repo root: `test_flrc_burst_duplex.py`,
`test_sustained_burst.py`, `test_burst_broadcast.py`,
`benchmark_flrc_burst.py`, `measure_raw_speed.py`,
`test_tx_with_stats.py`, `test_simple_ping.py`. Note: the radio
parameters in the scripts must match the deployed config (a stale default
produces zero delivery — CR and sync word matter).


## Building from scratch (fresh machine)

Prerequisites: Zephyr SDK **0.17.4** at `~/zephyrproject/zephyr-sdk-0.17.4`,
`pip install west pyocd`, USB serial + debug probes.

```bash
./bootstrap.sh            # west workspace init (parent dir) + fetch all pinned modules
./build.sh                # pristine build → bin/zephyr.hex
./flash.sh [probe-id]     # pyocd flash + reset
```

- `build.sh --incremental` reuses the build directory.
- Pruning (proof-of-latest-state) limit: `--db-size-limit-mb <N>` is a
  NODE (superlexicon) flag, not a firmware flag.
- Dependency changes: `west.yml` pins; see `PATCHES.md` for the usp fork
  rationale and why the old patch workflow is retired.
