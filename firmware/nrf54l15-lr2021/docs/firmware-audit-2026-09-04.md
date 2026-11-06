# Firmware Audit — 2026-09-04

Scope: `lr2021-flrc-firmware` (main.c 2180 lines, 90 functions, ~20
custom globals, 82-commit history) versus the owner-provided reference
implementation: the Semtech `flrc_burst` example
(`modules/lib/usp/examples/main_examples/flrc_burst_example/`, ~1077
lines total) on top of the RAC burst protocol library
(`smtc_rac_lib/smtc_rac/smtc_rac_flrc_burst.c`, 858 lines), and the
Zephyr sample (`usp_zephyr/samples/usp/rac/flrc_burst/src/main.c`,
298 lines).

---

## 1. The reference pattern (what the examples do)

- App layer is thin: init, poll `smtc_rac_run_engine()`, and on trigger
  call `flrc_burst_protocol_tx_start_new_transaction(data, size)` after
  checking the ready flag. That is the whole app.
- The burst protocol library owns everything we hand-rolled:
  - 2-deep TX FIFO ping-pong (`RAC_FLRC_BURST_NUMBER_OF_TX_FIFO_PTR
    = 2`), byte-push tracking into the radio FIFO;
  - TX/RX RAC callbacks, lock callbacks, and **post callbacks with
    `rp_status_t`** (the completion + status path);
  - `smtc_rac_flrc_burst_rx_done()` to re-arm RX after a reception;
  - chunk sequencing and FIFO reads/writes.
- Nothing in the example implements retry ladders, backoff timers,
  watchdog escalations, diagnostic event streams, or RX ring buffers.

## 2. What our main.c hand-rolls instead

main.c does not use the burst protocol library (the only library call
is `smtc_rac_flrc_burst_rx_done`, 5×; `flrc_burst_protocol.c` is a
27-line CRC stub). Instead it reimplements the protocol with ~20 custom
globals and layered experimental machinery:

| hand-rolled | globals / functions | library equivalent |
|---|---|---|
| TX FIFO ping-pong | `g_tx_buf`, `g_tx_fifo_len`, `g_tx_bytes_sent`, `g_tx_next_buf_idx`, `g_tx_len` | `tx_fifo[]` ping-pong in `smtc_rac_flrc_burst()` |
| ready / staging gate | `send_ready()`, staging counters (host `TX_IN_FLIGHT` mirrors it) | `flrc_burst_protocol_tx_is_ready` / 2-deep FIFO |
| completion | `rac_post_callback` + custom `'G'`/`'E'` host terminals | `post_tx_callback(rp_status)` → `is_successful` |
| RX re-arm after TX | `g_tx_rearm_pending`, `g_tx_only_rx_aborted` | `smtc_rac_flrc_burst_rx_done()` |
| retry | `g_tx_repeat_count`, `g_tx_backoff_until_ms` | none (the protocol re-submits per app) |
| RX buffering | `g_rx_ring`, `g_rx_ring_head/tail/dropped`, `g_rx_slots` | RX FIFO reads inside the callback |
| link-quality tracking | `g_rx_mode_bad_ticks`, `g_rx_last_rssi/ms` | none |
| recovery | watchdog escalations: launch-loss RAC recycle, inert-radio 3-strike, deaf-rx 2-strike, 1 s chip-mode probe, deep-NRST pulses | none (the library never wedges: it holds no state outside the RAC) |
| diagnostics | custom diag event stream (events 29/31/34/35/38/39/42) to the host | none |

Git history: 82 commits, with at least six distinct watchdog/escalation
mechanisms added, two reverted (priority class, verify-after-launch),
plus IRQ resuscitation polls and guards — the experimentation arc the
owner described.

## 3. Bug-class mapping (observed failures ↔ hand-rolled code)

| observed failure | our code path | library behavior |
|---|---|---|
| `stgovr` staging overruns | host/queue credit drift against the custom staging counters | the library's FIFO pointer arithmetic is the reference; no credit ledger exists to drift |
| terminal losses → 15 s stall → MCU reset | `TX_IN_FLIGHT` gate waiting `'G'`/`'E'` that never come | `post_tx_callback(rp_status)` fires from the RAC — completion cannot be silently lost (status is the payload) |
| deaf episodes (chip RX climbs, app receives nothing, hours) | `g_tx_rearm_pending` / `g_tx_only_rx_aborted` state holes after TX bursts; `pkilled=2` (firmware task kill) on the affected board | `smtc_rac_flrc_burst_rx_done()` re-arms RX inside the callback chain — no app-visible re-arm state to lose |
| vote-chunk-1 loss into the worst node | multi-frame TX sessions overrun slots; receiver ring drops (`g_rx_ring_dropped`) | single-frame votes (post refs) no longer stress this; the ring is our addition |
| MCU-cold-reset as the universal recovery | the reconnect loop's `firmware_reset_reopen` | the RAC recycle paths already in the vendor code |

## 4. Vendor patches (0001–0007) — KEEP, they are genuine upstream bugs

- 0001/0002 HAL irq-kick declarations — build/API fix.
- 0003 LR20xx EDGE-triggering documentation — documentation fix.
- 0004 planner runs the arbiter on the no-task IRQ path — real planner
  bug (enqueued ASAP RX never launched).
- 0005 burst TX callback reads the real IRQ status — real.
- 0006 planner never strands an enqueued ASAP task — real (the launch-
  loss wedge root cause).
- 0007 TCXO shutdown guarded when no task is running — real.

These are independent of our app code and should remain.

## 5. Categorization

**KEEP (necessities the bench protocol requires):**
- The host framing protocol: the SLRF/TX header, chunk sessions, and
  the `'G'`/`'E'` terminal notifications — the host application's
  transport contract.
- `flrc_config.h` (frequency/rate/sync config).
- Minimal counters for the host stats line (btx/brx/hwrx and friends).
- The vendor patches 0001–0007.

**REPLACE (hand-rolled protocol → the library):**
- The TX FIFO/staging/ready machinery → `smtc_rac_flrc_burst()`
  submission + `flrc_burst_protocol_tx_is_ready` pattern.
- The completion path → the library's post callbacks (map
  `rp_status` → keep the host `'G'`/`'E'` terminals).
- RX re-arm → `smtc_rac_flrc_burst_rx_done()`.

**DELETE (residual experimentation):**
- `g_tx_repeat_count` / `g_tx_backoff_until_ms` retry ladder.
- The watchdog escalation stack (launch-loss, inert-radio, deaf-rx
  2-strike, 1 s probes) — replaced by the library's non-wedging design;
  keep at most the existing FW stats counters for observability.
- The custom diag event stream (events 29/31/34/35/38/39/42).
- `g_rx_mode_bad_ticks` link-quality tracking.

## 6. Rewrite proposal (phased)

1. **Phase A — harness parity**: build the `flrc_burst` example target
   for our board; confirm it compiles/flashes against our patched
   USP tree (the patches ride the same modules).
2. **Phase B — host contract port**: wrap the library with a ~300-line
   app file that (a) frames the host protocol (SLRF/TX header, chunk
   sessions), (b) emits `'G'`/`'E'` terminals from the library's
   post-callback status (map `RP_STATUS_TX_DONE` → 'G', error statuses
   → 'E'), (c) keeps the stats counters, (d) keeps the host-triggered
   reset path.
3. **Phase C — cut-over**: main.c reduced to the sample pattern +
   host framing; delete the hand-rolled machinery (DELETE list). Bench
   validate: 3-node soak with the votes metric stable at 2, zero
   pipeline-stuck resets, zero deaf episodes, or a histogram signature
   that indicts hardware instead.
4. **Phase D — keep the app-side self-healing** (RX-histogram-empty →
   MCU reopen) as the only watchdog, sized at the app layer.

## 7. Risks

- The hand-roll exists because early dual-radio/duplex needs outran the
  example (history: "multi-slot reassembly, LBT/CSMA-CA, dual-radio
  roles"). If duplex or dual-radio is still required, confirm the
  library supports the needed configuration before deleting; the
  current bench is single-radio simplex-with-turns, which the library
  covers.
- The host `'G'`/`'E'` contract is load-bearing (the app's TX_IN_FLIGHT
  gate). Port it exactly; a dropped terminal is the 15 s stall.
- Chunk format on the air must stay identical or both firmware
  generations must be flashed together — the host does not decode the
  RF payload, so this is firmware-to-firmware only.
