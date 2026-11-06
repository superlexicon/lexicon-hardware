# Dependency Changes — SUPERSEDED BY FORK PINS

As of 2026-09-09 the west manifest pins the `usp` module to the
**superlexicon fork** (`lost-edge-resuscitation` @ `877ae25`) instead of
the upstream release tag, and the fork carries the fixes that matter
(TX-fault visibility + evidence latch, abort-on-latched-fault, SetTx
retry on latched CHIP_BUSY, TX_DONE on the host line). `usp_zephyr` is
pinned to unmodified upstream `v1.1.2-feature-202604` (its two fork
commits were unused and withdrawn).

**Do NOT run `scripts/apply-patches.sh` against the current pins.** The
patch files below are retained as historical reference only:

- 0001–0003, 0010–0013 — superseded: equivalent changes are IN the
  pinned fork commits; re-applying is a detected no-op at best.
- 0004–0009 — radio_planner / smtc_rac fixes from the earlier
  firmware era. They are NOT in the pinned fork and are NOT validated
  against the current application; the bench (600+ blocks, 25 s
  cadence, 2026-09-09) runs without them. Do not apply without re-
  validating end-to-end.

Historical workflow (upstream tags + apply-patches.sh) is retained in
git history and in `scripts/apply-patches.sh` / `patches/` if the
fork-pin approach is ever reversed.
