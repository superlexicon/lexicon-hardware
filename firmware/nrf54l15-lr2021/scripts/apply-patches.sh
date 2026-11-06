#!/usr/bin/env bash
# Re-apply the SuperLexicon dependency patches after `west update`.
#
# west manages usp/ and usp_zephyr/ at the manifest revisions (upstream
# release tags). Our fixes live in ../patches/ as format-patch files.
# Run this after any `west update` (or west init + update) so the build
# carries the lost-IRQ-edge resuscitation that the radio stack depends on.
#
# Safe to re-run: already-applied patches are detected and skipped.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIRMWARE_ROOT="$(dirname "$HERE")"
PATCHES="$FIRMWARE_ROOT/patches"

# Locate the west workspace topdir (parent of the firmware repo by default;
# override with WEST_TOP=... when the workspace lives elsewhere).
WEST_TOP="${WEST_TOP:-$(dirname "$FIRMWARE_ROOT")}"

apply_one() {
    local module="$1" patch="$2" sig="$3"
    local moddir="$WEST_TOP/modules/lib/$module"
    if [ ! -d "$moddir/.git" ]; then
        echo "✗ $module not found at $moddir — is this run from a west workspace?" >&2
        return 1
    fi
    # Skip when the patch's signature is already in the log. Signatures are
    # ASCII substrings (patch Subjects are MIME-encoded when they contain
    # non-ASCII, so matching the full title is unreliable).
    if git -C "$moddir" log --format=%s | grep -qF "$sig"; then
        echo "= $module: already applied ($sig)"
        return 0
    fi
    if git -C "$moddir" am --keep-cr "$patch" >/dev/null 2>&1; then
        echo "+ $module: applied $title"
    else
        git -C "$moddir" am --abort >/dev/null 2>&1 || true
        echo "✗ $module: failed to apply $(basename "$patch") — apply manually:" >&2
        echo "    git -C $moddir am '$patch'" >&2
        return 1
    fi
}

echo "Applying SuperLexicon dependency patches (workspace: $WEST_TOP)"
rc=0
apply_one usp         "$PATCHES/0001-usp-hal-declare-smtc_modem_hal_radio_irq_kick.patch" "declare smtc_modem_hal_radio_irq_kick" || rc=1
apply_one usp         "$PATCHES/0004-usp-radio-planner-run-arbiter-on-no-task-irq.patch" "run the arbiter on the no-task IRQ path" || rc=1
apply_one usp         "$PATCHES/0005-usp-rac-flrc-burst-tx-callback-reads-real-irq-status.patch" "TX callback reads the real IRQ status" || rc=1
apply_one usp         "$PATCHES/0006-usp-radio-planner-never-strand-enqueued-asap-task.patch" "never strand an enqueued ASAP task" || rc=1
apply_one usp         "$PATCHES/0007-usp-radio-planner-stop-tcxo-only-when-no-task-running.patch" "stop the TCXO only when no task is running" || rc=1
apply_one usp_zephyr  "$PATCHES/0002-usp_zephyr-hal-add-smtc_modem_hal_radio_irq_kick.patch" "add smtc_modem_hal_radio_irq_kick" || rc=1
apply_one usp_zephyr  "$PATCHES/0003-usp_zephyr-lr20xx-document-EDGE-triggering-rationale.patch" "document why EDGE" || rc=1

if [ "$rc" -eq 0 ]; then
    echo "✓ All dependency patches applied."
else
    exit "$rc"
fi
