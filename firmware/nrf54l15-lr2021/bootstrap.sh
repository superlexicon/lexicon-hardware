#!/usr/bin/env bash
# One-time workspace bootstrap (owner 2026-09-09): initialize the west
# workspace in the PARENT directory and fetch every pinned module
# (Zephyr v4.3.0, upstream usp_zephyr, the superlexicon usp fork).
set -e
cd "$(dirname "$0")"

if ! command -v west >/dev/null 2>&1; then
    echo "Installing west (pip install --user west)…"
    pip install --user west
fi

# Zephyr's multi-repo model: the workspace root is the PARENT directory.
if [ ! -f ../.west/westconfig.yml ] && [ ! -d ../.west ]; then
    west init -l .
fi
west update

# Environment expectations — verify and print what a build needs.
: "${ZEPHYR_BASE:=$PWD/../zephyr}"
: "${ZEPHYR_SDK_INSTALL_DIR:=$HOME/zephyrproject/zephyr-sdk-0.17.4}"
export ZEPHYR_BASE ZEPHYR_SDK_INSTALL_DIR ZEPHYR_TOOLCHAIN_VARIANT=zephyr
if [ ! -d "$ZEPHYR_SDK_INSTALL_DIR" ]; then
    echo "⚠️  Zephyr SDK not found at $ZEPHYR_SDK_INSTALL_DIR"
    echo "   Install SDK 0.17.4: https://github.com/zephyrproject-rtos/sdk-ng/releases (0.17.4)"
fi
echo "✅ Workspace ready. Build with: ./build.sh"
