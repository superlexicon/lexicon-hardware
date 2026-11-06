# Lexicon Hardware, Radio Firmware & Device Drivers

Unified open-hardware repository containing the radio transceiver firmware, Linux/RT-Smart device drivers, 3D enclosures, and reference hardware designs for the **Lexicon** radio-native blockchain ecosystem.

---

## Repository Structure

```text
lexicon-hardware/
├── firmware/
│   ├── nrf54l15-lr2021/          # Device 1: Seeed Studio XIAO nRF54L15 + LR2021 Eval Kit (Zephyr RTOS)
│   └── am36-lr2021/              # Device 2: Lierda AM36-Pico ESP32-S3 + LR2021 USB Dongle (ESP-IDF + 3D Enclosure)
├── drivers/
│   └── k230-lr2021-radio/        # Device 3: LilyGO T-Display K230 RISC-V SPI Radio Service & Host Daemon
└── designs/
    └── lexicon-dual-rf-dongle/   # Turnkey EMS Reference Design: Dual-LR2021 USB Dongle (Schematics, Gerber, BOM, 3D CAD)
```

---

## Supported Hardware Targets

| Target Device | MCU / SoC | Radio Transceiver | Stack | Directory |
| :--- | :--- | :--- | :--- | :--- |
| **Seeed Studio LR2021 LoRa Plus Eval Kit** | Nordic `nRF54L15` (Cortex-M33) | Semtech `LR2021` (915 MHz / 2.4 GHz FLRC) | Zephyr RTOS | [`firmware/nrf54l15-lr2021/`](./firmware/nrf54l15-lr2021/) |
| **Lierda AM36-Pico (`L-LRMAM36-FANN4-PK02`)** | Espressif `ESP32-S3` (Dual Xtensa LX7) | Semtech `LR2021` (Sub-GHz / 2.4 GHz FLRC) | ESP-IDF v5.x | [`firmware/am36-lr2021/`](./firmware/am36-lr2021/) |
| **LilyGO T-Display K230 Kit** | Canaan `Kendryte K230` (Dual RISC-V C908) | Semtech `LR2021` (SPI Daughterboard) | RT-Smart / Linux C | [`drivers/k230-lr2021-radio/`](./drivers/k230-lr2021-radio/) |
| **Lexicon Dual-RF USB Dongle (Reference)** | `ESP32-S3` + Dual `LR2021` | 2x Parallel `LR2021` (`RF0` + `RF1`) | Turnkey EMS Package | [`designs/lexicon-dual-rf-dongle/`](./designs/lexicon-dual-rf-dongle/) |

---

## Common FLRC Air & Serial Protocol

All three hardware targets implement the Lexicon **Phase-C FLRC Burst Protocol**:
- **Modulation:** Semtech `LR2021` FLRC up to **2.6 Mbps** raw air data rate (`511 B` physical frames / `495 B` payload chunks, `220 µs` interframe pacing).
- **Reassembly Capacity:** Up to **64 KB (`65,024 B` / 132 packets)** per burst with per-burst random IDs and 4 concurrent reassembly slots.
- **Host Serial / IPC Interface:** Dedicated control lane (`'G'`, `'T'`, `'R'`, `'C'`, `'W'`, `'S'`, `'Q'`) ensuring TX terminal notifications are never dropped under payload ring pressure.
