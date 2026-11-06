# Lierda AM36-Pico (`ESP32-S3 + Semtech LR2021`) SuperLexicon FLRC Bridge Firmware & 3D Enclosure

This repository contains the dedicated ESP-IDF firmware port and 3D-printable enclosure for the **Lierda AM36-Pico Development Board (`L-LRMAM36-FANN4-PK02`)**, featuring an **Espressif ESP32-S3** (Dual-Core 240 MHz + 8 MB Flash + 8 MB Octal PSRAM) integrated with a **Semtech LR2021** multi-band transceiver inside a shielded RF module (`L-LRMAM36-FANN4`).

It is wire-protocol compatible (`SLB1` binary framing + JSON telemetry + 2.6 Mbps FLRC burst FEC) with the existing SuperLexicon validator radio repos:
- [`lr2021-flrc-firmware`](file:///home/mano/Projects/lr2021-flrc-firmware) (Seeed Studio XIAO nRF54L15 + LR2021)
- [`k230-lr2021-radio`](file:///home/mano/Projects/k230-lr2021-radio) (CanMV K230 + LR2021)

---

## 1. Key Upgrades Over the XIAO nRF54L15 Dev Board

| Feature | Previous Board (`XIAO nRF54L15 + LR2021`) | Lierda AM36-Pico (`L-LRMAM36-FANN4-PK02`) |
| :--- | :--- | :--- |
| **MCU & Clock** | Single-core Cortex-M33 @ 128 MHz | **Dual-core Xtensa LX7 @ 240 MHz** |
| **Memory** | 256 KB SRAM | **512 KB SRAM + 8 MB Octal PSRAM + 8 MB Flash** |
| **Host Serial Interface** | `UART0` via SAMD11 bridge (**1 Mbps bottleneck**) | **Native USB-Serial-JTAG (`CON2`) @ ~5–8 Mbps** + Dual UART (`CON1` CP2105) |
| **Core Separation** | Radio superloop & UART share 1 core | **Core 1 dedicated to `smtc_rac_run_engine()`**, **Core 0 dedicated to USB RX/TX** |
| **Concurrent RX Reassembly** | 1 slot (`burst_id_conflicts` when 2 nodes overlap) | **4 concurrent 32 KB reassembly slots (`BURST_REASM_SLOTS = 4`)** |
| **TX Burst Queue Depth** | 2 slots | **8 slots (`HOST_TX_QUEUE_DEPTH = 8`)** |
| **RF Shielding & Matching** | Unshielded external jumper wires | **Factory-shielded module with internal Sub-GHz + 2.4 GHz Diplexer** |
| **Band Switching** | Fixed single band | **Runtime switchable between `2.442 GHz` (+12 dBm) and `915 MHz` (+22 dBm)** via `KEY` button or USB command |

---

## 2. Internal Pinout (`ESP32-S3` ↔ `LR2021` inside `L-LRMAM36-FANN4`)

Verified from [`docs/L-LRMAM36-FANN4-PK02_SCH_V01.pdf`](file:///home/mano/Projects/am36-lr2021-firmware/docs/L-LRMAM36-FANN4-PK02_SCH_V01.pdf) and [`docs/Lierda_L-LRMAM36-FANN4_Hardware_Design_Manual_EN_Rev1.0.pdf`](file:///home/mano/Projects/am36-lr2021-firmware/docs/Lierda_L-LRMAM36-FANN4_Hardware_Design_Manual_EN_Rev1.0.pdf):

| Signal | ESP32-S3 GPIO | LR2021 Pin | Description |
| :--- | :--- | :--- | :--- |
| `LR2021_BUSY` | `GPIO17` | `BUSY` | Radio busy status input |
| `LR2021_nRST` | `GPIO38` | `NRESET` | Hardware deep reset (active low) |
| `LR2021_DIO7` | `GPIO21` | `DIO7` | Radio IRQ line to host |
| `SPI3_CS` | `GPIO39` | `NSS` | SPI3 Chip Select |
| `SPI3_CLK` | `GPIO40` | `SCK` | SPI3 Clock |
| `SPI3_MISO` | `GPIO41` | `MISO` | SPI3 MISO |
| `SPI3_MOSI` | `GPIO42` | `MOSI` | SPI3 MOSI |
| `RF Switch DIOs` | *(Internal)* | `DIO8..DIO11` | `DIO8`=`TX_LF`, `DIO9`=`RX_LF`, `DIO10`=`RX_HF`, `DIO11`=`TX_HF` -> Diplexer -> `ANT1` SMA |
| `K1 (RST)` | `CHIP_PU` | — | Left tactile button: Hardware ESP32-S3 Reset |
| `K2 (BOOT)` | `GPIO0` | — | Center tactile button: Bootloader mode |
| `K3 (KEY)` | `GPIO4` | — | Right tactile button: Toggles `2.442 GHz` ↔ `915 MHz` band & emits `STATUS` |
| `D7 (RGB LED)` | `GPIO18` | — | Onboard WS2812 RGB status LED (Blue = 2.4 GHz, Purple = 915 MHz, Green = RX, Cyan = TX, Red = Fault) |
| `CON2 (USB)` | `GPIO19`/`20` | — | Bottom-left USB-C: Native USB-Serial-JTAG (`SLB1` high-speed data bridge) |
| `CON1 (UART)` | `GPIO43`/`44` | — | Bottom-right USB-C: CP2105 Dual UART (`ESP_LOG` console output) |

---

## 3. Recommended Dual-Band (`915 MHz + 2.4 GHz`) Antennas

Because the `L-LRMAM36-FANN4` module merges the Sub-GHz (`LF`) and 2.4 GHz (`HF`) RF switches through an internal **Diplexer** onto the single `ANT1` **SMA Female** connector, use a **true dual-band `915 MHz + 2.4 GHz` SMA Male (center pin) antenna** (do **not** use `RP-SMA`):

1. **Best RF Performance (Lab-Grade Dipole)**: **Taoglas `TG.30.8113`** (Hinged right-angle SMA Male dipole covering `868/915 MHz` + `2.4 GHz` with VSWR < 2.0 on both bands; available on DigiKey/Mouser).
2. **Best High-Gain Dual-Band LoRa Antenna**: **RadioMaster Bandit / Nomad Dual-Band `900 MHz + 2.4 GHz` T-Antenna (SMA Male)** — engineered specifically for Semtech dual-band Sub-GHz + 2.4 GHz transceivers.
3. **Budget AliExpress Option**: Search `"4G LTE WiFi Dual Band SMA Male Antenna 824-960MHz 2400-2500MHz"` (verify the connector is **SMA Male with center pin**).

---

## 4. 3D-Printable Enclosure with Tactile Buttons (`enclosure/`)

The `L-LRMAM36-FANN4-PK02` board measures **`80.00 mm × 25.40 mm`** (wider and longer than a standard Raspberry Pi Pico) and has **dual side-by-side USB-C ports** at the bottom, an **edge-mount SMA jack** at the top-left, and **three SMD buttons (`RST`, `BOOT`, `KEY`)** across the lower middle.

Pre-generated watertight binary `.stl` files and parametric OpenSCAD source are provided in [`enclosure/`](file:///home/mano/Projects/am36-lr2021-firmware/enclosure):
- [`enclosure/am36_pico_both_print_plate.stl`](file:///home/mano/Projects/am36-lr2021-firmware/enclosure/am36_pico_both_print_plate.stl) — Complete single-plate print containing both the bottom cradle shell and top button shell laid flat (`0.20 mm` layer height, **zero supports required**).
- [`enclosure/am36_pico_bottom.stl`](file:///home/mano/Projects/am36-lr2021-firmware/enclosure/am36_pico_bottom.stl) — Bottom cradle shell with dual USB-C cutouts, SMA U-slot, and underside pin-header slots.
- [`enclosure/am36_pico_top_buttons.stl`](file:///home/mano/Projects/am36-lr2021-firmware/enclosure/am36_pico_top_buttons.stl) — Snap-fit top lid with **3 integrated cantilevered tactile plungers (`RST`, `BOOT`, `KEY`)**, `D7` RGB LED viewing window, cooling vents over the `L-LRMAM36-FANN4` shield, and RF-transparent clearance over `ANT2`.
- [`enclosure/am36_pico_case.scad`](file:///home/mano/Projects/am36-lr2021-firmware/enclosure/am36_pico_case.scad) & [`enclosure/generate_stl.py`](file:///home/mano/Projects/am36-lr2021-firmware/enclosure/generate_stl.py) — Parametric source and STL generator.

> **Printing Tip**: Print in **PETG, ABS, or ASA** (3 perimeters, 0.20 mm layer height, no supports) so the 3 cantilevered button fingers flex smoothly and snap back cleanly. You can also upload `am36_pico_both_print_plate.stl` directly to **JLCPCB 3D Printing** (`MJF Nylon PA12` or `SLA Tough Resin`, ~SGD \$2–\$3 per case).

---

## 5. Building & Flashing (ESP-IDF v5.4+)

```bash
# 1. Activate ESP-IDF v5.4+ environment
. $HOME/esp/esp-idf/export.sh

# 2. Configure target (loads sdkconfig.defaults automatically)
cd /home/mano/Projects/am36-lr2021-firmware
idf.py set-target esp32s3

# 3. Build firmware (IDF Component Manager automatically fetches lierda-iot/esp_lora_driver)
idf.py build

# 4. Flash & monitor via CON2 (Native USB-Serial-JTAG)
idf.py -p /dev/ttyACM0 flash monitor
```

### Applying Semtech USP Burst Reliability Patches (`patches/`)
When building in production validator deployments, apply patches `0004`–`0013` in [`patches/`](file:///home/mano/Projects/am36-lr2021-firmware/patches) (documented in [`PATCHES.md`](file:///home/mano/Projects/am36-lr2021-firmware/PATCHES.md)) to `managed_components/lierda-iot__esp_lora_driver/` to enable single-packet `TX_DONE` IRQ mapping and automatic recovery from latched `CHIP_BUSY` states.
