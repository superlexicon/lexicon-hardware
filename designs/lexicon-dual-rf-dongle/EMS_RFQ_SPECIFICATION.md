# EMS Turnkey RFQ & Manufacturing Specification: Lexicon AM36-Pico Dual-LR2021 USB Radio Dongle (`LEX-AM36-PCBA-R1.0`)

## 1. Executive Summary for EMS Quotation
- **Product Name**: Lexicon AM36-Pico Dual-LR2021 USB Radio Dongle (Device A / Wave 1)
- **Part Number / Revision**: `LEX-AM36-PCBA-R1.0` (Rev 1.0 Production Release)
- **Target Quotation Volumes**:
  - **EVT / DVT Pilot Run**: `250` units
  - **PVT / Wave 1 Kickstarter Batch (Month 6 Delivery)**: `11,000` units
  - **Year 1–2 Mass Production**: `25,000 – 100,000` units/year (`448,000` 5-Yr cumulative units)
- **Included Open-Source Manufacturing Files in Package**:
  1. `kicad/lexicon_am36_pico_dual_lr2021.kicad_pro` — KiCad 8.0 Project & Netclass / DRC Rules
  2. `kicad/lexicon_am36_pico_dual_lr2021.kicad_sch` — KiCad 8.0 Native Circuitry Schematic
  3. `kicad/lexicon_am36_pico_dual_lr2021.kicad_pcb` — KiCad 8.0 Native 6-Layer Controlled-Impedance PCB Layout
  4. `gerbers/lexicon_am36_pico_gerbers_v1.0.zip` — Complete RS-274X (Gerber X2) 6-Layer Copper, Solder Mask, Paste, Silkscreen & Excellon Drill (`PTH.drl`) files
  5. `bom/lexicon_am36_pico_bom_ems.csv` — 28-Line (69-Component) Production BOM with MPNs, LCSC & DigiKey Part Numbers, and 1k/10k/100k Pricing
  6. `bom/lexicon_am36_pico_cpl_pick_and_place.csv` — XYRS Centroid Pick-and-Place File (mm, Top-side single-pass reflow)
  7. `3d/lexicon_am36_pico_pcba.step` / `.stl` / `.obj` / `.scad` — 3D Mechanical Assembly Models (STEP AP214, STL, OBJ, OpenSCAD)
  8. `docs/lexicon_am36_pico_schematic.svg` (`.png`), `docs/lexicon_am36_pico_pcb_layout.svg` (`.png`), `docs/lexicon_am36_pcba_3d.jpg`

---

## 2. Bare PCB Fabrication Specification (`LEX-AM36-PCB-R1.0`)
| Parameter | Specification |
| :--- | :--- |
| **Board Dimensions** | `60.40 mm × 28.40 mm` (`±0.10 mm` CNC routed outline) |
| **Layer Count** | **6 Copper Layers** (`F.Cu`, `In1.Cu`, `In2.Cu`, `In3.Cu`, `In4.Cu`, `B.Cu`) |
| **Total Board Thickness** | `1.60 mm ± 0.10 mm` |
| **Base Laminate Material** | High-Tg FR4 (`Tg ≥ 170°C`, Shengyi S1000-2M or Isola 370HR, RoHS Lead-Free) |
| **Controlled Impedance Stackup** | Standard **JLC06161H-3313** (`3313` prepreg `0.0994 mm`, `0.55 mm` core, `2116` prepreg `0.1408 mm`) |
| **Copper Weights** | Outer (`L1/L6`): `1 oz (35 µm)` finished; Inner (`L2/L3/L4/L5`): `0.5 oz (17.5 µm)` |
| **RF Trace Impedance (`RF_50OHM_CPWG`)** | **`50.0 Ω ± 5%` Coplanar Waveguide with Ground (`W = 0.23 mm`, `Gap = 0.15 mm` on `L1` referenced to `L2 GND1`)** |
| **USB Differential Impedance (`USB_DIFF_90OHM`)** | **`90.0 Ω ± 10%` Edge-Coupled Microstrip (`W = 0.22 mm`, `Spacing = 0.15 mm` on `L1` referenced to `L2 GND1`)** |
| **Surface Finish** | **ENIG (Electroless Nickel Immersion Gold)**: `2 µ"` (`0.05 µm`) Au over `120–200 µ"` (`3–5 µm`) Ni |
| **Solder Mask & Silkscreen** | Matte Black Solder Mask (`Taiyo PSR-4000`) both sides; Crisp White Epoxy Silkscreen |
| **Via Treatment** | `0.20 mm` drill / `0.40 mm` pad RF fence & QFN thermal vias plugged/tented per IPC-4761 Type VI |
| **Panelization** | `2 × 5` (10-up) array with `5.0 mm` breakaway mouse-bite/V-score rails and 3 global optical fiducials |

---

## 3. SMT PCBA Assembly & Quality Requirements
1. **Single-Sided Reflow Efficiency**: All 69 active and passive SMD components are placed on the **Top Layer (`F.Cu`)** to allow a single-pass SAC305 lead-free reflow profile, minimizing SMT assembly cost (`~$1.45/board` at 10k volume).
2. **X-Ray & AOI Inspection**:
   - `100%` 3D Solder Paste Inspection (SPI) and 3D Automated Optical Inspection (AOI).
   - X-Ray inspection on `U2`, `U3` (`Semtech LR2021IMLTRT` VQFN-32 `5×5 mm`), and `U4` (`CP2102N` QFN-28) center ground thermal pads (`< 20%` voiding per IPC-A-610 Class 2/3).
3. **Two-Piece RF Shield Cans (`SH1`, `SH2`)**:
   - Reflow the stamped nickel-silver frames (`Masach S02-16110300`, `16 × 11 × 2.5 mm`) during primary SMT; snap on the removable shield lids after optical/RF inspection.
