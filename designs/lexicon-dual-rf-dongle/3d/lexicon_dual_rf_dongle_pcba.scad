// ============================================================================
// LEXICON DUAL-RF DONGLE DUAL-LR2021 6-LAYER PCBA — PARAMETRIC OPENSCAD 3D MODEL
// Dimensions: 60.40 mm x 28.40 mm x 1.60 mm (Plus Edge-Launch SMA Bulkhead)
// ============================================================================
$fn = 48;
module lexicon_am36_pcba() {
  // PCB_Substrate_6L_FR4_MatteBlack
  color([0.10, 0.12, 0.15]) translate([0.00, 0.00, 0.00]) cube([60.40, 28.40, 1.60]);
  // ENIG_Perimeter_Ring_Top
  color([0.88, 0.72, 0.24]) translate([0.80, 27.20, 1.60]) cube([58.80, 0.80, 0.04]);
  // ENIG_Perimeter_Ring_Bot
  color([0.88, 0.72, 0.24]) translate([0.80, 0.40, 1.60]) cube([58.80, 0.80, 0.04]);
  // ENIG_Perimeter_Ring_Left
  color([0.88, 0.72, 0.24]) translate([0.40, 0.80, 1.60]) cube([0.80, 26.80, 0.04]);
  // ENIG_Perimeter_Ring_Right
  color([0.88, 0.72, 0.24]) translate([59.20, 0.80, 1.60]) cube([0.80, 26.80, 0.04]);
  // J1_USBC_Native_JTAG
  color([0.82, 0.85, 0.88]) translate([-0.47, 16.73, 1.60]) cube([7.35, 8.94, 3.16]);
  // J2_USBC_UART_Bridge
  color([0.82, 0.85, 0.88]) translate([-0.47, 2.73, 1.60]) cube([7.35, 8.94, 3.16]);
  // D1_USBLC6_ESD1
  color([0.16, 0.17, 0.20]) translate([7.35, 20.40, 1.60]) cube([2.90, 1.60, 1.10]);
  // D2_USBLC6_ESD2
  color([0.16, 0.17, 0.20]) translate([7.35, 6.40, 1.60]) cube([2.90, 1.60, 1.10]);
  // U4_CP2102N_QFN28
  color([0.16, 0.17, 0.20]) translate([11.30, 4.70, 1.60]) cube([5.00, 5.00, 0.85]);
  // U5_LDO_3V3_DIG
  color([0.16, 0.17, 0.20]) translate([12.35, 15.70, 1.60]) cube([2.90, 1.60, 1.10]);
  // U6_LDO_3V3_RF0
  color([0.16, 0.17, 0.20]) translate([12.35, 19.70, 1.60]) cube([2.90, 1.60, 1.10]);
  // U7_LDO_3V3_RF1
  color([0.16, 0.17, 0.20]) translate([12.35, 23.40, 1.60]) cube([2.90, 1.60, 1.10]);
  // SW1_RST_Button
  color([0.82, 0.85, 0.88]) translate([9.50, 24.65, 1.60]) cube([6.00, 3.50, 2.50]);
  // SW2_BOOT_Button
  color([0.82, 0.85, 0.88]) translate([17.50, 24.65, 1.60]) cube([6.00, 3.50, 2.50]);
  // SW3_KEY_Button
  color([0.82, 0.85, 0.88]) translate([25.50, 24.65, 1.60]) cube([6.00, 3.50, 2.50]);
  // D5_WS2812B_RGB_LED
  color([0.00, 0.95, 0.75]) translate([33.50, 25.40, 1.60]) cube([2.00, 2.00, 0.90]);
  // U1_ESP32_S3_WROOM_1_Body
  color([0.06, 0.08, 0.10]) translate([17.50, 0.75, 1.60]) cube([18.00, 25.50, 0.80]);
  // U1_ESP32_S3_Shield_Can
  color([0.82, 0.85, 0.88]) translate([18.00, 1.50, 2.40]) cube([17.00, 19.00, 2.30]);
  // SH1_RF0_Shield_Frame_WallN
  color([0.82, 0.85, 0.88]) translate([36.50, 25.90, 1.60]) cube([16.00, 0.40, 2.50]);
  // SH1_RF0_Shield_Frame_WallS
  color([0.82, 0.85, 0.88]) translate([36.50, 15.30, 1.60]) cube([16.00, 0.40, 2.50]);
  // SH1_RF0_Shield_Frame_WallW
  color([0.82, 0.85, 0.88]) translate([36.50, 15.30, 1.60]) cube([0.40, 11.00, 2.50]);
  // SH1_RF0_Shield_Frame_WallE
  color([0.82, 0.85, 0.88]) translate([52.10, 15.30, 1.60]) cube([0.40, 11.00, 2.50]);
  // U2_Semtech_LR2021_RF0
  color([0.16, 0.17, 0.20]) translate([38.70, 18.30, 1.60]) cube([5.00, 5.00, 0.85]);
  // Y1_TCXO_32MHz_RF0
  color([0.88, 0.72, 0.24]) translate([40.20, 23.80, 1.60]) cube([2.00, 1.60, 0.75]);
  // FL1_SAW_915MHz_RF0
  color([0.16, 0.17, 0.20]) translate([49.70, 20.25, 1.60]) cube([1.40, 1.10, 0.65]);
  // SH2_RF1_Shield_Can_Closed
  color([0.82, 0.85, 0.88]) translate([36.50, 2.10, 1.60]) cube([16.00, 11.00, 2.50]);
  // U3_Semtech_LR2021_RF1
  color([0.16, 0.17, 0.20]) translate([38.70, 5.10, 1.60]) cube([5.00, 5.00, 0.85]);
  // Y2_TCXO_32MHz_RF1
  color([0.88, 0.72, 0.24]) translate([40.20, 3.00, 1.60]) cube([2.00, 1.60, 0.75]);
  // FL2_SAW_Diplexer_RF1
  color([0.16, 0.17, 0.20]) translate([49.60, 7.20, 1.60]) cube([1.60, 0.80, 0.65]);
  // U8_SKY13373_RF_Switch
  color([0.16, 0.17, 0.20]) translate([53.20, 13.20, 1.60]) cube([2.00, 2.00, 0.75]);
  // J4_UFL_Aux_Antenna
  color([0.88, 0.72, 0.24]) translate([54.90, 3.50, 1.60]) cube([2.60, 2.60, 1.25]);
  // J3_SMA_Edge_Base
  color([0.88, 0.72, 0.24]) translate([57.60, 11.02, -1.20]) cube([3.20, 6.35, 6.35]);
  // J3_SMA_Threaded_Barrel
  color([0.88, 0.72, 0.24]) translate([60.75, 11.60, -0.50]) cube([7.50, 5.20, 5.20]);
}
lexicon_am36_pcba();
