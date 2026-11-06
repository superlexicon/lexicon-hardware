/**
 * Lierda AM36-Pico (L-LRMAM36-FANN4-PK02) Snap-Fit Enclosure with Tactile Button Actuators
 * Exact dimensions from L-LRMAM36-FANN4-PK02_layout_V01.pdf & Hardware Design Manual Rev 1.0:
 *   - PCB Size: 80.00 mm (Y) x 25.40 mm (X) x 1.60 mm (Z)
 *   - Top-Left (Y=80.0, X=6.0): ANT1 Edge-Mount SMA Jack (Sub-GHz + 2.4 GHz LR2021 RF)
 *   - Top-Right (Y=80.0, X=19.4): ANT2 2.4 GHz Wi-Fi/BLE Onboard Antenna (enclosed under RF-transparent ABS/PETG roof)
 *   - Bottom-Left (Y=0.0, X=6.8): CON2 USB Type-C (ESP32-S3 Native USB-Serial-JTAG)
 *   - Bottom-Right (Y=0.0, X=18.6): CON1 USB Type-C (CP2105 Dual UART)
 *   - Tactile Buttons (Y=19.5 from bottom USB edge):
 *       * K1 (RST):  X = 6.2 mm
 *       * K2 (BOOT): X = 12.7 mm
 *       * K3 (KEY):  X = 19.2 mm
 *   - RGB Status LED (D7 WS2812): X = 12.7 mm, Y = 25.0 mm
 *
 * Render options:
 *   part = "both"   -> Side-by-side print plate (bottom tray + top lid with buttons)
 *   part = "bottom" -> Bottom cradle shell only
 *   part = "top"    -> Top button shell only
 */

$fn = 48;
part = "both";

pcb_w = 25.40;
pcb_l = 80.00;
pcb_t = 1.60;
tol   = 0.35;     // XY clearance around PCB
wall  = 2.00;     // Outer wall thickness

inner_w = pcb_w + 2 * tol; // 26.10 mm
inner_l = pcb_l + 2 * tol; // 80.70 mm
outer_w = inner_w + 2 * wall; // 30.10 mm
outer_l = inner_l + 2 * wall; // 84.70 mm

bot_floor = 1.80;
pcb_ledge_h = 3.20; // Clearance under PCB for solder joints / components
bot_h = bot_floor + pcb_ledge_h + pcb_t + 1.0; // 7.60 mm total bottom tray height

top_roof = 1.80;
top_comp_h = 5.40; // Clearance above PCB top surface (L-LRMAM36-FANN4 shield is 3.15mm tall)
top_h = top_roof + top_comp_h; // 7.20 mm total top lid height

// Button coordinates relative to PCB origin (bottom-left corner)
btn_y = 19.50;
btn_x_rst  = 6.20;
btn_x_boot = 12.70;
btn_x_key  = 19.20;
btn_top_z_above_pcb = 2.50; // Standard SMD tactile switch height

// LED D7 coordinate relative to PCB origin
led_x = 12.70;
led_y = 25.00;

module bottom_shell() {
    difference() {
        // Outer shell
        cube([outer_w, outer_l, bot_h]);

        // Main interior cavity above PCB support ledge
        translate([wall, wall, bot_floor + pcb_ledge_h])
            cube([inner_w, inner_l, bot_h]);

        // Under-PCB cavity (leaving 1.4mm perimeter ledge and 4 corner support rails)
        translate([wall + 1.4, wall + 2.0, bot_floor])
            cube([inner_w - 2.8, inner_l - 4.0, pcb_ledge_h + 0.1]);

        // Optional underside pin header slots for J3 (left) and J4 (right) 2.54mm rows
        translate([wall + 0.6, wall + 9.0, -0.1])
            cube([2.6, 52.0, bot_floor + 0.2]);
        translate([outer_w - wall - 3.2, wall + 9.0, -0.1])
            cube([2.6, 52.0, bot_floor + 0.2]);

        // Bottom-Left USB-C cutout (CON2: Native USB)
        translate([wall + tol + 6.8 - 4.8, -0.5, bot_floor + pcb_ledge_h + pcb_t - 0.4])
            cube([9.6, wall + 1.5, 4.5]);

        // Bottom-Right USB-C cutout (CON1: UART)
        translate([wall + tol + 18.6 - 4.8, -0.5, bot_floor + pcb_ledge_h + pcb_t - 0.4])
            cube([9.6, wall + 1.5, 4.5]);

        // Top-Left SMA bulk-head U-slot (ANT1)
        translate([wall + tol + 6.0 - 3.5, outer_l - wall - 0.5, bot_floor + pcb_ledge_h - 0.5])
            cube([7.0, wall + 1.0, 6.0]);
    }
}

module cantilever_button(cx, cy) {
    // Cut U-shaped flex slot around a 4.2mm x 7.0mm cantilever finger hinged at +Y
    btn_w = 4.2;
    btn_l = 7.0;
    slot  = 0.7;
    translate([cx - btn_w/2 - slot, cy - btn_l/2 - slot, -0.1]) {
        // Left slot
        cube([slot, btn_l + slot, top_roof + 0.2]);
        // Right slot
        translate([btn_w + slot, 0, 0])
            cube([slot, btn_l + slot, top_roof + 0.2]);
        // Front slot (-Y end)
        cube([btn_w + 2*slot, slot, top_roof + 0.2]);
    }
}

module button_plunger(cx, cy) {
    // Raised external cap + internal actuator post reaching down to SMD tactile switch
    plunger_h = top_comp_h - btn_top_z_above_pcb - 0.25;
    translate([cx - 1.5, cy - 1.5, -plunger_h])
        cube([3.0, 3.0, plunger_h + top_roof + 0.8]);
}

module top_shell() {
    bx1 = wall + tol + btn_x_rst;
    bx2 = wall + tol + btn_x_boot;
    bx3 = wall + tol + btn_x_key;
    by  = wall + tol + btn_y;

    union() {
        difference() {
            union() {
                // Outer upper shell
                cube([outer_w, outer_l, top_h]);
                // Inner snap-fit lip that nests 1.2mm into bottom_shell
                translate([wall + 0.15, wall + 10.0, -1.2])
                    cube([inner_w - 0.3, inner_l - 20.0, 1.2]);
            }

            // Main interior cavity
            translate([wall, wall, -0.1])
                cube([inner_w, inner_l, top_comp_h + 0.1]);

            // Inner relief inside snap-fit lip
            translate([wall + 1.35, wall + 10.0, -1.3])
                cube([inner_w - 2.7, inner_l - 20.0, 1.4]);

            // Bottom-Left USB-C upper arch (CON2)
            translate([wall + tol + 6.8 - 4.8, -0.5, -0.1])
                cube([9.6, wall + 1.5, 3.4]);

            // Bottom-Right USB-C upper arch (CON1)
            translate([wall + tol + 18.6 - 4.8, -0.5, -0.1])
                cube([9.6, wall + 1.5, 3.4]);

            // Top-Left SMA connector upper arch (ANT1)
            translate([wall + tol + 6.0 - 3.5, outer_l - wall - 0.5, -0.1])
                cube([7.0, wall + 1.0, 4.2]);

            // D7 RGB Status LED light-pipe window
            translate([wall + tol + led_x - 1.2, wall + tol + led_y - 1.2, top_comp_h - 0.2])
                cube([2.4, 2.4, top_roof + 0.5]);

            // Cooling vents over L-LRMAM36-FANN4 RF shield (Y = 34..64 mm)
            for (vy = [36 : 5 : 61]) {
                translate([wall + 4.0, wall + tol + vy, top_comp_h - 0.2])
                    cube([inner_w - 8.0, 2.0, top_roof + 0.5]);
            }

            // 3 Cantilever U-slots for K1 (RST), K2 (BOOT), K3 (KEY)
            translate([0, 0, top_comp_h]) {
                cantilever_button(bx1, by);
                cantilever_button(bx2, by);
                cantilever_button(bx3, by);
            }
        }

        // Add internal tactile plungers + 0.8mm raised finger caps on K1, K2, K3
        translate([0, 0, top_comp_h]) {
            button_plunger(bx1, by);
            button_plunger(bx2, by);
            button_plunger(bx3, by);
        }
    }
}

if (part == "bottom") {
    bottom_shell();
} else if (part == "top") {
    // Flip upside down for support-free 3D printing
    translate([outer_w, 0, top_h + 0.8])
        rotate([0, 180, 0])
            top_shell();
} else {
    bottom_shell();
    translate([outer_w + 8.0 + outer_w, 0, top_h + 0.8])
        rotate([0, 180, 0])
            top_shell();
}
