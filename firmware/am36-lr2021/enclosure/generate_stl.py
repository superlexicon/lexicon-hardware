#!/usr/bin/env python3
"""
Generate ready-to-print binary STL files for the Lierda AM36-Pico (L-LRMAM36-FANN4-PK02)
enclosure with integrated tactile button actuators (K1 RST, K2 BOOT, K3 KEY),
D7 RGB LED window, dual USB-C cutouts (CON2 USB + CON1 UART), and ANT1 SMA port cutout.

Outputs:
  - am36_pico_bottom.stl            (Bottom cradle shell)
  - am36_pico_top_buttons.stl       (Top shell with 3 cantilever button plungers, print orientation)
  - am36_pico_both_print_plate.stl  (Both shells laid flat side-by-side for single-plate printing)
"""
import os
import struct
import math


class Mesh:
    def __init__(self):
        self.tris = []  # list of ((x0,y0,z0), (x1,y1,z1), (x2,y2,z2))

    def add_quad(self, p0, p1, p2, p3):
        """Add counter-clockwise quad (p0, p1, p2, p3) as two triangles."""
        self.tris.append((p0, p1, p2))
        self.tris.append((p0, p2, p3))

    def add_box(self, x0, y0, z0, x1, y1, z1):
        """Add watertight axis-aligned box [x0, x1] x [y0, y1] x [z0, z1]."""
        if x1 <= x0 or y1 <= y0 or z1 <= z0:
            return
        # Bottom (-Z)
        self.add_quad((x0, y0, z0), (x0, y1, z0), (x1, y1, z0), (x1, y0, z0))
        # Top (+Z)
        self.add_quad((x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1))
        # Front (-Y)
        self.add_quad((x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1))
        # Back (+Y)
        self.add_quad((x0, y1, z0), (x0, y1, z1), (x1, y1, z1), (x1, y1, z0))
        # Left (-X)
        self.add_quad((x0, y0, z0), (x0, y0, z1), (x0, y1, z1), (x0, y1, z0))
        # Right (+X)
        self.add_quad((x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1))

    def add_voxel_grid(self, xs, ys, zs, filled_fn):
        """
        Build an exact watertight surface mesh from a rectilinear 3D grid of boxes
        where `filled_fn(xc, yc, zc)` returns True if cell (i,j,k) is solid.
        Only boundary faces between solid and empty cells are emitted!
        """
        nx = len(xs) - 1
        ny = len(ys) - 1
        nz = len(zs) - 1
        grid = [[[False] * nz for _ in range(ny)] for _ in range(nx)]
        for i in range(nx):
            xc = 0.5 * (xs[i] + xs[i + 1])
            for j in range(ny):
                yc = 0.5 * (ys[j] + ys[j + 1])
                for k in range(nz):
                    zc = 0.5 * (zs[k] + zs[k + 1])
                    grid[i][j][k] = bool(filled_fn(xc, yc, zc))

        def is_solid(i, j, k):
            if 0 <= i < nx and 0 <= j < ny and 0 <= k < nz:
                return grid[i][j][k]
            return False

        for i in range(nx):
            x0, x1 = xs[i], xs[i + 1]
            for j in range(ny):
                y0, y1 = ys[j], ys[j + 1]
                for k in range(nz):
                    if not grid[i][j][k]:
                        continue
                    z0, z1 = zs[k], zs[k + 1]
                    if not is_solid(i - 1, j, k):
                        self.add_quad((x0, y0, z0), (x0, y0, z1), (x0, y1, z1), (x0, y1, z0))
                    if not is_solid(i + 1, j, k):
                        self.add_quad((x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1))
                    if not is_solid(i, j - 1, k):
                        self.add_quad((x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1))
                    if not is_solid(i, j + 1, k):
                        self.add_quad((x0, y1, z0), (x0, y1, z1), (x1, y1, z1), (x1, y1, z0))
                    if not is_solid(i, j, k - 1):
                        self.add_quad((x0, y0, z0), (x0, y1, z0), (x1, y1, z0), (x1, y0, z0))
                    if not is_solid(i, j, k + 1):
                        self.add_quad((x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1))

    def write_binary_stl(self, path, header_text="AM36-Pico-LR2021-Enclosure"):
        hdr = header_text.encode("ascii")[:80].ljust(80, b"\0")
        with open(path, "wb") as f:
            f.write(hdr)
            f.write(struct.pack("<I", len(self.tris)))
            for p0, p1, p2 in self.tris:
                ux, uy, uz = p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]
                vx, vy, vz = p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]
                nx = uy * vz - uz * vy
                ny = uz * vx - ux * vz
                nz = ux * vy - uy * vx
                norm = math.sqrt(nx * nx + ny * ny + nz * nz)
                if norm > 1e-12:
                    nx, ny, nz = nx / norm, ny / norm, nz / norm
                else:
                    nx, ny, nz = 0.0, 0.0, 1.0
                f.write(struct.pack(
                    "<12fH",
                    nx, ny, nz,
                    p0[0], p0[1], p0[2],
                    p1[0], p1[1], p1[2],
                    p2[0], p2[1], p2[2],
                    0
                ))


# Dimensions matching enclosure/am36_pico_case.scad
PCB_W = 25.40
PCB_L = 80.00
PCB_T = 1.60
TOL   = 0.35
WALL  = 2.00

INNER_W = PCB_W + 2 * TOL   # 26.10 mm
INNER_L = PCB_L + 2 * TOL   # 80.70 mm
OUTER_W = INNER_W + 2 * WALL  # 30.10 mm
OUTER_L = INNER_L + 2 * WALL  # 84.70 mm

BOT_FLOOR   = 1.80
PCB_LEDGE_H = 3.20
BOT_H       = BOT_FLOOR + PCB_LEDGE_H + PCB_T + 1.0  # 7.60 mm

TOP_ROOF   = 1.80
TOP_COMP_H = 5.40
TOP_H      = TOP_ROOF + TOP_COMP_H  # 7.20 mm


def build_bottom_mesh():
    # USB-C centers in outer coords
    usb_l_x = WALL + TOL + 6.80
    usb_r_x = WALL + TOL + 18.60
    sma_x   = WALL + TOL + 6.00

    xs = sorted(set([
        0.0,
        WALL,
        WALL + 0.6,
        WALL + 1.4,
        WALL + 3.2,
        usb_l_x - 4.8,
        usb_l_x + 4.8,
        usb_r_x - 4.8,
        usb_r_x + 4.8,
        sma_x - 3.5,
        sma_x + 3.5,
        OUTER_W - WALL - 3.2,
        OUTER_W - WALL - 1.4,
        OUTER_W - WALL - 0.6,
        OUTER_W - WALL,
        OUTER_W,
    ]))
    ys = sorted(set([
        0.0,
        WALL,
        WALL + 2.0,
        WALL + 9.0,
        WALL + 61.0,
        OUTER_L - WALL - 2.0,
        OUTER_L - WALL,
        OUTER_L,
    ]))
    zs = sorted(set([
        0.0,
        BOT_FLOOR,
        BOT_FLOOR + PCB_LEDGE_H - 0.5,
        BOT_FLOOR + PCB_LEDGE_H,
        BOT_FLOOR + PCB_LEDGE_H + PCB_T - 0.4,
        BOT_H,
    ]))

    def filled(x, y, z):
        if not (0.0 <= x <= OUTER_W and 0.0 <= y <= OUTER_L and 0.0 <= z <= BOT_H):
            return False
        # Main upper cavity above PCB ledge
        if WALL < x < OUTER_W - WALL and WALL < y < OUTER_L - WALL and z > BOT_FLOOR + PCB_LEDGE_H:
            return False
        # Under-PCB cavity
        if (WALL + 1.4 < x < OUTER_W - WALL - 1.4 and
                WALL + 2.0 < y < OUTER_L - WALL - 2.0 and
                BOT_FLOOR < z <= BOT_FLOOR + PCB_LEDGE_H):
            return False
        # Pin header slots (J3 & J4)
        if WALL + 9.0 < y < WALL + 61.0 and z <= BOT_FLOOR:
            if WALL + 0.6 < x < WALL + 3.2:
                return False
            if OUTER_W - WALL - 3.2 < x < OUTER_W - WALL - 0.6:
                return False
        # Dual USB-C cutouts on front wall (y < WALL)
        if y <= WALL and z > BOT_FLOOR + PCB_LEDGE_H + PCB_T - 0.4:
            if usb_l_x - 4.8 < x < usb_l_x + 4.8:
                return False
            if usb_r_x - 4.8 < x < usb_r_x + 4.8:
                return False
        # Top-Left SMA U-slot on back wall (y > OUTER_L - WALL)
        if y >= OUTER_L - WALL and z > BOT_FLOOR + PCB_LEDGE_H - 0.5:
            if sma_x - 3.5 < x < sma_x + 3.5:
                return False
        return True

    m = Mesh()
    m.add_voxel_grid(xs, ys, zs, filled)
    return m


def build_top_mesh(x_offset=0.0):
    """
    Build top lid in support-free upside-down print orientation:
      - Z = 0.0 .. 0.8: Raised tactile button caps on bed (or 0.8mm bed standoffs)
      - Actually, placing outer roof flat at Z = 0.8 .. 0.8 + TOP_H and button caps at Z = 0.0 .. 0.8
        or keeping outer roof flat on bed (Z = 0.0 .. TOP_ROOF) with flush cantilever buttons
        so the entire top surface prints flat on the build plate with zero supports!
    Let's make the outer roof sit flat on the print bed at Z = 0.0, with the 3 cantilever buttons
    flush at Z = 0.0 and having a 0.4mm tactile ring / plunger extending up inside the lid!
    """
    bx1 = WALL + TOL + 6.20
    bx2 = WALL + TOL + 12.70
    bx3 = WALL + TOL + 19.20
    by  = WALL + TOL + 19.50

    led_x = WALL + TOL + 12.70
    led_y = WALL + TOL + 25.00

    usb_l_x = WALL + TOL + 6.80
    usb_r_x = WALL + TOL + 18.60
    sma_x   = WALL + TOL + 6.00

    plunger_h = TOP_COMP_H - 2.50 - 0.25  # 2.65 mm internal post above roof inner face

    xs_set = {
        0.0,
        WALL,
        WALL + 0.15,
        WALL + 1.35,
        WALL + 4.0,
        OUTER_W - WALL - 4.0,
        OUTER_W - WALL - 1.35,
        OUTER_W - WALL - 0.15,
        OUTER_W - WALL,
        OUTER_W,
        usb_l_x - 4.8, usb_l_x + 4.8,
        usb_r_x - 4.8, usb_r_x + 4.8,
        sma_x - 3.5, sma_x + 3.5,
        led_x - 1.2, led_x + 1.2,
    }
    for bx in (bx1, bx2, bx3):
        xs_set.update([bx - 2.8, bx - 2.1, bx - 1.5, bx + 1.5, bx + 2.1, bx + 2.8])

    ys_set = {
        0.0,
        WALL,
        WALL + 10.0,
        OUTER_L - WALL - 10.0,
        OUTER_L - WALL,
        OUTER_L,
        by - 4.2, by - 3.5, by - 1.5, by + 1.5, by + 3.5,
        led_y - 1.2, led_y + 1.2,
    }
    for vy in range(36, 62, 5):
        ys_set.add(WALL + TOL + vy)
        ys_set.add(WALL + TOL + vy + 2.0)

    zs_set = {
        0.0,
        TOP_ROOF,
        TOP_ROOF + plunger_h,
        TOP_H - 4.2,
        TOP_H - 3.4,
        TOP_H,
        TOP_H + 1.2,
    }

    xs = sorted(xs_set)
    ys = sorted(ys_set)
    zs = sorted(zs_set)

    def filled(x, y, z):
        # Print orientation: Z=0 is outer roof surface, Z=TOP_H is lip edge, Z=TOP_H+1.2 is snap lip
        if z <= TOP_H:
            if not (0.0 <= x <= OUTER_W and 0.0 <= y <= OUTER_L):
                return False
            # Interior cavity above roof (Z > TOP_ROOF)
            if WALL < x < OUTER_W - WALL and WALL < y < OUTER_L - WALL and z > TOP_ROOF:
                # Check if inside one of the 3 button tactile plungers
                for bx in (bx1, bx2, bx3):
                    if bx - 1.5 < x < bx + 1.5 and by - 1.5 < y < by + 1.5 and z <= TOP_ROOF + plunger_h:
                        return True
                return False

            # Roof features (Z <= TOP_ROOF)
            if z <= TOP_ROOF:
                # LED window
                if led_x - 1.2 < x < led_x + 1.2 and led_y - 1.2 < y < led_y + 1.2:
                    return False
                # Cooling vents
                if WALL + 4.0 < x < OUTER_W - WALL - 4.0:
                    for vy in range(36, 62, 5):
                        if WALL + TOL + vy < y < WALL + TOL + vy + 2.0:
                            return False
                # 3 Cantilever button U-slots (hinged at +Y, U-slot on -X, +X, -Y)
                for bx in (bx1, bx2, bx3):
                    if by - 4.2 < y < by + 3.5:
                        if bx - 2.8 < x < bx - 2.1 or bx + 2.1 < x < bx + 2.8:
                            return False
                    if by - 4.2 < y < by - 3.5 and bx - 2.8 < x < bx + 2.8:
                        return False

            # Front wall USB-C arches (y <= WALL, near rim z > TOP_H - 3.4)
            if y <= WALL and z > TOP_H - 3.4:
                if usb_l_x - 4.8 < x < usb_l_x + 4.8:
                    return False
                if usb_r_x - 4.8 < x < usb_r_x + 4.8:
                    return False

            # Back wall SMA arch (y >= OUTER_L - WALL, near rim z > TOP_H - 4.2)
            if y >= OUTER_L - WALL and z > TOP_H - 4.2:
                if sma_x - 3.5 < x < sma_x + 3.5:
                    return False

            return True
        else:
            # Snap-fit lip (TOP_H < z <= TOP_H + 1.2)
            if WALL + 10.0 < y < OUTER_L - WALL - 10.0:
                if (WALL + 0.15 < x < WALL + 1.35) or (OUTER_W - WALL - 1.35 < x < OUTER_W - WALL - 0.15):
                    return True
            return False

    m = Mesh()
    m.add_voxel_grid(xs, ys, zs, filled)
    if x_offset != 0.0:
        shifted = []
        for p0, p1, p2 in m.tris:
            shifted.append((
                (p0[0] + x_offset, p0[1], p0[2]),
                (p1[0] + x_offset, p1[1], p1[2]),
                (p2[0] + x_offset, p2[1], p2[2]),
            ))
        m.tris = shifted
    return m


def main():
    out_dir = os.path.dirname(os.path.abspath(__file__))
    bot = build_bottom_mesh()
    bot_path = os.path.join(out_dir, "am36_pico_bottom.stl")
    bot.write_binary_stl(bot_path, "AM36-Pico Bottom Shell")

    top = build_top_mesh(x_offset=0.0)
    top_path = os.path.join(out_dir, "am36_pico_top_buttons.stl")
    top.write_binary_stl(top_path, "AM36-Pico Top Button Shell")

    both = Mesh()
    both.tris.extend(bot.tris)
    top_shifted = build_top_mesh(x_offset=OUTER_W + 8.0)
    both.tris.extend(top_shifted.tris)
    both_path = os.path.join(out_dir, "am36_pico_both_print_plate.stl")
    both.write_binary_stl(both_path, "AM36-Pico Complete Print Plate")

    print(f"Generated {bot_path} ({len(bot.tris)} triangles)")
    print(f"Generated {top_path} ({len(top.tris)} triangles)")
    print(f"Generated {both_path} ({len(both.tris)} triangles)")


if __name__ == "__main__":
    main()
