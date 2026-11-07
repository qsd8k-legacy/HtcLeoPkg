#!/usr/bin/env python3
"""decode_beacon.py - read the KEP beacon "barcode" rows from a phone photo
or an emulator screenshot.

What is on the panel (KernelErrataPatcherDxe, BeaconMark):
  The 480x800 panel is split into 64 KiB bands of 34 pixel rows, band N =
  framebuffer bytes N*64K..  A "row" is one such band painted full width as
  32 cells of 15 px: bit 31 is the LEFT cell, white = 1, dark grey = 0,
  1 px black gaps between cells, the band's first pixel row black.
    rows 19-22  bugcheck (always on): 19 = stop code, 20/21/22 = P1/P2/P3
    rows 8-22   HAL_TRACE builds (call brackets, see stage-hw/README.md)
    rows 20-22  BEACON_DIAG_ROWS builds: SCTLR / PRRR / NMRR
  Milestone stripes (BEACON_MILESTONES) are the coloured 64 px blocks in the
  left column, band N = milestone N, colours red green yellow blue magenta
  cyan white repeating.

Usage:
  decode_beacon.py photo.jpg                  # bottom band = row 22
  decode_beacon.py photo.jpg --rotate 90      # phone photographed sideways
  decode_beacon.py photo.jpg --last-row 20    # bottom band is row 20
  decode_beacon.py shot.png --exact           # emulator screendump (480x800)
  decode_beacon.py photo.jpg --crop 250,900,1400,2150   # grid region only
  decode_beacon.py photo.jpg --debug          # print band / edge detection

Photos: keep the whole grid in frame, avoid glare on the lower rows.  If a
row reads wrong, count the cells by hand (32 per row, left = bit 31) - the
tool prints the bit string so you can compare cell by cell.

Needs Pillow (python3 -m pip install pillow).
"""
import argparse
import sys

from PIL import Image

PANEL_W, PANEL_H, BAND_BYTES = 480, 800, 0x10000
BUGCHECKS = {
    0x0A: "IRQL_NOT_LESS_OR_EQUAL", 0x1E: "KMODE_EXCEPTION_NOT_HANDLED",
    0x3B: "SYSTEM_SERVICE_EXCEPTION", 0x50: "PAGE_FAULT_IN_NONPAGED_AREA",
    0x5C: "HAL_INITIALIZATION_FAILED", 0x5D: "UNSUPPORTED_PROCESSOR",
    0x67: "CONFIG_INITIALIZATION_FAILED", 0x7B: "INACCESSIBLE_BOOT_DEVICE",
    0x7E: "SYSTEM_THREAD_EXCEPTION_NOT_HANDLED", 0x100: "LOADER_BLOCK_MISMATCH",
    0xC0DE8250: "halextmsmvic checkpoint (P1 = checkpoint)",
    0xC0DE0001: "halextmsmtmr checkpoint (P1 = checkpoint)",
}


def band_rows(r):
    """first and last panel pixel row of band r (emulator geometry)"""
    first = r * BAND_BYTES // 4 // PANEL_W
    last = ((r + 1) * BAND_BYTES // 4 - 1) // PANEL_W
    return first, min(last, PANEL_H - 1)


def decode_exact(img, rows):
    """emulator screenshot: geometry is exact, cell c centre at x = 15c + 7"""
    px = img.convert("RGB").load()
    out = {}
    for r in rows:
        first, last = band_rows(r)
        y = (first + last) // 2
        if y >= img.size[1]:
            continue
        bits = "".join("1" if px[c * 15 + 7, y][0] > 200 and px[c * 15 + 7, y][1] > 200 else "0"
                       for c in range(32))
        out[r] = bits
    return out


def smooth_row(px, y, w, h):
    ys = [yy for yy in (y - 8, y - 4, y, y + 4, y + 8) if 0 <= yy < h]
    return [sum(px[x, yy] for yy in ys) / len(ys) for x in range(w)]


def find_bands(px, w, h, debug):
    """grid bands of the photo.  The grid is the big lit area (grey/white
    cells); rows are separated by thin dark lines that photo blur partly
    washes out, so: brightness profile across the grid, separator dips, band
    height = median dip spacing, then walk up from the grid's bottom edge and
    re-sync to the nearest dip each band (perspective changes the height)."""
    lit = []
    for y in range(h):
        row = [px[x, y] for x in range(0, w, 4)]
        lit.append(sum(1 for v in row if v > 90) > len(row) * 0.35)
    runs, start = [], None
    for y, on in enumerate(lit + [False]):
        if on and start is None:
            start = y
        elif not on and start is not None:
            runs.append([start, y - 1])
            start = None
    merged = []
    for r in runs:                       # close small gaps (dark separator lines)
        if merged and r[0] - merged[-1][1] < 12:
            merged[-1][1] = r[1]
        elif r[1] - r[0] > 12:
            merged.append(r)
    if not merged:
        return []
    top, bottom = max(merged, key=lambda r: r[1] - r[0])
    mid = (top + bottom) // 2
    xs = [x for x in range(w) if px[x, mid] > 90]
    gw = xs[-1] - xs[0]
    x0, x1 = xs[0] + gw // 10, xs[-1] - gw // 10
    cols = range(x0, x1, 3)
    prof = [sum(px[x, y] for x in cols) / len(cols) for y in range(h)]
    # a band is 34 px of the 480 px wide panel: height from the grid width
    bh = gw * 34.0 / PANEL_W
    dips = []
    for y in range(max(top + 3, 7), min(bottom - 3, h - 7)):
        win = prof[max(top, y - 20):y + 21]
        if prof[y] == min(prof[y - 6:y + 7]) and prof[y] < sum(win) / len(win) - 4:
            if not dips or y - dips[-1] > 15:
                dips.append(y)
    bands, cur = [], bottom
    while cur - top > bh * 0.6:
        lo, hi = int(cur - 1.25 * bh), int(cur - 0.75 * bh)
        cand = [d for d in dips if lo <= d <= hi]
        sep = min(cand, key=lambda d: prof[d]) if cand else int(cur - bh)
        bands.append((max(sep, top), cur))
        cur = sep
    bands.reverse()
    find_bands.xrange = (xs[0], xs[-1])
    if debug:
        print("grid y %d-%d x %d-%d, band height %.0f, %d bands" % (top, bottom, x0, x1, bh, len(bands)), file=sys.stderr)
    return bands


def decode_photo(img, last_row, debug):
    g = img.convert("L")
    w, h = g.size
    px = g.load()
    bands = find_bands(px, w, h, debug)
    out = {}
    for i, (y0, y1) in enumerate(reversed(bands)):
        r = last_row - i
        if r < 0:
            break
        y = (y0 + y1) // 2
        row = smooth_row(px, y, w, h)
        # grid edges on this line: first / last lit pixel near the grid
        gx0, gx1 = find_bands.xrange
        m = (gx1 - gx0) // 10
        # adaptive "lit" level: halfway between the dark surroundings and the
        # grid's grey (one side of a photo can be much darker than the other)
        inner = sorted(row[gx0 + (gx1 - gx0) // 5:gx1 - (gx1 - gx0) // 5])
        dark = sorted(row)[len(row) // 20]
        thr = (dark + inner[len(inner) // 3]) / 2
        lit = [x for x in range(max(0, gx0 - m), min(w, gx1 + m)) if row[x] > thr]
        if len(lit) < (gx1 - gx0) // 2:
            continue
        left, right = lit[0], lit[-1]
        cw = (right - left) / 32.0
        vals = []
        for c in range(32):
            seg = row[int(left + (c + .3) * cw):int(left + (c + .7) * cw)] or [0]
            vals.append(sum(seg) / len(seg))
        base = sorted(vals)[12]                     # grey level of this row
        bits = "".join("1" if v > base + 45 else "0" for v in vals)
        if debug:
            print("row %d: y %d-%d edges %d-%d grey %d" % (r, y0, y1, left, right, base), file=sys.stderr)
        out[r] = bits
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image")
    ap.add_argument("--rotate", type=int, default=0, choices=(0, 90, 180, 270),
                    help="rotate counter-clockwise first (90: photo with the panel top on the right)")
    ap.add_argument("--last-row", type=int, default=22, help="row number of the lowest band (default 22)")
    ap.add_argument("--exact", action="store_true", help="emulator screenshot, 480x800, no detection")
    ap.add_argument("--rows", default="8-22", help="rows to read with --exact (default 8-22)")
    ap.add_argument("--crop", help="x0,y0,x1,y1: cut the photo to the grid first (after --rotate), "
                    "when glare or bright surroundings merge with it")
    ap.add_argument("--debug", action="store_true")
    a = ap.parse_args()

    img = Image.open(a.image)
    if a.rotate:
        img = img.rotate(a.rotate, expand=True)
    if a.crop:
        img = img.crop(tuple(int(v) for v in a.crop.split(",")))
    if a.exact:
        lo, hi = (int(v) for v in a.rows.split("-"))
        rows = decode_exact(img, range(lo, hi + 1))
    else:
        rows = decode_photo(img, a.last_row, a.debug)

    for r in sorted(rows):
        v = int(rows[r], 2)
        if v == 0 and a.exact:
            continue
        print("row %2d  %s  0x%08X" % (r, rows[r], v))
    if 19 in rows:
        code = int(rows[19], 2)
        params = ["0x%08X" % int(rows[r], 2) for r in (20, 21, 22) if r in rows]
        name = BUGCHECKS.get(code)
        if name:
            print("bugcheck 0x%X %s  P1-P3 %s" % (code, name, " ".join(params)))
        else:
            print("row 19 = 0x%X (a stop code if this is a bugcheck screen; HAL_TRACE builds use rows 19-22 too)" % code)


if __name__ == "__main__":
    main()
