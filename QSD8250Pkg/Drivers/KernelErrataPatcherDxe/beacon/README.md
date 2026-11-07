# KEP beacon: panel "barcodes" from the Windows kernel

On the real HD2 there is no serial console and no debugger: the KEP
(KernelErrataPatcherDxe) paints progress and bugcheck information straight
into the framebuffer from inside the Windows kernel.  This directory holds
the code that does the painting (`mark.c`) and the tool that reads it back
from a photo (`decode_beacon.py`).

## What is on the panel

The 480x800 panel is split into bands of 34 pixel rows (64 KiB of
framebuffer each), band N starting at framebuffer offset N * 64K.

| what | where | when |
|---|---|---|
| bugcheck rows | rows 19, 20, 21, 22 = stop code, P1, P2, P3 | always (KeBugCheck2 hook) |
| milestone stripes | left 64 px of band N = milestone N | `BEACON_MILESTONES` |
| HAL call trace | rows 8-22 (see `qemu-hd2/build/claude/leont/stage-hw/README.md`) | `HAL_TRACE` |
| CPU registers | rows 20, 21, 22 = SCTLR, PRRR, NMRR (before the KEP's change) | `BEACON_DIAG_ROWS` |

A "row" is a 32-bit value painted across the full width as 32 cells of 15 px:
**bit 31 is the leftmost cell, white = 1, dark grey = 0**, with 1 px black
gaps between cells and a black line on top of each band.

Reading one by hand: write the 32 cells left to right as 1 (white) / 0
(grey), that is the binary value, most significant bit first.  Example, the
real HD2's first bugcheck after the HAL hang was fixed (beacon v18):

    row 19  00000000000000000000000001011100  0x5C        HAL_INITIALIZATION_FAILED
    row 20  00000000000000000000001000000000  0x200       P1
    row 21  00000101001001010000011000110001  0x05250631  P2
    row 22  00000000000000000000000000000010  0x2         P3

Milestone stripes: band N's left column, colours repeat every 7: red, green,
yellow, blue, magenta, cyan, white.  The lowest stripe is the last milestone
reached (table in `KernelErrataPatcherDxe.c`, `mMilestones[]`): 0 =
KiSystemStartup -> KiInitializeBootStructures ... 7 = HalInitializeProcessor
returned, 14 = HalInitSystem(0) returned, 16 = MmInitSystem called, 17 =
InbvDriverInitialize ... 21 = Phase 1 SeInitSystem.  Stripe 2 is never
painted on the HD2 (path not taken).  Each stripe is two calls (left / right
32 px); a stripe with only its left half means the painter itself hung.

## decode_beacon.py

    decode_beacon.py photo.jpg                  # bottom band = row 22
    decode_beacon.py photo.jpg --rotate 90      # phone photographed sideways
                                                #   (panel top on the right)
    decode_beacon.py photo.jpg --last-row 20    # grid ends at row 20
    decode_beacon.py photo.jpg --crop x0,y0,x1,y1   # cut to the grid first
    decode_beacon.py shot.png --exact           # emulator screendump, exact
    decode_beacon.py photo.jpg --debug          # band / edge detection

It prints every row as bits and hex, and names the stop code when row 19 is
one.  How it reads a photo:

1. The grid = the largest lit area (grey/white cells); the dark separator
   lines are bridged.
2. Band height from the panel geometry: 34 / 480 of the grid width (no need
   to find every separator; perspective is handled by re-syncing each band
   to the nearest separator dip, walking up from the grid's bottom edge).
3. The bottom band is `--last-row` (22 unless the grid ends earlier).
4. Per row: grid edges = first / last pixel above an adaptive level (halfway
   between the dark surroundings and the row's grey), 32 equal cells, the
   middle 40 % of each cell sampled, white = more than 45 above the row's
   grey level.

`--exact` skips all of that for emulator screenshots (cell c at x = 15c + 7).

Verified against hand-read photos: beaconv12 (`--last-row 20`), v15, v16
(TTBR0/TTBR1 rows), v18 (`--rotate 90`), and emulator screenshots.  Known
failure: beaconv13 (heavy glare, the right half far darker) - read such
photos by hand.  Tips for photos: whole grid in frame, phone flat, no lamp
reflecting on the lower rows; sideways is fine (`--rotate 90`).

Needs Pillow.  Run it with `python3 -I`.

## mark.c

Source of `mBeaconMark[]` in `KernelErrataPatcherDxe.c` (position
independent, no relocations, constants only):

- `BeaconMark(id, borrow, value)` (offset 0): paints a stripe or a 32-bit
  row from any kernel context.  It borrows the 16 PTEs of PAGEVRFY (never
  runs) through Windows' page-table self-map at 0xC0000000, points them at
  the framebuffer band, paints, cleans and restores the PTEs.
- `BeaconMapPayload(va, phys, pages, flags)` (`BEACON_MAP_OFFSET`): called
  once at milestone 0, before the HAL.  Maps the firmware-reserved debug
  payload (MemoryMap.h "KEP Debug Payload") at nt + 0x450000, and per flags:
  1 = TTBR0.S, 2 = Normal memory write-through (NMRR; required on the HD2),
  4 = paint SCTLR / PRRR / NMRR on rows 20-22.

Rebuild after a change:

    clang --target=armv7a-none-eabi -mthumb -march=armv7-a -Os -ffreestanding \
      -fno-builtin -fomit-frame-pointer -fno-jump-tables -c mark.c -o mark.o
    llvm-readelf -r mark.o          # only .ARM.exidx relocations allowed
    llvm-nm -n mark.o               # BeaconMark must be at 0; update BEACON_MAP_OFFSET
    llvm-objcopy -O binary -j .text mark.o mark.bin

then paste `mark.bin` as bytes into `mBeaconMark[]`.

## KEP switches (KernelErrataPatcherDxe.c)

| switch | default | |
|---|---|---|
| `BEACON_ENABLE` | 1 | bugcheck rows + payload mapper (needed for the NMRR fix) |
| `BEACON_STALL` | 1 | spin after painting a bugcheck |
| `BEACON_MILESTONES` | 0 | milestone stripes |
| `BEACON_DIAG_ROWS` | 0 | SCTLR / PRRR / NMRR rows |
| `HAL_TRACE` | 0 | hal.dll call brackets (rows 8-22, stubs in the payload) |
| `KEP_TTBR_SHARED` | 1 | TTBR0.S (no visible effect on the HD2) |
| `KEP_NMRR_WT` | 1 | write-back Normal memory -> write-through: **required** |
