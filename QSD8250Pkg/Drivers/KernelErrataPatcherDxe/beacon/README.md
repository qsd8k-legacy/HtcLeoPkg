# Beacon / payload mapper blob

`mark.c` is the source of `mBeaconMark[]` in `KernelErrataPatcherDxe.c`
(position independent, no relocations, constants only):

- `BeaconMark` (offset 0): paints milestone stripes / 32-bit rows on the panel
  from any kernel context (borrowed PAGEVRFY PTEs through the 0xC0000000 self-map).
- `BeaconMapPayload` (`BEACON_MAP_OFFSET`): called once from the milestone 0
  stub; maps the firmware-reserved debug payload, sets TTBR0.S, and turns all
  write-back Normal memory write-through (NMRR), which the Scorpion's
  non-coherent table walker needs (Windows never cleans its PTEs).

Rebuild:

    clang --target=armv7a-none-eabi -mthumb -march=armv7-a -Os -ffreestanding \
      -fno-builtin -fomit-frame-pointer -fno-jump-tables -c mark.c -o mark.o
    llvm-readelf -r mark.o          # only .ARM.exidx relocations allowed
    llvm-nm -n mark.o               # BeaconMark must be at 0; update BEACON_MAP_OFFSET
    llvm-objcopy -O binary -j .text mark.o mark.bin

then paste `mark.bin` as bytes into `mBeaconMark[]`.
