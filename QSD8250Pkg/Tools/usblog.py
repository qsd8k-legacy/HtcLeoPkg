#!/usr/bin/env python3
"""Receive the UEFI boot log from UsbLogDxe (1209:0001, bulk IN 0x81).

Waits for the phone, prints the log as it arrives and appends it to a file,
then waits again after a disconnect (every reboot is a new device).  Each
boot gets its own file: <dir>/boot-YYYYmmdd-HHMMSS.log, plus latest.log as
a symlink to the current one.

Uses libusb-1.0 through ctypes (no pip packages); run as root or add a udev
rule for 1209:0001.

    sudo python3 Tools/usblog.py [log-dir]
"""
import ctypes
import os
import sys
import time

VID, PID, EP_IN = 0x1209, 0x0001, 0x81
LIBUSB_ERROR_TIMEOUT = -7

lib = ctypes.CDLL("libusb-1.0.so.0")
lib.libusb_open_device_with_vid_pid.restype = ctypes.c_void_p
lib.libusb_open_device_with_vid_pid.argtypes = [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint16]
lib.libusb_bulk_transfer.argtypes = [ctypes.c_void_p, ctypes.c_ubyte, ctypes.c_char_p,
                                     ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.c_uint]
lib.libusb_claim_interface.argtypes = [ctypes.c_void_p, ctypes.c_int]
lib.libusb_close.argtypes = [ctypes.c_void_p]

logdir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "..", "..", "logusb")
os.makedirs(logdir, exist_ok=True)

if lib.libusb_init(None) != 0:
    sys.exit("libusb_init failed")

buf = ctypes.create_string_buffer(16384)
while True:
    h = lib.libusb_open_device_with_vid_pid(None, VID, PID)
    if not h:
        time.sleep(0.5)
        continue
    if lib.libusb_claim_interface(h, 0) != 0:
        lib.libusb_close(h)
        time.sleep(0.5)
        continue

    name = time.strftime("boot-%Y%m%d-%H%M%S.log")
    path = os.path.join(logdir, name)
    latest = os.path.join(logdir, "latest.log")
    if os.path.lexists(latest):
        os.unlink(latest)
    os.symlink(name, latest)
    print(f"--- connected, logging to {path}", file=sys.stderr, flush=True)

    with open(path, "ab", buffering=0) as out:
        while True:
            got = ctypes.c_int(0)
            rc = lib.libusb_bulk_transfer(h, EP_IN, buf, len(buf), ctypes.byref(got), 1000)
            if got.value:
                data = buf.raw[:got.value]
                out.write(data)
                sys.stdout.write(data.decode("ascii", "replace"))
                sys.stdout.flush()
            if rc not in (0, LIBUSB_ERROR_TIMEOUT):
                break   # disconnected (phone rebooted / unplugged)

    lib.libusb_close(h)
    print("\n--- disconnected", file=sys.stderr, flush=True)
