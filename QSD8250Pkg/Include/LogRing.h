/** @file
  Boot log ring: every DEBUG() byte from PrePi on is also stored here, so a
  late transport (UsbLogDxe, over USB) can ship the whole log to a host.

  Fixed 1 MiB right below the KEP debug payload (0x2CB00000-0x2CC00000),
  reserved in Include/Device/MemoryMap.h ("Log Ring").  PrePi resets it on
  every boot.

  Head counts bytes ever written; the byte for position N lives at
  Data[N % LOG_RING_DATA_SIZE].  A reader keeps its own position and skips
  forward if it falls more than one ring behind.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef __LOG_RING_H__
#define __LOG_RING_H__

#define LOG_RING_BASE       0x2CB00000
#define LOG_RING_SIZE       0x00100000
#define LOG_RING_MAGIC      SIGNATURE_32 ('L', 'O', 'G', 'R')

typedef struct {
  UINT32  Magic;
  UINT32  Head;
  UINT32  Reserved[2];
  UINT8   Data[1];
} BOOT_LOG_RING;

#define LOG_RING_DATA_SIZE  (LOG_RING_SIZE - OFFSET_OF (BOOT_LOG_RING, Data))
#define LOG_RING            ((volatile BOOT_LOG_RING *)(UINTN)LOG_RING_BASE)

/**
  Start an empty log (PrePi, once per boot).
**/
STATIC inline
VOID
LogRingReset (
  VOID
  )
{
  LOG_RING->Head  = 0;
  LOG_RING->Magic = LOG_RING_MAGIC;
}

/**
  Append bytes; a no-op until LogRingReset() has run this boot.
**/
STATIC inline
VOID
LogRingAppend (
  IN CONST UINT8  *Buffer,
  IN UINTN        Length
  )
{
  volatile BOOT_LOG_RING  *Ring;
  UINT32                  Head;

  Ring = LOG_RING;
  if (Ring->Magic != LOG_RING_MAGIC) {
    return;
  }

  Head = Ring->Head;
  while (Length-- > 0) {
    Ring->Data[Head % LOG_RING_DATA_SIZE] = *Buffer++;
    Head++;
  }
  Ring->Head = Head;
}

#endif
