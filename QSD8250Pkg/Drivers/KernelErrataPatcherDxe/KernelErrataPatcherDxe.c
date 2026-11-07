/** @file
  Patches Windows RT ntoskrnl KiSetFeatureBits to bypass 0x5D
  (UNSUPPORTED_PROCESSOR) bugcheck on Qualcomm Scorpion (QSD8250).

  Hooks ExitBootServices. When winload calls it, ntoskrnl is already
  loaded in memory. The driver
   1. NOPs the three KiSetFeatureBits branches that bugcheck 0x5D(2) on
      ID_MMFR2 TLB level and ID_MMFR3 broadcast / coherent walk.
   2. Rewrites, in ntoskrnl's executable sections, every CP15 op the
      Scorpion does not implement (UNDEF on a real HD2, CacheCoherencyTest)
      into its local equivalent; same 4-byte MCR, only the 2nd halfword
      changes.  The core is uniprocessor, so local == broadcast:
        TLBIALLIS  c8,c3,0 -> TLBIALL  c8,c7,0
        TLBIMVAIS  c8,c3,1 -> TLBIMVA  c8,c7,1
        TLBIASIDIS c8,c3,2 -> TLBIASID c8,c7,2
        TLBIMVAAIS c8,c3,3 -> TLBIALL  c8,c7,0  (MVA+any ASID: no local op;
        TLBIMVAA   c8,c7,3 -> TLBIALL  c8,c7,0   TLBIMVA misses non-global)
        ICIALLUIS  c7,c1,0 -> ICIALLU  c7,c5,0
        BPIALLIS   c7,c1,6 -> BPIALL   c7,c5,6
  The Scorpion's table walk does NOT see L1 with the attributes Windows runs
  (TTBR0 & 0x7F = 0x09, measured on the phone; setting S did not help), and
  with the MMFR3 check gone Windows never cleans its PTEs.  The HAL's mapper
  (HalpMap / HalpUnmapVirtualAddress) is therefore redirected to stubs that
  clean the written PTEs to PoC before TLBIALL (real HD2, beacon v18: the
  silent hang in HAL init is gone).  For everything else (Mm) the payload
  mapper, run at the first kernel milestone, turns all write-back Normal
  memory write-through (NMRR, KEP_NMRR_WT): PTE stores reach memory, where
  the walker reads.  With that the real HD2 boots to the spinner.

  Debug aids (panel "barcodes": bugcheck rows always, milestone stripes /
  HAL call trace / register rows behind switches) and the photo decoder:
  beacon/README.md, beacon/decode_beacon.py.

  Copyright (c) 2024, J0SH1X. All rights reserved.
  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <Uefi.h>
#include <Library/UefiLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/DebugLib.h>
#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/CacheMaintenanceLib.h>

/* ------------------------------------------------------------------ */
/*  Byte patterns for the four failing KiSetFeatureBits checks.       */
/*  Each pattern is the MRC + AND + CMP that precedes the BCC.W we    */
/*  want to NOP.  The BCC.W itself is the 4 bytes right after.        */
/* ------------------------------------------------------------------ */

/*
 * Check 1 – ID_MMFR2[19:16] (TLB maintenance) >= 3
 *   MRC p15,0,R3,c0,c1,6   10 EE D1 3F
 *   AND.W R3,R3,#0xF0000   03 F4 70 23
 *   CMP.W R3,#0x30000      B3 F5 40 3F
 *   BCC.W <bugcheck>        xx xx xx xx  ← NOP this
 */
STATIC CONST UINT8 PatMmfr2Tlb[] = {
  0x10, 0xEE, 0xD1, 0x3F,
  0x03, 0xF4, 0x70, 0x23,
  0xB3, 0xF5, 0x40, 0x3F
};

/*
 * Check 2 – ID_MMFR3[15:12] (maintenance broadcast) >= 2
 *   MRC p15,0,R3,c0,c1,7   10 EE F1 3F
 *   AND.W R3,R3,#0xF000    03 F4 70 43
 *   CMP.W R3,#0x2000       B3 F5 00 5F
 *   BCC.W <bugcheck>        xx xx xx xx  ← NOP this
 */
STATIC CONST UINT8 PatMmfr3Bcast[] = {
  0x10, 0xEE, 0xF1, 0x3F,
  0x03, 0xF4, 0x70, 0x43,
  0xB3, 0xF5, 0x00, 0x5F
};

/*
 * Check 3 – ID_MMFR3[23:20] (coherent page-table walk) >= 1
 *   MRC p15,0,R3,c0,c1,7   10 EE F1 3F
 *   AND.W R3,R3,#0xF00000  03 F4 70 03
 *   CMP.W R3,#0x100000     B3 F5 80 1F
 *   BCC.W <bugcheck>        xx xx xx xx  ← NOP this
 */
STATIC CONST UINT8 PatMmfr3Walk[] = {
  0x10, 0xEE, 0xF1, 0x3F,
  0x03, 0xF4, 0x70, 0x03,
  0xB3, 0xF5, 0x80, 0x1F
};

/*
 * Check 4 - CTR[15:14] L1 I-cache policy must be 2 (VIPT) or 3 (PIPT);
 * Scorpion reports 1 (ASID-tagged VIVT) -> 0x5D(8, CTR).  Safe to accept:
 * KeSweepIcacheRange does a full ICIALLU whenever CTR is not PIPT.
 *   UBFX R3,R4,#14,#2   C4 F3 81 33
 *   CMP  R3,#2          02 2B
 *   BNE.W <check PIPT / bugcheck 8>   <- NOP this
 */
STATIC CONST UINT8 PatCtrL1Ip[] = { 0xC4, 0xF3, 0x81, 0x33, 0x02, 0x2B };

STATIC UINTN PatchKernelMcr (UINT8 *P);
STATIC VOID  InstallBeacon (UINT8 *KernelBase);

/* Two 16-bit Thumb NOPs = 4 bytes */
STATIC CONST UINT8 Nop4[] = { 0x00, 0xBF, 0x00, 0xBF };

/* ------------------------------------------------------------------ */
/*  Helper: validate that 4 bytes look like a Thumb-2 B<cond>.W       */
/* ------------------------------------------------------------------ */
STATIC
BOOLEAN
IsThumb2CondBranch (
  CONST UINT8  *P
  )
{
  /*
   * Thumb-2 B<cond>.W encoding (T3):
   *   First halfword:  1111 0 S cond(4) imm6(6)
   *   Second halfword: 10 J1 0 J2 imm11(11)
   *
   * In little-endian memory:
   *   P[1] bits [7:4] = 1111, bits [3:2] = 0 S, bits [1:0] = cond[3:2]
   *   P[3] bit  [7]   = 1, bit [6] = 0, bit [4] = 0
   *
   * We accept any condition code.
   */
  if ((P[1] & 0xF0) != 0xF0)
    return FALSE;
  if ((P[3] & 0xD0) != 0x80)
    return FALSE;
  return TRUE;
}

/* ------------------------------------------------------------------ */
/*  Scan a memory range and apply patches                             */
/* ------------------------------------------------------------------ */

/* Patch the 4-byte B<cond>.W right after Pat at Base+Off, if it is one */
STATIC
BOOLEAN
PatchAt (
  UINT8          *Base,
  UINTN          Off,
  UINTN          Size,
  CONST UINT8    *Pat,
  UINTN          PatLen,
  CONST CHAR8    *Name
  )
{
  UINT8  *Bcc;

  if ((Off + PatLen + 4 > Size) || (CompareMem (Base + Off, Pat, PatLen) != 0)) {
    return FALSE;
  }

  Bcc = Base + Off + PatLen;
  if (!IsThumb2CondBranch (Bcc)) {
    return FALSE;
  }

  CopyMem (Bcc, Nop4, sizeof (Nop4));
  WriteBackInvalidateDataCacheRange (Bcc, sizeof (Nop4));
  InvalidateInstructionCacheRange (Bcc, sizeof (Nop4));
  DEBUG ((DEBUG_ERROR, "KernelErrataPatcher: patched %a at %p\n", Name, Bcc));
  return TRUE;
}

/* The three MMFR checks are back to back (16 bytes apart) in KiSetFeatureBits. */
STATIC
UINTN
PatchRange (
  UINT8  *Base,
  UINTN  Size
  )
{
  UINTN  Count = 0;
  UINTN  i;

  for (i = 0; i + sizeof (PatMmfr2Tlb) + 4 <= Size; i += 2) {
    Count += PatchAt (Base, i, Size, PatCtrL1Ip, sizeof (PatCtrL1Ip), "CTR L1Ip VIVT");

    if (CompareMem (Base + i, PatMmfr2Tlb, sizeof (PatMmfr2Tlb)) != 0) {
      continue;
    }

    /* MMFR2 check, then MMFR3 broadcast and walk, 16 bytes apart */
    if ((i + 48 > Size) ||
        (CompareMem (Base + i + 16, PatMmfr3Bcast, sizeof (PatMmfr3Bcast)) != 0) ||
        (CompareMem (Base + i + 32, PatMmfr3Walk, sizeof (PatMmfr3Walk)) != 0))
    {
      continue;
    }

    Count += PatchAt (Base, i, Size, PatMmfr2Tlb, sizeof (PatMmfr2Tlb), "MMFR2 TLB-maint");
    Count += PatchAt (Base, i + 16, Size, PatMmfr3Bcast, sizeof (PatMmfr3Bcast), "MMFR3 broadcast");
    Count += PatchAt (Base, i + 32, Size, PatMmfr3Walk, sizeof (PatMmfr3Walk), "MMFR3 coherent-walk");


    /* rewrite the UNDEF CP15 ops in the image holding this KiSetFeatureBits */
    Count += PatchKernelMcr (Base + i);
  }

  return Count;
}

/* ------------------------------------------------------------------ */
/*  UNDEF CP15 ops -> local equivalents                               */
/* ------------------------------------------------------------------ */

/*
  Thumb-2 MCR p15, 0, Rt, CRn, CRm, op2:
    hw1 = 0xEE00 | CRn            (op1 = 0, L = 0)
    hw2 = Rt<<12 | 0x0F10 | op2<<5 | CRm
*/
typedef struct {
  UINT16    Hw1;
  UINT16    Hw2;      /* without Rt */
  UINT16    NewHw2;   /* without Rt */
} MCR_FIX;

#define MCR_HW2(op2, crm)  (UINT16)(0x0F10 | ((op2) << 5) | (crm))

STATIC CONST MCR_FIX  mMcrFix[] = {
  { 0xEE08, MCR_HW2 (0, 3), MCR_HW2 (0, 7) },   /* TLBIALLIS  -> TLBIALL  */
  { 0xEE08, MCR_HW2 (1, 3), MCR_HW2 (1, 7) },   /* TLBIMVAIS  -> TLBIMVA  */
  { 0xEE08, MCR_HW2 (2, 3), MCR_HW2 (2, 7) },   /* TLBIASIDIS -> TLBIASID */
  { 0xEE08, MCR_HW2 (3, 3), MCR_HW2 (0, 7) },   /* TLBIMVAAIS -> TLBIALL  */
  { 0xEE08, MCR_HW2 (3, 7), MCR_HW2 (0, 7) },   /* TLBIMVAA   -> TLBIALL  */
  { 0xEE07, MCR_HW2 (0, 1), MCR_HW2 (0, 5) },   /* ICIALLUIS  -> ICIALLU  */
  { 0xEE07, MCR_HW2 (6, 1), MCR_HW2 (6, 5) },   /* BPIALLIS   -> BPIALL   */
};

STATIC
UINTN
RewriteMcr (
  UINT8  *Start,
  UINTN  Size
  )
{
  UINTN   Count = 0;
  UINTN   i;
  UINTN   f;
  UINT16  *H;

  for (i = 0; i + 4 <= Size; i += 2) {
    H = (UINT16 *)(Start + i);
    if ((H[0] & 0xFFF0) != 0xEE00) {
      continue;
    }

    for (f = 0; f < ARRAY_SIZE (mMcrFix); f++) {
      if ((H[0] == mMcrFix[f].Hw1) && ((H[1] & 0x0FFF) == mMcrFix[f].Hw2)) {
        H[1] = (H[1] & 0xF000) | mMcrFix[f].NewHw2;
        Count++;
        i += 2;                       /* skip the 2nd halfword */
        break;
      }
    }
  }

  return Count;
}

/*
  Find the PE image that contains P: walk back page by page to an 'MZ'
  header whose ARM PE covers P, then rewrite its executable sections.
*/
#define MAX_IMAGE_SIZE  0x02000000

STATIC
UINTN
PatchKernelMcr (
  UINT8  *P
  )
{
  UINTN                 Page;
  UINT8                 *Base = NULL;
  UINT32                PeOff;
  UINT16                NumSec, OptSize;
  UINT32                SizeOfImage;
  UINT8                 *Sec;
  UINTN                 s;
  UINTN                 Count = 0;
  UINT32                Va, VSize, Chars;

  for (Page = (UINTN)P & ~(UINTN)0xFFF; (UINTN)P - Page < MAX_IMAGE_SIZE; Page -= 0x1000) {
    UINT8  *M = (UINT8 *)Page;

    if ((M[0] != 'M') || (M[1] != 'Z')) {
      continue;
    }

    PeOff = ReadUnaligned32 ((UINT32 *)(M + 0x3C));
    if ((PeOff > 0x1000) || (ReadUnaligned32 ((UINT32 *)(M + PeOff)) != 0x00004550) ||
        (ReadUnaligned16 ((UINT16 *)(M + PeOff + 4)) != 0x01C4))
    {
      continue;
    }

    SizeOfImage = ReadUnaligned32 ((UINT32 *)(M + PeOff + 24 + 56));
    if ((UINTN)P - Page < SizeOfImage) {
      Base = M;
    }

    break;
  }

  if (Base == NULL) {
    DEBUG ((DEBUG_ERROR, "KernelErrataPatcher: ntoskrnl PE header not found\n"));
    return 0;
  }

  NumSec  = ReadUnaligned16 ((UINT16 *)(Base + PeOff + 6));
  OptSize = ReadUnaligned16 ((UINT16 *)(Base + PeOff + 20));
  Sec     = Base + PeOff + 24 + OptSize;
  for (s = 0; s < NumSec; s++, Sec += 40) {
    VSize = ReadUnaligned32 ((UINT32 *)(Sec + 8));
    Va    = ReadUnaligned32 ((UINT32 *)(Sec + 12));
    Chars = ReadUnaligned32 ((UINT32 *)(Sec + 36));
    if (((Chars & 0x20000000) == 0) || (Va + VSize > SizeOfImage)) {
      continue;                         /* not IMAGE_SCN_MEM_EXECUTE */
    }

    Count += RewriteMcr (Base + Va, VSize);
    WriteBackInvalidateDataCacheRange (Base + Va, VSize);
    InvalidateInstructionCacheRange (Base + Va, VSize);
  }

  DEBUG ((DEBUG_ERROR, "KernelErrataPatcher: ntoskrnl at %p, %u CP15 ops rewritten\n", Base, Count));

  InstallBeacon (Base);
  return Count;
}

/* ------------------------------------------------------------------ */
/*  Progress beacon v2: milestone stripes on the panel from early kernel */
/*  code, no boot graphics and no UART needed (the HD2 has none).        */
/*                                                                       */
/*  BeaconMark (leont/src/beacon/mark.c) borrows 16 PTEs of an unused    */
/*  kernel-image range through the Windows ARM page-table self-map      */
/*  (0xC0000000), points them at framebuffer phys 0x02A00000+id*64K,     */
/*  paints stripe <id> (34 rows, ids 0..22 top to bottom) and restores   */
/*  the PTEs.  A boot-path BL to a known function is retargeted to a     */
/*  stub that paints and then tail-branches to the original function,   */
/*  so the lowest stripe on screen is the last milestone reached.  The   */
/*  KeBugCheck2 hook paints the stop code and P1-P3 as 32-bit rows       */
/*  19-22 (white cell = 1, bit 31 left) and (BEACON_STALL) spins: early   */
/*  bugchecks, before boot graphics, become visible.                     */
/* ------------------------------------------------------------------ */

#define BEACON_ENABLE      1
#define BEACON_STALL       1            /* bugcheck: paint then spin      */
/* Options (debug aids stay in, guarded):
   BEACON_MILESTONES  kernel milestone stripes in the left column
   BEACON_DIAG_ROWS   mapper paints SCTLR / PRRR / old NMRR on rows 20-22
   KEP_TTBR_SHARED    mapper sets TTBR0.S (no visible effect on the HD2)
   KEP_NMRR_WT        mapper turns write-back Normal memory write-through:
                      REQUIRED on the real HD2 (non-coherent table walk,
                      Windows never cleans its PTEs) - leave on
   The bugcheck rows (19-22, KeBugCheck2 hook) are always on. */
#define BEACON_MILESTONES  0
#define BEACON_DIAG_ROWS   0
#define KEP_TTBR_SHARED    1
#define KEP_NMRR_WT        1
#define KEP_MAP_FLAGS      ((KEP_TTBR_SHARED ? 1 : 0) | (KEP_NMRR_WT ? 2 : 0) | (BEACON_DIAG_ROWS ? 4 : 0))
#define BEACON_HOOK_RVA    0x0018148C   /* KeBugCheck2 entry (stmdb.w)    */
#define BEACON_BUGCHECK_ID 19           /* rows 19..22: code, P1, P2, P3   */
#define BEACON_CAVE_RVA    0x001BE23C   /* zero pad at end of .text       */
#define BEACON_BORROW_RVA  0x00440000   /* PAGEVRFY: 16 pages, never run  */
#define BEACON_CAVE_END_RVA 0x001BF000  /* .text's last page ends here    */

/*
  Debug payload: the cave in .text holds only BeaconMark, the bugcheck
  trampoline, the milestone stubs and the HAL TLB fix.  Everything else (the
  hal.dll trace stubs, as many as needed) goes to a firmware-reserved region
  (MemoryMap.h "KEP Debug Payload", top of HLOS 3, EfiReservedMemoryType).
  The milestone 0 stub first calls BeaconMapPayload, which points the PTEs
  of PAGEVRFY pages 16..29 (never run; pages 0..15 are the painter's borrow)
  at it through the 0xC0000000 self-map: kernel VA = nt + KEP_PAYLOAD_RVA.
*/
#define KEP_PAYLOAD_PHYS   0x2CC00000
#define KEP_PAYLOAD_SIZE   0x00100000   /* reserved in the memory map     */
#define KEP_PAYLOAD_RVA    0x00450000
#define KEP_PAYLOAD_PAGES  14           /* mapped at the kernel VA: 56 KiB */

STATIC UINT8   *mBeaconKernel;       /* ntoskrnl, physical (= where we patch) */
STATIC UINT8   *mBeaconNextStub;     /* free space in the cave after the milestone stubs */
STATIC UINT8   *mBeaconMarkPhys;
STATIC BOOLEAN mPayloadMapped;       /* milestone 0 maps the payload    */
STATIC UINT8   *mStubMark;           /* BeaconMark the HAL stubs call (payload copy) */
STATIC UINT32  mStubBorrowVa;        /* absolute borrow VA for the payload stubs */

typedef struct {
  UINT32  Site;     /* RVA of a BL                          */
  UINT32  Target;   /* RVA it must call (checked)           */
} BEACON_MILESTONE;

/* Index = stripe id.  Kernel entry through Phase 0, then two Phase 1 points.
   1..7 bracket every call KiInitializeBootStructures makes before
   KiSetFeatureBits (the real HD2 stopped between 0 and 1 of the old list;
   7 = HAL!HalInitializeProcessor returned). */
STATIC CONST BEACON_MILESTONE  mMilestones[] = {
  { 0x0023C060, 0x0023DD04 },   /*  0 KiSystemStartup -> KiInitializeBootStructures */
  { 0x0023DDAE, 0x00021D60 },   /*  1   -> memset                                  */
  { 0x0023DDD4, 0x000F08D0 },   /*  2   -> KiInitializeExceptionVectorTable (a)    */
  { 0x0023DDEA, 0x000F09E0 },   /*  3   -> RtlInitTickLock                         */
  { 0x0023DDF0, 0x000F08D0 },   /*  4   -> KiInitializeExceptionVectorTable (b)    */
  { 0x0023DDFC, 0x004BD8B8 },   /*  5   -> InbvDriverInitialize                    */
  { 0x0023DE0C, 0x000E5C90 },   /*  6   -> strstr (next: HalInitializeProcessor)   */
  { 0x0023DE26, 0x0023E274 },   /*  7   -> KiSetFeatureBits                        */
  { 0x0023C06E, 0x0045F1CC },   /*  8 KiSystemStartup -> KdInitSystem              */
  { 0x0023C0A2, 0x0023E3A4 },   /*  9 KiSystemStartup -> KiInitializeKernel        */
  { 0x0023E51A, 0x004C2F94 },   /* 10 KiInitializeKernel -> KiInitSystem           */
  { 0x0023E5DE, 0x004C9598 },   /* 11 KiInitializeKernel -> InitBootProcessor      */
  { 0x004C95AA, 0x004C52EC },   /* 12 InitBootProcessor -> ExpValidateLoader       */
  { 0x004C9612, 0x004BA170 },   /* 13   -> Whea default config (before HalInitSystem 0) */
  { 0x004C9682, 0x004BAD5C },   /* 14   -> KeInitializeClock (HalInitSystem 0 returned) */
  { 0x004C97A2, 0x004B6278 },   /* 15   -> ExInitSystem                            */
  { 0x004C97BE, 0x004C3DC0 },   /* 16   -> MmInitSystem                            */
  { 0x004C97D2, 0x004BD8B8 },   /* 17   -> InbvDriverInitialize                    */
  { 0x004C998E, 0x004CA914 },   /* 18   -> ObInitSystem                            */
  { 0x004C99A0, 0x004C111C },   /* 19   -> PsInitSystem                            */
  { 0x004C7140, 0x004B6278 },   /* 20 Phase1InitializationDiscard -> ExInitSystem  */
  { 0x004C71E2, 0x004B7EE8 },   /* 21   -> SeInitSystem                            */
};

/*
  BeaconMark(id, borrowVA): compiled from leont/src/beacon/mark.c with
  clang --target=armv7a-none-eabi -mthumb -Os -ffreestanding -fno-builtin
  -fno-jump-tables; verified: no relocations, no calls, constants only.
*/
#define BEACON_MAP_OFFSET  0x1C2     /* BeaconMapPayload in the blob (v19: + NMRR write-through) */
STATIC CONST UINT8  mBeaconMark[] = {
  0xff,0xe7,0x2d,0xe9,0xf0,0x4f,0x92,0xb0,0x01,0x92,0xef,0xf3,0x00,0x82,0x72,0xb6,
  0x6b,0x46,0x9b,0x0a,0x03,0xf1,0x40,0x43,0x23,0xf0,0x03,0x03,0x1f,0x68,0xbb,0x07,
  0x40,0xf1,0xc9,0x80,0x00,0xf0,0x3f,0x09,0x6f,0xf3,0xdf,0x27,0xb9,0xf1,0x16,0x0f,
  0x28,0xbf,0x4f,0xf0,0x16,0x09,0x47,0xea,0x09,0x47,0x4f,0xf6,0xfc,0x7b,0x47,0xf0,
  0x28,0x7c,0x02,0xad,0xc0,0xf2,0x3f,0x0b,0x00,0x26,0x4f,0xf0,0x40,0x4e,0x8b,0x19,
  0x0c,0xeb,0x06,0x07,0x06,0xf5,0x80,0x56,0x0b,0xea,0x93,0x23,0xb6,0xf5,0x80,0x3f,
  0x53,0xf8,0x0e,0x40,0x43,0xf8,0x0e,0x70,0x03,0xf1,0x40,0x43,0x45,0xf8,0x04,0x4b,
  0x07,0xee,0x3a,0x3f,0xeb,0xd1,0x00,0x92,0x00,0x27,0xbf,0xf3,0x4f,0x8f,0xcb,0x19,
  0x07,0xf5,0x80,0x57,0xb7,0xf5,0x80,0x3f,0x08,0xee,0x37,0x3f,0xf7,0xd1,0x25,0x23,
  0x4f,0xf0,0x7f,0x4c,0x09,0xfb,0x03,0xf3,0x48,0xf6,0x89,0x04,0xc8,0xf6,0x88,0x04,
  0x1b,0x0a,0xa3,0xeb,0xc3,0x03,0x4b,0x44,0x4f,0xea,0x89,0x39,0x4d,0x46,0xdf,0x07,
  0x04,0xbf,0x40,0xf2,0x00,0x0c,0xcf,0xf6,0xff,0x7c,0x01,0x27,0x57,0xfa,0x83,0xf3,
  0x9f,0x07,0x48,0xbf,0x0c,0xf5,0x7f,0x4c,0x5b,0x07,0x48,0xbf,0x0c,0xf1,0xff,0x0c,
  0x20,0x23,0x03,0xea,0x50,0x08,0x08,0xf1,0x20,0x02,0xbf,0xf3,0x4f,0x8f,0x00,0x27,
  0xbf,0xf3,0x6f,0x8f,0xa5,0xfb,0x04,0x36,0x33,0x0a,0x1b,0x01,0xa3,0xeb,0x16,0x23,
  0xa9,0xeb,0x43,0x13,0x07,0xeb,0x03,0x0e,0x03,0x06,0x05,0xd4,0xc6,0x45,0x22,0xd3,
  0x96,0x45,0x20,0xd2,0xe2,0x46,0x17,0xe0,0x1f,0xfa,0x8e,0xf3,0x48,0xf6,0x89,0x06,
  0x73,0x43,0x01,0x9e,0x4f,0xf0,0xff,0x3a,0xdb,0x0c,0x9e,0x40,0xa3,0xeb,0x03,0x13,
  0x73,0x44,0xf6,0x0f,0x04,0xbf,0x43,0xf2,0x30,0x0a,0xcf,0xf6,0x30,0x7a,0x9b,0xb2,
  0x0e,0x2b,0x08,0xbf,0x4f,0xf0,0x7f,0x4a,0xb7,0xf5,0xf0,0x7f,0x38,0xbf,0x4f,0xf0,
  0x7f,0x4a,0x41,0xf8,0x27,0xa0,0x01,0x37,0x01,0x35,0xb7,0xf5,0x80,0x4f,0xc9,0xd1,
  0x00,0x20,0xcf,0xf6,0xff,0x70,0x81,0x42,0x07,0xd2,0x01,0xf5,0x80,0x30,0x0a,0x46,
  0x07,0xee,0x3a,0x2f,0x20,0x32,0x82,0x42,0xfa,0xd3,0x6f,0xf0,0x03,0x00,0x02,0xad,
  0x00,0xea,0x91,0x20,0x00,0x22,0x4f,0xf0,0x40,0x47,0xbf,0xf3,0x4f,0x8f,0x86,0x18,
  0xab,0x58,0x06,0xea,0x0b,0x06,0x04,0x32,0xf3,0x51,0x06,0xf1,0x40,0x43,0x40,0x2a,
  0x07,0xee,0x3a,0x3f,0xf3,0xd1,0x00,0x20,0xbf,0xf3,0x4f,0x8f,0x0a,0x18,0x00,0xf5,
  0x80,0x50,0xb0,0xf5,0x80,0x3f,0x08,0xee,0x37,0x2f,0xf7,0xd1,0xbf,0xf3,0x4f,0x8f,
  0xbf,0xf3,0x6f,0x8f,0x00,0x9a,0x10,0x06,0x00,0xd4,0x62,0xb6,0x12,0xb0,0xbd,0xe8,
  0xf0,0x8f,0x2d,0xe9,0xf0,0x4f,0x83,0xb0,0xdf,0x07,0xef,0xf3,0x00,0x89,0x72,0xb6,
  0x6f,0x46,0x12,0xd0,0x12,0xee,0x10,0x6f,0xb5,0x07,0x0e,0xd4,0xbf,0xf3,0x4f,0x8f,
  0x46,0xf0,0x02,0x06,0x02,0xee,0x10,0x6f,0x00,0x26,0xbf,0xf3,0x6f,0x8f,0x08,0xee,
  0x17,0x6f,0xbf,0xf3,0x4f,0x8f,0xbf,0xf3,0x6f,0x8f,0xbf,0x0a,0x07,0xf1,0x40,0x47,
  0x27,0xf0,0x03,0x07,0x3f,0x68,0xbe,0x07,0x40,0xf1,0xbd,0x80,0x2a,0xb3,0x4f,0xf6,
  0xfc,0x7c,0x40,0xf2,0xfe,0x76,0x07,0xea,0x06,0x08,0xc0,0xf2,0x3f,0x0c,0x4f,0xf0,
  0x40,0x4e,0x04,0x46,0x16,0x46,0x0c,0xea,0x94,0x25,0x41,0xea,0x08,0x07,0x01,0xf5,
  0x80,0x51,0x04,0xf5,0x80,0x54,0x01,0x3e,0x45,0xf8,0x0e,0x70,0x05,0xf1,0x40,0x45,
  0x07,0xee,0x3a,0x5f,0xef,0xd1,0x01,0x46,0xbf,0xf3,0x4f,0x8f,0x08,0xee,0x37,0x1f,
  0x01,0xf5,0x80,0x51,0x01,0x3a,0xf9,0xd1,0x01,0xe0,0xbf,0xf3,0x4f,0x8f,0x00,0x21,
  0x07,0xee,0x15,0x1f,0x07,0xee,0xd5,0x1f,0x99,0x07,0xbf,0xf3,0x4f,0x8f,0xbf,0xf3,
  0x6f,0x8f,0x1a,0xee,0x32,0x4f,0x40,0xf1,0x70,0x80,0x00,0x21,0x00,0x22,0xcd,0xe9,
  0x01,0x09,0x24,0xfa,0x01,0xf7,0xfe,0x07,0x4f,0xf0,0x02,0x06,0x08,0xbf,0x07,0xf0,
  0x03,0x06,0x06,0xfa,0x01,0xf7,0x02,0x31,0x3a,0x43,0x20,0x29,0xf1,0xd1,0x00,0x94,
  0x00,0x21,0xbf,0xf3,0x4f,0x8f,0x0a,0xee,0x32,0x2f,0xbf,0xf3,0x6f,0x8f,0x08,0xee,
  0x17,0x1f,0xbf,0xf3,0x4f,0x8f,0xbf,0xf3,0x6f,0x8f,0x30,0xee,0x30,0x0f,0xc0,0xf3,
  0x02,0x6e,0xbe,0xf1,0x00,0x0f,0x37,0xd0,0x4f,0xf0,0x00,0x0a,0x0a,0xeb,0x4a,0x01,
  0x20,0xfa,0x01,0xf1,0x11,0xf0,0x06,0x0f,0x2a,0xd0,0x4f,0xea,0x4a,0x0b,0x40,0xee,
  0x10,0xbf,0x00,0x26,0xbf,0xf3,0x6f,0x8f,0x30,0xee,0x10,0x2f,0xc2,0xf3,0xc9,0x0c,
  0xc2,0xf3,0x4e,0x31,0x02,0xf0,0x07,0x02,0x01,0x31,0x04,0x32,0x67,0x46,0xbc,0xf1,
  0x00,0x0f,0x18,0xbf,0xb7,0xfa,0x87,0xf7,0x06,0xfa,0x07,0xf5,0x4f,0xf0,0x00,0x08,
  0x45,0xea,0x0b,0x05,0x08,0xfa,0x02,0xf9,0x08,0xf1,0x01,0x08,0x49,0xea,0x05,0x04,
  0x41,0x45,0x07,0xee,0x5e,0x4f,0xf5,0xd1,0x74,0x1c,0x66,0x45,0x26,0x46,0xeb,0xd1,
  0x0a,0xf1,0x01,0x0a,0xf2,0x45,0xc9,0xd1,0xbf,0xf3,0x4f,0x8f,0x00,0x20,0xbf,0xf3,
  0x6f,0x8f,0x07,0xee,0x15,0x0f,0x07,0xee,0xd5,0x0f,0xbf,0xf3,0x4f,0x8f,0xbf,0xf3,
  0x6f,0x8f,0xdd,0xf8,0x08,0x90,0xdd,0xe9,0x00,0x40,0x59,0x07,0x13,0xd5,0xa0,0xf5,
  0x80,0x35,0x94,0x20,0x11,0xee,0x10,0x2f,0x1a,0xee,0x12,0x6f,0x29,0x46,0xff,0xf7,
  0x48,0xfe,0x95,0x20,0x29,0x46,0x32,0x46,0xff,0xf7,0x43,0xfe,0x96,0x20,0x29,0x46,
  0x22,0x46,0xff,0xf7,0x3e,0xfe,0x5f,0xea,0x09,0x60,0x00,0xd4,0x62,0xb6,0x03,0xb0,
  0xbd,0xe8,0xf0,0x8f,

};

/* Encode a Thumb-2 32-bit branch (BL if Link, else B.W) at At -> Target. */
STATIC
VOID
ThumbBranch (
  UINT8    *At,
  UINT8    *Target,
  BOOLEAN  Link
  )
{
  INT32   Off = (INT32)((INTN)Target - ((INTN)At + 4));
  UINT32  S     = ((UINT32)Off >> 24) & 1;
  UINT32  I1    = ((UINT32)Off >> 23) & 1;
  UINT32  I2    = ((UINT32)Off >> 22) & 1;
  UINT32  Imm10 = ((UINT32)Off >> 12) & 0x3FF;
  UINT32  Imm11 = ((UINT32)Off >> 1) & 0x7FF;
  UINT32  J1    = ((~I1) & 1) ^ S;
  UINT32  J2    = ((~I2) & 1) ^ S;
  UINT16  Hw1   = (UINT16)(0xF000 | (S << 10) | Imm10);
  UINT16  Hw2   = (UINT16)((Link ? 0xD000 : 0x9000) | (J1 << 13) | (J2 << 11) | Imm11);

  At[0] = (UINT8)Hw1; At[1] = (UINT8)(Hw1 >> 8);
  At[2] = (UINT8)Hw2; At[3] = (UINT8)(Hw2 >> 8);
}

/* Encode Thumb-2 MOVW (Top=FALSE) / MOVT (Top=TRUE) Rd, #Imm16 at At. */
STATIC
VOID
ThumbMovwt (
  UINT8    *At,
  UINT32   Rd,
  UINT16   Imm,
  BOOLEAN  Top
  )
{
  UINT16  Hw1 = (UINT16)((Top ? 0xF2C0 : 0xF240) | (((Imm >> 11) & 1) << 10) | ((Imm >> 12) & 0xF));
  UINT16  Hw2 = (UINT16)((((Imm >> 8) & 7) << 12) | (Rd << 8) | (Imm & 0xFF));

  At[0] = (UINT8)Hw1; At[1] = (UINT8)(Hw1 >> 8);
  At[2] = (UINT8)Hw2; At[3] = (UINT8)(Hw2 >> 8);
}

/* Target of the Thumb-2 BL at At, or NULL if At does not hold a BL. */
STATIC
UINT8 *
ThumbBlTarget (
  UINT8  *At
  )
{
  UINT16  Hw1 = (UINT16)(At[0] | (At[1] << 8));
  UINT16  Hw2 = (UINT16)(At[2] | (At[3] << 8));
  UINT32  S, J1, J2, I1, I2, Imm;
  INT32   Off;

  if ((Hw1 & 0xF800) != 0xF000 || (Hw2 & 0xD000) != 0xD000) {
    return NULL;
  }
  S  = (Hw1 >> 10) & 1;
  J1 = (Hw2 >> 13) & 1;
  J2 = (Hw2 >> 11) & 1;
  I1 = (~(J1 ^ S)) & 1;
  I2 = (~(J2 ^ S)) & 1;
  Imm = (S << 24) | (I1 << 23) | (I2 << 22) | ((UINT32)(Hw1 & 0x3FF) << 12) | ((UINT32)(Hw2 & 0x7FF) << 1);
  Off = (INT32)(Imm << 7) >> 7;
  return At + 4 + Off;
}

/* movs r0,#Id ; movw/movt r1,#delta ; add r1,pc  (r1 = borrow VA) at At, 12 bytes */
STATIC
VOID
BeaconArgs (
  UINT8  *At,
  UINT8  *KernelBase,
  UINT32 Id
  )
{
  UINT32  Delta = (UINT32)((INTN)(KernelBase + BEACON_BORROW_RVA) - ((INTN)At + 10 + 4));

  ((UINT16 *)At)[0] = (UINT16)(0x2000 | (Id & 0xFF));
  ThumbMovwt (At + 2, 1, (UINT16)Delta, FALSE);
  ThumbMovwt (At + 6, 1, (UINT16)(Delta >> 16), TRUE);
  ((UINT16 *)At)[5] = 0x4479;                       /* add r1, pc */
}

STATIC
VOID
InstallBeacon (
  UINT8  *KernelBase
  )
{
 #if BEACON_ENABLE
  UINT8   *Cave  = KernelBase + BEACON_CAVE_RVA;
  UINT8   *Mark  = Cave;
  UINT8   *Tramp = Cave + ((sizeof (mBeaconMark) + 3) & ~3U);
  UINT8   *Stub  = Tramp + 84;
  UINT8   *Hook  = KernelBase + BEACON_HOOK_RVA;
  UINTN   Hooked = 0;
  UINTN   Index;

  CopyMem (Mark, mBeaconMark, sizeof (mBeaconMark));

  /*
    bugcheck trampoline (B.W from KeBugCheck2's first instruction; never
    returns, so r4-r7 are free):
      +0  mov r4..r7, r0..r3            code, P1, P2, P3
      +8  4 x { r0 = 0x80 | (19 + n), r1 = borrow VA ; mov r2, r4+n ;
                bl BeaconMark }          18 bytes each
      +80 b .                            (BEACON_STALL)
    All offsets are image-relative, so the same bytes are right at the
    physical load address we patch and at the kernel VA it runs from.
  */
  ((UINT16 *)Tramp)[0] = 0x4604; ((UINT16 *)Tramp)[1] = 0x460D;    /* mov r4,r0 ; mov r5,r1 */
  ((UINT16 *)Tramp)[2] = 0x4616; ((UINT16 *)Tramp)[3] = 0x461F;    /* mov r6,r2 ; mov r7,r3 */
  for (Index = 0; Index < 4; Index++) {
    UINT8  *At = Tramp + 8 + 18 * Index;

    BeaconArgs (At, KernelBase, 0x80 | (BEACON_BUGCHECK_ID + (UINT32)Index));
    ((UINT16 *)(At + 12))[0] = (UINT16)(0x4622 | (Index << 3));   /* mov r2, r4+Index */
    ThumbBranch (At + 14, Mark, TRUE);
  }
 #if BEACON_STALL
  ((UINT16 *)Tramp)[40] = 0xE7FE;                    /* b . */
 #endif
  ThumbBranch (Hook, Tramp, FALSE);
  WriteBackInvalidateDataCacheRange (Hook, 4);
  InvalidateInstructionCacheRange (Hook, 4);

  /*
    milestone stub (44 bytes), the hooked BL now calls it:
      +0  push.w {r0-r3, r12, lr}
      +4  r0 = id,        r1 = borrow VA ; +16 bl BeaconMark  (left 32 px)
      +20 r0 = id | 0x40, r1 = borrow VA ; +32 bl BeaconMark  (right 32 px:
          only there if the first call came back)
      +36 pop.w  {r0-r3, r12, lr}
      +40 b.w  original target       (returns straight to the hooked caller)
  */
  for (Index = 0; BEACON_MILESTONES && (Index < ARRAY_SIZE (mMilestones)); Index++, Stub += 44) {
    UINT8  *Site   = KernelBase + mMilestones[Index].Site;
    UINT8  *Target = KernelBase + mMilestones[Index].Target;

    if (ThumbBlTarget (Site) != Target) {
      DEBUG ((DEBUG_ERROR, "KernelErrataPatcher: milestone %u: no BL to %x at %x, skipped\n",
              Index, mMilestones[Index].Target, mMilestones[Index].Site));
      continue;
    }
    ((UINT16 *)Stub)[0] = 0xE92D;  ((UINT16 *)Stub)[1] = 0x500F;   /* push.w {r0-r3,r12,lr} */
    BeaconArgs (Stub + 4, KernelBase, (UINT32)Index);
    ThumbBranch (Stub + 16, Mark, TRUE);
    BeaconArgs (Stub + 20, KernelBase, (UINT32)Index | 0x40);
    ThumbBranch (Stub + 32, Mark, TRUE);
    ((UINT16 *)Stub)[18] = 0xE8BD; ((UINT16 *)Stub)[19] = 0x500F;  /* pop.w {r0-r3,r12,lr} */
    ThumbBranch (Stub + 40, Target, FALSE);
    ThumbBranch (Site, Stub, TRUE);
    WriteBackInvalidateDataCacheRange (Site, 4);
    InvalidateInstructionCacheRange (Site, 4);
    Hooked++;
  }

  /*
    payload mapper, called first at milestone 0 (KiSystemStartup ->
    KiInitializeBootStructures), always installed: it carries the NMRR
    write-through fix.  38 bytes:
      +0  push.w {r0-r3, r12, lr}
      +4  movw/movt r0, delta ; +12 add r0, pc      r0 = payload kernel VA
      +14 movw/movt r1, KEP_PAYLOAD_PHYS ; +22 movs r2, #pages
      +24 movs r3, #KEP_MAP_FLAGS
      +26 bl BeaconMapPayload
      +30 pop.w {r0-r3, r12, lr}
      +34 b.w milestone 0 stub (BEACON_MILESTONES) / KiInitializeBootStructures
  */
  {
    UINT16  *H     = (UINT16 *)Stub;
    UINT32  Delta  = (UINT32)((INTN)(KernelBase + KEP_PAYLOAD_RVA) - ((INTN)Stub + 12 + 4));
    UINT8   *Site0 = KernelBase + mMilestones[0].Site;
    UINT8   *Next  = BEACON_MILESTONES ? Tramp + 84 : KernelBase + mMilestones[0].Target;

    if (ThumbBlTarget (Site0) == Next) {
      H[0] = 0xE92D; H[1] = 0x500F;
      ThumbMovwt (Stub + 4, 0, (UINT16)Delta, FALSE);
      ThumbMovwt (Stub + 8, 0, (UINT16)(Delta >> 16), TRUE);
      H[6] = 0x4478;                                        /* add r0, pc */
      ThumbMovwt (Stub + 14, 1, (UINT16)KEP_PAYLOAD_PHYS, FALSE);
      ThumbMovwt (Stub + 18, 1, (UINT16)(KEP_PAYLOAD_PHYS >> 16), TRUE);
      H[11] = (UINT16)(0x2200 | KEP_PAYLOAD_PAGES);         /* movs r2, #pages */
      H[12] = (UINT16)(0x2300 | KEP_MAP_FLAGS);             /* movs r3, #flags */
      ThumbBranch (Stub + 26, Mark + BEACON_MAP_OFFSET, TRUE);
      H[15] = 0xE8BD; H[16] = 0x500F;
      ThumbBranch (Stub + 34, Next, FALSE);
      ThumbBranch (Site0, Stub, TRUE);
      WriteBackInvalidateDataCacheRange (Site0, 4);
      InvalidateInstructionCacheRange (Site0, 4);
      Stub += 40;
      mPayloadMapped = TRUE;
    }
  }
  DEBUG ((DEBUG_ERROR, "KernelErrataPatcher: payload: %a at phys %x -> nt+%x, %u pages, flags %x\n",
          mPayloadMapped ? "mapper armed" : "NO MAPPER (milestone 0 missing) - NO NMRR FIX",
          KEP_PAYLOAD_PHYS, KEP_PAYLOAD_RVA, KEP_PAYLOAD_PAGES, KEP_MAP_FLAGS));

  WriteBackInvalidateDataCacheRange (Cave, (UINTN)(Stub - Cave));
  InvalidateInstructionCacheRange (Cave, (UINTN)(Stub - Cave));
  mBeaconKernel   = KernelBase;
  mBeaconMarkPhys = Mark;
  mBeaconNextStub = Stub;

  DEBUG ((DEBUG_ERROR, "KernelErrataPatcher: beacon v2: %u/%u milestones, bugcheck hook %x, cave %p (%u bytes)\n",
          Hooked, ARRAY_SIZE (mMilestones), BEACON_HOOK_RVA, Cave, (UINT32)(Stub - Cave)));
 #endif
}

/* ------------------------------------------------------------------ */
/*  HAL beacon: which HAL init subsystem callback never returns.         */
/*                                                                       */
/*  HalInitializeProcessor / HalInitSystem run HAL init stages through   */
/*  HalpInitSystemHelper, which calls the 19 subsystem callbacks of a    */
/*  table in hal.dll's .data (RVA 0x1E6D8, 8 bytes per entry, first word */
/*  the callback).  Each entry is redirected to a stub in ntoskrnl's     */
/*  cave that paints row 8 = stage << 8 | index on entry, calls the       */
/*  callback with r0-r3 unchanged, paints row 9 = the same on return and */
/*  returns the callback's result.  Rows 8 != 9 on screen = the callback */
/*  in row 8 never came back.                                            */
/*                                                                       */
/*  Runtime VAs come from pointers winload relocated before              */
/*  ExitBootServices: the callback table itself, and an ntoskrnl pointer */
/*  to its own .data (RVA 0x23E270, link-time value 0x005DF794).         */
/* ------------------------------------------------------------------ */

#define HAL_TIMESTAMP      0x538BAE0F
#define HAL_SIZE_OF_IMAGE  0x0003B000
#define HAL_TABLE_RVA      0x0001E6D8
#define HAL_TABLE_ENTRIES  19
#define HAL_LINK_BASE      0x80010000
#define NT_LINK_BASE       0x00400000
#define NT_SELFPTR_RVA     0x0023E270
#define NT_SELFPTR_LINK    0x005DF794
#define HAL_ROW_ENTER      8
#define HAL_ROW_RETURN     9
#define HAL_ROW_SUB_ENTER  10
#define HAL_TRACE          0    /* v17: OFF - test the TTBR0 S-bit fix without the stubs' cache side effects */
#define HAL_ROW_SUB_RETURN 11


STATIC
UINT8 *
FindHal (
  UINT8  *Start,
  UINTN  Size
  )
{
  UINTN  o;

  for (o = 0; o + 0x1000 <= Size; o += 0x1000) {
    UINT8   *M = Start + o;
    UINT32  Pe;

    if ((M[0] != 'M') || (M[1] != 'Z')) {
      continue;
    }
    Pe = ReadUnaligned32 ((UINT32 *)(M + 0x3C));
    if ((Pe > 0x800) || (ReadUnaligned32 ((UINT32 *)(M + Pe)) != 0x00004550)) {
      continue;
    }
    if ((ReadUnaligned32 ((UINT32 *)(M + Pe + 8)) == HAL_TIMESTAMP) &&
        (ReadUnaligned32 ((UINT32 *)(M + Pe + 24 + 56)) == HAL_SIZE_OF_IMAGE)) {
      /*
        Booting from disk, winload also keeps hal.dll's raw FILE in memory
        (sections not at their RVAs, callback table 0).  Only the mapped,
        relocated image has a Thumb pointer there with a page-aligned delta.
      */
      UINT32  T0 = ReadUnaligned32 ((UINT32 *)(M + HAL_TABLE_RVA));

      if ((o + HAL_TABLE_RVA + 8 * HAL_TABLE_ENTRIES <= Size) && ((T0 & 1) != 0) &&
          (((T0 - 0x8003B229) & 0xFFF) == 0)) {
        return M;
      }
      DEBUG ((DEBUG_ERROR, "KernelErrataPatcher: HAL beacon: hal.dll copy at %p is not the mapped image, skipped\n", M));
    }
  }
  return NULL;
}

/* Thumb-2 BL at At (runtime VA AtVa) to runtime VA TargetVa. */
STATIC
VOID
ThumbBlVa (
  UINT8   *At,
  UINT32  AtVa,
  UINT32  TargetVa
  )
{
  ThumbBranch (At, At + (INT32)((TargetVa & ~1u) - AtVa), TRUE);
}

/*
  BLs inside hal.dll on the hanging path, bracketed like the callbacks: a
  level per nesting depth, rows 10+2*(level-1) (entered) / +1 (returned),
  value level << 12 | n.  Real HD2 so far: HalpDbgInitSystem stage 1 ->
  HalpAcpiGetTable (LoaderBlock, 'DBGP') -> HalpAcpiTableCacheInit ->
  HalpAcpiFindRsdp never returns (v8).
*/
typedef struct {
  UINT8   Level;     /* 2..7 (7 -> rows 10/11)          */
  UINT8   Id;        /* n                               */
  UINT32  Site;      /* hal RVA of a BL                 */
  UINT32  Target;    /* hal RVA it must call (checked) */
  UINT8   StackArgs; /* callee takes stack arguments: forward 6 words */
} HAL_BL_HOOK;

STATIC CONST HAL_BL_HOOK  mHalBlHooks[] = {
  /* level 1: HalpDbgInitSystem stage 1 */
  /* (level 1 = HalpDbgInitSystem's calls: known since v7, 0x1003 / 0x1002;
     dropped in v15, rows 10/11 now show level 7) */
  /* level 2: HalpAcpiGetTable work routine */
  { 2, 1, 0x0000B596, 0x0002EC4C },   /* HalpAcpiTableCacheInit         */
  { 2, 2, 0x0000B57E, 0x0000B5B0 },   /* HalpAcpiGetTableWork           */
  /* level 3: HalpAcpiTableCacheInit */
  { 3, 1, 0x0002EC86, 0x0002F70C },   /* HalpAcpiFindRsdp               */
  { 3, 2, 0x0002ECAC, 0x0000B664 },   /* HalpAcpiGetCachedTable (XSDT)  */
  { 3, 3, 0x0002ECBC, 0x0000B664 },   /* HalpAcpiGetCachedTable (RSDT)  */
  { 3, 4, 0x0002ECEC, 0x0000B938 },   /* HalpMapPhysicalMemory64 (head) */
  { 3, 5, 0x0002ED0E, 0x0000BA60 },   /* HalpUnmapVirtualAddress        */
  { 3, 6, 0x0002ED22, 0x0000B938 },   /* HalpMapPhysicalMemory64 (all)  */
  { 3, 7, 0x0002ED36, 0x0000AFFC },   /* HalpAcpiCopyBiosTable          */
  { 3, 8, 0x0002ED54, 0x0000BA60 },   /* HalpUnmapVirtualAddress        */
  { 3, 9, 0x0002ED5C, 0x0000AF84 },   /* HalpAcpiCacheTable             */
  /* level 4: HalpAcpiFindRsdp (v8: entered, never returned).  Its ARC
     configuration tree walk (KeFind[Next]ConfigurationEntry through the
     IAT) brackets strcmp; the found branch allocates and copies */
  { 4, 1, 0x0002F742, 0x0000E434 },   /* strcmp (Identifier, "ACPI BIOS") */
  { 4, 2, 0x0002F776, 0x0000B0AC },   /* HalpAcpiAllocateMemory         */
  { 4, 3, 0x0002F78A, 0x00001E80 },   /* memmove (RSDP data)            */
  /* level 5: HalpAcpiCopyBiosTable (v9/v11: entered, never returned) */
  { 5, 1, 0x0000B00E, 0x0000B0AC },   /* HalpAcpiAllocateMemory (length + 16) */
  { 5, 2, 0x0000B020, 0x00002700 },   /* memset                         */
  { 5, 3, 0x0000B034, 0x00001E80 },   /* memmove (the table)            */
  { 5, 4, 0x0000B03C, 0x0000B050 },   /* HalpAcpiValidateAcpiTable      */
  /* level 4 (cont.): HalpAcpiGetTableWork ('DBGP'; v12: entered, never
     returned, after TableCacheInit completed) */
  { 4, 4, 0x0000B5CC, 0x0000B664 },   /* HalpAcpiGetCachedTable (list walk) */
  { 4, 5, 0x0000B60C, 0x0000B6A4, 1 }, /* HalpAcpiGetTableFromBios (2 stack args) */
  { 4, 6, 0x0000B61E, 0x0000AFFC },   /* HalpAcpiCopyBiosTable          */
  { 4, 7, 0x0000B63A, 0x0000BA60 },   /* HalpUnmapVirtualAddress        */
  { 4, 8, 0x0000B644, 0x0000AF84 },   /* HalpAcpiCacheTable             */
  /* level 5 (cont.): HalpAcpiGetTableFromBios */
  { 5, 5, 0x0000B6E0, 0x0000B3F0 },   /* HalpAcpiGetRsdt                */
  { 5, 6, 0x0000B738, 0x0000B7DC, 1 }, /* HalpAcpiCheckAndMapTable (XSDT loop, 6 stack args) */
  { 5, 7, 0x0000B7B0, 0x0000B7DC, 1 }, /* HalpAcpiCheckAndMapTable (FADT path) */
  /* level 6 (rows 20/21): HalpAcpiGetRsdt (v13: entered, never returned) */
  { 6, 1, 0x0000B406, 0x0000B664 },   /* HalpAcpiGetCachedTable ('RSDT') */
  { 6, 2, 0x0000B418, 0x0000B664 },   /* HalpAcpiGetCachedTable ('XSDT') */
  { 6, 3, 0x0000B44A, 0x0000B938 },   /* HalpMapPhysicalMemory64 (firmware XSDT) */
  { 6, 4, 0x0000B45A, 0x0000276C },   /* memcmp (cached vs firmware header) */
  { 6, 5, 0x0000B472, 0x0000BA60 },   /* HalpUnmapVirtualAddress        */
  { 6, 6, 0x00011A92, 0x0000B7DC, 1 }, /* HalpAcpiCheckAndMapTable (header mismatch) */
  { 6, 7, 0x0000B48C, 0x0000BAB8, 1 }, /* HalGetMemoryCachingRequirements (no LoaderBlock) */
  /* level 7 (rows 10/11): HalpAcpiCheckAndMapTable (v14: entered from the
     XSDT loop, never returned).  Not hookable: two indirect calls through
     hal .data pointers and the checksum loop between 7.4 and 7.5 */
  { 7, 1, 0x0000B816, 0x0000B938 },   /* HalpMapPhysicalMemory64 (header) */
  { 7, 2, 0x0000B846, 0x0000BA60 },   /* HalpUnmapVirtualAddress (header) */
  { 7, 3, 0x0000B860, 0x0000BAB8, 1 }, /* HalGetMemoryCachingRequirements */
  { 7, 4, 0x0000B8B2, 0x0000BAB8, 1 }, /* HalGetMemoryCachingRequirements */
  { 7, 5, 0x0000B8FE, 0x0000BA60 },   /* HalpUnmapVirtualAddress        */
  { 7, 6, 0x0000B914, 0x0000B938 },   /* HalpMapPhysicalMemory64 (whole table) */
};

/* Row 20: the size CopyBiosTable asks HalpAcpiAllocateMemory for (r1 =
   table header Length + 16): shows whether the HAL read a sane length. */
#define HAL_ROW_ALLOC_SIZE 20
#define HAL_ALLOC_ROW      0    /* v14: row 20 = level 6 (alloc size was 0x5C, sane) */
/* v15: paint a register at one hook's entry: r2 (table phys, low) of
   HalpAcpiCheckAndMapTable from the XSDT loop -> row 22 (= bugcheck P3 row;
   a bugcheck also repaints rows 19-21) */
#define HAL_ARG_LEVEL      5
#define HAL_ARG_ID         6
#define HAL_ARG_REG        2
#define HAL_ARG_ROW        22

/* movs r0,#Id ; movw/movt r1,#borrow VA ; nop  (12 bytes, position-free:
   for stubs in the payload, which is not inside the kernel image) */
STATIC
VOID
StubArgs (
  UINT8   *At,
  UINT32  Id
  )
{
  ((UINT16 *)At)[0] = (UINT16)(0x2000 | (Id & 0xFF));
  ThumbMovwt (At + 2, 1, (UINT16)mStubBorrowVa, FALSE);
  ThumbMovwt (At + 6, 1, (UINT16)(mStubBorrowVa >> 16), TRUE);
  ((UINT16 *)At)[5] = 0xBF00;
}

/*
  As HalBracketStub, for a callee with stack arguments (128 bytes): the
  plain stub's 32-byte push would hand the callee its saved registers as
  arguments 5+ (v13 in the emulator: HalpAcpiGetTableFromBios stored its
  result through the saved r0 = LoaderBlock -> bugcheck 0x100).  Copies the
  caller's 6 stack words below the push before the call.
*/
STATIC
UINT8 *
HalBracketStubStack (
  UINT8   *Stub,
  UINT32  Hi,
  UINT32  Lo,
  UINT32  RowEnter,
  UINT32  RowReturn,
  UINT32  Target
  )
{
  UINT16  *H = (UINT16 *)Stub;
  UINTN   i;

  H[0] = 0xE92D; H[1] = 0x407F;                            /* push.w {r0-r6, lr}  */
  H[2] = (UINT16)(0x2400 | Hi); H[3] = (UINT16)(0x2500 | Lo); H[4] = 0x0222; H[5] = 0x432A;
  StubArgs (Stub + 12, 0x80 | RowEnter);
  ThumbBranch (Stub + 24, mStubMark, TRUE);
  H[14] = 0xB086;                                          /* sub sp, #24         */
  for (i = 0; i < 6; i++) {                                /* +30: 6 x 8 bytes    */
    H[15 + 4 * i] = 0xF8DD; H[16 + 4 * i] = (UINT16)(0xC000 | (56 + 4 * i));   /* ldr.w r12, [sp, #56+4i] */
    H[17 + 4 * i] = 0xF8CD; H[18 + 4 * i] = (UINT16)(0xC000 | (4 * i));        /* str.w r12, [sp, #4i]    */
  }
  H[39] = 0xF10D; H[40] = 0x0C18;                          /* +78 add.w r12, sp, #24 */
  H[41] = 0xE89C; H[42] = 0x000F;                          /* +82 ldm.w r12, {r0-r3} */
  ThumbMovwt (Stub + 86, 12, (UINT16)Target, FALSE);
  ThumbMovwt (Stub + 90, 12, (UINT16)(Target >> 16), TRUE);
  H[47] = 0x47E0;                                          /* +94 blx r12          */
  H[48] = 0xB006;                                          /* +96 add sp, #24      */
  H[49] = 0x4606; H[50] = 0x0222; H[51] = 0x432A;          /* +98 mov r6, r0 ; value */
  StubArgs (Stub + 104, 0x80 | RowReturn);
  ThumbBranch (Stub + 116, mStubMark, TRUE);
  H[60] = 0x4630; H[61] = 0xB004; H[62] = 0xBD70; H[63] = 0xBF00;
  return Stub + 128;
}

STATIC
UINT8 *
HalBracketStub (
  UINT8   *Stub,
  UINT32  Hi,
  UINT32  Lo,
  UINT32  RowEnter,
  UINT32  RowReturn,
  UINT32  Target
  )
{
  UINT16  *H = (UINT16 *)Stub;

  /* as the callback stub, with r4 = Hi, r5 = Lo: value = Hi << 8 | Lo */
  H[0] = 0xE92D; H[1] = 0x407F;
  H[2] = (UINT16)(0x2400 | Hi); H[3] = (UINT16)(0x2500 | Lo); H[4] = 0x0222; H[5] = 0x432A;
  StubArgs (Stub + 12, 0x80 | RowEnter);
  ThumbBranch (Stub + 24, mStubMark, TRUE);
  H[14] = 0xE89D; H[15] = 0x000F;
  ThumbMovwt (Stub + 32, 12, (UINT16)Target, FALSE);
  ThumbMovwt (Stub + 36, 12, (UINT16)(Target >> 16), TRUE);
  H[20] = 0x47E0;
  H[21] = 0x4606; H[22] = 0x0222; H[23] = 0x432A;
  StubArgs (Stub + 48, 0x80 | RowReturn);
  ThumbBranch (Stub + 60, mStubMark, TRUE);
  H[32] = 0x4630; H[33] = 0xB004; H[34] = 0xBD70; H[35] = 0xBF00;
  return Stub + 72;
}

/*
  HAL TLB fix (real Scorpion, beacons v8/v9): HalpUnmapVirtualAddress clears
  the PTEs and then does TLBIALL; BPIALL; DSB; ISB - no DSB between the PTE
  stores and the TLB invalidate, which ARMv7 requires.  The Scorpion can walk
  the old PTE again right after the invalidate and keep a stale translation;
  HalpMap writes new PTEs without any invalidate, so the next mapping at the
  same VA (map header / unmap / map whole table in the ACPI code) reads the
  old physical page and the HAL hangs.  QEMU's TLB is exact, so the emulator
  never showed it; every beacon stub's TLBIALL hid it.
  Both sequences are redirected to a stub: DSB; TLBIALL; BPIALL; DSB; ISB.
*/
typedef struct {
  UINT32  Rva;        /* hal RVA of the sequence           */
  UINT32  Back;       /* hal RVA to continue at            */
  UINT8   Bytes[16];  /* original bytes (checked)          */
  UINT32  Length;
} HAL_TLB_FIX;

STATIC CONST HAL_TLB_FIX  mHalTlbFix[] = {
  /* HalpUnmapVirtualAddress: mcr TLBIALL(r3) ; mcr BPIALL(r3) ; dsb ; isb */
  { 0x0000BA94, 0x0000BAA4, { 0x08, 0xEE, 0x17, 0x3F, 0x07, 0xEE, 0xD5, 0x3F,
                              0xBF, 0xF3, 0x4F, 0x8F, 0xBF, 0xF3, 0x6F, 0x8F }, 16 },
  /* HalpMap after the PTE store: dsb ; isb */
  { 0x0000B9F2, 0x0000B9FA, { 0xBF, 0xF3, 0x4F, 0x8F, 0xBF, 0xF3, 0x6F, 0x8F }, 8 },
};

STATIC
VOID
ThumbBranchVa (
  UINT8   *At,
  UINT32  AtVa,
  UINT32  TargetVa
  )
{
  ThumbBranch (At, At + (INT32)((TargetVa & ~1u) - AtVa), FALSE);
}

STATIC
UINT8 *
InstallHalTlbFix (
  UINT8   *Hal,
  UINT32  HalRuntime,
  UINT32  NtRuntime,
  UINT8   *Stub
  )
{
  /*
    v18: clean the written PTEs to memory before the TLB invalidate.  The
    real Scorpion walks Windows' page tables (TTBR0 attrs 0x09) without
    seeing L1; the KEP's MMFR3 patch tells Windows the walk is coherent, so
    nothing else cleans them (beacons v6..v16: every hang was a fresh HAL
    mapping; each paint evicted L1 and "healed" the next call).
  */
  STATIC CONST UINT16  SeqUnmap[] = {    /* r2 = end of the zeroed PTE run, r1 = count */
    0xB159,                     /* cbz r1, done                 */
    0xEBA2, 0x0381,             /* sub.w r3, r2, r1, lsl #2     */
    0xF023, 0x031F,             /* bic r3, r3, #31              */
    0xF3BF, 0x8F4F,             /* dsb sy                       */
    0x4293, 0xD203,             /* 1: cmp r3, r2 ; bhs done     */
    0xEE07, 0x3F3A,             /* mcr DCCMVAC r3               */
    0x3320, 0xE7F9,             /* adds r3, #32 ; b 1b          */
    0xF3BF, 0x8F4F,             /* done: dsb sy                 */
    0xEE08, 0x3F17,             /* TLBIALL                      */
    0xEE07, 0x3FD5,             /* BPIALL                       */
    0xF3BF, 0x8F4F,             /* dsb sy                       */
    0xF3BF, 0x8F6F,             /* isb sy                       */
  };
  STATIC CONST UINT16  SeqMap[] = {      /* PTE was stored with str.w r2, [r3, r12] */
    0x4463,                     /* add r3, r12                  */
    0xF3BF, 0x8F4F,             /* dsb sy                       */
    0xEE07, 0x3F3A,             /* mcr DCCMVAC r3               */
    0xF3BF, 0x8F4F,             /* dsb sy                       */
    0xEE08, 0x3F17,             /* TLBIALL                      */
    0xEE07, 0x3FD5,             /* BPIALL                       */
    0xF3BF, 0x8F4F,             /* dsb sy                       */
    0xF3BF, 0x8F6F,             /* isb sy                       */
  };
  UINTN  Index, Done = 0;

  for (Index = 0; Index < ARRAY_SIZE (mHalTlbFix); Index++) {
    UINT8   *Site  = Hal + mHalTlbFix[Index].Rva;
    UINT32  StubVa = NtRuntime + (UINT32)(Stub - mBeaconKernel);

    if (CompareMem (Site, mHalTlbFix[Index].Bytes, mHalTlbFix[Index].Length) != 0) {
      DEBUG ((DEBUG_ERROR, "KernelErrataPatcher: HAL TLB fix: hal+%x does not match, skipped\n", mHalTlbFix[Index].Rva));
      continue;
    }
    CONST UINT16  *Seq    = (Index == 0) ? SeqUnmap : SeqMap;
    UINTN         SeqSize = (Index == 0) ? sizeof (SeqUnmap) : sizeof (SeqMap);

    CopyMem (Stub, Seq, SeqSize);
    ThumbBranchVa (Stub + SeqSize, StubVa + SeqSize, HalRuntime + mHalTlbFix[Index].Back);
    ThumbBranchVa (Site, HalRuntime + mHalTlbFix[Index].Rva, StubVa);
    WriteBackInvalidateDataCacheRange (Site, 4);
    InvalidateInstructionCacheRange (Site, 4);
    WriteBackInvalidateDataCacheRange (Stub, SeqSize + 4);
    InvalidateInstructionCacheRange (Stub, SeqSize + 4);
    Stub += (SeqSize + 4 + 7) & ~7u;
    Done++;
  }
  DEBUG ((DEBUG_ERROR, "KernelErrataPatcher: HAL TLB fix + PTE clean: %u of %u sequences\n", Done, ARRAY_SIZE (mHalTlbFix)));
  return Stub;
}

STATIC
VOID
InstallHalBeacon (
  UINT8  *Hal
  )
{
  UINT32  NtDelta, HalDelta, NtRuntime, PayloadVa, *Table;
  UINTN   Index;
  UINT8   *Stub, *CaveEnd;
  UINT8   *Payload = (UINT8 *)(UINTN)KEP_PAYLOAD_PHYS;

  if ((mBeaconKernel == NULL) || (mBeaconNextStub == NULL)) {
    DEBUG ((DEBUG_ERROR, "KernelErrataPatcher: HAL beacon: no ntoskrnl beacon\n"));
    return;
  }
  NtDelta   = ReadUnaligned32 ((UINT32 *)(mBeaconKernel + NT_SELFPTR_RVA)) - NT_SELFPTR_LINK;
  NtRuntime = NT_LINK_BASE + NtDelta;
  PayloadVa = NtRuntime + KEP_PAYLOAD_RVA;
  Table     = (UINT32 *)(Hal + HAL_TABLE_RVA);
  HalDelta  = Table[0] - 0x8003B229;                       /* entry 0's link-time value */
  if (((NtDelta & 0xFFF) != 0) || ((HalDelta & 0xFFF) != 0)) {
    DEBUG ((DEBUG_ERROR, "KernelErrataPatcher: HAL beacon: odd deltas nt %x hal %x, skipped\n", NtDelta, HalDelta));
    return;
  }

  /*
    stub (72 bytes) per callback n, original target T:
      +0  push.w {r0-r6, lr}
      +4  mov r4, r0 ; movs r5, #n ; lsls r2, r4, #8 ; orrs r2, r5
      +12 r0 = 0x80 | 8, r1 = borrow VA ; +24 bl BeaconMark
      +28 ldm.w sp, {r0-r3}
      +32 movw/movt r12, T ; +40 blx r12
      +42 mov r6, r0 ; lsls r2, r4, #8 ; orrs r2, r5
      +48 r0 = 0x80 | 9, r1 = borrow VA ; +60 bl BeaconMark
      +64 mov r0, r6 ; add sp, #16 ; pop {r4-r6, pc}
  */
  Stub = (UINT8 *)(((UINTN)mBeaconNextStub + 7) & ~(UINTN)7);
  Stub = InstallHalTlbFix (Hal, HAL_LINK_BASE + HalDelta, NtRuntime, Stub);
  CaveEnd = Stub;
  WriteBackInvalidateDataCacheRange (mBeaconNextStub, (UINTN)(CaveEnd - mBeaconNextStub));
  InvalidateInstructionCacheRange (mBeaconNextStub, (UINTN)(CaveEnd - mBeaconNextStub));
  if (CaveEnd > mBeaconKernel + BEACON_CAVE_END_RVA) {
    DEBUG ((DEBUG_ERROR, "KernelErrataPatcher: CAVE OVERFLOW: ends at +%x, limit +%x\n",
            (UINT32)(CaveEnd - mBeaconKernel), BEACON_CAVE_END_RVA));
  }
  if (!mPayloadMapped) {
    DEBUG ((DEBUG_ERROR, "KernelErrataPatcher: HAL beacon: payload not mapped, trace skipped\n"));
    return;
  }

  /* the trace stubs go to the payload: written at its physical address, run
     at PayloadVa; every reference in them is absolute or payload-relative */
  ZeroMem (Payload, KEP_PAYLOAD_PAGES * EFI_PAGE_SIZE);
  CopyMem (Payload, mBeaconMark, sizeof (mBeaconMark));
  mStubMark     = Payload;
  mStubBorrowVa = NtRuntime + BEACON_BORROW_RVA;
  Stub = Payload + ((sizeof (mBeaconMark) + 7) & ~7u);
  for (Index = 0; HAL_TRACE && (Index < HAL_TABLE_ENTRIES); Index++) {
    UINT32  T = Table[2 * Index];
    UINT16  *H = (UINT16 *)Stub;

    if ((T & 1) == 0) {
      continue;    /* not a Thumb pointer (v16: all callbacks again; v15 got
                      HalpDbgInitSystem stage 1 through) */
    }
    H[0] = 0xE92D; H[1] = 0x407F;
    H[2] = 0x4604; H[3] = (UINT16)(0x2500 | Index); H[4] = 0x0222; H[5] = 0x432A;
    StubArgs (Stub + 12, 0x80 | HAL_ROW_ENTER);
    ThumbBranch (Stub + 24, mStubMark, TRUE);
    H[14] = 0xE89D; H[15] = 0x000F;
    ThumbMovwt (Stub + 32, 12, (UINT16)T, FALSE);
    ThumbMovwt (Stub + 36, 12, (UINT16)(T >> 16), TRUE);
    H[20] = 0x47E0;
    H[21] = 0x4606; H[22] = 0x0222; H[23] = 0x432A;
    StubArgs (Stub + 48, 0x80 | HAL_ROW_RETURN);
    ThumbBranch (Stub + 60, mStubMark, TRUE);
    H[32] = 0x4630; H[33] = 0xB004; H[34] = 0xBD70; H[35] = 0xBF00;
    Table[2 * Index] = PayloadVa + (UINT32)(Stub - Payload) + 1;
    Stub += 72;
  }
  for (Index = 0; HAL_TRACE && (Index < ARRAY_SIZE (mHalBlHooks)); Index++) {
    UINT8   *Site   = Hal + mHalBlHooks[Index].Site;
    UINT32  SiteVa  = HAL_LINK_BASE + HalDelta + mHalBlHooks[Index].Site;
    UINT32  StubVa  = PayloadVa + (UINT32)(Stub - Payload);

    if (ThumbBlTarget (Site) != Hal + mHalBlHooks[Index].Target) {
      DEBUG ((DEBUG_ERROR, "KernelErrataPatcher: HAL beacon: no BL at hal+%x, skipped\n", mHalBlHooks[Index].Site));
      continue;
    }
    if ((mHalBlHooks[Index].Level == HAL_ARG_LEVEL) && (mHalBlHooks[Index].Id == HAL_ARG_ID)) {
      /*
        v16: paint TTBR1 (row 22) and TTBR0 (row 21) - the walk attributes
        Windows really runs with on the Scorpion - then fall into the
        bracket stub (48 bytes, stack balanced):
          push.w {r0-r6, lr} ; mrc TTBR1 -> r2 ; row 22 ; bl Mark ;
          mrc TTBR0 -> r2 ; row 21 ; bl Mark ; pop.w {r0-r6, lr}
      */
      UINT16  *H = (UINT16 *)Stub;

      H[0] = 0xE92D; H[1] = 0x407F;
      H[2] = 0xEE12; H[3] = 0x2F30;                        /* mrc p15,0,r2,c2,c0,1 TTBR1 */
      StubArgs (Stub + 8, 0x80 | 22);
      ThumbBranch (Stub + 20, mStubMark, TRUE);
      H[12] = 0xEE12; H[13] = 0x2F10;                      /* mrc p15,0,r2,c2,c0,0 TTBR0 */
      StubArgs (Stub + 28, 0x80 | 21);
      ThumbBranch (Stub + 40, mStubMark, TRUE);
      H[22] = 0xE8BD; H[23] = 0x407F;                      /* pop.w {r0-r6, lr} */
      Stub += 48;                                          /* falls through into the bracket stub */
    }
    Stub = (mHalBlHooks[Index].StackArgs ? HalBracketStubStack : HalBracketStub) (
                           Stub, (UINT32)mHalBlHooks[Index].Level << 4, mHalBlHooks[Index].Id,
                           (mHalBlHooks[Index].Level == 7) ? HAL_ROW_SUB_ENTER : HAL_ROW_SUB_ENTER + 2 * (mHalBlHooks[Index].Level - 1u),
                           (mHalBlHooks[Index].Level == 7) ? HAL_ROW_SUB_RETURN : HAL_ROW_SUB_RETURN + 2 * (mHalBlHooks[Index].Level - 1u),
                           HAL_LINK_BASE + HalDelta + mHalBlHooks[Index].Target + 1);
    if ((INT32)(StubVa - SiteVa) >= 0x1000000 || (INT32)(StubVa - SiteVa) < -0x1000000) {
      DEBUG ((DEBUG_ERROR, "KernelErrataPatcher: HAL beacon: hal+%x -> %x out of BL range, skipped\n",
              mHalBlHooks[Index].Site, StubVa));
      continue;
    }
    ThumbBlVa (Site, SiteVa, StubVa);
    WriteBackInvalidateDataCacheRange (Site, 4);
    InvalidateInstructionCacheRange (Site, 4);
  }
  if (Stub > Payload + KEP_PAYLOAD_PAGES * EFI_PAGE_SIZE) {
    DEBUG ((DEBUG_ERROR, "KernelErrataPatcher: PAYLOAD OVERFLOW: %x bytes\n", (UINT32)(Stub - Payload)));
  }
  WriteBackInvalidateDataCacheRange (Payload, KEP_PAYLOAD_PAGES * EFI_PAGE_SIZE);
  InvalidateInstructionCacheRange (Payload, KEP_PAYLOAD_PAGES * EFI_PAGE_SIZE);
  WriteBackInvalidateDataCacheRange (Table, HAL_TABLE_ENTRIES * 8);
  DEBUG ((DEBUG_ERROR, "KernelErrataPatcher: HAL beacon: hal at %p, nt runtime %x, %u BL hooks, cave end +%x, payload %x bytes at %x\n",
          Hal, NtRuntime, ARRAY_SIZE (mHalBlHooks), (UINT32)(CaveEnd - (mBeaconKernel + BEACON_CAVE_RVA)),
          (UINT32)(Stub - Payload), PayloadVa));
}

/* ------------------------------------------------------------------ */
/*  ExitBootServices hook                                             */
/* ------------------------------------------------------------------ */

STATIC EFI_EXIT_BOOT_SERVICES  gOrigExitBootServices;
STATIC BOOLEAN                 gPatchApplied = FALSE;

/*
  Static so the hook never allocates (that would invalidate winload's MapKey).
  UINT64[] for alignment: as UINT8[] it landed on an odd address and
  CoreGetMemoryMap's strd into the first descriptor alignment-faulted.
*/
STATIC UINT64  gMemMap[0x4000 / sizeof (UINT64)];

STATIC
VOID
RestoreExitBootServices (
  VOID
  )
{
  UINT32  Crc = 0;

  gBS->ExitBootServices = gOrigExitBootServices;
  gBS->Hdr.CRC32        = 0;
  gBS->CalculateCrc32 (gBS, gBS->Hdr.HeaderSize, &Crc);
  gBS->Hdr.CRC32 = Crc;
}

STATIC
EFI_STATUS
EFIAPI
ExitBootServicesHook (
  IN EFI_HANDLE  ImageHandle,
  IN UINTN       MapKey
  )
{
  EFI_STATUS             Status;
  UINTN                  MemMapSize;
  EFI_MEMORY_DESCRIPTOR  *Desc;
  UINTN                  DescSize;
  UINT32                 DescVer;
  UINTN                  Key;
  UINTN                  Entries;
  UINTN                  e;
  UINTN                  TotalPatched = 0;

  if (!gPatchApplied) {
    gPatchApplied = TRUE;
    MemMapSize    = sizeof (gMemMap);
    Status        = gBS->GetMemoryMap (&MemMapSize, (EFI_MEMORY_DESCRIPTOR *)gMemMap, &Key, &DescSize, &DescVer);
    if (!EFI_ERROR (Status)) {
      Desc    = (EFI_MEMORY_DESCRIPTOR *)gMemMap;
      Entries = MemMapSize / DescSize;
      for (e = 0; e < Entries; e++) {
        if (((Desc->Type == EfiLoaderData) || (Desc->Type == EfiLoaderCode)) &&
            (Desc->NumberOfPages > 1))
        {
          TotalPatched += PatchRange (
                            (UINT8 *)(UINTN)Desc->PhysicalStart,
                            EFI_PAGES_TO_SIZE ((UINTN)Desc->NumberOfPages)
                            );
        }

        Desc = NEXT_MEMORY_DESCRIPTOR (Desc, DescSize);
      }
    } else {
      DEBUG ((DEBUG_ERROR, "KernelErrataPatcher: GetMemoryMap %r size %u\n", Status, MemMapSize));
    }

    DEBUG ((DEBUG_ERROR, "KernelErrataPatcher: %u patches applied\n", TotalPatched));
#if BEACON_ENABLE
    if (!EFI_ERROR (Status)) {
      UINT8  *Hal = NULL;

      Desc = (EFI_MEMORY_DESCRIPTOR *)gMemMap;
      for (e = 0; (e < Entries) && (Hal == NULL); e++) {
        if (((Desc->Type == EfiLoaderData) || (Desc->Type == EfiLoaderCode)) && (Desc->NumberOfPages > 1)) {
          Hal = FindHal ((UINT8 *)(UINTN)Desc->PhysicalStart, EFI_PAGES_TO_SIZE ((UINTN)Desc->NumberOfPages));
        }
        Desc = NEXT_MEMORY_DESCRIPTOR (Desc, DescSize);
      }
      if (Hal != NULL) {
        InstallHalBeacon (Hal);
      } else {
        DEBUG ((DEBUG_ERROR, "KernelErrataPatcher: HAL beacon: hal.dll not found\n"));
      }
    }
#endif
  }

  RestoreExitBootServices ();
  return gOrigExitBootServices (ImageHandle, MapKey);
}

/* ------------------------------------------------------------------ */
/*  Driver entry point                                                */
/* ------------------------------------------------------------------ */

EFI_STATUS
EFIAPI
KernelErrataPatcherDxeEntry (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  UINT32  Crc = 0;

  DEBUG ((DEBUG_ERROR, "KernelErrataPatcher: loaded, hooking ExitBootServices\n"));

  gOrigExitBootServices = gBS->ExitBootServices;
  gBS->ExitBootServices = ExitBootServicesHook;
  gBS->Hdr.CRC32        = 0;
  gBS->CalculateCrc32 (gBS, gBS->Hdr.HeaderSize, &Crc);
  gBS->Hdr.CRC32 = Crc;

  return EFI_SUCCESS;
}
