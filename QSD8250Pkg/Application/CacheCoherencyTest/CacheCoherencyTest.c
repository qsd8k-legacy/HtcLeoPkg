/** @file
  CacheCoherencyTest - probe on the bare CPU (no Windows) whether the
  Scorpion actually supports the memory-system features that Windows RT's
  KiSetFeatureBits refuses it for (bugcheck 0x5D, param1 = 2):

    ID_MMFR2[19:16] >= 3  unified TLB maintenance incl. TLBIMVAA (c8,c7,3)
    ID_MMFR3[15:12] >= 2  cache/TLB maintenance broadcast (the *IS ops)
    ID_MMFR3[23:20] >= 1  coherent table walk (PTE writes need no clean)

  If the hardware does it anyway, NOPping those checks in ntoskrnl is safe.
  If not, the kernel will run with stale translations and corrupt memory.

  Test groups
    1. ID registers + what Windows would decide from them
    2. UNDEF probes: does each instruction / CP15 op exist at all
    3. TLB ops, functional: map a private 1 MiB section at a free VA, let the
       TLB cache it, repoint it to another physical MiB (properly cleaned),
       issue ONE maintenance op, read back: new data = the op works
    4. Coherent walk: repoint the section with a plain store only (dirty line
       in L1 D, no clean), TLBIALL, read back: new data = the walker sees the
       D-cache. Repeated for several TTBR0 walk-cacheability attributes.
    5. I/D coherency (informational): self-modifying code with partial
       maintenance

  Results go to the screen, the debug serial and \cachetest.txt on the
  volume the app was started from. Boots as \EFI\BOOT\BOOTARM.EFI.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <Uefi.h>
#include <Library/ArmLib.h>
#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PrintLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Protocol/Cpu.h>
#include <Protocol/LoadedImage.h>
#include <Protocol/SimpleFileSystem.h>
#include <Guid/FileInfo.h>

#define SZ_1M  0x100000u

/* ------------------------------------------------------------------ log */

STATIC CHAR8  mLog[32768];
STATIC UINTN  mLogLen;

STATIC
VOID
EFIAPI
Log (
  IN CONST CHAR8  *Fmt,
  ...
  )
{
  VA_LIST  Va;
  CHAR8    Line[192];
  UINTN    Len;

  VA_START (Va, Fmt);
  Len = AsciiVSPrint (Line, sizeof (Line), Fmt, Va);
  VA_END (Va);

  AsciiPrint ("%a", Line);
  DEBUG ((DEBUG_ERROR, "%a", Line));
  if (mLogLen + Len < sizeof (mLog)) {
    CopyMem (mLog + mLogLen, Line, Len);
    mLogLen += Len;
  }
}

STATIC
VOID
SaveLog (
  IN EFI_HANDLE  ImageHandle
  )
{
  EFI_LOADED_IMAGE_PROTOCOL        *Li;
  EFI_SIMPLE_FILE_SYSTEM_PROTOCOL  *Fs;
  EFI_FILE_PROTOCOL                *Root, *File;
  EFI_STATUS                       Status;
  UINTN                            Size;

  Status = gBS->HandleProtocol (ImageHandle, &gEfiLoadedImageProtocolGuid, (VOID **)&Li);
  if (EFI_ERROR (Status)) {
    return;
  }

  Status = gBS->HandleProtocol (Li->DeviceHandle, &gEfiSimpleFileSystemProtocolGuid, (VOID **)&Fs);
  if (EFI_ERROR (Status)) {
    Log ("log: no filesystem on boot device (%r)\n", Status);
    return;
  }

  if (EFI_ERROR (Fs->OpenVolume (Fs, &Root))) {
    return;
  }

  /* truncate: delete an old copy first */
  if (!EFI_ERROR (Root->Open (Root, &File, L"\\cachetest.txt", EFI_FILE_MODE_READ | EFI_FILE_MODE_WRITE, 0))) {
    File->Delete (File);
  }

  Status = Root->Open (
                   Root,
                   &File,
                   L"\\cachetest.txt",
                   EFI_FILE_MODE_READ | EFI_FILE_MODE_WRITE | EFI_FILE_MODE_CREATE,
                   0
                   );
  if (!EFI_ERROR (Status)) {
    Size = mLogLen;
    Status = File->Write (File, &Size, mLog);
    File->Close (File);
    Print (L"log: \\cachetest.txt written (%r)\n", Status);
  } else {
    Print (L"log: cannot create \\cachetest.txt (%r)\n", Status);
  }

  Root->Close (Root);
}

/* ------------------------------------------------------- CP15 helpers */

#define MRC(op1, crn, crm, op2) ({ UINT32 _v; \
  __asm__ volatile ("mrc p15, " #op1 ", %0, " #crn ", " #crm ", " #op2 : "=r" (_v) :: "memory"); _v; })
#define MCR(op1, crn, crm, op2, v) \
  __asm__ volatile ("mcr p15, " #op1 ", %0, " #crn ", " #crm ", " #op2 :: "r" ((UINT32)(v)) : "memory")

#define DSB()  __asm__ volatile ("dsb" ::: "memory")
#define ISB()  __asm__ volatile ("isb" ::: "memory")

/* by-MVA cache ops on one line */
#define DCCMVAC(a)   MCR (0, c7, c10, 1, (a))  /* clean to PoC         */
#define DCCMVAU(a)   MCR (0, c7, c11, 1, (a))  /* clean to PoU         */
#define DCCIMVAC(a)  MCR (0, c7, c14, 1, (a))  /* clean+inval to PoC   */
#define DCIMVAC(a)   MCR (0, c7, c6, 1, (a))   /* invalidate to PoC    */
#define ICIMVAU(a)   MCR (0, c7, c5, 1, (a))   /* I-cache line to PoU  */
#define ICIALLU()    MCR (0, c7, c5, 0, 0)
#define BPIALL()     MCR (0, c7, c5, 6, 0)
#define TLBIALL()    MCR (0, c8, c7, 0, 0)

/* ------------------------------------------- UNDEF / abort trapping */

STATIC volatile UINT32  mProbing;
STATIC volatile UINT32  mTrapped;   /* exception type + 1, 0 = none */
STATIC volatile UINT32  mTrapFsr;
STATIC volatile UINT32  mTrapFar;

STATIC
UINT32
InsnSize (
  IN UINT32  Pc,
  IN UINT32  Cpsr
  )
{
  UINT16  Hw;

  if ((Cpsr & BIT5) == 0) {
    return 4;                                   /* ARM */
  }

  Hw = *(volatile UINT16 *)(UINTN)Pc;
  return ((Hw >> 11) >= 0x1D) ? 4 : 2;          /* Thumb-2 32-bit prefix */
}

STATIC
VOID
EFIAPI
TrapHandler (
  IN EFI_EXCEPTION_TYPE  Type,
  IN EFI_SYSTEM_CONTEXT  Ctx
  )
{
  EFI_SYSTEM_CONTEXT_ARM  *C = Ctx.SystemContextArm;

  if (!mProbing) {
    /* not ours: report and hang, like the default handler would */
    DEBUG ((DEBUG_ERROR, "CacheCoherencyTest: unexpected exception %d PC %08x\n", Type, C->PC));
    for ( ; ; ) {
    }
  }

  mTrapped = (UINT32)Type + 1;
  mTrapFsr = (Type == EXCEPT_ARM_DATA_ABORT) ? C->DFSR : C->IFSR;
  mTrapFar = (Type == EXCEPT_ARM_DATA_ABORT) ? C->DFAR : C->IFAR;
  if (Type == EXCEPT_ARM_PREFETCH_ABORT) {
    /* the I/D stub page is not executable: return to its caller */
    C->R0   = 0xDEAD;
    C->PC   = C->LR & ~1u;
    C->CPSR = (C->LR & 1) ? (C->CPSR | BIT5) : (C->CPSR & ~BIT5);
    return;
  }

  C->PC += InsnSize (C->PC, C->CPSR);           /* skip the faulting insn */
}

STATIC EFI_CPU_ARCH_PROTOCOL  *mCpu;

STATIC
EFI_STATUS
InstallTraps (
  VOID
  )
{
  EFI_STATUS  Status;

  Status = gBS->LocateProtocol (&gEfiCpuArchProtocolGuid, NULL, (VOID **)&mCpu);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = mCpu->RegisterInterruptHandler (mCpu, EXCEPT_ARM_UNDEFINED_INSTRUCTION, TrapHandler);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = mCpu->RegisterInterruptHandler (mCpu, EXCEPT_ARM_DATA_ABORT, TrapHandler);
  if (EFI_ERROR (Status)) {
    mCpu->RegisterInterruptHandler (mCpu, EXCEPT_ARM_UNDEFINED_INSTRUCTION, NULL);
    return Status;
  }

  Status = mCpu->RegisterInterruptHandler (mCpu, EXCEPT_ARM_PREFETCH_ABORT, TrapHandler);
  if (EFI_ERROR (Status)) {
    mCpu->RegisterInterruptHandler (mCpu, EXCEPT_ARM_UNDEFINED_INSTRUCTION, NULL);
    mCpu->RegisterInterruptHandler (mCpu, EXCEPT_ARM_DATA_ABORT, NULL);
  }

  return Status;
}

STATIC
VOID
RemoveTraps (
  VOID
  )
{
  if (mCpu != NULL) {
    mCpu->RegisterInterruptHandler (mCpu, EXCEPT_ARM_UNDEFINED_INSTRUCTION, NULL);
    mCpu->RegisterInterruptHandler (mCpu, EXCEPT_ARM_DATA_ABORT, NULL);
    mCpu->RegisterInterruptHandler (mCpu, EXCEPT_ARM_PREFETCH_ABORT, NULL);
  }
}

#define PROBE_BEGIN()  do { mTrapped = 0; mProbing = 1; } while (0)
#define PROBE_END()    do { mProbing = 0; } while (0)

/* --------------------------------------------- 1. ID registers */

STATIC UINT32  mMmfr2, mMmfr3;

STATIC
VOID
TestIdRegs (
  VOID
  )
{
  UINT32  Midr, Mpidr, Ctr, Clidr, Sctlr, Actlr, Ttbcr, Ttbr0, Dacr;
  UINT32  Isar0, Isar2, Isar3, Isar4, Pfr1, Dfr0;

  Midr   = MRC (0, c0, c0, 0);
  Ctr    = MRC (0, c0, c0, 1);
  Mpidr  = MRC (0, c0, c0, 5);
  Pfr1   = MRC (0, c0, c1, 1);
  Dfr0   = MRC (0, c0, c1, 2);
  mMmfr2 = MRC (0, c0, c1, 6);
  mMmfr3 = MRC (0, c0, c1, 7);
  Isar0  = MRC (0, c0, c2, 0);
  Isar2  = MRC (0, c0, c2, 2);
  Isar3  = MRC (0, c0, c2, 3);
  Isar4  = MRC (0, c0, c2, 4);
  Clidr  = MRC (1, c0, c0, 1);
  Sctlr  = MRC (0, c1, c0, 0);
  Actlr  = MRC (0, c1, c0, 1);
  Ttbr0  = MRC (0, c2, c0, 0);
  Ttbcr  = MRC (0, c2, c0, 2);
  Dacr   = MRC (0, c3, c0, 0);

  Log ("== 1. ID registers\n");
  Log ("MIDR  %08x MPIDR %08x CTR   %08x\n", Midr, Mpidr, Ctr);
  Log ("MMFR2 %08x MMFR3 %08x CLIDR %08x\n", mMmfr2, mMmfr3, Clidr);
  Log ("ISAR0 %08x ISAR2 %08x ISAR3 %08x\n", Isar0, Isar2, Isar3);
  Log ("ISAR4 %08x PFR1  %08x DFR0  %08x\n", Isar4, Pfr1, Dfr0);
  Log ("SCTLR %08x ACTLR %08x DACR  %08x\n", Sctlr, Actlr, Dacr);
  Log ("TTBR0 %08x TTBCR %08x\n", Ttbr0, Ttbcr);
  Log ("ID says: TLB lvl %d (need 3), bcast %d (need 2),\n",
       (mMmfr2 >> 16) & 0xF, (mMmfr3 >> 12) & 0xF);
  Log ("  coherent walk %d (need 1), MP ext %a\n",
       (mMmfr3 >> 20) & 0xF, (Mpidr & BIT31) ? "yes" : "no (UP)");
}

/* --------------------------------------------- 2. UNDEF probes */

typedef struct {
  CONST CHAR8  *Name;
  VOID (*Fn)(VOID);
} PROBE;

STATIC UINT32  mScratch[4] __attribute__ ((aligned (16)));

STATIC VOID PrTlbiall (VOID)  { MCR (0, c8, c7, 0, 0); }
STATIC VOID PrTlbimva (VOID)  { MCR (0, c8, c7, 1, 0xFFF00000); }
STATIC VOID PrTlbiasid (VOID) { MCR (0, c8, c7, 2, 0xFF); }
STATIC VOID PrTlbimvaa (VOID) { MCR (0, c8, c7, 3, 0xFFF00000); }
STATIC VOID PrTlbiallis (VOID)  { MCR (0, c8, c3, 0, 0); }
STATIC VOID PrTlbimvais (VOID)  { MCR (0, c8, c3, 1, 0xFFF00000); }
STATIC VOID PrTlbiasidis (VOID) { MCR (0, c8, c3, 2, 0xFF); }
STATIC VOID PrTlbimvaais (VOID) { MCR (0, c8, c3, 3, 0xFFF00000); }
STATIC VOID PrDtlbiall (VOID) { MCR (0, c8, c6, 0, 0); }
STATIC VOID PrItlbiall (VOID) { MCR (0, c8, c5, 0, 0); }
STATIC VOID PrIciallu (VOID)  { MCR (0, c7, c5, 0, 0); }
STATIC VOID PrIcialluis (VOID) { MCR (0, c7, c1, 0, 0); }
STATIC VOID PrBpiall (VOID)   { MCR (0, c7, c5, 6, 0); }
STATIC VOID PrBpiallis (VOID) { MCR (0, c7, c1, 6, 0); }
STATIC VOID PrDccmvau (VOID)  { DCCMVAU ((UINTN)mScratch); }
STATIC VOID PrDccimvac (VOID) { DCCIMVAC ((UINTN)mScratch); }
STATIC VOID PrCp15dmb (VOID)  { MCR (0, c7, c10, 5, 0); }
STATIC VOID PrCp15dsb (VOID)  { MCR (0, c7, c10, 4, 0); }
STATIC VOID PrCp15isb (VOID)  { MCR (0, c7, c5, 4, 0); }
STATIC VOID PrDmb (VOID)      { __asm__ volatile ("dmb ish" ::: "memory"); }
STATIC VOID PrClrex (VOID)    { __asm__ volatile ("clrex" ::: "memory"); }

STATIC VOID
PrLdrexb (VOID)
{
  UINT32  V, F;

  __asm__ volatile ("ldrexb %0, [%2]\n strexb %1, %0, [%2]" : "=&r" (V), "=&r" (F) : "r" (mScratch) : "memory");
}

STATIC VOID
PrLdrexh (VOID)
{
  UINT32  V, F;

  __asm__ volatile ("ldrexh %0, [%2]\n strexh %1, %0, [%2]" : "=&r" (V), "=&r" (F) : "r" (mScratch) : "memory");
}

STATIC VOID
PrLdrexd (VOID)
{
  register UINT32  R0 __asm__ ("r0");
  register UINT32  R1 __asm__ ("r1");
  register UINT32  R2 __asm__ ("r2");

  __asm__ volatile (
    "ldrexd r0, r1, [%3]\n"
    "strexd r2, r0, r1, [%3]"
    : "=&r" (R0), "=&r" (R1), "=&r" (R2) : "r" (mScratch) : "memory");
}

/* UDIV r0, r1, r2 - hand-encoded so the assembler accepts it for armv7-a */
STATIC VOID
PrUdiv (VOID)
{
  register UINT32  R0 __asm__ ("r0");
  register UINT32  R1 __asm__ ("r1") = 100;
  register UINT32  R2 __asm__ ("r2") = 7;

#if defined (__thumb__)
  __asm__ volatile (".inst.w 0xfbb1f0f2" : "=r" (R0) : "r" (R1), "r" (R2));
#else
  __asm__ volatile (".inst 0xe730f211" : "=r" (R0) : "r" (R1), "r" (R2));
#endif
  mScratch[3] = R0;
}

STATIC CONST PROBE  mProbes[] = {
  { "TLBIALL    c8,c7,0",  PrTlbiall    },
  { "TLBIMVA    c8,c7,1",  PrTlbimva    },
  { "TLBIASID   c8,c7,2",  PrTlbiasid   },
  { "TLBIMVAA   c8,c7,3",  PrTlbimvaa   },
  { "DTLBIALL   c8,c6,0",  PrDtlbiall   },
  { "ITLBIALL   c8,c5,0",  PrItlbiall   },
  { "TLBIALLIS  c8,c3,0",  PrTlbiallis  },
  { "TLBIMVAIS  c8,c3,1",  PrTlbimvais  },
  { "TLBIASIDIS c8,c3,2",  PrTlbiasidis },
  { "TLBIMVAAIS c8,c3,3",  PrTlbimvaais },
  { "ICIALLU    c7,c5,0",  PrIciallu    },
  { "ICIALLUIS  c7,c1,0",  PrIcialluis  },
  { "BPIALL     c7,c5,6",  PrBpiall     },
  { "BPIALLIS   c7,c1,6",  PrBpiallis   },
  { "DCCMVAU    c7,c11,1", PrDccmvau    },
  { "DCCIMVAC   c7,c14,1", PrDccimvac   },
  { "CP15DMB    c7,c10,5", PrCp15dmb    },
  { "CP15DSB    c7,c10,4", PrCp15dsb    },
  { "CP15ISB    c7,c5,4",  PrCp15isb    },
  { "DMB ISH",             PrDmb        },
  { "CLREX",               PrClrex      },
  { "LDREXB/STREXB",       PrLdrexb     },
  { "LDREXH/STREXH",       PrLdrexh     },
  { "LDREXD/STREXD",       PrLdrexd     },
  { "UDIV",                PrUdiv       },
};

/* index into mProbes for the verdict */
STATIC BOOLEAN  mHave[ARRAY_SIZE (mProbes)];

STATIC
VOID
TestUndef (
  VOID
  )
{
  UINTN  i;

  Log ("== 2. instruction probes (ok / UNDEF)\n");
  for (i = 0; i < ARRAY_SIZE (mProbes); i++) {
    PROBE_BEGIN ();
    mProbes[i].Fn ();
    PROBE_END ();
    mHave[i] = (mTrapped == 0);
    if (mTrapped == 0) {
      Log ("  %-20a ok\n", mProbes[i].Name);
    } else {
      Log ("  %-20a %a\n", mProbes[i].Name,
           (mTrapped == EXCEPT_ARM_UNDEFINED_INSTRUCTION + 1) ? "UNDEF" : "ABORT");
    }
  }

  /* the probes may have dropped TLB / BP state: put things back in order */
  TLBIALL ();
  BPIALL ();
  DSB ();
  ISB ();
}

/* ------------------------------- test section mapping (3 and 4) */

STATIC volatile UINT32  *mL1;           /* L1 translation table (identity mapped) */
STATIC UINTN            mIdx;           /* free L1 slot we borrow           */
STATIC UINTN            mTestVa;        /* mIdx << 20                       */
STATIC UINTN            mPaA, mPaB;     /* two 1 MiB-aligned physical MiBs  */
STATIC UINT32           mDescA, mDescB;
STATIC EFI_PHYSICAL_ADDRESS  mBuf;

#define MARK_A  0xAAAA5A5Au
#define MARK_B  0xBBBB5B5Bu

/*
  Section descriptor: Normal WB-WA (TEX=001 C=1 B=1), non-shared, global,
  XN, domain 0, AP=11 (AP[0] is also the access flag when SCTLR.AFE=1).
*/
STATIC
UINT32
SectionDesc (
  IN UINTN  Pa
  )
{
  return (UINT32)(Pa & 0xFFF00000u) | (1u << 12) | (3u << 10) | BIT4 | BIT3 | BIT2 | 0x2;
}

STATIC
EFI_STATUS
SetupMapping (
  VOID
  )
{
  UINT32                Ttbcr;
  EFI_PHYSICAL_ADDRESS  Buf;
  EFI_STATUS            Status;
  UINTN                 i;

  Ttbcr = MRC (0, c2, c0, 2);
  if ((Ttbcr & 0x7) != 0 || (Ttbcr & BIT31) != 0) {
    Log ("TTBCR %08x: N!=0 or LPAE, mapping tests skipped\n", Ttbcr);
    return EFI_UNSUPPORTED;
  }

  mL1 = (volatile UINT32 *)(UINTN)(MRC (0, c2, c0, 0) & 0xFFFFC000u);

  /* a free (fault) 1 MiB slot, high up and away from RAM and the SoC MMIO */
  mIdx = 0;
  for (i = 0xFF0; i >= 0x400; i--) {
    if ((mL1[i] & 0x3) == 0) {
      mIdx = i;
      break;
    }
  }

  if (mIdx == 0) {
    Log ("no free L1 slot, mapping tests skipped\n");
    return EFI_NOT_FOUND;
  }

  mTestVa = mIdx << 20;

  /* 3 MiB holds two aligned MiBs */
  Status = gBS->AllocatePages (AllocateAnyPages, EfiBootServicesData, EFI_SIZE_TO_PAGES (3 * SZ_1M), &Buf);
  mBuf   = Buf;
  if (EFI_ERROR (Status)) {
    Log ("AllocatePages 3M: %r\n", Status);
    return Status;
  }

  mPaA = ALIGN_VALUE ((UINTN)Buf, SZ_1M);
  mPaB = mPaA + SZ_1M;

  *(volatile UINT32 *)mPaA = MARK_A;
  *(volatile UINT32 *)mPaB = MARK_B;
  DCCMVAC (mPaA);
  DCCMVAC (mPaB);
  DSB ();

  mDescA = SectionDesc (mPaA);
  mDescB = SectionDesc (mPaB);

  Log ("L1 %08x slot %03x VA %08x A %08x B %08x\n", (UINTN)mL1, mIdx, mTestVa, mPaA, mPaB);
  return EFI_SUCCESS;
}

STATIC
VOID
TeardownMapping (
  VOID
  )
{
  if (mIdx != 0) {
    mL1[mIdx] = 0;
    DCCMVAC ((UINTN)&mL1[mIdx]);
    DSB ();
    TLBIALL ();
    DSB ();
    ISB ();
    gBS->FreePages (mBuf, EFI_SIZE_TO_PAGES (3 * SZ_1M));
  }
}

/* write the slot and make it visible to any walker (clean to PoC) */
STATIC
VOID
PutDescClean (
  IN UINT32  Desc
  )
{
  mL1[mIdx] = Desc;
  DCCMVAC ((UINTN)&mL1[mIdx]);
  DSB ();
}

/* read the test VA; returns 'A', 'B', 'F' (fault) or '?' */
STATIC
CHAR8
ReadVa (
  VOID
  )
{
  UINT32  V;

  PROBE_BEGIN ();
  V = *(volatile UINT32 *)mTestVa;
  PROBE_END ();

  if (mTrapped != 0) {
    return 'F';
  }

  return (V == MARK_A) ? 'A' : (V == MARK_B) ? 'B' : '?';
}

/* map A, flush TLB, touch it so the TLB holds VA->A */
STATIC
CHAR8
Prime (
  VOID
  )
{
  PutDescClean (mDescA);
  TLBIALL ();
  DSB ();
  ISB ();
  return ReadVa ();
}

/* --------------------------------------------- 3. TLB ops */

typedef enum {
  OpNone, OpTlbiall, OpTlbimva, OpTlbimvaa, OpDtlbiall, OpDtlbimva,
  OpTlbiallis, OpTlbimvais, OpTlbimvaais, OpMax
} TLB_OP;

STATIC CONST CHAR8  *mOpName[OpMax] = {
  "none (control)", "TLBIALL", "TLBIMVA", "TLBIMVAA", "DTLBIALL", "DTLBIMVA",
  "TLBIALLIS", "TLBIMVAIS", "TLBIMVAAIS"
};

STATIC CHAR8  mOpResult[OpMax];

STATIC
BOOLEAN
DoTlbOp (
  IN TLB_OP  Op
  )
{
  UINT32  Asid = MRC (0, c13, c0, 1) & 0xFF;
  UINT32  Mva  = (UINT32)mTestVa;

  PROBE_BEGIN ();
  switch (Op) {
    case OpTlbiall:    MCR (0, c8, c7, 0, 0);          break;
    case OpTlbimva:    MCR (0, c8, c7, 1, Mva | Asid); break;
    case OpTlbimvaa:   MCR (0, c8, c7, 3, Mva);        break;
    case OpDtlbiall:   MCR (0, c8, c6, 0, 0);          break;
    case OpDtlbimva:   MCR (0, c8, c6, 1, Mva | Asid); break;
    case OpTlbiallis:  MCR (0, c8, c3, 0, 0);          break;
    case OpTlbimvais:  MCR (0, c8, c3, 1, Mva | Asid); break;
    case OpTlbimvaais: MCR (0, c8, c3, 3, Mva);        break;
    default:                                           break;
  }

  PROBE_END ();
  DSB ();
  ISB ();
  return mTrapped == EXCEPT_ARM_UNDEFINED_INSTRUCTION + 1;
}

STATIC
VOID
TestTlbOps (
  VOID
  )
{
  UINTN  Op;
  CHAR8    P, R;
  BOOLEAN  Undef;

  Log ("== 3. TLB ops: prime VA->A, remap to B (cleaned),\n");
  Log ("   one op, read. B = op invalidated, A = stale\n");
  for (Op = 0; Op < OpMax; Op++) {
    P = Prime ();
    PutDescClean (mDescB);
    Undef = DoTlbOp ((TLB_OP)Op);
    R = ReadVa ();
    mOpResult[Op] = R;
    Log ("  %-15a prime %c -> %c%a\n", mOpName[Op], P, R,
         Undef ? " (UNDEF)" : "");
  }

  if (mOpResult[OpNone] == 'B') {
    Log ("  control saw B: TLB did not hold the entry,\n");
    Log ("  op results above are NOT conclusive\n");
  }
}

/* --------------------------------------------- 4. coherent walk */

typedef struct {
  CONST CHAR8  *Name;
  UINT32       Attr;      /* TTBR0[6:0] */
} TTBR_VARIANT;

/*
  TTBR0 low bits without MP extensions: [0] C (inner cacheable walk),
  [1] S, [4:3] RGN (outer: 00 NC, 01 WBWA, 10 WT, 11 WB).  With MP ext the
  inner attribute is IRGN = {bit0, bit6}.  0x6A is the MP-style "inner+outer
  WBWA, shareable, NOS" value an MP-aware kernel would program.
*/
STATIC CONST TTBR_VARIANT  mTtbrVariants[] = {
  { "UEFI current",       0xFFFFFFFF },
  { "C=1 RGN=WBWA (09)",  0x09       },
  { "C=1 RGN=WB   (19)",  0x19       },
  { "C=1 RGN=WT   (11)",  0x11       },
  { "C=0 RGN=NC   (00)",  0x00       },
  { "MP WBWA S NOS (6A)", 0x6A       },
};

STATIC
CHAR8
WalkOnce (
  IN BOOLEAN  CleanToPoU
  )
{
  if (Prime () != 'A') {
    return 'p';                         /* priming failed */
  }

  /*
    Plain store: the new descriptor is only in the (write-back) L1 D-cache;
    memory still holds descA because Prime() cleaned it.
  */
  mL1[mIdx] = mDescB;
  if (CleanToPoU) {
    DCCMVAU ((UINTN)&mL1[mIdx]);
  }

  DSB ();
  TLBIALL ();
  DSB ();
  ISB ();
  return ReadVa ();
}

STATIC CHAR8  mWalkCurrent = '-';

STATIC
VOID
TestCoherentWalk (
  VOID
  )
{
  UINT32  Ttbr0, Base;
  UINTN   v, n;
  CHAR8   NoClean[4], PoU;

  Ttbr0 = MRC (0, c2, c0, 0);
  Base  = Ttbr0 & 0xFFFFC000u;

  Log ("== 4. coherent walk: prime A, plain store B (no\n");
  Log ("   clean), TLBIALL, read. B = walker sees D-cache\n");

  for (v = 0; v < ARRAY_SIZE (mTtbrVariants); v++) {
    UINT32  T = (mTtbrVariants[v].Attr == 0xFFFFFFFF) ? Ttbr0
                                                      : (Base | mTtbrVariants[v].Attr);

    MCR (0, c2, c0, 0, T);
    ISB ();
    TLBIALL ();
    DSB ();
    ISB ();

    for (n = 0; n < 3; n++) {
      NoClean[n] = WalkOnce (FALSE);
    }

    NoClean[3] = 0;
    PoU        = WalkOnce (TRUE);

    /* leave the slot clean in memory before switching attributes again */
    PutDescClean (mDescA);

    Log ("  %-19a TTBR %08x\n", mTtbrVariants[v].Name, MRC (0, c2, c0, 0));
    Log ("    no clean x3: %a   clean PoU: %c\n", NoClean, PoU);

    if (v == 0) {
      mWalkCurrent = NoClean[0];
    }
  }

  MCR (0, c2, c0, 0, Ttbr0);
  ISB ();
  TLBIALL ();
  DSB ();
  ISB ();
}

/* --------------------------------------------- 5. I/D coherency */

typedef UINT32 (*RET_FN)(VOID);

STATIC
VOID
TestIdCoherency (
  VOID
  )
{
  EFI_PHYSICAL_ADDRESS  Page;
  volatile UINT32       *Code;
  RET_FN                Fn;
  UINT32                r[5];

  if (EFI_ERROR (gBS->AllocatePages (AllocateAnyPages, EfiLoaderCode, 1, &Page))) {
    Log ("== 5. I/D: no code page\n");
    return;
  }

  Code = (volatile UINT32 *)(UINTN)Page;
  Fn   = (RET_FN)(UINTN)Page;               /* bit0 = 0: ARM state */

  #define SET_IMM(i)  (Code[0] = 0xE3A00000u | (i))   /* mov r0, #i */
  #define SYNC_FULL()                                    \
    do { DCCMVAU ((UINTN)Code); DSB (); ICIALLU (); BPIALL (); DSB (); ISB (); } while (0)

  Code[1] = 0xE12FFF1Eu;                    /* bx lr */
  mProbing = 1;
  SET_IMM (1);
  SYNC_FULL ();
  r[0] = Fn ();                             /* 1: baseline, I-line now cached */

  SET_IMM (2);                              /* plain store only */
  DSB ();
  ISB ();
  r[1] = Fn ();

  SET_IMM (3);                              /* D clean only, no I inval */
  DCCMVAU ((UINTN)Code);
  DSB ();
  ISB ();
  r[2] = Fn ();

  SET_IMM (4);                              /* I inval only, no D clean */
  ICIALLU ();
  BPIALL ();
  DSB ();
  ISB ();
  r[3] = Fn ();

  SET_IMM (5);
  SYNC_FULL ();
  r[4] = Fn ();
  mProbing = 0;

  Log ("== 5. I/D coherency (self-modifying code)\n");
  Log ("  full sync      expect 1 got %d\n", r[0]);
  Log ("  store only     new=2    got %d\n", r[1]);
  Log ("  D clean only   new=3    got %d\n", r[2]);
  Log ("  I inval only   new=4    got %d\n", r[3]);
  Log ("  full sync      expect 5 got %d\n", r[4]);

  gBS->FreePages (Page, 1);
}

/* --------------------------------------------- verdict */

STATIC
BOOLEAN
HaveProbe (
  IN CONST CHAR8  *Prefix
  )
{
  UINTN  i;

  for (i = 0; i < ARRAY_SIZE (mProbes); i++) {
    if (AsciiStrnCmp (mProbes[i].Name, Prefix, AsciiStrLen (Prefix)) == 0) {
      return mHave[i];
    }
  }

  return FALSE;
}

STATIC
VOID
Verdict (
  BOOLEAN  Mapped
  )
{
  BOOLEAN  Ctl  = Mapped && mOpResult[OpNone] == 'A';
  BOOLEAN  Mvaa = HaveProbe ("TLBIMVAA ") && (!Ctl || mOpResult[OpTlbimvaa] == 'B');
  BOOLEAN  Is   = HaveProbe ("TLBIALLIS") && HaveProbe ("TLBIMVAIS") &&
                  HaveProbe ("TLBIMVAAIS") && HaveProbe ("ICIALLUIS") &&
                  HaveProbe ("BPIALLIS") &&
                  (!Ctl || (mOpResult[OpTlbiallis] == 'B' &&
                            mOpResult[OpTlbimvais] == 'B' &&
                            mOpResult[OpTlbimvaais] == 'B'));

  Log ("== verdict (for NOPping the 0x5D p1=2 checks)\n");
  Log ("  MMFR2 TLB lvl3 / TLBIMVAA : %a\n", Mvaa ? "HW OK" : "MISSING");
  Log ("  MMFR3 broadcast / *IS ops : %a\n", Is ? "HW OK (UP: local)" : "MISSING");
  if (!Mapped) {
    Log ("  MMFR3 coherent walk       : not tested\n");
  } else {
    Log ("  MMFR3 coherent walk       : %a\n",
         (mWalkCurrent == 'B') ? "HW OK (UEFI TTBR)" : "NO - PTEs need clean");
  }

  if (!Ctl && Mapped) {
    Log ("  (TLB control not stale: TLB results unproven)\n");
  }
}

/* --------------------------------------------- entry */

EFI_STATUS
EFIAPI
CacheCoherencyTestMain (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS  Status;
  BOOLEAN     IrqWasOn;
  BOOLEAN     Mapped;
  EFI_EVENT   Ev[2];
  UINTN       Idx;

  gST->ConOut->ClearScreen (gST->ConOut);
  Log ("CacheCoherencyTest 0.1 - Scorpion vs 0x5D\n");

  Status = InstallTraps ();
  if (EFI_ERROR (Status)) {
    Log ("cannot hook UNDEF/abort vectors: %r\n", Status);
    return Status;
  }

  TestIdRegs ();

  IrqWasOn = ArmGetInterruptState ();
  ArmDisableInterrupts ();

  TestUndef ();
  Mapped = !EFI_ERROR (SetupMapping ());
  if (Mapped) {
    TestTlbOps ();
    TestCoherentWalk ();
    TeardownMapping ();
  }

  if (IrqWasOn) {
    ArmEnableInterrupts ();
  }

  TestIdCoherency ();
  Verdict (Mapped);
  RemoveTraps ();

  SaveLog (ImageHandle);

  /* stay on screen for a key or 120 s, then return to the boot manager */
  Print (L"press a key (120 s)\n");
  Ev[0] = gST->ConIn->WaitForKey;
  if (!EFI_ERROR (gBS->CreateEvent (EVT_TIMER, 0, NULL, NULL, &Ev[1]))) {
    gBS->SetTimer (Ev[1], TimerRelative, 120 * 10000000ull);
    gBS->WaitForEvent (2, Ev, &Idx);
    gBS->CloseEvent (Ev[1]);
  }

  return EFI_SUCCESS;
}
