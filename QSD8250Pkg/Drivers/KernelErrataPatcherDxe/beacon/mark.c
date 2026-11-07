/* BeaconMark(id, borrow, value): paint milestone stripe <id> on the HD2 panel from
   any kernel context, without boot graphics and without a mapping of the
   framebuffer.

   Windows ARM keeps its page tables self-mapped at 0xC0000000 (PTE of VA v
   at 0xC0000000 + (v >> 12) * 4).  The routine borrows the 16 PTEs of
   <borrow> (a 64 KiB run of kernel-image pages that never run at boot),
   points them at framebuffer phys 0x02A00000 + id * 64 KiB with the
   attributes of the current stack page (kernel RW normal memory), fills the
   stripe, cleans it to memory and puts the original PTEs back.  IRQs are off
   for the whole borrow.  64 KiB = 34 rows of 480 px, so ids 0..22 tile the
   800-row panel top to bottom, in the left 64 px (id | 0x40 paints the right
   32 px of it, the left 32 px otherwise); the first row of each
   stripe stays black as a separator.  id | 0x80 paints <value> instead, as
   32 full-width cells (bit 31 left, white = 1).

   Self-contained: no calls, no globals, constants only, so the compiled
   bytes run from anywhere. */
typedef unsigned int u32;

#define PTE(va)    ((volatile u32 *)(0xC0000000u + (((u32)(va)) >> 12) * 4u))
#define FB_PHYS    0x02A00000u
#define STRIPE     0x10000u
#define PAGES      (STRIPE / 0x1000u)
#define MAX_ID     22u

static inline void dccmvac(u32 a) { __asm__ volatile("mcr p15, 0, %0, c7, c10, 1" :: "r"(a) : "memory"); }
static inline void tlbimva(u32 a) { __asm__ volatile("mcr p15, 0, %0, c8, c7, 1" :: "r"(a) : "memory"); }
static inline void dsb(void) { __asm__ volatile("dsb" ::: "memory"); }
static inline void isb(void) { __asm__ volatile("isb" ::: "memory"); }
static inline void iciallu(void) { __asm__ volatile("mcr p15, 0, %0, c7, c5, 0" :: "r"(0) : "memory"); }
static inline void bpiall(void)  { __asm__ volatile("mcr p15, 0, %0, c7, c5, 6" :: "r"(0) : "memory"); }

static void mark(u32 id, u32 borrow, u32 value);

void BeaconMark(u32 id, u32 borrow, u32 value)
{
    mark(id, borrow, value);
}

static __attribute__((noinline)) void mark(u32 id, u32 borrow, u32 value)
{
    u32 cpsr, sp, attr, phys, color, k, i, a, x0, bits;
    u32 old[PAGES];
    volatile u32 *fb = (volatile u32 *)borrow;

    __asm__ volatile("mrs %0, cpsr" : "=r"(cpsr));
    __asm__ volatile("cpsid i" ::: "memory");
    __asm__ volatile("mov %0, sp" : "=r"(sp));

    /* nG cleared: a global mapping, so the TLB flush below hits it whatever
       ASID is current (the real Scorpion kept stale non-global entries) */
    attr = *PTE(sp) & 0xFFFu & ~0x800u;
    if ((attr & 2u) == 0)               /* stack is not a small page: give up */
        goto out;
    bits = (id & 0x80u) != 0;           /* 0x80: value as 32 bit cells */
    x0 = (id & 0x40u) ? 32u : 0u;       /* 0x40: right half (second call) */
    id &= 0x3Fu;
    if (id > MAX_ID)
        id = MAX_ID;
    phys = FB_PHYS + id * STRIPE;
    k = id % 7u + 1u;                   /* 1..7: R, G, RG, B, RB, GB, RGB */
    color = 0xFF000000u | ((k & 1u) ? 0xFF0000u : 0) | ((k & 2u) ? 0xFF00u : 0) | ((k & 4u) ? 0xFFu : 0);

    for (k = 0; k < PAGES; k++) {
        volatile u32 *p = PTE(borrow + k * 0x1000u);
        old[k] = *p;
        *p = (phys + k * 0x1000u) | attr;
        dccmvac((u32)p);
    }
    dsb();
    /* per page, NOT TLBIALL: a full flush would also drop stale entries of
       the code being traced and hide the bug (beacon v9).  The mapping is
       global (nG cleared), so TLBIMVA hits it on the Scorpion too. */
    for (k = 0; k < PAGES; k++)
        tlbimva(borrow + k * 0x1000u);
    dsb();
    isb();

    /* left 64 px column only, so the HAL extensions' text console (x >= 72)
       and the firmware's log stay readable next to the stripes */
    for (i = 0; i < STRIPE / 4u; i++) {
        u32 g = id * (STRIPE / 4u) + i;     /* pixel index in the panel */
        u32 x = g % 480u;
        if (bits) {
            /* 32 cells of 15 px across the panel, MSB left: set = white,
               clear = dark grey, 1 px black gaps; first row black */
            u32 cell = x / 15u, on = (value >> (31u - cell)) & 1u;
            fb[i] = (i < 480u || x % 15u == 14u) ? 0xFF000000u : (on ? 0xFFFFFFFFu : 0xFF303030u);
        } else if (x >= x0 && x < x0 + 32u) {
            fb[i] = (i < 480u) ? 0xFF000000u : color;
        }
    }
    for (a = borrow; a < borrow + STRIPE; a += 32u)
        dccmvac(a);
    dsb();

    for (k = 0; k < PAGES; k++) {
        volatile u32 *p = PTE(borrow + k * 0x1000u);
        *p = old[k];
        dccmvac((u32)p);
    }
    dsb();
    /* per page, NOT TLBIALL: a full flush would also drop stale entries of
       the code being traced and hide the bug (beacon v9).  The mapping is
       global (nG cleared), so TLBIMVA hits it on the Scorpion too. */
    for (k = 0; k < PAGES; k++)
        tlbimva(borrow + k * 0x1000u);
    dsb();
    isb();
out:
    if (!(cpsr & 0x80u))
        __asm__ volatile("cpsie i" ::: "memory");
}

/* BeaconMapPayload(va, phys, pages): point the PTEs of <pages> kernel-image
   pages at <va> (PAGEVRFY, never runs) at the firmware's reserved debug
   payload at <phys> (KEP_PAYLOAD_PHYS), executable, global, with the stack
   page's memory attributes.  Called once from the milestone 0 stub, before
   any HAL code runs; the hal.dll trace stubs live in the payload. */

/* Clean + invalidate every data / unified cache level up to LoC by set/way
   (Scorpion: CLIDR 0A000023 = L1 I/D + L2 unified). */
static void dcache_cisw_all(void)
{
    u32 clidr, loc, level;

    __asm__ volatile("mrc p15, 1, %0, c0, c0, 1" : "=r"(clidr));
    loc = (clidr >> 24) & 7u;
    for (level = 0; level < loc; level++) {
        u32 ccsidr, lsize, ways, sets, wshift, w, st;
        if (((clidr >> (level * 3u)) & 7u) < 2u)
            continue;                       /* no data cache at this level */
        __asm__ volatile("mcr p15, 2, %0, c0, c0, 0" :: "r"(level << 1) : "memory");
        isb();
        __asm__ volatile("mrc p15, 1, %0, c0, c0, 0" : "=r"(ccsidr));
        lsize = (ccsidr & 7u) + 4u;
        ways = (ccsidr >> 3) & 0x3FFu;
        sets = (ccsidr >> 13) & 0x7FFFu;
        wshift = ways ? (u32)__builtin_clz(ways) : 0u;
        for (w = 0; w <= ways; w++)
            for (st = 0; st <= sets; st++)
                __asm__ volatile("mcr p15, 0, %0, c7, c14, 2"
                                 :: "r"((w << wshift) | (st << lsize) | (level << 1)) : "memory");
    }
    dsb();
    isb();
}

/* Make every write-back Normal memory type write-through (NMRR inner and
   outer fields 01 / 11 -> 10): PTE stores then reach memory, where the
   Scorpion's table walker reads (Windows never cleans PTEs; Linux does,
   cpu_v7_set_pte_ext flush_pte).  Needs SCTLR.TRE (TEX remap).  Returns
   the old NMRR. */
static u32 nmrr_write_through(void)
{
    u32 old, nw = 0, i;

    __asm__ volatile("mrc p15, 0, %0, c10, c2, 1" : "=r"(old));
    for (i = 0; i < 16u; i++) {
        u32 f = (old >> (2u * i)) & 3u;
        if (f == 1u || f == 3u)
            f = 2u;
        nw |= f << (2u * i);
    }
    dsb();
    __asm__ volatile("mcr p15, 0, %0, c10, c2, 1" :: "r"(nw) : "memory");
    isb();
    __asm__ volatile("mcr p15, 0, %0, c8, c7, 0" :: "r"(0) : "memory");   /* TLBIALL */
    dsb();
    isb();
    dcache_cisw_all();                  /* no dirty write-back lines left */
    iciallu();
    bpiall();
    dsb();
    isb();
    return old;
}

/* flags (r3, set by the KEP's mapper stub) */
#define MAP_TTBR_SHARED  1u     /* TTBR0.S = 1 (no visible effect on the HD2) */
#define MAP_NMRR_WT      2u     /* write-back Normal memory -> write-through (REQUIRED on the HD2) */
#define MAP_DIAG_ROWS    4u     /* paint SCTLR / PRRR / old NMRR on rows 20-22 */

void BeaconMapPayload(u32 va, u32 phys, u32 pages, u32 flags)
{
    u32 cpsr, sp, attr, k;

    __asm__ volatile("mrs %0, cpsr" : "=r"(cpsr));
    __asm__ volatile("cpsid i" ::: "memory");
    __asm__ volatile("mov %0, sp" : "=r"(sp));

    /* Walk attributes: winload hands over TTBR0 with 0x09 (inner WB,
       NON-shared, outer WB).  The real Scorpion's table walker does not see
       L1 with those (measured: 09/19/11 stale, 0B/0A/00 coherent), and the KEP
       makes Windows believe the walk is coherent (MMFR3 patch), so fresh PTEs
       are walked stale -> silent fault loops (beacons v6..v16).  Set S (bit
       1) -> 0x0B before the HAL runs; KiSetFeatureBits copies TTBR0 & 0x7F
       into every later page directory base. */
    if (flags & MAP_TTBR_SHARED) {
        u32 t;
        __asm__ volatile("mrc p15, 0, %0, c2, c0, 0" : "=r"(t));
        if (!(t & 2u)) {
            dsb();
            __asm__ volatile("mcr p15, 0, %0, c2, c0, 0" :: "r"(t | 2u) : "memory");
            isb();
            __asm__ volatile("mcr p15, 0, %0, c8, c7, 0" :: "r"(0) : "memory");   /* TLBIALL */
            dsb();
            isb();
        }
    }

    attr = *PTE(sp) & 0xFFFu & ~0x801u;     /* clear nG and XN (small page bit 0) */
    if ((attr & 2u) == 0)
        goto out;
    for (k = 0; k < pages; k++) {
        volatile u32 *p = PTE(va + k * 0x1000u);
        *p = (phys + k * 0x1000u) | attr;
        dccmvac((u32)p);
    }
    dsb();
    for (k = 0; k < pages; k++)
        tlbimva(va + k * 0x1000u);
    iciallu();
    bpiall();
    dsb();
    isb();

    {
        u32 sctlr, prrr, nmrr_old, borrow = va - 0x10000u;   /* PAGEVRFY base */
        if (flags & MAP_NMRR_WT)
            nmrr_old = nmrr_write_through();
        else
            __asm__ volatile("mrc p15, 0, %0, c10, c2, 1" : "=r"(nmrr_old));
        if (flags & MAP_DIAG_ROWS) {
            __asm__ volatile("mrc p15, 0, %0, c1, c0, 0" : "=r"(sctlr));
            __asm__ volatile("mrc p15, 0, %0, c10, c2, 0" : "=r"(prrr));
            mark(0x80u | 20u, borrow, sctlr);   /* rows 20-22: what Windows runs with */
            mark(0x80u | 21u, borrow, prrr);
            mark(0x80u | 22u, borrow, nmrr_old);
        }
    }
out:
    if (!(cpsr & 0x80u))
        __asm__ volatile("cpsie i" ::: "memory");
}
