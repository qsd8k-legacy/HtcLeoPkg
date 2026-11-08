/** @file
  ResetSystemLib for the HTC HD2 (QSD8250), for ResetSystemRuntimeDxe: the
  OS restarts and powers off through it.

  Reset and power-off are proc_comm commands to the modem (PCOM_RESET_CHIP,
  PCOM_POWER_DOWN), as in PlatformResetLib: the command goes into the SMEM
  mailbox (APP_COMMAND/APP_DATA1/APP_DATA2) and the modem is rung through
  the A2M doorbell (MSM_A2M_INT(6)).

  Runtime: this code runs after ExitBootServices / SetVirtualAddressMap
  (Windows' restart and shut down).  The SMEM mailbox page and the doorbell
  page are made runtime MMIO so the OS maps them, both addresses are
  converted on the virtual address change, and the runtime paths use
  neither TimerLib nor DEBUG (both touch hardware at physical addresses the
  OS has not mapped).  PlatformResetLib did all three, so every Windows
  restart bugchecked 0x1E (KMODE_EXCEPTION_NOT_HANDLED, C0000005) inside
  the firmware's ResetSystem.

  The LK boot reason (LK_BOOTREASON_ADDR) lies in the UEFI memory pool,
  which the OS owns after ExitBootServices: it is written only before.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <PiDxe.h>

#include <Guid/EventGroup.h>
#include <Library/ArmLib.h>
#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/DxeServicesTableLib.h>
#include <Library/IoLib.h>
#include <Library/ResetSystemLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeLib.h>

#include <Chipset/iomap.h>
#include <Library/BootReason.h>
#include <Library/pcom.h>
#include <Library/EmuWorkaround.h>

//
// Emulator only (EMU_WORKAROUND, build.sh --emu): QEMU's modem acks
// PCOM_RESET_CHIP but does not restart the machine; it restarts on what
// the HSPL and Linux's htcleo_reset() do instead - PS_HOLD (GPIO 25) dropped
// after the radio raised it.  A real HD2 is reset by the modem.
//
#define PS_HOLD_GPIO_BIT     (1U << (25 - 16))   // GPIO bank 1 (16..42)
#define GPIOCFG2_PAGE        0xA8F00000          // MSM_GPIOCFG2_BASE
#define GPIO_OWNER_1_OFF     0x504               // GPIO2_REG_R (0x104)
#define GPIO2_PAGE           0xA9100000          // MSM_GPIO2_BASE
#define GPIO_OUT_1_OFF       0xC00               // GPIO2_REG (0x00)
#define GPIO_OE_1_OFF        0xC08               // GPIO2_REG (0x08)

#define PCOM_PAGE_SIZE   SIZE_4KB

// mailbox offsets in SMEM (pcom.h APP_* / MDM_*)
#define SMEM_APP_COMMAND  0x00
#define SMEM_APP_STATUS   0x04
#define SMEM_APP_DATA1    0x08
#define SMEM_APP_DATA2    0x0C
#define SMEM_MDM_STATUS   0x14

// MSM_A2M_INT(6) relative to MSM_CSR_BASE
#define CSR_A2M_INT6      (0x400 + 6 * 4)

//
// Bounded waits without a timer: uncached register reads, a few seconds at
// most.  The reset itself does not depend on the answer.
//
#define PCOM_SPINS        20000000u

STATIC UINTN      mSmemBase = MSM_SHARED_BASE;
STATIC UINTN      mCsrBase  = MSM_CSR_BASE;
#if EMU_WORKAROUND
STATIC UINTN      mGpioCfg2Base = GPIOCFG2_PAGE;
STATIC UINTN      mGpio2Base    = GPIO2_PAGE;
#endif
STATIC EFI_EVENT  mVirtualAddressChangeEvent;

STATIC
BOOLEAN
WaitFor (
  IN UINTN   Address,
  IN UINT32  Value
  )
{
  UINT32  Spin;

  for (Spin = 0; Spin < PCOM_SPINS; Spin++) {
    if (MmioRead32 (Address) == Value) {
      return TRUE;
    }
  }

  return FALSE;
}

/**
  proc_comm with the modem, runtime-safe (msm_proc_comm without TimerLib
  and DEBUG).
**/
STATIC
VOID
ProcComm (
  IN UINT32  Command,
  IN UINT32  Data1
  )
{
  WaitFor (mSmemBase + SMEM_MDM_STATUS, PCOM_READY);

  MmioWrite32 (mSmemBase + SMEM_APP_COMMAND, Command);
  MmioWrite32 (mSmemBase + SMEM_APP_DATA1, Data1);
  MmioWrite32 (mSmemBase + SMEM_APP_DATA2, 0);
  ArmDataSynchronizationBarrier ();
  MmioWrite32 (mCsrBase + CSR_A2M_INT6, 1);

  WaitFor (mSmemBase + SMEM_APP_COMMAND, PCOM_CMD_DONE);
}

STATIC
VOID
PlatformReboot (
  IN UINT32  RebootReason
  )
{
  if (!EfiAtRuntime ()) {
    MmioWrite32 (LK_BOOTREASON_ADDR, RebootReason);
    MmioWrite32 (LK_BOOTREASON_ADDR + 4, RebootReason ^ MARK_LK_TAG);
  }

  ProcComm (PCOM_RESET_CHIP, RebootReason);

#if EMU_WORKAROUND
  //
  // htcleo_reset(): the apps processor owns the pin, drives PS_HOLD low.
  //
  MmioOr32 (mGpioCfg2Base + GPIO_OWNER_1_OFF, PS_HOLD_GPIO_BIT);
  MmioOr32 (mGpio2Base + GPIO_OE_1_OFF, PS_HOLD_GPIO_BIT);
  MmioAnd32 (mGpio2Base + GPIO_OUT_1_OFF, ~PS_HOLD_GPIO_BIT);
#endif

  //
  // Should never get here.  Spin so no code runs during the reset.
  //
  CpuDeadLoop ();
}

VOID
EFIAPI
ResetCold (
  VOID
  )
{
  PlatformReboot (ANDRBOOT_MODE);
}

VOID
EFIAPI
ResetWarm (
  VOID
  )
{
  //
  // No warm reset on this platform: a cold one.
  //
  PlatformReboot (ANDRBOOT_MODE);
}

VOID
EFIAPI
ResetShutdown (
  VOID
  )
{
  ProcComm (PCOM_POWER_DOWN, 0);
  CpuDeadLoop ();
}

VOID
EFIAPI
ResetPlatformSpecific (
  IN UINTN  DataSize,
  IN VOID   *ResetData
  )
{
  PlatformReboot (ANDRBOOT_MODE);
}

STATIC
VOID
EFIAPI
VirtualAddressChange (
  IN EFI_EVENT  Event,
  IN VOID       *Context
  )
{
  EfiConvertPointer (0, (VOID **)&mSmemBase);
  EfiConvertPointer (0, (VOID **)&mCsrBase);
#if EMU_WORKAROUND
  EfiConvertPointer (0, (VOID **)&mGpioCfg2Base);
  EfiConvertPointer (0, (VOID **)&mGpio2Base);
#endif
}

/**
  Mark one page uncached + runtime so the OS maps it for the runtime
  services (adding it to the GCD first if nothing covers it).
**/
STATIC
EFI_STATUS
MakeRuntimePage (
  IN EFI_PHYSICAL_ADDRESS  Base,
  IN CONST CHAR8           *Name
  )
{
  EFI_GCD_MEMORY_SPACE_DESCRIPTOR  Desc;
  EFI_STATUS                       Status;

  Status = gDS->GetMemorySpaceDescriptor (Base, &Desc);
  if (EFI_ERROR (Status) || (Desc.GcdMemoryType == EfiGcdMemoryTypeNonExistent)) {
    Status = gDS->AddMemorySpace (EfiGcdMemoryTypeMemoryMappedIo, Base, PCOM_PAGE_SIZE,
                                  EFI_MEMORY_UC | EFI_MEMORY_RUNTIME);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "PlatformResetRuntimeLib: adding %a failed - %r\n", Name, Status));
      return Status;
    }
  } else {
    Status = gDS->SetMemorySpaceCapabilities (Base, PCOM_PAGE_SIZE,
                                              Desc.Capabilities | EFI_MEMORY_UC | EFI_MEMORY_RUNTIME);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "PlatformResetRuntimeLib: %a capabilities - %r\n", Name, Status));
    }
  }

  Status = gDS->SetMemorySpaceAttributes (Base, PCOM_PAGE_SIZE, EFI_MEMORY_UC | EFI_MEMORY_RUNTIME);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "PlatformResetRuntimeLib: %a runtime attribute - %r\n", Name, Status));
  }

  return Status;
}

EFI_STATUS
EFIAPI
PlatformResetRuntimeLibConstructor (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS  Status;

  Status = MakeRuntimePage (MSM_SHARED_BASE, "SMEM proc_comm mailbox");
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = MakeRuntimePage (MSM_CSR_BASE, "CSR A2M doorbell");
  if (EFI_ERROR (Status)) {
    return Status;
  }

#if EMU_WORKAROUND
  Status = MakeRuntimePage (GPIOCFG2_PAGE, "GPIOCFG2 (PS_HOLD owner)");
  if (EFI_ERROR (Status)) {
    return Status;
  }
  Status = MakeRuntimePage (GPIO2_PAGE, "GPIO2 (PS_HOLD)");
  if (EFI_ERROR (Status)) {
    return Status;
  }
#endif

  return gBS->CreateEventEx (EVT_NOTIFY_SIGNAL, TPL_NOTIFY, VirtualAddressChange, NULL,
                             &gEfiEventVirtualAddressChangeGuid, &mVirtualAddressChangeEvent);
}
