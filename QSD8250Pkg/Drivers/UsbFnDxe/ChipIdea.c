/** @file
  ChipIdea (msm72k) device controller backend for UsbFnDxe.

  Sequences from the htcleo kernel: otg_reset() (link reset, ULPI PHY,
  AHB, USBMODE), usb_reset() (ITC 0, ENDPOINTLISTADDR, queue heads),
  usb_ept_start() (dTD + ENDPTPRIME), handle_setup(), handle_endpoint()
  and the USBSTS handling of usb_interrupt().  Clocks and the PHY reset go
  through proc_comm, like the kernel's clock driver on this SoC.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include "UsbFnDxe.h"

#include <Library/pcom.h>

#define REG(o)  (CI_BASE + (o))

STATIC UINT32
PoolToDma (
  IN VOID  *Ptr
  )
{
  return (UINT32)(UINTN)Ptr;   // identity mapped, below 4 GiB
}

STATIC BOOLEAN
WaitClear32 (
  IN UINTN   Reg,
  IN UINT32  Mask,
  IN UINTN   TimeoutUs
  )
{
  while ((MmioRead32 (Reg) & Mask) != 0) {
    if (TimeoutUs-- == 0) {
      return FALSE;
    }
    MicroSecondDelay (1);
  }
  return TRUE;
}

STATIC VOID
UlpiWrite (
  IN UINT8  Value,
  IN UINT8  Register
  )
{
  MmioWrite32 (REG (CI_ULPI_VIEWPORT), ULPI_RUN | ULPI_WRITE | ULPI_ADDR (Register) | ULPI_DATA (Value));
  if (!WaitClear32 (REG (CI_ULPI_VIEWPORT), ULPI_RUN, 10000)) {
    DEBUG ((DEBUG_ERROR, "UsbFn: ULPI write %02x <- %02x timed out\n", Register, Value));
  }
}

STATIC USBFN_XFER *
XferOf (
  IN USBFN_DEV  *Dev,
  IN UINT8      Phys
  )
{
  return (Phys & 1) ? &Dev->In[Phys >> 1] : &Dev->Out[Phys >> 1];
}

STATIC UINT32
EptBit (
  IN UINT8  Phys
  )
{
  return (Phys & 1) ? EPT_TX (Phys >> 1) : EPT_RX (Phys >> 1);
}

STATIC VOID
Flush (
  IN UINT32  Bits
  )
{
  UINTN  Tries;

  for (Tries = 0; Tries < 100; Tries++) {
    MmioWrite32 (REG (CI_ENDPTFLUSH), Bits);
    WaitClear32 (REG (CI_ENDPTFLUSH), Bits, 10000);
    if ((MmioRead32 (REG (CI_ENDPTSTAT)) & Bits) == 0) {
      return;
    }
  }
  DEBUG ((DEBUG_ERROR, "UsbFn: flush %08x stuck\n", Bits));
}

/**
  Hand Length bytes at Buffer (a pool bounce buffer) to the controller on
  physical endpoint Phys (usb_ept_start).
**/
STATIC EFI_STATUS
PrimeTd (
  IN USBFN_DEV  *Dev,
  IN UINT8      Phys,
  IN UINT8      *Buffer,
  IN UINTN      Length
  )
{
  CI_TD       *Td;
  CI_QH       *Qh;
  USBFN_XFER  *X;
  UINT32      Dma;
  UINTN       Page;

  Td  = Dev->Td[Phys];
  Qh  = &Dev->Qh[Phys];
  X   = XferOf (Dev, Phys);
  Dma = PoolToDma (Buffer);

  ZeroMem (Td, sizeof (*Td));
  Td->Next  = TD_TERMINATE;
  Td->Token = TD_BYTES (Length) | TD_IOC | TD_ACTIVE;
  Td->Page[0] = Dma;
  for (Page = 1; Page < 5; Page++) {
    Td->Page[Page] = (Dma + (UINT32)(Page * SIZE_4KB)) & ~0xFFFU;
  }
  WriteBackDataCacheRange (Td, sizeof (*Td));

  Qh->Next  = PoolToDma (Td);
  Qh->Token = 0;
  WriteBackDataCacheRange (Qh, sizeof (*Qh));
  ArmDataSynchronizationBarrier ();

  X->Started    = TRUE;
  X->Programmed = Length;
  MmioWrite32 (REG (CI_ENDPTPRIME), EptBit (Phys));
  return EFI_SUCCESS;
}

/**
  Bytes moved by the finished dTD of Phys; MAX_UINTN if it is still active.
**/
STATIC UINTN
TdDone (
  IN USBFN_DEV  *Dev,
  IN UINT8      Phys
  )
{
  CI_TD       *Td;
  USBFN_XFER  *X;
  UINT32      Token;

  Td = Dev->Td[Phys];
  X  = XferOf (Dev, Phys);
  InvalidateDataCacheRange (Td, sizeof (*Td));
  Token = Td->Token;
  if (Token & TD_ACTIVE) {
    return MAX_UINTN;
  }
  if (Token & TD_ERRORS) {
    DEBUG ((DEBUG_ERROR, "UsbFn: ep%u %a dTD error %08x\n", Phys >> 1, (Phys & 1) ? "in" : "out", Token));
    return 0;
  }
  return X->Programmed - MIN ((UINTN)TD_BYTES_LEFT (Token), X->Programmed);
}

EFI_STATUS
CiAllocatePool (
  IN USBFN_DEV  *Dev
  )
{
  EFI_PHYSICAL_ADDRESS  Base;
  UINT8                 Phys;
  EFI_STATUS            Status;

  Dev->PoolPages = EFI_SIZE_TO_PAGES (CI_POOL_SIZE);
  Base           = SIZE_4GB - 1;
  Status         = gBS->AllocatePages (AllocateMaxAddress, EfiBootServicesData, Dev->PoolPages, &Base);
  if (EFI_ERROR (Status)) {
    return Status;
  }
  Dev->Pool = (UINT8 *)(UINTN)Base;
  ZeroMem (Dev->Pool, EFI_PAGES_TO_SIZE (Dev->PoolPages));

  //
  // Uncached if the CPU driver allows it (descriptors are written by the
  // controller); the cache maintenance below covers the other case.
  //
  Status = gDS->SetMemorySpaceAttributes (Base, EFI_PAGES_TO_SIZE (Dev->PoolPages), EFI_MEMORY_UC);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_INFO, "UsbFn: pool stays cached (%r)\n", Status));
  }
  WriteBackInvalidateDataCacheRange (Dev->Pool, EFI_PAGES_TO_SIZE (Dev->PoolPages));

  Dev->Qh = (CI_QH *)Dev->Pool;
  for (Phys = 0; Phys < CI_PHYS_EPS; Phys++) {
    Dev->Td[Phys] = (CI_TD *)(Dev->Pool + CI_POOL_TD_OFFSET + Phys * CI_TD_SLOT);
    if (Phys < 2) {
      Dev->Bounce[Phys] = Dev->Pool + CI_POOL_EP0_OFFSET + Phys * USBFN_EP0_BUFFER;
    } else {
      Dev->Bounce[Phys] = Dev->Pool + CI_POOL_BULK_OFFSET + (Phys - 2) * USBFN_MAX_TRANSFER;
    }
  }
  return EFI_SUCCESS;
}

//
// ---------------------------------------------------------------------------
// EP0.
// ---------------------------------------------------------------------------
//

STATIC VOID
Ep0Reset (
  IN USBFN_DEV  *Dev
  )
{
  Flush (EPT_RX (0) | EPT_TX (0));
  ZeroMem (&Dev->In[0], sizeof (Dev->In[0]));
  ZeroMem (&Dev->Out[0], sizeof (Dev->Out[0]));
  Dev->Ep0State       = Ep0StateSetup;
  Dev->Ep0StatusReady = FALSE;
}

/**
  Status stage: a ZLP against the data direction (IN for no-data requests).
**/
STATIC VOID
Ep0StartStatus (
  IN USBFN_DEV  *Dev
  )
{
  BOOLEAN  In;

  In            = Dev->Ep0ThreeStage ? !Dev->Ep0DirIn : TRUE;
  Dev->Ep0State = Ep0StateStatus;
  PrimeTd (Dev, PHYS_EP (0, In), Dev->Bounce[PHYS_EP (0, In)], 0);
}

VOID
CiEp0Stall (
  IN USBFN_DEV  *Dev
  )
{
  Ep0Reset (Dev);
  MmioOr32 (REG (CI_ENDPTCTRL (0)), CTRL_TXS | CTRL_RXS);   // cleared by the next SETUP
}

VOID
CiEp0StatusReady (
  IN USBFN_DEV  *Dev
  )
{
  Dev->Ep0StatusReady = TRUE;
  if (Dev->Ep0State == Ep0StateWaitStatus) {
    Ep0StartStatus (Dev);
  }
}

/**
  SETUP in queue head 0 (handle_setup): read it under the tripwire, ack it,
  cancel whatever EP0 was doing and hand the request on.
**/
STATIC VOID
Ep0Setup (
  IN USBFN_DEV  *Dev
  )
{
  EFI_USB_DEVICE_REQUEST  Req;
  UINTN                   Tries;

  for (Tries = 0; Tries < 100; Tries++) {
    MmioOr32 (REG (CI_USBCMD), USBCMD_SUTW);
    InvalidateDataCacheRange (&Dev->Qh[0], sizeof (CI_QH));
    CopyMem (&Req, (VOID *)Dev->Qh[0].Setup, sizeof (Req));
    if (MmioRead32 (REG (CI_USBCMD)) & USBCMD_SUTW) {
      break;   // no newer SETUP overwrote it while we copied
    }
  }
  MmioAnd32 (REG (CI_USBCMD), ~USBCMD_SUTW);
  MmioWrite32 (REG (CI_ENDPTSETUPSTAT), EPT_RX (0));

  Ep0Reset (Dev);
  Dev->Ep0ThreeStage  = (Req.Length != 0);
  Dev->Ep0DirIn       = (Req.RequestType & USB_ENDPOINT_DIR_IN) != 0;
  Dev->Ep0State       = Dev->Ep0ThreeStage ? Ep0StateData : Ep0StateWaitStatus;

  UsbFnEp0Setup (Dev, &Req);
}

STATIC VOID
Ep0Complete (
  IN USBFN_DEV  *Dev,
  IN UINT8      Phys
  )
{
  USBFN_XFER  *X;
  UINTN       Done;

  X    = XferOf (Dev, Phys);
  Done = TdDone (Dev, Phys);
  if (Done == MAX_UINTN) {
    return;
  }
  X->Started = FALSE;

  switch (Dev->Ep0State) {
    case Ep0StateData:
      if (((Phys & 1) != 0) != Dev->Ep0DirIn) {
        break;
      }
      if (X->ZlpSent) {
        // the terminating ZLP of an IN reply went out
      } else {
        if (!Dev->Ep0DirIn) {
          Done = MIN (Done, X->Length);
          InvalidateDataCacheRange (Dev->Bounce[Phys], USBFN_EP0_BUFFER);
          CopyMem (X->Buffer, Dev->Bounce[Phys], Done);
        }
        X->Done = Done;
        if (Dev->Ep0DirIn && X->SendZlp) {
          X->ZlpSent = TRUE;
          PrimeTd (Dev, Phys, Dev->Bounce[Phys], 0);
          return;
        }
      }
      X->Active = FALSE;
      if (!X->Internal) {
        UsbFnQueueTransfer (Dev, 0, Dev->Ep0DirIn);
      }
      Dev->Ep0State = Ep0StateWaitStatus;
      CiEp0StatusReady (Dev);   // data done: status follows on its own
      break;

    case Ep0StateStatus:
      Dev->Ep0State = Ep0StateSetup;
      break;

    default:
      break;
  }
}

//
// ---------------------------------------------------------------------------
// Transfers.
// ---------------------------------------------------------------------------
//

EFI_STATUS
CiStartTransfer (
  IN USBFN_DEV  *Dev,
  IN UINT8      Ep,
  IN BOOLEAN    In
  )
{
  USBFN_XFER  *X;
  UINT8       Phys;
  UINTN       Length;
  UINT16      Mps;

  X    = In ? &Dev->In[Ep] : &Dev->Out[Ep];
  Phys = PHYS_EP (Ep, In);

  if (X->Started) {
    return EFI_NOT_READY;
  }

  if (Ep == 0) {
    //
    // Status acknowledgement of a two-stage request by its owner.
    //
    if ((Dev->Ep0State == Ep0StateWaitStatus) && !Dev->Ep0ThreeStage && In &&
        (X->Length == 0)) {
      X->Active = FALSE;
      X->Done   = 0;
      if (!X->Internal) {
        UsbFnQueueTransfer (Dev, 0, TRUE);
      }
      CiEp0StatusReady (Dev);
      return EFI_SUCCESS;
    }

    if ((Dev->Ep0State != Ep0StateData) || (In != Dev->Ep0DirIn)) {
      return EFI_NOT_READY;
    }

    Length = MIN (X->Length, (UINTN)USBFN_EP0_BUFFER);
    if (In) {
      CopyMem (Dev->Bounce[Phys], X->Buffer, Length);
      WriteBackDataCacheRange (Dev->Bounce[Phys], USBFN_EP0_BUFFER);
    } else {
      Length = ALIGN_VALUE (MAX (Length, (UINTN)1), USBFN_EP0_MPS);
      Length = MIN (Length, (UINTN)USBFN_EP0_BUFFER);
      InvalidateDataCacheRange (Dev->Bounce[Phys], USBFN_EP0_BUFFER);
    }
    return PrimeTd (Dev, Phys, Dev->Bounce[Phys], Length);
  }

  if (!Dev->EpsConfigured) {
    return EFI_NOT_READY;
  }

  Mps    = (Dev->EpMps[Ep] != 0) ? Dev->EpMps[Ep] : 512;
  Length = MIN (X->Length, (UINTN)USBFN_MAX_TRANSFER);
  if (In) {
    CopyMem (Dev->Bounce[Phys], X->Buffer, Length);
    WriteBackDataCacheRange (Dev->Bounce[Phys], MAX (Length, (UINTN)1));
  } else {
    Length = ALIGN_VALUE (MAX (Length, (UINTN)1), Mps);
    Length = MIN (Length, (UINTN)USBFN_MAX_TRANSFER);
    InvalidateDataCacheRange (Dev->Bounce[Phys], Length);
  }
  return PrimeTd (Dev, Phys, Dev->Bounce[Phys], Length);
}

STATIC VOID
EpComplete (
  IN USBFN_DEV  *Dev,
  IN UINT8      Phys
  )
{
  USBFN_XFER  *X;
  UINTN       Done;
  BOOLEAN     In;

  X  = XferOf (Dev, Phys);
  In = (Phys & 1) != 0;
  if (!X->Started) {
    return;
  }
  Done = TdDone (Dev, Phys);
  if (Done == MAX_UINTN) {
    return;
  }
  X->Started = FALSE;

  if (!In) {
    Done = MIN (Done, X->Length);
    InvalidateDataCacheRange (Dev->Bounce[Phys], X->Programmed);
    CopyMem (X->Buffer, Dev->Bounce[Phys], Done);
  }
  X->Done = Done;

  if (X->Active) {
    X->Active = FALSE;
    UsbFnQueueTransfer (Dev, Phys >> 1, In);
  }
}

VOID
CiAbortEp (
  IN USBFN_DEV  *Dev,
  IN UINT8      Ep,
  IN BOOLEAN    In
  )
{
  UINT8  Phys;

  Phys = PHYS_EP (Ep, In);
  Flush (EptBit (Phys));
  XferOf (Dev, Phys)->Started = FALSE;
}

VOID
CiSetStall (
  IN USBFN_DEV  *Dev,
  IN UINT8      Ep,
  IN BOOLEAN    In,
  IN BOOLEAN    Stall
  )
{
  UINT32  Ctrl;

  if (Ep == 0) {
    if (Stall) {
      CiEp0Stall (Dev);
    }
    return;
  }

  Ctrl = MmioRead32 (REG (CI_ENDPTCTRL (Ep)));
  if (In) {
    Ctrl = Stall ? (Ctrl | CTRL_TXS) : ((Ctrl & ~CTRL_TXS) | CTRL_TXR);
  } else {
    Ctrl = Stall ? (Ctrl | CTRL_RXS) : ((Ctrl & ~CTRL_RXS) | CTRL_RXR);
  }
  MmioWrite32 (REG (CI_ENDPTCTRL (Ep)), Ctrl);
}

BOOLEAN
CiGetStall (
  IN USBFN_DEV  *Dev,
  IN UINT8      Ep,
  IN BOOLEAN    In
  )
{
  UINT32  Ctrl;

  Ctrl = MmioRead32 (REG (CI_ENDPTCTRL (Ep)));
  return (Ctrl & (In ? CTRL_TXS : CTRL_RXS)) != 0;
}

VOID
CiSetAddress (
  IN UINT8  Address
  )
{
  //
  // msm72k_udc: the address applies once the status IN of SET_ADDRESS is out.
  //
  MmioWrite32 (REG (CI_DEVICEADDR), DEVICEADDR (Address) | DEVICEADDR_ADV);
}

/**
  SET_CONFIGURATION: queue heads + ENDPTCTRL for the bulk endpoints of the
  configuration (config_ept / usb_ept_enable).
**/
VOID
CiActivateEndpoints (
  IN USBFN_DEV  *Dev
  )
{
  UINT8   Ep;
  UINT32  Ctrl;
  UINT16  Mps;

  for (Ep = 1; Ep < USBFN_MAX_EPS; Ep++) {
    if (Dev->EpDirs[Ep] == 0) {
      continue;
    }
    Mps  = (Dev->EpMps[Ep] != 0) ? Dev->EpMps[Ep] : 512;
    Ctrl = 0;
    if (Dev->EpDirs[Ep] & EP_DIR_OUT) {
      ZeroMem (&Dev->Qh[PHYS_EP (Ep, FALSE)], sizeof (CI_QH));
      Dev->Qh[PHYS_EP (Ep, FALSE)].Config = QH_MAX_PKT (Mps) | QH_ZLT;
      Dev->Qh[PHYS_EP (Ep, FALSE)].Next   = TD_TERMINATE;
      Ctrl |= ((UINT32)Dev->EpType[Ep] << CTRL_RXT_SHIFT) | CTRL_RXE | CTRL_RXR;
    }
    if (Dev->EpDirs[Ep] & EP_DIR_IN) {
      ZeroMem (&Dev->Qh[PHYS_EP (Ep, TRUE)], sizeof (CI_QH));
      Dev->Qh[PHYS_EP (Ep, TRUE)].Config = QH_MAX_PKT (Mps) | QH_ZLT;
      Dev->Qh[PHYS_EP (Ep, TRUE)].Next   = TD_TERMINATE;
      Ctrl |= ((UINT32)Dev->EpType[Ep] << CTRL_TXT_SHIFT) | CTRL_TXE | CTRL_TXR;
    } else {
      Ctrl |= (USB_ENDPOINT_BULK << CTRL_TXT_SHIFT);   // unused half must not be control
    }
    if (!(Dev->EpDirs[Ep] & EP_DIR_OUT)) {
      Ctrl |= (USB_ENDPOINT_BULK << CTRL_RXT_SHIFT);
    }
    WriteBackDataCacheRange (&Dev->Qh[PHYS_EP (Ep, FALSE)], 2 * sizeof (CI_QH));
    ArmDataSynchronizationBarrier ();
    MmioWrite32 (REG (CI_ENDPTCTRL (Ep)), Ctrl);
    ZeroMem (&Dev->In[Ep], sizeof (Dev->In[Ep]));
    ZeroMem (&Dev->Out[Ep], sizeof (Dev->Out[Ep]));
  }
  Dev->EpsConfigured = TRUE;
}

STATIC VOID
DeactivateEndpoints (
  IN USBFN_DEV  *Dev
  )
{
  UINT8  Ep;

  Flush (0xFFFFFFFFU & ~(EPT_RX (0) | EPT_TX (0)));
  for (Ep = 1; Ep < USBFN_MAX_EPS; Ep++) {
    MmioWrite32 (REG (CI_ENDPTCTRL (Ep)), 0);
    ZeroMem (&Dev->In[Ep], sizeof (Dev->In[Ep]));
    ZeroMem (&Dev->Out[Ep], sizeof (Dev->Out[Ep]));
  }
  Dev->EpsConfigured = FALSE;
}

//
// ---------------------------------------------------------------------------
// Events (usb_interrupt).
// ---------------------------------------------------------------------------
//

STATIC VOID
OnBusReset (
  IN USBFN_DEV  *Dev
  )
{
  MmioWrite32 (REG (CI_ENDPTSETUPSTAT), MmioRead32 (REG (CI_ENDPTSETUPSTAT)));
  MmioWrite32 (REG (CI_ENDPTCOMPLETE), MmioRead32 (REG (CI_ENDPTCOMPLETE)));
  WaitClear32 (REG (CI_ENDPTPRIME), 0xFFFFFFFFU, 100000);
  DeactivateEndpoints (Dev);
  Ep0Reset (Dev);
  MmioWrite32 (REG (CI_DEVICEADDR), 0);
  Dev->Configuration = 0;
  Dev->Speed         = UsbBusSpeedUnknown;
  UsbFnQueueEvent (Dev, EfiUsbMsgBusEventReset, NULL);
}

STATIC VOID
OnPortChange (
  IN USBFN_DEV  *Dev
  )
{
  EFI_USBFN_MESSAGE_PAYLOAD  Payload;
  EFI_USB_BUS_SPEED          Speed;

  Speed = (PORTSC_PSPD (MmioRead32 (REG (CI_PORTSC))) == PSPD_HIGH) ? UsbBusSpeedHigh : UsbBusSpeedFull;
  if (Speed == Dev->Speed) {
    return;   // a resume, not the end of a reset
  }
  Dev->Speed = Speed;
  DEBUG ((DEBUG_ERROR, "UsbFn: connected, %a speed\n", (Speed == UsbBusSpeedHigh) ? "high" : "full"));
  ZeroMem (&Payload, sizeof (Payload));
  Payload.ubs = Speed;
  UsbFnQueueEvent (Dev, EfiUsbMsgBusEventSpeed, &Payload);
}

BOOLEAN
CiEventPending (
  VOID
  )
{
  return ((MmioRead32 (REG (CI_USBSTS)) & STS_EVENTS) != 0) ||
         (MmioRead32 (REG (CI_ENDPTSETUPSTAT)) != 0) ||
         (MmioRead32 (REG (CI_ENDPTCOMPLETE)) != 0);
}

VOID
CiServiceEvents (
  IN USBFN_DEV  *Dev
  )
{
  UINT32  Sts;
  UINT32  Complete;
  UINT8   Bit;
  UINT8   Phys;

  Sts = MmioRead32 (REG (CI_USBSTS));
  MmioWrite32 (REG (CI_USBSTS), Sts & STS_EVENTS);

  if (Sts & STS_URI) {
    OnBusReset (Dev);
  }
  if (Sts & STS_PCI) {
    OnPortChange (Dev);
  }
  if (Sts & STS_SLI) {
    UsbFnQueueEvent (Dev, EfiUsbMsgBusEventSuspend, NULL);
  }

  //
  // Completions first: a finished status stage must not be taken for part
  // of the request that follows it.
  //
  Complete = MmioRead32 (REG (CI_ENDPTCOMPLETE));
  MmioWrite32 (REG (CI_ENDPTCOMPLETE), Complete);
  for (Bit = 0; Bit < 32; Bit++) {
    if ((Complete & (1U << Bit)) == 0) {
      continue;
    }
    Phys = (Bit < 16) ? PHYS_EP (Bit, FALSE) : PHYS_EP (Bit - 16, TRUE);
    if ((Phys >> 1) >= USBFN_MAX_EPS) {
      continue;
    }
    if ((Phys >> 1) == 0) {
      Ep0Complete (Dev, Phys);
    } else {
      EpComplete (Dev, Phys);
    }
  }

  if (MmioRead32 (REG (CI_ENDPTSETUPSTAT)) & EPT_RX (0)) {
    Ep0Setup (Dev);
  }
}

//
// ---------------------------------------------------------------------------
// Bring-up (otg_reset + usb_reset).
// ---------------------------------------------------------------------------
//

EFI_STATUS
CiCoreInit (
  IN USBFN_DEV  *Dev
  )
{
  STATIC CONST UINT8  PhyInit[] = CI_PHY_INIT_SEQ;
  UINTN               Index;

  //
  // Clocks and PHY reset through the modem (clock regime / proc_comm).
  //
  pcom_clock_enable (PCOM_USB_HS_PCLK);
  pcom_clock_enable (PCOM_USB_HS_CLK);
  msm_proc_comm (PCOM_MSM_HSUSB_PHY_RESET, NULL, NULL);
  MicroSecondDelay (1000);

  //
  // Link reset, ULPI PHY, AHB (otg_reset).
  //
  MmioWrite32 (REG (CI_USBCMD), USBCMD_RST);
  if (!WaitClear32 (REG (CI_USBCMD), USBCMD_RST, 100000)) {
    DEBUG ((DEBUG_ERROR, "UsbFn: link reset timeout\n"));
    return EFI_DEVICE_ERROR;
  }
  MmioWrite32 (REG (CI_PORTSC), PORTSC_PTS_ULPI);
  MmioWrite32 (REG (CI_AHB_BURST), 0);
  MmioWrite32 (REG (CI_AHB_MODE), 0);
  MmioWrite32 (REG (CI_USBMODE), USBMODE_SDIS | USBMODE_DEVICE);

  //
  // usb_reset: interrupt threshold 0, PHY init sequence, queue heads.
  //
  MmioAnd32 (REG (CI_USBCMD), ~USBCMD_ITC_MASK);
  for (Index = 0; Index + 1 < sizeof (PhyInit); Index += 2) {
    UlpiWrite (PhyInit[Index], PhyInit[Index + 1]);
  }

  ZeroMem (Dev->Qh, CI_QH_COUNT * sizeof (CI_QH));
  Dev->Qh[PHYS_EP (0, FALSE)].Config = QH_MAX_PKT (USBFN_EP0_MPS) | QH_ZLT | QH_IOS;
  Dev->Qh[PHYS_EP (0, FALSE)].Next   = TD_TERMINATE;
  Dev->Qh[PHYS_EP (0, TRUE)].Config  = QH_MAX_PKT (USBFN_EP0_MPS) | QH_ZLT;
  Dev->Qh[PHYS_EP (0, TRUE)].Next    = TD_TERMINATE;
  WriteBackDataCacheRange (Dev->Qh, CI_QH_COUNT * sizeof (CI_QH));
  ArmDataSynchronizationBarrier ();
  MmioWrite32 (REG (CI_ENDPOINTLISTADDR), PoolToDma (Dev->Qh));

  MmioWrite32 (REG (CI_USBSTS), MmioRead32 (REG (CI_USBSTS)));
  MmioWrite32 (REG (CI_ENDPTSETUPSTAT), MmioRead32 (REG (CI_ENDPTSETUPSTAT)));
  MmioWrite32 (REG (CI_ENDPTCOMPLETE), MmioRead32 (REG (CI_ENDPTCOMPLETE)));

  ZeroMem (Dev->In, sizeof (Dev->In));
  ZeroMem (Dev->Out, sizeof (Dev->Out));
  Dev->EpsConfigured  = FALSE;
  Dev->Configuration  = 0;
  Dev->Speed          = UsbBusSpeedUnknown;
  Dev->Ep0State       = Ep0StateSetup;
  Dev->Ep0StatusReady = FALSE;

  MmioWrite32 (REG (CI_USBINTR), CI_USBINTR_MASK);

  DEBUG ((DEBUG_ERROR, "UsbFn: ChipIdea up, ID %08x, DCIVERSION %04x\n",
          MmioRead32 (REG (0)), MmioRead16 (REG (0x120))));
  return EFI_SUCCESS;
}

/**
  D+ pull-up on/off (msm72k_pullup: USBCMD.RS).
**/
VOID
CiConnect (
  IN USBFN_DEV  *Dev,
  IN BOOLEAN    Connect
  )
{
  if (Connect) {
    MmioOr32 (REG (CI_USBCMD), USBCMD_RS);
  } else {
    MmioAnd32 (REG (CI_USBCMD), ~USBCMD_RS);
  }
}

VOID
CiShutdown (
  IN USBFN_DEV  *Dev
  )
{
  CiConnect (Dev, FALSE);
  MicroSecondDelay (10000);   // let the host see the disconnect
  Flush (0xFFFFFFFFU);
  MmioWrite32 (REG (CI_USBINTR), 0);
  Dev->Ep0State = Ep0StateIdle;
}
