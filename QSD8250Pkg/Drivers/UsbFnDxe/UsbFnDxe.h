/** @file
  QSD8250 (HTC HD2) USB peripheral driver: EFI_USBFN_IO_PROTOCOL on the
  MSM high speed USB controller (ChipIdea core, "msm72k", 0xA0800000).

  Register sequences from the htcleo Linux kernel (drivers/usb/gadget/
  msm72k_udc.c, drivers/usb/otg/msm72k_otg.c otg_reset, board-htcleo.c
  htcleo_phy_init_seq), not from cLK.

  Peripheral mode only, high speed.  Interrupt driven: the controller
  interrupt (VIC INT_USB_HS = 32 + 26, level, USBSTS & USBINTR) services
  USBSTS, ENDPTSETUPSTAT and ENDPTCOMPLETE, answers the EP0 standard
  requests the driver owns and queues UsbFn messages; EventHandler() only
  dequeues them (and services a pending event itself, so a lost interrupt
  cannot wedge a polling client).

  Data moves by DMA through one transfer descriptor (dTD) per endpoint and
  direction and a bounce buffer each.  The queue heads, dTDs and buffers
  live in one pool that is mapped uncached when the CPU driver allows it;
  cache maintenance is done anyway, so a cached pool is still correct.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef __USBFN_DXE_H__
#define __USBFN_DXE_H__

#include <Uefi.h>
#include <IndustryStandard/Usb.h>
#include <Library/ArmLib.h>
#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/CacheMaintenanceLib.h>
#include <Library/DebugLib.h>
#include <Library/DxeServicesTableLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/TimerLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Protocol/HardwareInterrupt.h>
#include <Protocol/UsbFunctionIo.h>

//
// Controller (msm_hsusb_hw.h; MSM_HSUSB_PHYS = 0xA0800000 on QSD8x50).
//
#define CI_BASE               0xA0800000
#define CI_IRQ                (32 + 26)    // irqs-8x50.h INT_USB_HS

#define CI_AHB_BURST          0x0090
#define CI_AHB_MODE           0x0098
#define CI_USBCMD             0x0140
#define CI_USBSTS             0x0144
#define CI_USBINTR            0x0148
#define CI_DEVICEADDR         0x0154
#define CI_ENDPOINTLISTADDR   0x0158
#define CI_ULPI_VIEWPORT      0x0170
#define CI_PORTSC             0x0184
#define CI_OTGSC              0x01A4
#define CI_USBMODE            0x01A8
#define CI_ENDPTSETUPSTAT     0x01AC
#define CI_ENDPTPRIME         0x01B0
#define CI_ENDPTFLUSH         0x01B4
#define CI_ENDPTSTAT          0x01B8
#define CI_ENDPTCOMPLETE      0x01BC
#define CI_ENDPTCTRL(n)       (0x01C0 + 4 * (n))

#define USBCMD_RS             (1U << 0)
#define USBCMD_RST            (1U << 1)
#define USBCMD_SUTW           (1U << 13)   // setup tripwire
#define USBCMD_ITC_MASK       (0xFFU << 16)

#define STS_UI                (1U << 0)    // transfer complete
#define STS_UEI               (1U << 1)    // USB error
#define STS_PCI               (1U << 2)    // port change (reset done, resume)
#define STS_URI               (1U << 6)    // bus reset
#define STS_SLI               (1U << 8)    // suspend
#define STS_EVENTS            (STS_UI | STS_UEI | STS_PCI | STS_URI | STS_SLI)
#define CI_USBINTR_MASK       (STS_URI | STS_SLI | STS_UI | STS_PCI)   // usb_reset()

#define DEVICEADDR_ADV        (1U << 24)   // apply after the status IN
#define DEVICEADDR(a)         ((UINT32)(a) << 25)

#define PORTSC_PTS_ULPI       (2U << 30)
#define PORTSC_PSPD(x)        (((x) >> 26) & 0x3)
#define PSPD_FULL             0
#define PSPD_HIGH             2

#define USBMODE_DEVICE        2U
#define USBMODE_SDIS          (1U << 4)    // stream disable

#define ULPI_RUN              (1U << 30)
#define ULPI_WRITE            (1U << 29)
#define ULPI_ADDR(r)          ((UINT32)(r) << 16)
#define ULPI_DATA(v)          ((UINT32)(v) & 0xFF)

#define EPT_RX(n)             (1U << (n))
#define EPT_TX(n)             (1U << ((n) + 16))

#define CTRL_TXE              (1U << 23)
#define CTRL_TXR              (1U << 22)   // data toggle reset
#define CTRL_TXS              (1U << 16)   // stall
#define CTRL_TXT_SHIFT        18
#define CTRL_RXE              (1U << 7)
#define CTRL_RXR              (1U << 6)
#define CTRL_RXS              (1U << 0)
#define CTRL_RXT_SHIFT        2

//
// Queue head (64 bytes; 32 of them, OUT then IN per endpoint number, at
// ENDPOINTLISTADDR, 2 KiB aligned) and transfer descriptor (32 bytes).
//
typedef struct {
  UINT32  Config;
  UINT32  Current;
  UINT32  Next;         // overlay: next dTD
  UINT32  Token;        // overlay: status
  UINT32  Page[5];
  UINT32  Reserved0;
  UINT8   Setup[8];
  UINT32  Reserved1[4];
} CI_QH;

typedef struct {
  UINT32  Next;
  UINT32  Token;
  UINT32  Page[5];
  UINT32  Reserved;
} CI_TD;

#define QH_MAX_PKT(n)         ((UINT32)(n) << 16)
#define QH_ZLT                (1U << 29)   // no automatic zero-length packet
#define QH_IOS                (1U << 15)   // interrupt on setup

#define TD_TERMINATE          1U
#define TD_BYTES(n)           ((UINT32)(n) << 16)
#define TD_BYTES_LEFT(t)      (((t) >> 16) & 0x7FFF)
#define TD_IOC                (1U << 15)
#define TD_ACTIVE             (1U << 7)
#define TD_HALTED             (1U << 6)
#define TD_BUFFER_ERROR       (1U << 5)
#define TD_TXN_ERROR          (1U << 3)
#define TD_ERRORS             (TD_HALTED | TD_BUFFER_ERROR | TD_TXN_ERROR)

//
// htcleo_phy_init_seq (board-htcleo.c): ULPI {value, register} pairs.
//
#define CI_PHY_INIT_SEQ       { 0x0C, 0x31, 0x30, 0x32, 0x1D, 0x0D, 0x1D, 0x10 }

//
// Bring-up self test: off, UsbLogDxe is the client.
//
#define USBFN_MAX_EPS          4      // EP0 + bulk endpoint numbers 1..3
#define USBFN_EP0_MPS          64
#define USBFN_MAX_TRANSFER     (16 * 1024)    // one dTD from a page-aligned buffer
#define USBFN_EVENT_QUEUE      32
#define USBFN_EP0_BUFFER       512

#define USBFN_VID              0x1209
#define USBFN_PID              0x0001

//
// Pool layout (one allocation, page aligned).
//
#define CI_QH_COUNT            32
#define CI_PHYS_EPS            (USBFN_MAX_EPS * 2)
#define CI_TD_SLOT             64
#define CI_POOL_TD_OFFSET      0x0800
#define CI_POOL_EP0_OFFSET     0x1000
#define CI_POOL_BULK_OFFSET    0x2000
#define CI_POOL_SIZE           (CI_POOL_BULK_OFFSET + (CI_PHYS_EPS - 2) * USBFN_MAX_TRANSFER)

//
// EP0 control transfer state.
//
typedef enum {
  Ep0StateIdle = 0,         // before start / after stop
  Ep0StateSetup,            // waiting for a SETUP
  Ep0StateData,             // data stage expected / in flight
  Ep0StateWaitStatus,       // waiting for the request to be answered
  Ep0StateStatus            // status ZLP in flight
} EP0_STATE;

//
// One transfer per endpoint and direction.
//
typedef struct {
  BOOLEAN    Active;      // client transfer owned by this endpoint
  BOOLEAN    Internal;    // EP0 data stage owned by the driver, not reported
  BOOLEAN    SendZlp;     // EP0 IN: terminate a max-packet-multiple reply
  BOOLEAN    ZlpSent;     // that ZLP went out
  BOOLEAN    Started;     // dTD primed
  UINT8      *Buffer;     // client buffer
  UINTN      Length;      // client length
  UINTN      Programmed;  // bytes in the dTD
  UINTN      Done;        // bytes actually moved
} USBFN_XFER;

typedef struct {
  EFI_USBFN_MESSAGE          Message;
  EFI_USBFN_MESSAGE_PAYLOAD  Payload;
} USBFN_EVENT;

typedef struct {
  UINT32                    Signature;
  EFI_USBFN_IO_PROTOCOL     UsbFnIo;
  EFI_HANDLE                Handle;
  EFI_HARDWARE_INTERRUPT_PROTOCOL *Interrupt;
  EFI_EVENT                 ExitBootServices;

  BOOLEAN                   Started;
  EFI_USB_BUS_SPEED         Speed;
  UINT8                     Configuration;
  BOOLEAN                   EpsConfigured;

  EFI_USB_DEVICE_INFO       *DeviceInfo;
  UINT8                     *ConfigBlob;
  UINT16                    ConfigBlobLength;

  USBFN_XFER                In[USBFN_MAX_EPS];
  USBFN_XFER                Out[USBFN_MAX_EPS];
  UINT8                     EpType[USBFN_MAX_EPS];
  UINT16                    EpMps[USBFN_MAX_EPS];
  UINT8                     EpDirs[USBFN_MAX_EPS];

  //
  // EP0 control state machine.
  //
  EP0_STATE                 Ep0State;
  BOOLEAN                   Ep0ThreeStage;    // wLength != 0
  BOOLEAN                   Ep0DirIn;         // data stage direction
  BOOLEAN                   Ep0StatusReady;   // status may be sent
  UINT8                     Ep0Buffer[USBFN_EP0_BUFFER];

  //
  // DMA pool.
  //
  UINT8                     *Pool;
  UINTN                     PoolPages;
  CI_QH                     *Qh;                     // 32 queue heads
  CI_TD                     *Td[CI_PHYS_EPS];
  UINT8                     *Bounce[CI_PHYS_EPS];

  USBFN_EVENT               Queue[USBFN_EVENT_QUEUE];
  UINTN                     QueueHead;
  UINTN                     QueueTail;
  UINTN                     QueueDropped;

  UINTN                     IrqCount;         // controller interrupts taken
  UINTN                     PollCount;
} USBFN_DEV;

#define USBFN_SIGNATURE    SIGNATURE_32 ('U', 'F', 'N', 'D')
#define USBFN_FROM_THIS(a) CR (a, USBFN_DEV, UsbFnIo, USBFN_SIGNATURE)

#define PHYS_EP(Ep, In)    ((UINT8)(((Ep) << 1) | ((In) ? 1 : 0)))
#define EP_DIR_OUT         (1U << 0)
#define EP_DIR_IN          (1U << 1)

//
// ChipIdea.c
//
EFI_STATUS  CiAllocatePool (IN USBFN_DEV *Dev);
EFI_STATUS  CiCoreInit (IN USBFN_DEV *Dev);
VOID        CiConnect (IN USBFN_DEV *Dev, IN BOOLEAN Connect);
VOID        CiShutdown (IN USBFN_DEV *Dev);
BOOLEAN     CiEventPending (VOID);
VOID        CiServiceEvents (IN USBFN_DEV *Dev);
VOID        CiSetAddress (IN UINT8 Address);
VOID        CiActivateEndpoints (IN USBFN_DEV *Dev);
EFI_STATUS  CiStartTransfer (IN USBFN_DEV *Dev, IN UINT8 Ep, IN BOOLEAN In);
VOID        CiSetStall (IN USBFN_DEV *Dev, IN UINT8 Ep, IN BOOLEAN In, IN BOOLEAN Stall);
BOOLEAN     CiGetStall (IN USBFN_DEV *Dev, IN UINT8 Ep, IN BOOLEAN In);
VOID        CiEp0Stall (IN USBFN_DEV *Dev);
VOID        CiEp0StatusReady (IN USBFN_DEV *Dev);
VOID        CiAbortEp (IN USBFN_DEV *Dev, IN UINT8 Ep, IN BOOLEAN In);

//
// UsbFnDxe.c: callbacks from the event handler.
//
VOID
UsbFnQueueEvent (
  IN USBFN_DEV                        *Dev,
  IN EFI_USBFN_MESSAGE                Message,
  IN CONST EFI_USBFN_MESSAGE_PAYLOAD  *Payload OPTIONAL
  );

VOID
UsbFnQueueTransfer (
  IN USBFN_DEV  *Dev,
  IN UINT8      Ep,
  IN BOOLEAN    In
  );

VOID
UsbFnEp0Setup (
  IN USBFN_DEV                     *Dev,
  IN CONST EFI_USB_DEVICE_REQUEST  *Req
  );

#endif // __USBFN_DXE_H__
