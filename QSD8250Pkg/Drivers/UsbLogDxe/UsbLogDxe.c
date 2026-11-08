/** @file
  USB log + debug channel (EFI_USBFN_IO_PROTOCOL client).

  Vendor device 1209:0001 with two interfaces, so a log reader and a debug
  tool can hold them at the same time:

  - Interface 0, bulk IN 0x81: the boot log ring (LogRing.h), every
    DEBUG() byte since PrePi entry, then new output as it appears.  The read
    position survives bus resets (up to one ring of history).
  - Interface 1, bulk OUT 0x02 / IN 0x82: commands (USBDBG_*): ping, memory
    read, 32-bit write, reset into normal / download / recovery mode.
    Host side: Tools/usbdbg.py.  File transfer to/from the FAT volumes
    (USBDBG_VOLUMES / FOPEN / FWRITE / FREAD / FCLOSE) rides on the same
    channel: requests may carry up to 4 KiB of payload after the header.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <Uefi.h>
#include <IndustryStandard/Usb.h>
#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Library/DevicePathLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PrintLib.h>
#include <Guid/FileInfo.h>
#include <Protocol/SimpleFileSystem.h>
#include <Protocol/UsbFunctionIo.h>

#include <LogRing.h>

#define USBLOG_VID         0x1209
#define USBLOG_PID         0x0001
#define USBLOG_CHUNK       4096
#define USBLOG_POLL_100NS  (10 * 10000)   // 10 ms

//
// Debug channel wire format (little endian).  Request: one 16-byte header.
// Reply: a 16-byte header, then Length bytes of data (USBDBG_READ only).
//
#define USBDBG_REQ_MAGIC   SIGNATURE_32 ('U', 'D', 'B', 'G')
#define USBDBG_RSP_MAGIC   SIGNATURE_32 ('U', 'D', 'B', 'R')
#define USBDBG_MAX_READ    4096

#define USBDBG_PING        1   // -> "PONG"
#define USBDBG_READ        2   // Arg0 address, Arg1 length (<= 4096)
#define USBDBG_WRITE32     3   // Arg0 address, Arg1 value
#define USBDBG_RESET       4   // Arg0: 0 normal, 1 download, 4 recovery
#define USBDBG_VOLUMES     5   // -> "index: device path" lines
#define USBDBG_FOPEN       6   // Arg0 volume | mode << 8 (1 = write), payload ASCII path
#define USBDBG_FWRITE      7   // payload appended to the open file
#define USBDBG_FREAD       8   // Arg0 length (<= 4096) -> next bytes (short = EOF)
#define USBDBG_FCLOSE      9   // flush + close
#define USBDBG_MAX_PAYLOAD 4096

#define USBDBG_OK          0
#define USBDBG_EBADCMD     1
#define USBDBG_EBADARG     2
#define USBDBG_EFILE       3   // + EFI status in Reply.Reserved

typedef struct {
  UINT32  Magic;
  UINT32  Command;
  UINT32  Arg0;
  UINT32  Arg1;
} USBDBG_HEADER;

typedef struct {
  UINT32  Magic;
  UINT32  Status;
  UINT32  Length;
  UINT32  Reserved;
} USBDBG_REPLY;

STATIC EFI_USB_DEVICE_DESCRIPTOR  mDevice = {
  sizeof (EFI_USB_DEVICE_DESCRIPTOR), USB_DESC_TYPE_DEVICE,
  0x0200, 0xFF, 0x00, 0x00, 64,
  USBLOG_VID, USBLOG_PID, 0x0100,
  1, 2, 3, 1
};

STATIC EFI_USB_CONFIG_DESCRIPTOR  mConfig = {
  sizeof (EFI_USB_CONFIG_DESCRIPTOR), USB_DESC_TYPE_CONFIG,
  0, 2, 1, 0, 0x80, 250
};

STATIC EFI_USB_INTERFACE_DESCRIPTOR  mInterface = {
  sizeof (EFI_USB_INTERFACE_DESCRIPTOR), USB_DESC_TYPE_INTERFACE,
  0, 0, 1, 0xFF, 0x00, 0x00, 0
};

STATIC EFI_USB_ENDPOINT_DESCRIPTOR  mEpIn = {
  sizeof (EFI_USB_ENDPOINT_DESCRIPTOR), USB_DESC_TYPE_ENDPOINT,
  0x81, USB_ENDPOINT_BULK, 512, 0
};

STATIC EFI_USB_INTERFACE_DESCRIPTOR  mDbgInterface = {
  sizeof (EFI_USB_INTERFACE_DESCRIPTOR), USB_DESC_TYPE_INTERFACE,
  1, 0, 2, 0xFF, 0x00, 0x00, 0
};

STATIC EFI_USB_ENDPOINT_DESCRIPTOR  mDbgEpOut = {
  sizeof (EFI_USB_ENDPOINT_DESCRIPTOR), USB_DESC_TYPE_ENDPOINT,
  0x02, USB_ENDPOINT_BULK, 512, 0
};

STATIC EFI_USB_ENDPOINT_DESCRIPTOR  mDbgEpIn = {
  sizeof (EFI_USB_ENDPOINT_DESCRIPTOR), USB_DESC_TYPE_ENDPOINT,
  0x82, USB_ENDPOINT_BULK, 512, 0
};

STATIC EFI_USB_ENDPOINT_DESCRIPTOR  *mEndpoints[]    = { &mEpIn };
STATIC EFI_USB_ENDPOINT_DESCRIPTOR  *mDbgEndpoints[] = { &mDbgEpOut, &mDbgEpIn };
STATIC EFI_USB_INTERFACE_INFO       mIntfInfo        = { &mInterface, mEndpoints };
STATIC EFI_USB_INTERFACE_INFO       mDbgIntfInfo     = { &mDbgInterface, mDbgEndpoints };
STATIC EFI_USB_INTERFACE_INFO       *mIntfTable[]    = { &mIntfInfo, &mDbgIntfInfo };
STATIC EFI_USB_CONFIG_INFO          mConfigInfo     = { &mConfig, mIntfTable };
STATIC EFI_USB_CONFIG_INFO          *mConfigTable[] = { &mConfigInfo };
STATIC EFI_USB_DEVICE_INFO          mDeviceInfo     = { &mDevice, mConfigTable };

STATIC EFI_USBFN_IO_PROTOCOL  *mIo;
STATIC EFI_EVENT              mPoll;
STATIC BOOLEAN                mTxBusy;
STATIC UINT32                 mReadPos;         // ring position already sent
STATIC UINT32                 mTxEnd;           // ring position after mTx
STATIC UINT8                  mTx[USBLOG_CHUNK];
STATIC UINT8                  mEp0[64];

//
// Debug channel state.
//
#define DBG_EP  2
STATIC BOOLEAN                mDbgRxPosted;
STATIC BOOLEAN                mDbgTxBusy;
STATIC UINT8                  mDbgRx[sizeof (USBDBG_HEADER) + USBDBG_MAX_PAYLOAD];
STATIC EFI_FILE_PROTOCOL      *mDbgFile;            // file open for FWRITE/FREAD
STATIC BOOLEAN                mDbgFileWrite;        // opened for writing
STATIC UINT8                  mDbgTx[sizeof (USBDBG_REPLY) + USBDBG_MAX_READ];
STATIC INT32                  mPendingReset = -1;   // reboot mode once the reply is out

/**
  GET_DESCRIPTOR(STRING) is the client's job; anything else unknown stalls.
**/
STATIC
VOID
UsbLogSetup (
  IN EFI_USB_DEVICE_REQUEST  *Req
  )
{
  CONST CHAR16  *Str;
  UINTN         Size;
  UINTN         Index;
  BOOLEAN       Stall;

  if ((Req->Request != USB_REQ_GET_DESCRIPTOR) ||
      ((Req->Value >> 8) != USB_DESC_TYPE_STRING)) {
    Stall = TRUE;
    mIo->SetEndpointStallState (mIo, 0, EfiUsbEndpointDirectionDeviceTx, &Stall);
    return;
  }

  switch (Req->Value & 0xFF) {
    case 0:  Str = NULL;                 break;   // language ids
    case 1:  Str = L"HTC";           break;
    case 2:  Str = L"HD2 UEFI log"; break;
    default: Str = L"HTC HD2";           break;
  }

  mEp0[1] = USB_DESC_TYPE_STRING;
  if (Str == NULL) {
    mEp0[2] = 0x09;   // en-US
    mEp0[3] = 0x04;
    Size    = 4;
  } else {
    Size = 2;
    for (Index = 0; (Str[Index] != 0) && (Size + 2 <= sizeof (mEp0)); Index++) {
      mEp0[Size++] = (UINT8)Str[Index];
      mEp0[Size++] = (UINT8)(Str[Index] >> 8);
    }
  }
  mEp0[0] = (UINT8)Size;

  Size = MIN (Size, (UINTN)Req->Length);
  mIo->Transfer (mIo, 0, EfiUsbEndpointDirectionDeviceTx, &Size, mEp0);
}

/**
  Queue the next piece of unsent log, if any.
**/
STATIC
VOID
UsbLogKick (
  VOID
  )
{
  UINT32  Head;
  UINT32  Pending;
  UINT32  Index;
  UINTN   Size;

  if (mTxBusy) {
    return;
  }

  Head = LOG_RING->Head;
  if (Head - mReadPos > LOG_RING_DATA_SIZE) {
    mReadPos = Head - LOG_RING_DATA_SIZE;   // fell a whole ring behind
  }

  Pending = MIN (Head - mReadPos, (UINT32)USBLOG_CHUNK);
  if (Pending == 0) {
    return;
  }

  //
  // Copy out of the ring: it keeps moving while the transfer is in flight.
  // Stop one short of a full 512-byte multiple so every transfer ends in a
  // short packet and the host read returns without needing a ZLP.
  //
  if ((Pending % 512) == 0) {
    Pending--;
  }
  for (Index = 0; Index < Pending; Index++) {
    mTx[Index] = LOG_RING->Data[(mReadPos + Index) % LOG_RING_DATA_SIZE];
  }

  //
  // EFI_NOT_READY until the host has configured the device: retried on the
  // next poll.
  //
  Size = Pending;
  if (!EFI_ERROR (mIo->Transfer (mIo, 1, EfiUsbEndpointDirectionDeviceTx, &Size, mTx))) {
    mTxEnd  = mReadPos + Pending;
    mTxBusy = TRUE;
  }
}

/**
  Keep one command receive armed on EP 0x02 (EFI_NOT_READY until the host
  configured the device: retried on the next poll).
**/
STATIC
VOID
DbgPostRx (
  VOID
  )
{
  UINTN  Size;

  if (mDbgRxPosted) {
    return;
  }
  Size = sizeof (mDbgRx);
  if (!EFI_ERROR (mIo->Transfer (mIo, DBG_EP, EfiUsbEndpointDirectionDeviceRx,
                                 &Size, mDbgRx))) {
    mDbgRxPosted = TRUE;
  }
}

/**
  Reboot now (ResetSystem: PlatformResetRuntimeLib, proc_comm reset)
  (PlatformResetRuntimeLib).
**/
STATIC
VOID
DbgReset (
  IN UINT32  Mode
  )
{
  CONST CHAR16  *Name;

  Name = (Mode == 1) ? L"download" : (Mode == 4) ? L"recovery" : NULL;
  DEBUG ((DEBUG_INFO, "UsbDbg: reset (%s)\n", (Name != NULL) ? Name : L"normal"));
  if (Name != NULL) {
    gRT->ResetSystem (EfiResetPlatformSpecific, EFI_SUCCESS, StrSize (Name), (VOID *)Name);
  }
  gRT->ResetSystem (EfiResetCold, EFI_SUCCESS, 0, NULL);
}

/**
  Root directory of the Index-th FAT volume (LocateHandleBuffer order).
**/
STATIC
EFI_STATUS
DbgOpenVolume (
  IN  UINTN              Index,
  OUT EFI_FILE_PROTOCOL  **Root
  )
{
  EFI_STATUS                       Status;
  EFI_HANDLE                       *Handles;
  UINTN                            Count;
  EFI_SIMPLE_FILE_SYSTEM_PROTOCOL  *Fs;

  Status = gBS->LocateHandleBuffer (ByProtocol, &gEfiSimpleFileSystemProtocolGuid,
                                    NULL, &Count, &Handles);
  if (EFI_ERROR (Status)) {
    return Status;
  }
  Status = EFI_NOT_FOUND;
  if (Index < Count) {
    Status = gBS->HandleProtocol (Handles[Index], &gEfiSimpleFileSystemProtocolGuid,
                                  (VOID **)&Fs);
    if (!EFI_ERROR (Status)) {
      Status = Fs->OpenVolume (Fs, Root);
    }
  }
  FreePool (Handles);
  return Status;
}

/**
  "index: device path" per FAT volume into Out (ASCII, NUL-free).
**/
STATIC
UINT32
DbgListVolumes (
  OUT CHAR8  *Out,
  IN  UINTN  OutSize
  )
{
  EFI_HANDLE  *Handles;
  UINTN       Count;
  UINTN       Index;
  UINTN       Used;
  CHAR16      *Text;

  Used = 0;
  if (EFI_ERROR (gBS->LocateHandleBuffer (ByProtocol, &gEfiSimpleFileSystemProtocolGuid,
                                          NULL, &Count, &Handles))) {
    return 0;
  }
  for (Index = 0; (Index < Count) && (Used + 1 < OutSize); Index++) {
    Text  = ConvertDevicePathToText (DevicePathFromHandle (Handles[Index]), TRUE, TRUE);
    Used += AsciiSPrint (Out + Used, OutSize - Used, "%u: %s\n", Index,
                         (Text != NULL) ? Text : L"?");
    if (Text != NULL) {
      FreePool (Text);
    }
  }
  FreePool (Handles);
  return (UINT32)Used;
}

/**
  FOPEN: Arg0 bits 7:0 volume, bit 8 write (create, truncating an existing
  file).  Payload: ASCII path such as \\zImage.
**/
STATIC
EFI_STATUS
DbgFileOpen (
  IN UINT32       Arg0,
  IN CONST UINT8  *Payload,
  IN UINTN        Length
  )
{
  EFI_STATUS         Status;
  EFI_FILE_PROTOCOL  *Root;
  CHAR16             Path[256];
  UINTN              Index;
  BOOLEAN            Write;

  if (mDbgFile != NULL) {
    mDbgFile->Close (mDbgFile);
    mDbgFile = NULL;
  }
  if ((Length == 0) || (Length >= ARRAY_SIZE (Path))) {
    return EFI_INVALID_PARAMETER;
  }
  for (Index = 0; Index < Length; Index++) {
    Path[Index] = Payload[Index];
  }
  Path[Length] = L'\0';
  Write = ((Arg0 >> 8) & 1) != 0;

  Status = DbgOpenVolume (Arg0 & 0xFF, &Root);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  if (Write) {
    //
    // Truncate: delete an existing file, then create it fresh.
    //
    if (!EFI_ERROR (Root->Open (Root, &mDbgFile, Path,
                                EFI_FILE_MODE_READ | EFI_FILE_MODE_WRITE, 0))) {
      mDbgFile->Delete (mDbgFile);
      mDbgFile = NULL;
    }
    Status = Root->Open (Root, &mDbgFile, Path,
                         EFI_FILE_MODE_READ | EFI_FILE_MODE_WRITE | EFI_FILE_MODE_CREATE, 0);
  } else {
    Status = Root->Open (Root, &mDbgFile, Path, EFI_FILE_MODE_READ, 0);
  }
  if (EFI_ERROR (Status)) {
    mDbgFile = NULL;
  }
  mDbgFileWrite = Write;
  Root->Close (Root);
  return Status;
}

/**
  Execute one request from mDbgRx and send its reply on EP 0x82.
**/
STATIC
VOID
DbgHandle (
  IN UINTN  Length
  )
{
  USBDBG_HEADER  Req;
  USBDBG_REPLY   *Rsp;
  UINT8          *Data;
  UINT32         Index;
  UINTN          Size;
  UINT8          *Payload;
  UINTN          PayloadLength;
  EFI_STATUS     Status;

  Rsp  = (USBDBG_REPLY *)mDbgTx;
  Data = mDbgTx + sizeof (USBDBG_REPLY);
  ZeroMem (Rsp, sizeof (*Rsp));
  Rsp->Magic  = USBDBG_RSP_MAGIC;
  Rsp->Status = USBDBG_OK;

  CopyMem (&Req, mDbgRx, MIN (Length, sizeof (Req)));
  Payload       = mDbgRx + sizeof (USBDBG_HEADER);
  PayloadLength = (Length > sizeof (Req)) ? (Length - sizeof (Req)) : 0;
  Status        = EFI_SUCCESS;
  if ((Length < sizeof (Req)) || (Req.Magic != USBDBG_REQ_MAGIC)) {
    Rsp->Status = USBDBG_EBADCMD;
  } else {
    switch (Req.Command) {
      case USBDBG_PING:
        CopyMem (Data, "PONG", 4);
        Rsp->Length = 4;
        break;

      case USBDBG_READ:
        //
        // Aligned reads go word by word so MMIO registers see 32-bit
        // accesses.  An unmapped address aborts like any other bad access.
        //
        if ((Req.Arg1 == 0) || (Req.Arg1 > USBDBG_MAX_READ)) {
          Rsp->Status = USBDBG_EBADARG;
          break;
        }
        if (((Req.Arg0 | Req.Arg1) & 3) == 0) {
          for (Index = 0; Index < Req.Arg1; Index += 4) {
            *(UINT32 *)(Data + Index) = MmioRead32 (Req.Arg0 + Index);
          }
        } else {
          for (Index = 0; Index < Req.Arg1; Index++) {
            Data[Index] = MmioRead8 (Req.Arg0 + Index);
          }
        }
        Rsp->Length = Req.Arg1;
        break;

      case USBDBG_WRITE32:
        if ((Req.Arg0 & 3) != 0) {
          Rsp->Status = USBDBG_EBADARG;
          break;
        }
        MmioWrite32 (Req.Arg0, Req.Arg1);
        break;

      case USBDBG_RESET:
        mPendingReset = (INT32)Req.Arg0;   // after the reply went out
        break;

      case USBDBG_VOLUMES:
        Rsp->Length = DbgListVolumes ((CHAR8 *)Data, USBDBG_MAX_READ);
        break;

      case USBDBG_FOPEN:
        Status = DbgFileOpen (Req.Arg0, Payload, PayloadLength);
        break;

      case USBDBG_FWRITE:
        if (mDbgFile == NULL) {
          Status = EFI_NOT_READY;
          break;
        }
        Size   = PayloadLength;
        Status = mDbgFile->Write (mDbgFile, &Size, Payload);
        if (!EFI_ERROR (Status) && (Size != PayloadLength)) {
          Status = EFI_VOLUME_FULL;
        }
        break;

      case USBDBG_FREAD:
        if ((mDbgFile == NULL) || (Req.Arg0 > USBDBG_MAX_READ)) {
          Status = (mDbgFile == NULL) ? EFI_NOT_READY : EFI_INVALID_PARAMETER;
          break;
        }
        Size   = Req.Arg0;
        Status = mDbgFile->Read (mDbgFile, &Size, Data);
        if (!EFI_ERROR (Status)) {
          //
          // A short reply that is still a whole number of 512-byte packets
          // would not end the host's larger read: hand one byte back.
          //
          if ((Size != 0) && (Size < Req.Arg0) &&
              (((sizeof (USBDBG_REPLY) + Size) % 512) == 0)) {
            UINT64  Pos;

            mDbgFile->GetPosition (mDbgFile, &Pos);
            mDbgFile->SetPosition (mDbgFile, Pos - 1);
            Size--;
          }
          Rsp->Length = (UINT32)Size;
        }
        break;

      case USBDBG_FCLOSE:
        if (mDbgFile != NULL) {
          if (mDbgFileWrite) {
            Status = mDbgFile->Flush (mDbgFile);   // FAT refuses Flush on read-only opens
          }
          mDbgFile->Close (mDbgFile);
          mDbgFile = NULL;
        }
        break;

      default:
        Rsp->Status = USBDBG_EBADCMD;
        break;
    }
  }

  if (EFI_ERROR (Status)) {
    Rsp->Status   = USBDBG_EFILE;
    Rsp->Reserved = (UINT32)Status;
    Rsp->Length   = 0;
  }

  Size   = sizeof (USBDBG_REPLY) + Rsp->Length;
  Status = mIo->Transfer (mIo, DBG_EP, EfiUsbEndpointDirectionDeviceTx, &Size, mDbgTx);
  DEBUG ((DEBUG_ERROR, "UsbDbg: cmd %u -> status %u, %u bytes, transfer %r\n",
          Req.Command, Rsp->Status, (UINT32)Size, Status));
  if (!EFI_ERROR (Status)) {
    mDbgTxBusy = TRUE;
  } else if (mPendingReset >= 0) {
    DbgReset ((UINT32)mPendingReset);
  }
}

STATIC
VOID
EFIAPI
UsbLogPoll (
  IN EFI_EVENT  Event,
  IN VOID       *Context
  )
{
  EFI_USBFN_MESSAGE          Msg;
  EFI_USBFN_MESSAGE_PAYLOAD  Payload;
  UINTN                      Size;

  for ( ; ; ) {
    Size = sizeof (Payload);
    if (EFI_ERROR (mIo->EventHandler (mIo, &Msg, &Size, &Payload)) ||
        (Msg == EfiUsbMsgNone)) {
      break;
    }

    switch (Msg) {
      case EfiUsbMsgBusEventReset:
        //
        // The driver dropped any transfer in flight: resend that piece.
        //
        mTxBusy      = FALSE;
        mDbgTxBusy   = FALSE;
        mDbgRxPosted = FALSE;
        break;

      case EfiUsbMsgSetupPacket:
        UsbLogSetup (&Payload.udr);
        break;

      case EfiUsbMsgEndpointStatusChangedTx:
        if (Payload.utr.EndpointIndex == 1) {
          mReadPos = mTxEnd;
          mTxBusy  = FALSE;
        } else if (Payload.utr.EndpointIndex == DBG_EP) {
          mDbgTxBusy = FALSE;
          if (mPendingReset >= 0) {
            DbgReset ((UINT32)mPendingReset);
          }
        }
        break;

      case EfiUsbMsgEndpointStatusChangedRx:
        if (Payload.utr.EndpointIndex == DBG_EP) {
          mDbgRxPosted = FALSE;
          if (!mDbgTxBusy) {
            DbgHandle (Payload.utr.BytesTransferred);
          } else {
            DEBUG ((DEBUG_ERROR, "UsbDbg: request dropped, reply still in flight\n"));
          }
        }
        break;

      default:
        break;
    }
  }

  DbgPostRx ();
  UsbLogKick ();
}

STATIC
VOID
EFIAPI
UsbLogStart (
  IN EFI_EVENT  Event,
  IN VOID       *Context
  )
{
  EFI_STATUS  Status;

  if (mIo != NULL) {
    return;
  }

  Status = gBS->LocateProtocol (&gEfiUsbFunctionIoProtocolGuid, NULL, (VOID **)&mIo);
  if (EFI_ERROR (Status)) {
    mIo = NULL;
    return;
  }

  Status = mIo->ConfigureEnableEndpoints (mIo, &mDeviceInfo);
  if (!EFI_ERROR (Status)) {
    Status = mIo->StartController (mIo);
  }
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "UsbLog: start failed - %r\n", Status));
    return;
  }

  gBS->CreateEvent (EVT_TIMER | EVT_NOTIFY_SIGNAL, TPL_CALLBACK, UsbLogPoll, NULL, &mPoll);
  gBS->SetTimer (mPoll, TimerPeriodic, USBLOG_POLL_100NS);
  DEBUG ((DEBUG_INFO, "UsbLog: streaming the boot log on %04x:%04x EP 0x81\n",
          USBLOG_VID, USBLOG_PID));
}

EFI_STATUS
EFIAPI
UsbLogDxeInitialize (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  UsbLogStart (NULL, NULL);
  return EFI_SUCCESS;
}
