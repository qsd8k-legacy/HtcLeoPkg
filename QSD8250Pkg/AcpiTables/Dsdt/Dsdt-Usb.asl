    //
    // HS-USB controller (ChipIdea "msm72k", MSM_HSUSB_PHYS) in peripheral
    // mode: msmusbfn presents the USB log + debug device (1209:0001, same
    // as UEFI's UsbLogDxe) under Windows.  Interrupt: VIC INT_USB_HS
    // (32 + 26) -> GSI 32 + 58 = 90 (halextqcvic: GSI = 32 + VIC source).
    //
    Device (UFN0)
    {
        Name (_HID, "HTCL0005")
        Name (_UID, 1)
        Method (_STA) { Return (0xF) }
        Name (_CRS, ResourceTemplate () {
            Memory32Fixed (ReadWrite, 0xA0800000, 0x00001000)
            Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 90 }
        })
    }
