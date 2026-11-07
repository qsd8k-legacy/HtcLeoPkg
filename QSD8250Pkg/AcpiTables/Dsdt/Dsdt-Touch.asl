    //
    // Touchscreen (type 2A at I2C 0x15): htcleots drives the QSD8250 I2C
    // controller (0xA9900000) and the data-ready GPIO 92 (TLMM) itself.
    //
    Device (TSC0)
    {
        Name (_HID, "HTCL0003")
        Name (_UID, 1)
        Method (_STA) { Return (0xF) }
        // data-ready: TLMM GPIO group 1 -> SIRC 9 -> GSI 96 + 9 (halextqcvic)
        Name (_CRS, ResourceTemplate () {
            Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 105 }
        })
    }
