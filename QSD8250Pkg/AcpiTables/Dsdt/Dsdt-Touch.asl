    //
    // Touchscreen (type 2A at I2C 0x15): htcleots drives the QSD8250 I2C
    // controller (0xA9900000) and the data-ready GPIO 92 (TLMM) itself.
    //
    Device (TSC0)
    {
        Name (_HID, "HTCL0003")
        Name (_UID, 1)
        Method (_STA) { Return (0xF) }
        // No _CRS: with Interrupt (..) { 105 } (TLMM GPIO group 1 -> SIRC 9 ->
        // GSI 96 + 9, halextqcvic) Windows never starts the device - the
        // stack gets FILTER_RESOURCE_REQUIREMENTS but no START (2026-11-08).
        // htcleots then polls the data-ready line on a timer.
    }
