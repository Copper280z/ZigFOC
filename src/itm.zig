const std = @import("std");
const builtin = @import("builtin");
const microzig = @import("microzig");
const mmio = microzig.mmio;

pub fn enable_itm() void {
    CoreDebug.DEMCR.raw |= 1 << 24;
    ITM.LAR.raw = 0xC5ACCE55;
    ITM.TCR.modify(.{ .SWOENA = 1, .ITMENA = 1 });
    ITM.TER.raw = 0xFFFF;
    ITM.TPR.raw = 0x0;
    TPIU.ACPR.modify(.{ .SWOSCALER = 90 });
    TPIU.SPPR.modify(.{ .TXMODE = 0b01 });
}

pub inline fn ITM_SendChar(ch: u8) void {
    // if ((ITM.TCR.read().ITMENA == 1)) {
    while (ITM.PORT[0].raw == 0) {
        asm volatile ("nop");
    }
    ITM.PORT[0].raw = @as(u32, ch);
    // }
    // return ch;
}

pub const ITM: *volatile types.ITM = @ptrFromInt(0xE0000000);
pub const CoreDebug: *volatile types.CoreDebug = @ptrFromInt(0xE000EDF0);
pub const TPIU: *volatile types.TPIU = @ptrFromInt(0xE0040000);
// Cortex-M4 ROM table components
// 0xE00FF00C ITM val->0xFFF01003
// Reads as 0xFFF01002 if no ITM is implemented.

pub const types = struct {
    pub const TPIU = extern struct {
        SSPSR: mmio.Mmio(packed struct(u32) { //
            SWIDTH: u32,
        }),
        CSPSR: mmio.Mmio(packed struct(u32) { //
            CWIDTH: u32,
        }),
        ACPR: mmio.Mmio(packed struct(u32) {
            reserved0: u16,
            SWOSCALER: u16,
        }),
        SPPR: mmio.Mmio(packed struct(u32) {
            reserved: u30,
            TXMODE: u2,
        }),
        TYPE: mmio.Mmio(packed struct(u32) { //
            reserved: u16,
            impl0: u4,
            NRZVALID: u1,
            MANCVALID: u1,
            PTINVALID: u1,
            FIFOSZ: u3,
            impl1: u6,
        }),
    };
    pub const CoreDebug = extern struct {
        DHCSR: mmio.Mmio(packed struct(u32) {
            DHCSR: u32,
        }),
        DCRSR: mmio.Mmio(packed struct(u32) {
            DCRSR: u32,
        }),
        DCRDR: mmio.Mmio(packed struct(u32) {
            DCRDR: u32,
        }),
        DEMCR: mmio.Mmio(packed struct(u32) {
            DEMCR: u32,
        }),
    };
    pub const ITM = extern struct {
        PORT: [32]mmio.Mmio(packed struct(u32) {
            PORT: u32,
        }),
        reserved0: [864]u32 = undefined,
        TER: mmio.Mmio(packed struct(u32) {
            TER: u32,
        }),
        reserved1: [15]u32 = undefined,
        TPR: mmio.Mmio(packed struct(u32) {
            TPR: u32,
        }),
        reserved2: [15]u32 = undefined,
        TCR: mmio.Mmio(packed struct(u32) {
            reserved0: u8,
            BUSY: u1,
            TraceBusID: u7,
            reserved1: u4,
            GTSFREQ: u2,
            TSPrescale: u2,
            reserved2: u3,
            SWOENA: u1,
            TXENA: u1,
            SYNCENA: u1,
            TSENA: u1,
            ITMENA: u1,
        }),
        reserved3: [32]u32 = undefined,
        reserved4: [43]u32 = undefined,
        LAR: mmio.Mmio(packed struct(u32) {
            LAR: u32,
        }),
        LSR: mmio.Mmio(packed struct(u32) {
            LSR: u32,
        }),
        reserved5: [6]u32 = undefined,
        PID4: mmio.Mmio(packed struct(u32) {
            PID4: u32,
        }),
        PID5: mmio.Mmio(packed struct(u32) {
            PID5: u32,
        }),
        PID6: mmio.Mmio(packed struct(u32) {
            PID6: u32,
        }),
        PID7: mmio.Mmio(packed struct(u32) {
            PID7: u32,
        }),
        PID0: mmio.Mmio(packed struct(u32) {
            PID0: u32,
        }),
        PID1: mmio.Mmio(packed struct(u32) {
            PID1: u32,
        }),
        PID2: mmio.Mmio(packed struct(u32) {
            PID2: u32,
        }),
        PID3: mmio.Mmio(packed struct(u32) {
            PID3: u32,
        }),
        CID0: mmio.Mmio(packed struct(u32) {
            CID0: u32,
        }),
        CID1: mmio.Mmio(packed struct(u32) {
            CID1: u32,
        }),
        CID2: mmio.Mmio(packed struct(u32) {
            CID2: u32,
        }),
        CID3: mmio.Mmio(packed struct(u32) {
            CID3: u32,
        }),
    };
};
