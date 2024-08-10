const std = @import("std");
const builtin = @import("builtin");
const microzig = @import("microzig");
const mmio = microzig.mmio;
pub const peripherals = microzig.chip.peripherals;

pub fn enable_itm(cpu_freq: u32, baud: u32) void {
    const SWOPrescaler = (cpu_freq / baud) - 1;
    CoreDebug.DEMCR.raw = 1 << 24;
    peripherals.DBG.DBGMCU_CR.raw = 0x27;

    TPIU.SPPR.modify(.{ .TXMODE = 0x2 });

    TPIU.ACPR.modify(.{ .SWOSCALER = @as(u16, @intCast(SWOPrescaler)) });

    ITM.LAR.raw = 0xC5ACCE55;
    ITM.TCR.modify(.{ .SYNCENA = 1, .ITMENA = 1, .TraceBusID = 1 });
    ITM.TPR.raw = 0xFFFFFFFF;
    ITM.TER.raw = 0x1;
    DWT.CTRL.raw = 0x400003FE;
    TPIU.FFCR.raw = 0x100;
}

const WriteError = error{};
pub const Writer = std.io.Writer(c_uint, WriteError, struct {
    pub fn write(context: c_uint, payload: []const u8) WriteError!usize {
        _ = context;
        for (payload) |chr| {
            ITM_SendChar(chr);
        }
        return payload.len;
    }
}.write);

var default_log_writter: Writer = .{ .context = 0 };

pub fn log(
    comptime level: std.log.Level,
    comptime scope: @TypeOf(.EnumLiteral),
    comptime format: []const u8,
    args: anytype,
) void {
    // Need to have Clock config nailed down and stored at comptime before this really works.
    // const state = struct {
    //     var init = false;
    // };

    // if (!state.init) {
    //     enable_itm();
    //     state.init = true;
    // }
    const level_prefix = comptime "[{}.{:0>6}] " ++ level.asText();
    const prefix = comptime level_prefix ++ switch (scope) {
        .default => ": ",
        else => " (" ++ @tagName(scope) ++ "): ",
    };
    for (format) |chr| {
        ITM_SendChar(chr);
    }
    default_log_writter.print(prefix ++ format ++ "\r\n", .{ 0, 0 } ++ args) catch {};
}

pub inline fn ITM_SendChar(chr: u8) void {
    // if ((ITM.TCR.read().ITMENA == 1)) {
    while ((ITM.PORT[0].raw & 0x1) == 0) {
        asm volatile ("nop");
    }
    const itm_stim_u8: *volatile u8 = @ptrCast(&ITM.PORT[0].raw);
    itm_stim_u8.* = chr;
}

pub inline fn static_print(msg: []const u8) void {
    for (msg) |chr| {
        ITM_SendChar(chr);
    }
}

pub const ITM: *volatile types.ITM = @ptrFromInt(0xE0000000);
pub const CoreDebug: *volatile types.CoreDebug = @ptrFromInt(0xE000EDF0);
pub const TPIU: *volatile types.TPIU = @ptrFromInt(0xE0040000);
pub const DWT: *volatile types.DWT = @ptrFromInt(0xE0001000);
// Cortex-M4 ROM table components
// 0xE00FF00C ITM val->0xFFF01003
// Reads as 0xFFF01002 if no ITM is implemented.

pub const types = struct {
    pub const DWT = extern struct {
        CTRL: mmio.Mmio(packed struct(u32) { //
            CTRL: u32,
        }),
        CYCCNT: mmio.Mmio(packed struct(u32) { //
            CYCCNT: u32,
        }),
        EXCCNT: mmio.Mmio(packed struct(u32) { //
            EXCCNT: u32,
        }),
        SLEEPCNT: mmio.Mmio(packed struct(u32) { //
            SLEEPCNT: u32,
        }),
        LSUCNT: mmio.Mmio(packed struct(u32) { //
            LSUCNT: u32,
        }),
        FOLDCNT: mmio.Mmio(packed struct(u32) { //
            FOLDCNT: u32,
        }),
        PCSR: mmio.Mmio(packed struct(u32) { //
            PCSR: u32,
        }),
        COMP0: mmio.Mmio(packed struct(u32) { //
            COMP0: u32,
        }),
        MASK0: mmio.Mmio(packed struct(u32) { //
            MASK0: u32,
        }),
        FUNCTION0: mmio.Mmio(packed struct(u32) { //
            FUNCTION0: u32,
        }),
        reserved0: mmio.Mmio(packed struct(u32) { //
            reserved: u32,
        }),
        COMP1: mmio.Mmio(packed struct(u32) { //
            COMP1: u32,
        }),
        MASK1: mmio.Mmio(packed struct(u32) { //
            MASK1: u32,
        }),
        FUNCTION1: mmio.Mmio(packed struct(u32) { //
            FUNCTION1: u32,
        }),
        reserved1: mmio.Mmio(packed struct(u32) { //
            reserved: u32,
        }),
        COMP2: mmio.Mmio(packed struct(u32) { //
            COMP2: u32,
        }),
        MASK2: mmio.Mmio(packed struct(u32) { //
            MASK2: u32,
        }),
        FUNCTION2: mmio.Mmio(packed struct(u32) { //
            FUNCTION2: u32,
        }),
        reserved2: mmio.Mmio(packed struct(u32) { //
            reserved: u32,
        }),
        COMP3: mmio.Mmio(packed struct(u32) { //
            COMP3: u32,
        }),
        MASK3: mmio.Mmio(packed struct(u32) { //
            MASK3: u32,
        }),
        FUNCTION3: mmio.Mmio(packed struct(u32) { //
            FUNCTION3: u32,
        }),
    };
    pub const TPIU = extern struct {
        SSPSR: mmio.Mmio(packed struct(u32) { //
            SWIDTH: u32,
        }),
        CSPSR: mmio.Mmio(packed struct(u32) { //
            CWIDTH: u32,
        }),
        reserved0: [2]mmio.Mmio(packed struct(u32) { //
            reserved: u32,
        }),
        ACPR: mmio.Mmio(packed struct(u32) {
            reserved0: u16,
            SWOSCALER: u16,
        }),
        reserved1: [55]mmio.Mmio(packed struct(u32) { //
            reserved: u32,
        }),
        SPPR: mmio.Mmio(packed struct(u32) {
            reserved: u30,
            TXMODE: u2,
        }),
        reserved2: [131]mmio.Mmio(packed struct(u32) { //
            reserved: u32,
        }),
        FFSR: mmio.Mmio(packed struct(u32) {
            FFSR: u32,
        }),
        FFCR: mmio.Mmio(packed struct(u32) {
            FFCR: u32,
        }),
        reserved3: [759]mmio.Mmio(packed struct(u32) { //
            reserved: u32,
        }),
        TRIGGER: mmio.Mmio(packed struct(u32) {
            TRIGGER: u32,
        }),
        FIFO0: mmio.Mmio(packed struct(u32) {
            FIFO0: u32,
        }),
        ITATTBCTR2: mmio.Mmio(packed struct(u32) {
            ITATTBCTR2: u32,
        }),
        reserved4: mmio.Mmio(packed struct(u32) { //
            reserved: u32,
        }),
        ITATBCTR0: mmio.Mmio(packed struct(u32) {
            ITATBCTR0: u32,
        }),
        FIFO1: mmio.Mmio(packed struct(u32) {
            FIFO1: u32,
        }),
        ITCTRL: mmio.Mmio(packed struct(u32) {
            ITCTRL: u32,
        }),
        reserved5: [39]mmio.Mmio(packed struct(u32) { //
            reserved: u32,
        }),
        CLAIMSET: mmio.Mmio(packed struct(u32) { //
            CLAIMSET: u32,
        }),
        CLAIMCLR: mmio.Mmio(packed struct(u32) { //
            CLAIMCLR: u32,
        }),
        reserved7: [8]mmio.Mmio(packed struct(u32) { // yeah, 7, that's what core_cm4.h says
            reserved: u32,
        }),
        DEVID: mmio.Mmio(packed struct(u32) { //
            reserved: u16,
            impl0: u4,
            NRZVALID: u1,
            MANCVALID: u1,
            PTINVALID: u1,
            FIFOSZ: u3,
            impl1: u6,
        }),
        DEVTYPE: [39]mmio.Mmio(packed struct(u32) { //
            DEVTYPE: u32,
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
