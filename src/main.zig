const std = @import("std");
const microzig = @import("microzig");
const builtin = @import("builtin");
const itm = @import("itm.zig");
// const rtt = @import("rtt");
const logger = std.log.scoped(.main);

const bsp = microzig.board;
const stm32 = microzig.hal;

const uart = stm32.Uart;
const Handler = stm32.interrupt.Handler;

const approx = @import("approx_math.zig");
const PID = @import("pid.zig");

pub fn panic(message: []const u8, _: ?*std.builtin.StackTrace, _: ?usize) noreturn {

    // put whatever you want here
    std.log.info("{s}", .{message});
    // bsp.init_uart();
    // for (message) |chr| {
    //     bsp.tx(chr);
    // }
    // bsp.tx('\r');
    // bsp.tx('\n');
    @breakpoint();
    while (true) {}
}

fn HardFault() callconv(.C) void {
    @panic("We got a hard fault!");
}

pub const microzig_options = .{ //
    .interrupts = .{
        .HardFault = Handler{ .C = &HardFault },
    },
    .logFn = itm.log,
};

// pub const std_options = .{ .log_level = .info, .logFn = bsp.log };

pub fn enable_interrupt(IRQn: stm32.IRQn_Type) void {
    asm volatile ("" ::: "memory");
    const idx = @as(u32, @intFromEnum(IRQn) >> 5);
    const shift: u5 = @intCast(@intFromEnum(IRQn) & 0x1F);
    switch (idx) {
        // writing zeros to this register has no effect, so we just write a 1 where we want it
        0 => {
            stm32.peripherals.NVIC.ISER0.raw = @as(u32, 1) << shift;
        },
        1 => {
            stm32.peripherals.NVIC.ISER1.raw = @as(u32, 1) << shift;
        },
        2 => {
            stm32.peripherals.NVIC.ISER2.raw = @as(u32, 1) << shift;
        },
        else => unreachable,
    }
    asm volatile ("" ::: "memory");
}

pub fn disable_interrupt(IRQn: stm32.IRQn_Type) void {
    asm volatile ("" ::: "memory");
    const idx = @as(u32, @intFromEnum(IRQn) >> 5);
    const shift: u5 = @intCast(@intFromEnum(IRQn) & 0x1F);
    switch (idx) {
        // writing zeros to this register has no effect, so we just write a 1 where we want it
        0 => {
            stm32.peripherals.NVIC.ICER0.raw = @as(u32, 1) << shift;
        },
        1 => {
            stm32.peripherals.NVIC.ICER1.raw = @as(u32, 1) << shift;
        },
        2 => {
            stm32.peripherals.NVIC.ICER2.raw = @as(u32, 1) << shift;
        },
        else => unreachable,
    }
    asm volatile ("" ::: "memory");
}

pub fn do_some_nops() void {
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
}

pub fn main() !void {
    stm32.peripherals.FPU_CPACR.CPACR.modify(.{ .CP = 0b1111 });
    // stm32.cpu.peripherals.SysTick.CTRL.modify(.{ .ENABLE = 1 });
    // bsp.init_rcc();
    itm.enable_itm(16000000, 2000000);
    itm.ITM_EnableChannel(0);
    itm.ITM_EnableChannel(1);
    itm.ITM_EnableChannel(2);
    // bsp.init_gpio();
    // bsp.init_uart();
    // for ("\r\nuart active\r\n") |chr| {
    //     itm.ITM_SendChar(chr);
    // }
    // logger.info("UART up\n", .{});

    for ("starting loop\r\n") |chr| {
        itm.ITM_SendChar(0, chr);
    }

    // const printer: itm.Writer = .{ .context = 0 };

    var counts: u32 = 0;

    while (true) {
        const angle = @as(f32, @floatFromInt(counts)) * 6.28 / 101.0;
        const approx_sine = @as(i32, @intFromFloat(approx._sin(angle) * 100));
        const real_sine = @as(i32, @intFromFloat(@cos(angle) * 100));
        // const angle: f32 = 567;
        // std.mem.doNotOptimizeAway(std.log.info("a: {}", .{counts}));
        itm.ITM_SendChar(1, @as(u32, @bitCast(approx_sine)));
        itm.ITM_SendChar(2, @as(u32, @bitCast(real_sine)));
        counts += 1;
        if (counts > 100) {
            counts = 0;
        }
        for (0..50_000) |_| {
            do_some_nops();
        }
    }
}
