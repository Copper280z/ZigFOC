//! Logging backend for the RTT protocol

const std = @import("std");
const rtt = @import("rtt");

fn init() void {
    rtt.config_up_buffer(.{
        .index = 0,
        .name = "debug",
        .mode = .BlockIfFifoFull,
    });
}

pub fn log(
    comptime level: std.log.Level,
    comptime scope: @TypeOf(.EnumLiteral),
    comptime format: []const u8,
    args: anytype,
) void {
    const state = struct {
        var init = false;
    };

    if (!state.init) {
        init();
        state.init = true;
    }

    rtt.logFn(level, scope, format, args);
}
