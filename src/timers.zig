const std = @import("std");
const microzig = @import("microzig");
const stm32 = microzig.hal;

pub fn Timer(comptime timer_type: type) type {
    // const tim_name = @typeInfo(timer_type).Struct.decls[0].name;
    // std.debug.print("", .{tim_name});
    // const available_tims = std.meta.fieldNames(stm32);
    return struct {
        timer: timer_type,

        pub inline fn init(timer: @This()) void {
            _ = timer;
            // comptime func to build list of available timer types
            // then iterate through that here to find a match?
            if (@hasDecl(stm32, "TIM1")) {
                if (@TypeOf(@field(stm32, "TIM1")) == timer_type) {
                    stm32.RCC.APB2ENR.modify(.{ .TIM1EN = 1 });
                }
            }
            if (@hasDecl(stm32, "TIMfarts")) {
                if (@TypeOf(@field(stm32, "TIMfarts")) == timer_type) {
                    stm32.RCC.APB1ENR.modify(.{ .TIM2EN = 0 });
                }
            }
        }

        pub inline fn setCenterAlignedMode(timer: @This(), mode: u2) void {
            timer.CR1.modify(.{ .CMS = mode }); // center-aligned 3
        }
        pub inline fn enable(tim: @This()) void {
            tim.timer.CR1.modify(.{ .CEN = 1 });
        }
        pub inline fn disable(tim: @This()) void {
            tim.timer.CR1.modify(.{ .CEN = 0 });
        }
        pub inline fn setTRGO(tim: @This(), val: u3) void { // TODO: make TRGO Enum
            tim.timer.CR2.modify(.{ .MMS = val }); // trgo config
        }
    };
}
