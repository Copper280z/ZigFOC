const std = @import("std");
const microzig = @import("microzig");
const NVIC = microzig.cpu.nvic;
const builtin = @import("builtin");

const TBD = @import("tbd.zig");

// ICTR = Controller Type (RO)
// ISER = Set Enable - Can read to check enable status, writing 1 enables, writing 0 ignored
// ICER = Clear Enable - Can read to check enable status, writing 1 DISABLES, writing 0 ignored
// ISPR = Set Pending - Can read to check pend status, writing 1 pends, writing 0 ignored
// ICPR = Clear Pending - Can read to check pend status, writing 1 clears pend, writing 0 ignored
// IABR = Active Bit (RO) - Bitwise read shows active IRQ. What's 'active', though?
// IPR  = Priority - 8 bit priority, 4 per register, N0 at LSB, LOWER -> MORE IMPORTANT

pub const arch_funcs = struct {
    pub inline fn crit_enter() void {
        asm volatile ("cpsid i");
    }
    pub inline fn crit_exit() void {
        asm volatile ("cpsie i");
    }
};

const arm_funcs = arch_funcs{};

// basic task implementation that uses ticks to time activations
// task could totally ignore the ticks and use some other decision making process
pub fn Task(context_t: anytype, TaskFn: fn (context_t) void) type {
    const use_queue = true;
    // add task description
    return struct {
        is_armed: bool = false,
        tick_counter: u32 = undefined,
        interval: u32 = undefined,
        ctx: context_t = undefined,
        task_queue: u8, // just a counter for now, could be more complex
        queue_max: u8 = std.math.maxInt(u8),

        pub inline fn armed(self: @This()) bool {
            return self.armed;
        }

        pub inline fn expiring(self: @This()) bool {
            return (self.tick_counter == 0);
        }

        pub inline fn reset(self: *@This()) void {
            self.tick_counter = self.interval;
        }

        pub inline fn tick(self: *@This()) void {
            self.tick_counter -= 1;
        }
        pub inline fn post(self: *@This(), comptime in_crit: bool) void {
            if (in_crit) {
                self.pend_task();
                self.inc_queue();
            } else {
                arm_funcs.crit_enter();
                self.pend_task();
                self.inc_queue();
                arm_funcs.crit_exit();
            }
        }
        pub inline fn execute(self: *@This()) void {
            self.dec_queue();
            TaskFn(self.ctx);
        }

        // vv stuff that shouldn't really need to be used externally
        inline fn pend_task(self: *@This()) void {
            _ = self;
        }

        inline fn inc_queue(self: *@This()) void {
            if (use_queue) {
                if (self.task_queue < self.queue_max) {
                    self.task_queue += 1;
                } else {
                    // use task description here
                    @panic("Overflowed Task Queue");
                }
            }
        }

        inline fn dec_queue(self: *@This()) void {
            if (use_queue) {
                if (self.task_queue > 0) {
                    self.task_queue -= 1;
                } else {
                    // use task description here
                    @panic("Underflowed Task Queue");
                }
            }
        }
    };
}
