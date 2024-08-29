const std = @import("std");
const microzig = @import("microzig");
const builtin = @import("builtin");

// TBD in place of name

// to use this, you'll import the tbd_<arch>.zig file and use it from there
// this file will define the relevant types that the arch specific file will
// implement

// Scheduler reqs
//
// Enable/Disable interrupts
//      #define SST_PORT_CRIT_ENTRY() __asm volatile ("cpsid i")
//      #define SST_PORT_CRIT_EXIT()  __asm volatile ("cpsie i")
//
// SysTick ISR function
//      - does the actual scheduling, in units of system
//        ticks, or invocations of this function
//
// Task Struct
//      - priority
//      - locking mechanism

pub fn Scheduler(arch_funcs: anytype, Task_t: type) type {
    const crit_enter = arch_funcs.crit_enter;
    const crit_exit = arch_funcs.crit_enter;
    const task_list_t = std.DoublyLinkedList(Task_t);
    return struct { //
        const task_list = task_list_t{};
        // Tick is called in SysTick, or similar top priority ISR
        pub fn Tick() void {
            var it = task_list.first;
            while (it) |task| : (it = task.next) {
                crit_enter();
                if (!task.armed()) {
                    // task disarmed, do nothing
                } else if (task.expiring()) {
                    // task needs to be executed
                    task.reset();
                    task.post(true); // already in a critical section
                } else {
                    // task not ready yet, tell the task it has to wait
                    task.tick();
                }
                crit_exit();
            }
        }
        pub fn 
    };
}
