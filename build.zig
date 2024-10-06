const std = @import("std");
const MicroZig = @import("microzig/build");
const stm32 = @import("microzig/bsp/stmicro/stm32");

fn root() []const u8 {
    return comptime (std.fs.path.dirname(@src().file) orelse ".");
}
const build_root = root();
const KiB = 1024;

const targets = [_]Example{
    .{ .target = Orbmule, .name = "Orbmule", .file = "src/main.zig", .board_name = "orbmule", .bsp_file = "src/bsp/stm32f730.zig" },
};

pub fn build(b: *std.Build) void {
    const microzig = MicroZig.init(b, .{});
    const optimize = b.standardOptimizeOption(.{});

    const rtt_dep = b.dependency("rtt", .{}).module("rtt");

    for (targets) |example| {
        // `add_firmware` basically works like addExecutable, but takes a
        // `microzig.Target` for target instead of a `std.zig.CrossTarget`.
        //
        // The target will convey all necessary information on the chip,
        // cpu and potentially the board as well.
        const firmware = microzig.add_firmware(b, .{
            .name = example.name,
            .target = example.target,
            .optimize = optimize,
            .root_source_file = b.path(example.file),
            .board = .{ .name = example.board_name, .root_source_file = b.path(example.bsp_file) },
        });
        // firmware.add_app_import("rtt", rtt_dep, .{});
        _ = rtt_dep;

        // `install_firmware()` is the MicroZig pendant to `Build.installArtifact()`
        // and allows installing the firmware as a typical firmware file.
        //
        // This will also install into `$prefix/firmware` instead of `$prefix/bin`.
        microzig.install_firmware(b, firmware, .{});

        // For debugging, we also always install the firmware as an ELF file
        microzig.install_firmware(b, firmware, .{ .format = .elf });
    }
}

const Example = struct {
    target: MicroZig.Target,
    name: []const u8,
    file: []const u8,
    board_name: []const u8,
    bsp_file: []const u8,
};

pub const Orbmule = MicroZig.Target{
    .preferred_format = .elf,
    .chip = .{
        .name = "STM32F730",
        .cpu = cortex_m7f,
        .memory_regions = &.{
            .{ .offset = 0x08000000, .length = 64 * KiB, .kind = .flash },
            .{ .offset = 0x20000000, .length = 256 * KiB, .kind = .ram },
        },
        .register_definition = .{
            .svd = .{ .cwd_relative = build_root ++ "/hal/STM32F730.svd" },
        },
    },
    .hal = .{
        .root_source_file = .{ .cwd_relative = build_root ++ "/hal/STM32F730.zig" },
    },
};

pub const cortex_m7 = MicroZig.Cpu{
    .name = "ARM Cortex-M7",
    .root_source_file = .{ .cwd_relative = "../microzig/core/src/cpus/cortex_m.zig" },
    .target = std.Target.Query{
        .cpu_arch = .thumb,
        .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m7 },
        .os_tag = .freestanding,
        .abi = .eabi,
    },
};

pub const cortex_m7f = MicroZig.Cpu{
    .name = "ARM Cortex-M7F",
    .root_source_file = .{ .cwd_relative = "../microzig/core/src/cpus/cortex_m.zig" },
    .target = std.zig.CrossTarget{
        .cpu_arch = .thumb,
        .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m7 },
        .cpu_features_add = std.Target.arm.featureSet(&.{.fp_armv8d16sp}),
        .os_tag = .freestanding,
        .abi = .eabihf,
    },
};
