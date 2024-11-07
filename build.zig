const std = @import("std");
const MicroZig = @import("microzig/build");

fn root() []const u8 {
    return comptime (std.fs.path.dirname(@src().file) orelse ".");
}
const build_root = root();
const KiB = 1024;

const targets = [_]Example{
    .{ .target = f446re, .name = "STM32F446", .file = "src/motor.zig", .board_name = "f446_board", .bsp_file = "src/bsp/stm32f446.zig" },
    .{ .target = f401cc, .name = "STM32F401", .file = "src/motor.zig", .board_name = "f401_board", .bsp_file = "src/bsp/stm32f401.zig" },
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

pub const f401cc = MicroZig.Target{
    .preferred_format = .elf,
    .chip = .{
        .name = "STM32F401",
        .cpu = MicroZig.cpus.cortex_m4f,
        .memory_regions = &.{
            .{ .offset = 0x08000000, .length = 256 * KiB, .kind = .flash },
            .{ .offset = 0x20000000, .length = 64 * KiB, .kind = .ram },
        },
        .register_definition = .{
            .svd = .{ .cwd_relative = build_root ++ "/hal/STM32F401.svd" },
        },
    },
    .hal = .{
        .root_source_file = .{ .cwd_relative = build_root ++ "/hal/STM32F401.zig" },
    },
};

pub const f446re = MicroZig.Target{
    .preferred_format = .elf,
    .chip = .{
        .name = "STM32F446",
        .cpu = MicroZig.cpus.cortex_m4f,
        .memory_regions = &.{
            .{ .offset = 0x08000000, .length = 512 * KiB, .kind = .flash },
            .{ .offset = 0x20000000, .length = 128 * KiB, .kind = .ram },
        },
        .register_definition = .{
            .svd = .{ .cwd_relative = build_root ++ "/hal/STM32F446.svd" },
        },
    },
    .hal = .{
        .root_source_file = .{ .cwd_relative = build_root ++ "/hal/STM32F446.zig" },
    },
};
