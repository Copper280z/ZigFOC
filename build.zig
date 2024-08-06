const std = @import("std");
const MicroZig = @import("microzig/build");
const stm32 = @import("microzig/bsp/stmicro/stm32");

fn root() []const u8 {
    return comptime (std.fs.path.dirname(@src().file) orelse ".");
}
const build_root = root();
const KiB = 1024;

pub const F446 = MicroZig.Target{
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

const targets = [_]Example{
    .{ .target = F446, .name = "STM32F446", .file = "src/motor.zig" },
};

pub fn build(b: *std.Build) void {
    const microzig = MicroZig.init(b, .{});
    const optimize = b.standardOptimizeOption(.{});

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
        });

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
};
