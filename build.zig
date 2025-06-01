const std = @import("std");
const MicroZig = @import("microzig");

const MicroBuild = MicroZig.MicroBuild(.{
    .stm32 = true,
});

// fn root() []const u8 {
//     return comptime (std.fs.path.dirname(@src().file) orelse ".");
// }
// const build_root = root();

fn svdPath(comptime suffix: []const u8) []const u8 {
    if (suffix[0] != '/') @compileError("relToPath requires an absolute path!");
    return comptime blk: {
        const root_dir = std.fs.path.dirname(@src().file) orelse ".";
        break :blk root_dir ++ suffix;
    };
}

const KiB = 1024;

// const targets = [_]Example{
//     .{ .target = f446re, .name = "STM32F446", .file = "src/motor.zig", .board_name = "f446_board", .bsp_file = "src/bsp/stm32f446.zig" },
//     .{ .target = f401cc, .name = "STM32F401", .file = "src/motor.zig", .board_name = "f401_board", .bsp_file = "src/bsp/stm32f401.zig" },
// };

pub fn build(b: *std.Build) void {
    const optimize = b.standardOptimizeOption(.{});
    const mz_dep = b.dependency("microzig", .{});
    const mb = MicroBuild.init(b, mz_dep) orelse return;

    const targets = init(b, mz_dep);

    for (targets) |example| {
        const firmware = mb.add_firmware(.{
            .name = example.name,
            .target = &example.target,
            .optimize = optimize,
            .root_source_file = b.path(example.file),
            .board = .{ .name = example.board_name, .root_source_file = b.path(example.bsp_file) },
        });

        // `install_firmware()` is the MicroZig pendant to `Build.installArtifact()`
        // and allows installing the firmware as a typical firmware file.
        //
        // This will also install into `$prefix/firmware` instead of `$prefix/bin`.
        mb.install_firmware(firmware, .{});

        // For debugging, we also always install the firmware as an ELF file
        mb.install_firmware(firmware, .{ .format = .elf });
    }
}

const Example = struct {
    target: MicroZig.Target,
    name: []const u8,
    file: []const u8,
    board_name: []const u8,
    bsp_file: []const u8,
};

// , build_root: std.Build.Cache.Directory
pub fn init(b: *std.Build, dep: *std.Build.Dependency) [2]Example {
    // var buffer: [1000]u8 = undefined;
    // var FBA = std.heap.FixedBufferAllocator.init(&buffer);
    // const fba = FBA.allocator();

    const f401cc = MicroZig.Target{
        .dep = dep,
        .preferred_binary_format = .elf,
        .zig_target = .{
            .cpu_arch = .thumb,
            .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m4 },
            .os_tag = .freestanding,
            .abi = .eabihf,
        },
        .chip = .{
            .name = "STM32F401",
            .memory_regions = &.{
                .{ .offset = 0x08000000, .length = 256 * KiB, .kind = .flash },
                .{ .offset = 0x20000000, .length = 64 * KiB, .kind = .ram },
            },
            .register_definition = .{
                .svd = b.path("hal/STM32F401.svd"),
            },
        },
        .hal = .{
            .root_source_file = b.path("hal/STM32F401.zig"),
        },
    };

    const f446re = MicroZig.Target{
        .dep = dep,
        .preferred_binary_format = .elf,
        .zig_target = .{
            .cpu_arch = .thumb,
            .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m4 },
            .os_tag = .freestanding,
            .abi = .eabihf,
        },
        .chip = .{
            .name = "STM32F446",
            .memory_regions = &.{
                .{ .offset = 0x08000000, .length = 512 * KiB, .kind = .flash },
                .{ .offset = 0x20000000, .length = 128 * KiB, .kind = .ram },
            },
            .register_definition = .{
                .svd = b.path("hal/STM32F446.svd"),
            },
        },
        .hal = .{
            .root_source_file = b.path("hal/STM32F446.zig"),
        },
    };

    // std.debug.print("Build root: {s}\n", .{build_root.join(fba, &[_][]const u8{"hal/STM32F446.svd"}).?});
    // std.debug.print("Build root: {s}\n", .{"hal/STM32F446.svd"});
    // std.debug.print("{s}\n", .{@src().file});
    return .{
        .{ .target = f446re, .name = "STM32F446", .file = "src/motor.zig", .board_name = "f446_board", .bsp_file = "src/bsp/stm32f446.zig" },
        .{ .target = f401cc, .name = "STM32F401", .file = "src/motor.zig", .board_name = "f401_board", .bsp_file = "src/bsp/stm32f401.zig" },
    };
}
