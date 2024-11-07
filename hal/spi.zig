const std = @import("std");
const microzig = @import("microzig");
const builtin = @import("builtin");
const stm32 = microzig.chip;

const clocks = struct { GlobalConfiguration: u32 = 0 };

const SpiRegs = microzig.chip.types.peripherals.I2S2ext;

const SPI1_reg = stm32.peripherals.I2S2ext;
const SPI2_reg = stm32.peripherals.SPI2;

pub const FrameFormat = union(enum) {
    pub const Motorola = struct {
        pub const Polarity = enum(u1) {
            default_low = 0,
            default_high = 1,
        };

        pub const Phase = enum(u1) {
            first_edge = 0,
            second_edge = 1,
        };

        /// default_low means clock is steady state low, default_high means steady state high
        clock_polarity: Polarity = .default_low,

        /// Controls whether data is captured on the first or second clock edge transition
        clock_phase: Phase = .first_edge,
    };

    motorola: Motorola,
    texas_instruments,
    ns_microwire,
};

pub const DataWidthBits = enum(u4) {
    eight = 7,
    sixteen = 15,
};

pub const Config = struct {
    clock_config: clocks.GlobalConfiguration,
    data_width: DataWidthBits = .eight,
    frame_format: FrameFormat = .{ .motorola = .{} },
    baud_rate: u32 = 1_000_000,
};

pub const instance = struct {
    pub const SPI1: SPI = @as(SPI, @enumFromInt(0));
    pub const SPI2: SPI = @as(SPI, @enumFromInt(1));
    pub fn num(instance_number: u1) SPI {
        return @as(SPI, @enumFromInt(instance_number));
    }
};

pub const ConfigError = error{
    InvalidDataWidth,
    InputFreqTooLow,
    InputFreqTooHigh,
    UnsupportedBaudRate,
};

pub const SPI = enum(u4) {
    _,

    fn get_regs(spi: SPI) *volatile SpiRegs {
        return switch (@intFromEnum(spi)) {
            0 => SPI1_reg,
            1 => SPI2_reg,
        };
    }
    /// Initializes the SPI HW block per the Config provided
    /// Per the API limitations discussed above, the following settings are fixed and not configurable:
    /// - Controller Mode only
    /// - DREQ signalling is always enabled, harmless if DMA isn't configured to listen for this
    pub fn apply(spi: SPI, comptime config: Config) ConfigError!void {

        // TODO - these are all still rp2040

        const peri_freq = config.clock_config.peri.?.output_freq;
        try spi.set_baudrate(config.baud_rate, peri_freq);

        const spi_regs = spi.get_regs();

        switch (config.frame_format) {
            .motorola => |ff| {
                spi_regs.SSPCR0.modify(.{
                    .FRF = 0b00,
                    .DSS = @intFromEnum(config.data_width),
                    .SPO = @intFromEnum(ff.clock_polarity),
                    .SPH = @intFromEnum(ff.clock_phase),
                });
            },

            .texas_instruments => {
                spi_regs.SSPCR0.modify(.{
                    .FRF = 0b01,
                    .DSS = @intFromEnum(config.data_width),
                    .SPO = 0,
                    .SPH = 0,
                });
            },

            .ns_microwire => {
                spi_regs.SSPCR0.modify(.{
                    .FRF = 0b10,
                    .DSS = @intFromEnum(config.data_width),
                    .SPO = 0,
                    .SPH = 0,
                });
            },
        }

        spi_regs.SSPDMACR.modify(.{
            .TXDMAE = 1,
            .RXDMAE = 1,
        });

        // Enable SPI
        spi_regs.SSPCR1.modify(.{
            .SSE = 1,
        });
    }
};
