const std = @import("std");
const assert = std.debug.assert;

const microzig = @import("microzig");
pub const peripherals = microzig.chip.peripherals;

const GPIOA = peripherals.GPIOA;
const GPIO = @TypeOf(GPIOA);

const GPIOB: GPIO = @ptrCast(peripherals.GPIOB);
const GPIOC: GPIO = @ptrCast(peripherals.GPIOC);
const GPIOD: GPIO = @ptrCast(peripherals.GPIOD);
const GPIOE: GPIO = @ptrCast(peripherals.GPIOE);
const GPIOF: GPIO = @ptrCast(peripherals.GPIOF);
const GPIOG: GPIO = @ptrCast(peripherals.GPIOG);

const log = std.log.scoped(.gpio);

pub const Function = enum {};

pub const Mode = union(enum) {
    input: InputMode,
    output: OutputMode,
};

pub const InputMode = enum(u2) {
    analog,
    floating,
    pull,
    reserved,
};

pub const OutputMode = enum(u2) {
    general_purpose_push_pull,
    general_purpose_open_drain,
    alternate_function_push_pull,
    alternate_function_open_drain,
};

pub const PinMode = enum(u2) { input, gpio, af, analog };

pub const Speed = enum(u2) {
    low,
    medium,
    high,
    fast,
};

pub const IrqLevel = enum(u2) {
    low,
    high,
    fall,
    rise,
};

pub const IrqCallback = fn (gpio: u32, events: u32) callconv(.C) void;

pub const Enabled = enum { // doesn't exist?
    disabled,
    enabled,
};

pub const Pull = enum(u2) { OC, up, down, reserved };

pub const AlternateFunction = enum(u4) {
    af0,
    af1,
    af2,
    af3,
    af4,
    af5,
    af6,
    af7,
    af8,
    af9,
    af10,
    af11,
    af12,
    af13,
    af14,
    af15,
};

// NOTE: With this current setup, every time we want to do anythting we go through a switch
//       Do we want this?
pub const Pin = packed struct(u8) {
    number: u4,
    port: u3,
    padding: u1,

    pub fn init(port: u3, number: u4) Pin {
        return Pin{
            .number = number,
            .port = port,
            .padding = 0,
        };
    }

    inline fn write_io_mode(gpio: Pin, io_mode: u2) void {
        const port = gpio.get_port();
        var val = port.MODER.raw;
        val &= ~@as(u32, io_mode << gpio.number);
        val |= @as(u32, io_mode << gpio.number);
        port.MODER.write_raw(val);
    }

    inline fn write_speed(gpio: Pin, speed: u2) void {
        const port = gpio.get_port();
        var val = port.OSPEEDR.raw;
        val &= ~@as(u32, speed << gpio.number);
        val |= @as(u32, speed << gpio.number);
        port.OSPEEDR.write_raw(val);
    }

    fn mask(gpio: Pin) u16 {
        return @as(u16, 1) << gpio.number;
    }

    // NOTE: Im not sure I like this
    //       We could probably calculate an offset from GPIOA?
    pub fn get_port(gpio: Pin) GPIO {
        return switch (gpio.port) {
            0 => GPIOA,
            1 => GPIOB,
            2 => GPIOC,
            3 => GPIOD,
            4 => GPIOE,
            5 => GPIOF,
            6 => GPIOG,
            7 => @panic("The STM32 only has ports 0..6 (A..G)"),
        };
    }

    // pub inline fn set_mode(gpio: Pin, mode: Mode) void {
    //     switch (mode) {
    //         .input => |in| gpio.set_input_mode(in),
    //         .output => |out| gpio.set_output_mode(out, .max_2MHz),
    //     }
    // }

    // pub inline fn set_input_mode(gpio: Pin, mode: PinMode) void {
    //     gpio.write_io_mode(@intFromEnum(mode));
    // }

    pub inline fn set_pin_mode(gpio: Pin, mode: PinMode, speed: Speed) void {
        const s_speed = @as(u32, @intFromEnum(speed));
        const m_mode = @as(u32, @intFromEnum(mode));
        gpio.write_io_mode(m_mode);
        gpio.write_speed(s_speed);
    }

    pub inline fn set_pull(gpio: Pin, pull: Pull) void {
        var port = gpio.get_port();
        switch (pull) {
            .up => port.BSRR.raw = gpio.mask(),
            .down => port.BRR.raw = gpio.mask(),
        }
    }

    pub inline fn read(gpio: Pin) u1 {
        const port = gpio.get_port();
        return if (port.IDR.raw & gpio.mask() != 0)
            1
        else
            0;
    }

    pub inline fn put(gpio: Pin, value: u1) void {
        var port = gpio.get_port();
        switch (value) {
            0 => port.BSRR.raw = gpio.mask() << 16,
            1 => port.BSRR.raw = gpio.mask(),
        }
    }

    pub inline fn toggle(gpio: Pin) void {
        var port = gpio.get_port();
        port.ODR.raw ^= gpio.mask();
    }

    pub inline fn set_af(gpio: Pin, af: AlternateFunction) void {
        const af_port = switch (gpio.number) {
            0...7 => gpio.get_port().AFRL,
            8...15 => gpio.get_port().AFRH,
            else => @panic("Invalid pin number, only 0-15!"),
        };
        const af_int = @intFromEnum(af);
        const val = af_port.raw;
        val &= ~@as(u32, af_int << gpio.number);
        val |= @as(u32, af_int << gpio.number);
        af_port.write_raw(val);
    }
};
