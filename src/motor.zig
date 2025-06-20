const std = @import("std");
const microzig = @import("microzig");
const builtin = @import("builtin");
const itm = @import("itm.zig");
// const rtt = @import("rtt");
const logger = std.log.scoped(.main);

const bsp = microzig.board;
const stm32 = microzig.hal;

const uart = stm32.Uart;
const Handler = stm32.interrupt.Handler;

const approx = @import("approx_math.zig");
const PID = @import("pid.zig");

const DQVoltage_s = struct { d: f32 = 0, q: f32 = 0 };
const DQCurrent_s = struct { d: f32 = 0, q: f32 = 0 };
const PhaseCurrent_s = struct { a: f32 = 0, b: f32 = 0, c: f32 = 0 };
const ABCurrent_s = struct { alpha: f32 = 0, beta: f32 = 0 };

pub fn Driver(
    comptime motor: type,
    comptime setPwmFn: fn (motor: motor, pwm_a: f32, pwm_b: f32, pwm_c: f32) void,
) type {
    return struct {
        mot: motor,
        pub fn setPwm(driver: @This(), pwm_a: f32, pwm_b: f32, pwm_c: f32) void {
            return setPwmFn(driver.mot, pwm_a, pwm_b, pwm_c);
        }
    };
}

pub fn BuildMotor(comptime timer_type: type) type {
    const silent_hfi = false;
    const core_clock = bsp.core_clock;
    const pwm_freq: f32 = 40000;
    const TS: f32 = 2 / pwm_freq;
    // const _1_TS: f32 = pwm_freq;
    const pi_type = PID.PID_const_Ts(TS, true);

    return struct {
        tim: timer_type,

        enabled: bool = false,
        hfi_on: bool = false,
        hfi_firstcycle: bool = true,
        hfi_high: bool = false,
        hfi_out: f32 = 0,
        hfi_full_turns: f32 = 0,

        electrical_angle: f32 = 0,
        hfi_curangleest: f32 = 0,
        hfi_error: f32 = 0,
        hfi_int: f32 = 0,
        error_saturation_limit: f32 = 0.1,
        Ts: f32 = TS,
        Ld: f32 = 19.5e-6,
        Lq: f32 = 26.0e-6,
        phase_resistance: f32 = 0.09,
        voltage_power_supply: f32 = 12,
        voltage_limit: f32 = 11,
        hfi_v: f32 = 2.0,
        amps_per_volt: f32 = 16.36,
        hfi_gain1: f32 = 750 * approx._2PI,
        hfi_gain2: f32 = 5 * approx._2PI,
        Ualpha: f32 = 0,
        Ubeta: f32 = 0,
        Ua: f32 = 0,
        Ub: f32 = 0,
        Uc: f32 = 0,

        voltage: DQVoltage_s = .{ .d = 0, .q = 0 },
        current_meas: DQCurrent_s = .{ .d = 0, .q = 0 },
        current_high: DQCurrent_s = .{ .d = 0, .q = 0 },
        current_low: DQCurrent_s = .{ .d = 0, .q = 0 },
        delta_current: DQCurrent_s = .{ .d = 0, .q = 0 },
        current_err: DQCurrent_s = .{ .d = 0, .q = 0 },
        current_setpoint: DQCurrent_s = .{ .d = 0, .q = 0 },

        current_bandwidth: f32 = 300,
        PI_current_q: pi_type = undefined,
        PI_current_d: pi_type = undefined,

        inline fn init_timer(self: *@This()) void {
            // comptime func to build list of available timer types
            // then iterate through that here to find a match?
            stm32.RCC.APB2ENR.modify(.{ .TIM1EN = 1 });
            // if (@hasDecl(stm32, "TIM1")) {
            //     if (@TypeOf(@field(stm32, "TIM1")) == timer_type) {
            //         stm32.RCC.APB2ENR.modify(.{ .TIM1EN = 1 });
            //     }
            // }
            // if (@hasDecl(stm32, "TIMfake")) {
            //     if (@TypeOf(@field(stm32, "TIMfake")) == timer_type) {
            //         stm32.RCC.APB1ENR.modify(.{ .TIM2EN = 0 });
            //     }
            // }

            // should query RCC and figure out what the input clock freq is
            self.tim.CR1.modify(.{ .CEN = 1 });

            self.tim.CR1.modify(.{ .CMS = 3 }); // center-aligned 3
            self.tim.CR1.modify(.{ .URS = 1 });
            self.tim.CR1.modify(.{ .ARPE = 1 }); // 1 -> ARR is buffered
            self.tim.CR2.modify(.{ .MMS = 0b010 }); // trgo config
            // these 2 regs set pwm frequency together, need to do math to set them appropriately
            // center aligned pwm counts up to ARR then back down
            // so it's 2xARR clocks per period
            // max clock config is 180MHz timer clock,
            // 180MHz / 20000 = (2*4500)
            const arr = core_clock / pwm_freq / 2;
            self.tim.PSC.modify(.{ .PSC = 0 }); // prescaler
            self.tim.ARR.modify(.{ .ARR = arr }); // auto-reload

            // ch1
            self.tim.CCER.modify(.{ .CC1E = 0 }); // disable channel for config
            self.tim.CCMR1_Output.modify(.{ .OC1M = 0b111 }); // CCMR1.OCxM = 0b111 output is active while CNT > CCRx
            self.tim.CCMR1_Output.modify(.{ .CC1S = 0 }); // 0 -> CC1 is an output, channel must be disabled to write
            self.tim.CCMR1_Output.modify(.{ .OC1PE = 1 }); // enable CCR preload
            self.tim.CCER.modify(.{ .CC1E = 1 }); // re-enable channel
            self.tim.CCER.modify(.{ .CC1NE = 1 }); // re-enable channel

            self.tim.CCER.modify(.{ .CC2E = 0 }); // disable channel for config
            self.tim.CCMR1_Output.modify(.{ .OC2M = 0b111 }); // CCMR1.OCxM = 0b111 output is active while CNT > CCRx
            self.tim.CCMR1_Output.modify(.{ .CC2S = 0 }); // 0 -> CC1 is an output, channel must be disabled to write
            self.tim.CCMR1_Output.modify(.{ .OC2PE = 1 }); // enable CCR preload
            self.tim.CCER.modify(.{ .CC2E = 1 }); // re-enable channel
            self.tim.CCER.modify(.{ .CC2NE = 1 }); // re-enable channel

            self.tim.CCER.modify(.{ .CC3E = 0 }); // disable channel for config
            self.tim.CCMR2_Output.modify(.{ .OC3M = 0b111 }); // CCMR1.OCxM = 0b111 output is active while CNT > CCRx
            self.tim.CCMR2_Output.modify(.{ .CC3S = 0 }); // 0 -> CC1 is an output, channel must be disabled to write
            self.tim.CCMR2_Output.modify(.{ .OC3PE = 1 }); // enable CCR preload
            self.tim.CCER.modify(.{ .CC3E = 1 }); // re-enable channel
            self.tim.CCER.modify(.{ .CC3NE = 1 }); // re-enable channel

            self.tim.CCR1.modify(.{ .CCR1 = 1000 });
            self.tim.CCR2.modify(.{ .CCR2 = 1000 });
            self.tim.CCR3.modify(.{ .CCR3 = 1000 });

            self.tim.BDTR.modify(.{ .MOE = 1, .DTG = 0x10 }); // Main Output Enable, only exists on TIM1/8?
            // if 6pwm then BDTR.DTG needs to be set, 0x10 gives ~300ns dead time
            self.tim.EGR.modify(.{ .UG = 1 }); // Write update bit
        }

        inline fn getPwmState(self: @This()) bool {
            return self.tim.CR1.read().DIR == 0;
        }

        fn getPhaseCurrents(self: @This()) PhaseCurrent_s {
            const volts = bsp.get_adc_vals();
            const va: f32 = self.amps_per_volt * (@as(f32, @floatFromInt(volts[0])) - 2048) * (3.3 / 4096.0);
            const vb: f32 = self.amps_per_volt * (@as(f32, @floatFromInt(volts[1])) - 2048) * (3.3 / 4096.0);
            return PhaseCurrent_s{ .a = va, .b = vb, .c = 0 };
        }

        fn getABCurrents(phase_current: PhaseCurrent_s) ABCurrent_s {
            const i_alpha = phase_current.a;
            const i_beta = approx._1_SQRT3 * phase_current.a + approx._2_SQRT3 * phase_current.b;
            return ABCurrent_s{ .alpha = i_alpha, .beta = i_beta };
        }

        pub fn init(self: *@This()) void {
            init_timer(self);
            // now init ADC

            bsp.init_adc();

            self.PI_current_d = pi_type{ //
                .Kp = self.Ld * self.current_bandwidth * approx._2PI,
                .Ki = self.current_bandwidth * approx._2PI * self.phase_resistance,
            };
            self.PI_current_q = pi_type{ //
                .Kp = self.Lq * self.current_bandwidth * approx._2PI,
                .Ki = self.current_bandwidth * approx._2PI * self.phase_resistance,
            };
            self.PI_current_d.init();
            self.PI_current_q.init();

            self.PI_current_d.set_lpf(5 * self.current_bandwidth);
            self.PI_current_q.set_lpf(5 * self.current_bandwidth);
        }

        fn do_hfi(self: *@This()) void {
            bsp.benchmark_toggle();

            if ((self.hfi_on == false) or (self.enabled == false)) {
                self.hfi_firstcycle = true;
                self.hfi_int = 0;
                self.hfi_out = 0;
                self.hfi_full_turns = 0;
                // stm32.GPIOC.ODR.modify(.{ .ODR14 = 0 });
                return;
            }

            if (!silent_hfi) {
                const is_v0 = self.getPwmState();
                if (!is_v0) {
                    self.setpwm(self.Ua, self.Ub, self.Uc);
                    bsp.benchmark_toggle();
                    bsp.benchmark_toggle();
                    return;
                }
            }
            var center: f32 = undefined;
            var voltage_pid: DQVoltage_s = undefined;
            // var current_err: DQCurrent_s = undefined;
            var _ca: f32 = undefined;
            var _sa: f32 = undefined;
            // var hfi_v_act:f32 = undefined;
            approx._sincos(self.electrical_angle, &_sa, &_ca);

            const phase_current: PhaseCurrent_s = self.getPhaseCurrents();
            const ABcurrent: ABCurrent_s = getABCurrents(phase_current);
            self.current_meas.d = ABcurrent.alpha * _ca + ABcurrent.beta * _sa;
            self.current_meas.q = ABcurrent.beta * _ca - ABcurrent.alpha * _sa;

            var hfi_v_act = self.hfi_v;

            if (self.hfi_firstcycle) {
                hfi_v_act /= 2.0;
                self.hfi_firstcycle = false;
            }

            if (self.hfi_high) {
                self.current_high = self.current_meas;
            } else {
                self.current_low = self.current_meas;
                hfi_v_act = -1.0 * self.hfi_v;
            }

            self.hfi_high = !self.hfi_high;

            self.delta_current.q = self.current_high.q - self.current_low.q;
            self.delta_current.d = self.current_high.d - self.current_low.d;

            // hfi_curangleest = delta_current.q / (hfi_v * Ts_L );  // this is about half a us faster than vv
            self.hfi_curangleest = 0.5 * self.delta_current.q / (self.hfi_v * self.Ts * (1.0 / self.Lq - 1.0 / self.Ld));
            if (self.hfi_curangleest > self.error_saturation_limit) self.hfi_curangleest = self.error_saturation_limit;
            if (self.hfi_curangleest < -self.error_saturation_limit) self.hfi_curangleest = -self.error_saturation_limit;
            self.hfi_error = -self.hfi_curangleest;
            self.hfi_int += self.Ts * self.hfi_error * self.hfi_gain2; //This the the double integrator
            self.hfi_out += self.hfi_gain1 * self.Ts * self.hfi_error + self.hfi_int; //This is the integrator and the double integrator

            self.current_err.q = self.current_setpoint.q - self.current_meas.q;
            self.current_err.d = self.current_setpoint.d - self.current_meas.d;

            // PID contains the low pass filter
            voltage_pid.d = self.PI_current_d.calc(self.current_err.d);
            voltage_pid.q = self.PI_current_q.calc(self.current_err.q);

            self.voltage.d = voltage_pid.d;
            self.voltage.q = voltage_pid.q;

            self.voltage.d += hfi_v_act;

            // Inverse park transform
            self.Ualpha = _ca * self.voltage.d - _sa * self.voltage.q; // -sin(angle) * Uq;
            self.Ubeta = _sa * self.voltage.d + _ca * self.voltage.q; //  cos(angle) * Uq;

            // Clarke transform
            self.Ua = self.Ualpha;
            self.Ub = -0.5 * self.Ualpha + approx._SQRT3_2 * self.Ubeta;
            self.Uc = -0.5 * self.Ualpha - approx._SQRT3_2 * self.Ubeta;

            center = self.voltage_limit / 2.0;
            const Umin = approx.fmin(self.Ua, approx.fmin(self.Ub, self.Uc));
            const Umax = approx.fmax(self.Ua, approx.fmax(self.Ub, self.Uc));
            center -= (Umax + Umin) / 2.0;

            self.Ua += center;
            self.Ub += center;
            self.Uc += center;

            if (silent_hfi) {
                self.setpwm(self.Ua, self.Ub, self.Uc);
            }

            while (self.hfi_out < 0) {
                self.hfi_out += approx._2PI;
            }
            while (self.hfi_out >= approx._2PI) {
                self.hfi_out -= approx._2PI;
            }

            while (self.hfi_int < -approx._PI) {
                self.hfi_int += approx._2PI;
            }
            while (self.hfi_int >= approx._PI) {
                self.hfi_int -= approx._2PI;
            }

            const d_angle = self.hfi_out - self.electrical_angle;
            if (@abs(d_angle) > (0.8 * approx._2PI)) {
                if (d_angle > 0.0) {
                    self.hfi_full_turns += -1.0;
                } else {
                    self.hfi_full_turns += 1.0;
                }
            }

            self.electrical_angle = self.hfi_out;

            bsp.benchmark_toggle();
            bsp.benchmark_toggle();
        }

        fn setpwm(self: @This(), Ua: f32, Ub: f32, Uc: f32) void {
            const Ua_c = constrain(Ua, self.voltage_limit, -self.voltage_limit);
            const Ub_c = constrain(Ub, self.voltage_limit, -self.voltage_limit);
            const Uc_c = constrain(Uc, self.voltage_limit, -self.voltage_limit);

            const arr: f32 = @floatFromInt(self.tim.ARR.read().ARR);
            const dc_a = constrain(Ua_c / self.voltage_power_supply, 1.0, 0.0);
            const dc_b = constrain(Ub_c / self.voltage_power_supply, 1.0, 0.0);
            const dc_c = constrain(Uc_c / self.voltage_power_supply, 1.0, 0.0);
            self.tim.CCR1.modify(.{ .CCR1 = @as(u16, @intFromFloat(dc_a * arr)) });
            self.tim.CCR2.modify(.{ .CCR2 = @as(u16, @intFromFloat(dc_b * arr)) });
            self.tim.CCR3.modify(.{ .CCR3 = @as(u16, @intFromFloat(dc_c * arr)) });
        }

        inline fn constrain(val: f32, upper_limit: f32, lower_limit: f32) f32 {
            var out = val;
            if (val > upper_limit) {
                out = upper_limit;
            } else if (val < lower_limit) {
                out = lower_limit;
            }
            return out;
        }

        const D = Driver(@This(), setpwm);
        pub fn driver(mot: @This()) D {
            return .{ .mot = mot };
        }
    };
}

const M1 = BuildMotor(@TypeOf(stm32.TIM1));
var motor_1 = M1{ .tim = stm32.TIM1 };

pub fn panic(message: []const u8, _: ?*std.builtin.StackTrace, _: ?usize) noreturn {

    // put whatever you want here
    bsp.init_uart();
    for (message) |chr| {
        bsp.tx(chr);
    }
    bsp.tx('\r');
    bsp.tx('\n');
    @breakpoint();
    while (true) {}
}

fn HardFault() callconv(.C) void {
    @panic("We got a hard fault!");
}

fn ADC_ISR() callconv(.C) void {
    motor_1.do_hfi();
    bsp.clear_adc_isr_flag();
}

pub const microzig_options: microzig.Options = .{ //
    .interrupts = .{
        .HardFault = Handler{ .c = &HardFault },
        .ADC = Handler{ .c = &ADC_ISR },
    },
    .logFn = bsp.log,
};

// pub const std_options = .{ .log_level = .info, .logFn = bsp.log };

pub fn enable_interrupt(IRQn: stm32.IRQn_Type) void {
    //     // __COMPILER_BARRIER();
    //     // NVIC->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << ( ( (uint32_t)IRQn) & 0x1FUL));
    //     // __COMPILER_BARRIER();
    asm volatile ("" ::: "memory");
    const idx = @as(u32, @intFromEnum(IRQn) >> 5);
    const shift: u5 = @intCast(@intFromEnum(IRQn) & 0x1F);
    switch (idx) {
        // writing zeros to this register has no effect, so we just write a 1 where we want it
        0 => {
            stm32.peripherals.NVIC.ISER0.raw = @as(u32, 1) << shift;
        },
        1 => {
            stm32.peripherals.NVIC.ISER1.raw = @as(u32, 1) << shift;
        },
        2 => {
            stm32.peripherals.NVIC.ISER2.raw = @as(u32, 1) << shift;
        },
        else => unreachable,
    }
    asm volatile ("" ::: "memory");
}

pub fn disable_interrupt(IRQn: stm32.IRQn_Type) void {
    asm volatile ("" ::: "memory");
    const idx = @as(u32, @intFromEnum(IRQn) >> 5);
    const shift: u5 = @intCast(@intFromEnum(IRQn) & 0x1F);
    switch (idx) {
        // writing zeros to this register has no effect, so we just write a 1 where we want it
        0 => {
            stm32.peripherals.NVIC.ICER0.raw = @as(u32, 1) << shift;
        },
        1 => {
            stm32.peripherals.NVIC.ICER1.raw = @as(u32, 1) << shift;
        },
        2 => {
            stm32.peripherals.NVIC.ICER2.raw = @as(u32, 1) << shift;
        },
        else => unreachable,
    }
    asm volatile ("" ::: "memory");
}

pub fn main() !void {
    stm32.peripherals.FPU_CPACR.CPACR.modify(.{ .CP = 0b1111 });
    // stm32.cpu.peripherals.SysTick.CTRL.modify(.{ .ENABLE = 1 });
    bsp.init_rcc();
    itm.enable_itm(84000000, 2000000);

    bsp.init_gpio();
    bsp.init_uart();
    for ("\r\nuart active\r\n") |chr| {
        bsp.tx(chr);
    }
    // logger.info("UART up\n", .{});

    motor_1.init();
    motor_1.enabled = true;
    motor_1.hfi_on = true;
    var state = false;
    for ("starting loop\r\n") |chr| {
        bsp.tx(chr);
    }
    // const printer: itm.Writer = .{ .context = 0 };
    // logger.info("Starting loop!\n", .{});
    // try printer.print("Starting loop!\n", .{});
    // itm.static_print("print static starting Loop!\n");

    // ADC_IRQn = 18
    enable_interrupt(stm32.IRQn_Type.ADC_IRQn);
    stm32.cpu.interrupt.enable_interrupts();

    motor_1.current_setpoint = .{ .q = 0.0 };

    while (true) {
        var i: u32 = 0;
        while (i < 5_000_000) {
            asm volatile ("nop");
            i += 1;
        }

        if (state) {
            // motor_1.enabled = true;
            motor_1.current_setpoint = .{ .q = 5.0 };
            state = false;
        } else {
            motor_1.current_setpoint = .{ .q = -5.0 };
            // motor_1.enabled = false;
            state = true;
        }
        // std.log.info("el: {d:3}", .{motor_1.electrical_angle});
        // const phase_currents = motor_1.getPhaseCurrents();
        // const adc_vals = bsp.get_adc_vals();

        std.log.info("el: {d:3}", .{motor_1.electrical_angle});
        // std.log.info("phA current: {d:3}", .{phase_currents.a});
        // std.log.info("adc[0]: {}", .{adc_vals[0]});
        // printer.print("printing from printer!\n", .{}) catch {};
        // itm.static_print("print static Loop!\n");
        // const float_bytes: [4]u8 = @bitCast(motor_1.current_meas.d);
        // for (float_bytes) |chr| {
        //     bsp.tx(chr);
        // }
        // bsp.tx('1');
        // bsp.tx('\n');
        // logger.debug("current: {}!", motor_1.current_setpoint.q);
        // motor_1.do_hfi();
    }
}
