const std = @import("std");
const microzig = @import("microzig");
const builtin = @import("builtin");

const stm32 = microzig.hal;

pub inline fn init_uart() void {
    stm32.RCC.APB1ENR.modify(.{ .USART2EN = 1 });
    // stm32.USART2.CR1.write_raw(0);

    const usartdiv = @as(u16, @intCast(@divTrunc(45_000_000, 115_200)));
    stm32.USART2.BRR.write_raw(@as(u32, usartdiv));
    stm32.USART2.CR1.modify(.{ //
        .UE = 1,
        .RE = 1,
        .TE = 1,
    });

    stm32.GPIOA.MODER.modify(.{ .MODER2 = 0b10 });
    stm32.GPIOA.MODER.modify(.{ .MODER3 = 0b10 });

    stm32.GPIOA.OTYPER.modify(.{ .OT2 = 0 });
    stm32.GPIOA.OTYPER.modify(.{ .OT3 = 0 });

    stm32.GPIOA.OSPEEDR.modify(.{ .OSPEEDR2 = 0b11 });
    stm32.GPIOA.OSPEEDR.modify(.{ .OSPEEDR3 = 0b11 });

    stm32.GPIOA.AFRL.modify(.{ .AFRL2 = 7 });
    stm32.GPIOA.AFRL.modify(.{ .AFRL3 = 7 });
}

pub fn tx(ch: u8) void {
    while (!(stm32.USART2.SR.read().TXE == 1)) {} // Wait for Previous transmission
    stm32.USART2.DR.modify(.{ .DR = @as(u9, ch) });
}

pub inline fn init_gpio() void {
    stm32.RCC.AHB1ENR.modify(.{ .GPIOAEN = 1 });
    // TIM1 outputs -> PA8, PA9, PA10
    stm32.GPIOA.MODER.modify(.{ .MODER8 = 0b10 });
    stm32.GPIOA.MODER.modify(.{ .MODER9 = 0b10 });
    stm32.GPIOA.MODER.modify(.{ .MODER10 = 0b10 });

    // push/pull
    stm32.GPIOA.OTYPER.modify(.{ .OT8 = 0 });
    stm32.GPIOA.OTYPER.modify(.{ .OT9 = 0 });
    stm32.GPIOA.OTYPER.modify(.{ .OT10 = 0 });

    // fastest
    stm32.GPIOA.OSPEEDR.modify(.{ .OSPEEDR8 = 0b11 });
    stm32.GPIOA.OSPEEDR.modify(.{ .OSPEEDR9 = 0b11 });
    stm32.GPIOA.OSPEEDR.modify(.{ .OSPEEDR10 = 0b11 });

    // AF2
    stm32.GPIOA.AFRH.modify(.{ .AFRH8 = 1 });
    stm32.GPIOA.AFRH.modify(.{ .AFRH9 = 1 });
    stm32.GPIOA.AFRH.modify(.{ .AFRH10 = 1 });

    // PC14 - timing pulses for benchmarking
    stm32.RCC.AHB1ENR.modify(.{ .GPIOCEN = 1 });
    stm32.GPIOC.MODER.modify(.{ .MODER14 = 0b01 });
    stm32.GPIOC.OSPEEDR.modify(.{ .OSPEEDR14 = 0b11 });

    stm32.RCC.AHB1ENR.modify(.{ .GPIOBEN = 1 });
    // PB13, PB14, PB15 - connected to LS PWM pins, but disabled for now, using 3pwm
    stm32.GPIOB.MODER.modify(.{ //
        .MODER13 = 0b01,
        .MODER14 = 0b01,
        .MODER15 = 0b01,
    });
    stm32.GPIOB.OSPEEDR.modify(.{ //
        .OSPEEDR13 = 0b01,
        .OSPEEDR14 = 0b01,
        .OSPEEDR15 = 0b01,
    });
    stm32.GPIOB.ODR.modify(.{ //
        .ODR13 = 0,
        .ODR14 = 0,
        .ODR15 = 0,
    });

    // PA2 - OC_ADJ - high->max current
    // PA5 - M_PWM - HIGH->3pwm, LOW->6pwm
    // PA6 - GAIN - LOW->10x gain, HIGH->40x gain
    stm32.GPIOA.MODER.modify(.{ //
        .MODER2 = 0b01,
        .MODER5 = 0b01,
        .MODER6 = 0b01,
    });
    stm32.GPIOA.OSPEEDR.modify(.{ //
        .OSPEEDR2 = 0b01,
        .OSPEEDR5 = 0b01,
        .OSPEEDR6 = 0b01,
    });
    stm32.GPIOB.ODR.modify(.{ //
        .ODR2 = 1,
        .ODR5 = 0,
        .ODR6 = 0,
    });
}

inline fn wait_for_flag(register: anytype, field: anytype) void {
    while (true) {
        const cr = register.read();
        const val = @field(cr, field);
        if (val == 1) break;
    }
}

pub inline fn init_rcc() void {
    // verify we're using HSI with no PLL

    stm32.RCC.APB1ENR.modify(.{ .PWREN = 1 });
    // set voltage scale 2
    stm32.PWR.CR.modify(.{ .VOS = 0b10 });

    // stm32.RCC.CFGR.modify(.{ .MCO1PRE = 0 });
    // stm32.RCC.CFGR.modify(.{ .MCO1 = 0b11 });

    // if using USB, we should use HSE instead of HSI

    // HSI ON
    stm32.RCC.CR.modify(.{ .HSION = 1 });
    // PLLON = 0
    stm32.RCC.CR.modify(.{ .PLLON = 0 });

    // PLLM =  /16 - min value of 2, max 63
    const PLLM: u6 = 25;
    stm32.RCC.PLLCFGR.modify(.{
        .PLLM0 = @as(u1, PLLM & 0b1),
        .PLLM1 = @as(u1, (PLLM >> 1) & 0b1),
        .PLLM2 = @as(u1, (PLLM >> 2) & 0b1),
        .PLLM3 = @as(u1, (PLLM >> 3) & 0b1),
        .PLLM4 = @as(u1, (PLLM >> 4) & 0b1),
        .PLLM5 = @as(u1, (PLLM >> 5) & 0b1),
    }); //

    // PLLN = x360
    const PLLN: u9 = 336;
    stm32.RCC.PLLCFGR.modify(.{
        .PLLN0 = @as(u1, PLLN & 0b1),
        .PLLN1 = @as(u1, (PLLN >> 1) & 0b1),
        .PLLN2 = @as(u1, (PLLN >> 2) & 0b1),
        .PLLN3 = @as(u1, (PLLN >> 3) & 0b1),
        .PLLN4 = @as(u1, (PLLN >> 4) & 0b1),
        .PLLN5 = @as(u1, (PLLN >> 5) & 0b1),
        .PLLN6 = @as(u1, (PLLN >> 6) & 0b1),
        .PLLN7 = @as(u1, (PLLN >> 7) & 0b1),
        .PLLN8 = @as(u1, (PLLN >> 8) & 0b1),
    }); //

    // var pllcfgr = stm32.RCC.PLLCFGR.raw;
    // // pllcfgr &= ~@as(u32, PLLM);
    // // pllcfgr |= @as(u32, PLLM);

    // // pllcfgr &= ~@as(u32, PLLN << 6);
    // // pllcfgr |= @as(u32, PLLN << 6);

    // pllcfgr &= ~@as(u32, 16);
    // pllcfgr |= @as(u32, 16);

    // pllcfgr &= ~@as(u32, (@as(u32, 360) << 6));
    // pllcfgr |= @as(u32, (@as(u32, 360) << 6));

    // pllcfgr &= ~@as(u32, (@as(u32, 2) << 16));
    // pllcfgr |= @as(u32, (@as(u32, 2) << 16));
    // stm32.RCC.PLLCFGR.write_raw(pllcfgr);

    // PLLP = /2 - 0 -> 2, 1 -> 4, 2 -> 6
    const PLLP: u2 = 1;
    stm32.RCC.PLLCFGR.modify(.{ //
        .PLLP0 = @as(u1, PLLP & 0b1), //
        .PLLP1 = @as(u1, (PLLP >> 1) & 0b1), //
    });

    // PLLQ should be set to 7 for USB use

    // AHB Prescaler = 1
    stm32.RCC.CFGR.modify(.{ .HPRE = 0b0000 });

    // APB1 Prescaler = 2 - 0b100->2, 0b101->4
    stm32.RCC.CFGR.modify(.{ .PPRE1 = 0b100 });

    // APB2 Prescaler = 4
    stm32.RCC.CFGR.modify(.{ .PPRE2 = 0b100 });

    // wait for HSI Ready
    wait_for_flag(&stm32.RCC.CR, "HSIRDY");

    // now it's safe to change the sys clock source
    // PLL Source Mux = HSI
    stm32.RCC.PLLCFGR.modify(.{ .PLLSRC = 0 });

    // PLLON = 1
    stm32.RCC.CR.modify(.{ .PLLON = 1 });

    // set voltage mode - no Overdrive mode on f401
    // stm32.PWR.CR.modify(.{ .ODEN = 1 });
    // wait_for_flag(&stm32.PWR.CSR, "ODRDY");
    // stm32.PWR.CR.modify(.{ .ODSWEN = 1 });
    // wait_for_flag(&stm32.PWR.CSR, "ODSWRDY");

    // set flash wait states and ART prefetch
    stm32.FLASH.ACR.modify(.{ //
        .LATENCY = 2,
        .PRFTEN = 1,
        .ICEN = 1,
        .DCEN = 1,
    });

    // wait for PLL Lock
    wait_for_flag(&stm32.RCC.CR, "PLLRDY");

    // sys clock mux = PLLCLK
    stm32.RCC.CFGR.modify(.{ .SW0 = 0 });
    stm32.RCC.CFGR.modify(.{ .SW1 = 1 });

    // f401 has the TIMPRE bit, but the svd is wrong so it doesn't show up
    // pg 95 of rm0368 rev5
    // stm32.RCC.DCKCFGR.modify(.{ .TIMPRE = 1 });
}

pub fn init_adc() void {
    stm32.RCC.APB2ENR.modify(.{ .ADC1EN = 1 });
    stm32.ADC1.CR1.modify(.{ //
        .SCAN = 1,
        .JEOCIE = 1,
    }); // enable interrupt for injected channels
    // extern trigger rising - JEXTEN->0b01
    // tim1 trgo - JEXTSEL->0b0001
    stm32.ADC1.CR2.modify(.{
        .ADON = 1, //
        .CONT = 0,
        .JEXTSEL = 0b0001,
        .JEXTEN = 1,
    });
    // set sequence in ADC_JSQR
    stm32.ADC1.JSQR.modify(.{ //
        .JL = 1,
        .JSQ4 = 0, //pg358 of ref manual, fill as 4-3-2-1
        .JSQ3 = 1, //pg358 of ref manual, fill as 4-3-2-1
    });

    // sampling time 3 cycles - SMPx -> 0b0
    // not actually setting anything right now
    // move to func and set more later
    const sample_time = 0b0;
    const chan = 3;
    if (chan > 9) {
        var smpr1_val: u32 = stm32.ADC1.SMPR1.raw;
        smpr1_val &= ~@as(u32, @intCast(sample_time << ((chan - 9) * sample_time)));
        smpr1_val |= @as(u32, @intCast(sample_time << ((chan - 9) * sample_time)));
        stm32.ADC1.SMPR1.write_raw(smpr1_val);
    } else {
        var smpr2_val: u32 = stm32.ADC1.SMPR2.raw;
        smpr2_val &= ~@as(u32, @intCast(sample_time << (chan * sample_time)));
        smpr2_val |= @as(u32, @intCast(sample_time << (chan * sample_time)));
        stm32.ADC1.SMPR2.write_raw(smpr2_val);
    }

    stm32.GPIOA.MODER.modify(.{ //
        .MODER0 = 0b11,
        .MODER1 = 0b11,
    });

    stm32.GPIOA.OTYPER.modify(.{ //
        .OT0 = 0,
        .OT1 = 0,
    });

    // maybe use JOFFSETx for setting offset

    //
}

pub inline fn get_adc_vals() [2]u16 {
    const adc1_val = stm32.ADC1.JDR1.read().JDATA;
    const adc2_val = stm32.ADC1.JDR2.read().JDATA;

    return [2]u16{ adc1_val, adc2_val };
}

pub inline fn clear_adc_isr_flag() void {
    stm32.ADC1.SR.modify(.{ .JEOC = 0 });
}

pub inline fn benchmark_toggle() void {
    stm32.GPIOC.ODR.modify(.{ .ODR14 = 1 });
    stm32.GPIOC.ODR.modify(.{ .ODR14 = 0 });
}
