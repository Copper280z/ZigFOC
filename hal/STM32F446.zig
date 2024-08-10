//! this is mostly a copy/paste from F407, so nothing really works

const std = @import("std");
const micro = @import("microzig");
pub const cpu = micro.cpu;
pub const interrupt = micro.interrupt;
pub const chip = micro.chip;
pub const peripherals = micro.chip.peripherals;
pub const GPIOA = peripherals.GPIOA;
pub const GPIOB = peripherals.GPIOB;
pub const GPIOC = peripherals.GPIOC;
pub const TIM1 = peripherals.TIM1;
pub const RCC = peripherals.RCC;
pub const FLASH = peripherals.FLASH;
pub const PWR = peripherals.PWR;
pub const ADC1 = peripherals.ADC1;
pub const ADC2 = peripherals.ADC2;
pub const ADC3 = peripherals.ADC3;
pub const C_ADC = peripherals.C_ADC;
pub const USART2 = peripherals.USART2;
pub const DBG = peripherals.DBG;

pub const IRQn_Type = enum(u8) {
    //******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
    // NonMaskableInt_IRQn = -14, //*!< 2 Non Maskable Interrupt                                          */
    // MemoryManagement_IRQn = -12, //*!< 4 Cortex-M4 Memory Management Interrupt                           */
    // BusFault_IRQn = -11, //*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
    // UsageFault_IRQn = -10, //*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
    // SVCall_IRQn = -5, //*!< 11 Cortex-M4 SV Call Interrupt                                    */
    // DebugMonitor_IRQn = -4, //*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
    // PendSV_IRQn = -2, //*!< 14 Cortex-M4 Pend SV Interrupt                                    */
    // SysTick_IRQn = -1, //*!< 15 Cortex-M4 System Tick Interrupt                                */
    //******  STM32 specific Interrupt Num/bers **********************************************************************/
    WWDG_IRQn = 0, //*!< Window WatchDog Interrupt                                         */
    PVD_IRQn = 1, //*!< PVD through EXTI Line detection Interrupt                         */
    TAMP_STAMP_IRQn = 2, //*!< Tamper and TimeStamp interrupts through the EXTI line             */
    RTC_WKUP_IRQn = 3, //*!< RTC Wakeup interrupt through the EXTI line                        */
    FLASH_IRQn = 4, //*!< FLASH global Interrupt                                            */
    RCC_IRQn = 5, //*!< RCC global Interrupt                                              */
    EXTI0_IRQn = 6, //*!< EXTI Line0 Interrupt                                              */
    EXTI1_IRQn = 7, //*!< EXTI Line1 Interrupt                                              */
    EXTI2_IRQn = 8, //*!< EXTI Line2 Interrupt                                              */
    EXTI3_IRQn = 9, //*!< EXTI Line3 Interrupt                                              */
    EXTI4_IRQn = 10, //*!< EXTI Line4 Interrupt                                              */
    DMA1_Stream0_IRQn = 11, //*!< DMA1 Stream 0 global Interrupt                                    */
    DMA1_Stream1_IRQn = 12, //*!< DMA1 Stream 1 global Interrupt                                    */
    DMA1_Stream2_IRQn = 13, //*!< DMA1 Stream 2 global Interrupt                                    */
    DMA1_Stream3_IRQn = 14, //*!< DMA1 Stream 3 global Interrupt                                    */
    DMA1_Stream4_IRQn = 15, //*!< DMA1 Stream 4 global Interrupt                                    */
    DMA1_Stream5_IRQn = 16, //*!< DMA1 Stream 5 global Interrupt                                    */
    DMA1_Stream6_IRQn = 17, //*!< DMA1 Stream 6 global Interrupt                                    */
    ADC_IRQn = 18, //*!< ADC1, ADC2 and ADC3 global Interrupts                             */
    CAN1_TX_IRQn = 19, //*!< CAN1 TX Interrupt                                                 */
    CAN1_RX0_IRQn = 20, //*!< CAN1 RX0 Interrupt                                                */
    CAN1_RX1_IRQn = 21, //*!< CAN1 RX1 Interrupt                                                */
    CAN1_SCE_IRQn = 22, //*!< CAN1 SCE Interrupt                                                */
    EXTI9_5_IRQn = 23, //*!< External Line[9:5] Interrupts                                     */
    TIM1_BRK_TIM9_IRQn = 24, //*!< TIM1 Break interrupt and TIM9 global interrupt                    */
    TIM1_UP_TIM10_IRQn = 25, //*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
    TIM1_TRG_COM_TIM11_IRQn = 26, //*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
    TIM1_CC_IRQn = 27, //*!< TIM1 Capture Compare Interrupt                                    */
    TIM2_IRQn = 28, //*!< TIM2 global Interrupt                                             */
    TIM3_IRQn = 29, //*!< TIM3 global Interrupt                                             */
    TIM4_IRQn = 30, //*!< TIM4 global Interrupt                                             */
    I2C1_EV_IRQn = 31, //*!< I2C1 Event Interrupt                                              */
    I2C1_ER_IRQn = 32, //*!< I2C1 Error Interrupt                                              */
    I2C2_EV_IRQn = 33, //*!< I2C2 Event Interrupt                                              */
    I2C2_ER_IRQn = 34, //*!< I2C2 Error Interrupt                                              */
    SPI1_IRQn = 35, //*!< SPI1 global Interrupt                                             */
    SPI2_IRQn = 36, //*!< SPI2 global Interrupt                                             */
    USART1_IRQn = 37, //*!< USART1 global Interrupt                                           */
    USART2_IRQn = 38, //*!< USART2 global Interrupt                                           */
    USART3_IRQn = 39, //*!< USART3 global Interrupt                                           */
    EXTI15_10_IRQn = 40, //*!< External Line[15:10] Interrupts                                   */
    RTC_Alarm_IRQn = 41, //*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
    OTG_FS_WKUP_IRQn = 42, //*!< USB OTG FS Wakeup through EXTI line interrupt                     */
    TIM8_BRK_TIM12_IRQn = 43, //*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
    TIM8_UP_TIM13_IRQn = 44, //*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
    TIM8_TRG_COM_TIM14_IRQn = 45, //*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
    TIM8_CC_IRQn = 46, //*!< TIM8 Capture Compare global interrupt                             */
    DMA1_Stream7_IRQn = 47, //*!< DMA1 Stream7 Interrupt                                            */
    FMC_IRQn = 48, //*!< FMC global Interrupt                                              */
    SDIO_IRQn = 49, //*!< SDIO global Interrupt                                             */
    TIM5_IRQn = 50, //*!< TIM5 global Interrupt                                             */
    SPI3_IRQn = 51, //*!< SPI3 global Interrupt                                             */
    UART4_IRQn = 52, //*!< UART4 global Interrupt                                            */
    UART5_IRQn = 53, //*!< UART5 global Interrupt                                            */
    TIM6_DAC_IRQn = 54, //*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
    TIM7_IRQn = 55, //*!< TIM7 global interrupt                                             */
    DMA2_Stream0_IRQn = 56, //*!< DMA2 Stream 0 global Interrupt                                    */
    DMA2_Stream1_IRQn = 57, //*!< DMA2 Stream 1 global Interrupt                                    */
    DMA2_Stream2_IRQn = 58, //*!< DMA2 Stream 2 global Interrupt                                    */
    DMA2_Stream3_IRQn = 59, //*!< DMA2 Stream 3 global Interrupt                                    */
    DMA2_Stream4_IRQn = 60, //*!< DMA2 Stream 4 global Interrupt                                    */
    CAN2_TX_IRQn = 63, //*!< CAN2 TX Interrupt                                                 */
    CAN2_RX0_IRQn = 64, //*!< CAN2 RX0 Interrupt                                                */
    CAN2_RX1_IRQn = 65, //*!< CAN2 RX1 Interrupt                                                */
    CAN2_SCE_IRQn = 66, //*!< CAN2 SCE Interrupt                                                */
    OTG_FS_IRQn = 67, //*!< USB OTG FS global Interrupt                                       */
    DMA2_Stream5_IRQn = 68, //*!< DMA2 Stream 5 global interrupt                                    */
    DMA2_Stream6_IRQn = 69, //*!< DMA2 Stream 6 global interrupt                                    */
    DMA2_Stream7_IRQn = 70, //*!< DMA2 Stream 7 global interrupt                                    */
    USART6_IRQn = 71, //*!< USART6 global interrupt                                           */
    I2C3_EV_IRQn = 72, //*!< I2C3 event interrupt                                              */
    I2C3_ER_IRQn = 73, //*!< I2C3 error interrupt                                              */
    OTG_HS_EP1_OUT_IRQn = 74, //*!< USB OTG HS End Point 1 Out global interrupt                       */
    OTG_HS_EP1_IN_IRQn = 75, //*!< USB OTG HS End Point 1 In global interrupt                        */
    OTG_HS_WKUP_IRQn = 76, //*!< USB OTG HS Wakeup through EXTI interrupt                          */
    OTG_HS_IRQn = 77, //*!< USB OTG HS global interrupt                                       */
    DCMI_IRQn = 78, //*!< DCMI global interrupt                                             */
    FPU_IRQn = 81, //*!< FPU global interrupt                                              */
    SPI4_IRQn = 84, //*!< SPI4 global Interrupt                                             */
    SAI1_IRQn = 87, //*!< SAI1 global Interrupt                                             */
    SAI2_IRQn = 91, //*!< SAI2 global Interrupt                                             */
    QUADSPI_IRQn = 92, //*!< QuadSPI global Interrupt                                          */
    CEC_IRQn = 93, //*!< CEC global Interrupt                                              */
    SPDIF_RX_IRQn = 94, //*!< SPDIF-RX global Interrupt                                          */
    FMPI2C1_EV_IRQn = 95, //*!< FMPI2C1 Event Interrupt                                           */
    FMPI2C1_ER_IRQn = 96, //*!< FMPI2C1 Error Interrupt                                           */
};

// pub const clock = struct {/
//     pub const Domain = enum {
//         cpu,
//         ahb,
//         apb1,
//         apb2,
//     };
// };

// // Default clock frequencies after reset, see top comment for calculation
// pub const clock_frequencies = .{
//     .cpu = 16_000_000,
//     .ahb = 16_000_000,
//     .apb1 = 16_000_000,
//     .apb2 = 16_000_000,
// };

// pub fn parse_pin(comptime spec: []const u8) type {
//     const invalid_format_msg = "The given pin '" ++ spec ++ "' has an invalid format. Pins must follow the format \"P{Port}{Pin}\" scheme.";

//     if (spec[0] != 'P')
//         @compileError(invalid_format_msg);
//     if (spec[1] < 'A' or spec[1] > 'I')
//         @compileError(invalid_format_msg);

//     return struct {
//         const pin_number: comptime_int = std.fmt.parseInt(u4, spec[2..], 10) catch @compileError(invalid_format_msg);
//         /// 'A'...'I'
//         const gpio_port_name = spec[1..2];
//         const gpio_port = @field(peripherals, "GPIO" ++ gpio_port_name);
//         const suffix = std.fmt.comptimePrint("{d}", .{pin_number});
//     };
// }

// fn set_reg_field(reg: anytype, comptime field_name: anytype, value: anytype) void {
//     var temp = reg.read();
//     @field(temp, field_name) = value;
//     reg.write(temp);
// }

// pub const gpio = struct {
//     pub const AlternateFunction = enum(u4) {
//         af0,
//         af1,
//         af2,
//         af3,
//         af4,
//         af5,
//         af6,
//         af7,
//         af8,
//         af9,
//         af10,
//         af11,
//         af12,
//         af13,
//         af14,
//         af15,
//     };

//     pub fn set_output(comptime pin: type) void {
//         set_reg_field(RCC.AHB1ENR, "GPIO" ++ pin.gpio_port_name ++ "EN", 1);
//         set_reg_field(@field(pin.gpio_port, "MODER"), "MODER" ++ pin.suffix, 0b01);
//     }

//     pub fn set_input(comptime pin: type) void {
//         set_reg_field(RCC.AHB1ENR, "GPIO" ++ pin.gpio_port_name ++ "EN", 1);
//         set_reg_field(@field(pin.gpio_port, "MODER"), "MODER" ++ pin.suffix, 0b00);
//     }

//     pub fn set_alternate_function(comptime pin: type, af: AlternateFunction) void {
//         set_reg_field(RCC.AHB1ENR, "GPIO" ++ pin.gpio_port_name ++ "EN", 1);
//         set_reg_field(@field(pin.gpio_port, "MODER"), "MODER" ++ pin.suffix, 0b10);
//         if (pin.pin_number < 8) {
//             set_reg_field(@field(pin.gpio_port, "AFRL"), "AFRL" ++ pin.suffix, @intFromEnum(af));
//         } else {
//             set_reg_field(@field(pin.gpio_port, "AFRH"), "AFRH" ++ pin.suffix, @intFromEnum(af));
//         }
//     }

//     pub fn read(comptime pin: type) micro.core.experimental.gpio.State {
//         const idr_reg = pin.gpio_port.IDR;
//         const reg_value = @field(idr_reg.read(), "IDR" ++ pin.suffix); // TODO extract to getRegField()?
//         return @as(micro.gpio.State, @enumFromInt(reg_value));
//     }

//     pub fn write(comptime pin: type, state: micro.core.experimental.gpio.State) void {
//         switch (state) {
//             .low => set_reg_field(pin.gpio_port.BSRR, "BR" ++ pin.suffix, 1),
//             .high => set_reg_field(pin.gpio_port.BSRR, "BS" ++ pin.suffix, 1),
//         }
//     }
// };

// pub const uart = struct {
//     pub const DataBits = enum {
//         seven,
//         eight,
//         nine,
//     };

//     /// uses the values of USART_CR2.STOP
//     pub const StopBits = enum(u2) {
//         one = 0b00,
//         half = 0b01,
//         two = 0b10,
//         one_and_half = 0b11,
//     };

//     /// uses the values of USART_CR1.PS
//     pub const Parity = enum(u1) {
//         even = 0,
//         odd = 1,
//     };

//     const PinDirection = std.meta.FieldEnum(micro.uart.Pins);

//     /// Checks if a pin is valid for a given uart index and direction
//     pub fn is_valid_pin(comptime pin: type, comptime index: usize, comptime direction: PinDirection) bool {
//         const pin_name = pin.name;

//         return switch (direction) {
//             .tx => switch (index) {
//                 1 => std.mem.eql(u8, pin_name, "PA9") or std.mem.eql(u8, pin_name, "PB6"),
//                 2 => std.mem.eql(u8, pin_name, "PA2") or std.mem.eql(u8, pin_name, "PD5"),
//                 3 => std.mem.eql(u8, pin_name, "PB10") or std.mem.eql(u8, pin_name, "PC10") or std.mem.eql(u8, pin_name, "PD8"),
//                 4 => std.mem.eql(u8, pin_name, "PA0") or std.mem.eql(u8, pin_name, "PC10"),
//                 5 => std.mem.eql(u8, pin_name, "PC12"),
//                 6 => std.mem.eql(u8, pin_name, "PC6") or std.mem.eql(u8, pin_name, "PG14"),
//                 else => unreachable,
//             },
//             // Valid RX pins for the UARTs
//             .rx => switch (index) {
//                 1 => std.mem.eql(u8, pin_name, "PA10") or std.mem.eql(u8, pin_name, "PB7"),
//                 2 => std.mem.eql(u8, pin_name, "PA3") or std.mem.eql(u8, pin_name, "PD6"),
//                 3 => std.mem.eql(u8, pin_name, "PB11") or std.mem.eql(u8, pin_name, "PC11") or std.mem.eql(u8, pin_name, "PD9"),
//                 4 => std.mem.eql(u8, pin_name, "PA1") or std.mem.eql(u8, pin_name, "PC11"),
//                 5 => std.mem.eql(u8, pin_name, "PD2"),
//                 6 => std.mem.eql(u8, pin_name, "PC7") or std.mem.eql(u8, pin_name, "PG9"),
//                 else => unreachable,
//             },
//         };
//     }
// };

// pub fn Uart(comptime index: usize, comptime pins: micro.uart.Pins) type {
//     if (index < 1 or index > 6) @compileError("Valid USART index are 1..6");

//     const usart_name = std.fmt.comptimePrint("USART{d}", .{index});
//     const tx_pin =
//         if (pins.tx) |tx|
//         if (uart.is_valid_pin(tx, index, .tx))
//             tx
//         else
//             @compileError(std.fmt.comptimePrint("Tx pin {s} is not valid for UART{}", .{ tx.name, index }))
//     else switch (index) {
//         // Provide default tx pins if no pin is specified
//         1 => micro.Pin("PA9"),
//         2 => micro.Pin("PA2"),
//         3 => micro.Pin("PB10"),
//         4 => micro.Pin("PA0"),
//         5 => micro.Pin("PC12"),
//         6 => micro.Pin("PC6"),
//         else => unreachable,
//     };

//     const rx_pin =
//         if (pins.rx) |rx|
//         if (uart.is_valid_pin(rx, index, .rx))
//             rx
//         else
//             @compileError(std.fmt.comptimePrint("Rx pin {s} is not valid for UART{}", .{ rx.name, index }))
//     else switch (index) {
//         // Provide default rx pins if no pin is specified
//         1 => micro.Pin("PA10"),
//         2 => micro.Pin("PA3"),
//         3 => micro.Pin("PB11"),
//         4 => micro.Pin("PA1"),
//         5 => micro.Pin("PD2"),
//         6 => micro.Pin("PC7"),
//         else => unreachable,
//     };

//     // USART1..3 are AF7, USART 4..6 are AF8
//     const alternate_function = if (index <= 3) .af7 else .af8;

//     const tx_gpio = micro.Gpio(tx_pin, .{
//         .mode = .alternate_function,
//         .alternate_function = alternate_function,
//     });
//     const rx_gpio = micro.Gpio(rx_pin, .{
//         .mode = .alternate_function,
//         .alternate_function = alternate_function,
//     });

//     return struct {
//         parity_read_mask: u8,

//         const Self = @This();

//         pub fn init(config: micro.uart.Config) !Self {
//             // The following must all be written when the USART is disabled (UE=0).
//             if (@field(peripherals, usart_name).CR1.read().UE == 1)
//                 @panic("Trying to initialize " ++ usart_name ++ " while it is already enabled");
//             // LATER: Alternatively, set UE=0 at this point?  Then wait for something?
//             // Or add a destroy() function which disables the USART?

//             // enable the USART clock
//             const clk_enable_reg = switch (index) {
//                 1, 6 => RCC.APB2ENR,
//                 2...5 => RCC.APB1ENR,
//                 else => unreachable,
//             };
//             set_reg_field(clk_enable_reg, usart_name ++ "EN", 1);

//             tx_gpio.init();
//             rx_gpio.init();

//             // clear USART configuration to its default
//             @field(peripherals, usart_name).CR1.raw = 0;
//             @field(peripherals, usart_name).CR2.raw = 0;
//             @field(peripherals, usart_name).CR3.raw = 0;

//             // Return error for unsupported combinations
//             if (config.data_bits == .nine and config.parity != null) {
//                 // TODO: should we consider this an unsupported word size or unsupported parity?
//                 return error.UnsupportedWordSize;
//             } else if (config.data_bits == .seven and config.parity == null) {
//                 // TODO: should we consider this an unsupported word size or unsupported parity?
//                 return error.UnsupportedWordSize;
//             }

//             // set word length
//             // Per the reference manual, M means
//             // - 0: 1 start bit, 8 data bits (7 data + 1 parity, or 8 data), n stop bits, the chip default
//             // - 1: 1 start bit, 9 data bits (8 data + 1 parity, or 9 data), n stop bits
//             const m: u1 = if (config.data_bits == .nine or (config.data_bits == .eight and config.parity != null)) 1 else 0;
//             @field(peripherals, usart_name).CR1.modify(.{ .M = m });

//             // set parity
//             if (config.parity) |parity| {
//                 @field(peripherals, usart_name).CR1.modify(.{ .PCE = 1, .PS = @intFromEnum(parity) });
//             } // otherwise, no need to set no parity since we reset Control Registers above, and it's the default

//             // set number of stop bits
//             @field(peripherals, usart_name).CR2.modify(.{ .STOP = @intFromEnum(config.stop_bits) });

//             // set the baud rate
//             // Despite the reference manual talking about fractional calculation and other buzzwords,
//             // it is actually just a simple divider. Just ignore DIV_Mantissa and DIV_Fraction and
//             // set the result of the division as the lower 16 bits of BRR.
//             // TODO: We assume the default OVER8=0 configuration above (i.e. 16x oversampling).
//             // TODO: Do some checks to see if the baud rate is too high (or perhaps too low)
//             // TODO: Do a rounding div, instead of a truncating div?
//             const clocks = micro.clock.get();
//             const bus_frequency = switch (index) {
//                 1, 6 => clocks.apb2,
//                 2...5 => clocks.apb1,
//                 else => unreachable,
//             };
//             const usartdiv = @as(u16, @intCast(@divTrunc(bus_frequency, config.baud_rate)));
//             @field(peripherals, usart_name).BRR.raw = usartdiv;

//             // enable USART, and its transmitter and receiver
//             @field(peripherals, usart_name).CR1.modify(.{ .UE = 1 });
//             @field(peripherals, usart_name).CR1.modify(.{ .TE = 1 });
//             @field(peripherals, usart_name).CR1.modify(.{ .RE = 1 });

//             // For code simplicity, at cost of one or more register reads,
//             // we read back the actual configuration from the registers,
//             // instead of using the `config` values.
//             return read_from_registers();
//         }

//         pub fn get_or_init(config: micro.uart.Config) !Self {
//             if (@field(peripherals, usart_name).CR1.read().UE == 1) {
//                 // UART1 already enabled, don't reinitialize and disturb things;
//                 // instead read and use the actual configuration.
//                 return read_from_registers();
//             } else return init(config);
//         }

//         fn read_from_registers() Self {
//             const cr1 = @field(peripherals, usart_name).CR1.read();
//             // As documented in `init()`, M0==1 means 'the 9th bit (not the 8th bit) is the parity bit'.
//             // So we always mask away the 9th bit, and if parity is enabled and it is in the 8th bit,
//             // then we also mask away the 8th bit.
//             return Self{ .parity_read_mask = if (cr1.PCE == 1 and cr1.M == 0) 0x7F else 0xFF };
//         }

//         pub fn can_write(self: Self) bool {
//             _ = self;
//             return switch (@field(peripherals, usart_name).SR.read().TXE) {
//                 1 => true,
//                 0 => false,
//             };
//         }

//         pub fn tx(self: Self, ch: u8) void {
//             while (!self.can_write()) {} // Wait for Previous transmission
//             @field(peripherals, usart_name).DR.modify(ch);
//         }

//         pub fn txflush(_: Self) void {
//             while (@field(peripherals, usart_name).SR.read().TC == 0) {}
//         }

//         pub fn can_read(self: Self) bool {
//             _ = self;
//             return switch (@field(peripherals, usart_name).SR.read().RXNE) {
//                 1 => true,
//                 0 => false,
//             };
//         }

//         pub fn rx(self: Self) u8 {
//             while (!self.can_read()) {} // Wait till the data is received
//             const data_with_parity_bit: u9 = @field(peripherals, usart_name).DR.read();
//             return @as(u8, @intCast(data_with_parity_bit & self.parity_read_mask));
//         }
//     };
// }

// pub const i2c = struct {
//     const PinLine = std.meta.FieldEnum(micro.i2c.Pins);

//     /// Checks if a pin is valid for a given i2c index and line
//     pub fn is_valid_pin(comptime pin: type, comptime index: usize, comptime line: PinLine) bool {
//         const pin_name = pin.name;

//         return switch (line) {
//             .scl => switch (index) {
//                 1 => std.mem.eql(u8, pin_name, "PB6") or std.mem.eql(u8, pin_name, "PB8"),
//                 2 => std.mem.eql(u8, pin_name, "PB10") or std.mem.eql(u8, pin_name, "PF1") or std.mem.eql(u8, pin_name, "PH4"),
//                 3 => std.mem.eql(u8, pin_name, "PA8") or std.mem.eql(u8, pin_name, "PH7"),
//                 else => unreachable,
//             },
//             // Valid RX pins for the UARTs
//             .sda => switch (index) {
//                 1 => std.mem.eql(u8, pin_name, "PB7") or std.mem.eql(u8, pin_name, "PB9"),
//                 2 => std.mem.eql(u8, pin_name, "PB11") or std.mem.eql(u8, pin_name, "PF0") or std.mem.eql(u8, pin_name, "PH5"),
//                 3 => std.mem.eql(u8, pin_name, "PC9") or std.mem.eql(u8, pin_name, "PH8"),
//                 else => unreachable,
//             },
//         };
//     }
// };

// pub fn I2CController(comptime index: usize, comptime pins: micro.i2c.Pins) type {
//     if (index < 1 or index > 3) @compileError("Valid I2C index are 1..3");

//     const i2c_name = std.fmt.comptimePrint("I2C{d}", .{index});
//     const scl_pin =
//         if (pins.scl) |scl|
//         if (uart.is_valid_pin(scl, index, .scl))
//             scl
//         else
//             @compileError(std.fmt.comptimePrint("SCL pin {s} is not valid for I2C{}", .{ scl.name, index }))
//     else switch (index) {
//         // Provide default scl pins if no pin is specified
//         1 => micro.Pin("PB6"),
//         2 => micro.Pin("PB10"),
//         3 => micro.Pin("PA8"),
//         else => unreachable,
//     };

//     const sda_pin =
//         if (pins.sda) |sda|
//         if (uart.is_valid_pin(sda, index, .sda))
//             sda
//         else
//             @compileError(std.fmt.comptimePrint("SDA pin {s} is not valid for UART{}", .{ sda.name, index }))
//     else switch (index) {
//         // Provide default sda pins if no pin is specified
//         1 => micro.Pin("PB7"),
//         2 => micro.Pin("PB11"),
//         3 => micro.Pin("PC9"),
//         else => unreachable,
//     };

//     const scl_gpio = micro.Gpio(scl_pin, .{
//         .mode = .alternate_function,
//         .alternate_function = .af4,
//     });
//     const sda_gpio = micro.Gpio(sda_pin, .{
//         .mode = .alternate_function,
//         .alternate_function = .af4,
//     });

//     // Base field of the specific I2C peripheral
//     const i2c_base = @field(peripherals, i2c_name);

//     return struct {
//         const Self = @This();

//         pub fn init(config: micro.i2c.Config) !Self {
//             // Configure I2C

//             // 1. Enable the I2C CLOCK and GPIO CLOCK
//             RCC.APB1ENR.modify(.{ .I2C1EN = 1 });
//             RCC.AHB1ENR.modify(.{ .GPIOBEN = 1 });

//             // 2. Configure the I2C PINs
//             // This takes care of setting them alternate function mode with the correct AF
//             scl_gpio.init();
//             sda_gpio.init();

//             // TODO: the stuff below will probably use the microzig gpio API in the future
//             const scl = scl_pin.source_pin;
//             const sda = sda_pin.source_pin;
//             // Select Open Drain Output
//             set_reg_field(@field(scl.gpio_port, "OTYPER"), "OT" ++ scl.suffix, 1);
//             set_reg_field(@field(sda.gpio_port, "OTYPER"), "OT" ++ sda.suffix, 1);
//             // Select High Speed
//             set_reg_field(@field(scl.gpio_port, "OSPEEDR"), "OSPEEDR" ++ scl.suffix, 0b10);
//             set_reg_field(@field(sda.gpio_port, "OSPEEDR"), "OSPEEDR" ++ sda.suffix, 0b10);
//             // Activate Pull-up
//             set_reg_field(@field(scl.gpio_port, "PUPDR"), "PUPDR" ++ scl.suffix, 0b01);
//             set_reg_field(@field(sda.gpio_port, "PUPDR"), "PUPDR" ++ sda.suffix, 0b01);

//             // 3. Reset the I2C
//             i2c_base.CR1.modify(.{ .PE = 0 });
//             while (i2c_base.CR1.read().PE == 1) {}

//             // 4. Configure I2C timing
//             const bus_frequency_hz = micro.clock.get().apb1;
//             const bus_frequency_mhz: u6 = @as(u6, @intCast(@divExact(bus_frequency_hz, 1_000_000)));

//             if (bus_frequency_mhz < 2 or bus_frequency_mhz > 50) {
//                 return error.InvalidBusFrequency;
//             }

//             // .FREQ is set to the bus frequency in Mhz
//             i2c_base.CR2.modify(.{ .FREQ = bus_frequency_mhz });

//             switch (config.target_speed) {
//                 10_000...100_000 => {
//                     // CCR is bus_freq / (target_speed * 2). We use floor to avoid exceeding the target speed.
//                     const ccr = @as(u12, @intCast(@divFloor(bus_frequency_hz, config.target_speed * 2)));
//                     i2c_base.CCR.modify(.{ .CCR = ccr });
//                     // Trise is bus frequency in Mhz + 1
//                     i2c_base.TRISE.modify(bus_frequency_mhz + 1);
//                 },
//                 100_001...400_000 => {
//                     // TODO: handle fast mode
//                     return error.InvalidSpeed;
//                 },
//                 else => return error.InvalidSpeed,
//             }

//             // 5. Program the I2C_CR1 register to enable the peripheral
//             i2c_base.CR1.modify(.{ .PE = 1 });

//             return Self{};
//         }

//         pub const WriteState = struct {
//             address: u7,
//             buffer: [255]u8 = undefined,
//             buffer_size: u8 = 0,

//             pub fn start(address: u7) !WriteState {
//                 return WriteState{ .address = address };
//             }

//             pub fn write_all(self: *WriteState, bytes: []const u8) !void {
//                 std.debug.assert(self.buffer_size < 255);
//                 for (bytes) |b| {
//                     self.buffer[self.buffer_size] = b;
//                     self.buffer_size += 1;
//                     if (self.buffer_size == 255) {
//                         try self.send_buffer();
//                     }
//                 }
//             }

//             fn send_buffer(self: *WriteState) !void {
//                 if (self.buffer_size == 0) @panic("write of 0 bytes not supported");

//                 // Wait for the bus to be free
//                 while (i2c_base.SR2.read().BUSY == 1) {}

//                 // Send start
//                 i2c_base.CR1.modify(.{ .START = 1 });

//                 // Wait for the end of the start condition, master mode selected, and BUSY bit set
//                 while ((i2c_base.SR1.read().SB == 0 or
//                     i2c_base.SR2.read().MSL == 0 or
//                     i2c_base.SR2.read().BUSY == 0))
//                 {}

//                 // Write the address to bits 7..1, bit 0 stays at 0 to indicate write operation
//                 i2c_base.DR.modify(@as(u8, @intCast(self.address)) << 1);

//                 // Wait for address confirmation
//                 while (i2c_base.SR1.read().ADDR == 0) {}

//                 // Read SR2 to clear address condition
//                 _ = i2c_base.SR2.read();

//                 for (self.buffer[0..self.buffer_size]) |b| {
//                     // Write data byte
//                     i2c_base.DR.modify(b);
//                     // Wait for transfer finished
//                     while (i2c_base.SR1.read().BTF == 0) {}
//                 }
//                 self.buffer_size = 0;
//             }

//             pub fn stop(self: *WriteState) !void {
//                 try self.send_buffer();
//                 // Communication STOP
//                 i2c_base.CR1.modify(.{ .STOP = 1 });
//                 while (i2c_base.SR2.read().BUSY == 1) {}
//             }

//             pub fn restart_read(self: *WriteState) !ReadState {
//                 try self.send_buffer();
//                 return ReadState{ .address = self.address };
//             }
//             pub fn restart_write(self: *WriteState) !WriteState {
//                 try self.send_buffer();
//                 return WriteState{ .address = self.address };
//             }
//         };

//         pub const ReadState = struct {
//             address: u7,

//             pub fn start(address: u7) !ReadState {
//                 return ReadState{ .address = address };
//             }

//             /// Fails with ReadError if incorrect number of bytes is received.
//             pub fn read_no_eof(self: *ReadState, buffer: []u8) !void {
//                 std.debug.assert(buffer.len < 256);

//                 // Send start and enable ACK
//                 i2c_base.CR1.modify(.{ .START = 1, .ACK = 1 });

//                 // Wait for the end of the start condition, master mode selected, and BUSY bit set
//                 while ((i2c_base.SR1.read().SB == 0 or
//                     i2c_base.SR2.read().MSL == 0 or
//                     i2c_base.SR2.read().BUSY == 0))
//                 {}

//                 // Write the address to bits 7..1, bit 0 set to 1 to indicate read operation
//                 i2c_base.DR.modify((@as(u8, @intCast(self.address)) << 1) | 1);

//                 // Wait for address confirmation
//                 while (i2c_base.SR1.read().ADDR == 0) {}

//                 // Read SR2 to clear address condition
//                 _ = i2c_base.SR2.read();

//                 for (buffer, 0..) |_, i| {
//                     if (i == buffer.len - 1) {
//                         // Disable ACK
//                         i2c_base.CR1.modify(.{ .ACK = 0 });
//                     }

//                     // Wait for data to be received
//                     while (i2c_base.SR1.read().RxNE == 0) {}

//                     // Read data byte
//                     buffer[i] = i2c_base.DR.read();
//                 }
//             }

//             pub fn stop(_: *ReadState) !void {
//                 // Communication STOP
//                 i2c_base.CR1.modify(.{ .STOP = 1 });
//                 while (i2c_base.SR2.read().BUSY == 1) {}
//             }

//             pub fn restart_read(self: *ReadState) !ReadState {
//                 return ReadState{ .address = self.address };
//             }
//             pub fn restart_write(self: *ReadState) !WriteState {
//                 return WriteState{ .address = self.address };
//             }
//         };
//     };
// }
