//! USB device implementation
//!
//! Inspired by cbiffle's Rust [implementation](https://github.com/cbiffle/rp2040-usb-device-in-one-file/blob/main/src/main.rs)

const std = @import("std");

const microzig = @import("microzig");
const peripherals = microzig.chip.peripherals;

/// Human Interface Device (HID)
pub const usb = microzig.core.usb;
pub const types = usb.types;
pub const hid = usb.hid;
pub const cdc = usb.cdc;
pub const vendor = usb.vendor;
pub const templates = usb.templates.DescriptorsConfigTemplates;
pub const utils = usb.UsbUtils;

pub const otg_global = peripherals.OTG_FS_GLOBAL;
pub const otg_pwrclk = peripherals.OTG_FS_PWRCLK;
pub const otg_fs_device = peripherals.OTG_FS_DEVICE;

pub const EP0_OUT_IDX = 0;
pub const EP0_IN_IDX = 1;

// +++++++++++++++++++++++++++++++++++++++++++++++++
// User Interface
// +++++++++++++++++++++++++++++++++++++++++++++++++

/// The rp2040 usb device impl
///
/// We create a concrete implementaion by passing a handful
/// of system specific functions to Usb(). Those functions
/// are used by the abstract USB impl of microzig.
pub const Usb = usb.Usb(F);

pub const DeviceConfiguration = usb.DeviceConfiguration;
pub const DeviceDescriptor = usb.DeviceDescriptor;
pub const DescType = usb.types.DescType;
pub const InterfaceDescriptor = usb.types.InterfaceDescriptor;
pub const ConfigurationDescriptor = usb.types.ConfigurationDescriptor;
pub const EndpointDescriptor = usb.types.EndpointDescriptor;
pub const EndpointConfiguration = usb.EndpointConfiguration;
pub const Dir = usb.types.Dir;
pub const TransferType = usb.types.TransferType;
pub const Endpoint = usb.types.Endpoint;

pub const utf8ToUtf16Le = usb.utf8Toutf16Le;

const HardwareEndpoint = struct {
    configured: bool,
    ep_addr: u8,
    next_pid_1: bool,
    max_packet_size: u16,
    transfer_type: types.TransferType,
    endpoint_control_index: usize,
    buffer_control_index: usize,
    data_buffer_index: usize,
};

pub const DPID = enum(u2) {
    DATA0 = 0x0,
    DATA2 = 0x1,
    DATA1 = 0x2,
    MDATA = 0x3,
};

pub const DSPD = enum(u2) {
    ///  High speed
    HIGH_SPEED = 0x0,
    ///  Full speed using external ULPI PHY
    FULL_SPEED_EXTERNAL = 0x1,
    ///  Full speed using internal embedded PHY
    FULL_SPEED_INTERNAL = 0x3,
    _,
};

pub const EPTYP = enum(u2) {
    CONTROL = 0x0,
    ISOCHRONOUS = 0x1,
    BULK = 0x2,
    INTERRUPT = 0x3,
};

pub const PFIVL = enum(u2) {
    ///  80% of the frame interval
    FRAME_INTERVAL_80 = 0x0,
    ///  85% of the frame interval
    FRAME_INTERVAL_85 = 0x1,
    ///  90% of the frame interval
    FRAME_INTERVAL_90 = 0x2,
    ///  95% of the frame interval
    FRAME_INTERVAL_95 = 0x3,
};

pub const PKTSTSD = enum(u4) {
    ///  Global OUT NAK (triggers an interrupt)
    OUT_NAK = 0x1,
    ///  OUT data packet received
    OUT_DATA_RX = 0x2,
    ///  OUT transfer completed (triggers an interrupt)
    OUT_DATA_DONE = 0x3,
    ///  SETUP transaction completed (triggers an interrupt)
    SETUP_DATA_DONE = 0x4,
    ///  SETUP data packet received
    SETUP_DATA_RX = 0x6,
    _,
};

pub const PKTSTSH = enum(u4) {
    ///  IN data packet received
    IN_DATA_RX = 0x2,
    ///  IN transfer completed (triggers an interrupt)
    IN_DATA_DONE = 0x3,
    ///  Data toggle error (triggers an interrupt)
    DATA_TOGGLE_ERR = 0x5,
    ///  Channel halted (triggers an interrupt)
    CHANNEL_HALTED = 0x7,
    _,
};

// +++++++++++++++++++++++++++++++++++++++++++++++++
// Reference to endpoint buffers
// +++++++++++++++++++++++++++++++++++++++++++++++++

/// USB data buffers
pub const buffers = struct {
    // Address 0x100-0xfff (3840 bytes) can be used for data buffers.
    const USBDPRAM_BASE = 0x50100100;
    // Data buffers are 64 bytes long as this is the max normal packet size
    const BUFFER_SIZE = 64;
    /// EP0 buffer 0 (shared between in and out)
    const USB_EP0_BUFFER0 = USBDPRAM_BASE;
    /// Optional EP0 buffer 1
    const USB_EP0_BUFFER1 = USBDPRAM_BASE + BUFFER_SIZE;
    /// Data buffers
    const USB_BUFFERS = USBDPRAM_BASE + (2 * BUFFER_SIZE);

    /// Mapping to the different data buffers in DPSRAM
    pub var B: usb.Buffers = .{
        .ep0_buffer0 = @as([*]u8, @ptrFromInt(USB_EP0_BUFFER0)),
        .ep0_buffer1 = @as([*]u8, @ptrFromInt(USB_EP0_BUFFER1)),
        // We will initialize this comptime in a loop
        .rest = .{
            @as([*]u8, @ptrFromInt(USB_BUFFERS + (0 * BUFFER_SIZE))),
            @as([*]u8, @ptrFromInt(USB_BUFFERS + (1 * BUFFER_SIZE))),
            @as([*]u8, @ptrFromInt(USB_BUFFERS + (2 * BUFFER_SIZE))),
            @as([*]u8, @ptrFromInt(USB_BUFFERS + (3 * BUFFER_SIZE))),
            @as([*]u8, @ptrFromInt(USB_BUFFERS + (4 * BUFFER_SIZE))),
            @as([*]u8, @ptrFromInt(USB_BUFFERS + (5 * BUFFER_SIZE))),
            @as([*]u8, @ptrFromInt(USB_BUFFERS + (6 * BUFFER_SIZE))),
            @as([*]u8, @ptrFromInt(USB_BUFFERS + (7 * BUFFER_SIZE))),
            @as([*]u8, @ptrFromInt(USB_BUFFERS + (8 * BUFFER_SIZE))),
            @as([*]u8, @ptrFromInt(USB_BUFFERS + (9 * BUFFER_SIZE))),
            @as([*]u8, @ptrFromInt(USB_BUFFERS + (10 * BUFFER_SIZE))),
            @as([*]u8, @ptrFromInt(USB_BUFFERS + (11 * BUFFER_SIZE))),
            @as([*]u8, @ptrFromInt(USB_BUFFERS + (12 * BUFFER_SIZE))),
            @as([*]u8, @ptrFromInt(USB_BUFFERS + (13 * BUFFER_SIZE))),
            @as([*]u8, @ptrFromInt(USB_BUFFERS + (14 * BUFFER_SIZE))),
            @as([*]u8, @ptrFromInt(USB_BUFFERS + (15 * BUFFER_SIZE))),
        },
    };
};

pub const buffer_status = enum(u1) { HALF_EMPTY = 0, COMPLETELY_EMPTY = 1 };

pub const masking = enum(u1) { MASKED = 0, UNMASKED = 1 };

pub const enabledisable = enum(u1) { DISABLED = 0, ENABLED = 1 };
pub const flag_status = enum(u1) { INACTIVE = 0, ACTIVE = 1 };

// +++++++++++++++++++++++++++++++++++++++++++++++++
// Code
// +++++++++++++++++++++++++++++++++++++++++++++++++

/// A set of functions required by the abstract USB impl to
/// create a concrete one.
pub const F = struct {
    // Fixed number 4, probably should be comptime number up to 16
    var endpoints: [4][2]HardwareEndpoint = undefined;

    /// Initialize the USB clock to 48 MHz
    ///
    /// This requres that the system clock has been set up before hand
    /// using the 12 MHz crystal.
    pub fn usb_init_clk() void {
        // This section explains the initialization of the OTG_FS controller after power-on. The
        // application must follow the initialization sequence irrespective of host or device mode
        // operation. All core global registers are initialized according to the core’s configuration:
        // 1. Program the following fields in the OTG_FS_GAHBCFG register:
        // – Global interrupt mask bit GINTMSK = 1
        // – RxFIFO non-empty (RXFLVL bit in OTG_FS_GINTSTS)
        // – Periodic TxFIFO empty level
        // 2. Program the following fields in the OTG_FS_GUSBCFG register:
        // – HNP capable bit
        // – SRP capable bit
        // – FS timeout calibration field
        // – USB turnaround time field
        // 3. The software must unmask the following bits in the OTG_FS_GINTMSK register:
        // OTG interrupt mask
        // Mode mismatch interrupt mask
        // 4. The software can read the CMOD bit in OTG_FS_GINTSTS to determine whether the
        // OTG_FS controller is operating in host or device mode

        // assume the clocks are setup already and just setup the global config

        // 1.
        otg_global.FS_GAHBCFG.modify(.{ //
            .GINT = .UNMASKED,
            .TXFELVL = .COMPLETELY_EMPTY,
            .PTXFELVL = .COMPLETELY_EMPTY, // something about this only available in host mode(?)
        });

        // 2.
        otg_global.FS_GUSBCFG.modify(.{ //
            .HNPCAP = .DISABLED,
            .SRPCAP = .DISABLED,
            .TOCAL = 5, // may need to write calibration code?
            .TRDT = 0x6 + 5, // minimum time specified in ref manual table
        });

        // 3.
        otg_global.FS_GINTMSK.modify(.{ //
            .OTGINT = .UNMASKED,
            .MMISM = .UNMASKED,
        });

        // 4.
        // _CAN_ read, but then do what?
        // otg_global.FS_GINTSTS.read()
    }

    pub fn usb_init_device(_: *usb.DeviceConfiguration) void {
        // The application must perform the following steps to initialize the core as a device on power-
        // up or after a mode change from host to device.
        // 1. Program the following fields in the OTG_FS_DCFG register:
        // – Device speed
        // – Non-zero-length status OUT handshake
        // 2. Program the OTG_FS_GINTMSK register to unmask the following interrupts:
        // – USB reset
        // – Enumeration done
        // – Early suspend
        // – USB suspend
        // – SOF
        // 3. Program the VBUSBSEN bit in the OTG_FS_GCCFG register to enable VBUS sensing
        // in “B” device mode and supply the 5 volts across the pull-up resistor on the DP line.
        // 4. Wait for the USBRST interrupt in OTG_FS_GINTSTS. It indicates that a reset has been
        // detected on the USB that lasts for about 10 ms on receiving this interrupt.
        //
        // Wait for the ENUMDNE interrupt in OTG_FS_GINTSTS. This interrupt indicates the end of
        // reset on the USB. On receiving this interrupt, the application must read the OTG_FS_DSTS
        // register to determine the enumeration speed and perform the steps listed in Endpoint
        // initialization on enumeration completion on page 782.
        // At this point, the device is ready to accept SOF packets and perform control transfers on
        // control endpoint 0.

        otg_fs_device.FS_DCFG.modify(.{ //
            .DSPD = .FULL_SPEED_INTERNAL,
            // .NZLSOHSK = 1, // going to leave at default until I know what it does.
        });
        otg_global.FS_GINTMSK.modify(.{ //
            .USBRST = .UNMASKED,
            .ENUMDNEM = .UNMASKED,
            .ESUSPM = .UNMASKED,
            .USBSUSPM = .UNMASKED,
            .SOFM = .UNMASKED,
        });

        // 3. VBUS detection
        // https://electrical.codidact.com/posts/291709/292007#answer-292007
        // F401 SVD is wrong here, it doesn't have the NOVBUSSENS bit listed
        // otg_global.FS_GINTMSK.modify(.{.NOVBUSSENS = 1}); // use this with the generated regs
        otg_global.FS_GINTMSK.raw |= 1 << 21; // tell the hardware to pretend VBUS is always asserted

        // Not sure what to do about the waiting for interrupts since the zig usb impl does all the checking in the loop
        // and doesn't have a callback.
    }

    /// Configures a given endpoint to send data (device-to-host, IN) when the host
    /// next asks for it.
    ///
    /// The contents of `buffer` will be _copied_ into USB SRAM, so you can
    /// reuse `buffer` immediately after this returns. No need to wait for the
    /// packet to be sent.
    pub fn usb_start_tx(
        ep_addr: u8,
        buffer: []const u8,
    ) void {
        _ = ep_addr;
        _ = buffer;
    }

    pub fn usb_start_rx(
        ep_addr: u8,
        len: usize,
    ) void {
        _ = ep_addr;
        _ = len;
    }

    /// Check which interrupt flags are set
    pub fn get_interrupts() usb.InterruptStatus {
        const ints = otg_global.FS_GINTSTS.read();

        return .{
            .BuffStatus = if (ints.SOF == 1) true else false,
            .BusReset = if (ints.USBRST == 1) true else false,
            .DevConnDis = if (ints.DISCINT == 1) true else false, // or SRQINT ?
            .DevSuspend = if (ints.USBSUSP == 1) true else false,
            .DevResumeFromHost = if (ints.WKUPINT == 1) true else false,
            .SetupReq = if (ints.SETUP_REQ == 1) true else false,
        };
    }

    /// Returns a received USB setup packet
    ///
    /// Side effect: The setup request status flag will be cleared
    ///
    /// One can assume that this function is only called if the
    /// setup request flag is set.
    pub fn get_setup_packet() usb.types.SetupPacket {
        // return std.mem.bytesToValue(usb.types.SetupPacket, &setup_packet);
        return usb.types.SetupPacket{};
    }
    pub fn reset_ep0() void {
        // var ep = hardware_endpoint_get_by_address(Endpoint.EP0_IN_IDX);
        // ep.next_pid_1 = true;
    }
    /// Called on a bus reset interrupt
    pub fn bus_reset() void {
        // Endpoint initialization on USB reset
        // 1. Set the NAK bit for all OUT endpoints
        // – SNAK = 1 in OTG_FS_DOEPCTLx (for all OUT endpoints)
        // 2. Unmask the following interrupt bits
        // – INEP0 = 1 in OTG_FS_DAINTMSK (control 0 IN endpoint)
        // – OUTEP0 = 1 in OTG_FS_DAINTMSK (control 0 OUT endpoint)
        // – STUP = 1 in DOEPMSK
        // – XFRC = 1 in DOEPMSK
        // – XFRC = 1 in DIEPMSK
        // – TOC = 1 in DIEPMSK
        // 3. Set up the Data FIFO RAM for each of the FIFOs
        // – Program the OTG_FS_GRXFSIZ register, to be able to receive control OUT data
        // and setup data. If thresholding is not enabled, at a minimum, this must be equal to
        // 1 max packet size of control endpoint 0 + 2 words (for the status of the control
        // OUT data packet) + 10 words (for setup packets).
        // – Program the OTG_FS_TX0FSIZ register (depending on the FIFO number chosen)
        // to be able to transmit control IN data. At a minimum, this must be equal to 1 max
        // packet size of control endpoint 0.
        // 4. Program the following fields in the endpoint-specific registers for control OUT endpoint
        // 0 to receive a SETUP packet
        // – STUPCNT = 3 in OTG_FS_DOEPTSIZ0 (to receive up to 3 back-to-back SETUP
        // packets)
        // At this point, all initialization required to receive SETUP packets is done.
    }

    // Bits 1:0 MPSIZ: Maximum packet size
    // The application must program this field with the maximum packet size for the current logical
    // endpoint.
    // 00: 64 bytes
    // 01: 32 bytes
    // 10: 16 bytes
    // 11: 8 bytes

    const packet_size_MPSIZ = enum(u2) { b64 = 0, b32 = 0b01, b16 = 0b10, b8 = 0b11 };

    // 00: High Speed using HS PHY
    // 01: Full Speed using HS PHY
    // 11: Full speed using embedded FS PHY

    fn enum_done() void {
        // Endpoint initialization on enumeration completion
        // 1. On the Enumeration Done interrupt (ENUMDNE in OTG_FS_GINTSTS), read the
        // OTG_FS_DSTS register to determine the enumeration speed.
        // 2. Program the MPSIZ field in OTG_FS_DIEPCTL0 to set the maximum packet size. This
        // step configures control endpoint 0. The maximum packet size for a control endpoint
        // depends on the enumeration speed.
        // At this point, the device is ready to receive SOF packets and is configured to perform control
        // transfers on control endpoint 0.

        // should only ever be 0b11 for full speed on the f401
        const enumerated_speed = otg_fs_device.FS_DSTS.read().ENUMSPD;

        _ = enumerated_speed;

        otg_fs_device.FS_DIEPCTL0.modify(.{ .MPSIZ = .b64 });
        // need to write a 1 to the ENUMDNE bit to clear it
    }

    pub fn set_address(addr: u7) void {
        // Endpoint initialization on SetAddress command
        // This section describes what the application must do when it receives a SetAddress
        // command in a SETUP packet.
        // 1. Program the OTG_FS_DCFG register with the device address received in the
        // SetAddress command
        // 2. Program the core to send out a status IN packet
        _ = addr;
    }

    pub fn get_EPBIter(dc: *const usb.DeviceConfiguration) usb.EPBIter {
        return .{
            .bufbits = peripherals.USBCTRL_REGS.BUFF_STATUS.raw,
            .device_config = dc,
            .next = next,
        };
    }

    pub fn int_callback(ints: usb.InterruptStatus) void {
        if (ints.ENUMDNE == .ACTIVE) {
            enum_done();
        }
    }
};

pub fn next(self: *usb.EPBIter) ?usb.EPB {
    if (self.last_bit) |lb| {
        // Acknowledge the last handled buffer
        peripherals.USBCTRL_REGS.BUFF_STATUS.write_raw(lb);
        self.last_bit = null;
    }
    // All input buffers handled?
    if (self.bufbits == 0) return null;

    // Who's still outstanding? Find their bit index by counting how
    // many LSBs are zero.
    var lowbit_index: u5 = 0;
    while ((self.bufbits >> lowbit_index) & 0x01 == 0) : (lowbit_index += 1) {}
    // Remove their bit from our set.
    const lowbit = @as(u32, @intCast(1)) << lowbit_index;
    self.last_bit = lowbit;
    self.bufbits ^= lowbit;

    // Here we exploit knowledge of the ordering of buffer control
    // registers in the peripheral. Each endpoint has a pair of
    // registers, so we can determine the endpoint number by:
    const epnum = @as(u8, @intCast(lowbit_index >> 1));
    // Of the pair, the IN endpoint comes first, followed by OUT, so
    // we can get the direction by:
    const dir = if (lowbit_index & 1 == 0) usb.types.Dir.In else usb.types.Dir.Out;

    const ep_addr = Endpoint.to_address(epnum, dir);
    // Process the buffer-done event.

    // Process the buffer-done event.
    //
    // Scan the device table to figure out which endpoint struct
    // corresponds to this address. We could use a smarter
    // method here, but in practice, the number of endpoints is
    // small so a linear scan doesn't kill us.

    const endpoint = F.hardware_endpoint_get_by_address(ep_addr);

    // Buffer event for unknown EP?!
    // TODO: if (endpoint == null) return EPBError.UnknownEndpoint;
    // Read the buffer control register to check status.
    // const bc = read_raw_buffer_control(endpoint.buffer_control_index);

    // We should only get here if we've been notified that
    // the buffer is ours again. This is indicated by the hw
    // _clearing_ the AVAILABLE bit.
    //
    // This ensures that we can return a shared reference to
    // the databuffer contents without races.
    // TODO: if ((bc & (1 << 10)) == 1) return EPBError.NotAvailable;

    // Cool. Checks out.

    // Get a pointer to the buffer in USB SRAM. This is the
    // buffer _contents_. See the safety comments below.
    const epbuffer = buffers.B.get(endpoint.data_buffer_index);

    // Get the actual length of the data, which may be less
    // than the buffer size.
    // const len = @as(usize, @intCast(bc & 0x3ff));
    const len = @as(usize, @intCast(1 & 0x3ff));

    // Copy the data from SRAM
    return usb.EPB{
        .endpoint_address = ep_addr,
        .buffer = epbuffer[0..len],
    };
}
