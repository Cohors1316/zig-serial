const std = @import("std");
const File = std.fs.File;
const serial = @import("../serial.zig");
const Pins = serial.Pins;
const Buffers = serial.Buffers;
const Config = serial.Config;
const externs = @import("externs.zig");
const SerialPort = @import("../SerialPort.zig");
const DCB = externs.DCB;

pub fn setPins(file: File, pins: Pins) !void {
    if (pins.rts) |rts| {
        if (externs.EscapeCommFunction(file.handle, if (rts) 3 else 4) == 0)
            return error.ControlPins;
    }
    if (pins.dtr) |dtr| {
        if (externs.EscapeCommFunction(file.handle, if (dtr) 5 else 6) == 0)
            return error.ControlPins;
    }
}

pub fn flush(file: File, buffers: Buffers) !void {
    const mode = switch (buffers) {
        .input => 0x0008,
        .output => 0x0004,
        .both => 0x0008 | 0x0004,
    };
    if (externs.PurgeComm(file.handle, mode) == 0)
        return error.PurgeBuffers;
}

pub fn configure(self: *SerialPort) !void {
    const parity = switch (self.p.config.parity) {
        .none => "n",
        .even => "e",
        .odd => "o",
        .mark => "m",
        .space => "s",
    };
    const data_bits = @intFromEnum(self.p.config.word_size);
    const stop_bits = switch (self.p.config.stop_bits) {
        .one => "1",
        .two => "2",
        .one_point_five => "1.5",
    };
    // Specifies whether infinite time-out processing is on or off. The default is off.
    const to = "off";
    // Specifies whether the xon or xoff protocol for data-flow control is on or off.
    const xon = "off";
    // Specifies whether output handshaking that uses the Data Set Ready (DSR) circuit is on or off.
    const odsr = "off";
    // Specifies whether output handshaking that uses the Clear To Send (CTS) circuit is on or off.
    const octs = "off";
    // Specifies whether the Data Terminal Ready (DTR) circuit is on or off or set to handshake.
    const dtr = "off";
    // Specifies whether the Request To Send (RTS) circuit is set to on, off, handshake, or toggle.
    const rts = "off";
    // Specifies whether the DSR circuit sensitivity is on or off.
    const idsr = "off";
    var buffer: [110]u8 = undefined;
    const string = try std.fmt.bufPrint(
        &buffer,
        "COM{d}: parity={s} data={d} stop={s} to={s} xon={s} odsr={s} octs={s} dtr={s} rts={s} idsr={s}",
        .{ self.p.number, parity, data_bits, stop_bits, to, xon, odsr, octs, dtr, rts, idsr },
    );

    var dcb = DCB{};
    var timeouts = externs.COMMTIMEOUTS{};
    if (externs.BuildCommDCBAndTimeoutsA(
        &string,
        &dcb,
        &timeouts,
    ) == 0) return error.failed_building_dcb_and_timeouts;
    dcb.BaudRate = self.p.config.baud_rate;
    timeouts.ReadTotalTimeoutConstant = self.p.config.read_timeout;
    timeouts.WriteTotalTimeoutConstant = self.p.config.write_timeout;

    if (externs.SetCommState(file.handle, dcb) == 0)
        return error.failed_setting_comm_state;
    if (externs.SetCommTimeouts(file.handle, &timeouts) == 0)
        return error.failed_setting_timeouts;
}

// pub fn configure(file: std.fs.File, config: common_serial.Config) !void {
//     var dcb = std.mem.zeroes(DCB);
//     dcb.DCBlength = @sizeOf(DCB);
//     if (GetCommState(file.handle, &dcb) == 0) return error.configure;
//     dcb.BaudRate = config.baud_rate;
//     const dcb_flags = DCBFlags{
//         .fBinary = 1,
//         .fParity = @intFromBool(config.parity != .none),
//         .fOutxCtsFlow = @intFromBool(config.handshake == .hardware),
//         .fOutX = @intFromBool(config.handshake == .software),
//         .fInX = @intFromBool(config.handshake == .software),
//         .fRtsControl = @intFromBool(config.handshake == .hardware),
//     };
//     dcb.flags = dcb_flags.toNumeric();
//     dcb.wReserved = 0;
//     dcb.wReserved1 = 0;
//     dcb.XonChar = 0x11;
//     dcb.XoffChar = 0x13;
//     dcb.StopBits = switch (config.stop_bits) {
//         .one => 0,
//         .two => 2,
//     };
//     dcb.ByteSize = switch (config.word_size) {
//         .five => 5,
//         .six => 6,
//         .seven => 7,
//         .eight => 8,
//     };
//     dcb.Parity = switch (config.parity) {
//         .none => 0,
//         .odd => 1,
//         .even => 2,
//         .mark => 3,
//         .space => 4,
//     };
//     if (SetCommState(file.handle, &dcb) == 0) return error.configure;
// }

// const ModemStat = enum(u32) {
//     /// The CTS (clear-to-send) signal is on.
//     MS_CTS_ON = 0x0010,
//     /// The DSR (data-set-ready) signal is on.
//     MS_DSR_ON = 0x0020,
//     /// The ring indicator signal is on.
//     MS_RING_ON = 0x0040,
//     /// The RLSD (receive-line-signal-detect) signal is on.
//     MS_RLSD_ON = 0x0080,
// };
// const someMask = enum(u32) {
//     /// The hardware detected a break condition.
//     CE_BREAK = 0x0010,
//     /// The hardware detected a framing error.
//     CE_FRAME = 0x0008,
//     /// A character-buffer-overrun has occurred. The next character is lost.
//     CE_OVERRUN = 0x0002,
//     /// An input buffer overflow has occurred. There is either no room in the input buffer, or a character was received after the end-of-line (EOF) character.
//     CE_RXOVER = 0x0001,
//     /// The hardware detected a parity error.
//     CE_RXPARITY = 0x0004,
// };
