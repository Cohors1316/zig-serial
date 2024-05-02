const std = @import("std");
const builtin = @import("builtin");
// const c = @cImport(@cInclude("termios.h"));
const windows = @import("windows.zig");
const linux = @import("linux.zig");
const darwin = @import("darwin.zig");
const SerialPort = @import("SerialPort.zig");

pub fn list() !PortIterator {
    return try PortIterator.init();
}

pub fn list_info() !InformationIterator {
    return try InformationIterator.init();
}

pub const PortIterator = switch (builtin.os.tag) {
    .windows => windows.PortIterator,
    .linux => linux.PortIterator,
    .macos => darwin.PortIterator,
    else => @compileError("OS is not supported for port iteration"),
};

pub const InformationIterator = switch (builtin.os.tag) {
    .windows => windows.InformationIterator,
    .linux, .macos => @panic("'Port Information' not yet implemented for this OS"),
    else => @compileError("OS is not supported for information iteration"),
};

pub const SerialPortDescription = struct {
    file_name: []const u8,
    display_name: []const u8,
    driver: ?[]const u8,
};

pub const PortInformation = struct {
    port_name: []const u8,
    system_location: []const u8,
    friendly_name: []const u8,
    description: []const u8,
    manufacturer: []const u8,
    serial_number: []const u8,
    // TODO: review whether to remove `hw_id`.
    // Is this useless/being used in a Windows-only way?
    hw_id: []const u8,
    vid: u16,
    pid: u16,
};

// const HKEY = std.os.windows.HKEY;
// const HWND = std.os.windows.HANDLE;
// const HDEVINFO = std.os.windows.HANDLE;
// const DEVINST = std.os.windows.DWORD;
// const SP_DEVINFO_DATA = extern struct {
//     cbSize: std.os.windows.DWORD,
//     classGuid: std.os.windows.GUID,
//     devInst: std.os.windows.DWORD,
//     reserved: std.os.windows.ULONG_PTR,
// };


pub const Config = struct {
    /// Symbol rate in bits/second. Not that these
    /// include also parity and stop bits.
    baud_rate: u32,

    /// Parity to verify transport integrity.
    parity: SerialPort.Parity = .none,

    /// Number of stop bits after the data
    stop_bits: SerialPort.StopBits = .one,

    /// Number of data bits per word.
    /// Allowed values are 5, 6, 7, 8
    word_size: SerialPort.WordSize = .eight,

    /// Defines the handshake protocol used.
    handshake: SerialPort.Handshake = .none,
};

const CBAUD = 0o000000010017; //Baud speed mask (not in POSIX).
const CMSPAR = 0o010000000000;
const CRTSCTS = 0o020000000000;

const VTIME = 5;
const VMIN = 6;
const VSTART = 8;
const VSTOP = 9;

/// This function configures a serial port with the given config.
/// `port` is an already opened serial port, on windows these
/// are either called `\\.\COMxx\` or `COMx`, on unixes the serial
/// port is called `/dev/ttyXXX`.
pub fn configureSerialPort(port: std.fs.File, config: Config) !void {
    switch (builtin.os.tag) {
        .windows => try windows.configureSerialPort(port, config),
        .linux, .macos => |tag| {
            var settings = try std.posix.tcgetattr(port.handle);

            settings.iflag = .{};
            settings.oflag = .{};
            settings.cflag = .{ .CREAD = true };
            settings.lflag = .{};
            settings.ispeed = .B0;
            settings.ospeed = .B0;

            switch (config.parity) {
                .none => {},
                .odd => settings.cflag.PARODD = true,
                .even => {}, // even parity is default when parity is enabled
                .mark => {
                    settings.cflag.PARODD = true;
                    // settings.cflag.CMSPAR = true;
                    settings.cflag._ |= (1 << 14);
                },
                .space => settings.cflag._ |= 1,
            }
            if (config.parity != .none) {
                settings.iflag.INPCK = true; // enable parity checking
                settings.cflag.PARENB = true; // enable parity generation
            }

            switch (config.handshake) {
                .none => settings.cflag.CLOCAL = true,
                .software => {
                    settings.iflag.IXON = true;
                    settings.iflag.IXOFF = true;
                },
                // .hardware => settings.cflag.CRTSCTS = true,
                .hardware => settings.cflag._ |= 1 << 15,
            }

            switch (config.stop_bits) {
                .one => {},
                .two => settings.cflag.CSTOPB = true,
            }

            switch (config.word_size) {
                .five => settings.cflag.CSIZE = .CS5,
                .six => settings.cflag.CSIZE = .CS6,
                .seven => settings.cflag.CSIZE = .CS7,
                .eight => settings.cflag.CSIZE = .CS8,
            }

            const baudmask = switch (tag) {
                .macos => try darwin.mapBaudToEnum(config.baud_rate),
                .linux => try linux.mapBaudToEnum(config.baud_rate),
                else => unreachable,
            };

            // settings.cflag &= ~@as(os.tcflag_t, CBAUD);
            // settings.cflag |= baudmask;
            settings.ispeed = baudmask;
            settings.ospeed = baudmask;

            settings.cc[VMIN] = 1;
            settings.cc[VSTOP] = 0x13; // XOFF
            settings.cc[VSTART] = 0x11; // XON
            settings.cc[VTIME] = 0;

            try std.posix.tcsetattr(port.handle, .NOW, settings);
        },
        else => @compileError("unsupported OS, please implement!"),
    }
}

/// Flushes the serial port `port`. If `input` is set, all pending data in
/// the receive buffer is flushed, if `output` is set all pending data in
/// the send buffer is flushed.
pub fn flushSerialPort(port: std.fs.File, input: bool, output: bool) !void {
    if (!input and !output)
        return;
    switch (builtin.os.tag) {
        .windows => try windows.flushSerialPort(port, input, output),
        .linux => try linux.flushSerialPort(port, input, output),
        .macos => try darwin.flushSerialPort(port, input, output),
        else => @compileError("unsupported OS, please implement!"),
    }
}

pub const ControlPins = struct {
    rts: ?bool = null,
    dtr: ?bool = null,
};

pub fn changeControlPins(port: std.fs.File, pins: ControlPins) !void {
    switch (builtin.os.tag) {
        .windows => windows.changeControlPins(port, pins),
        .linux => linux.changeControlPins(port, pins),
        .macos => {},
        else => @compileError("changeControlPins not implemented for " ++ @tagName(builtin.os.tag)),
    }
}

test "iterate ports" {
    var it = try list();
    while (try it.next()) |port| {
        _ = port;
        // std.debug.print("{s} (file: {s}, driver: {s})\n", .{ port.display_name, port.file_name, port.driver });
    }
}

test "basic configuration test" {
    const cfg = Config{
        .handshake = .none,
        .baud_rate = 115200,
        .parity = .none,
        .word_size = .eight,
        .stop_bits = .one,
    };

    var tty: []const u8 = undefined;

    switch (builtin.os.tag) {
        .windows => tty = "\\\\.\\COM3",
        .linux => tty = "/dev/ttyUSB0",
        .macos => tty = "/dev/cu.usbmodem101",
        else => unreachable,
    }

    var port = try std.fs.cwd().openFile(tty, .{ .mode = .read_write });
    defer port.close();

    try configureSerialPort(port, cfg);
}

test "basic flush test" {
    var tty: []const u8 = undefined;
    // if any, these will likely exist on a machine
    switch (builtin.os.tag) {
        .windows => tty = "\\\\.\\COM3",
        .linux => tty = "/dev/ttyUSB0",
        .macos => tty = "/dev/cu.usbmodem101",
        else => unreachable,
    }
    var port = try std.fs.cwd().openFile(tty, .{ .mode = .read_write });
    defer port.close();

    try flushSerialPort(port, true, true);
    try flushSerialPort(port, true, false);
    try flushSerialPort(port, false, true);
    try flushSerialPort(port, false, false);
}

test "change control pins" {
    _ = changeControlPins;
}
