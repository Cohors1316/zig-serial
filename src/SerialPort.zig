/// File name of the serial port
name: []const u8,

/// configuration options
config: Config = .{},

/// used with writeLine for convenience
new_line: []const u8 = "\n",

/// don't directly access the file
private: private = .{},

/// Initialize the serial port with the given name and configuration.
pub fn init(name: []const u8, config: Config) !SerialPort {
    const sp = SerialPort{
        .name = name,
        .config = config,
        .private = .{
            .file = try std.fs.cwd().openFile(name, .{ .mode = .read_write }),
        },
    };
    errdefer sp.private.file.close();
    try sp.configure();
    return sp;
}


/// Deinitialize the serial port.
pub fn deinit(self: *SerialPort) void {
    self.private.file.close();
}


/// Write the given bytes to the serial port.
pub fn write(self: *SerialPort, bytes: []const u8) !void {
    try self.private.file.writeAll(bytes);
}

/// Write the given bytes to the serial port and append newline character(s).
pub fn writeLine(self: *SerialPort, bytes: []const u8) !void {
    try self.private.file.writeAll(bytes);
    try self.private.file.writeAll(self.new_line);
}

/// Read bytes from the serial port into the given buffer.
pub fn read(self: *SerialPort, buffer: []u8) !usize {
    return self.private.file.readAll(buffer);
}

/// used for flushBuffers parameter
const Buffers = enum { in, out, both };

/// Flush the serial port buffers.
pub fn flushBuffers(self: *SerialPort, buffers: Buffers) !void {
    switch (native_os) {
        .windows => {
            const flags = switch (buffers) {
                .in => 0x0008, // RXCLEAR
                .out => 0x0004, // TXCLEAR
                .both => 0x0004 | 0x0008,
            };
            const success = windows.PurgeComm(self.private.file, flags);
            if (success == 0) return error.serial_flush_error;
        },
        .linux => {
            const mode = switch (buffers) {
                .in => 0, // TCIFLUSH
                .out => 1, // TCOFLUSH
                .both => 2, // TCIOFLUSH
            };
            try linux.tcFlush(self.private.file, mode);
        },
        .macos => {
            const mode = switch (buffers) {
                .in => termios.TCIFLUSH,
                .out => termios.TCOFLUSH,
                .both => termios.TCIOFLUSH,
            };
            const err = termios.tcflush(self.private.file, @as(c_int, @intCast(mode)));
            if (err != 0) return error.serial_flush_error;
        },
        else => @compileError("unsupported OS, please implement!"),
    }
}

/// used for setPins parameter
const Pins = struct { dtr: ?bool = null, rts: ?bool = null };

/// Set the state of the DTR and RTS pins.
pub fn setPins(self: *SerialPort, pins: Pins) !void {
    switch (native_os) {
        .windows => windows.setPins(self.private.file, pins),
        .linux => linux.setPins(self.private.file, pins),
        .macos => {},
        else => @compileError("unsupported OS, please implement!"),
    }
}

/// get iterator for available serial ports
pub fn list() !PortIterator {
    return try PortIterator.init();
}

/// get description of serial ports
const PortIterator = struct {
    switch (native_os) {
        .windows => windows.PortIterator,
        .linux => linux.PortIterator,
        .macos => macos.PortIterator,
        else => @compileError("unsupported OS, please implement!"),
    }
};

pub const Config = struct {
    /// Symbol rate in bits/second. Not that these
    /// include also parity and stop bits.
    baud_rate: BaudRate = .B9600,

    /// Parity to verify transport integrity.
    parity: Parity = .none,

    /// Number of stop bits after the data
    stop_bits: StopBits = .one,

    /// Number of data bits per word.
    /// Allowed values are 5, 6, 7, 8
    word_size: WordSize = .eight,

    /// Defines the handshake protocol used.
    handshake: Handshake = .none,
};

/// Baud rate in bits/second.
/// Windows is the odd ball out. speed_t contains the baud rates for most
/// platforms. Windows method will strip out the `B` from the enum name and
/// cast to u32.
pub const BaudRate = std.c.speed_t;

pub const Parity = enum {
    /// No parity bit is used
    none,
    /// Parity bit is `0` when an even number of bits is set in the data.
    even,
    /// Parity bit is `0` when an odd number of bits is set in the data.
    odd,
    /// Parity bit is always `1`
    mark,
    /// Parity bit is always `0`
    space,
};

pub const StopBits = enum {
    /// The length of the stop bit is 1 bit
    one,
    /// The length of the stop bit is 2 bits
    two,
};

pub const WordSize = enum {
    five,
    six,
    seven,
    eight,
};

pub const Handshake = enum {
    /// No handshake is used
    none,
    /// XON-XOFF software handshake is used.
    software,
    /// Hardware handshake with RTS/CTS is used.
    hardware,
};

const private = struct {
    file: std.fs.File,
};

pub const Description = struct {
    file_name: []const u8,
    display_name: []const u8,
    driver: ?[]const u8,
};

const Details = struct {};

fn configure(self: *SerialPort) !void {
    const file = self.private.file;
    const config = self.config;
    switch (native_os) {
        .windows => return windows.configure(file, config),
        .linux, .macos => {},
        else => @compileError("unsupported OS, please implement!"),
    }
    var settings = try std.posix.tcgetattr(file.handle);
    settings.iflag = .{};
    settings.iflag.INPCK = (config.parity != .none);
    settings.iflag.IXON = (config.handshake == .software);
    settings.iflag.IXOFF = (config.handshake == .software);
    settings.oflag = .{};
    settings.cflag = .{};
    settings.cflag.CREAD = true;
    settings.cflag.PARODD = (config.parity == .mark or config.parity == .odd);
    settings.cflag.PARENB = (config.parity != .none);
    settings.cflag.CLOCAL = (config.handshake == .none);
    settings.cflag.CSTOPB = (config.stop_bits == .two);
    settings.cflag.CSIZE = switch (config.word_size) {
        .five => .CS5,
        .six => .CS6,
        .seven => .CS7,
        .eight => .CS8,
    };
    settings.lflag = .{};
    settings.ispeed = config.baud_rate;
    settings.ospeed = config.baud_rate;
    settings.cc[6] = 1; // VMIN
    settings.cc[9] = 0x13; // VSTOP XOFF
    settings.cc[8] = 0x11; // VSTART XON
    settings.cc[5] = 0; // VTIME
    if (config.parity == .space) {
        settings.cflag._ |= 1;
    } else if (config.parity == .mark) {
        settings.cflag._ |= (1 << 14);
    }
    if (config.handshake == .hardware) {
        settings.cflag._ |= 1 << 15;
    }
}

const std = @import("std");
const builtin = @import("builtin");
const zig_serial = @import("zig-serial.zig");
const windows = @import("windows.zig");
const linux = @import("linux.zig");
const macos = @import("macos.zig");
const SerialPort = @This();
const native_os = builtin.os.tag;
const termios = @cImport(@cInclude("termios.h"));

test "windows loop" {
    const hello = "Hello, World!";
    const sp1 = try SerialPort.init("\\\\.\\COM100", .{});
    defer sp1.deinit();
    const sp2 = try SerialPort.init("\\\\.\\COM200", .{});
    defer sp2.deinit();
    try sp1.write(hello);
    const buffer: [hello.len]u8 = undefined;
    _ = try sp2.read(buffer);
    std.testing.expectEqualSlices(u8, hello, buffer);
}

test "linux loop" {
    const hello = "Hello, World!";
    const sp1 = try SerialPort.init("/dev/ttyUSB0", .{});
    defer sp1.deinit();
    const sp2 = try SerialPort.init("/dev/ttyUSB1", .{});
    defer sp2.deinit();
    try sp1.write(hello);
    const buffer: [hello.len]u8 = undefined;
    _ = try sp2.read(buffer);
    std.testing.expectEqualSlices(u8, hello, buffer);
}