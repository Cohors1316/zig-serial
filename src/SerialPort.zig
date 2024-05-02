name: []const u8,
config: Config = .{},
private: Private = .{},

pub fn init(name: []const u8, config: Config) Self {
    const sp = Self{
        .name = name,
        .config = config,
    };
    return sp;
}

const Buffer = enum { in, out, both };
pub fn flushBuffers(self: *Self, buffers: Buffer) void {
    switch (builtin.os.tag) {
        .windows => {
            const PURGE_RXCLEAR = 0x0008;
            const PURGE_TXCLEAR = 0x0004;
            const flags = switch (buffers) {
                .in => PURGE_RXCLEAR,
                .out => PURGE_TXCLEAR,
                .both => PURGE_TXCLEAR | PURGE_RXCLEAR,
            };
            const success = windows.PurgeComm(self.private.file, flags);
            if (success == 0) return error.serial_flush_error;
        },
        .linux => {
            const flags = switch (buffers) {
                .in => 0, //TCIFLUSH
                .out => 1, //TCOFLUSH
                .both => 2, //TCIOFLUSH
            };
            try linux.tcFlush(self.private.file, flags);
        },
        .macos => {
            const flags = switch (buffers) {
                .in => termios.TCIFLUSH,
                .out => termios.TCOFLUSH,
                .both => termios.TCIOFLUSH,
            };
            try macos.tcFlush(self.private.file, flags);
        },
    }
}

const Config = struct {
    /// Symbol rate in bits/second. Not that these
    /// include also parity and stop bits.
    baud_rate: u32 = 9600,

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

const Parity = enum {
    none,
    even,
    odd,
    mark,
    space,
};

// const Parity = switch (builtin.os.tag) {
//     .windows => {enum {
//     /// No parity bit is used
//     none,
//     /// Parity bit is `0` when an even number of bits is set in the data.
//     even,
//     /// Parity bit is `0` when an odd number of bits is set in the data.
//     odd,
//     /// Parity bit is always `1`
//     mark,
//     /// Parity bit is always `0`
//     space,}
// }
// }

const StopBits = enum {
    /// The length of the stop bit is 1 bit
    one,
    /// The length of the stop bit is 2 bits
    two,
};

const WordSize = enum {
    five,
    six,
    seven,
    eight,
};

const Handshake = enum {
    /// No handshake is used
    none,
    /// XON-XOFF software handshake is used.
    software,
    /// Hardware handshake with RTS/CTS is used.
    hardware,
};

const Private = struct {
    file: std.fs.File,
};

const std = @import("std");
const builtin = @import("builtin");
const zig_serial = @import("zig-serial.zig");
const windows = @import("windows.zig");
const linux = @import("linux.zig");
const macos = @import("darwin.zig");
const Self = @This();
const termios = @cImport(@cInclude("termios.h"));
