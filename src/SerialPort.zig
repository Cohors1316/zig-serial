p: private = .{},

const SerialPort = @This();
const std = @import("std");
const termios = @cImport(@cInclude("termios.h"));
const File = std.fs.File;
const native_os = @import("builtin").os.tag;

const private = struct {
    file: ?File = null,
    name: []const u8,
    config: Config = .{},
    // these are required for windows stupidity
    write_timeout: u32 = 100,
    read_timeout: u32 = 100,
};

const Config = struct {
    baud_rate: u32 = 9600,
    parity: Parity = .none,
    stop_bits: StopBits = .one,
    word_size: WordSize = .eight,
};

const Parity = enum {
    none,
    even,
    odd,
    space,
    mark,
};

const StopBits = switch (native_os) {
    .windows => enum {
        one,
        two,
        // #(*$&) windows
        one_point_five,
    },
    else => enum { one, two },
};

const HandShake = enum { none, software, hardware };

const WordSize = switch (native_os) {
    .windows => enum(u32) {
        CS5 = 5,
        CS6 = 6,
        CS7 = 7,
        CS8 = 8,
    },
    else => std.posix.CSIZE,
};

const Buffers = switch (native_os) {
    .windows => enum(u32) {
        input = 0x0008,
        output = 0x0004,
        both = 0x0004 | 0x0008,
    },
    .macos => enum(u32) {
        input = termios.TCIFLUSH,
        output = termios.TCOFLUSH,
        both = termios.TCIOFLUSH,
    },
    else => enum(usize) {
        input = 1,
        output = 2,
        both = 3,
    },
};

const Pins = struct {
    rts: ?bool = null,
    dtr: ?bool = null,
};