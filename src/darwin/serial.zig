const c = @cImport(@cInclude("termios.h"));
const serial = @import("../serial.zig");
const Buffers = serial.Buffers;
const File = @import("std").fs.File;

pub fn flush(file: File, buffers: Buffers) !void {
    const mode = switch (buffers) {
        .input => c.TCIFLUSH,
        .output => c.TCOFLUSH,
        .both => c.TCIOFLUSH,
    };
    if (c.tcflush(file.handle, @as(c_int, @intCast(mode))) != 0)
        return error.FlushError;
}