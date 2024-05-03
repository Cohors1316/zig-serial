const std = @import("std");
const builtin = @import("builtin");
const SerialPort = @import("../src/SerialPort.zig");

pub fn main() !void {
    const name = switch (builtin.os.tag) {
        .windows => "\\\\.\\COM3",
        .linux => "/dev/ttyUSB0",
        .macos => "/dev/cu.usbserial-0001", // don't have a mac
    };
    var sp = try SerialPort.init(
        name,
        .{
            .baud_rate = .B9600,
            .parity = .none,
            .stop_bits = .one,
            .word_size = .eight,
            .handshake = .none,
        },
    );
    defer sp.deinit();
    try sp.flushBuffers(.both);
    try sp.setPins(.{ .dtr = true, .rts = true });
    try sp.write("Hello, World!\n");
    try sp.writeLine("Hello, World!");
    const buffer: [10]u8 = undefined;
    try sp.read(buffer);
    std.debug.print("Response: {s}\n", .{buffer});
}