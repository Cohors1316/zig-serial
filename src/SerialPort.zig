p: private = .{},

pub fn init(port: []const u8, config: Config) SerialPort {
    return .{
        .p = .{
            .name = port,
            .config = config,
        },
    };
}

pub fn iterate() !Iterator {
    return try Iterator.init();
}

pub fn open(self: *SerialPort) !void {
    if (self.p.file) return;
    var buffer: [std.fs.max_path_bytes]u8 = undefined;
    const fmt_string = switch (native_os) {
        .windows => "\\\\.\\{s}",
        .macos => "/dev/{s}",
        else => "/dev/{s}",
    };
    const path = try std.fmt.bufPrint(&buffer, fmt_string, .{self.p.name});
    self.p.file = try std.fs.openFileAbsolute(path, .{.mode = .read_write});
    errdefer {
        if (self.p.file) |file| file.close();
        self.p.file = null;
    }
    try self.configure();
}

pub fn close(self: *SerialPort) void {
    if (self.p.file) |file| {
        file.close();
        self.p.file = null;
    }
}

pub fn write(self: *SerialPort, bytes: []const u8) !void {
    if (self.p.file) |file| {
        return file.writeAll(bytes);
    }
    return error.PortClosed;
}

pub fn read(self: *SerialPort, buffer: []u8) !u32 {
    if (self.p.file) |file| {
        return switch (native_os) {
            .windows => win_serial.read(file, buffer),
            else => serial.read(file, buffer),
        };
    }
    return error.PortClosed;
}

pub fn flush(self: *SerialPort, buffers: Buffers) !void {
    if (self.p.file) |file| {
        return switch (native_os) {
            .windows => win_serial.flush(file, buffers),
            .macos => dar_serial.flush(file, buffers),
            else => lin_serial.flush(file, buffers),
        };
    }
    return error.PortClosed;
}

pub fn setPins(self: *SerialPort, pins: Pins) !void {
    if (self.p.file) |file| {
        return switch (native_os) {
            .windows => win_serial.setPins(file, pins),
            .macos => @panic("MacOs not supported"),
            else => lin_serial.setPins(file, pins),
        };
    }
    return error.PortClosed;
}

const SerialPort = @This();
const std = @import("std");
const win_serial = @import("windows/serial.zig");
const dar_serial = @import("darwin/serial.zig");
const lin_serial = @import("linux/serial.zig");
const File = std.fs.File;
const native_os = @import("builtin").os.tag;
const serial = @import("serial.zig");
const Buffers = serial.Buffers;
const Pins = serial.Pins;
const Parity = serial.Parity;
const Handshake = serial.HandShake;
const Config = serial.Config;
const WordSize = serial.WordSize;
const StopBits = serial.StopBits;

const private = struct {
    file: ?File = null,
    name: []const u8,
    config: Config = .{},
};

const Iterator = switch (native_os) {
    .windows => @import("windows/Iterator.zig"),
    .macos => @import("darwin/Iterator.zig"),
    else => @import("linux/Iterator.zig"),
};

fn configure(self: *SerialPort) !void {
    return switch (native_os) {
        .windows => win_serial.configure(self),
        else => serial.configure(self),
    };
}