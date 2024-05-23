key: HKEY,
index: u32,
name: [256:0]u8 = undefined,

name_size: u32 = 256,

data: [256]u8 = undefined,
filepath_data: [256]u8 = undefined,
data_size: u32 = 256,

pub fn init() !Iterator {
    const HKEY_LOCAL_MACHINE = @as(
        HKEY,
        @ptrFromInt(0x80000002),
    );
    const KEY_READ = 0x20019;

    var self: Iterator = undefined;
    self.index = 0;
    if (externs.RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM\\", 0, KEY_READ, &self.key) != 0)
        return error.iterator_error;

    return self;
}

pub fn deinit(self: *Iterator) void {
    _ = externs.RegCloseKey(self.key);
    self.* = undefined;
}

pub fn next(self: *Iterator) !?Description {
    defer self.index += 1;

    self.name_size = 256;
    self.data_size = 256;

    return switch (externs.RegEnumValueA(self.key, self.index, &self.name, &self.name_size, null, null, &self.data, &self.data_size)) {
        0 => Description{
            .file_name = try std.fmt.bufPrint(&self.filepath_data, "\\\\.\\{s}", .{self.data[0 .. self.data_size - 1]}),
            .display_name = self.data[0 .. self.data_size - 1],
            .driver = self.name[0..self.name_size],
        },
        259 => null,
        else => error.iterator_error,
    };
}

const Iterator = @This();
const std = @import("std");
const Description = @import("../serial.zig").Description;
const externs = @import("externs.zig");
const HKEY = externs.HKEY;