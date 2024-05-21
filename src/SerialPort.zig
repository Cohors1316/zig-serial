p: Private = .{},


pub fn init(port_name: []const u8, config: Config) !SerialPort {
    return .{
        .p = .{
            .name = port_name,
            .config = config,
        }
    };
}

pub fn open(self: *SerialPort) !void {
    self.open();
}

pub fn close(self: *SerialPort) void {
    if (self.p.file) |file| {
        file.close();
        self.p.file = null;
    }
}

pub fn list() !void {}

pub fn write(self: *SerialPort, bytes: []const u8) !void {
    if (self.p.file) |file| return file.writeAll(bytes);
    return error.port_closed;
}

pub fn read(self: *SerialPort, buffer: []u8) !usize {
    if (self.p.file) |file| return file.readAll(buffer);
    return error.port_closed;
}






const SerialPort = @This();
const std = @import("std");
const Writer = std.fs.File.Writer;
const Reader = std.fs.File.Reader;
const File = std.fs.File;

const Private = struct {
    file: ?File = null,
    name: []const u8,
    config: Config = .{},
};

const Config = struct {
    baud_rate: u32 = 9600,

};