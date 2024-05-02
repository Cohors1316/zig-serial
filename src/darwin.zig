const std = @import("std");
const serial = @import("zig-serial.zig");
const SerialPortDescription = serial.SerialPortDescription;
const c = @cImport(@cInclude("termios.h"));

fn tcFlush(fd: std.os.fd_t, mode: usize) !void {
    const err = c.tcflush(fd, @as(c_int, @intCast(mode)));
    if (err != 0) {
        std.debug.print("tcflush failed: {d}\r\n", .{err});
        return error.FlushError;
    }
}

pub fn flushSerialPort(port: std.fs.File, input: bool, output: bool) !void {
    if (input and output)
        try tcFlush(port.handle, c.TCIOFLUSH)
    else if (input)
        try tcFlush(port.handle, c.TCIFLUSH)
    else if (output)
        try tcFlush(port.handle, c.TCOFLUSH);
}

pub fn mapBaudToEnum(baudrate: usize) !std.os.darwin.speed_t {
    return switch (baudrate) {
        // from termios.h
        50 => .B50,
        75 => .B75,
        110 => .B110,
        134 => .B134,
        150 => .B150,
        200 => .B200,
        300 => .B300,
        600 => .B600,
        1200 => .B1200,
        1800 => .B1800,
        2400 => .B2400,
        4800 => .B4800,
        9600 => .B9600,
        19200 => .B19200,
        38400 => .B38400,
        7200 => .B7200,
        14400 => .B14400,
        28800 => .B28800,
        57600 => .B57600,
        76800 => .B76800,
        115200 => .B115200,
        230400 => .B230400,
        else => error.UnsupportedBaudRate,
    };
}

const PortIterator = struct {
    const Self = @This();

    const root_dir = "/dev/";

    dir: std.fs.IterableDir,
    iterator: std.fs.IterableDir.Iterator,

    full_path_buffer: [std.fs.MAX_PATH_BYTES]u8 = undefined,
    driver_path_buffer: [std.fs.MAX_PATH_BYTES]u8 = undefined,

    pub fn init() !Self {
        var dir = try std.fs.cwd().openIterableDir(root_dir, .{});
        errdefer dir.close();

        return Self{
            .dir = dir,
            .iterator = dir.iterate(),
        };
    }

    pub fn deinit(self: *Self) void {
        self.dir.close();
        self.* = undefined;
    }

    pub fn next(self: *Self) !?SerialPortDescription {
        while (true) {
            if (try self.iterator.next()) |entry| {
                if (!std.mem.startsWith(u8, entry.name, "cu.")) {
                    continue;
                } else {
                    var fba = std.heap.FixedBufferAllocator.init(&self.full_path_buffer);

                    const path = try std.fs.path.join(fba.allocator(), &.{
                        "/dev/",
                        entry.name,
                    });

                    return SerialPortDescription{
                        .file_name = path,
                        .display_name = path,
                        .driver = "darwin",
                    };
                }
            } else {
                return null;
            }
        }
        return null;
    }
};
