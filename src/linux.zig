const std = @import("std");
const serial = @import("zig-serial.zig");
const SerialPortDescription = serial.SerialPortDescription;
const ControlPins = serial.ControlPins;

const TCIFLUSH = 0;
const TCOFLUSH = 1;
const TCIOFLUSH = 2;
const TCFLSH = 0x540B;

pub fn tcFlush(fd: std.os.fd_t, mode: usize) !void {
    if (std.os.linux.syscall3(
        .ioctl,
        @as(usize, @bitCast(@as(isize, fd))),
        TCFLSH,
        mode,
    ) != 0)
        return error.serial_flush_error;
}

pub fn flushSerialPort(port: std.fs.File, input: bool, output: bool) !void {
    if (input and output)
        try tcFlush(port.handle, TCIOFLUSH)
    else if (input)
        try tcFlush(port.handle, TCIFLUSH)
    else if (output)
        try tcFlush(port.handle, TCOFLUSH);
}

pub fn mapBaudToEnum(baudrate: usize) !std.os.linux.speed_t {
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
        // from termios-baud.h
        57600 => .B57600,
        115200 => .B115200,
        230400 => .B230400,
        460800 => .B460800,
        500000 => .B500000,
        576000 => .B576000,
        921600 => .B921600,
        1000000 => .B1000000,
        1152000 => .B1152000,
        1500000 => .B1500000,
        2000000 => .B2000000,
        2500000 => .B2500000,
        3000000 => .B3000000,
        3500000 => .B3500000,
        4000000 => .B4000000,
        else => error.unsupported_baud_rate,
    };
}

pub fn changeControlPins(port: std.fs.File, pins: ControlPins) !void {
    const TIOCM_RTS: c_int = 0x004;
    const TIOCM_DTR: c_int = 0x002;

    // from /usr/include/asm-generic/ioctls.h
    // const TIOCMBIS: u32 = 0x5416;
    // const TIOCMBIC: u32 = 0x5417;
    const TIOCMGET: u32 = 0x5415;
    const TIOCMSET: u32 = 0x5418;

    var flags: c_int = 0;
    if (std.os.linux.ioctl(port.handle, TIOCMGET, @intFromPtr(&flags)) != 0)
        return error.Unexpected;

    if (pins.dtr) |dtr| {
        if (dtr) {
            flags |= TIOCM_DTR;
        } else {
            flags &= ~TIOCM_DTR;
        }
    }
    if (pins.rts) |rts| {
        if (rts) {
            flags |= TIOCM_RTS;
        } else {
            flags &= ~TIOCM_RTS;
        }
    }

    if (std.os.linux.ioctl(port.handle, TIOCMSET, @intFromPtr(&flags)) != 0)
        return error.Unexpected;
}

const PortIterator = struct {
    const Self = @This();

    const root_dir = "/sys/class/tty";

    // ls -hal /sys/class/tty/*/device/driver

    dir: std.fs.Dir,
    iterator: std.fs.Dir.Iterator,

    full_path_buffer: [std.fs.MAX_PATH_BYTES]u8 = undefined,
    driver_path_buffer: [std.fs.MAX_PATH_BYTES]u8 = undefined,

    pub fn init() !Self {
        var dir = try std.fs.cwd().openDir(root_dir, .{ .iterate = true });
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
                // not a dir => we don't care
                var tty_dir = self.dir.openDir(entry.name, .{}) catch continue;
                defer tty_dir.close();

                // we need the device dir
                // no device dir =>  virtual device
                var device_dir = tty_dir.openDir("device", .{}) catch continue;
                defer device_dir.close();

                // We need the symlink for "driver"
                const link = device_dir.readLink("driver", &self.driver_path_buffer) catch continue;

                // full_path_buffer
                // driver_path_buffer

                var fba = std.heap.FixedBufferAllocator.init(&self.full_path_buffer);

                const path = try std.fs.path.join(fba.allocator(), &.{
                    "/dev/",
                    entry.name,
                });

                return SerialPortDescription{
                    .file_name = path,
                    .display_name = path,
                    .driver = std.fs.path.basename(link),
                };
            } else {
                return null;
            }
        }
        return null;
    }
};
