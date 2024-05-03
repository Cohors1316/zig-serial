const std = @import("std");
const SerialPort = @import("SerialPort.zig");
const Description = SerialPort.Description;
const Pins = SerialPort.Pins;

// const TCIFLUSH = 0;
// const TCOFLUSH = 1;
// const TCIOFLUSH = 2;
// const TCFLSH = 0x540B;

pub fn tcFlush(fd: std.os.fd_t, mode: usize) !void {
    if (std.os.linux.syscall3(
        .ioctl,
        @as(usize, @bitCast(@as(isize, fd))),
        0x540B, // TCFLSH
        mode,
    ) != 0)
        return error.serial_flush_error;
}

pub fn setPins(port: std.fs.File, pins: Pins) !void {
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

    pub fn next(self: *Self) !?Description {
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

                return Description{
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
