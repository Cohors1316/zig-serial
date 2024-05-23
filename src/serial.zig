const std = @import("std");
const native_os = @import("builtin").os.tag;
const File = std.fs.File;
const SerialPort = @import("SerialPort.zig");

pub const Pins = struct {
    rts: ?bool = null,
    dtr: ?bool = null,
};

pub const Parity = enum {
    none,
    even,
    odd,
    space,
    mark,
};

pub const StopBits = switch (native_os) {
    .windows => enum {
        one,
        two,
        // #(*$&) windows
        one_point_five,
    },
    else => enum { one, two },
};

pub const HandShake = enum { none, software, hardware };

pub const WordSize = switch (native_os) {
    .windows => enum(u32) {
        CS5 = 5,
        CS6 = 6,
        CS7 = 7,
        CS8 = 8,
    },
    else => std.posix.CSIZE,
};

pub const Buffers = enum {
    input,
    output,
    both,
};

pub const Config = struct {
    baud_rate: u32 = 9600,
    parity: Parity = .none,
    stop_bits: StopBits = .one,
    word_size: WordSize = .eight,
    // these must exist for windows
    // 0 will cause ports to hand indefinitly
    read_timeout: u32 = 100,
    write_timeout: u32 = 100,
};

pub const Description = struct {
    file_name: []const u8,
    display_name: []const u8,
    driver: ?[]const u8,
};

pub const Information = struct {
    port_name: []const u8,
    system_location: []const u8,
    friendly_name: []const u8,
    description: []const u8,
    manufacturer: []const u8,
    serial_number: []const u8,
    // TODO: review whether to remove `hw_id`.
    // Is this useless/being used in a Windows-only way?
    hw_id: []const u8,
    vid: u16,
    pid: u16,
};

pub fn configure(file: std.fs.File, config: Config) !void {
    // I say just let it rip, if an os is unsupported posix will error
    const VTIME = 5;
    const VMIN = 6;
    const VSTART = 8;
    const VSTOP = 9;

    var settings = try std.posix.tcgetattr(file.handle);
    settings.ispeed = config.baud_rate;
    settings.ospeed = config.baud_rate;
    settings.lflag = .{};
    settings.oflag = .{};
    settings.iflag = .{
        .INPCK = (config.parity != .none),
        .IXON = (config.handshake == .software),
        .IXOFF = (config.handshake == .software),
    };
    settings.cflag = .{
        .CREAD = true,
        .PARENB = (config.parity != .none),
        .PARODD = (config.parity == .odd or config.parity == .mark),
        .CLOCAL = (config.handshake == .none),
        .CSTOPB = (config.stop_bits == .two),
        .CSIZE = config.word_size,
    };
    switch (config.parity) {
        .mark => settings.cflag._ |= (1 << 14),
        .space => settings.cflag._ |= 1,
        else => {},
    }
    if (config.handshake == .hardware) settings.cflag._ |= (1 << 15);
    settings.cc[VMIN] = 1;
    settings.cc[VSTOP] = 0x13; // XOFF
    settings.cc[VSTART] = 0x11; // XON
    settings.cc[VTIME] = 0;
    // settings.cflag &= ~@as(os.tcflag_t, CBAUD);
    // settings.cflag |= baudmask;
    try std.posix.tcsetattr(file.handle, .NOW, settings);
}

// test "iterate ports" {
//     var it = try iterator();
//     while (try it.next()) |port| {
//         std.debug.print("{s} (file: {s}, driver: {?s})\n", .{ port.display_name, port.file_name, port.driver });
//     }
// }

// test "basic configuration test" {
//     const cfg = Config{
//         .handshake = .none,
//         .baud_rate = .B115200,
//         .parity = .none,
//         .word_size = .eight,
//         .stop_bits = .one,
//     };

//     var tty: []const u8 = undefined;

//     switch (native_os) {
//         .windows => tty = "\\\\.\\COM3",
//         .linux => tty = "/dev/ttyUSB0",
//         .macos => tty = "/dev/cu.usbmodem101",
//         else => unreachable,
//     }

//     var port = try std.fs.cwd().openFile(tty, .{ .mode = .read_write });
//     defer port.close();

//     try configure(port, cfg);
// }

// test "basic flush test" {
//     var tty: []const u8 = undefined;
//     // if any, these will likely exist on a machine
//     switch (native_os) {
//         .windows => tty = "\\\\.\\COM3",
//         .linux => tty = "/dev/ttyUSB0",
//         .macos => tty = "/dev/cu.usbmodem101",
//         else => unreachable,
//     }
//     var port = try std.fs.cwd().openFile(tty, .{ .mode = .read_write });
//     defer port.close();

//     try flush(port, .both);
//     try flush(port, .input);
//     try flush(port, .output);
// }

// test "change control pins" {
//     _ = changeControlPins;
// }

// test "init" {

//     const tty = switch (native_os) {
//         .windows => "\\\\.\\COM3",
//         .linux => "/dev/ttyUSB0",
//         .macos => "/dev/cu.usbmodem101",
//         else => @compileError("unsupported os"),
//     };

//     var port = try init(tty, .{
//         .baud_rate = .B9600,
//         .handshake = .none,
//         .parity = .none,
//         .word_size = .eight,
//         .stop_bits = .one,
//     });
//     defer port.close();
// }
