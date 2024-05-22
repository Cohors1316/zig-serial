p: Private = .{},

pub fn init(port: []const u8, config: Config) !SerialPort {
    return .{
        .p = .{
            .name = port,
            .config = config,
        },
    };
}

pub fn open(self: *SerialPort) !void {
    if (self.p.file) return;
    var buffer: [std.fs.max_path_bytes]u8 = undefined;
    const path = try std.fmt.bufPrint(&buffer, "\\\\.\\", .{self.p.name});
    self.p.file = try std.fs.openDirAbsolute(path, .{.mode = .read_write});
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
    return error.port_closed;
}

pub fn read(self: *SerialPort, buffer: []u8) !u32 {
    if (self.p.file) |file| {
        var bytes_read: u32 = undefined;
        const result = kernal32.ReadFile(
            file.handle,
            buffer,
            @min(buffer.len, self.status().cbInQue),
            &bytes_read,
            null,
        );
        if (result == 0) return error.could_not_read_buffer;
        return bytes_read;
    }
    return error.port_closed;
}

pub fn flush(self: *SerialPort, buffers: Buffers) !void {
    if (self.p.file) |file| {
        const success = externs.PurgeComm(file.handle, @intFromEnum(buffers)) == 0;
        return if (success) null else kernal32.GetLastError();
    }
    return error.port_closed;
}

pub fn controlPins(self: *SerialPort, pins: Pins) !void {
    if (self.p.file) |file| {
        if (pins.rts) |rts| {
            const success = externs.EscapeCommFunction(file.handle, if (rts) 3 else 4) == 0;
            return if (success) null else kernal32.GetLastError();
        }
        if (pins.dtr) |dtr| {
            const success = externs.EscapeCommFunction(file.handle, if (dtr) 5 else 6) == 0;
            return if (success) null else kernal32.GetLastError();
        }
    }
    return error.port_closed;
}

const SerialPort = @This();
const std = @import("std");
const externs = @import("externs.zig");
const DCB = @import("DCB.zig");
const COMSTAT = @import("COMSTAT.zig");
const COMMTIMEOUTS = @import("COMMTIMEOUTS.zig");
const File = std.fs.File;
const OVERLAPPED = std.os.windows.OVERLAPPED;
const kernal32 = std.os.windows.kernel32;
const WINAPI = std.os.windows.WINAPI;
const native_os = @import("builtin").os.tag;

const Private = struct {
    name: []const u8,
    file: ?File = null,
    config: Config = .{},
};

const Config = struct {
    baud_rate: u32 = 9600,
    parity: Parity = .none,
    stop_bits: StopBits = .one,
    word_size: WordSize = .eight,
    handshake: Handshake = .none,
    read_timeout: u32 = 100,
    write_timeout: u32 = 100,
};

const WordSize = enum(u32) {
    five = 5,
    six = 6,
    seven = 7,
    eight = 8,
};

const Parity = enum {
    none,
    odd,
    even,
    mark,
    space,
};

const StopBits = enum {
    one,
    two,
    one_point_five,
};

const Handshake = enum(u32) {
    none,
    software,
    hardware,
};

const Buffers = enum(u32) {
    input = 0x0008,
    output = 0x0004,
    both = 0x0004 | 0x0008,
};

const Pins = struct {
    rts: ?bool = null,
    dtr: ?bool = null,
};

fn status(self: *SerialPort) COMSTAT {
    if (self.p.handle) |handle| {
        var stats = COMSTAT{};
        if (externs.ClearCommError(handle, null, &stats) == 0)
            return error.could_not_get_status;
        return stats;
    }
    return error.port_closed;
}

fn configure(self: *SerialPort, file: File) !void {
    const parity = switch (self.p.config.parity) {
        .none => "n",
        .even => "e",
        .odd => "o",
        .mark => "m",
        .space => "s",
    };
    const data_bits = @intFromEnum(self.p.config.word_size);
    const stop_bits = switch (self.p.config.stop_bits) {
        .one => "1",
        .two => "2",
        .one_point_five => "1.5",
    };
    // Specifies whether infinite time-out processing is on or off. The default is off.
    const to = "off";
    // Specifies whether the xon or xoff protocol for data-flow control is on or off.
    const xon = "off";
    // Specifies whether output handshaking that uses the Data Set Ready (DSR) circuit is on or off.
    const odsr = "off";
    // Specifies whether output handshaking that uses the Clear To Send (CTS) circuit is on or off.
    const octs = "off";
    // Specifies whether the Data Terminal Ready (DTR) circuit is on or off or set to handshake.
    const dtr = "off";
    // Specifies whether the Request To Send (RTS) circuit is set to on, off, handshake, or toggle.
    const rts = "off";
    // Specifies whether the DSR circuit sensitivity is on or off.
    const idsr = "off";
    var buffer: [110]u8 = undefined;
    const string = try std.fmt.bufPrint(
        &buffer,
        "COM{d}: parity={s} data={d} stop={s} to={s} xon={s} odsr={s} octs={s} dtr={s} rts={s} idsr={s}",
        .{ self.p.number, parity, data_bits, stop_bits, to, xon, odsr, octs, dtr, rts, idsr },
    );

    var dcb = DCB{};
    var timeouts = COMMTIMEOUTS{};
    if (externs.BuildCommDCBAndTimeoutsA(
        &string,
        &dcb,
        &timeouts,
    ) == 0) return error.failed_building_dcb_and_timeouts;
    dcb.BaudRate = self.p.config.baud_rate;
    timeouts.ReadTotalTimeoutConstant = self.p.config.read_timeout;
    timeouts.WriteTotalTimeoutConstant = self.p.config.write_timeout;

    if (externs.SetCommState(file.handle, dcb) == 0)
        return error.failed_setting_comm_state;
    if (externs.SetCommTimeouts(file.handle, &timeouts) == 0)
        return error.failed_setting_timeouts;
}
