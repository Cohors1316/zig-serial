p: Private = .{},

pub fn init(port_name: []const u8, config: Config) !SerialPort {

    const prefix = switch (native_os) {
        .windows => "\\\\.\\{s}",
        else => "/dev/{s}",
    };
    const buffer: [std.fs.max_path_bytes]u8 = undefined;
    const path = try std.fmt.bufPrint(&buffer, prefix, .{port_name});
    return .{
        .p = .{
            .path = path,
            .config = config,
        }
    };
}

pub fn open(self: *SerialPort) !void {
    if (self.p.file) return;
    self.p.file = try std.fs.openFileAbsolute(self.p.path, .{.mode = .read_write});
    errdefer self.close();
    var dcb = try DCB.init(self.p.path, self.p.config);




}

pub fn close(self: *SerialPort) void {
    if (self.p.file) |file| {
        file.close();
        self.p.file = null;
    }
}

pub fn write(self: *SerialPort, bytes: []const u8) !void {
    if (self.p.file) |file| return file.writeAll(bytes);
    return error.port_closed;
}

pub fn flush(self: *SerialPort, buffers: Buffers) !?Win32Error {
    if (self.p.file) |file| {
        const success = PurgeComm(file.handle, @intFromEnum(buffers)) == 0;
        return if (success) null else kernal32.GetLastError();
    }
    return error.port_closed;
}

pub fn controlPins(self: *SerialPort, pins: Pins) !?Win32Error {
    if (self.p.file) |file| {
        if (pins.rts) |rts| {
            const success = EscapeCommFunction(file.handle, if (rts) 3 else 4) == 0;
            return if (success) null else kernal32.GetLastError();
        }
        if (pins.dtr) |dtr| {
            const success = EscapeCommFunction(file.handle, if (dtr) 5 else 6) == 0;
            return if (success) null else kernal32.GetLastError();
        }
    }
    return error.port_closed;
}



const SerialPort = @This();
const std = @import("std");
const File = std.fs.File;
const HANDLE = std.os.windows.HANDLE;
const DWORD = std.os.windows.DWORD;
const BOOL = std.os.windows.BOOL;
const HWND = std.os.windows.HWND;
const OVERLAPPED = std.os.windows.OVERLAPPED;
const BYTE = std.os.windows.BYTE;
const WORD = std.os.windows.WORD;
const ULONG = std.os.windows.ULONG;
const CSTR = []const u8;
const Win32Error = std.os.windows.Win32Error;
const kernal32 = std.os.windows.kernel32;
const WINAPI = std.os.windows.WINAPI;
const native_os = @import("builtin").os.tag;

const Private = struct {
    file: ?File = null,
    path: []const u8,
    config: Config = .{},
};

const Config = struct {
    baud_rate: u32 = 9600,
    parity: Parity = .none,
    stop_bits: StopBits = .one,
    word_size: WordSize = .eight,
    handshake: Handshake = .none,
};

const WordSize = enum(u32) {
    five = 5,
    six = 6,
    seven = 7,
    eight = 8,
};

const Parity = enum(u32) {
    none = 0,
    odd = 1,
    even = 2,
    mark = 3,
    space = 4,
};

const StopBits = enum(u32) {
    one = 0,
    two = 2,
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

const DCB = struct {
    DCBlength: DWORD,
    BaudRate: DWORD,
    f: u32,
    wReserved: WORD,
    XonLin: WORD,
    XoffLim: WORD,
    ByteSize: BYTE,
    Parity: BYTE,
    StopBits: BYTE,
    XonChar: c_char,
    XoffChar: c_char,
    ErrorChar: c_char,
    EofChar: c_char,
    EvtChar: c_char,
    wReserved1: WORD,

    pub fn init(port_name: []const u8, config: Config) !DCB {
        var buffer:[110]u8 = undefined;
        const string = try std.fmt.bufPrint(
            &buffer,
            "{s}: baud={d} parity=N data={d} stop={d} to={s} xon={s} odsr={s} octs={s} dtr={s} rts={s} idsr={s}",
            .{
                port_name,
                @intFromEnum(config.baud_rate),
                switch (config.parity) {.even => "E", .mark => "M", .none => "N", .odd => "O", .space => "S"},
                @intFromEnum(config.word_size),
                @intFromEnum(config.stop_bits),
                "off", // to
                "off", // xon
                "off", // osdr
                "off", // octs
                "off", // dtr
                "off", // rts
                "off", // isdr
            }
        );
        var dcb = DCB{};
        const success = BuildCommDCBA(&string, &dcb) != 0;
        return if (success) dcb else error.windows_dcb;
    }
};

/// Fills a specified DCB structure with values specified in a device-control string.
extern "kernel32" fn BuildCommDCBA(in_lpDef: *CSTR, out_lpDCB: *DCB) callconv(WINAPI) BOOL;

/// Translates a device-definition string into appropriate device-control block codes and places them into a device control block.
extern "kernel32" fn BuildCommDCBAndTimeoutsA(in_lpDef: *CSTR, out_lpDCB: *DCB, out_lpCommTimeouts: *COMMTIMEOUTS) callconv(WINAPI) BOOL;

/// Restores character transmission for a specified communications device and places the transmission line in a nonbreak state.
extern "kernel32" fn ClearCommBreak(in_hFile: HANDLE) callconv(WINAPI) BOOL;

/// Retrieves information about a communications error and reports the current status of a communications device.
extern "kernel32" fn ClearCommError(in_hfile: HANDLE, out_lpErrors: ?*DWORD, out_lpStat: ?*COMSTAT) BOOL;

/// Displays a driver-supplied configuration dialog box.
extern "kernel32" fn CommConfigDialogA(in_lpszName: *CSTR, in_hWnd: HWND, in_out_lpCC: *COMMCONFIG) callconv(WINAPI) BOOL;

/// Directs a specified communications device to perform an extended function.
extern "kernel32" fn EscapeCommFunction(in_hFile: HANDLE, in_dwFunc: DWORD) callconv(WINAPI) BOOL;

/// Retrieves the current configuration of a communications device.
extern "kernel32" fn GetCommConfig(in_hCommDev: HANDLE, out_lpCC: *COMMCONFIG, in_out_lpdwSize: *DWORD) callconv(WINAPI) BOOL;

/// Retrieves the value of the event mask for a specified communications device.
extern "kernel32" fn GetCommMask(in_hFile: HANDLE, out_lpEvtMask: *DWORD) callconv(WINAPI) BOOL;

/// Retrieves modem control-register values.
extern "kernel32" fn GetCommModemStatus(in_hFile: HANDLE, out_lpModemStat: *DWORD) callconv(WINAPI) BOOL;

/// Gets an array that contains the well-formed COM ports.
extern "kernel32" fn GetCommPorts(
    /// An array for the port numbers.
    out_lpPortNumbers: *ULONG,
    /// The length of the array in the lpPortNumbersparameter.
    in_uPortNumbersCount: ULONG,
    ///The numeber of port numbers written to the lpPortNumbers of the length of the array required for the port numbers.
    out_puPortNumbersFound: *ULONG) ULONG;

/// Retrieves information about the communications properties for a specified communications device.
extern "kernel32" fn GetCommProperties(in_hFile: HANDLE, out_lpCommProp: *COMMPROP) callconv(WINAPI) BOOL;

/// Retrieves the current control settings for a specified communications device.
extern "kernel32" fn GetCommState(in_hFile: HANDLE, in_out_lpDCB: *DCB) callconv(WINAPI) std.os.windows.BOOL;

/// Retrieves the time-out parameters for all read and write operations on a specified communications device.
extern "kernel32" fn GetCommTimeouts(in_hFile: HANDLE, out_lpCommTimeouts: *COMMTIMEOUTS) callconv(WINAPI) BOOL;

/// Retrieves the default configuration for the specified communications device.
extern "kernel32" fn GetDefaultCommConfigA(in_lpszName: *CSTR, out_lpCC: *COMMCONFIG, in_out_lpdwSize: *DWORD) callconv(WINAPI) BOOL;

/// Discards all characters from the output or input buffer of a specified communications resource.
extern "kernel32" fn PurgeComm(in_hFile: HANDLE, in_dwFlags: DWORD) callconv(WINAPI) BOOL;

/// Suspends character transmission for a specified communications device and places the transmission line in a break state.
extern "kernel32" fn SetCommBreak(in_hFile: HANDLE) callconv(WINAPI) BOOL;

/// Sets the current configuration of a communications device.
extern "kernel32" fn SetCommConfig(in_hCommDev: HANDLE, in_lpCC: *COMMCONFIG, in_dwSize: DWORD) callconv(WINAPI) BOOL;

/// Specifies a set of events to be monitored for a communications device.
extern "kernel32" fn SetCommMask(in_hFile: HANDLE, in_dwEvtMask: DWORD) callconv(WINAPI) BOOL;

/// Configures a communications device according to the specifications in a device-control block.
extern "kernel32" fn SetCommState(in_hFile: HANDLE, in_lpDCB: *DCB) callconv(WINAPI) BOOL;

/// Sets the time-out parameters for all read and write operations on a specified communications device.
extern "kernel32" fn SetCommTimeouts(in_hFile: HANDLE, in_lpCommTimeouts: *COMMTIMEOUTS) callconv(WINAPI) BOOL;

/// Sets the default configuration for a communications device.
extern "kernel32" fn SetDefaultCommConfigA(in_lpszName: *CSTR, in_lpCC: *COMMCONFIG, in_dwSize: DWORD) callconv(WINAPI) BOOL;

/// Initializes the communications parameters for a specified communications device.
extern "kernel32" fn SetupComm(in_hFile: HANDLE, in_dwInQueue: DWORD, in_dwOutQueue: DWORD) callconv(WINAPI) BOOL;

/// Transmits a specified character ahead of any pending data in the output buffer of the specified communications device.
extern "kernel32" fn TransmitCommChar(in_hFile: HANDLE, in_cChar: c_char) callconv(WINAPI) BOOL;

/// Waits for an event to occur for a specified communications device.
extern "kernel32" fn WaitCommEvent(in_hFile: HANDLE, out_lpEvntMask: *DWORD, in_lpOverlapped: *OVERLAPPED) callconv(WINAPI) BOOL;
